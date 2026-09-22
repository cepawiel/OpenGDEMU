//! ATA/ATAPI command-handling task on the MCU side.
//!
//! The FPGA latches incoming IDE register writes and sets `COMMAND_PEND`
//! whenever the host writes the Command register. This task polls that
//! flag, reads the command word + features + LBA + sector count, decodes
//! the opcode, and writes back the resulting status / error and clears
//! `COMMAND_PEND` so the FPGA accepts the next command.
//!
//! Phase scope: trivial commands only — `DEVICE_RESET`, `SET_FEATURES`,
//! `IDENTIFY DEVICE` (ABRTed because we are ATAPI). `PACKET` (`0xA0`) is
//! still a stub: we ack with ABRT for now until the GDI parser + FIFO
//! data path land. Every command — known or not — is logged via defmt so
//! we can see what the DC's GD-ROM driver issues during bring-up.

use core::ptr::{read_volatile, write_volatile};

use defmt::{info, warn};
use embassy_atsam3::smc;
use embassy_futures::yield_now;
use embassy_time::{Duration, Instant, Timer};

// ----- Per-device 7-bit register window (matches FPGA hw/ide_device.py). ---

const REG_HW_INFO: usize = 0;
const REG_STATUS: usize = 1;
#[allow(dead_code)]
const REG_DEVICE: usize = 2;
const REG_ERROR: usize = 3;
const REG_CMD_FEAT: usize = 4;
const REG_SECTOR_COUNT: usize = 5;
const REG_LBA_LOW_MID: usize = 6;
const REG_LBA_HIGH_LOWP: usize = 7;
const REG_LBA_MIDP_HIGHP: usize = 8;
const REG_DATA_FIFO: usize = 9;
const REG_READ_FIFO_LEVEL: usize = 10;
const REG_WRITE_FIFO_LEVEL: usize = 11;
/// IDE INTRQ-pending flop. Write 1 to assert; cleared in hardware when
/// the host reads the IDE STATUS register.
const REG_INTRQ_PENDING: usize = 13;
/// DMA mode arm/disarm. Write 1 before staging the data-in payload of a
/// DMA-mode PACKET so DMARQ goes high once the read FIFO has data; write
/// 0 once the host has finished draining. Without this the FPGA's DMARQ
/// stays low and the host's DMA controller waits forever.
const REG_DMA_MODE: usize = 19;
/// In-fabric signal capture (opt-in in the FPGA). Slot 20 arms and reports,
/// slot 21 is write-index / read-value.
const REG_CAP_CTRL: usize = 20;
const REG_CAP_DATA: usize = 21;
const CAP_DEPTH: usize = 512;
/// Host access counters since the last Command write: Status/Alt Status
/// reads (slot 24, lo/hi bytes), Data reads (slot 25), last Device Control
/// byte / write count (slot 26). Read-only; the FPGA clears 24/25 itself.
const REG_HOST_STATUS_RDS: usize = 24;
const REG_HOST_DATA_RDS: usize = 25;
const REG_HOST_DEVCTL: usize = 26;
/// Slots 24/25 frozen at the most recent Command write: what the host did
/// during the *previous* command, readable after it has moved on.
const REG_HOST_PREV_STATUS_RDS: usize = 27;
const REG_HOST_PREV_DATA_RDS: usize = 28;

// HW_INFO bit layout.
const HW_CMD_PEND: u16 = 0x0002;
/// Sticky: the host reset the device (ATA hard reset or SRST) since the
/// MCU last cleared it. Write 1 to clear.
const HW_RESET_SEEN: u16 = 0x0004;
const HW_NIEN: u16 = 0x0008;
const HW_IN_RESET: u16 = 0x0010;

// Status register bits (matches ATA: BSY DRDY DF DSC DRQ - - ERR).
const STATUS_BSY: u8 = 0x80;
const STATUS_DRDY: u8 = 0x40;
#[allow(dead_code)]
const STATUS_DF: u8 = 0x20;
const STATUS_DSC: u8 = 0x10;
const STATUS_DRQ: u8 = 0x08;
const STATUS_ERR: u8 = 0x01;

// Error register bits (we only model ABRT for now).
const ERR_ABRT: u8 = 0x04;

// ATA / ATAPI command opcodes we handle (or explicitly ABRT).
const CMD_NOP: u8 = 0x00;
const CMD_DEVICE_RESET: u8 = 0x08;
const CMD_PACKET: u8 = 0xA0;
const CMD_IDENTIFY_PACKET: u8 = 0xA1;
const CMD_IDENTIFY: u8 = 0xEC;
const CMD_SET_FEATURES: u8 = 0xEF;

const STATUS_IDLE: u8 = STATUS_DRDY | STATUS_DSC;
const STATUS_IDLE_ERR: u8 = STATUS_DRDY | STATUS_DSC | STATUS_ERR;

/// Wrapper around one of the two FPGA-emulated IDE devices.
#[derive(Clone, Copy)]
pub struct IdeDevice {
    pub(crate) base: *mut u16,
    /// Words this device's read FIFO can hold. The two devices are built with
    /// different depths (`READ_FIFO_DEPTH` / `SECONDARY_READ_FIFO_DEPTH` in
    /// hw/ide_device.py) and this used to be one global constant set to the
    /// primary's 4096. On the secondary -- 256 words, one sector exactly --
    /// that told `push_buf_to_fifo` there were thousands of words of room, so
    /// it pushed a whole 512-byte block into a FIFO with less space than that
    /// and the overflow was dropped on the floor. The host then ran the FIFO
    /// dry before the end of the sector, read a word that was never serviced,
    /// and every word after it in the transfer was shifted by one.
    read_fifo_depth: u16,
}

unsafe impl Send for IdeDevice {}

impl IdeDevice {
    pub fn primary() -> Self {
        Self { base: smc::fpga_window(), read_fifo_depth: PRIMARY_FIFO_DEPTH_WORDS }
    }

    pub fn secondary() -> Self {
        // Secondary device window is at MCU_ADDR[7]=1 → +0x80 words.
        Self {
            base: unsafe { smc::fpga_window().add(0x80) },
            read_fifo_depth: SECONDARY_FIFO_DEPTH_WORDS,
        }
    }

    /// Words this device's read FIFO holds. Must match the depth the FPGA was
    /// built with, or a push overruns it silently.
    #[inline]
    fn fifo_depth_words(&self) -> u16 {
        self.read_fifo_depth
    }

    #[inline]
    fn read(&self, reg: usize) -> u16 {
        unsafe { read_volatile(self.base.add(reg)) }
    }

    #[inline]
    fn write(&self, reg: usize, val: u16) {
        unsafe { write_volatile(self.base.add(reg), val) }
    }

    fn command_pending(&self) -> bool {
        (self.read(REG_HW_INFO) & HW_CMD_PEND) != 0
    }

    /// The host reset the device since `clear_reset_seen`.
    fn reset_seen(&self) -> bool {
        (self.read(REG_HW_INFO) & HW_RESET_SEEN) != 0
    }

    /// Clear the sticky reset flag without disturbing COMMAND_PEND (slot 0
    /// writes carry both).
    fn clear_reset_seen(&self) {
        let pend = self.read(REG_HW_INFO) & HW_CMD_PEND;
        self.write(REG_HW_INFO, pend | HW_RESET_SEEN);
    }

    fn is_primary(&self) -> bool {
        self.base == smc::fpga_window()
    }

    /// The other device on the bus.
    fn peer(&self) -> IdeDevice {
        if self.is_primary() { Self::secondary() } else { Self::primary() }
    }

    /// Read the latched command opcode + features + LBA + sector count.
    fn snapshot_command(&self) -> CommandSnapshot {
        let cmd_feat = self.read(REG_CMD_FEAT);
        let low_mid = self.read(REG_LBA_LOW_MID);
        let high_lowp = self.read(REG_LBA_HIGH_LOWP);
        let midp_highp = self.read(REG_LBA_MIDP_HIGHP);
        let count = self.read(REG_SECTOR_COUNT);
        let device = self.read(REG_DEVICE) as u8;
        CommandSnapshot {
            device,
            command: (cmd_feat & 0xFF) as u8,
            features: (cmd_feat >> 8) as u8,
            lba_low: (low_mid & 0xFF) as u8,
            lba_mid: (low_mid >> 8) as u8,
            lba_high: (high_lowp & 0xFF) as u8,
            // The LBA48 shadow half of the sliding window: the host's first
            // write to each register, pushed aside by its second.
            lba_low_prev: (high_lowp >> 8) as u8,
            lba_mid_prev: (midp_highp & 0xFF) as u8,
            lba_high_prev: (midp_highp >> 8) as u8,
            sector_count: count,
        }
    }

    fn set_error(&self, err: u8) {
        self.write(REG_ERROR, err as u16);
    }

    fn set_status(&self, status: u8) {
        self.write(REG_STATUS, status as u16);
    }

    /// Post a new status and assert INTRQ in the same FPGA clock (slot 1
    /// bit 8). Every host-visible transition that interrupts -- DRQ up for
    /// a data-in block, command completion -- goes through here, never
    /// through `set_status` followed by `assert_intrq`: the sub-microsecond
    /// gap between those two writes was enough for a host polling Alt
    /// Status to see BSY drop, read Status and write its next command, so
    /// the interrupt then landed on that command as a spurious one. Quake
    /// III's WinCE driver, hit that way, slept through the real DRQ
    /// interrupt of its next small PIO read and dropped to the BIOS menu.
    fn set_status_intrq(&self, status: u8) {
        self.write(REG_STATUS, status as u16 | 0x0100);
    }

    /// Post a command's *completion* (error + status + INTRQ) -- unless the
    /// host has already written its next command, in which case the
    /// completion is dropped and `false` returned. Once a new command is
    /// latched the old one's status is dead (ATA-5 §9.1), and writing it
    /// anyway lands on the new command: for PACKET the FPGA has just opened
    /// the CDB phase (DRQ=1) on the Command write, so a stale 0x50 here
    /// clears DRQ before the host sees it and the CDB never arrives.
    /// g1bios_replay's disc probe does exactly that -- a 1-sector CD_READ it
    /// never drains, then straight into the next PACKET; its Command write
    /// clears our FIFO, which satisfied the drain wait.
    fn complete(&self, error: u8, status: u8) -> bool {
        // COMMAND_PEND can latch without the host having written a command:
        // measured at the end of the DMA read that delivers a MIL-CD's
        // 1ST_READ.BIN, where the flag came up with the Command register
        // reading 0x00 and every other register still holding the finished
        // read's values (byte count 0x3800, SECNR 0x22). Dropping the
        // completion there drops the one the host is still waiting on: its
        // read syscall never returns, the bootstrap never jumps, and the
        // console sits on the disc's licence screen forever.
        //
        // 0x00 is not a command the host issues -- ATA-5 has it as NOP, which
        // nothing in this boot path uses -- so treat it as the spurious latch
        // it is and post the completion anyway. A real command is a non-zero
        // opcode, and for that the guard below still applies.
        if self.command_pending() && (self.read(REG_CMD_FEAT) & 0xFF) == 0 {
            info!(
                "{}: COMMAND_PEND with no command (cmd=0x00); completing anyway",
                if self.is_primary() { "primary" } else { "secondary" }
            );
            self.set_error(error);
            self.set_status_intrq(status);
            return true;
        }
        if self.command_pending() {
            // Log what the "next command" actually is. A real one is a
            // plausible ATA opcode; a command register reading 0x00 means
            // COMMAND_PEND latched without the host writing a command, and
            // the completion we are about to drop was the one the host was
            // still waiting for.
            let cmd_feat = self.read(REG_CMD_FEAT);
            info!("{}: host already issued the next command; completion dropped (hw={:#06x} cmd={:#04x} feat={:#04x} st={:#04x} rfifo={})",
                  if self.is_primary() { "primary" } else { "secondary" },
                  self.read(REG_HW_INFO), cmd_feat & 0xFF, cmd_feat >> 8,
                  self.read(REG_STATUS) & 0xFF, self.read(REG_READ_FIFO_LEVEL));
            return false;
        }
        self.set_error(error);
        self.set_status_intrq(status);
        true
    }

    /// Clear COMMAND_PEND so the FPGA stops asserting MCU_IRQ and
    /// accepts the next command.
    fn ack_command(&self) {
        self.write(REG_HW_INFO, 0);
    }

    /// Pulse the IDE-side INTRQ line. Real ATAPI drives INTRQ high
    /// when the device wants the host's attention (data ready, command
    /// done); the host clears it by reading the STATUS register —
    /// which the FPGA observes and clears the latched flop. Without
    /// this the BIOS waits on the IRQ pin for a transition that never
    /// happens and times out.
    fn assert_intrq(&self) {
        self.write(REG_INTRQ_PENDING, 0x0001);
    }

    /// Arm or disarm DMA mode in the FPGA. While armed (1), the FPGA
    /// asserts DMARQ whenever the read FIFO has at least one word and
    /// this device is selected — driving the host's DMA controller
    /// through the data-in phase. Disarm (0) once the host has drained
    /// the transfer.
    /// Arm the FPGA's DMA path in a direction, or disarm it.
    ///
    /// The direction matters because DMARQ means opposite things each way:
    /// reading it says "I have data", writing it says "I have room".
    ///
    /// Reading, the FPGA only raises DMARQ with a whole 16-word Holly burst
    /// in the FIFO (a grant with fewer let the host outrun the refill and
    /// read stale words). So data staged for a DMA read must total a multiple
    /// of 32 bytes or its tail never drains -- true of every whole-sector
    /// path today; pad here if a caller ever stages anything else.
    fn set_dma_mode(&self, mode: DmaMode) {
        self.write(REG_DMA_MODE, mode as u16);
    }

    /// Arm hardware backpressure for a PIO data-out phase: while set, the
    /// FPGA drops the host-visible DRQ whenever the write FIFO is full.
    /// Without it a multi-sector WRITE SECTORS overruns the FIFO -- the MCU
    /// polls far too slowly to drop DRQ in time by itself.
    /// Arm the fabric capture: it triggers on the next matching event and
    /// records until full. No-op unless the FPGA was built with capture=True.
    fn capture_arm(&self) {
        self.write(REG_CAP_CTRL, 1);
    }

    /// Dump whatever was recorded, complete or not -- a partial capture is
    /// exactly what a stalled transfer produces.
    fn capture_dump(&self, label: &'static str) {
        let ctrl = self.read(REG_CAP_CTRL);
        // Bit 2 means the buffer filled. The event field is wptr >> 1 in eight
        // bits, so a full 512-entry capture reports 256 there and wraps to 0 --
        // trust the done bit over the count.
        let count = if ctrl & 0x04 != 0 {
            CAP_DEPTH
        } else {
            ((ctrl >> 8) as usize) * 2
        };
        info!("{}: capture ctrl={:#06x} events={}", label, ctrl, count);
        // Always read a few entries even when the count field says zero: it is
        // wptr >> 1, so a single captured event reports 0 and would otherwise
        // be invisible. cap_run still being set means it triggered.
        let count = count.max(8);
        for i in 0..count.min(CAP_DEPTH) {
            self.write(REG_CAP_DATA, i as u16);
            let v = self.read(REG_CAP_DATA);
            if i < 16 || i + 2 >= count {
                info!(
                    "  i={} byte={:#04x} addr={} csn={} iscmd={} bsy={} drq={}",
                    i, v & 0xFF, (v >> 8) & 7, (v >> 11) & 3,
                    (v >> 13) & 1, (v >> 14) & 1, (v >> 15) & 1,
                                );
            }
        }
    }

    fn set_pio_flow_control(&self, read: bool, write: bool) {
        self.write(REG_DMA_MODE, (if read { 8 } else { 0 }) | (if write { 4 } else { 0 }));
    }

    /// Update the ATAPI Interrupt Reason register (lives in the
    /// physical sector-count register, low byte). Bits:
    ///   bit 0 (C/D): 1 = command/packet phase, 0 = data
    ///   bit 1 (I/O): 1 = device→host (data-in), 0 = host→device
    ///   bit 2 (REL): bus release (we don't use)
    /// Standard combinations:
    ///   0x01 — host is writing the 12-byte packet to us
    ///   0x02 — we have data-in ready for the host
    ///   0x03 — command finished, host should read STATUS
    fn set_irr(&self, ir: u8) {
        self.write(REG_SECTOR_COUNT, ir as u16);
    }

    /// Force the Sega GD-ROM signature back into LBA_LOW/MID/HIGH and
    /// the sector count — used by DEVICE_RESET (soft reset). Matches
    /// the values the FPGA loads in its hardware reset state machine.
    fn load_atapi_signature(&self) {
        // LBA_LOW = disc format | status 0x02 (standby), LBA_MID=0x14,
        // (the FPGA's own reset value is 0x81 -- GD-ROM | pause -- not 0x82;
        // see `reassert_disc_format`.)
        // packed into addr 6. The format follows the mounted image rather
        // than being pinned to GD-ROM: the FPGA's reset state machine can
        // only load one fixed value, so a MIL-CD's CD-ROM XA format has to
        // be (re)asserted from here.
        self.write(REG_LBA_LOW_MID, 0x1400 | secnr(GD_STANDBY) as u16);
        // LBA_HIGH=0xEB low byte (LBA_LOW_PREV in high byte we leave alone).
        let high_lowp = self.read(REG_LBA_HIGH_LOWP);
        self.write(REG_LBA_HIGH_LOWP, (high_lowp & 0xFF00) | 0x00EB);
        self.write(REG_SECTOR_COUNT, 0x0001);
    }

    /// Set the GD-ROM "SECNR" byte the host reads back in Sector Count
    /// after a command: disc type in the high nibble, drive status low.
    /// Lives in the LBA_LOW slot's low byte; the byte-count half is kept.
    pub fn set_secnr(&self, v: u8) {
        let low_mid = self.read(REG_LBA_LOW_MID) & 0xFF00;
        self.write(REG_LBA_LOW_MID, low_mid | v as u16);
    }

    /// Re-apply the mounted disc's format to SECNR, keeping the drive status
    /// nibble that is currently latched.
    ///
    /// The FPGA's registers reset to a fixed GD-ROM signature -- LBA_LOW is
    /// `8'h81` in `fpga/amaranth/out/OpenGDEMUCore.v`, reloaded by its reset
    /// handling -- so every host bus reset silently reverts a MIL-CD's
    /// CD-ROM XA format to GD-ROM. Nothing used to put it back: the format is
    /// otherwise only written at `ide_task` startup and by DEVICE RESET, and a
    /// host *bus* reset is neither. The console then saw a disc that had
    /// announced itself as a CD go back to claiming GD-ROM mid-boot, which is
    /// the contradiction it refuses to boot across. Only the format nibble is
    /// touched, because the status nibble is the FPGA's to choose.
    fn reassert_disc_format(&self) {
        let low_mid = self.read(REG_LBA_LOW_MID);
        let status = (low_mid & 0x0F) as u8;
        self.write(REG_LBA_LOW_MID, (low_mid & 0xFF00) | (disc_format() | status) as u16);
    }

    fn write_fifo_level(&self) -> u16 {
        self.read(REG_WRITE_FIFO_LEVEL)
    }

    #[allow(dead_code)]
    fn read_fifo_level(&self) -> u16 {
        self.read(REG_READ_FIFO_LEVEL)
    }

    /// Log what the host has done on the bus since the last Command
    /// write: register reads, the live status byte it would see, whether
    /// INTRQ is still pending and the last Device Control write.
    fn log_host_activity(&self, label: &str) {
        let st = self.read(REG_HOST_STATUS_RDS);
        let data = self.read(REG_HOST_DATA_RDS);
        let pst = self.read(REG_HOST_PREV_STATUS_RDS);
        let pdata = self.read(REG_HOST_PREV_DATA_RDS);
        let dc = self.read(REG_HOST_DEVCTL);
        let hw = self.read(REG_HW_INFO);
        info!(
            "{}: host status_rds={} altstat_rds={} data_rds={} (prev cmd {}/{}/{}) status={:#04x} intrq={} nIEN={} devctl={:#04x} x{} rfifo={}",
            label, st & 0xFF, st >> 8, data, pst & 0xFF, pst >> 8, pdata,
            self.read(REG_STATUS) & 0xFF,
            self.read(REG_INTRQ_PENDING) & 1,
            (hw & HW_NIEN) != 0,
            dc & 0xFF, dc >> 8,
            self.read(REG_READ_FIFO_LEVEL),
        );
    }

    /// Pop one 16-bit word from the IDE-side write FIFO (data the host
    /// pushed into the Data register, e.g. an ATAPI command packet).
    fn pop_write_fifo(&self) -> u16 {
        self.read(REG_DATA_FIFO)
    }

    /// Push one 16-bit word into the IDE-side read FIFO (data the host
    /// will pop on its next Data register read).
    fn push_read_fifo(&self, word: u16) {
        self.write(REG_DATA_FIFO, word);
    }

    /// Return the host-requested ATAPI byte count (Cylinder Low / High,
    /// our LBA_MID / LBA_HIGH). Hosts use this to ask for a specific
    /// transfer length on data-in/data-out PIO transfers.
    fn atapi_byte_count(&self) -> u16 {
        let low_mid = self.read(REG_LBA_LOW_MID);
        let high_lowp = self.read(REG_LBA_HIGH_LOWP);
        let mid = (low_mid >> 8) as u8;
        let high = (high_lowp & 0xFF) as u8;
        ((high as u16) << 8) | (mid as u16)
    }

    fn set_atapi_byte_count(&self, count: u16) {
        // LBA_MID = byte count low, LBA_HIGH = byte count high. Preserve
        // the LBA_LOW byte (low byte of addr 6) and LBA_LOW_PREV (high
        // byte of addr 7) so we don't disturb the sliding-window state.
        let low_mid = self.read(REG_LBA_LOW_MID) & 0x00FF;
        self.write(REG_LBA_LOW_MID, low_mid | (((count & 0xFF) as u16) << 8));
        let high_lowp = self.read(REG_LBA_HIGH_LOWP) & 0xFF00;
        self.write(REG_LBA_HIGH_LOWP, high_lowp | ((count >> 8) & 0xFF));
    }
}

#[derive(defmt::Format)]
struct CommandSnapshot {
    /// Device/Head register. Bit 6 = LBA mode, bits 3..0 = LBA[27:24].
    device: u8,
    command: u8,
    features: u8,
    lba_low: u8,
    lba_mid: u8,
    lba_high: u8,
    /// LBA[31:24] under LBA48 -- the FPGA's sliding-window shadow of
    /// LBA_LOW, holding the host's *first* of its two writes.
    lba_low_prev: u8,
    /// LBA[39:32].
    lba_mid_prev: u8,
    /// LBA[47:40].
    lba_high_prev: u8,
    /// Full 16 bits. LBA28 commands use only the low byte.
    sector_count: u16,
}

impl CommandSnapshot {
    /// The 28-bit address, whose top nibble comes from the Device register.
    fn lba28(&self) -> u64 {
        ((self.device as u64 & 0x0F) << 24)
            | ((self.lba_high as u64) << 16)
            | ((self.lba_mid as u64) << 8)
            | (self.lba_low as u64)
    }

    /// The 48-bit address. The Device register carries no address bits here;
    /// the top three bytes are the shadow registers instead.
    fn lba48(&self) -> u64 {
        ((self.lba_high_prev as u64) << 40)
            | ((self.lba_mid_prev as u64) << 32)
            | ((self.lba_low_prev as u64) << 24)
            | ((self.lba_high as u64) << 16)
            | ((self.lba_mid as u64) << 8)
            | (self.lba_low as u64)
    }

    /// Starting LBA for this command, in whichever mode it was issued.
    fn lba(&self, ext: bool) -> u64 {
        if ext { self.lba48() } else { self.lba28() }
    }

    /// Sectors requested. Zero means the maximum, which differs between the
    /// two modes: 256 for LBA28's one byte, 65536 for LBA48's two.
    fn count(&self, ext: bool) -> u32 {
        if ext {
            if self.sector_count == 0 { 65536 } else { self.sector_count as u32 }
        } else if self.sector_count & 0xFF == 0 {
            256
        } else {
            (self.sector_count & 0xFF) as u32
        }
    }
}

/// Long-running task that services both IDE devices' command queues.
/// `gdi` is the GD-ROM image map for the primary device; the secondary
/// (HDD) currently ABRTs CD_READ.
///
/// The FPGA OR-merges per-device `command_pend` (with a 4-cycle delay)
/// onto a single active-low `MCU_IRQn` line. We drive this off
/// `embassy_atsam3::gpio::pioa_wait_irq()` — falling-edge interrupt on
/// PA22 — so the task is event-driven rather than polling. After each
/// wake-up we sweep both devices in case both arrived in the same edge.

/// Per-device state we keep on the MCU side (the FPGA register file
/// only models the wire-level register file; ATAPI semantics like
/// UNIT_ATTENTION pending live here).
struct DeviceState {
    /// Set on power-up and on hardware reset (DC_RSTn pulse). The first
    /// PACKET command other than REQUEST_SENSE/REQ_ERROR completes with
    /// CHECK_CONDITION, sense key 6 (UNIT_ATTENTION), per ATAPI; the
    /// host clears it by issuing REQ_SENSE next.
    unit_attention: bool,
    /// Sense key to report on the next REQ_SENSE / REQ_ERROR. Reflects
    /// the most-recent CHECK_CONDITION cause (6 = UA after reset, 0 =
    /// nothing pending). Reset to 0 once REQ_SENSE drains it.
    pending_sense_key: u8,
}

impl DeviceState {
    const fn new() -> Self {
        Self {
            unit_attention: true,
            pending_sense_key: 0x06, // UNIT_ATTENTION pending at boot
        }
    }
}

#[embassy_executor::task]
pub async fn ide_task(
    primary: IdeDevice,
    secondary: IdeDevice,
) {
    info!("IDE task started (gdi: {})", gdi_ref().is_some());

    // Assert the mounted disc's format before serving anything. The FPGA's
    // reset state machine loads a fixed GD-ROM SECNR, and until now nothing
    // but the CDDA paths ever wrote the register -- so a MIL-CD was served
    // to a host that had been told it was looking at a GD-ROM.
    primary.set_secnr(secnr(GD_STANDBY));
    info!("disc format: SECNR=0x{:02x}", secnr(GD_STANDBY));

    let mut primary_state = DeviceState::new();
    let mut secondary_state = DeviceState::new();

    // Always do a sweep at startup in case the FPGA already had
    // `command_pend` set when we attached (e.g. host issued a command
    // during MCU init).
    sweep(&primary, &mut primary_state, "primary", Role::Packet).await;
    sweep(&secondary, &mut secondary_state, "secondary", Role::Disk).await;

    loop {
        // The FPGA's command line, or a button press: the button signals
        // the same waker rather than having this loop poll, so an idle bus
        // stays idle and a swap still lands immediately.
        let _ = embassy_atsam3::gpio::pioa_wait_irq().await;
        sweep(&primary, &mut primary_state, "primary", Role::Packet).await;
        sweep(&secondary, &mut secondary_state, "secondary", Role::Disk).await;
        service_swap_request(&mut primary_state, "primary").await;
    }
}

/// Which kind of device an `IdeDevice` presents itself as.
///
/// The two share a bus, and the host tells them apart by the signature in
/// LBA_LOW/MID/HIGH plus which IDENTIFY they answer. A packet device ABRTs
/// IDENTIFY DEVICE so the host retries with IDENTIFY PACKET DEVICE; a plain
/// ATA disk does the opposite. Getting this backwards means the host either
/// never finds the disk or tries to send it ATAPI packets.
#[derive(Copy, Clone, PartialEq, Eq)]
pub enum Role {
    /// GD-ROM: ATAPI packet device, signature 0x81/0x14/0xEB.
    Packet,
    /// Hard disk: plain ATA, signature 0x01/0x00/0x00.
    Disk,
}

async fn sweep(
    dev: &IdeDevice,
    state: &mut DeviceState,
    label: &'static str,
    role: Role,
) {
    // Drain all currently-pending commands. The IRQ asserts while
    // command_pend is set, but we ack as part of `handle`, so the
    // line goes high again after each — re-checking here also covers
    // the race where a second command arrives before we returned to
    // the wait.
    while dev.command_pending() {
        // Command turnaround timing. The host arms a transfer and then polls
        // for DRQ with a finite patience; if we take longer than that to
        // respond it gives up and retries, and the retry's BSY is what both
        // sides then see. These two numbers say whether the MCU was late and,
        // if so, whether it was busy with the previous command or slow on
        // this one.
        let cmd_at = Instant::now();
        let gap_us = unsafe { LAST_CMD_DONE }
            .map(|t: Instant| (cmd_at - t).as_micros())
            .unwrap_or(0);
        unsafe { CMD_SEEN_AT = Some(cmd_at) };

        let snap = dev.snapshot_command();
        // A reset between commands is normal (the BIOS does one at boot);
        // one that landed mid-transfer already aborted the wait that saw
        // it. Either way, log it and clear the flag before the handler runs
        // so a wait inside it only trips on a *new* reset.
        {
            let hw = dev.read(REG_HW_INFO);
            if hw & HW_RESET_SEEN != 0 {
                info!("{}: host reset seen (nIEN={}, in_reset={})", label,
                      hw & HW_NIEN != 0, hw & HW_IN_RESET != 0);
                dev.clear_reset_seen();
                crate::cdda::stop();
                // The FPGA reloaded its fixed GD-ROM signature; put the
                // mounted disc's real format back before answering anything.
                dev.reassert_disc_format();
                info!("{}: disc format re-asserted after reset: SECNR low nibble kept, fmt={:#04x}",
                      label, disc_format());
            }
        }
        // Mirror command snapshot to UART — defmt/RTT goes away when
        // the DC power-cycles the board, but UART stays alive for the
        // host's screen/cat session. Keep this short so tail logs are
        // readable.
        info!(
            "{}: cmd 0x{:02x} feat 0x{:02x} sc {} lba {:02x}{:02x}{:02x}",
            label,
            snap.command,
            snap.features,
            snap.sector_count,
            snap.lba_high,
            snap.lba_mid,
            snap.lba_low,
        );
        handle(dev, state, label, role, &snap).await;
        let done = Instant::now();
        info!("{}: cmd {:#04x} gap={}us total={}us", label, snap.command,
               gap_us, (done - cmd_at).as_micros());
        dev.log_host_activity(label);
        unsafe { LAST_CMD_DONE = Some(done) };
    }
}

async fn handle(
    dev: &IdeDevice,
    state: &mut DeviceState,
    label: &'static str,
    role: Role,
    snap: &CommandSnapshot,
) {
    info!("{}: cmd={:?}", label, snap);
    match snap.command {
        CMD_NOP => {
            // ATA NOP. Real-world drives just succeed; the BIOS issues
            // it as a kind of "are you alive" probe. Returning ABRT
            // makes the BIOS retry forever.
            dev.set_error(0);
            dev.set_status_intrq(STATUS_IDLE);
            dev.ack_command();
        }
        CMD_DEVICE_RESET => {
            crate::cdda::stop();
            // Soft reset: reload ATAPI signature, return to idle, no error.
            dev.load_atapi_signature();
            dev.set_error(0);
            dev.set_status_intrq(STATUS_IDLE);
            dev.ack_command();
        }
        CMD_SET_FEATURES => {
            // We don't actually honor any feature bits yet — just ack so
            // the host's PIO-mode set / etc. can proceed.
            dev.set_error(0);
            dev.set_status_intrq(STATUS_IDLE);
            dev.ack_command();
        }
        CMD_IDENTIFY => match role {
            // ATA spec: a packet device shall ABRT IDENTIFY DEVICE so the
            // host falls back to IDENTIFY PACKET DEVICE.
            Role::Packet => {
                dev.set_error(ERR_ABRT);
                dev.set_status_intrq(STATUS_IDLE_ERR);
                dev.ack_command();
            }
            // A disk answers it -- this is how DreamShell / KOS discover an
            // IDE hard disk on the G1 bus.
            Role::Disk => {
                stage_identify_disk(dev);
                dev.set_error(0);
                dev.set_status_intrq(STATUS_DRDY | STATUS_DSC | STATUS_DRQ);
                dev.ack_command();
                // A PIO data-in command is not finished when the data is
                // staged -- DRQ has to drop once the host has taken it all,
                // or a host that polls for "DRQ clear, BSY clear" before
                // issuing the next command waits forever.
                finish_pio_in(dev).await;
            }
        },
        CMD_IDENTIFY_PACKET => match role {
            // 256 words = 512 bytes of ATAPI device ID. Stage it, then
            // signal DRQ=1 so the host can start reading.
            Role::Packet => {
                stage_identify_packet(dev);
                dev.set_atapi_byte_count(512);
                dev.set_error(0);
                dev.set_status_intrq(STATUS_DRDY | STATUS_DSC | STATUS_DRQ);
                dev.ack_command();
            }
            // Mirror image: a non-packet device ABRTs the packet IDENTIFY.
            Role::Disk => {
                dev.set_error(ERR_ABRT);
                dev.set_status_intrq(STATUS_IDLE_ERR);
                dev.ack_command();
            }
        },
        CMD_WRITE_SECTORS | CMD_WRITE_DMA | CMD_WRITE_SECTORS_EXT | CMD_WRITE_DMA_EXT
            if role == Role::Disk =>
        {
            let dma = matches!(snap.command, CMD_WRITE_DMA | CMD_WRITE_DMA_EXT);
            let ext = matches!(snap.command, CMD_WRITE_SECTORS_EXT | CMD_WRITE_DMA_EXT);
            handle_disk_write(dev, label, snap, dma, ext).await;
        }
        CMD_READ_SECTORS | CMD_READ_DMA | CMD_READ_SECTORS_EXT | CMD_READ_DMA_EXT
            if role == Role::Disk =>
        {
            let dma = matches!(snap.command, CMD_READ_DMA | CMD_READ_DMA_EXT);
            let ext = matches!(snap.command, CMD_READ_SECTORS_EXT | CMD_READ_DMA_EXT);
            handle_disk_read(dev, label, snap, dma, ext).await;
        }
        crate::fwupdate::CMD_FW if role == Role::Disk => {
            handle_fw_command(dev, label, snap);
        }
        CMD_PACKET => {
            handle_packet(dev, state, label).await;
            // ack_command happens inside handle_packet after the packet
            // protocol fully completes.
        }
        other => {
            warn!("{}: unhandled cmd 0x{:02x}", label, other);
            dev.set_error(ERR_ABRT);
            dev.set_status_intrq(STATUS_IDLE_ERR);
            dev.ack_command();
        }
    }
}

// ---------------------------------------------------------------------------
// PACKET (0xA0) command — ATAPI packet protocol.
//
// Sequence:
//   1. Host writes 0xA0 to the Command register. FPGA latches command_pend
//      and, for the primary (ATAPI) device, opens the CDB phase itself:
//      IRR=0x01, DRQ=1, BSY=0 within one core clock.
//   2. We just acknowledge the command latch.
//   3. Host writes 6 16-bit words to the Data register. FPGA pushes them
//      into write_fifo. We poll write_fifo level until it hits 6.
//   4. We pop the 12-byte packet, parse opcode = byte 0.
//   5. For data-in commands: stage response into read_fifo, set
//      byte-count registers, DRQ=1, BSY=0. Host PIO-reads until done.
//   6. We clear DRQ, leave STATUS=0x50 idle. ack_command() releases the
//      command_pend latch so the next command can land.
// ---------------------------------------------------------------------------

const PACKET_LEN_WORDS: usize = 6;
const PACKET_LEN_BYTES: usize = 12;

// ATAPI command opcodes (subset).
const CMD_READ_SECTORS: u8 = 0x20;
const CMD_READ_DMA: u8 = 0xC8;
const CMD_WRITE_SECTORS: u8 = 0x30;
const CMD_WRITE_DMA: u8 = 0xCA;
/// The LBA48 forms. Same transfers, but the address comes from the sliding
/// window instead of the Device register, and the sector count is 16 bits.
const CMD_READ_SECTORS_EXT: u8 = 0x24;
const CMD_READ_DMA_EXT: u8 = 0x25;
const CMD_WRITE_SECTORS_EXT: u8 = 0x34;
const CMD_WRITE_DMA_EXT: u8 = 0x35;

/// Direction of an armed DMA transfer, as the FPGA's slot-19 register encodes it.
#[derive(Copy, Clone, PartialEq, Eq)]
#[repr(u16)]
enum DmaMode {
    Off = 0,
    /// Device to host: CD_READ, disk reads.
    ToHost = 1,
    /// Host to device: disk writes.
    ToDevice = 2,
}

/// The largest sector count a 28-bit address can name. Anything past this is
/// reachable only through the LBA48 command set.
const MAX_LBA28_SECTORS: u32 = 0x1000_0000;

/// Addressable sectors on the emulated disk, published in IDENTIFY. Set once
/// the SD card is up; the whole card is exposed, MBR included, so the host
/// sees a normal partitioned disk and can mount the same filesystem that
/// holds the GD-ROM images.
static DISK_SECTORS: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

pub fn set_disk_sectors(n: u32) {
    DISK_SECTORS.store(n, core::sync::atomic::Ordering::Relaxed);
}

/// Size the emulated disk from the card's partition table, falling back to
/// `csd_blocks` if there is no usable MBR.
///
/// The CSD-derived figure is not trustworthy here: on this card it reports
/// 2,688,000 blocks (1.3 GB) for what the partition table says is a 500 GB
/// volume -- a ~360x undercount, most likely a CSD v2 bit-alignment problem in
/// the HSMCI response registers. Since disk reads range-check against this
/// number, believing it made everything past LBA 2,688,000 unreachable while
/// still appearing to work, because the root directory happens to land below
/// the cut. Take the end of the furthest partition instead; main.rs already
/// noted the partition table is the authoritative source.
pub fn size_disk_from_mbr(csd_blocks: u32) {
    let mut mbr = [0u8; 512];
    if unsafe { embassy_atsam3::hsmci::read_block(0, &mut mbr) }.is_err() {
        warn!("disk sizing: MBR read failed, using CSD value {}", csd_blocks);
        set_disk_sectors(csd_blocks);
        return;
    }
    if mbr[510] != 0x55 || mbr[511] != 0xAA {
        warn!("disk sizing: no MBR signature, using CSD value {}", csd_blocks);
        set_disk_sectors(csd_blocks);
        return;
    }
    let mut end = 0u32;
    for i in 0..4 {
        let e = &mbr[446 + i * 16..446 + i * 16 + 16];
        if e[4] == 0 {
            continue; // unused entry
        }
        let start = u32::from_le_bytes([e[8], e[9], e[10], e[11]]);
        let count = u32::from_le_bytes([e[12], e[13], e[14], e[15]]);
        end = end.max(start.saturating_add(count));
    }
    if end == 0 {
        warn!("disk sizing: no partitions, using CSD value {}", csd_blocks);
        set_disk_sectors(csd_blocks);
    } else {
        info!("disk sizing: {} blocks from partition table (CSD said {})", end, csd_blocks);
        set_disk_sectors(end);
    }
}

const ATAPI_TEST_UNIT_READY: u8 = 0x00;
const ATAPI_REQUEST_SENSE: u8 = 0x03;
const ATAPI_START_STOP_UNIT: u8 = 0x1B;

/// Whether the GD-ROM currently has media loaded.
///
/// This is the coherency interlock for the shared card. The emulated disk on
/// the secondary exposes the same FAT32 volume the GD-ROM images live on, so
/// the host must be able to tell us "I am about to touch that filesystem" and
/// have us let go first. Ejecting drops the staged-sector cache and makes
/// CD_READ report NOT READY; loading raises a media-change UNIT ATTENTION so
/// the host re-reads the TOC.
static MEDIA_PRESENT: core::sync::atomic::AtomicBool =
    core::sync::atomic::AtomicBool::new(true);

/// The currently mounted GD-ROM image.
///
/// Owned here rather than handed in as a `&'static` so it can be replaced: a
/// StaticCell initialises once, which meant an image added while ejected could
/// never be picked up. Only `ide_task` (and `main` before that task spawns)
/// touches this, on a single-threaded executor with no interrupt access.
static mut GDI: Option<crate::gdi::Gdi> = None;

/// Command turnaround instrumentation. Only `ide_task` touches these.
static mut CMD_SEEN_AT: Option<Instant> = None;
static mut LAST_CMD_DONE: Option<Instant> = None;

/// Replace the mounted image. Safe to call before `ide_task` starts or from
/// inside it; nothing else may.
pub fn mount_gdi(g: crate::gdi::Gdi) {
    unsafe { GDI = Some(g) };
}

pub fn gdi_ref() -> Option<&'static crate::gdi::Gdi> {
    unsafe { (*core::ptr::addr_of!(GDI)).as_ref() }
}

/// Prefetch IP.BIN for whatever is currently mounted, if anything.
pub async fn prefetch_mounted() {
    if let Some(g) = gdi_ref() {
        prefetch_ip_bin(g).await;
    }
}

/// Re-read the card and mount whatever image is there now. Called on LOAD so
/// a GDI added or removed while ejected is actually picked up.
async fn rescan_media() -> bool {
    invalidate_cd_read_cache();
    match crate::gdi::load_default(disk_sectors()) {
        Ok(g) => {
            mount_gdi(g);
            if let Some(g) = gdi_ref() {
                prefetch_ip_bin(g).await;
            }
            true
        }
        Err(e) => {
            warn!("rescan_media: no usable image after load: {:?}", e);
            unsafe { GDI = None };
            false
        }
    }
}

/// The LOAD half of a tray cycle: mount whatever is selected now and tell
/// the host the medium may have changed, so it re-reads the TOC rather
/// than trusting what it cached before. Returns whether an image mounted.
async fn load_media(state: &mut DeviceState) -> bool {
    crate::cdda::stop();
    let ok = rescan_media().await;
    MEDIA_PRESENT.store(ok, core::sync::atomic::Ordering::Relaxed);
    state.unit_attention = true;
    state.pending_sense_key = 0x06;
    ok
}

/// Swap discs without the host's help: eject, mount the selection, load.
/// This is what the GDEMU-style commands imply -- the menu that sends them
/// returns straight to the BIOS, which expects to find the new disc there.
/// A swap the button has asked for, as the folder to mount.
///
/// A name, not an index: which discs exist is the menu's business, and the
/// firmware should not have to enumerate the card to be told "mount that
/// one". The button cannot perform the swap itself either -- that needs the
/// device state the IDE task owns -- so it leaves the name here and the task
/// picks it up between commands.
static SWAP_PENDING: core::sync::atomic::AtomicBool =
    core::sync::atomic::AtomicBool::new(false);
static mut SWAP_NAME: heapless::String<{ gamecfg::MAX_NEXT }> = heapless::String::new();

/// Ask for the folder called `name` to be mounted. Called from the button.
pub fn request_swap(name: &str) {
    // SAFETY: written only here and read only by the IDE task after the flag
    // is observed, and the button is the only writer.
    let buf = unsafe { &mut *core::ptr::addr_of_mut!(SWAP_NAME) };
    buf.clear();
    for c in name.chars() {
        if buf.push(c).is_err() {
            break;
        }
    }
    SWAP_PENDING.store(true, core::sync::atomic::Ordering::Release);
}

/// Perform a pending button swap, if one is waiting.
///
/// Mounts by name, which reads one directory rather than enumerating the
/// card. That is the whole point: enumeration returns a sixteen-entry vector
/// by value and this part has about two kilobytes of stack.
async fn service_swap_request(state: &mut DeviceState, label: &'static str) {
    if !SWAP_PENDING.swap(false, core::sync::atomic::Ordering::Acquire) {
        return;
    }
    let name = unsafe { &*core::ptr::addr_of!(SWAP_NAME) };
    crate::cdda::stop();
    MEDIA_PRESENT.store(false, core::sync::atomic::Ordering::Relaxed);
    invalidate_cd_read_cache();

    let ok = match crate::gdi::load_by_name(disk_sectors(), name.as_str()) {
        Ok(g) => {
            mount_gdi(g);
            if let Some(g) = gdi_ref() {
                prefetch_ip_bin(g).await;
            }
            true
        }
        Err(e) => {
            warn!("{}: button swap to '{}' failed: {:?}", label, name.as_str(), e);
            unsafe { GDI = None };
            false
        }
    };
    MEDIA_PRESENT.store(ok, core::sync::atomic::Ordering::Relaxed);
    state.unit_attention = true;
    state.pending_sense_key = 0x06;
    info!("{}: button swapped to '{}' (mounted: {})", label, name.as_str(), ok);
}

async fn swap_media(state: &mut DeviceState, label: &'static str) -> bool {
    MEDIA_PRESENT.store(false, core::sync::atomic::Ordering::Relaxed);
    invalidate_cd_read_cache();
    let ok = load_media(state).await;
    info!("{}: swapped media (mounted: {})", label, ok);
    ok
}


fn media_present() -> bool {
    MEDIA_PRESENT.load(core::sync::atomic::Ordering::Relaxed)
}
// Sega GD-ROM ATAPI extensions.
const SEGA_REQ_STAT: u8 = 0x10;
const SEGA_REQ_MODE: u8 = 0x11;
const SEGA_SET_MODE: u8 = 0x12;
const SEGA_REQ_ERROR: u8 = 0x13;
const SEGA_GET_TOC: u8 = 0x14;
const SEGA_REQ_SES: u8 = 0x15;
const SEGA_CD_OPEN: u8 = 0x16;
const SEGA_CD_PLAY: u8 = 0x20;
const SEGA_CD_SEEK: u8 = 0x21;
const SEGA_CD_SCAN: u8 = 0x22;
const SEGA_CD_READ: u8 = 0x30;
const SEGA_CD_READ2: u8 = 0x31;
const SEGA_GET_SCD: u8 = 0x40;
/// Sega vendor handshake. The BIOS issues 0x70 and 0x71 during disc
/// authentication and refuses to mount the disc unless the device
/// returns the exact 6-byte sequence `BA 06 0D CA 6A 1F` (per
/// iceGDROM's reverse engineering — ide.c:197-199 in that repo).
/// OpenGDEMU image control. Vendor space Sega leaves alone, so it cannot
/// collide with a real GD-ROM command. Data-in replies honour an allocation
/// length in packet[3..=4] (big-endian, 0 = everything).
///
/// Discs are named, never numbered. The console reads the card's filesystem
/// itself, so the firmware does not enumerate anything and there is no list
/// for an index to be an index *into* -- see `handle_image_ctl` for what that
/// replaced and why.
///
///   packet[1] = 2  MOUNT     -> mount the name staged by MOUNT_NAME, or the
///                               configured default if packet[2] = 0xFF or no
///                               name is staged. Swaps immediately, no
///                               eject/load needed; UNIT ATTENTION follows.
///   packet[1] = 3  VERSION   -> "OGDM", u8 protocol, 3 reserved bytes (zero),
///                               24-byte firmware string.
///   packet[1] = 5  MOUNT_NAME-> packet[2] = chunk index, packet[3..12] = up
///                               to nine bytes of the folder name. Chunk 0
///                               starts a new name; MOUNT consumes it.
const OGDM_IMAGE_CTL: u8 = 0xE0;
/// Bumped when the 0xE0 reply layouts or field meanings change.
///
/// 2: VERSION's max-images, count and selected bytes became reserved zeros.
///    The firmware keeps no catalog -- it mounts by name, opening one
///    directory -- so none of the three described anything. The reply is the
///    same 32 bytes; only the meaning of 5..8 changed.
const OGDM_PROTOCOL_VERSION: u8 = 2;
/// MOUNT's packet[2], meaning "forget the runtime choice and mount whatever
/// GDEMU.CFG names" -- what a power cycle would do.
const NO_SELECTION: u8 = 0xFF;
/// Disc format: the high nibble of SECNR.
///
/// Values and packing cross-checked against flycast, which is the closest
/// thing to a spec for this register: `core/hw/gdrom/gdrom_if.h` enumerates
/// `CdDA=0x00, CdRom=0x10, CdRom_XA=0x20, CdRom_Extra=0x30, CdRom_CDI=0x40,
/// GdRom=0x80`, and `gdromv3.h` declares SECNR as `Status:4` then
/// `DiscFormat:4` -- disc format high, drive status low, exactly as this
/// firmware already assumed. (`CdRom_CDI=0x40` is CD-**I**, an unrelated
/// format; it has nothing to do with DiscJuggler `.cdi` files.)
const DISC_FMT_CDROM_XA: u8 = 0x20;
const DISC_FMT_GDROM: u8 = 0x80;

/// Drive status: the low nibble of SECNR (flycast `gdromv3.h:301`).
pub const GD_PAUSE: u8 = 0x01;
pub const GD_STANDBY: u8 = 0x02;
pub const GD_PLAY: u8 = 0x03;

/// The disc format to report for whatever is mounted.
///
/// A GDI is a GD-ROM. A CDI is a MIL-CD, and a MIL-CD is a CD: its data
/// track is Mode 2, which is what a real drive reports as CD-ROM XA --
/// flycast's `GuessDiscType` returns `CdRom_XA` for any image with a Mode 2
/// track (`core/imgread/common.cpp`). Reporting GD-ROM for one contradicts
/// the disc's own IP.BIN, which says `CD-ROM1/1` in its media field, and the
/// BIOS refuses to boot across that contradiction: it reads IP.BIN, the
/// ISO9660 PVD and the root directory, then abandons the boot without ever
/// reading the `1ST_READ.BIN` the directory just pointed it at.
pub fn disc_format() -> u8 {
    match gdi_ref() {
        Some(g) if g.is_cdi => DISC_FMT_CDROM_XA,
        _ => DISC_FMT_GDROM,
    }
}

/// SECNR byte for a drive status: mounted disc's format high, `status` low.
pub fn secnr(status: u8) -> u8 {
    disc_format() | status
}
const SEGA_VENDOR_70: u8 = 0x70;
const SEGA_VENDOR_71: u8 = 0x71;

async fn handle_packet(
    dev: &IdeDevice,
    state: &mut DeviceState,
    label: &'static str,
) {
    // FEATURES bit 0 = DMA mode requested. The data-in path now
    // supports DMA via FPGA register 19 (REG_DMA_MODE) — see
    // `stream_from_sd` for the arm/disarm. Carry the flag through.
    let snap = dev.snapshot_command();
    let dma_requested = (snap.features & 0x01) != 0;
    // The host's PIO byte-count limit (ATAPI 10.8.10) lives in LBA Mid/High
    // at command time. Read it now: the data-in setters overwrite those
    // registers with our own byte counts.
    let byte_limit = (snap.lba_mid as u16) | ((snap.lba_high as u16) << 8);

    // Step 2: the FPGA opened the CDB phase (IRR C/D=1, DRQ=1, BSY=0,
    // packet word count armed) on the Command write itself -- by the time
    // we get here the host may already have written all six words and
    // the FPGA raised BSY on the last one. Writing Status here would
    // reopen DRQ over a CDB that is already in the write FIFO, so only
    // release the command latch.
    dev.ack_command();

    // Step 3: wait for 6 words on the write FIFO.
    let packet = match recv_packet(dev).await {
        Ok(p) => p,
        Err(()) => {
            warn!("{}: timeout waiting for ATAPI packet", label);
            dev.set_error(ERR_ABRT);
            dev.set_status_intrq(STATUS_IDLE_ERR);
            return;
        }
    };

    let opcode = packet[0];
    info!("{}: ATAPI opcode 0x{:02x} packet={:?}", label, opcode, packet);
    info!(
        "{}: ATAPI 0x{:02x}  [{:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x}]",
        label, opcode,
        packet[0], packet[1], packet[2], packet[3],
        packet[4], packet[5], packet[6], packet[7],
        packet[8], packet[9], packet[10], packet[11],
    );

    // ATAPI: when UNIT_ATTENTION is pending (set on reset), every PACKET
    // command other than REQUEST_SENSE / REQ_ERROR completes immediately
    // with CHECK_CONDITION and sense key 6, and the UA flag is cleared.
    // The host is expected to follow up with REQ_SENSE to read the sense
    // data, then retry the original command. Without this the BIOS sees
    // a "ready" drive after reset and never proceeds past TOC validation.
    // (Verified via dcload-serial against a stock GDEMU: TEST_UNIT_READY
    // first response = STATUS=0x11, ERR=0x60.)
    if state.unit_attention
        && opcode != ATAPI_REQUEST_SENSE
        && opcode != SEGA_REQ_ERROR
    {
        info!("{}: UNIT_ATTENTION served on opcode 0x{:02x}", label, opcode);
        dev.set_error(0x60); // sense key 6 << 4
        dev.set_irr(0x03);
        state.unit_attention = false;
        // Leave pending_sense_key at 6 — the host should follow up with
        // REQ_SENSE next, and we want to report UNIT_ATTENTION there.
        dev.set_status_intrq(STATUS_DRDY | STATUS_DSC | STATUS_ERR); // 0x51
        return;
    }

    // Step 4: dispatch.
    match opcode {
        ATAPI_TEST_UNIT_READY => {
            // No data, just "I'm here" -- unless the tray is open.
            dev.set_irr(0x03);
            if media_present() {
                dev.set_error(0);
                dev.set_status_intrq(STATUS_IDLE);
            } else {
                // NOT READY / MEDIUM NOT PRESENT.
                state.pending_sense_key = 0x02;
                dev.set_error(0x20);
                dev.set_status_intrq(STATUS_IDLE_ERR);
            }
        }
        ATAPI_START_STOP_UNIT => {
            // CDB byte 4: bit 1 = LoEj, bit 0 = Start. LoEj clear is just a
            // spin up/down request, which means nothing here.
            let loej = packet[4] & 0x02 != 0;
            let start = packet[4] & 0x01 != 0;
            if loej {
                if start {
                    // Re-read the card: the whole point of the eject window is
                    // that the host may have added or removed images while it
                    // was open, so trusting the previously parsed GDI would
                    // defeat it.
                    let ok = load_media(state).await;
                    info!("{}: LOAD -- media present (image: {})", label, ok);
                } else {
                    info!("{}: EJECT -- releasing media", label);
                    MEDIA_PRESENT.store(false, core::sync::atomic::Ordering::Relaxed);
                    // Staged sectors are no longer trustworthy: the host is
                    // free to rewrite the filesystem they came from -- and
                    // for the same reason the image list, if anything built
                    // one, may no longer describe what is on the card.
                    invalidate_cd_read_cache();
                    crate::cdda::stop();
                }
            }
            dev.set_irr(0x03);
            dev.set_error(0);
            dev.set_status_intrq(STATUS_IDLE);
        }
        ATAPI_REQUEST_SENSE => {
            // 18 bytes of fixed-format sense data. Allocation length is
            // packet byte 4. Byte 2 reports the current sense key — we
            // patch the canned reply with state.pending_sense_key so the
            // host learns about the UA condition. Reading sense data
            // clears any pending UNIT_ATTENTION condition per ATAPI spec.
            let alloc = packet[4] as u16;
            let mut sense = REQUEST_SENSE_REPLY;
            sense[2] = state.pending_sense_key;
            let n = core::cmp::min(alloc, sense.len() as u16);
            send_data_in(dev, &sense[..n as usize]).await;
            state.unit_attention = false;
            state.pending_sense_key = 0;
        }
        SEGA_REQ_STAT => {
            let alloc = packet[4] as u16;
            let reply = build_req_stat_reply();
            let n = core::cmp::min(alloc, reply.len() as u16);
            send_data_in(dev, &reply[..n as usize]).await;
        }
        SEGA_REQ_MODE => {
            // packet[2] = offset, packet[4] = length. Per iceGDROM
            // rv32/source/ide.c:202-213 the BIOS only ever asks for
            // (offset=18, len=8) → "Rev 5.07" version string and
            // (offset=0, len=10) → all zeros. Anything else aborts
            // with status 0x50.
            let offset = packet[2] as usize;
            let len = packet[4] as usize;
            if offset == 18 && len == 8 {
                // Stock GDEMU returns "Rev 6.42" here (verified via
                // g1bios_replay against a real GDEMU). iceGDROM's docs
                // claimed "Rev 5.07" but that appears to be wrong.
                send_data_in(dev, b"Rev 6.42").await;
            } else if offset == 0 && len == 10 {
                // Stock GDEMU returns [00 00 00 00 00 b4 19 00 00 08].
                // The b4 19 00 00 looks like a 32-bit value in some
                // big-endian-ish layout; meaning unknown but BIOS-
                // visible. Match stock byte-for-byte.
                let reply: [u8; 10] = [0x00, 0x00, 0x00, 0x00, 0x00,
                                       0xb4, 0x19, 0x00, 0x00, 0x08];
                send_data_in(dev, &reply).await;
            } else {
                warn!("{}: REQ_MODE off={} len={} unsupported", label, offset, len);
                dev.set_error(ERR_ABRT);
                dev.set_status_intrq(STATUS_IDLE_ERR);
            }
        }
        SEGA_SET_MODE => {
            // Sega SET_MODE is a data-OUT command: host writes
            // `packet[4]` bytes (typically 10) to the drive. Without
            // draining that phase the host hangs waiting for DRQ to
            // clear and the BIOS's post-CD_READ "configure drive" step
            // never completes — game launch never proceeds. Read and
            // log the bytes; we don't yet act on them.
            let len = packet[4] as usize;
            let data = recv_data_out(dev, len).await;
            info!(
                "{}: SET_MODE got {} bytes: {=[u8]:02x}", label, data.len(), data.as_slice()
            );
            dev.set_irr(0x03);
            dev.set_status_intrq(STATUS_IDLE);
        }
        SEGA_CD_PLAY => {
            // [1] parameter type, [2..5] start, [6] repeat count (low
            // nibble, 15 = forever), [8..11] end. Type 1 = FAD (24-bit BE),
            // 2 = MSF, 3 = resume from pause. Layout per iceGDROM cdda.c.
            let ptype = packet[1] & 0x0F;
            let repeat = packet[6] & 0x0F;
            let be24 = |b: &[u8]| ((b[0] as u32) << 16) | ((b[1] as u32) << 8) | b[2] as u32;
            let msf = |b: &[u8]| ((b[0] as u32) * 60 + b[1] as u32) * 75 + b[2] as u32;
            let ok = match ptype {
                1 => crate::cdda::play(be24(&packet[2..5]), be24(&packet[8..11]), repeat).is_ok(),
                2 => crate::cdda::play(msf(&packet[2..5]), msf(&packet[8..11]), repeat).is_ok(),
                3 => { crate::cdda::resume(); true }
                _ => false,
            };
            if !ok {
                warn!("{}: CD_PLAY rejected: {=[u8]:02x}", label, packet[..12]);
            }
            dev.set_irr(0x03);
            if ok {
                dev.set_secnr(secnr(GD_PLAY));
                dev.set_error(0);
                dev.set_status_intrq(STATUS_IDLE);
            } else {
                dev.set_error(ERR_ABRT);
                dev.set_status_intrq(STATUS_IDLE_ERR);
            }
        }
        SEGA_CD_SEEK => {
            // Parameter type 4 = pause; anything else (seek to FAD/MSF,
            // stop) ends playback, as iceGDROM does.
            if packet[1] & 0x0F == 4 {
                crate::cdda::pause();
                dev.set_secnr(secnr(GD_PAUSE));
            } else {
                crate::cdda::stop();
                dev.set_secnr(secnr(GD_STANDBY));
            }
            dev.set_irr(0x03);
            dev.set_error(0);
            dev.set_status_intrq(STATUS_IDLE);
        }
        SEGA_CD_OPEN | SEGA_CD_SCAN => {
            // No-data commands — just succeed. Status phase, no error.
            // Must assert INTRQ for the host's command-complete handler;
            // without this the BIOS waits forever after issuing CD_SEEK
            // post-REQ_SES and falls into CD_SCD spam.
            dev.set_irr(0x03);
            dev.set_error(0);
            dev.set_status_intrq(STATUS_IDLE);
        }
        SEGA_REQ_ERROR => {
            let alloc = packet[4] as u16;
            let n = core::cmp::min(alloc, REQ_ERROR_REPLY.len() as u16);
            send_data_in(dev, &REQ_ERROR_REPLY[..n as usize]).await;
        }
        SEGA_GET_TOC => {
            // Sega GET_TOC packet: byte 1 selects the session/area
            // (0 = SDA, 1 = HDA), bytes 3..4 = allocation length BE.
            // Build a real TOC from the parsed GDI when we have one;
            // fall back to ABRT if we don't (the BIOS shouldn't ever
            // ask GET_TOC before a disc is present, but be safe).
            if let Some(g) = gdi_ref() {
                let area = packet[1] & 0x01;
                let alloc = ((packet[3] as u16) << 8) | packet[4] as u16;
                let mut toc = [0u8; 408];
                g.build_toc(area, &mut toc);
                // Dump first 32 bytes of TOC + the trailing 12 bytes
                // (first/last/leadout) so we can compare against a known
                // reference at the bench.
                info!(
                    "{} GET_TOC area={} hd={:02x}{:02x}{:02x}{:02x} {:02x}{:02x}{:02x}{:02x} t3={:02x}{:02x}{:02x}{:02x} f={:02x}{:02x}{:02x}{:02x} l={:02x}{:02x}{:02x}{:02x} lo={:02x}{:02x}{:02x}{:02x}",
                    label, area,
                    toc[0], toc[1], toc[2], toc[3],
                    toc[4], toc[5], toc[6], toc[7],
                    toc[8], toc[9], toc[10], toc[11],
                    toc[396], toc[397], toc[398], toc[399],
                    toc[400], toc[401], toc[402], toc[403],
                    toc[404], toc[405], toc[406], toc[407],
                );
                let n = core::cmp::min(alloc as usize, toc.len());
                let n_aligned = n & !1; // host expects 16-bit-aligned PIO; trim odd trailing byte
                send_data_in(dev, &toc[..n_aligned]).await;
            } else {
                warn!("{}: GET_TOC with no GDI loaded", label);
                dev.set_error(ERR_ABRT);
                dev.set_status_intrq(STATUS_IDLE_ERR);
            }
        }
        SEGA_REQ_SES => {
            // Sega REQ_SES: packet[2] is a 0-indexed session number;
            // BIOS asks for sessions 0..num_sessions and our reply
            // describes that session's first track + start FAD. Session
            // 0 = SDA, session 1 = HDA on a GD-ROM. Sessions outside
            // the disc's range get ABRT (the BIOS uses that to learn
            // num_sessions). Per iceGDROM rv32/source/ide.c:437.
            let alloc = packet[4] as u16;
            let session = packet[2];
            match build_req_ses_reply(gdi_ref(), session) {
                Some(reply) => {
                    info!(
                        "{}: REQ_SES({}) -> {=[u8]:02x}", label, session, reply.as_slice()
                    );
                    let n = core::cmp::min(alloc as usize, reply.len());
                    send_data_in(dev, &reply[..n]).await;
                }
                None => {
                    info!(
                        "{}: REQ_SES({}) out of range, ABRT", label, session
                    );
                    dev.set_irr(0x03);
                    dev.set_error(ERR_ABRT);
                    dev.set_status_intrq(STATUS_IDLE_ERR);
                }
            }
        }
        SEGA_GET_SCD => {
            let alloc = ((packet[3] as u16) << 8) | packet[4] as u16;
            let mut reply = [0u8; 100];
            let len = build_get_scd_reply(packet[1] & 0x0F, &mut reply);
            let n = core::cmp::min(alloc, len as u16);
            send_data_in(dev, &reply[..n as usize]).await;
        }
        SEGA_CD_READ | SEGA_CD_READ2 => {
            handle_cd_read(dev, label, &packet, dma_requested, byte_limit).await;
        }
        OGDM_IMAGE_CTL => {
            handle_image_ctl(dev, state, label, &packet).await;
        }
        SEGA_VENDOR_70 => {
            // Per iceGDROM rv32/source/ide.c — 0x70 is a no-data probe
            // (treated identically to TEST_UNIT_READY — `finish_packet_ok`).
            // Earlier this branch returned the 6-byte magic; that was
            // wrong, the magic is for 0x71 only.
            dev.set_irr(0x03);
            dev.set_error(0);
            dev.set_status_intrq(STATUS_IDLE);
        }
        SEGA_VENDOR_71 => {
            // Sega disc-auth challenge. The reply is a fixed 6-byte
            // magic and the BIOS expects the full 6 bytes regardless
            // of what's in packet[4] (per iceGDROM, which ignores the
            // alloc-len field for this command).
            send_data_in(dev, &VENDOR_HANDSHAKE_REPLY).await;
        }
        _ => {
            warn!("{}: unhandled ATAPI opcode 0x{:02x}", label, opcode);
            dev.set_irr(0x03);
            dev.set_error(ERR_ABRT);
            dev.set_status_intrq(STATUS_IDLE_ERR);
        }
    }
    // Every arm posts its completion status and INTRQ together; there is
    // deliberately no catch-all assert_intrq here. A second INTRQ after
    // the host has already read Status for the first one rides into its
    // next command as a spurious interrupt.
}

/// Sega CD_READ packet layout:
///
/// ```text
///   byte 0     : 0x30
///   byte 1     : data type / parameters
///   byte 2..4  : starting FAD (frame addr; LBA + 150), big-endian 24-bit
///   byte 5..7  : reserved
///   byte 8..10 : sector count, big-endian 24-bit
///   byte 11    : reserved
/// ```
///
/// FAD = LBA + 150 in CD-frame addressing (the 150-frame offset is the
/// 2-second pre-gap before track 1 in MSF/CD-DA terms).
async fn handle_cd_read(
    dev: &IdeDevice,
    label: &'static str,
    packet: &[u8],
    dma: bool,
    byte_limit: u16,
) {
    let starting_fad = ((packet[2] as u32) << 16)
        | ((packet[3] as u32) << 8)
        | packet[4] as u32;
    let count = ((packet[8] as u32) << 16)
        | ((packet[9] as u32) << 8)
        | packet[10] as u32;
    let lba = starting_fad.saturating_sub(150);

    info!(
        "{}: CD_READ FAD={} ({} sectors), LBA={}, dma={}",
        label, starting_fad, count, lba, dma
    );
    info!(
        "{}: CD_READ FAD={} ({}sect) LBA={} dma={}",
        label, starting_fad, count, lba, dma
    );

    if !media_present() {
        // NOT READY / MEDIUM NOT PRESENT -- the host ejected and has not
        // loaded again, so it may be rewriting the filesystem right now.
        info!("{}: CD_READ while ejected", label);
        dev.set_irr(0x03);
        dev.set_error(0x20);
        dev.set_status_intrq(STATUS_IDLE_ERR);
        return;
    }
    let Some(gdi) = gdi_ref() else {
        warn!("{}: CD_READ but no GDI loaded", label);
        dev.set_error(ERR_ABRT);
        dev.set_status_intrq(STATUS_IDLE_ERR);
        return;
    };

    // For now: read the (cooked) 2048-byte data area of each sector.
    // Real GD-ROM mode 1 sectors on disk are 2352 bytes raw; for the
    // user-data subset we'd want a proper sync/header skip — punt for
    // now and read the whole 2352-byte sector as-is, sending the first
    // 2048 bytes to the host. Good enough to validate the SD path.
    const SECTOR_BYTES: usize = 2048;
    let total = (count as usize).saturating_mul(SECTOR_BYTES);
    // No upper bound on the request size any more: stream_from_sd stages the
    // transfer a chunk at a time rather than buffering it whole, so a request
    // is limited by the disc, not by our RAM. Games ask for far more than the
    // BIOS does -- the first real content read observed here was 572 sectors
    // (1.1 MB), which the old 64 KB guard aborted outright.
    //
    // Still reject a request that runs off the end of the track; better an
    // ABRT the host can see than a read that silently returns garbage.
    let track_sectors = gdi.track_for_lba(lba).map(|t| t.sector_count).unwrap_or(0);
    let available = track_sectors.saturating_sub(lba.saturating_sub(
        gdi.track_for_lba(lba).map(|t| t.start_lba).unwrap_or(0),
    ));
    if total == 0 || count > available {
        warn!(
            "{}: CD_READ rejected: {} sectors from LBA {} ({} available in track)",
            label, count, lba, available
        );
        dev.set_error(ERR_ABRT);
        dev.set_status_intrq(STATUS_IDLE_ERR);
        return;
    }
    stream_from_sd(dev, gdi, lba, count as u32, total, dma, byte_limit).await;
}

/// Maximum CD_READ payload we hold in RAM up-front. Sized to fit the
/// BIOS's 7-sector IP.BIN read (14 KB) — freed the 16 KB defmt-rtt
/// ring buffer to make room. Larger requests get truncated to this
/// and short-transferred.
const CD_READ_BUF_BYTES: usize = 14 * 1024;
#[link_section = ".sram1"]
static mut CD_READ_BUF: [u8; CD_READ_BUF_BYTES] = [0; CD_READ_BUF_BYTES];

/// Tracks the LBA range currently sitting in CD_READ_BUF. Set by
/// `prefetch_ip_bin()` at MCU startup and refreshed inside
/// `stream_from_sd`. If a CD_READ matches this exactly we skip the
/// SD read entirely, giving the host's DMA controller data to pull
/// the moment it issues DMA_STATUS=1 — which is what the BIOS does
/// right after the CDB. With SD-read latency in the path the BIOS's
/// DMA channel times out at ~10 KB; cached, it drains all 14 KB.
static mut CD_READ_BUF_LBA: u32 = u32::MAX;
static mut CD_READ_BUF_LEN: usize = 0;

/// Drop the staged-sector cache.
///
/// Called on eject/load: once the host can rewrite the filesystem those bytes
/// came from, a cache keyed on LBA is no longer meaningful.
fn invalidate_cd_read_cache() {
    unsafe {
        CD_READ_BUF_LBA = u32::MAX;
        CD_READ_BUF_LEN = 0;
    }
}

/// Lend CD_READ_BUF to the image loader (it reads the .gdi text through it).
/// Only valid while no CD_READ is in flight, i.e. at boot and from the
/// LOAD path inside `ide_task`; whatever was staged is forgotten.
pub fn loader_scratch() -> &'static mut [u8] {
    invalidate_cd_read_cache();
    unsafe { &mut *core::ptr::addr_of_mut!(CD_READ_BUF) }
}

/// How many sectors of the track holding `lba` fit in CD_READ_BUF per
/// staging round. Mapped raw tracks are read whole-sector (2352 bytes plus
/// up to a block of lead-in) and compacted in place, so fewer fit than the
/// 2048-byte payloads they turn into: 5 instead of 7.
fn stage_sectors_for(gdi: &crate::gdi::Gdi, lba: u32) -> u32 {
    let payload_max = (CD_READ_BUF_BYTES / 2048) as u32;
    match gdi.track_for_lba(lba) {
        Some(t) if t.ext_len != 0 && t.sector_size as usize > 2048 => {
            let sec = t.sector_size as usize;
            let mut n = payload_max as usize;
            while n > 1 && (n * sec + 511 + 511) / 512 * 512 > CD_READ_BUF_BYTES {
                n -= 1;
            }
            n as u32
        }
        _ => payload_max,
    }
}

/// Fast path for `fill_cd_read_buf_from_sd`: multi-block reads straight
/// from the card using the track's extent map, then compact each stored
/// sector's 2048-byte payload down to the front of the buffer. The
/// compaction always moves data towards lower addresses (payload `s` starts
/// at or before its raw source), so it is safe in place. `None` means the
/// track is unmapped, the request does not fit, or the card read failed;
/// the caller then falls back to the filesystem path.
fn fill_cd_read_buf_raw(
    track: &crate::gdi::Track,
    start_lba: u32,
    count: u32,
) -> Option<usize> {
    let sec_size = track.sector_size as usize;
    let byte0 = track.byte_offset_of(start_lba)?;
    let head = (byte0 % 512) as usize;
    let raw_len = head + count as usize * sec_size;
    let nblocks = (raw_len + 511) / 512;
    if nblocks * 512 > CD_READ_BUF_BYTES {
        return None;
    }
    let buf = unsafe { &mut *core::ptr::addr_of_mut!(CD_READ_BUF) };
    let first = byte0 - head as u32;
    let mut done = 0usize;
    while done < nblocks {
        let (block, _, contig) = track.locate_byte(first + (done as u32) * 512)?;
        let n = (nblocks - done)
            .min(contig as usize)
            .min(embassy_atsam3::hsmci::MAX_BLOCKS_PER_READ);
        let dst = &mut buf[done * 512..(done + n) * 512];
        if let Err(e) = unsafe { embassy_atsam3::hsmci::read_blocks_dma(block, dst) } {
            warn!("cd_read raw: SD read of {} blocks at {} failed: {:?}", n, block, e);
            return None;
        }
        done += n;
    }
    let uoff = track.user_data_offset();
    for s in 0..count as usize {
        let src = head + s * sec_size + uoff;
        buf.copy_within(src..src + 2048, s * 2048);
    }
    Some(count as usize * 2048)
}

/// Read `count` sectors starting at `start_lba` into CD_READ_BUF.
/// On success updates CD_READ_BUF_LBA / CD_READ_BUF_LEN. Used both
/// from `stream_from_sd` (cache miss path) and from the boot-time
/// `prefetch_ip_bin` so the BIOS's first CD_READ is a cache hit.
async fn fill_cd_read_buf_from_sd(
    gdi: &'static crate::gdi::Gdi,
    start_lba: u32,
    count: u32,
) -> Result<usize, ()> {
    let track = gdi.track_for_lba(start_lba).ok_or(())?;
    // The buffer is about to be overwritten; a failed read must not leave
    // the old range advertised over new bytes.
    invalidate_cd_read_cache();
    if let Some(len) = fill_cd_read_buf_raw(track, start_lba, count) {
        unsafe {
            CD_READ_BUF_LBA = start_lba;
            CD_READ_BUF_LEN = len;
        }
        return Ok(len);
    }
    let sec_size = track.sector_size as usize;
    // Include the track's base offset within its file. GDI gives every track
    // its own file so that base is 0, but CDI concatenates all tracks into
    // one, and without this every CDI read past track 1 lands in the wrong
    // track's data.
    let track_offset =
        (start_lba - track.start_lba) * sec_size as u32 + track.file_offset;
    let track_filename = track.filename.clone();

    let block_dev = crate::blockdev::HsmciBlockDevice::new();
    block_dev.set_num_blocks(gdi.num_blocks);
    let mgr = embedded_sdmmc::VolumeManager::new(block_dev, crate::blockdev::DummyClock);
    let buf = unsafe { &mut CD_READ_BUF[..(count as usize * 2048)] };

    let volume = mgr.open_volume(embedded_sdmmc::VolumeIdx(0)).map_err(|_| ())?;
    let mut dir = volume.open_root_dir().map_err(|_| ())?;
    for part in gdi.image_dir.iter() {
        dir.change_dir(part.clone()).map_err(|_| ())?;
    }
    let file = dir
        .open_file_in_dir(&track_filename, embedded_sdmmc::Mode::ReadOnly)
        .map_err(|_| ())?;
    file.seek_from_start(track_offset).map_err(|_| ())?;

    let mut sector_buf = [0u8; 2352];
    for s in 0..count {
        let sb = &mut sector_buf[..sec_size];
        let mut tot = 0usize;
        while tot < sb.len() {
            let n = file.read(&mut sb[tot..]).map_err(|_| ())?;
            if n == 0 { break; }
            tot += n;
        }
        // Where the 2048-byte user data sits inside a stored sector. 2352 is
        // raw (12 sync + 4 header); 2336 is the CDI data form, which drops
        // sync and header but keeps the 8-byte Mode 2 subheader -- taking the
        // first 2048 bytes there would hand the host the subheader and shift
        // every sector by 8. 2048 is already cooked.
        let payload: &[u8] = match sec_size {
            2352 => &sector_buf[16..16 + 2048],
            2336 => &sector_buf[8..8 + 2048],
            _ => &sector_buf[..2048.min(sec_size)],
        };
        let dst = (s as usize) * 2048;
        buf[dst..dst + payload.len()].copy_from_slice(payload);
    }
    let len = (count as usize) * 2048;
    unsafe {
        CD_READ_BUF_LBA = start_lba;
        CD_READ_BUF_LEN = len;
    }
    Ok(len)
}

/// FNV-1a 32. Used to fingerprint exactly the bytes we hand to the FIFO so a
/// DC-side checksum over what actually landed in host RAM can be compared
/// against it -- byte counts alone cannot distinguish "lost the tail" from
/// "reported the level wrong".
/// Overwrite the staged chunk with a 16-bit word counter keyed to the word's
/// position in the *whole* transfer, so the pattern is continuous across chunk
/// refills. `byte_off` is where this chunk starts within the request.
#[cfg(feature = "dma-pattern")]
fn fill_pattern(byte_off: usize, len: usize) {
    let buf = unsafe { &mut CD_READ_BUF[..len] };
    for i in (0..len).step_by(2) {
        let w = (((byte_off + i) / 2) & 0xFFFF) as u16;
        buf[i] = (w & 0xFF) as u8;
        buf[i + 1] = (w >> 8) as u8;
    }
}

const FNV_INIT: u32 = 2166136261;
fn fnv1a(mut h: u32, data: &[u8]) -> u32 {
    for &b in data {
        h ^= b as u32;
        h = h.wrapping_mul(16777619);
    }
    h
}

/// Put `count` sectors starting at `lba` into CD_READ_BUF, skipping the SD
/// read when the prefetch cache already holds exactly that range. Returns
/// false if the read failed.
///
/// The cache only ever matches the first chunk of a request (it is seeded by
/// `prefetch_ip_bin`), which is the one that matters: it is what lets the
/// BIOS's first CD_READ start pushing with no SD latency in front of it.
async fn stage_chunk(gdi: &'static crate::gdi::Gdi, lba: u32, count: u32) -> bool {
    let want = (count as usize) * 2048;
    let hit = unsafe { CD_READ_BUF_LBA == lba && CD_READ_BUF_LEN >= want };
    if hit {
        info!("  cache hit: LBA={} ({} sectors)", lba, count);
        return true;
    }
    fill_cd_read_buf_from_sd(gdi, lba, count).await.is_ok()
}

/// Pre-fill CD_READ_BUF with the first 7 sectors of HDA at MCU boot.
/// The BIOS's very first CD_READ on game-launch is a DMA-mode read of
/// IP.BIN (LBA = HDA start, 7 sectors). With this cache pre-loaded
/// the SD-read latency disappears for that read and the host's DMA
/// channel can drain all 14 KB before timing out.
/// Complete a no-data command successfully.
fn finish_ok(dev: &IdeDevice) {
    dev.set_irr(0x03);
    dev.set_error(0);
    dev.set_status_intrq(STATUS_IDLE);
}

/// Complete a no-data command with CHECK CONDITION / ILLEGAL REQUEST.
fn finish_illegal(dev: &IdeDevice, state: &mut DeviceState) {
    state.pending_sense_key = 0x05;
    dev.set_irr(0x03);
    dev.set_error(ERR_ABRT | 0x50); // sense key 5 << 4
    dev.set_status_intrq(STATUS_IDLE_ERR);
}

/// Send `data`, cut to the CDB's allocation length (packet[3..=4], 0 = all).
async fn send_alloc(dev: &IdeDevice, packet: &[u8; 12], data: &[u8]) {
    let alloc = ((packet[3] as usize) << 8) | packet[4] as usize;
    let n = if alloc == 0 { data.len() } else { alloc.min(data.len()) };
    send_data_in(dev, &data[..n]).await;
}

/// OpenGDEMU's native 0xE0 -- see `OGDM_IMAGE_CTL` for the layouts.
async fn handle_image_ctl(
    dev: &IdeDevice,
    state: &mut DeviceState,
    label: &'static str,
    packet: &[u8; 12],
) {
    match packet[1] {
        // 0 (LIST) and 4 (INFO) are gone. Enumerating the card is the
        // menu's job now -- it can read the filesystem itself, and doing it
        // here cost this part a sixteen-entry catalog in RAM plus a
        // by-value vector on a 2 KB stack. 1 (SELECT by index) went with
        // them: an index only means something against a list the firmware
        // keeps.
        SUB_MOUNT_NAME => {
            // A chunk of the folder name to mount. The CDB is twelve bytes
            // and a folder name is not, so the name arrives nine bytes at a
            // time; chunk zero starts a new name. Chunking through the
            // existing command path is simpler than giving this one command
            // a data-out phase of its own.
            let chunk = packet[2] as usize;
            let name = unsafe { &mut *core::ptr::addr_of_mut!(MOUNT_NAME) };
            if chunk == 0 {
                name.clear();
            }
            for b in &packet[3..12] {
                if *b == 0 {
                    break;
                }
                if name.push(*b as char).is_err() {
                    break;
                }
            }
            finish_ok(dev);
        }
        SUB_MOUNT => {
            // Mount whatever name was staged, or the configured default.
            let name = unsafe { &*core::ptr::addr_of!(MOUNT_NAME) };
            if packet[2] == NO_SELECTION || name.is_empty() {
                crate::gdi::select_default();
                swap_media(state, label).await;
                finish_ok(dev);
                return;
            }
            crate::cdda::stop();
            MEDIA_PRESENT.store(false, core::sync::atomic::Ordering::Relaxed);
            invalidate_cd_read_cache();
            let ok = match crate::gdi::load_by_name(disk_sectors(), name.as_str()) {
                Ok(g) => {
                    mount_gdi(g);
                    if let Some(g) = gdi_ref() {
                        prefetch_ip_bin(g).await;
                    }
                    true
                }
                Err(e) => {
                    warn!("{}: mount '{}' failed: {:?}", label, name.as_str(), e);
                    unsafe { GDI = None };
                    false
                }
            };
            MEDIA_PRESENT.store(ok, core::sync::atomic::Ordering::Relaxed);
            state.unit_attention = true;
            state.pending_sense_key = 0x06;
            info!("{}: mounted '{}' ({})", label, name.as_str(), ok);
            if ok {
                finish_ok(dev);
            } else {
                finish_illegal(dev, state);
            }
        }
        SUB_VERSION => {
            let mut out = [0u8; 32];
            out[..4].copy_from_slice(b"OGDM");
            out[4] = OGDM_PROTOCOL_VERSION;
            // Bytes 5..8 are reserved, and zero. They used to carry an
            // image limit, a count and a selected index -- all three
            // describing a catalog the firmware kept. It does not keep one:
            // it mounts by name, opening the one directory it needs, so
            // there is no limit, no count and no index. The host is the side
            // that can read the filesystem; what is on the card is its
            // question to answer. The bytes stay in the layout rather than
            // shortening the reply, so the offset of the version string does
            // not move.
            let fw = env!("CARGO_PKG_VERSION").as_bytes();
            let n = fw.len().min(24);
            out[8..8 + n].copy_from_slice(&fw[..n]);
            send_alloc(dev, packet, &out).await;
        }
        sub => {
            warn!("{}: IMAGE_CTL sub-command {} not supported", label, sub);
            finish_illegal(dev, state);
        }
    }
}

/// Sub-commands of [`OGDM_IMAGE_CTL`].
const SUB_MOUNT: u8 = 2;
const SUB_VERSION: u8 = 3;
const SUB_MOUNT_NAME: u8 = 5;

/// The folder name staged by [`SUB_MOUNT_NAME`], mounted by [`SUB_MOUNT`].
static mut MOUNT_NAME: heapless::String<{ gamecfg::MAX_NEXT }> = heapless::String::new();



pub async fn prefetch_ip_bin(gdi: &'static crate::gdi::Gdi) {
    let lba = gdi.boot_area_lba;
    // The BIOS asks for 7 sectors; cache whatever one staging round holds
    // so its first chunk needs no card access at all.
    let count: u32 = 7.min(stage_sectors_for(gdi, lba));
    info!("prefetching IP.BIN: LBA={} count={}", lba, count);
    match fill_cd_read_buf_from_sd(gdi, lba, count).await {
        Ok(n) => info!("  IP.BIN cached: {} bytes", n),
        Err(()) => warn!("prefetch_ip_bin: SD read failed"),
    }
}

async fn stream_from_sd(
    dev: &IdeDevice,
    gdi: &'static crate::gdi::Gdi,
    start_lba: u32,
    count: u32,
    total_bytes: usize,
    dma: bool,
    byte_limit: u16,
) {
    // CD_READ_BUF is a staging window, not a size limit. Requests larger
    // than it are streamed: stage a chunk, push it, refill, repeat. The
    // host tolerates the refill gaps because negating DMARQ is flow
    // control, not an error -- ATA/ATAPI-5 10.2.3 has the host pause and
    // wait indefinitely for the device to reassert it, and the FPGA
    // reasserts automatically once the FIFO refills (ide_dmarq is
    // combinational on read_fifo.r_rdy). Confirmed on hardware: the DC sat
    // through a 2 s stall without giving up.
    let stage_sectors = stage_sectors_for(gdi, start_lba);
    if count > stage_sectors {
        info!(
            "  CD_READ streaming: {} sectors in chunks of {}",
            count, stage_sectors
        );
    }
    if !dma {
        return stream_pio_from_sd(dev, gdi, start_lba, count, total_bytes, byte_limit).await;
    }

    // Stage the first chunk before arming DMA so the FIFO is primed and
    // DMARQ is already high when the host's channel starts.
    let first = count.min(stage_sectors);
    if !stage_chunk(gdi, start_lba, first).await {
        warn!("stream_from_sd: SD read failed");
        dev.set_irr(0x03);
        dev.set_error(ERR_ABRT);
        dev.set_status_intrq(STATUS_IDLE_ERR);
        return;
    }

    #[cfg(feature = "dma-pattern")]
    fill_pattern(0, (first as usize) * 2048);

    let staged_bytes = (first as usize) * 2048;
    let buf = unsafe { &mut CD_READ_BUF[..staged_bytes] };
    let mut hash = fnv1a(FNV_INIT, &buf[..]);
    info!("  staged {} of {} bytes; arming DMA", staged_bytes, total_bytes);
    if total_bytes >= 16 {
        info!(
            "  hdr [0..16]: {:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}",
            buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7],
            buf[8], buf[9], buf[10], buf[11], buf[12], buf[13], buf[14], buf[15]
        );
    }

    // Pre-prime the FIFO before notifying the host. Without this the
    // host's DMA controller starts on an empty FIFO, only sees DMARQ
    // assert as we trickle words in, and drops the channel partway
    // through the burst when a momentary push/pull mismatch underruns
    // the FIFO. Filling the FIFO first means DMARQ is already high
    // when the host's DMA channel arms, and there's a 4 KB cushion
    // before any underrun risk.
    let push_reg_pre = unsafe { dev.base.add(REG_DATA_FIFO) };
    let prime_words = (dev.fifo_depth_words() as usize).saturating_sub(8);
    let prime_bytes = (prime_words * 2).min(buf.len() & !1);
    let mut primed = 0usize;
    while primed + 1 < prime_bytes {
        let val = (buf[primed] as u16) | ((buf[primed + 1] as u16) << 8);
        unsafe { write_volatile(push_reg_pre, val) };
        primed += 2;
    }

    // Verify FIFO actually received data (sanity check for debugging DMA issues)
    let level_after_prime = dev.read_fifo_level();
    info!(
        "  FIFO primed: {} bytes -> level={}",
        primed,
        level_after_prime
    );
    if level_after_prime == 0 && primed > 0 {
        warn!("FIFO level is 0 after pushing {} bytes - SMC/FPGA issue?", primed);
    }

    // Phase 2: set up the data-in envelope and arm DMA, then notify
    // the host (INTRQ). The host reads STATUS, sees the data-phase
    // setup, and its DMA controller starts pulling on DMARQ.
    // The ATAPI byte-count limit must be even -- it counts bytes on a 16-bit
    // bus, and an odd value is not representable as whole words. Clamping a
    // large request with min(0xFFFF) produced 65535, and the host then never
    // started its DMA at all: a 572-sector (1.1 MB) read sat with our FIFO
    // full and zero bytes pulled, while a 16-sector read (byte count 32768,
    // even) worked. Clamp to 0xFFFE and force even.
    dev.set_atapi_byte_count((total_bytes.min(0xFFFE) & !1) as u16);
    dev.set_irr(0x02);
    dev.set_error(0);
    if dma {
        // Hold BSY for the whole DMA data phase and do not interrupt at its
        // start. The BIOS polls Status while its G1 DMA runs, and a status
        // of 0x50 (BSY=0, DRQ=0) reads as "command finished": it aborts the
        // DMA mid-burst and moves on. Transfers that fit one FIFO prime
        // drained before the next poll and looked fine; anything needing an
        // SD refill mid-transfer (a cold 9-sector IP.BIN read, the game's
        // 572-sector 1ST_READ.BIN) was cut at whatever word the poll landed
        // on -- 5308 and 4678 words, not burst multiples. DRQ=1 is not the
        // answer either: the BIOS then treats the phase as PIO and refuses.
        // iceGDROM does exactly this (packet_data_dma leaves BSY set from
        // the CDB and only finish_packet writes 0x50 + INTRQ).
        // Do NOT interrupt at the start of the DMA data phase. Tried and
        // measured: interrupting here while keeping BSY set (so the host can
        // never read 0x50 and mistake it for completion) does not help and
        // makes things much worse -- the host pulled 44 bytes of an 8228-byte
        // prime, push_buf_to_fifo hit its 10 s timeout twice, and the console
        // never got past IP.BIN's first chunk and fell back to the CD audio
        // player. The interrupt itself is the hazard here, not the cleared
        // BSY, so the "don't interrupt, hold BSY, let the host poll" rule
        // below is the real constraint.
        dev.set_status(STATUS_BSY | STATUS_DRDY | STATUS_DSC);
        dev.set_dma_mode(DmaMode::ToHost);
    } else {
        dev.set_status_intrq(STATUS_DRDY | STATUS_DSC | STATUS_DRQ);
    }

    // Push the balance of the primed chunk, then stage and push the rest of
    // the request a chunk at a time. push_buf_to_fifo blocks on FIFO
    // fullness, so this naturally paces to whatever rate the host drains at.
    let first_rest = buf.len() - primed;
    let mut pushed_bytes = primed + push_buf_to_fifo(dev, &buf[primed..]).await;
    let mut pushed_all = pushed_bytes == primed + first_rest;
    let mut sectors_done = first;
    while pushed_all && sectors_done < count {
        let this = (count - sectors_done).min(stage_sectors);
        if !stage_chunk(gdi, start_lba + sectors_done, this).await {
            warn!("stream_from_sd: SD read failed mid-stream at sector {}", sectors_done);
            break;
        }
        #[cfg(feature = "dma-pattern")]
        fill_pattern((sectors_done as usize) * 2048, (this as usize) * 2048);

        let chunk = unsafe { &CD_READ_BUF[..(this as usize) * 2048] };
        hash = fnv1a(hash, chunk);
        let n = push_buf_to_fifo(dev, chunk).await;
        pushed_bytes += n;
        pushed_all = n == chunk.len();
        sectors_done += this;
    }
    if !pushed_all {
        info!("  host stopped draining mid-stream (lvl={})", dev.read_fifo_level());
    }
    if sectors_done < count {
        warn!("  short transfer: {} of {} sectors staged", sectors_done, count);
    }

    let level_before_drain = dev.read_fifo_level();
    info!("  staged; awaiting drain (lvl={})", level_before_drain);

    // Wait for the host to finish draining. Use coarse 1 ms granularity
    // and cap at 200 ms — if the host hasn't drained by then it's not
    // going to, and we want STATUS to transition out of DRQ promptly so
    // the BIOS can move on instead of hanging at "Please wait".
    //
    // For DMA mode, we must wait at least a minimum time (100ms) even if
    // the FIFO appears empty, to give the DC time to trigger its G1 DMA
    // controller. Without this, we may complete the command before the
    // DC's DMA has even started. The DC needs to:
    //   1. See INTRQ / poll for IRR=0x02
    //   2. Configure G1 DMA registers
    //   3. Trigger DMA (write to DMA_STATUS)
    // All of this takes time on the DC side. Only start checking FIFO level
    // after the minimum wait has elapsed.
    //
    // The floor is skipped when the host has demonstrably finished. An empty
    // FIFO is ambiguous on its own -- it can mean "host hasn't started" --
    // but an empty FIFO *after* we primed it and pushed every byte is not:
    // the host took them. Waiting out the floor there holds BSY for 100 ms
    // after the transfer is over, and the BIOS soft-resets the drive rather
    // than wait for the completion. That is what killed the boot: the read
    // of 1ST_READ.BIN drained fully in 49 ms and then hit
    // `host reset during CD_READ; no completion`, three times, before the
    // console gave up and returned to its menu.
    let host_took_everything = pushed_all && primed > 0 && dev.read_fifo_level() == 0;
    let min_wait_ms = if dma && !host_took_everything { 100u32 } else { 0u32 };
    // Progress is logged whenever the level actually moves. Measured: the
    // host does NOT resume after a stall -- raising this cap to 2000 ms
    // changed nothing, the level stayed frozen for the full two seconds. So a
    // stall here means the host abandoned the DMA, not that it paused.
    let mut last_lvl = u16::MAX;
    let mut stalled_ms = 0u32;
    for i in 0..200u32 {
        let lvl = dev.read_fifo_level();
        if i >= min_wait_ms && lvl == 0 { break; }
        if lvl != last_lvl {
            if stalled_ms >= 20 {
                info!("  drain: resumed after {} ms stall (lvl={})", stalled_ms, lvl);
            }
            last_lvl = lvl;
            stalled_ms = 0;
        } else {
            stalled_ms += 1;
        }
        Timer::after(Duration::from_millis(1)).await;
    }
    if dev.reset_seen() {
        info!("  host reset during CD_READ; no completion");
        return;
    }
    let leftover = dev.read_fifo_level();
    // Count against what we actually managed to stage, not the request: on a
    // mid-stream SD failure `sectors_done` stops short of `count`, and
    // charging the shortfall against total_bytes would overstate delivery.
    // Count against bytes that actually reached the FIFO. Using the staged
    // total overstates delivery whenever push_buf_to_fifo bailed out early --
    // that is what made a transfer the host never touched report
    // "delivered=6152" instead of 0.
    let staged_total = pushed_bytes;
    let delivered = staged_total.saturating_sub((leftover as usize) * 2);
    info!(
        "  drain done (lvl={}, delivered={}/{} of {} requested) fnv1a={:08x}",
        leftover, delivered, staged_total, total_bytes, hash
    );
    // Disarm DMA mode now that the host has drained as much as it
    // wanted. If we leave it armed and the FIFO has stray data left,
    // DMARQ would re-assert and confuse the host.
    if dma {
        dev.set_dma_mode(DmaMode::Off);
    }
    // ATAPI short-transfer signal: byte_count register at command
    // completion reflects the actual bytes delivered to the host (vs
    // the full request we set up at data-phase start). Without this
    // the BIOS keeps thinking the transfer is still in flight.
    dev.set_atapi_byte_count((delivered.min(0xFFFE) & !1) as u16);
    dev.set_irr(0x03);
    dev.complete(0, STATUS_IDLE);
    info!("CD_READ complete ({} bytes)", delivered);
}

/// PIO CD_READ: the payload goes to the host in ATAPI DRQ blocks.
///
/// Each block is announced by its own byte count + DRQ + INTRQ, the host
/// reads exactly that many words, and the device shows BSY until the next
/// block (or the completion status) is ready. Presenting the whole request
/// as one DRQ block with byte count 0xFFFE did not work: Sega Rally 2's
/// driver read 32767 words per DRQ block it saw, straight through our
/// refill gaps (there is no IORDY pacing in PIO mode), and got junk.
/// A block never exceeds the host's byte-count limit from the command
/// (0xF000 for that driver) nor what is staged in CD_READ_BUF, so a block is
/// at most 5-7 sectors (see `stage_sectors_for`). The FPGA's read flow control raises BSY the instant
/// the host pops a block's last word, so the host cannot mistake the gap
/// before we re-arm DRQ for command completion.
async fn stream_pio_from_sd(
    dev: &IdeDevice,
    gdi: &'static crate::gdi::Gdi,
    start_lba: u32,
    count: u32,
    total_bytes: usize,
    byte_limit: u16,
) {
    let stage_sectors = stage_sectors_for(gdi, start_lba);
    // ATAPI: a limit of 0 (or 0xFFFF) means the host will take anything.
    let limit = match (byte_limit & !1) as usize {
        0 => 0xFFFE,
        l => l,
    };
    let prime_max = (dev.fifo_depth_words() as usize).saturating_sub(8) * 2;

    let t0 = Instant::now();
    let mut hash = FNV_INIT;
    let mut sent = 0usize;
    let mut blocks = 0u32;
    let mut chunk_start = 0u32;
    let mut chunk_sectors = 0u32;
    let mut ok = true;
    let mut reset = false;

    dev.set_irr(0x02);
    dev.set_error(0);
    dev.set_pio_flow_control(true, false);
    while sent < total_bytes {
        // Stage the chunk holding the next block. Blocks are cut at chunk
        // boundaries, so a block always lies inside the staged window.
        let sector = (sent / 2048) as u32;
        if chunk_sectors == 0 || sector >= chunk_start + chunk_sectors {
            chunk_start = sector;
            chunk_sectors = (count - sector).min(stage_sectors);
            if !stage_chunk(gdi, start_lba + sector, chunk_sectors).await {
                warn!("stream_pio_from_sd: SD read failed at sector {}", sector);
                ok = false;
                break;
            }
            let chunk = unsafe { &CD_READ_BUF[..(chunk_sectors as usize) * 2048] };
            hash = fnv1a(hash, chunk);
        }
        let off = sent - (chunk_start as usize) * 2048;
        let block = (total_bytes - sent)
            .min(limit)
            .min((chunk_sectors as usize) * 2048 - off)
            & !1;
        let data = unsafe { &CD_READ_BUF[off..off + block] };

        // Announce the block only once the FIFO holds as much of it as fits;
        // the balance streams behind the host's reads. push_buf_to_fifo
        // would otherwise wait on room the host has no reason to make yet.
        dev.set_status(STATUS_BSY | STATUS_DRDY | STATUS_DSC);
        dev.set_atapi_byte_count(block as u16);
        let prime = block.min(prime_max);
        if push_buf_to_fifo(dev, &data[..prime]).await != prime {
            ok = false;
            break;
        }
        dev.set_status_intrq(STATUS_DRDY | STATUS_DSC | STATUS_DRQ);
        if push_buf_to_fifo(dev, &data[prime..]).await != block - prime {
            warn!("  host stopped draining block {} (lvl={})", blocks, dev.read_fifo_level());
            ok = false;
            break;
        }
        // The host owes us exactly `block` bytes of reads; the FPGA shows BSY
        // from the last pop until we re-arm DRQ above or complete below.
        match wait_host(dev, || dev.read_fifo_level() == 0, 10_000).await {
            Ok(()) => {}
            Err(Stall::Reset) => { reset = true; break; }
            Err(why) => {
                if why == Stall::Timeout {
                    warn!("  host left block {} undrained (lvl={}, {} of {} bytes sent)",
                          blocks, dev.read_fifo_level(), sent, total_bytes);
                    dev.log_host_activity("pio block stall");
                }
                ok = false;
                break;
            }
        }
        sent += block;
        blocks += 1;
    }
    if reset || dev.reset_seen() {
        info!("  host reset during CD_READ; no completion");
        dev.set_pio_flow_control(false, false);
        return;
    }
    let leftover = dev.read_fifo_level() as usize * 2;
    let delivered = sent.saturating_sub(leftover);
    if dev.command_pending() {
        // The host moved on (its Command write is what emptied the FIFO).
        // Nothing may be written to Status now -- see `complete`.
        info!("  host abandoned CD_READ with a new command after {} of {} bytes; no completion",
              delivered, total_bytes);
        dev.set_pio_flow_control(false, false);
        return;
    }
    // Hold BSY (and drop DRQ) *before* releasing flow control: the FPGA's
    // synthetic BSY only lasts while pio_rd_fc is set, so releasing first
    // would show 0x58 with an empty FIFO until the completion write below.
    dev.set_status(STATUS_BSY | STATUS_DRDY | STATUS_DSC);
    dev.set_pio_flow_control(false, false);
    info!(
        "  PIO done: {} blocks, delivered={}/{} in {} ms fnv1a={:08x}",
        blocks, delivered, total_bytes, (Instant::now() - t0).as_millis(), hash
    );
    dev.set_atapi_byte_count((delivered.min(0xFFFE) & !1) as u16);
    dev.set_irr(0x03);
    if ok {
        dev.complete(0, STATUS_IDLE);
    } else {
        dev.complete(ERR_ABRT, STATUS_IDLE_ERR);
    }
    info!("CD_READ complete ({} bytes)", delivered);
}


/// Push `data` into the read FIFO via SAM3U DMAC, throttling on FIFO
/// fullness. Returns `true` on success; `false` if the host stopped
/// draining (FIFO full for >1 s) so the caller can wrap up the
/// command early instead of stalling forever waiting on DRQ.
///
/// DMAC pushes ~16+ MB/s steady-state, matching the DC's host DMA
/// pull rate — without it the CPU `write_volatile` loop jittered with
/// embassy task scheduling and the first CD_READ DMA underran
/// inconsistently (8/13/14 KB delivered). With DMAC the FIFO stays
/// near-full and DMARQ never blips low mid-transfer.
async fn push_buf_to_fifo(dev: &IdeDevice, data: &[u8]) -> usize {
    let mut i = 0;
    let push_reg = unsafe { dev.base.add(REG_DATA_FIFO) };
    // Leave 4 words of headroom — DMAC writes immediately on arm,
    // we don't want to race the FPGA's r_rdy edge.
    let room = || dev.fifo_depth_words().saturating_sub(dev.read_fifo_level() + 4) as usize;
    while i + 1 < data.len() {
        // A full FIFO drains in ~130 us at host DMA rate; sleeping here
        // cost a whole 1 ms tick per refill and was most of a CD_READ.
        // 5 s, not 10: the host's own patience is far shorter than either.
        // The longest legitimate stall measured is the console descrambling a
        // MIL-CD's 1ST_READ.BIN as it pulls, at 2.16 s -- and when the host
        // has genuinely gone the extra five seconds buy nothing but a job
        // that takes longer to fail.
        if let Err(why) = wait_host(dev, || room() > 0, 5_000).await {
            if why == Stall::Timeout {
                warn!("push_buf_to_fifo: FIFO never drained after {} bytes", i);
            }
            return i;
        }
        let room_words = room();
        let words_left = (data.len() - i) / 2;
        let n = room_words.min(words_left);
        // Source pointer needs to be halfword-aligned and stable for
        // the duration of the DMAC burst. CD_READ_BUF is byte-aligned
        // but `i` is always advanced by 2, so the running pointer is
        // halfword-aligned. Each transfer reads `n` halfwords from the
        // RAM buffer to the SMC FIFO write register at a fixed address.
        let src_ptr = unsafe { data.as_ptr().add(i) } as *const u16;
        unsafe {
            embassy_atsam3::dmac::push_halfwords(src_ptr, push_reg, n as u32);
        }
        // Wait for the DMAC burst to complete before the next FIFO-
        // level read; otherwise we'd see stale level and overshoot.
        // Future streaming refactor will overlap DMAC with the next
        // SD read instead of sleeping here.
        embassy_atsam3::dmac::wait_done();
        i += n * 2;
    }
    // Trailing odd byte (rare; CD sectors are always even).
    if i < data.len() {
        let val = data[i] as u16;
        unsafe { write_volatile(push_reg, val) };
    }
    i
}

async fn stream_pattern(dev: &IdeDevice, byte_count: usize) {
    // Push a sliding incrementing pattern, refilling the FIFO whenever
    // it has room. The FIFO is 1024 words = 2 KB, so for a single 2 KB
    // sector we fit it all up-front.
    dev.set_atapi_byte_count(byte_count.min(0xFFFF) as u16);
    dev.set_error(0);
    dev.set_status_intrq(STATUS_DRDY | STATUS_DSC | STATUS_DRQ);

    let mut byte_idx: usize = 0;
    let mut waited = 0u32;
    while byte_idx < byte_count {
        // FIFO has room? Push another word.
        if dev.read_fifo_level() < (dev.fifo_depth_words() - 4) {
            let lo = (byte_idx as u8) as u16;
            let hi = ((byte_idx + 1) as u8) as u16;
            dev.push_read_fifo(lo | (hi << 8));
            byte_idx += 2;
            waited = 0;
        } else {
            if waited >= 10_000 {
                warn!("stream_pattern: FIFO never drained, abandoning");
                break;
            }
            Timer::after(Duration::from_micros(100)).await;
            waited += 1;
        }
    }

    // Wait for the host to finish draining what we staged.
    let mut empty_waited = 0u32;
    while dev.read_fifo_level() > 0 {
        if empty_waited >= 10_000 {
            warn!("stream_pattern: FIFO drain timeout at end");
            break;
        }
        Timer::after(Duration::from_micros(100)).await;
        empty_waited += 1;
    }
    dev.set_status_intrq(STATUS_IDLE);
}

/// Words deep in the FPGA read FIFO (MCU→IDE; matches READ_FIFO_DEPTH
/// in ide_device.py). Deeper is better for DMA: the host can pull at
/// ~60x the rate the MCU can push, so a deeper FIFO rides out the
/// gap between push iterations without dropping DMARQ.
/// Mirrors READ_FIFO_DEPTH / SECONDARY_READ_FIFO_DEPTH in
/// fpga/amaranth/hw/ide_device.py. Keep them in step.
const PRIMARY_FIFO_DEPTH_WORDS: u16 = 4096;
const SECONDARY_FIFO_DEPTH_WORDS: u16 = 1024;

async fn recv_packet(dev: &IdeDevice) -> Result<[u8; PACKET_LEN_BYTES], ()> {
    // Wait for the host to write all 6 words. Yield-only: a sleep here is
    // a 1 ms tick (see `poll_until`), which was most of a PACKET command's
    // latency. A real drive has no CDB timeout; Sega Rally 2's WinCE
    // driver polls at ~50 ms and took >100 ms to get round to the CDB of
    // its first PIO read, which the old 100 ms cap turned into ABRT. Only
    // a reset or a command to the other device abandons the wait early.
    // Timestamp the host's first few Status/Alt Status reads (1 ms
    // resolution) so a host that stops before the CDB tells us whether it
    // polled for DRQ and gave up, or peeked once and went to sleep.
    let t0 = Instant::now();
    let mut last_st = dev.read(REG_HOST_STATUS_RDS);
    let mut marks: [(u32, u16); 8] = [(u32::MAX, 0); 8];
    let mut n_marks = 0usize;
    let r = wait_host(dev, || {
        let st = dev.read(REG_HOST_STATUS_RDS);
        if st != last_st {
            last_st = st;
            if n_marks < marks.len() {
                marks[n_marks] = ((Instant::now() - t0).as_millis() as u32, st);
                n_marks += 1;
            }
        }
        dev.write_fifo_level() >= PACKET_LEN_WORDS as u16
    }, 10_000).await;
    if r.is_err() {
        warn!("recv_packet: host reads (ms, st|alt<<8) before giving up: {:?}", &marks[..n_marks]);
        return Err(());
    }
    let mut bytes = [0u8; PACKET_LEN_BYTES];
    for i in 0..PACKET_LEN_WORDS {
        let w = dev.pop_write_fifo();
        bytes[2 * i] = (w & 0xFF) as u8;
        bytes[2 * i + 1] = (w >> 8) as u8;
    }
    Ok(bytes)
}

/// Drain `len` bytes of an ATAPI data-out phase from the host. Sets
/// up DRQ + IRR=0 (host-to-drive direction), pulses INTRQ, then waits
/// for the FPGA's write FIFO to receive the expected words. Returns
/// the bytes (or empty on timeout). Caller is responsible for the
/// command-complete (IRR=0x03 + STATUS_IDLE) status phase afterward.
async fn recv_data_out(dev: &IdeDevice, len: usize) -> heapless::Vec<u8, 64> {
    let mut out = heapless::Vec::<u8, 64>::new();
    let words = (len + 1) / 2;
    dev.set_atapi_byte_count(len as u16);
    // Interrupt Reason: data-out phase (I/O=0, C/D=0).
    dev.set_irr(0x00);
    dev.set_error(0);
    dev.set_status_intrq(STATUS_DRDY | STATUS_DSC | STATUS_DRQ);

    if let Err(why) = wait_host(dev, || dev.write_fifo_level() >= words as u16, 10_000).await {
        if why == Stall::Timeout {
            warn!("recv_data_out: host never wrote {} words", words);
        }
        return out;
    }
    for _ in 0..words {
        let w = dev.pop_write_fifo();
        let _ = out.push((w & 0xFF) as u8);
        if out.len() < len {
            let _ = out.push((w >> 8) as u8);
        }
    }
    out
}

async fn send_data_in(dev: &IdeDevice, data: &[u8]) {
    // Stage the response in the read FIFO. Pad to even byte count by
    // emitting a zero high byte if the response is odd-length.
    let mut i = 0;
    while i < data.len() {
        let lo = data[i] as u16;
        let hi = if i + 1 < data.len() { data[i + 1] as u16 } else { 0 };
        dev.push_read_fifo(lo | (hi << 8));
        i += 2;
    }
    let staged_level = dev.read_fifo_level();
    // Host reads so far (Status/Alt Status/Data) belong to the CDB phase;
    // anything above these afterwards is the host reacting to the data.
    let pre_st = dev.read(REG_HOST_STATUS_RDS);
    let pre_data = dev.read(REG_HOST_DATA_RDS);
    info!("send_data_in staged {} bytes; read_fifo level={} host reads so far st={} alt={} data={}",
          data.len(), staged_level, pre_st & 0xFF, pre_st >> 8, pre_data);
    dev.set_atapi_byte_count(data.len() as u16);
    // Interrupt Reason: data-in phase (I/O=1, C/D=0).
    dev.set_irr(0x02);
    dev.set_error(0);
        // Real ATAPI fires INTRQ when DRQ goes 0->1 (data ready). The host
    // reads STATUS to clear, then drains the data via the Data reg.
    dev.set_status_intrq(STATUS_DRDY | STATUS_DSC | STATUS_DRQ);

    // Wait for the host to drain the FIFO, then clear DRQ. Cap the wait
    // at ~1 s — if the host never reads, we abandon the transfer rather
    // than hang the IDE task.
    // A new command to either device ends the wait too: the FPGA clears the
    // FIFO on the command write anyway, and a host that took what it wanted
    // and moved on should not cost the next command ten seconds of BSY.
    // Sample the host's reads while waiting so a reply it never drains
    // still tells us when (if ever) it looked at Status afterwards.
    let t0 = Instant::now();
    let mut last_st = pre_st;
    let mut last_change_us: u32 = 0;
    let mut first_status_us: u32 = u32::MAX;
    let r = wait_host(dev, || {
        let st = dev.read(REG_HOST_STATUS_RDS);
        if st != last_st {
            last_st = st;
            last_change_us = (Instant::now() - t0).as_micros() as u32;
            // First Status read after INTRQ: microseconds = an ISR,
            // milliseconds = a per-frame poll.
            if first_status_us == u32::MAX && (st & 0xFF) > (pre_st & 0xFF) {
                first_status_us = last_change_us;
            }
        }
        dev.read_fifo_level() == 0 || dev.command_pending()
    }, 10_000).await;
    let waited_us = (Instant::now() - t0).as_micros() as u32;
    info!("send_data_in: waited {} us; first Status read at +{} us; last change at +{} us (st={} alt={})",
          waited_us, first_status_us, last_change_us, last_st & 0xFF, last_st >> 8);
    match r {
        Err(Stall::Timeout) => {
            warn!("send_data_in: host never drained FIFO, abandoning");
            dev.log_host_activity("send_data_in");
        }
        // The FPGA holds the post-reset state; a completion write here
        // would only leave a stale status and IRQ for the host to find.
        Err(Stall::Reset) => return,
        _ => {}
    }
    // Command complete. IRR = 0x03 (C/D|I/O = 1|1) tells the host the
    // status phase begins and we're done.
    dev.set_irr(0x03);
    dev.complete(0, STATUS_IDLE);
}

/// Read `n` consecutive SD blocks at `block` into `dst` (`n * 512` bytes):
/// CMD18 for runs of at least `multi_min` blocks, CMD17 per block below that.
fn fetch_run(block: u32, n: u32, dst: &mut [u8], multi_min: u32) -> bool {
    if n >= multi_min {
        return unsafe { embassy_atsam3::hsmci::read_blocks_dma(block, dst) }.is_ok();
    }
    for i in 0..n as usize {
        let one: &mut [u8; 512] = (&mut dst[i * 512..(i + 1) * 512]).try_into().unwrap();
        if unsafe { embassy_atsam3::hsmci::read_block(block + i as u32, one) }.is_err() {
            return false;
        }
    }
    true
}

/// ATA READ SECTORS / READ DMA on the emulated disk, LBA28 or LBA48.
///
/// Sectors map straight onto SD card blocks -- the whole card is exposed, so
/// LBA n is card block n and the host sees the same filesystem that holds the
/// GD-ROM images. `ext` picks where the address and count come from; see
/// [`CommandSnapshot::lba`].
async fn handle_disk_read(
    dev: &IdeDevice,
    label: &'static str,
    snap: &CommandSnapshot,
    dma: bool,
    ext: bool,
) {
    if snap.device & 0x40 == 0 {
        // CHS addressing. Nothing we serve is CHS-shaped and every host we
        // care about uses LBA, so refuse rather than silently mis-address.
        warn!("{}: disk read in CHS mode, refusing", label);
        dev.set_error(ERR_ABRT);
        dev.set_status_intrq(STATUS_IDLE_ERR);
        dev.ack_command();
        return;
    }
    // The card is addressed in 32-bit blocks, so a 48-bit LBA above that is
    // unreachable however big the card is. Refuse rather than truncate: a
    // silently wrapped address reads the wrong sector and looks like
    // corruption much later.
    let Ok(lba) = u32::try_from(snap.lba(ext)) else {
        warn!("{}: disk read at LBA {} beyond 32-bit block addressing", label, snap.lba(ext));
        dev.set_error(ERR_ABRT);
        dev.set_status_intrq(STATUS_IDLE_ERR);
        dev.ack_command();
        return;
    };
    let count = snap.count(ext);

    let total = disk_sectors();
    if total == 0 || lba.saturating_add(count) > total {
        warn!("{}: disk read {} sectors at LBA {} past end ({})", label, count, lba, total);
        dev.set_error(ERR_ABRT);
        dev.set_status_intrq(STATUS_IDLE_ERR);
        dev.ack_command();
        return;
    }
    info!("{}: disk read LBA={} count={} dma={}", label, lba, count, dma);

    // Sectors are fetched in runs of up to `chunk` through CD_READ_BUF with
    // CMD18 rather than one CMD17 each: per-block command overhead, not the
    // bus, is what capped a 64-sector read at ~35 ms. The buffer is the
    // GD-ROM's staging cache, but `ide_task` services one device at a time,
    // so borrowing it here is safe; the cost is that the next CD_READ misses
    // and re-reads from SD, which the host never notices since it does not
    // interleave disk and GD-ROM traffic. Short runs stay on CMD17 -- CMD18's
    // CMD12 tail makes it slower than single-block reads below ~8 blocks.
    const CHUNK: u32 = (CD_READ_BUF_BYTES / 512) as u32;
    const MULTI_MIN: u32 = 8;
    invalidate_cd_read_cache();
    let buf = unsafe { &mut CD_READ_BUF[..] };

    // Stage the first block *before* raising DRQ. The host starts reading the
    // moment DRQ goes high, so announcing readiness over an empty FIFO hands
    // it whatever the data path last held -- the first attempt at this
    // returned 256 copies of IDENTIFY word 0. send_data_in has the same
    // ordering for the same reason.
    let mut ok = true;
    let mut run = count.min(CHUNK);
    if !fetch_run(lba, run, &mut buf[..run as usize * 512], MULTI_MIN) {
        warn!("{}: SD read failed at block {}", label, lba);
        dev.set_error(ERR_ABRT);
        dev.set_status_intrq(STATUS_IDLE_ERR);
        dev.ack_command();
        return;
    }
    let mut fetched = run; // sectors read from SD into buf
    for i in (0..512).step_by(2) {
        dev.push_read_fifo((buf[i] as u16) | ((buf[i + 1] as u16) << 8));
    }
    let mut sent = 1u32; // sectors pushed to the FIFO

    dev.set_error(0);
    if dma {
        dev.set_status_intrq(STATUS_DRDY | STATUS_DSC);
        dev.set_dma_mode(DmaMode::ToHost);
    } else {
        dev.set_pio_flow_control(true, false);
        dev.set_status_intrq(STATUS_DRDY | STATUS_DSC | STATUS_DRQ);
    }
    dev.ack_command();

    // Remaining blocks stream behind it, paced by push_buf_to_fifo.
    while sent < count && ok {
        if sent == fetched {
            run = (count - fetched).min(CHUNK);
            if !fetch_run(lba + fetched, run, &mut buf[..run as usize * 512], MULTI_MIN) {
                warn!("{}: SD read failed at block {}", label, lba + fetched);
                ok = false;
                break;
            }
            fetched += run;
        }
        let off = ((sent - (fetched - run)) as usize) * 512;
        if dma {
            // The whole rest of the run in one paced push; DMARQ does the
            // flow control.
            let rest = &buf[off..run as usize * 512];
            if push_buf_to_fifo(dev, rest).await != rest.len() {
                warn!("{}: host stopped draining disk read at sector {}", label, sent);
                ok = false;
                break;
            }
            sent = fetched;
        } else {
            // PIO: wait for the host to take the previous sector, then
            // re-assert DRQ with an interrupt for the next one. The read
            // direction survives without this today because
            // push_buf_to_fifo blocks on room, so the host is paced by
            // accident -- but a host that waits for the per-sector interrupt
            // would hang forever, and ATA says it is entitled to.
            if let Err(why) = wait_host(dev, || dev.read_fifo_level() == 0, 10_000).await {
                if why == Stall::Timeout {
                    warn!("{}: host stopped draining disk read at sector {}", label, sent);
                }
                ok = false;
                break;
            }
            dev.set_status(STATUS_BSY | STATUS_DRDY | STATUS_DSC);
            let sector = &buf[off..off + 512];
            if push_buf_to_fifo(dev, sector).await != sector.len() {
                warn!("{}: host stopped draining disk read at sector {}", label, sent);
                ok = false;
                break;
            }
            dev.set_status_intrq(STATUS_DRDY | STATUS_DSC | STATUS_DRQ);
            sent += 1;
        }
    }

    // Let the host finish taking what is queued before clearing DRQ.
    // Give up the moment the host issues its next command. FatFs reads
    // exactly what it needs and leaves the rest of a sector in the FIFO, so
    // this loop would otherwise spin its full second on a transfer the host
    // has already finished with -- and the *next* command sits unserviced
    // behind it. That is what stalled the first WRITE SECTORS: the host armed
    // it, waited, timed out, and retried, and the retry's BSY was what we
    // then saw as 0xd8. The FPGA clears the FIFO on a command write anyway,
    // so there is nothing to preserve here.
    // A command to either device ends the wait: the data is all delivered,
    // and FatFs in particular reads what it needs and leaves the rest.
    if wait_host(dev, || dev.read_fifo_level() == 0 || dev.command_pending(), 10_000).await
        == Err(Stall::Timeout)
    {
        warn!("{}: host left {} words in the read FIFO (dma={}, bus={:#04x}, strobes={})",
              label, dev.read_fifo_level(), dma, dev.read(17), dev.read(15));
    }
    // BSY before flow control is released, so the host never sees DRQ with
    // an empty FIFO (see stream_pio_from_sd) -- unless the host has already
    // moved on, in which case Status belongs to its new command.
    if !dev.command_pending() {
        dev.set_status(STATUS_BSY | STATUS_DRDY | STATUS_DSC);
    }
    if dma {
        dev.set_dma_mode(DmaMode::Off);
    } else {
        dev.set_pio_flow_control(false, false);
    }
    dev.complete(if ok { 0 } else { ERR_ABRT }, if ok { STATUS_IDLE } else { STATUS_IDLE_ERR });
}

/// ATA WRITE SECTORS on the emulated disk.
///
/// Only accepted while the GD-ROM is ejected. The disk exposes the same FAT32
/// volume the images live on, so a host writing it while a CD_READ is
/// resolving file offsets against that volume would pull the ground out from
/// under an in-flight transfer. Requiring the eject makes the host state its
/// intent first, and is why START STOP UNIT exists here at all. Refusing with
/// ABRT is a normal, recoverable answer -- the host learns to eject first.
/// Poll `cond` until true, spinning on a yield rather than sleeping.
///
/// The time driver runs off the SAM3U's 32.768 kHz RTT with a /32 prescaler,
/// so ~1 ms is the finest sleep available and `Duration::from_micros(100)` is
/// really a whole millisecond. In the per-sector transfer loops that quantum
/// *is* the transfer rate: a 64-sector write spent ~64 ms asleep and a
/// millisecond moving data, which is most of why WRITE DMA measured no faster
/// than PIO. An earlier version yielded 400 times and then slept a tick to
/// keep count of the timeout; a host that takes longer than those 400 spins
/// to deliver a sector -- a PIO write of 256 words does -- paid the tick on
/// every sector, and 64-sector writes still cost 0.8 ms/sector. The timeout
/// is now measured, not counted, so nothing here ever sleeps. Returns false
/// if it gave up.
async fn poll_until(mut cond: impl FnMut() -> bool, timeout_ms: u32) -> bool {
    let start = Instant::now();
    while !cond() {
        if (Instant::now() - start).as_millis() >= timeout_ms as u64 {
            return false;
        }
        yield_now().await;
    }
    true
}

/// Why a host-paced wait gave up.
#[derive(Clone, Copy, PartialEq, Eq, defmt::Format)]
enum Stall {
    Timeout,
    /// The host wrote a command to the *other* device.
    PeerCommand,
    /// The host reset the bus (hard reset or SRST) mid-transfer.
    Reset,
}

/// `poll_until` for the waits where the host owes us something -- draining a
/// FIFO, filling one -- with one extra exit: the host issuing a command to
/// the other device. ATA's non-overlapped protocol forbids that until this
/// command has completed, and no Dreamcast host does it (KOS serialises the
/// GD-ROM and ATA drivers on one semaphore), but `ide_task` services one
/// command at a time, so the alternative to bailing here is holding the
/// other device's BSY until this wait times out. Callers treat it like the
/// host abandoning the transfer: give up, report ABRT, let the sweep move
/// on to the command that is actually waiting.
async fn wait_host(dev: &IdeDevice, mut cond: impl FnMut() -> bool, timeout_ms: u32)
    -> Result<(), Stall>
{
    let peer = dev.peer();
    let mut peer_cmd = false;
    let mut reset = false;
    let done = poll_until(
        || cond()
            || { reset = dev.reset_seen(); reset }
            || { peer_cmd = peer.command_pending(); peer_cmd },
        timeout_ms,
    ).await;
    if reset {
        // The FPGA already dropped the transfer (FIFOs, DMA mode, status);
        // the flag stays set for the command loop to log and clear.
        warn!("{}: host reset the bus mid-transfer, abandoning this command",
              if dev.is_primary() { "primary" } else { "secondary" });
        Err(Stall::Reset)
    } else if peer_cmd {
        warn!("{}: host issued a command to the other device mid-transfer, abandoning this one",
              if dev.is_primary() { "primary" } else { "secondary" });
        Err(Stall::PeerCommand)
    } else if done {
        Ok(())
    } else {
        Err(Stall::Timeout)
    }
}

/// The two firmware-update steps that normal firmware handles; everything
/// after them runs from RAM, out of the separate `fwupdater` program.
fn handle_fw_command(dev: &IdeDevice, label: &'static str, snap: &CommandSnapshot) {
    let key = snap.lba(true);

    if crate::fwupdate::should_enter(snap.features, key) {
        warn!("{}: entering the firmware updater -- no further logging, no way back", label);
        // Stop the one thing that keeps running off its own interrupt even
        // with the executor parked. The updater masks interrupts itself, but
        // the CDDA feed pokes the FPGA and the updater is about to own that
        // register file.
        crate::cdda::stop();
        // SAFETY: this never returns, so no other task resumes and nothing
        // else touches the MCU again. The IDE task is the only caller, and
        // it is the task being abandoned here.
        unsafe { crate::fwupdate::run(dev.base) }
    }

    if crate::fwupdate::handle(snap.features, key, label) {
        dev.set_error(0);
        dev.set_status_intrq(STATUS_IDLE);
    } else {
        dev.set_error(ERR_ABRT);
        dev.set_status_intrq(STATUS_IDLE_ERR);
    }
    dev.ack_command();
}

async fn handle_disk_write(
    dev: &IdeDevice,
    label: &'static str,
    snap: &CommandSnapshot,
    dma: bool,
    ext: bool,
) {
    if media_present() {
        warn!("{}: disk write refused -- eject the GD-ROM first", label);
        dev.set_error(ERR_ABRT);
        dev.set_status_intrq(STATUS_IDLE_ERR);
        dev.ack_command();
        return;
    }
    if snap.device & 0x40 == 0 {
        warn!("{}: disk write in CHS mode, refusing", label);
        dev.set_error(ERR_ABRT);
        dev.set_status_intrq(STATUS_IDLE_ERR);
        dev.ack_command();
        return;
    }
    // The card is addressed in 32-bit blocks, so a 48-bit LBA above that is
    // unreachable however big the card is. Refuse rather than truncate: a
    // silently wrapped address reads the wrong sector and looks like
    // corruption much later.
    let Ok(lba) = u32::try_from(snap.lba(ext)) else {
        warn!("{}: disk write at LBA {} beyond 32-bit block addressing", label, snap.lba(ext));
        dev.set_error(ERR_ABRT);
        dev.set_status_intrq(STATUS_IDLE_ERR);
        dev.ack_command();
        return;
    };
    let count = snap.count(ext);

    let total = disk_sectors();
    if total == 0 || lba.saturating_add(count) > total {
        warn!("{}: disk write {} sectors at LBA {} past end ({})", label, count, lba, total);
        dev.set_error(ERR_ABRT);
        dev.set_status_intrq(STATUS_IDLE_ERR);
        dev.ack_command();
        return;
    }
    info!("{}: disk write LBA={} count={} dma={}", label, lba, count, dma);

    // DRQ up means "send me a sector". Unlike the read direction there is
    // nothing to stage first: the host is the producer here.
    dev.set_error(0);
    if dma {
        // No DRQ in DMA mode: the host is driven by DMARQ, which the FPGA
        // raises from write-FIFO room rather than from the status register.
        // BSY stays set for the whole command. The host does not poll status
        // during a DMA burst -- DMARQ paces it, and the FPGA does not gate
        // DMARQ on BSY -- but it *does* poll once its length counter runs
        // out. Leaving BSY clear there let it call the command complete and
        // issue the next one while we were still draining, and that command's
        // fifo_clear discarded the tail of this one.
        dev.set_status_intrq(STATUS_BSY | STATUS_DRDY | STATUS_DSC);
        dev.set_dma_mode(DmaMode::ToDevice);
    } else {
        dev.set_pio_flow_control(false, true);
        dev.capture_arm();
        dev.set_status_intrq(STATUS_DRDY | STATUS_DSC | STATUS_DRQ);
    }
    dev.ack_command();
    if let Some(t) = unsafe { CMD_SEEN_AT } {
        info!("{}: DRQ armed {}us after the command was noticed", label,
              (Instant::now() - t).as_micros());
    }

    // One CMD25 for the whole command, fed a sector at a time as the host
    // delivers them. CMD24 per sector cost a command/response round trip
    // plus a full program-busy wait on every block, which capped writes at
    // ~340 KB/s whatever the IDE side did. Opened after DRQ so the CMD25
    // setup overlaps the host's first sector instead of delaying it.
    const WORDS: usize = 256;
    let mut block = [0u8; 512];
    let mut ok = true;
    let mut mw = match unsafe { embassy_atsam3::hsmci::write_multi_begin(lba, count) } {
        Ok(mw) => Some(mw),
        Err(e) => {
            warn!("{}: CMD25 at block {} failed: {:?}", label, lba, e);
            ok = false;
            None
        }
    };
    for sector in 0..count {
        if !ok {
            break;
        }
        // PIO flow control is DRQ, per sector. Leaving it asserted for the
        // whole command gives the host no reason to pause, and since the FPGA
        // asserts w_en regardless of w_rdy a host that runs ahead of our SD
        // writes silently overruns the 512-word FIFO and loses data. Dropping
        // DRQ while we program the card is what makes it wait.
        //
        // DMA needs none of this: DMARQ is driven from write-FIFO room, so
        // the flow control is in the FPGA and the host is already paced.
        if !dma && sector > 0 {
            dev.set_status_intrq(STATUS_DRDY | STATUS_DSC | STATUS_DRQ);
        }

        if let Err(why) = wait_host(dev, || dev.write_fifo_level() >= WORDS as u16, 10_000).await {
            if why == Stall::Timeout {
                // Sample what the FPGA holds while the host is polling. If
                // this shows 0x58 while the DC reads 0xd8 off the bus, the
                // two read paths genuinely disagree. COMMAND_PEND
                // distinguishes the two things that can set BSY: an IDE
                // command write sets both, an MCU status write sets only BSY.
                warn!("{}: host stopped sending at sector {}: wfifo={} lvl_at_read={} reads={} status={:#04x} hw={:#06x}",
                      label, sector, dev.write_fifo_level(), dev.read(22), dev.read(23),
                      dev.read(REG_STATUS) & 0xFF, dev.read(REG_HW_INFO));
                dev.capture_dump(label);
            }
            ok = false;
            break;
        }

        // DRQ down *and BSY up*: the host must not start the next sector --
        // or the next command -- while we drain the FIFO and program the
        // card. Dropping to a plain idle status here left BSY clear, so a
        // host that had just sent its last word saw the command as complete
        // and issued the next one, whose `fifo_clear` wiped the block out
        // from under this loop; the rest of the drain then returned the
        // FIFO's stale last word and that garbage reached the card.
        dev.set_status(STATUS_BSY | STATUS_DRDY | STATUS_DSC);
        // The tail of a sector came back as one repeated word, which is what
        // popping a drained FIFO yields. Check the level actually covers the
        // block before and after draining it.
        let lvl_before = dev.write_fifo_level();
        for i in 0..WORDS {
            let w = dev.pop_write_fifo();
            block[i * 2] = (w & 0xFF) as u8;
            block[i * 2 + 1] = (w >> 8) as u8;
        }
        let lvl_after = dev.write_fifo_level();
        if lvl_before < WORDS as u16 {
            warn!("{}: sector {} of {}: level {} before pop, {} after (short!)",
                  label, sector, count, lvl_before, lvl_after);
        }
        let mw = mw.as_mut().unwrap();
        if let Err(e) = unsafe { mw.feed(&block) } {
            warn!("{}: SD write failed at block {}: {:?}", label, lba + sector, e);
            ok = false;
            break;
        }
    }
    // CMD12 + the last block's program time. A short transfer (host stopped
    // sending) is closed the same way; the card keeps what it got.
    if let Some(mw) = mw.take() {
        if let Err(e) = unsafe { mw.end() } {
            warn!("{}: CMD12 after block {} failed: {:?}", label, lba + count - 1, e);
            ok = false;
        }
    }

    if dma {
        dev.set_dma_mode(DmaMode::Off);
    } else {
        dev.set_pio_flow_control(false, false);
    }
    dev.complete(if ok { 0 } else { ERR_ABRT }, if ok { STATUS_IDLE } else { STATUS_IDLE_ERR });
    if ok {
        // Anything cached from this volume may now be stale.
        invalidate_cd_read_cache();
    }
}

/// Wait for the host to drain a staged PIO data-in transfer, then drop DRQ
/// and raise the completion interrupt.
///
/// The ATAPI path folds this into `send_data_in`; plain ATA commands like
/// IDENTIFY DEVICE need the same ending without the packet-protocol byte
/// count / interrupt-reason bookkeeping.
async fn finish_pio_in(dev: &IdeDevice) {
    if wait_host(dev, || dev.read_fifo_level() == 0 || dev.command_pending(), 10_000).await
        == Err(Stall::Timeout)
    {
        warn!("finish_pio_in: host never drained FIFO, abandoning");
    }
    dev.complete(0, STATUS_IDLE);
}

// ---------------------------------------------------------------------------
// Canned responses
// ---------------------------------------------------------------------------

/// 6-byte Sega vendor disc-authentication response. The BIOS sends
/// opcode 0x70 / 0x71 during boot/launch and refuses to mount the disc
/// unless the device replies with this exact byte sequence.
static VENDOR_HANDSHAKE_REPLY: [u8; 6] = [0xBA, 0x06, 0x0D, 0xCA, 0x6A, 0x1F];

/// 18-byte ATAPI fixed-format sense data: "no sense, ready, no error".
static REQUEST_SENSE_REPLY: [u8; 18] = [
    0xF0, // valid + response code 0x70 (current error)
    0x00, // segment
    0x00, // sense key = NO SENSE
    0x00, 0x00, 0x00, 0x00, // information
    0x0A, // additional sense length = 10
    0x00, 0x00, 0x00, 0x00, // command-specific
    0x00, // ASC
    0x00, // ASCQ
    0x00, // FRU code
    0x00, 0x00, 0x00, // sense-key specific
];

/// Where the pickup "is" for REQ_STAT / GET_SCD: `(fad, track)` -- the
/// sector CDDA is playing or paused at, or the start of track 1 when idle.
fn head_position() -> (u32, Option<&'static crate::gdi::Track>) {
    let s = crate::cdda::snapshot();
    let gdi = gdi_ref();
    let fad = match s.status {
        crate::cdda::Status::Standby => 150,
        _ => s.fad,
    };
    let track = gdi.and_then(|g| g.track_for_lba(fad.saturating_sub(150)));
    (fad, track)
}

/// 10-byte Sega REQ_STAT reply (Marcus Comstedt's layout): drive status,
/// repeat count, CTRL/ADR, TNO, index, FAD MSB..LSB, max retry, 0.
fn build_req_stat_reply() -> [u8; 10] {
    let s = crate::cdda::snapshot();
    let status: u8 = match s.status {
        crate::cdda::Status::Playing => 3,
        crate::cdda::Status::Paused | crate::cdda::Status::Finished => 1,
        crate::cdda::Status::Error => 9,
        crate::cdda::Status::Standby => 2,
    };
    let (fad, track) = head_position();
    let (ctrl_adr, tno) = track.map(|t| ((t.ctrl << 4) | 1, t.number)).unwrap_or((0x41, 1));
    [
        status,
        s.repeat,
        ctrl_adr,
        tno,
        0x01,
        (fad >> 16) as u8,
        (fad >> 8) as u8,
        fad as u8,
        0x00,
        0x00,
    ]
}

fn bcd(n: u32) -> u8 {
    (((n / 10) << 4) | (n % 10)) as u8
}

/// Minute/second/frame of a block address, BCD, into `out[0..3]`.
fn put_msf(out: &mut [u8], blk: u32) {
    out[0] = bcd(blk / 4500);
    out[1] = bcd((blk % 4500) / 75);
    out[2] = bcd(blk % 75);
}

/// 32-byte REQ_MODE reply. The Sega GD-ROM REQ_MODE packet selects an
/// offset (packet[2]) and length (packet[4]) into this 32-byte block.
/// The BIOS in particular reads bytes 18..26 (alloc=8) and expects to
/// find the firmware version string `"Rev 5.07"` there — anything
/// else and it loops the read forever (per iceGDROM's empirical
/// findings, ide.c:195/207). Other ranges are best-effort filler that
/// looks like a real GD-ROM drive.
static REQ_MODE_REPLY: [u8; 32] = [
    0x00, 0x00, // bytes 0..2:    reserved
    b'S', b'E', b' ', b' ', b' ', b' ', b' ', b' ', // bytes 2..10:   hardware id
    b'G', b'D', b'-', b'R', b'O', b'M', b' ', b' ', // bytes 10..18:  model
    b'R', b'e', b'v', b' ', b'5', b'.', b'0', b'7', // bytes 18..26:  firmware ver
    b'9', b'6', b'3', b'6', // bytes 26..30:  date code
    0x00, 0x00, // bytes 30..32:  reserved
];

/// 10-byte REQ_ERROR reply (no error).
static REQ_ERROR_REPLY: [u8; 10] = [0; 10];

/// Build a 6-byte REQ_SES reply describing one session (0-indexed).
/// REQ_SES reply format (verified byte-for-byte against a stock
/// GDEMU via `tools/g1bios_replay`):
///   byte 0 = drive_status low nibble (= LBA_LOW & 0x0f = 0x01 ready)
///   byte 1 = 0x00
///   byte 2 = track number
///   bytes 3..6 = 24-bit FAD, big-endian
/// Subcommand semantics (packet[2]):
///   0 → "disc summary": session count + lead-out FAD
///   1 → first SDA track + its starting FAD (150 for a normal GDI)
///   2 → on a MIL-CD, the first track of session 2 + its starting FAD
///   past the last session → ABRT
/// The BIOS learns the session count from subcommand 0's track byte, not by
/// probing upwards until something aborts.
/// Stock GDEMU does NOT report an HDA session — even a dual-area
/// GD-ROM image gets reported as a single session here. Earlier
/// 0-indexed-iceGDROM-style mapping was wrong.
fn build_req_ses_reply(_gdi: Option<&'static crate::gdi::Gdi>, session: u8) -> Option<[u8; 6]> {
    let mut r = [0u8; 6];
    r[0] = 0x01; // drive status: ready (matches LBA_LOW=0x81 low nibble)
    let (track, fad) = if let Some(g) = gdi_ref() {
        match session {
            0 => {
                // Disc summary: session *count* + lead-out FAD.
                //
                // Not the first track number. The two are the same value on a
                // GD-ROM -- stock GDEMU reports one session, whose first track
                // is also 1 -- which is why sending the track number here
                // matched a stock unit byte-for-byte and still broke MIL-CD.
                // A MIL-CD has two sessions and keeps its IP.BIN, its ISO9660
                // filesystem and its 1ST_READ.BIN in the second one, so
                // claiming one session tells the BIOS the only bootable part
                // of the disc is not there. It then never probes session 2,
                // never reads a sector, and gives up.
                // flycast `core/imgread/common.h`, `GetSessionInfo`:
                //   if (session == 0) { to[2] = sessions.size(); ... }
                (g.sessions, g.leadout_lba() + 150)
            }
            1 => {
                // Session 1 = SDA: first SDA track + its starting FAD.
                let t = g.tracks().iter().find(|t| t.start_lba < g.boot_area_lba)?;
                (t.number, t.start_lba + 150)
            }
            2 if g.is_cdi => {
                // A MIL-CD really has two sessions, and its bootable data
                // track is the whole of session 2. The BIOS learns the
                // session count from which index first returns ABRT, so
                // stopping at 1 (correct for a GD-ROM, which stock GDEMU
                // reports as a single session) tells it session 2 does not
                // exist -- and session 2 is the only place a MIL-CD keeps
                // anything bootable. `boot_area_lba` is the first track
                // numbered 3 or above, which on a CDI is exactly where the
                // second session begins.
                let t = g.tracks().iter().find(|t| t.start_lba >= g.boot_area_lba)?;
                (t.number, t.start_lba + 150)
            }
            _ => return None, // out of range — caller should abort
        }
    } else {
        return None;
    };
    r[2] = track;
    r[3] = (fad >> 16) as u8;
    r[4] = (fad >> 8) as u8;
    r[5] = fad as u8;
    Some(r)
}

/// Build a 100-byte CD_SCD format-0 reply with a bit-expanded
/// Q-subcode payload. The 4-byte header is `[0, audio_status, 0, 100]`
/// and the remaining 96 bytes are 12 bytes of Q-subchannel expanded
/// MSB-first into one byte per bit (`0x7F` for 1, `0x3F` for 0), with
/// a CRC-CCITT (poly 0x1021, init 0) computed across the bit stream
/// and substituted into Q[10..12] partway through expansion. Sending
/// all-zero payload bytes makes the BIOS see ADR=0 / CTRL=0 / MSF=0
/// and re-poll forever, never proceeding to CD_READ. Per iceGDROM
/// `firmware-rv32/source/cdda.c`.
///
/// Format 1 (the one games poll while CDDA plays) is the plain 14-byte
/// form: the same header with length 14, then Q[0..10]. Returns the reply
/// length.
fn build_get_scd_reply(format: u8, r: &mut [u8; 100]) -> usize {
    let s = crate::cdda::snapshot();
    let audio_status: u8 = match s.status {
        crate::cdda::Status::Playing => 0x11,
        crate::cdda::Status::Paused => 0x12,
        crate::cdda::Status::Finished => 0x13,
        crate::cdda::Status::Error => 0x14,
        crate::cdda::Status::Standby => 0x15, // no current audio status
    };
    let (fad, track) = head_position();
    let mut q: [u8; 12] = [0; 12];
    match track {
        Some(t) => {
            q[0] = (t.ctrl << 4) | 1; // CTRL, ADR=1 (Q-mode 1)
            q[1] = bcd(t.number as u32);
            put_msf(&mut q[3..6], fad.saturating_sub(t.start_lba + 150));
        }
        None => q[0] = 0x41,
    }
    q[2] = 0x01; // index 1
    put_msf(&mut q[7..10], fad);
    // q[10..12] will be overwritten with CRC mid-expansion.

    r.fill(0);
    r[1] = audio_status;
    if format == 1 {
        r[3] = 14;
        r[4..14].copy_from_slice(&q[..10]);
        return 14;
    }
    r[3] = 100;        // data length

    // Expand bits MSB-first; compute CRC-CCITT across the bit stream.
    // After processing the first 80 bits (Q[0..10]) we substitute the
    // computed CRC into q[10..12] before continuing — matches iceGDROM.
    let mut crc: u16 = 0;
    for i in 0..12 {
        if i == 10 {
            let crc_done = !crc;
            q[10] = (crc_done >> 8) as u8;
            q[11] = (crc_done & 0xff) as u8;
        }
        let byte = q[i];
        for bit in 0..8 {
            let b = (byte >> (7 - bit)) & 1;
            // CRC update before emitting (poly 0x1021, MSB-first).
            let top = (crc >> 15) & 1;
            crc = (crc << 1) ^ if (top ^ b as u16) != 0 { 0x1021 } else { 0 };
            // Emit bit as 0x7F / 0x3F at offset 4 + i*8 + bit.
            r[4 + i * 8 + bit] = if b != 0 { 0x7F } else { 0x3F };
        }
    }
    100
}

// ---------------------------------------------------------------------------
/// Stage an ATA `IDENTIFY DEVICE` response for the secondary (hard disk).
///
/// 256 words. Only the fields a host actually needs to enumerate and address
/// the disk are filled; everything else stays zero, which is legal.
fn stage_identify_disk(dev: &IdeDevice) {
    let mut buf = [0u16; 256];

    // Word 0: general config. Bit 6 = non-removable, bit 15 clear = ATA.
    buf[0] = 0x0040;
    // Words 1/3/6: obsolete CHS geometry. Hosts that ignore LBA still read
    // these, and zeroes make some of them reject the drive outright.
    buf[1] = 0x3FFF;   // cylinders (clamped legacy value)
    buf[3] = 16;       // heads
    buf[6] = 63;       // sectors per track
    put_ata_string(&mut buf[10..20], b"OPENGDEMU0000000000  ");   // serial
    put_ata_string(&mut buf[23..27], b"1.0     ");                // firmware
    put_ata_string(&mut buf[27..47], b"OpenGDEMU SD Disk                       ");
    buf[47] = 0x8001;  // max sectors per READ/WRITE MULTIPLE
    buf[49] = 0x0200;  // capabilities: LBA supported
    buf[53] = 0x0007;  // words 54-58, 64-70 and 88 are valid
    let sectors = disk_sectors();
    // Words 60/61 are the LBA28 capacity and the spec caps them at the
    // 28-bit maximum. Reporting the true size of a 500 GB card here is not
    // just cosmetic: a host that believes it will happily issue an LBA28
    // read of a sector no 28-bit address can name, and the address it
    // actually puts on the bus is the low 28 bits -- the wrong sector,
    // returned as if it were the right one.
    buf[60] = (sectors.min(MAX_LBA28_SECTORS) & 0xFFFF) as u16;
    buf[61] = (sectors.min(MAX_LBA28_SECTORS) >> 16) as u16;
    buf[80] = 0x0070;  // ATA-4/5/6
    buf[82] = 0x0000;
    // Word 83 bit 10: 48-bit addressing supported. Bits 14/15 must read
    // 01b or the host treats the whole word as meaningless.
    buf[83] = 0x4400;
    // Word 86 bit 10: and it is enabled. Hosts check this one, not 83.
    buf[86] = 0x0400;
    buf[88] = 0x0000;  // no UDMA advertised -- PIO first, DMA once proven
    // Words 100-103: the real capacity, as a 48-bit little-endian count.
    buf[100] = (sectors & 0xFFFF) as u16;
    buf[101] = (sectors >> 16) as u16;
    buf[102] = 0;
    buf[103] = 0;

    for (i, w) in buf.iter().enumerate() {
        let _ = i;
        dev.push_read_fifo(*w);
    }
}

/// ATA identify strings are byte-swapped within each 16-bit word and space
/// padded. `dst` is sized in words; `src` is taken as-is and truncated or
/// space-filled to fit.
fn put_ata_string(dst: &mut [u16], src: &[u8]) {
    for (i, w) in dst.iter_mut().enumerate() {
        let a = *src.get(i * 2).unwrap_or(&b' ');
        let b = *src.get(i * 2 + 1).unwrap_or(&b' ');
        *w = ((a as u16) << 8) | (b as u16);
    }
}

/// Addressable sector count reported by the emulated disk.
///
/// Placeholder until the disk is backed by a real image or partition on the
/// SD card: 512 MB of 512-byte sectors, comfortably inside LBA28.
pub fn disk_sectors() -> u32 {
    DISK_SECTORS.load(core::sync::atomic::Ordering::Relaxed)
}

// IDENTIFY PACKET DEVICE response (256 words = 512 bytes)
// ---------------------------------------------------------------------------

fn stage_identify_packet(dev: &IdeDevice) {
    // Most fields are zero; we fill in the few that hosts actually look
    // at. Words are little-endian on the wire; ASCII strings are stored
    // big-endian-per-word per ATA convention.
    let mut buf = [0u16; 256];
    buf[0] = 0x8580; // word 0: ATAPI device, removable, CD-ROM, IRQ on PACKET
    write_ata_string(&mut buf[10..20], b"GDEMU0000000000000000"); // serial #
    write_ata_string(&mut buf[23..27], b"1.00    "); // firmware rev
    write_ata_string(&mut buf[27..47], b"OpenGDEMU GD-ROM        "); // model
    buf[49] = 0x0F00; // capabilities (LBA, IORDY, DMA)
    for w in buf.iter() {
        dev.push_read_fifo(*w);
    }
}

fn write_ata_string(words: &mut [u16], src: &[u8]) {
    // ATA stores ASCII with even-byte first within each word — so byte 0
    // is the *high* byte of word 0, byte 1 is the *low* byte, etc.
    for (i, w) in words.iter_mut().enumerate() {
        let hi = src.get(2 * i).copied().unwrap_or(b' ') as u16;
        let lo = src.get(2 * i + 1).copied().unwrap_or(b' ') as u16;
        *w = (hi << 8) | lo;
    }
}

// `STATUS_BSY` etc. are documented constants used only by the FPGA-side
// register file — keep around for reference / future use.
#[allow(dead_code)]
const _STATUS_BSY_DOC: u8 = STATUS_BSY;
