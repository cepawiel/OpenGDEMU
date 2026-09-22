//! Standalone flash writer for in-field firmware updates.
//!
//! This is not part of the firmware binary. It is built as its own program,
//! linked to run from the base of SRAM1 (see `updater.x`), emitted as a raw
//! binary, and embedded in the firmware image as a byte blob. When a firmware
//! update starts, `opengdemu`'s `fwupdate` shim copies the blob to
//! `0x2008_0000` and branches to offset 0 — [`_start`].
//!
//! # Why a separate binary
//!
//! While the EEFC is programming, its flash plane cannot be read, so an
//! instruction fetched from the plane being written faults. The SAM3U2E has a
//! single plane, so "run from the other bank" is not an option, and the ROM
//! IAP entry that would side-step this is documented for SAM3S/4S/4E, not
//! SAM3U. The flash writer therefore has to execute from RAM.
//!
//! The obvious way to get that — `#[link_section = ".data"]` on each function
//! — works, but only as long as nothing in the module gives the compiler a
//! reason to emit a call to a helper that is still in flash. A `[0u8; N]`
//! initialiser becoming `__aeabi_memclr4` is enough, the linker will happily
//! route it through a veneer, and the failure shows up as a hard fault
//! mid-update rather than a build error.
//!
//! Building it as its own program removes the hazard at the root: this
//! program links its own copy of `compiler_builtins`, so its `memcpy` and
//! friends are inside the blob at RAM addresses. Nothing it can call lives in
//! flash, because flash is not in its address space at all. Ordinary Rust is
//! fine in here.
//!
//! The caller's stack (in SRAM0) is borrowed as-is; SRAM0 is untouched by
//! everything below, which is why the link address is in SRAM1.
//!
//! [`_start`] never returns. The only ways out are a host-commanded reset or
//! the SWD probe.
//!
//! # Protocol
//!
//! Every step is vendor opcode `0xF9` on the disk device, selected by the
//! FEATURES byte. Getting here at all took a 56-bit key (see the firmware's
//! `fwupdate` shim); from this point the sequence is:
//!
//! ```text
//!   F9 / feat 55                header, one 512-byte block
//!   F9 / feat 56, count = n     n 512-byte payload blocks
//!   F9 / feat 57                verify: CRC32 what was written
//!   F9 / feat 58                reset
//! ```
//!
//! Header and payload are per flash plane, so a full update runs
//! header/data/verify once for the code plane and once for the bitstream
//! plane before the final reset.
//!
//! Results come back in the ERROR register with the ERR status bit, since
//! there is no logging in here: see [`Status`].

#![no_std]
#![no_main]

use core::ptr::{read_volatile, write_volatile};

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    // Nothing to report to and nowhere to go: a panic here means a bug in
    // this file. Stop, and leave the host's command un-answered so it times
    // out rather than believing the update succeeded.
    loop {
        core::hint::spin_loop();
    }
}

const CMD_FW: u8 = 0xF9;

const FEAT_HEADER: u8 = 0x55;
const FEAT_DATA: u8 = 0x56;
const FEAT_VERIFY: u8 = 0x57;
const FEAT_RESET: u8 = 0x58;

/// Header magic, at the start of the 512-byte header block: "OGFW".
const HEADER_MAGIC: u32 = 0x5746_474F;
const HEADER_VERSION: u32 = 1;

/// Reported in the ERROR register alongside the ERR status bit.
///
/// The encoding is forced by the hardware: the FPGA does not store a whole
/// ERROR byte. It keeps an ABRT flag and a 4-bit sense key, and rebuilds the
/// host-visible register as `[_, _, ABRT, _, sense_key]` (see `decode.error`
/// in `hw/ide_device.py`), so bits 0, 1 and 3 read back as zero no matter
/// what is written. A dense 0x10..0x16 numbering silently collapsed —
/// 0x13 and 0x11 both arrived as 0x10, 0x16 as 0x14.
///
/// So: ABRT set on every failure, and the reason in the sense-key nibble.
/// Plain 0x04 (ABRT, nibble 0) is what the *normal* firmware returns when it
/// refuses to arm, which keeps "refused to start" distinct from every way an
/// update can fail once it has started.
#[repr(u8)]
enum Status {
    Ok = 0x00,
    BadHeader = 0x14,
    BadDest = 0x24,
    BadLength = 0x34,
    NoHeader = 0x44,
    Overrun = 0x54,
    FlashError = 0x64,
    CrcMismatch = 0x74,
}

// --- EEFC ---------------------------------------------------------------

const EFC0: *mut u32 = 0x400E_0800 as *mut u32;
const EFC1: *mut u32 = 0x400E_0A00 as *mut u32;
const EEFC_FCR: usize = 1; // 0x04 / 4
const EEFC_FSR: usize = 2; // 0x08 / 4
const FSR_FRDY: u32 = 1 << 0;
/// FCMDE | FLOCKE: bad command, or the target lock region is locked.
const FSR_ERRORS: u32 = (1 << 1) | (1 << 2);

/// Erase page and write page, in one command. SAM3U still has EWP (SAM4
/// dropped it for a separate erase), which is why there is no erase pass
/// below — but also why a page must be filled completely before committing.
const FCMD_EWP: u32 = 0x03;
const FKEY: u32 = 0x5A;

/// Flash page size, and the granularity of EWP.
const PAGE: u32 = 256;

/// Plane 0 holds the code, plane 1 the compressed bitstream. On a SAM3U2E
/// only plane 0 exists, and `plane_of` simply never matches the second entry.
const PLANE0_BASE: u32 = 0x0008_0000;
const PLANE1_BASE: u32 = 0x0010_0000;
const PLANE_LEN: u32 = 0x0002_0000;

// --- FPGA register file (mirrors the constants in the firmware's ide.rs) --

const REG_HW_INFO: usize = 0;
const REG_STATUS: usize = 1;
const REG_ERROR: usize = 3;
const REG_CMD_FEAT: usize = 4;
const REG_SECTOR_COUNT: usize = 5;
const REG_DATA_FIFO: usize = 9;
const REG_WRITE_FIFO_LEVEL: usize = 11;
const REG_DMA_MODE: usize = 19;

const HW_CMD_PEND: u16 = 0x0002;

const STATUS_BSY: u8 = 0x80;
const STATUS_DRDY: u8 = 0x40;
const STATUS_DSC: u8 = 0x10;
const STATUS_DRQ: u8 = 0x08;
const STATUS_ERR: u8 = 0x01;
const STATUS_IDLE: u8 = STATUS_DRDY | STATUS_DSC;
const ERR_ABRT: u8 = 0x04;

/// Words per 512-byte block, matching the host's PIO sector size.
const WORDS_PER_BLOCK: u16 = 256;

/// The FPGA's register window, as a handle so the register pokes read
/// somewhat like the firmware's `IdeDevice`.
#[derive(Clone, Copy)]
struct Fpga(*mut u16);

impl Fpga {
    #[inline]
    fn read(self, reg: usize) -> u16 {
        unsafe { read_volatile(self.0.add(reg)) }
    }

    #[inline]
    fn write(self, reg: usize, val: u16) {
        unsafe { write_volatile(self.0.add(reg), val) }
    }

    /// Post a status and assert INTRQ in the same FPGA clock (bit 8). Split
    /// writes lose interrupts on hosts that poll Alt Status.
    fn set_status_intrq(self, status: u8) {
        self.write(REG_STATUS, status as u16 | 0x0100);
    }

    fn command_pending(self) -> bool {
        self.read(REG_HW_INFO) & HW_CMD_PEND != 0
    }

    /// Clear COMMAND_PEND so the FPGA stops asserting MCU_IRQ and accepts
    /// the next command.
    fn ack_command(self) {
        self.write(REG_HW_INFO, 0);
    }

    fn pop16(self) -> u16 {
        self.read(REG_DATA_FIFO)
    }

    fn pop32(self) -> u32 {
        let lo = self.pop16() as u32;
        let hi = self.pop16() as u32;
        lo | (hi << 16)
    }

    /// End a command: drop flow control, post the result, release the latch.
    fn finish(self, status: Status) {
        self.write(REG_DMA_MODE, 0);
        let err = status as u8;
        self.write(REG_ERROR, err as u16);
        self.set_status_intrq(if err == 0 {
            STATUS_IDLE
        } else {
            STATUS_IDLE | STATUS_ERR
        });
        self.ack_command();
    }

    /// Arm the FPGA's PIO write backpressure and raise DRQ for the first
    /// block. Bit 2 of REG_DMA_MODE makes the FPGA drop the host-visible DRQ
    /// whenever the write FIFO is full — polling from here is far too slow to
    /// do that in software, and the host would overrun the FIFO.
    fn open_data_out(self) {
        self.write(REG_DMA_MODE, 4);
        self.write(REG_ERROR, 0);
        self.set_status_intrq(STATUS_DRDY | STATUS_DSC | STATUS_DRQ);
        self.ack_command();
    }

    /// Spin until a whole block is in the write FIFO. False if the host gave
    /// up and issued another command instead.
    fn await_block(self) -> bool {
        loop {
            if self.read(REG_WRITE_FIFO_LEVEL) >= WORDS_PER_BLOCK {
                return true;
            }
            if self.command_pending() {
                return false;
            }
        }
    }

    /// DRQ down and BSY up. The host must not start the next block while a
    /// page programs: the FIFO is 512 words and an EWP takes milliseconds.
    fn close_block(self) {
        self.write(REG_STATUS, (STATUS_BSY | STATUS_DRDY | STATUS_DSC) as u16);
    }
}

/// Where the next payload block goes, and what to check at the end.
#[derive(Default)]
struct Target {
    /// Next flash address to program. `None` until a header is accepted.
    cursor: Option<u32>,
    /// First address of the current image, kept for the CRC pass.
    base: u32,
    /// Bytes still expected for the current image.
    remaining: u32,
    /// Total bytes in the current image.
    len: u32,
    crc: u32,
}

/// Blob entry point. The firmware branches here with the FPGA register
/// window in r0, having already stopped everything else on the MCU.
#[no_mangle]
#[link_section = ".entry"]
pub extern "C" fn _start(base: *mut u16) -> ! {
    // The vector table lives in the plane about to be erased.
    unsafe { core::arch::asm!("cpsid i", options(nomem, nostack)) };

    let fpga = Fpga(base);
    let mut t = Target::default();

    // Drop any flow control the interrupted command left armed, then report
    // ready so the host can start the sequence.
    fpga.write(REG_DMA_MODE, 0);
    fpga.write(REG_ERROR, 0);
    fpga.set_status_intrq(STATUS_IDLE);
    fpga.ack_command();

    loop {
        while !fpga.command_pending() {}

        let cmd_feat = fpga.read(REG_CMD_FEAT);
        let count = fpga.read(REG_SECTOR_COUNT) & 0xFF;

        if (cmd_feat & 0xFF) as u8 != CMD_FW {
            fpga.write(REG_DMA_MODE, 0);
            fpga.write(REG_ERROR, ERR_ABRT as u16);
            fpga.set_status_intrq(STATUS_IDLE | STATUS_ERR);
            fpga.ack_command();
            continue;
        }

        match (cmd_feat >> 8) as u8 {
            FEAT_HEADER => {
                let r = recv_header(fpga, &mut t);
                fpga.finish(r);
            }
            FEAT_DATA => {
                // Sector count 0 means 256, as everywhere else in ATA.
                let n = if count == 0 { 256 } else { count as u32 };
                let r = recv_data(fpga, &mut t, n);
                fpga.finish(r);
            }
            FEAT_VERIFY => {
                let r = match t.cursor {
                    Some(_) if t.remaining == 0 => {
                        if crc32(t.base, t.len) == t.crc {
                            Status::Ok
                        } else {
                            Status::CrcMismatch
                        }
                    }
                    Some(_) => Status::BadLength,
                    None => Status::NoHeader,
                };
                fpga.finish(r);
            }
            FEAT_RESET => {
                fpga.finish(Status::Ok);
                // AIRCR: VECTKEY | SYSRESETREQ.
                unsafe { write_volatile(0xE000_ED0C as *mut u32, 0x05FA_0004) };
                loop {
                    core::hint::spin_loop();
                }
            }
            _ => {
                fpga.write(REG_DMA_MODE, 0);
                fpga.write(REG_ERROR, ERR_ABRT as u16);
                fpga.set_status_intrq(STATUS_IDLE | STATUS_ERR);
                fpga.ack_command();
            }
        }
    }
}

/// Accept the 512-byte header block and set up `t` for the payload.
fn recv_header(fpga: Fpga, t: &mut Target) -> Status {
    *t = Target::default();

    fpga.open_data_out();
    if !fpga.await_block() {
        return Status::Overrun;
    }
    fpga.close_block();

    let magic = fpga.pop32();
    let len = fpga.pop32();
    let crc = fpga.pop32();
    let dest = fpga.pop32();
    let version = fpga.pop32();
    // Drain the rest of the block: the FIFO must be empty for the next one.
    for _ in 10..WORDS_PER_BLOCK {
        let _ = fpga.pop16();
    }

    if magic != HEADER_MAGIC || version != HEADER_VERSION {
        return Status::BadHeader;
    }
    // A destination must name a flash plane *and* start on a page: EWP
    // commits whole pages, so a misaligned start would silently write
    // somewhere other than where the header asked for.
    let Some(plane) = plane_of(dest) else {
        return Status::BadDest;
    };
    if dest % PAGE != 0 {
        return Status::BadDest;
    }
    // Non-empty, a whole number of blocks, and inside the plane.
    if len == 0 || len % 512 != 0 || (dest - plane).saturating_add(len) > PLANE_LEN {
        return Status::BadLength;
    }

    t.cursor = Some(dest);
    t.base = dest;
    t.len = len;
    t.remaining = len;
    t.crc = crc;
    Status::Ok
}

/// Receive `n` 512-byte blocks and program each as two flash pages.
fn recv_data(fpga: Fpga, t: &mut Target, n: u32) -> Status {
    let Some(mut cursor) = t.cursor else {
        return Status::NoHeader;
    };
    if n.saturating_mul(512) > t.remaining {
        return Status::Overrun;
    }

    fpga.open_data_out();
    for block in 0..n {
        if !fpga.await_block() {
            t.cursor = Some(cursor);
            return Status::Overrun;
        }
        fpga.close_block();

        // Two pages per block, popped straight from the FIFO into the flash
        // latch buffer. There is no intermediate copy to get out of step.
        for _ in 0..2 {
            let latch = cursor as *mut u32;
            for w in 0..(PAGE / 4) as usize {
                unsafe { write_volatile(latch.add(w), fpga.pop32()) };
            }
            if !program_page(cursor) {
                t.cursor = Some(cursor);
                return Status::FlashError;
            }
            cursor += PAGE;
            t.remaining -= PAGE;
        }

        if block + 1 < n {
            // Ask for the next one.
            fpga.set_status_intrq(STATUS_DRDY | STATUS_DSC | STATUS_DRQ);
        }
    }

    t.cursor = Some(cursor);
    fpga.write(REG_DMA_MODE, 0);
    Status::Ok
}

/// Which plane `addr` belongs to, or `None` if it is not in flash.
fn plane_of(addr: u32) -> Option<u32> {
    if (PLANE0_BASE..PLANE0_BASE + PLANE_LEN).contains(&addr) {
        Some(PLANE0_BASE)
    } else if (PLANE1_BASE..PLANE1_BASE + PLANE_LEN).contains(&addr) {
        Some(PLANE1_BASE)
    } else {
        None
    }
}

/// Commit the latch buffer to the page containing `addr`.
fn program_page(addr: u32) -> bool {
    let (efc, plane) = if addr >= PLANE1_BASE {
        (EFC1, PLANE1_BASE)
    } else {
        (EFC0, PLANE0_BASE)
    };
    // FARG is the page index within the plane; each EEFC numbers from 0.
    let page = (addr - plane) / PAGE;
    unsafe {
        write_volatile(
            efc.add(EEFC_FCR),
            (FKEY << 24) | ((page & 0xFFFF) << 8) | FCMD_EWP,
        );
        loop {
            let fsr = read_volatile(efc.add(EEFC_FSR));
            if fsr & FSR_ERRORS != 0 {
                return false;
            }
            if fsr & FSR_FRDY != 0 {
                return true;
            }
        }
    }
}

/// CRC-32 (the reflected, zlib/PNG one) over `len` bytes at `addr`.
///
/// Bitwise: a table would be 1 KB in a blob that has to be copied into RAM,
/// and at a few hundred milliseconds for a whole plane the speed is
/// irrelevant next to the programming time.
fn crc32(addr: u32, len: u32) -> u32 {
    let mut crc: u32 = 0xFFFF_FFFF;
    for i in 0..len {
        crc ^= unsafe { read_volatile((addr + i) as *const u8) } as u32;
        for _ in 0..8 {
            let mask = 0u32.wrapping_sub(crc & 1);
            crc = (crc >> 1) ^ (0xEDB8_8320 & mask);
        }
    }
    !crc
}
