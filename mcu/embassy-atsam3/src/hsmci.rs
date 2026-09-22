//! Minimal SAM3U4E HSMCI driver for SD-card bring-up.
//!
//! Polled, 1-bit, no DMA. Just enough to:
//! - Configure the HSMCI peripheral and its pins
//! - Send a command and receive its response (or detect timeout)
//!
//! Pin assignment is fixed by the SAM3U: PA3 MCCK, PA4 MCCDA, PA5..PA8 MCDA0..3,
//! all on peripheral A.

use crate::pac;
use defmt::debug;

/// Pins on PIOA used by HSMCI (MCCK + MCCDA + MCDA0..3, peripheral A).
const PIOA_HSMCI_MASK: u32 =
    (1 << 3) | (1 << 4) | (1 << 5) | (1 << 6) | (1 << 7) | (1 << 8);

/// Result of a command transaction.
#[derive(Debug, Copy, Clone, PartialEq, Eq, defmt::Format)]
pub enum CmdError {
    /// Card never replied within MAXLAT cycles.
    ResponseTimeout,
    /// Response had a CRC error.
    CrcError,
    /// Response had a wrong index, direction, or end-bit.
    BadResponse,
}

/// Response width — selects how the CMDR programs the response state machine.
#[derive(Debug, Copy, Clone, defmt::Format)]
pub enum Response {
    /// No response expected (e.g., CMD0).
    None,
    /// 48-bit response (R1, R3, R6, R7, etc.).
    Short,
    /// 136-bit response (R2, only used for CMD2/CMD9/CMD10).
    Long,
}

/// Bring up the HSMCI peripheral. Caller must have already enabled the HSMCI
/// peripheral clock (PMC PCER ID 17) and the PIOA clock.
///
/// After init, the HSMCI is enabled, slot A selected, 1-bit bus, ~400 kHz
/// SD clock for the standard initialization speed (good for CMD0..CMD7).
/// MCK as handed to `init()`. Kept so the clock can be re-derived later
/// (e.g. leaving the 400 kHz identification rate) without the caller having
/// to remember it.
static mut MCK_HZ: u32 = 0;

pub unsafe fn init(mck_hz: u32) {
    unsafe { MCK_HZ = mck_hz };
    let pac = unsafe { pac::Peripherals::steal() };

    // Pin mux: PA3..PA8 -> peripheral A (default ABSR=0), release from PIO.
    pac.PIOA
        .pdr
        .write_with_zero(|w| unsafe { w.bits(PIOA_HSMCI_MASK) });
    pac.PIOA.absr.modify(|r, w| unsafe {
        w.bits(r.bits() & !PIOA_HSMCI_MASK)
    });
    // SD card lines need pull-ups; let internal pull-ups take care of CMD/DATA
    // when the card and host are both tristated. (The board may have external
    // pull-ups too, harmless.)
    pac.PIOA
        .puer
        .write_with_zero(|w| unsafe { w.bits(PIOA_HSMCI_MASK) });

    let h = &pac.HSMCI;

    // Software reset.
    h.cr.write_with_zero(|w| w.swrst().set_bit());

    // Disable then re-enable, with power-save disabled.
    h.cr.write_with_zero(|w| w.mcidis().set_bit().pwsdis().set_bit());
    h.cr.write_with_zero(|w| w.mcien().set_bit().pwsdis().set_bit());

    // Clock: SD_CLK = MCK / (2 * (CLKDIV + 1)).
    let target_hz = 400_000u32;
    let clkdiv = ((mck_hz + (2 * target_hz) - 1) / (2 * target_hz)).saturating_sub(1);
    let clkdiv = clkdiv.min(0xFF) as u32;
    debug!("HSMCI clock: MCK={} Hz, CLKDIV={} -> ~{} Hz", mck_hz, clkdiv, mck_hz / (2 * (clkdiv + 1)));
    h.mr.write(|w| unsafe {
        w.clkdiv().bits(clkdiv as u8).pwsdiv().bits(7).rdproof().set_bit().wrproof().set_bit()
    });

    // Data timeout: maximum.
    h.dtor
        .write(|w| unsafe { w.dtocyc().bits(0xF).dtomul()._1048576() });

    // Completion-signal timeout — preserved across reset, harmless otherwise.
    h.cstor
        .write(|w| unsafe { w.cstocyc().bits(2).cstomul()._1048576() });

    // Slot A, 1-bit bus to start. Card init must be done in 1-bit mode.
    h.sdcr
        .write(|w| w.sdcsel().slota().sdcbus()._1());

    // CFG: FERRCTRL only — try FIFOMODE=0 (single-transfer) to see if
    // RXRDY works differently for the data path on this board.
    h.cfg.write(|w| w.ferrctrl().set_bit());
}

/// Reset the HSMCI peripheral while preserving its programmed configuration.
/// After a command failure the HSMCI can be left in a stuck state that
/// causes subsequent commands to also time out — `reset()` brings it back
/// to a clean state without losing MR/DTOR/SDCR/CSTOR/CFG. Modeled on
/// ASF's `hsmci_reset()`.
pub unsafe fn reset() {
    let h = unsafe { &*pac::HSMCI::ptr() };
    let mr = h.mr.read().bits();
    let dtor = h.dtor.read().bits();
    let sdcr = h.sdcr.read().bits();
    let cstor = h.cstor.read().bits();
    let cfg = h.cfg.read().bits();

    h.cr.write_with_zero(|w| w.swrst().set_bit());

    h.mr.write(|w| unsafe { w.bits(mr) });
    h.dtor.write(|w| unsafe { w.bits(dtor) });
    h.sdcr.write(|w| unsafe { w.bits(sdcr) });
    h.cstor.write(|w| unsafe { w.bits(cstor) });
    h.cfg.write(|w| unsafe { w.bits(cfg) });

    h.cr.write_with_zero(|w| w.mcien().set_bit().pwsdis().set_bit());
}

/// Send `count` clocks on the SD bus with no command — used at start of init
/// to power up the card. SD spec needs at least 74 clocks.
pub unsafe fn send_init_clocks() {
    let h = unsafe { &*pac::HSMCI::ptr() };
    // SPCMD=INITCMD and CMDNB=0: drives 74+ clocks with CMD line held high.
    h.cmdr.write_with_zero(|w| {
        w.spcmd()
            .init()
            .opdcmd()
            .opendrain()
            .rsptyp()
            .noresp()
    });
    while h.sr.read().cmdrdy().bit_is_clear() {}
}

/// Send a command and wait for the response stage to complete. `ignore_crc`
/// is for R3 (OCR / ACMD41) — the SD spec says the response has no CRC, but
/// the SAM3U HSMCI flags one anyway.
pub unsafe fn send_command(
    cmd_num: u8,
    arg: u32,
    response: Response,
    open_drain: bool,
    ignore_crc: bool,
) -> Result<u32, CmdError> {
    let h = unsafe { &*pac::HSMCI::ptr() };

    h.argr.write(|w| unsafe { w.bits(arg) });
    h.cmdr.write_with_zero(|w| {
        let w = w.cmdnb().bits(cmd_num).maxlat().set_bit();
        let w = match response {
            Response::None => w.rsptyp().noresp(),
            Response::Short => w.rsptyp()._48_bit(),
            Response::Long => w.rsptyp()._136_bit(),
        };
        if open_drain {
            w.opdcmd().opendrain()
        } else {
            w.opdcmd().pushpull()
        }
    });

    loop {
        let sr = h.sr.read();
        if sr.cmdrdy().bit_is_set() {
            if sr.rtoe().bit_is_set() {
                reset();
                return Err(CmdError::ResponseTimeout);
            }
            if sr.rcrce().bit_is_set() && !ignore_crc {
                reset();
                return Err(CmdError::CrcError);
            }
            if (sr.rinde().bit_is_set() || sr.rdire().bit_is_set() || sr.rende().bit_is_set())
                && !ignore_crc
            {
                reset();
                return Err(CmdError::BadResponse);
            }
            break;
        }
    }

    // RSPR is a FIFO. Popping it here for a long response would eat the first
    // of the four words, which is exactly what happened: the CSD came back as
    // CSD[95:0] with a zero word appended, so C_SIZE was read 32 bits out of
    // position and a 977,240,064-block card reported 2,688,000. Leave the FIFO
    // alone and let long_response() take all four.
    match response {
        Response::Long => Ok(0),
        _ => Ok(h.rspr[0].read().bits()),
    }
}

/// Read all four response words. For an R2 long response these hold the 128
/// CID/CSD bits, MSB first (rspr[0] = top 32 bits of the response).
///
/// On SAM3U the four RSPR addresses are independent registers; ASF reads
/// rspr[0] four times because some other Atmel HSMCI variants treat it as
/// a FIFO, but on SAM3U either pattern returns the same data.
pub unsafe fn long_response() -> [u32; 4] {
    let h = unsafe { &*pac::HSMCI::ptr() };
    // RSPR is a 4-deep FIFO, not four independent registers: ASF's
    // hsmci_get_response_128() reads RSPR[0] four times, and doing anything
    // else here returns the response shifted. The CSD came back as
    // db59000e 8fdf7f80 0a4000fb 00000000 -- valid data with a zero word at
    // the end, i.e. shifted 32 bits, which decoded to 2,688,000 blocks for a
    // 977,240,064-block card.
    [
        h.rspr[0].read().bits(),
        h.rspr[0].read().bits(),
        h.rspr[0].read().bits(),
        h.rspr[0].read().bits(),
    ]
}

/// Reset the SD clock divider. Effective bus rate is `mck_hz / (2 * (clkdiv + 1))`.
pub unsafe fn set_clock_div(clkdiv: u8) {
    let h = unsafe { &*pac::HSMCI::ptr() };
    h.mr.modify(|_, w| unsafe { w.clkdiv().bits(clkdiv) });
}

/// Pick the CLKDIV that lands at or just below `target_hz`, and apply it.
/// Returns the resulting bus clock. Rounding is deliberately upward on the
/// divider (downward on the clock) so we never overshoot the card's limit.
pub unsafe fn set_clock_hz(target_hz: u32) -> u32 {
    let mck_hz = unsafe { MCK_HZ };
    if mck_hz == 0 || target_hz == 0 {
        return 0;
    }
    let div = ((mck_hz + (2 * target_hz) - 1) / (2 * target_hz)).saturating_sub(1);
    let div = div.min(0xFF) as u8;
    unsafe { set_clock_div(div) };
    mck_hz / (2 * (div as u32 + 1))
}

/// Switch the host side of the bus to 4 data lines.
///
/// Only valid once the card itself has been switched with ACMD6 — the two
/// must agree or every subsequent data transfer returns garbage/CRC errors.
/// Card first, then this.
pub unsafe fn set_bus_width_4() {
    let h = unsafe { &*pac::HSMCI::ptr() };
    h.sdcr.write(|w| w.sdcsel().slota().sdcbus()._4());
}

/// Issue a CMD24 write of one 512-byte block. `block_address` is in the card's
/// native units -- block index for SDHC/SDXC, byte address for SDSC.
///
/// Polled, like `read_block`. The card pulls DAT0 low while it programs the
/// block, so this waits for NOTBUSY *after* the data as well as before the
/// command: returning while the card is still busy makes the next command
/// time out for reasons that look unrelated.
pub unsafe fn write_block(block_address: u32, src: &[u8; 512]) -> Result<(), CmdError> {
    let h = unsafe { &*pac::HSMCI::ptr() };

    let mut wait = 0u32;
    while h.sr.read().notbusy().bit_is_clear() {
        wait = wait.wrapping_add(1);
        if wait == 10_000_000 {
            debug!("write_block: notbusy timeout, sr={:#010x}", h.sr.read().bits());
            reset();
            return Err(CmdError::ResponseTimeout);
        }
    }

    h.dma.write(|w| unsafe { w.bits(0) });
    h.blkr
        .write(|w| unsafe { w.bcnt().bits(1).blklen().bits(512) });
    h.argr.write(|w| unsafe { w.bits(block_address) });
    h.cmdr.write_with_zero(|w| unsafe {
        w.cmdnb().bits(24)
            .rsptyp()._48_bit()
            .maxlat().set_bit()
            .opdcmd().pushpull()
            .trcmd().start_data()
            .trdir().write()
            .trtyp().single()
    });

    let mut wait = 0u32;
    loop {
        let sr = h.sr.read();
        if sr.cmdrdy().bit_is_set() {
            if sr.rtoe().bit_is_set() {
                reset();
                return Err(CmdError::ResponseTimeout);
            }
            if sr.rcrce().bit_is_set() {
                reset();
                return Err(CmdError::CrcError);
            }
            if sr.rinde().bit_is_set() || sr.rdire().bit_is_set() || sr.rende().bit_is_set() {
                reset();
                return Err(CmdError::BadResponse);
            }
            break;
        }
        wait = wait.wrapping_add(1);
        if wait == 10_000_000 {
            debug!("write_block: cmdrdy never set, sr={:#010x}", sr.bits());
            reset();
            return Err(CmdError::ResponseTimeout);
        }
    }

    // 128 x 32-bit writes = 512 bytes.
    for (i, chunk) in src.chunks_exact(4).enumerate() {
        let mut wait = 0u32;
        loop {
            let sr = h.sr.read();
            if sr.txrdy().bit_is_set() {
                break;
            }
            let bits = sr.bits();
            if bits & ((1 << 21) | (1 << 22) | (1 << 30) | (1 << 31)) != 0 {
                debug!("write_block: data error at word {}, sr={:#010x}", i, bits);
                reset();
                return Err(CmdError::CrcError);
            }
            wait = wait.wrapping_add(1);
            if wait == 10_000_000 {
                debug!("write_block: txrdy timeout at word {}, sr={:#010x}", i, bits);
                reset();
                return Err(CmdError::ResponseTimeout);
            }
        }
        let w = (chunk[0] as u32)
            | ((chunk[1] as u32) << 8)
            | ((chunk[2] as u32) << 16)
            | ((chunk[3] as u32) << 24);
        h.tdr.write_with_zero(|wr| unsafe { wr.bits(w) });
    }

    // Data sent; wait for the block to be flushed and then for the card to
    // finish programming it.
    let mut wait = 0u32;
    while h.sr.read().blke().bit_is_clear() {
        wait = wait.wrapping_add(1);
        if wait == 10_000_000 {
            debug!("write_block: blke timeout, sr={:#010x}", h.sr.read().bits());
            reset();
            return Err(CmdError::ResponseTimeout);
        }
    }
    let mut wait = 0u32;
    while h.sr.read().notbusy().bit_is_clear() {
        wait = wait.wrapping_add(1);
        if wait == 50_000_000 {
            debug!("write_block: card busy after program, sr={:#010x}", h.sr.read().bits());
            reset();
            return Err(CmdError::ResponseTimeout);
        }
    }

    let bits = h.sr.read().bits();
    if bits & ((1 << 21) | (1 << 22) | (1 << 30) | (1 << 31)) != 0 {
        debug!("write_block: data error at end, sr={:#010x}", bits);
        reset();
        return Err(CmdError::CrcError);
    }
    Ok(())
}

/// A CMD25 WRITE_MULTIPLE_BLOCK in progress. Created by `write_multi_begin`,
/// fed one 512-byte block at a time with `feed`, closed with `end`.
///
/// This is the streaming shape the ATA write path needs: the blocks arrive
/// from the host one sector at a time, so the whole transfer never sits in
/// RAM at once. Per block the card only sees data + CRC status; the command,
/// response and DAT0 program-busy wait that CMD24 pays on *every* block are
/// paid once per command instead, and the card pipelines programming of
/// block N against reception of block N+1.
///
/// The card stays in receive-data state until CMD12, so `end` must be called
/// on every path, including error paths, or the next command collides with
/// it. Dropping the guard without `end` resets the controller, which is the
/// blunt version of the same thing.
pub struct MultiWrite {
    remaining: u32,
    fed: u32,
    /// `abandon` already ran: the controller was reset and the card
    /// stopped, so `end` has nothing left to do.
    aborted: bool,
}

/// Open a CMD25 write of `count` blocks starting at `block_address` (card
/// native units, like `write_block`). Nothing is sent to the card yet beyond
/// the command itself.
pub unsafe fn write_multi_begin(block_address: u32, count: u32) -> Result<MultiWrite, CmdError> {
    let h = unsafe { &*pac::HSMCI::ptr() };
    if count == 0 || count > 0xFFFF {
        return Err(CmdError::BadResponse);
    }

    let mut wait = 0u32;
    while h.sr.read().notbusy().bit_is_clear() {
        wait = wait.wrapping_add(1);
        if wait == 10_000_000 {
            debug!("write_multi: notbusy timeout, sr={:#010x}", h.sr.read().bits());
            reset();
            return Err(CmdError::ResponseTimeout);
        }
    }

    h.dma.write(|w| unsafe { w.bits(0) });
    h.blkr
        .write(|w| unsafe { w.bcnt().bits(count as u16).blklen().bits(512) });
    h.argr.write(|w| unsafe { w.bits(block_address) });
    h.cmdr.write_with_zero(|w| unsafe {
        w.cmdnb().bits(25)
            .rsptyp()._48_bit()
            .maxlat().set_bit()
            .opdcmd().pushpull()
            .trcmd().start_data()
            .trdir().write()
            .trtyp().multiple()
    });

    let mut wait = 0u32;
    loop {
        let sr = h.sr.read();
        if sr.cmdrdy().bit_is_set() {
            if sr.rtoe().bit_is_set() {
                reset();
                return Err(CmdError::ResponseTimeout);
            }
            if sr.rcrce().bit_is_set() {
                reset();
                return Err(CmdError::CrcError);
            }
            if sr.rinde().bit_is_set() || sr.rdire().bit_is_set() || sr.rende().bit_is_set() {
                reset();
                return Err(CmdError::BadResponse);
            }
            break;
        }
        wait = wait.wrapping_add(1);
        if wait == 10_000_000 {
            debug!("write_multi: cmdrdy never set, sr={:#010x}", sr.bits());
            reset();
            return Err(CmdError::ResponseTimeout);
        }
    }
    Ok(MultiWrite { remaining: count, fed: 0, aborted: false })
}

impl MultiWrite {
    /// Push the next 512-byte block. Blocks while the card is still busy
    /// programming the previous one -- the controller holds TXRDY off until
    /// it has seen the CRC status token and DAT0 release -- which is where
    /// a slow card's program time shows up, and the only place it does.
    pub unsafe fn feed(&mut self, src: &[u8; 512]) -> Result<(), CmdError> {
        let h = unsafe { &*pac::HSMCI::ptr() };
        if self.remaining == 0 {
            return Err(CmdError::BadResponse);
        }
        for (i, chunk) in src.chunks_exact(4).enumerate() {
            let mut wait = 0u32;
            loop {
                let sr = h.sr.read();
                if sr.txrdy().bit_is_set() {
                    break;
                }
                let bits = sr.bits();
                if bits & ((1 << 21) | (1 << 22) | (1 << 30) | (1 << 31)) != 0 {
                    debug!("write_multi: data error at block {} word {}, sr={:#010x}",
                           self.fed, i, bits);
                    self.abandon();
                    return Err(CmdError::CrcError);
                }
                wait = wait.wrapping_add(1);
                // 50 M, not 10 M: the first word of a block waits out the
                // card's program time for the previous one.
                if wait == 50_000_000 {
                    debug!("write_multi: txrdy timeout at block {} word {}, sr={:#010x}",
                           self.fed, i, bits);
                    self.abandon();
                    return Err(CmdError::ResponseTimeout);
                }
            }
            let w = (chunk[0] as u32)
                | ((chunk[1] as u32) << 8)
                | ((chunk[2] as u32) << 16)
                | ((chunk[3] as u32) << 24);
            h.tdr.write_with_zero(|wr| unsafe { wr.bits(w) });
        }
        self.remaining -= 1;
        self.fed += 1;
        Ok(())
    }

    /// Close the transfer: CMD12 STOP_TRANSMISSION, then wait for the card
    /// to finish programming the last block. Valid after any number of
    /// `feed`s -- short transfers (host gave up) are stopped the same way,
    /// the card just programs what it got.
    pub unsafe fn end(mut self) -> Result<(), CmdError> {
        let h = unsafe { &*pac::HSMCI::ptr() };
        if self.aborted {
            return Err(CmdError::CrcError);
        }
        let complete = self.remaining == 0;
        self.remaining = 0;

        // Let the last block leave the controller before stopping. Only
        // meaningful when the block count was reached; on an aborted
        // transfer the controller is still waiting for data we will not
        // send, and CMD12 with STOP_DATA is what ends that.
        if complete {
            let mut wait = 0u32;
            while h.sr.read().xfrdone().bit_is_clear() {
                wait = wait.wrapping_add(1);
                if wait == 50_000_000 {
                    debug!("write_multi: xfrdone timeout after {} blocks, sr={:#010x}",
                           self.fed, h.sr.read().bits());
                    reset();
                    return Err(CmdError::ResponseTimeout);
                }
            }
        }

        h.cmdr.write_with_zero(|w| unsafe {
            w.cmdnb().bits(12)
                .rsptyp().r1b()
                .maxlat().set_bit()
                .opdcmd().pushpull()
                .trcmd().stop_data()
        });
        let mut wait = 0u32;
        while h.sr.read().cmdrdy().bit_is_clear() {
            wait = wait.wrapping_add(1);
            if wait == 10_000_000 {
                debug!("write_multi: CMD12 cmdrdy timeout, sr={:#010x}", h.sr.read().bits());
                reset();
                return Err(CmdError::ResponseTimeout);
            }
        }
        // R1b: the card holds DAT0 low while it programs the final block.
        let mut wait = 0u32;
        while h.sr.read().notbusy().bit_is_clear() {
            wait = wait.wrapping_add(1);
            if wait == 50_000_000 {
                debug!("write_multi: card busy after CMD12, sr={:#010x}", h.sr.read().bits());
                reset();
                return Err(CmdError::ResponseTimeout);
            }
        }

        let bits = h.sr.read().bits();
        if !complete {
            // We stopped early on purpose; the controller's unfinished-block
            // bookkeeping is not an error of the card's.
            return Ok(());
        }
        if bits & ((1 << 21) | (1 << 22) | (1 << 30) | (1 << 31)) != 0 {
            debug!("write_multi: data error at end, sr={:#010x}", bits);
            reset();
            return Err(CmdError::CrcError);
        }
        Ok(())
    }

    /// Error path: reset the controller, then a best-effort CMD12 so the
    /// card leaves receive-data state too -- a reset alone only fixes our
    /// side, and the card would otherwise treat the next command as more
    /// of this one. Errors here are not reported; the caller already has
    /// one.
    fn abandon(&mut self) {
        self.remaining = 0;
        self.aborted = true;
        let h = unsafe { &*pac::HSMCI::ptr() };
        unsafe {
            reset();
            h.cmdr.write_with_zero(|w| {
                w.cmdnb().bits(12)
                    .rsptyp().r1b()
                    .maxlat().set_bit()
                    .opdcmd().pushpull()
                    .trcmd().no_data()
            });
        }
        let mut wait = 0u32;
        while h.sr.read().cmdrdy().bit_is_clear() {
            wait = wait.wrapping_add(1);
            if wait == 10_000_000 {
                break;
            }
        }
        let mut wait = 0u32;
        while h.sr.read().notbusy().bit_is_clear() {
            wait = wait.wrapping_add(1);
            if wait == 50_000_000 {
                break;
            }
        }
        unsafe { reset() };
    }
}

impl Drop for MultiWrite {
    fn drop(&mut self) {
        if self.remaining != 0 {
            self.abandon();
        }
    }
}

/// Issue a CMD17 read of one 512-byte block. `block_address` is in the card's
/// native units — block index for SDHC/SDXC, byte address for SDSC.
pub unsafe fn read_block(block_address: u32, dst: &mut [u8; 512]) -> Result<(), CmdError> {
    let h = unsafe { &*pac::HSMCI::ptr() };

    // Wait for NOTBUSY first — the card may still be holding DAT0 low
    // from a previous command's busy phase, which would block data
    // reception even though CMDRDY came back fine.
    let mut wait = 0u32;
    while h.sr.read().notbusy().bit_is_clear() {
        wait = wait.wrapping_add(1);
        if wait == 10_000_000 {
            debug!("read_block: notbusy timeout, sr={:#010x}", h.sr.read().bits());
            reset();
            return Err(CmdError::ResponseTimeout);
        }
    }

    // Ensure DMA is off — without this the bytes would be routed to the
    // (uninitialized) DMA channel rather than the FIFO.
    h.dma.write(|w| unsafe { w.bits(0) });

    h.blkr
        .write(|w| unsafe { w.bcnt().bits(1).blklen().bits(512) });
    h.argr.write(|w| unsafe { w.bits(block_address) });
    h.cmdr.write_with_zero(|w| unsafe {
        w.cmdnb()
            .bits(17)
            .rsptyp()
            ._48_bit()
            .maxlat()
            .set_bit()
            .opdcmd()
            .pushpull()
            .trcmd()
            .start_data()
            .trdir()
            .read()
            .trtyp()
            .single()
    });

    // Wait for command response. Bound the spin so a hang doesn't lock us up
    // forever — at 400 kHz, the response is ~64 SD clocks = ~160 µs; many
    // millions of MCU cycles is plenty of headroom.
    let mut wait = 0u32;
    loop {
        let sr = h.sr.read();
        if sr.cmdrdy().bit_is_set() {
            if sr.rtoe().bit_is_set() {
                reset();
                return Err(CmdError::ResponseTimeout);
            }
            if sr.rcrce().bit_is_set() {
                reset();
                return Err(CmdError::CrcError);
            }
            if sr.rinde().bit_is_set() || sr.rdire().bit_is_set() || sr.rende().bit_is_set() {
                reset();
                return Err(CmdError::BadResponse);
            }
            break;
        }
        wait = wait.wrapping_add(1);
        if wait == 10_000_000 {
            debug!("read_block: cmdrdy never set, sr={:#010x}", sr.bits());
            reset();
            return Err(CmdError::ResponseTimeout);
        }
    }

    // 128 × 32-bit reads = 512 bytes.
    for (i, chunk) in dst.chunks_exact_mut(4).enumerate() {
        let mut wait = 0u32;
        loop {
            let sr = h.sr.read();
            if sr.rxrdy().bit_is_set() {
                break;
            }
            // Watch for data-path errors that ASF treats as fatal:
            // UNRE / OVRE / DTOE / DCRCE.
            let bits = sr.bits();
            if bits & ((1 << 21) | (1 << 22) | (1 << 30) | (1 << 31)) != 0 {
                debug!("read_block: data error at word {}, sr={:#010x}", i, bits);
                reset();
                return Err(CmdError::CrcError);
            }
            wait = wait.wrapping_add(1);
            if wait == 10_000_000 {
                debug!("read_block: rxrdy timeout at word {}, sr={:#010x}", i, bits);
                reset();
                return Err(CmdError::ResponseTimeout);
            }
        }
        let w = h.rdr.read().bits();
        chunk[0] = w as u8;
        chunk[1] = (w >> 8) as u8;
        chunk[2] = (w >> 16) as u8;
        chunk[3] = (w >> 24) as u8;
    }

    let mut wait = 0u32;
    while h.sr.read().xfrdone().bit_is_clear() {
        wait = wait.wrapping_add(1);
        if wait == 10_000_000 {
            debug!("read_block: xfrdone timeout, sr={:#010x}", h.sr.read().bits());
            reset();
            return Err(CmdError::ResponseTimeout);
        }
    }

    Ok(())
}

// ---- DMA-mode read --------------------------------------------------
//
// `read_block_dma_arm` issues a CMD17 + sets up DMAC channel 1 to move
// 512 bytes from HSMCI's RDR to `dst`, then returns immediately. The
// caller polls `read_block_dma_busy()` and finishes with
// `read_block_dma_finish()` to check status. Compared to the polled
// `read_block` above, the CPU is free during the actual SD card
// transfer (~50 µs at default clocks) — letting us overlap an SD read
// with a DMAC-driven FPGA push from the IDE data path.
//
// Channel 1 is dedicated to HSMCI; channel 0 is owned by `dmac` for
// MCU→FPGA pushes. The two never overlap on hardware.
//
// Hardware-handshake source ID for HSMCI on SAM3U4E is 0 per the
// datasheet's "DMA Controller (DMAC) Hardware Connection" table.
const DMAC_SRC_PER_HSMCI: u32 = 0;

pub unsafe fn read_block_dma_arm(
    block_address: u32,
    dst: &mut [u8; 512],
) -> Result<(), CmdError> {
    let h = unsafe { &*pac::HSMCI::ptr() };
    let dmac = unsafe { &*pac::DMAC::ptr() };

    // Wait for the previous busy phase to finish before issuing more.
    let mut wait = 0u32;
    while h.sr.read().notbusy().bit_is_clear() {
        wait = wait.wrapping_add(1);
        if wait == 10_000_000 {
            reset();
            return Err(CmdError::ResponseTimeout);
        }
    }

    // Make sure DMAC channel 1 isn't still serving a previous read.
    while dmac.chsr.read().ena1().bit_is_set() {}

    // Enable DMA mode on HSMCI. CHKSIZE=4 data per DMAC request trades
    // request overhead for FIFO underrun tolerance — matches the SAM ASF
    // default for SD reads. OFFSET stays 0; it is a write-buffer offset and
    // is meaningless for a read.
    //
    // Use the typed fields: DMAEN is bit 8, not bit 0. Writing a raw
    // `0x01 | (1 << 4)` here set OFFSET=1 and left DMAEN clear, so HSMCI
    // never entered handshaking mode and the armed DMAC channel waited
    // forever for a request that could not arrive.
    h.dma.write(|w| w.dmaen().set_bit().chksize()._4());

    // Block size = 512 bytes, count = 1.
    h.blkr
        .write(|w| unsafe { w.bcnt().bits(1).blklen().bits(512) });
    h.argr.write(|w| unsafe { w.bits(block_address) });

    // Configure DMAC channel 1: hardware-handshake source = HSMCI RDR,
    // memory destination = `dst`, transfer count = 128 words (512 B),
    // 32-bit width on both sides. Source is fixed (RDR), dest is
    // incrementing.
    let rdr_addr = (&h.rdr) as *const _ as u32;
    dmac.saddr1.write(|w| unsafe { w.bits(rdr_addr) });
    dmac.daddr1.write(|w| unsafe { w.bits(dst.as_mut_ptr() as u32) });

    let ctrla = (128u32 & 0xFFFF)
        | (1 << 16)         // SCSIZE = 4 transfers (matches HSMCI CHKSIZE)
        | (1 << 20)         // DCSIZE = 4 transfers
        | (2 << 24)         // SRC_WIDTH = word
        | (2 << 28);        // DST_WIDTH = word
    dmac.ctrla1.write(|w| unsafe { w.bits(ctrla) });

    let ctrlb = (1 << 16)        // SRC_DSCR (single, no descriptor list)
              | (1 << 20)        // DST_DSCR (single)
              | (2 << 21)        // FC = per-to-mem
              | (2 << 24)        // SRC_INCR = fixed (RDR)
              | (0 << 28);       // DST_INCR = incrementing
    dmac.ctrlb1.write(|w| unsafe { w.bits(ctrlb) });

    // CFG: SRC_PER = HSMCI ID (0), hardware handshaking on source side,
    // memory dest needs no peripheral handshake.
    let cfg = DMAC_SRC_PER_HSMCI
            | (1 << 9);         // SRC_H2SEL = HW
    dmac.cfg1.write(|w| unsafe { w.bits(cfg) });

    // Arm channel 1.
    dmac.cher.write_with_zero(|w| unsafe { w.bits(1 << 1) });

    // Issue CMD17 READ_SINGLE_BLOCK. HSMCI starts its data state
    // machine; the DMAC will pull from RDR as data lands.
    h.cmdr.write_with_zero(|w| unsafe {
        w.cmdnb().bits(17)
            .rsptyp()._48_bit()
            .maxlat().set_bit()
            .opdcmd().pushpull()
            .trcmd().start_data()
            .trdir().read()
            .trtyp().single()
    });

    Ok(())
}

/// True while the DMA-mode read is still in flight.
pub fn read_block_dma_busy() -> bool {
    let dmac = unsafe { &*pac::DMAC::ptr() };
    dmac.chsr.read().ena1().bit_is_set()
}

/// Wait for the in-flight DMA-mode read to finish, then check HSMCI's
/// status word for transfer errors. Disables HSMCI DMA mode on the
/// way out.
pub unsafe fn read_block_dma_finish() -> Result<(), CmdError> {
    let h = unsafe { &*pac::HSMCI::ptr() };
    let dmac = unsafe { &*pac::DMAC::ptr() };

    // Wait for DMAC done.
    let mut wait = 0u32;
    while dmac.chsr.read().ena1().bit_is_set() {
        wait = wait.wrapping_add(1);
        if wait == 10_000_000 {
            reset();
            h.dma.write(|w| unsafe { w.bits(0) });
            return Err(CmdError::ResponseTimeout);
        }
    }

    // Wait for HSMCI to confirm the data phase is complete.
    let mut wait = 0u32;
    while h.sr.read().xfrdone().bit_is_clear() {
        wait = wait.wrapping_add(1);
        if wait == 10_000_000 {
            reset();
            h.dma.write(|w| unsafe { w.bits(0) });
            return Err(CmdError::ResponseTimeout);
        }
    }

    // Check for data-path errors flagged in HSMCI.SR (UNRE / OVRE /
    // DTOE / DCRCE bits 21/22/30/31).
    let bits = h.sr.read().bits();
    h.dma.write(|w| unsafe { w.bits(0) });
    if bits & ((1 << 21) | (1 << 22) | (1 << 30) | (1 << 31)) != 0 {
        debug!("read_block_dma: data error sr={:#010x}", bits);
        reset();
        return Err(CmdError::CrcError);
    }

    Ok(())
}

/// Largest multi-block read we accept in one call.
///
/// Bounded by the DMAC's 16-bit BTSIZE: 65535 word transfers = 262140 bytes,
/// i.e. 511 blocks. 128 keeps a comfortable margin and is far more than any
/// single CD_READ needs.
pub const MAX_BLOCKS_PER_READ: usize = 128;

/// CMD18 READ_MULTIPLE_BLOCK straight into `dst` over DMAC channel 1.
///
/// One command and one DMAC transfer cover the whole run, which is the point:
/// per-block CMD17 overhead — not the bus — is what caps the single-block
/// paths at roughly 2 MB/s.
///
/// `dst.len()` must be a non-zero multiple of 512 and at most
/// `MAX_BLOCKS_PER_READ` blocks. Ends with CMD12 STOP_TRANSMISSION, which the
/// card requires to leave the multiple-read state.
pub unsafe fn read_blocks_dma(block_address: u32, dst: &mut [u8]) -> Result<(), CmdError> {
    let h = unsafe { &*pac::HSMCI::ptr() };
    let dmac = unsafe { &*pac::DMAC::ptr() };

    if dst.is_empty() || dst.len() % 512 != 0 || dst.len() / 512 > MAX_BLOCKS_PER_READ {
        return Err(CmdError::ResponseTimeout);
    }
    let nblocks = (dst.len() / 512) as u16;

    let mut wait = 0u32;
    while h.sr.read().notbusy().bit_is_clear() {
        wait = wait.wrapping_add(1);
        if wait == 10_000_000 {
            reset();
            return Err(CmdError::ResponseTimeout);
        }
    }
    while dmac.chsr.read().ena1().bit_is_set() {}

    h.dma.write(|w| w.dmaen().set_bit().chksize()._4());
    h.blkr
        .write(|w| unsafe { w.bcnt().bits(nblocks).blklen().bits(512) });
    h.argr.write(|w| unsafe { w.bits(block_address) });

    let rdr_addr = (&h.rdr) as *const _ as u32;
    dmac.saddr1.write(|w| unsafe { w.bits(rdr_addr) });
    dmac.daddr1.write(|w| unsafe { w.bits(dst.as_mut_ptr() as u32) });

    // BTSIZE counts SRC_WIDTH-sized transfers: 128 words per 512-byte block.
    let ctrla = ((nblocks as u32 * 128) & 0xFFFF)
        | (1 << 16)         // SCSIZE = 4 transfers (matches HSMCI CHKSIZE)
        | (1 << 20)         // DCSIZE = 4 transfers
        | (2 << 24)         // SRC_WIDTH = word
        | (2 << 28);        // DST_WIDTH = word
    dmac.ctrla1.write(|w| unsafe { w.bits(ctrla) });

    let ctrlb = (1 << 16)        // SRC_DSCR (single, no descriptor list)
              | (1 << 20)        // DST_DSCR (single)
              | (2 << 21)        // FC = per-to-mem
              | (2 << 24)        // SRC_INCR = fixed (RDR)
              | (0 << 28);       // DST_INCR = incrementing
    dmac.ctrlb1.write(|w| unsafe { w.bits(ctrlb) });

    dmac.cfg1.write(|w| unsafe { w.bits(DMAC_SRC_PER_HSMCI | (1 << 9)) });
    dmac.cher.write_with_zero(|w| unsafe { w.bits(1 << 1) });

    h.cmdr.write_with_zero(|w| unsafe {
        w.cmdnb().bits(18)
            .rsptyp()._48_bit()
            .maxlat().set_bit()
            .opdcmd().pushpull()
            .trcmd().start_data()
            .trdir().read()
            .trtyp().multiple()
    });

    let mut wait = 0u32;
    while dmac.chsr.read().ena1().bit_is_set() {
        wait = wait.wrapping_add(1);
        if wait == 50_000_000 {
            reset();
            h.dma.write(|w| unsafe { w.bits(0) });
            return Err(CmdError::ResponseTimeout);
        }
    }

    let mut wait = 0u32;
    while h.sr.read().xfrdone().bit_is_clear() {
        wait = wait.wrapping_add(1);
        if wait == 50_000_000 {
            reset();
            h.dma.write(|w| unsafe { w.bits(0) });
            return Err(CmdError::ResponseTimeout);
        }
    }

    // CMD12 STOP_TRANSMISSION — the card stays in read-multiple state until
    // it sees this, and the next command would collide with streaming data.
    h.cmdr.write_with_zero(|w| unsafe {
        w.cmdnb().bits(12)
            .rsptyp().r1b()
            .maxlat().set_bit()
            .opdcmd().pushpull()
            .trcmd().stop_data()
    });
    let mut wait = 0u32;
    while h.sr.read().cmdrdy().bit_is_clear() {
        wait = wait.wrapping_add(1);
        if wait == 10_000_000 {
            reset();
            h.dma.write(|w| unsafe { w.bits(0) });
            return Err(CmdError::ResponseTimeout);
        }
    }

    let bits = h.sr.read().bits();
    h.dma.write(|w| unsafe { w.bits(0) });
    if bits & ((1 << 21) | (1 << 22) | (1 << 30) | (1 << 31)) != 0 {
        debug!("read_blocks_dma: data error sr={:#010x}", bits);
        reset();
        return Err(CmdError::CrcError);
    }

    Ok(())
}

/// Convenience: arm + finish back-to-back. Equivalent throughput to the
/// polled `read_block` but the CPU is free during the actual transfer
/// and the producer-side data path is in DMA mode (no per-word CPU
/// copy from HSMCI.RDR).
pub unsafe fn read_block_dma(
    block_address: u32,
    dst: &mut [u8; 512],
) -> Result<(), CmdError> {
    read_block_dma_arm(block_address, dst)?;
    read_block_dma_finish()
}
