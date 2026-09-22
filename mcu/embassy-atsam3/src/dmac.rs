//! Minimal SAM3U DMAC driver — used to push the CD_READ payload from
//! the MCU's RAM buffer to the SMC-mapped FPGA write FIFO at full SMC
//! speed without the per-word `write_volatile` CPU loop. Without DMA
//! the MCU's push rate jitters with embassy task scheduling and the
//! first CD_READ DMA underruns intermittently (8/13/14 KB delivered
//! variability) — using the DMAC as the producer keeps the FIFO full
//! and the host's DMA channel can drain at its full ~16 MB/s without
//! seeing DMARQ blip low.
//!
//! Scope intentionally narrow: blocking single-channel transfers,
//! halfword (16-bit) writes only, source-incrementing / dest-fixed.
//! That's all the GD-ROM data path needs.

use crate::pac::{self, DMAC};

const PMC_DMAC_PID: u8 = 28;

/// Bring up the DMAC peripheral. Caller must have enabled the relevant
/// PMC clocks for SMC + SDRAM (already done by `init`/`smc::init`).
/// After this returns, channel 0 is ready to be programmed.
pub unsafe fn init() {
    let pac = unsafe { pac::Peripherals::steal() };

    // PMC: enable the DMAC peripheral clock (PID 28).
    pac.PMC
        .pmc_pcer0
        .write_with_zero(|w| unsafe { w.bits(1 << PMC_DMAC_PID) });

    let dmac = &pac.DMAC;
    // Enable the DMAC controller. ARB_CFG = fixed priority is fine for
    // single-channel use.
    dmac.en.write(|w| w.enable().set_bit());
    // Disable channel 0 in case it was running from a previous boot.
    dmac.chdr.write_with_zero(|w| unsafe { w.bits(0x1) });
}

/// True while DMAC channel 0 is mid-transfer.
pub fn busy() -> bool {
    let dmac = unsafe { &*DMAC::ptr() };
    dmac.chsr.read().ena0().bit_is_set()
}

/// Block until channel 0 finishes the in-flight transfer (or returns
/// immediately if there isn't one).
pub fn wait_done() {
    let dmac = unsafe { &*DMAC::ptr() };
    while dmac.chsr.read().ena0().bit_is_set() {}
}

/// Arm channel 0 to push `len` halfwords (16-bit) from `src` to `dst`,
/// then **return immediately** without blocking. `dst` is a fixed
/// (peripheral) address; `src` is incremented per transfer. Caller
/// must check completion with [`busy`] / [`wait_done`] before re-
/// arming or before reusing the source buffer.
///
/// Caller must guarantee `src` has `len` halfwords readable and that
/// `dst` is a halfword-aligned, valid SMC mapping. We don't bother
/// with cache flushes because the SAM3U4E doesn't have a data cache.
pub unsafe fn push_halfwords(src: *const u16, dst: *mut u16, len: u32) {
    if len == 0 {
        return;
    }
    let dmac = unsafe { &*DMAC::ptr() };

    // Make sure the channel is idle before re-arming. Callers using
    // the streaming A/B-buffer pattern should already wait_done() on
    // the previous transfer before producing a new one, but belt and
    // braces here too.
    while dmac.chsr.read().ena0().bit_is_set() {}

    // Source / destination physical addresses. The SAM3U DMAC looks at
    // the same address space as the CPU, so RAM and SMC pointers are
    // both fine here.
    dmac.saddr0.write(|w| unsafe { w.bits(src as u32) });
    dmac.daddr0.write(|w| unsafe { w.bits(dst as u32) });

    // CTRLA: BTSIZE = transfer count (halfwords), SRC/DST_WIDTH = 1 (halfword),
    // chunk sizes = 1 (no AHB burst — SMC doesn't support it).
    //   bits 0..15  : BTSIZE
    //   bits 16..18 : SCSIZE
    //   bits 20..22 : DCSIZE
    //   bits 24..25 : SRC_WIDTH (1 = halfword)
    //   bits 28..29 : DST_WIDTH (1 = halfword)
    let ctrla = (len & 0xFFFF)
        | (0 << 16)         // SCSIZE = 1 transfer
        | (0 << 20)         // DCSIZE = 1 transfer
        | (1 << 24)         // SRC_WIDTH = halfword
        | (1 << 28);        // DST_WIDTH = halfword
    dmac.ctrla0.write(|w| unsafe { w.bits(ctrla) });

    // CTRLB: FC = mem-to-mem (we're treating SMC as memory),
    // SRC_DSCR = 1 (single, no descriptor chain),
    // DST_DSCR = 1 (single),
    // SRC_INCR = 0 (increment),
    // DST_INCR = 2 (fixed).
    //   bits 0     : SRC_DSCR
    //   bits 4     : DST_DSCR
    //   bits 21..23: FC (000 = mem-to-mem)
    //   bits 24..25: SRC_INCR (00 = increment)
    //   bits 28..29: DST_INCR (10 = fixed)
    let ctrlb = (1 << 16)        // SRC_DSCR (single)
              | (1 << 20)        // DST_DSCR (single)
              | (0 << 21)        // FC = mem2mem
              | (0 << 24)        // SRC_INCR = increment
              | (2 << 28);       // DST_INCR = fixed
    dmac.ctrlb0.write(|w| unsafe { w.bits(ctrlb) });

    // CFG: AHB layer for source = 0 (M-AHB), dest = 0 (M-AHB).
    // No peripheral handshake. Source-on-done = 1.
    dmac.cfg0.write(|w| unsafe { w.bits(0) });

    // Enable channel 0 and return; caller polls or wait_done()s.
    dmac.cher.write_with_zero(|w| unsafe { w.bits(0x1) });
}

/// Channel 2: the CDDA sample feed. Same shape as channel 0 (halfwords,
/// source incrementing, destination fixed), kept on its own channel so an
/// interrupt-context push cannot collide with a CD_READ push on channel 0.
pub fn busy_ch2() -> bool {
    let dmac = unsafe { &*DMAC::ptr() };
    dmac.chsr.read().ena2().bit_is_set()
}

/// Block until channel 2 finishes the in-flight transfer.
pub fn wait_done_ch2() {
    let dmac = unsafe { &*DMAC::ptr() };
    while dmac.chsr.read().ena2().bit_is_set() {}
}

/// Channel-2 twin of [`push_halfwords`]; see there for the register
/// layout and the caller's obligations.
pub unsafe fn push_halfwords_ch2(src: *const u16, dst: *mut u16, len: u32) {
    if len == 0 {
        return;
    }
    let dmac = unsafe { &*DMAC::ptr() };
    while dmac.chsr.read().ena2().bit_is_set() {}
    dmac.saddr2.write(|w| unsafe { w.bits(src as u32) });
    dmac.daddr2.write(|w| unsafe { w.bits(dst as u32) });
    let ctrla = (len & 0xFFFF) | (1 << 24) | (1 << 28);
    dmac.ctrla2.write(|w| unsafe { w.bits(ctrla) });
    let ctrlb = (1 << 16) | (1 << 20) | (2 << 28);
    dmac.ctrlb2.write(|w| unsafe { w.bits(ctrlb) });
    dmac.cfg2.write(|w| unsafe { w.bits(0) });
    dmac.cher.write_with_zero(|w| unsafe { w.bits(0x4) });
}
