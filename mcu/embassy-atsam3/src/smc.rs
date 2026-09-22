use crate::pac;

/// EBI base address for chip select 3 — where the FPGA register window is mapped.
pub const FPGA_BASE: usize = 0x6300_0000;

/// Returns a 16-bit pointer to the FPGA register window. Each `+1` step on the
/// pointer advances `MCU_ADDR[8:1]` inside the FPGA by 1, i.e. one 16-bit word.
#[inline]
pub fn fpga_window() -> *mut u16 {
    FPGA_BASE as *mut u16
}

/// SMC pin set used by the FPGA bus. Pin assignments come from the SAM3U4E
/// peripheral muxing; see datasheet "PIO Controller Multiplexing of Peripheral
/// A and B" for the SAM3U4E.
///
///   - NCS3   : PC17 peripheral A
///   - A0/NBS0: PB7  peripheral B
///   - A1     : PB8  peripheral B
///   - A2..A8 : PC0..PC6 peripheral A
///   - NRD    : PB19 peripheral A
///   - NWE    : PB23 peripheral A
///   - NBS1   : PC15 peripheral A
///   - D0..D7 : PB9..PB16 peripheral A
///   - D8..D14: PB25..PB31 peripheral A
///   - D15    : PB6  peripheral B
const PIOB_MASK: u32 = (1 << 6) | (1 << 7) | (1 << 8)
    | (1 << 9) | (1 << 10) | (1 << 11) | (1 << 12) | (1 << 13) | (1 << 14) | (1 << 15) | (1 << 16)
    | (1 << 19) | (1 << 23)
    | (1 << 25) | (1 << 26) | (1 << 27) | (1 << 28) | (1 << 29) | (1 << 30) | (1 << 31);

/// Subset of `PIOB_MASK` whose pins use peripheral B (PB6, PB7, PB8). The rest
/// use peripheral A.
const PIOB_B_MASK: u32 = (1 << 6) | (1 << 7) | (1 << 8);

const PIOC_MASK: u32 = 0x7F | (1 << 15) | (1 << 17);

/// Bring up SMC chip select 3 with conservative timings suitable for a 48 MHz
/// FPGA core clock and a 96 MHz MCU MCK. Caller must have enabled the SMC and
/// PIOB/PIOC peripheral clocks before calling.
pub unsafe fn init() {
    let pac = unsafe { pac::Peripherals::steal() };

    // Release PIO control to the peripheral mux on every SMC pin.
    pac.PIOB
        .pdr
        .write_with_zero(|w| unsafe { w.bits(PIOB_MASK) });
    pac.PIOC
        .pdr
        .write_with_zero(|w| unsafe { w.bits(PIOC_MASK) });

    // Peripheral A (ABSR=0) for everything except PB6/PB7/PB8.
    pac.PIOB.absr.modify(|r, w| unsafe {
        w.bits((r.bits() & !PIOB_MASK) | PIOB_B_MASK)
    });
    pac.PIOC
        .absr
        .modify(|r, w| unsafe { w.bits(r.bits() & !PIOC_MASK) });

    // Disable internal pull-ups — the FPGA drives the bus.
    pac.PIOB
        .pudr
        .write_with_zero(|w| unsafe { w.bits(PIOB_MASK) });
    pac.PIOC
        .pudr
        .write_with_zero(|w| unsafe { w.bits(PIOC_MASK) });

    let smc = &pac.SMC;

    // SETUP3: 1-cycle setup on NWE/NRD and on NCS for both directions.
    smc.setup3.write(|w| unsafe {
        w.nwe_setup()
            .bits(1)
            .ncs_wr_setup()
            .bits(1)
            .nrd_setup()
            .bits(1)
            .ncs_rd_setup()
            .bits(1)
    });

    // PULSE3: 8 MCK cycles (~83 ns at 96 MHz). FPGA core clock is 48 MHz
    // (~20.8 ns period), so the strobe spans ~4 FPGA edges — comfortable
    // for the 2-FF synchronizer. Tried 6 cycles (~3 FPGA edges) once and
    // SMC reads of the write FIFO came back garbled (CDB bytes shifted /
    // duplicated), so 8 is the floor. Earlier comment said "11.2896 MHz
    // core clock" — that's the audio xtal, not the FPGA core clock.
    smc.pulse3.write(|w| unsafe {
        w.nwe_pulse()
            .bits(8)
            .ncs_wr_pulse()
            .bits(8)
            .nrd_pulse()
            .bits(8)
            .ncs_rd_pulse()
            .bits(8)
    });

    // CYCLE3: 12 MCK cycles total per access (~125 ns), giving an
    // effective ~16 MB/s for 16-bit writes — matches the DC's GD-ROM
    // DMA pull rate in MWDMA Mode 2. CD_READ tail delivery improved
    // from ~14180/14336 (with the original 32-cycle config) to
    // ~14240/14336 here. Tried cycle=8 and cycle=10 to push faster but
    // SMC reads of the write FIFO came back garbled (CDB bytes shifted
    // / duplicated like `[13 00 13 00 00 00 00 00 0a 00 0a 00]`).
    smc.cycle3
        .write(|w| unsafe { w.nwe_cycle().bits(12).nrd_cycle().bits(12) });

    // MODE3: 16-bit data bus, NRD/NWE-controlled (defaults). EXNW disabled.
    smc.mode3.write(|w| {
        w.read_mode()
            .nrd_ctrl()
            .write_mode()
            .nwe_ctrl()
            .exnw_mode()
            .disabled()
            .dbw()
            .bit_16()
    });
}
