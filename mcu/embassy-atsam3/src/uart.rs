//! Minimal blocking UART0 driver — used as a "log of last resort"
//! that survives MCU resets so we can watch the BIOS↔device command
//! stream across DC power cycles (which kill SWD/RTT).
//!
//! Pins (per `docs/PCB.md`):
//!   - PA11 = URXD0 (peripheral A)
//!   - PA12 = UTXD0 (peripheral A)
//!
//! UART0 base = 0x400E0600. Peripheral ID = 8 (PMC PCER bit 8).

use core::fmt::{self, Write};

use crate::pac::{self, PIOA, UART};

const PMC_UART0_PID: u8 = 8;

/// Initialize UART0 at 115200 8-N-1 driving PA11/PA12 peripheral A.
/// Caller must have enabled PIOA's clock; this function enables UART0
/// in PMC PCER on the way through.
pub unsafe fn init(mck_hz: u32) {
    let pac = unsafe { pac::Peripherals::steal() };

    // PIOA: hand PA11/PA12 to peripheral A.
    pac.PIOA.pdr.write_with_zero(|w| unsafe {
        w.bits((1 << 11) | (1 << 12))
    });
    pac.PIOA.absr.modify(|r, w| unsafe {
        w.bits(r.bits() & !((1 << 11) | (1 << 12)))
    });
    // PA12 (TX) doesn't need a pull, PA11 (RX) we pull up to idle high.
    pac.PIOA.puer.write_with_zero(|w| unsafe { w.bits(1 << 11) });

    // PMC: enable the UART peripheral clock (PID 8).
    pac.PMC
        .pmc_pcer0
        .write_with_zero(|w| unsafe { w.bits(1 << PMC_UART0_PID) });

    let uart = &pac.UART;
    // Reset & disable.
    uart.cr.write_with_zero(|w| {
        w.rstrx().set_bit().rsttx().set_bit().rxdis().set_bit().txdis().set_bit()
    });
    // Mode: no parity, normal channel.
    uart.mr.write(|w| w.par().no());
    // Baud rate divisor: BRGR = MCK / (16 * baud). 96 MHz / (16 * 115200) ≈ 52.
    let cd = ((mck_hz + 8 * 115200) / (16 * 115200)) & 0xFFFF;
    uart.brgr.write(|w| unsafe { w.cd().bits(cd as u16) });
    // Enable RX + TX.
    uart.cr.write_with_zero(|w| w.rxen().set_bit().txen().set_bit());
}

/// Block until the TX holding register can accept another byte, then
/// write `b`. Called from any context (interrupt-safe — no locking).
#[inline]
pub fn write_byte(b: u8) {
    let uart = unsafe { &*UART::ptr() };
    while uart.sr.read().txrdy().bit_is_clear() {}
    unsafe {
        uart.thr.write_with_zero(|w| w.txchr().bits(b));
    }
}

/// Block until every queued byte has been physically clocked out.
/// Useful at panic time so the message reaches the host before we halt.
#[inline]
#[allow(dead_code)]
pub fn drain() {
    let uart = unsafe { &*UART::ptr() };
    while uart.sr.read().txempty().bit_is_clear() {}
}

/// `core::fmt::Write` adapter so we can use `write!` / `writeln!`.
pub struct Uart;

impl Write for Uart {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        for b in s.bytes() {
            // Insert CR before LF so screen/picocom render correctly.
            if b == b'\n' {
                write_byte(b'\r');
            }
            write_byte(b);
        }
        Ok(())
    }
}

/// Convenience: like `println!` but for our UART. Always succeeds
/// (panic-free) — discards the formatting result.
#[macro_export]
macro_rules! uart_println {
    ($($arg:tt)*) => {{
        use core::fmt::Write as _;
        let mut u = $crate::uart::Uart;
        let _ = writeln!(u, $($arg)*);
    }};
}
