#![no_std]

pub use atsam3u4e_pac as pac;
pub use embassy_hal_internal::{Peri, PeripheralType};

use clocks::PowerManagementController;
use cortex_m::peripheral::syst::{SystClkSource, self};
use cortex_m::peripheral::{NVIC, SYST};
use pac::{WDT, PMC};
use defmt::debug;

use crate::gpio::Pin;

pub mod config;

pub mod wdt;
pub mod systick;
pub mod rtt;
pub mod clocks;
pub mod timer;
pub mod efc;
pub mod gpio;
pub mod smc;
pub mod dmac;
pub mod hsmci;
pub mod uart;


/// Macro to bind interrupts to handlers.
///
/// This defines the right interrupt handlers, and creates a unit struct (like `struct Irqs;`)
/// and implements the right [`Binding`]s for it. You can pass this struct to drivers to
/// prove at compile-time that the right interrupts have been bound.
// developer note: this macro can't be in `embassy-hal-internal` due to the use of `$crate`.
#[macro_export]
macro_rules! bind_interrupts {
    ($vis:vis struct $name:ident { $($irq:ident => $($handler:ty),*;)* }) => {
        #[derive(Copy, Clone)]
        $vis struct $name;

        $(
            #[allow(non_snake_case)]
            #[no_mangle]
            unsafe extern "C" fn $irq() {
                $(
                    <$handler as $crate::interrupt::typelevel::Handler<$crate::interrupt::typelevel::$irq>>::on_interrupt();
                )*
            }

            $(
                unsafe impl $crate::interrupt::typelevel::Binding<$crate::interrupt::typelevel::$irq, $handler> for $name {}
            )*
        )*
    };
}

// pub use crate::pac::NVIC_PRIO_BITS;

embassy_hal_internal::interrupt_mod!(
    ID_RTT,
    ID_PMC,
    ID_EFC0,
    ID_EFC1,
    ID_UART,
    ID_PIOA,
    ID_PIOB,
    ID_PIOC,
    ID_USART0,
    ID_USART1,
    ID_USART2,
    ID_USART3,
    ID_HSMCI,
    ID_TWI0,
    ID_TWI1,
    ID_SPI,
    ID_SSC,
    ID_TC0,
    ID_TC1,
    ID_TC2,
    ID_PWM,
    ID_ADC12B,
    ID_ADC,
    ID_DMAC,
    ID_UDPHS,
);

use pac::HSMCI as LED;

// Holds peripherial for each pin in Ports A, B, & C
embassy_hal_internal::peripherals! {
    PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PA8, PA9, PA10, PA11, PA12, PA13, PA14, PA15, PA16, PA17, PA18, PA19, PA20, PA21, PA22, PA23, PA24, PA25, PA26, PA27, PA28, PA29, PA30, PA31,
    PB0, PB1, PB2, PB3, PB4, PB5, PB6, PB7, PB8, PB9, PB10, PB11, PB12, PB13, PB14, PB15, PB16, PB17, PB18, PB19, PB20, PB21, PB22, PB23, PB24, PB25, PB26, PB27, PB28, PB29, PB30, PB31,
    PC0, PC1, PC2, PC3, PC4, PC5, PC6, PC7, PC8, PC9, PC10, PC11, PC12, PC13, PC14, PC15, PC16, PC17, PC18, PC19, PC20, PC21, PC22, PC23, PC24, PC25, PC26, PC27, PC28, PC29, PC30, PC31,
}

pub struct UserPeripherals {
    // led
    pub led: Peri<'static, peripherals::PB18>,

    // Cyclone2 Programing Pins
    pub data: Peri<'static, peripherals::PA23>,
    pub clk: Peri<'static, peripherals::PA24>,
    pub nconfig: Peri<'static, peripherals::PA25>,
    pub done: Peri<'static, peripherals::PB1>,
    pub nstatus: Peri<'static, peripherals::PB2>,

    /// FPGA → SAM3U interrupt line (active-low). Connected to FPGA pin
    /// 59 / SAM3U PA22. The FPGA asserts this when either emulated IDE
    /// device latches a new command (4 cycles after `command_pend` rises).
    pub mcu_irqn: Peri<'static, peripherals::PA22>,
}

pub fn init(config: config::Config) -> UserPeripherals {
    // Do this first, so that it panics if user is calling `init` a second time
    // before doing anything important.
    let mut core_p = cortex_m::Peripherals::take().unwrap();
    let pac_p = atsam3u4e_pac::Peripherals::take().unwrap();

    let p = Peripherals::take();

    unsafe {
        efc::init();
        PowerManagementController::init(config.clocks);
    }
    wdt::disable();
    unsafe {
        // Enable Peripheral Clocks. HSMCI = pid17 — the SVD doesn't expose
        // a typed accessor, so set it via raw bits alongside the named ones.
        pac_p.PMC.pmc_pcer0.write_with_zero(|w| {
            w
                .pid9().set_bit() // SMC
                .pid10().set_bit() // PIOA
                .pid11().set_bit() // PIOB
                .pid12().set_bit() // PIOC
        });
        pac_p.PMC.pmc_pcer0.write_with_zero(|w| w.bits(1 << 17)); // HSMCI

        // Configure the SMC chip select 3 window where the FPGA lives.
        smc::init();

        // Bring up the DMAC so the IDE data path can push the CD_READ
        // payload to the FPGA write FIFO without a CPU loop.
        dmac::init();

        // Bring up HSMCI for the SD card slot. MCK = PLLA = 96 MHz.
        hsmci::init(96_000_000);

        // UART0 on PA11/PA12 — survives MCU resets across DC power
        // cycles, so we use it as the durable log channel for IDE
        // command tracing alongside RTT.
        uart::init(96_000_000);
        for &b in b"\r\n[opengdemu uart up]\r\n" {
            uart::write_byte(b);
        }

        // Route PCK0 to PA21 (peripheral B) and enable it. This drives one of
        // the FPGA's input clocks.
        gpio::PeripheralB::new(p.PA21);
        PowerManagementController::enable_pck0();
    }


    #[cfg(feature = "time-driver")]
    unsafe {
        systick::init(config.systick);
        // timer::init();
    }

    // peripherals
    UserPeripherals{
        led: p.PB18,

        data: p.PA23,
        clk: p.PA24,
        nconfig: p.PA25,
        done: p.PB1,
        nstatus: p.PB2,

        mcu_irqn: p.PA22,
    }
}