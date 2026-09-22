


// The PAC only re-exports `interrupt` under its own `rt` feature, which our
// `rt` turns on. The handlers below are vector-table entries, so they are
// meaningless without it anyway.
#[cfg(feature = "rt")]
use crate::pac::interrupt;
// use embedded_hal_async::digital::Wait;
use embedded_hal::digital::v2::{InputPin, PinState, OutputPin, toggleable, StatefulOutputPin};
use crate::pac::{pioa, PIOA, PIOB, PIOC};
use crate::{peripherals, Peri};
use defmt::debug;

use crate::pac::pioa::RegisterBlock;

use crate::PeripheralType;

#[derive(Debug, Eq, PartialEq, Clone, Copy)]
pub enum Port {
    A = 0,
    B = 1,
    C = 2,
}

impl From<u8> for Port {
    fn from(val: u8) -> Self {
        match val {
            0 => Self::A,
            1 => Self::B,
            2 => Self::C,
            _ => unsafe { unreachable_unchecked() },
        }
    }
}

impl From<Port> for u8 {
    fn from(port: Port) -> u8 {
        match port {
            Port::A => 0,
            Port::B => 1,
            Port::C => 2,
        }
    }
}

/// Represents a digital input or output level.
#[derive(Debug, Eq, PartialEq, Clone, Copy)]
pub enum Level {
    Low,
    High,
}

impl From<bool> for Level {
    fn from(val: bool) -> Self {
        match val {
            true => Self::High,
            false => Self::Low,
        }
    }
}

impl From<Level> for bool {
    fn from(level: Level) -> bool {
        match level {
            Level::Low => false,
            Level::High => true,
        }
    }
}

#[derive(Debug, Eq, PartialEq, Clone, Copy)]
pub enum Pull {
    None,
    Up,
}

pub(crate) mod sealed {
    use super::*;

    pub trait Pin {
        fn pin_port(&self) -> u8;
            
        #[inline]
        fn _pin(&self) -> u8 {
            self.pin_port() % 32
        }

        #[inline]
        fn block(&self) -> &'static RegisterBlock {
            unsafe {
                match self.pin_port() / 32 {
                    0 => &*PIOA::ptr(),
                    1 => &*PIOB::ptr(),
                    2 => &*PIOC::ptr(),
                    _ => unreachable_unchecked(),
                }
            }
        }

    }
}


/// Latest pin-bitmask that fired a PIOA interrupt. Bits set = pins
/// whose interrupt was asserted on the most recent ID_PIOA dispatch.
/// Tasks call `pioa_wait_irq().await` to receive a wake-up.
static PIOA_IRQ_SIGNAL: embassy_sync::signal::Signal<
    embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
    u32,
> = embassy_sync::signal::Signal::new();

/// Wait for the next PIOA interrupt event and return the pending mask
/// from `PIO_ISR` (reading ISR also clears the flags inside the IRQ
/// handler). Bits are pin numbers within port A.
pub async fn pioa_wait_irq() -> u32 {
    PIOA_IRQ_SIGNAL.wait().await
}

/// Configure a PIOA pin as a falling-edge-triggered input with internal
/// pull-up, then unmask the PIOA interrupt at the NVIC. After this,
/// `pioa_wait_irq().await` returns whenever the pin transitions HIGH→LOW
/// (or any other PIOA pin set up the same way fires).
///
/// Caller is responsible for ensuring the PIOA peripheral clock is on
/// (handled by `embassy_atsam3::init` for any non-zero PIOA usage).
pub unsafe fn enable_pioa_falling_irq(pin: u8) {
    let pioa = unsafe { &*PIOA::ptr() };
    let bit = 1u32 << pin;

    // Take the pin away from any peripheral mux and put it in PIO mode,
    // configured as an input with a pull-up. Active-low signals (FPGA's
    // `MCU_IRQn`) idle high via the pull-up.
    pioa.per.write_with_zero(|w| unsafe { w.bits(bit) });
    pioa.odr.write_with_zero(|w| unsafe { w.bits(bit) });
    pioa.puer.write_with_zero(|w| unsafe { w.bits(bit) });

    // Switch from "any-edge" default to falling-edge interrupt:
    //   AIMER  → use additional interrupt modes for this pin
    //   ESR    → edge-triggered (clear LSR for level)
    //   FELLSR → falling-edge / low-level select
    pioa.aimer.write_with_zero(|w| unsafe { w.bits(bit) });
    pioa.esr.write_with_zero(|w| unsafe { w.bits(bit) });
    pioa.fellsr.write_with_zero(|w| unsafe { w.bits(bit) });

    // Clear any latched-pending bit by reading ISR, then enable.
    let _ = pioa.isr.read();
    pioa.ier.write_with_zero(|w| unsafe { w.bits(bit) });

    // Unmask in the NVIC. ID_PIOA is interrupt #11 on the SAM3U4E.
    cortex_m::peripheral::NVIC::unmask(crate::pac::Interrupt::ID_PIOA);
}

/// Wake anything waiting on [`pioa_wait_irq`], without an interrupt having
/// happened.
///
/// For work that has to reach the task servicing that signal but does not
/// originate from the pin -- a button press asking for a disc swap, which
/// the IDE task has to perform because it owns the device state. Waking it
/// directly is better than having it poll: sweeping the FPGA on a timer when
/// no command is pending is work in the hot path for an event that happens
/// once a session.
pub fn pioa_signal_wake() {
    PIOA_IRQ_SIGNAL.signal(0);
}

/// Configure a PIOA pin as a plain input with its internal pull-up, for
/// something polled rather than interrupt-driven.
///
/// The front-panel button is on PA20 and is read this way: a press is a
/// human-timescale event, so sampling it costs nothing and avoids putting a
/// bouncing mechanical contact on the same interrupt path as the FPGA's
/// command line.
///
/// # Safety
///
/// Touches PIOA registers directly and assumes the peripheral clock is on,
/// which `init` arranges for any non-zero PIOA use.
pub unsafe fn configure_pioa_input_pullup(pin: u8) {
    let pioa = unsafe { &*PIOA::ptr() };
    let bit = 1u32 << pin;
    pioa.per.write_with_zero(|w| unsafe { w.bits(bit) });
    pioa.odr.write_with_zero(|w| unsafe { w.bits(bit) });
    pioa.puer.write_with_zero(|w| unsafe { w.bits(bit) });
}

/// Is a PIOA pin currently low? A button to ground reads low when pressed.
pub fn pioa_is_low(pin: u8) -> bool {
    let pioa = unsafe { &*PIOA::ptr() };
    pioa.pdsr.read().bits() & (1u32 << pin) == 0
}

#[cfg(feature = "rt")]
#[interrupt]
unsafe fn ID_PIOA() {
    // Reading PIO_ISR clears the latched interrupt sources.
    let pioa = unsafe { &*PIOA::ptr() };
    let mask = pioa.isr.read().bits();
    if mask != 0 {
        PIOA_IRQ_SIGNAL.signal(mask);
    }
}

#[cfg(feature = "rt")]
#[interrupt]
unsafe fn ID_PIOB() {
    debug!("PIOB IRQ");
}

#[cfg(feature = "rt")]
#[interrupt]
unsafe fn ID_PIOC() {
    debug!("PIOC IRQ");
}

pub struct PeripheralA<'d, T: PeripheralType> {
    pin: Flex<'d, T>,
}
impl<'d, T: Pin> PeripheralA<'d, T> {
    pub fn new(pin: Peri<'d, T>) -> Self {
        let mut pin = Flex::new(pin);
        pin.set_perip_a();
        pin.disable_pio();
        Self { pin }
    }
}
pub struct PeripheralB<'d, T: PeripheralType> {
    pin: Flex<'d, T>,
}
impl<'d, T: Pin> PeripheralB<'d, T> {
    pub fn new(pin: Peri<'d, T>) -> Self {
        let mut pin = Flex::new(pin);
        pin.set_perip_b();
        pin.disable_pio();
        Self { pin }
    }
}

/// Type-erased GPIO pin
pub struct AnyPin {
    pin_port: u8,
}

pub trait Pin: PeripheralType + Into<AnyPin> + sealed::Pin + Sized + 'static {
    #[inline]
    fn pin(&self) -> u8 {
        self._pin()
    }

    #[inline]
    fn port(&self) -> Port {
        match self.pin_port() / 32 {
            0 => Port::A,
            1 => Port::B,
            2 => Port::C,
            _ => unsafe { unreachable_unchecked() },
        }
    }

    #[inline]
    fn degrade(self) -> AnyPin {
        AnyPin {
            pin_port: self.pin_port(),
        }
    }
}

macro_rules! impl_pin {
    ($name:ident, $port:expr, $pin_num:expr) => {
        impl sealed::Pin for peripherals::$name {
            #[inline]
            fn pin_port(&self) -> u8 {
                let p : u8 = $port.into();
                p * 32u8 + $pin_num
            }
        }
        impl Pin for peripherals::$name {}

        impl From<peripherals::$name> for crate::gpio::AnyPin {
            fn from(val: peripherals::$name) -> Self {
                crate::gpio::Pin::degrade(val)
            }
        }
    };
}

// Impl Macro for all pins in PIOA
impl_pin!(PA0, Port::A, 0);
impl_pin!(PA1, Port::A, 1);
impl_pin!(PA2, Port::A, 2);
impl_pin!(PA3, Port::A, 3);
impl_pin!(PA4, Port::A, 4);
impl_pin!(PA5, Port::A, 5);
impl_pin!(PA6, Port::A, 6);
impl_pin!(PA7, Port::A, 7);
impl_pin!(PA8, Port::A, 8);
impl_pin!(PA9, Port::A, 9);
impl_pin!(PA10, Port::A, 10);
impl_pin!(PA11, Port::A, 11);
impl_pin!(PA12, Port::A, 12);
impl_pin!(PA13, Port::A, 13);
impl_pin!(PA14, Port::A, 14);
impl_pin!(PA15, Port::A, 15);
impl_pin!(PA16, Port::A, 16);
impl_pin!(PA17, Port::A, 17);
impl_pin!(PA18, Port::A, 18);
impl_pin!(PA19, Port::A, 19);
impl_pin!(PA20, Port::A, 20);
impl_pin!(PA21, Port::A, 21);
impl_pin!(PA22, Port::A, 22);
impl_pin!(PA23, Port::A, 23);
impl_pin!(PA24, Port::A, 24);
impl_pin!(PA25, Port::A, 25);
impl_pin!(PA26, Port::A, 26);
impl_pin!(PA27, Port::A, 27);
impl_pin!(PA28, Port::A, 28);
impl_pin!(PA29, Port::A, 29);
impl_pin!(PA30, Port::A, 30);
impl_pin!(PA31, Port::A, 31);

// Impl Macro for all pins in Port::B
impl_pin!(PB0, Port::B, 0);
impl_pin!(PB1, Port::B, 1);
impl_pin!(PB2, Port::B, 2);
impl_pin!(PB3, Port::B, 3);
impl_pin!(PB4, Port::B, 4);
impl_pin!(PB5, Port::B, 5);
impl_pin!(PB6, Port::B, 6);
impl_pin!(PB7, Port::B, 7);
impl_pin!(PB8, Port::B, 8);
impl_pin!(PB9, Port::B, 9);
impl_pin!(PB10, Port::B, 10);
impl_pin!(PB11, Port::B, 11);
impl_pin!(PB12, Port::B, 12);
impl_pin!(PB13, Port::B, 13);
impl_pin!(PB14, Port::B, 14);
impl_pin!(PB15, Port::B, 15);
impl_pin!(PB16, Port::B, 16);
impl_pin!(PB17, Port::B, 17);
impl_pin!(PB18, Port::B, 18);
impl_pin!(PB19, Port::B, 19);
impl_pin!(PB20, Port::B, 20);
impl_pin!(PB21, Port::B, 21);
impl_pin!(PB22, Port::B, 22);
impl_pin!(PB23, Port::B, 23);
impl_pin!(PB24, Port::B, 24);
impl_pin!(PB25, Port::B, 25);
impl_pin!(PB26, Port::B, 26);
impl_pin!(PB27, Port::B, 27);
impl_pin!(PB28, Port::B, 28);
impl_pin!(PB29, Port::B, 29);
impl_pin!(PB30, Port::B, 30);
impl_pin!(PB31, Port::B, 31);

// Impl Macro for all pins in Port::C
impl_pin!(PC0, Port::C, 0);
impl_pin!(PC1, Port::C, 1);
impl_pin!(PC2, Port::C, 2);
impl_pin!(PC3, Port::C, 3);
impl_pin!(PC4, Port::C, 4);
impl_pin!(PC5, Port::C, 5);
impl_pin!(PC6, Port::C, 6);
impl_pin!(PC7, Port::C, 7);
impl_pin!(PC8, Port::C, 8);
impl_pin!(PC9, Port::C, 9);
impl_pin!(PC10, Port::C, 10);
impl_pin!(PC11, Port::C, 11);
impl_pin!(PC12, Port::C, 12);
impl_pin!(PC13, Port::C, 13);
impl_pin!(PC14, Port::C, 14);
impl_pin!(PC15, Port::C, 15);
impl_pin!(PC16, Port::C, 16);
impl_pin!(PC17, Port::C, 17);
impl_pin!(PC18, Port::C, 18);
impl_pin!(PC19, Port::C, 19);
impl_pin!(PC20, Port::C, 20);
impl_pin!(PC21, Port::C, 21);
impl_pin!(PC22, Port::C, 22);
impl_pin!(PC23, Port::C, 23);
impl_pin!(PC24, Port::C, 24);
impl_pin!(PC25, Port::C, 25);
impl_pin!(PC26, Port::C, 26);
impl_pin!(PC27, Port::C, 27);
impl_pin!(PC28, Port::C, 28);
impl_pin!(PC29, Port::C, 29);
impl_pin!(PC30, Port::C, 30);
impl_pin!(PC31, Port::C, 31);

/// GPIO flexible pin.
///
/// This pin can be either an input or output pin. The output level register bit will remain
/// set while not in output mode, so the pin's level will be 'remembered' when it is not in output
/// mode.
pub struct Flex<'d, T: PeripheralType> {
    pin: Peri<'d, T>,
}

impl<'d, T: Pin> Flex<'d, T> {
    #[inline]
    pub fn new(pin: Peri<'d, T>) -> Self {
        Self { pin }
    }

    #[inline]
    fn regs(&self) -> &'static RegisterBlock { self.pin.block() }
    #[inline]
    fn bitmask(&self) -> u32 { 1u32 << self.pin.pin() }

    // output regs
    #[inline]
    fn disable_output(&mut self) {
        unsafe { self.pin.block().odr.write_with_zero(|w| { w.bits(self.bitmask()) }) }
    }

    #[inline]
    fn enable_output(&mut self) {
        unsafe { self.pin.block().oer.write_with_zero(|w| { w.bits(self.bitmask()) }) }
    }

    #[inline]
    fn set_pull_up(&mut self) {
        unsafe { self.pin.block().puer.write_with_zero(|w| { w.bits(self.bitmask()) }) }
    }

    #[inline]
    fn set_pull_none(&mut self) {
        unsafe { self.pin.block().pudr.write_with_zero(|w| { w.bits(self.bitmask()) }) }
    }

    #[inline]
    fn set_pull(&mut self, pull: Pull){
        match pull {
            Pull::None => self.set_pull_none(),
            Pull::Up => self.set_pull_up(),
        }
    }

    #[inline]
    fn set_perip_a(&mut self) {
        self.pin.block().absr.modify(|r, w| { unsafe { w.bits(r.bits() & !self.bitmask() ) }})
    }

    #[inline]
    fn set_perip_b(&mut self) {
        self.pin.block().absr.modify(|r, w| { unsafe { w.bits(r.bits() | self.bitmask() ) }})
    }

    /// Release the pin from PIO control to the peripheral mux selected via
    /// ABSR. Without this, the pin remains a GPIO regardless of the ABSR
    /// setting (SAM3U datasheet: "PIO Controller Multiplexing").
    #[inline]
    fn disable_pio(&mut self) {
        unsafe { self.pin.block().pdr.write_with_zero(|w| { w.bits(self.bitmask()) }) }
    }

    #[inline]
    fn set_high(&mut self) {
        unsafe { self.pin.block().sodr.write_with_zero(|w| { w.bits(self.bitmask()) }) }
    }

    #[inline]
    fn set_low(&mut self) {
        unsafe { self.pin.block().codr.write_with_zero(|w| { w.bits(self.bitmask()) }) }
    }

    #[inline]
    fn set_level(&mut self, level: Level) {
        match level {
            Level::Low => self.set_low(),
            Level::High => self.set_high(),
        }
    }

    fn is_set_low(&self) -> bool {
        (self.pin.block().odsr.read().bits() & self.bitmask()) == 0
    }

    fn is_set_high(&self) -> bool {
        (self.pin.block().odsr.read().bits() & self.bitmask()) != 0
    }

    #[inline]
    fn get_output_level(&self) -> Level {
        ((self.pin.block().odsr.read().bits() & self.bitmask()) != 0).into()
    }

    // input regs
    #[inline]
    fn get_pin_level(&self) ->Level {
        ((self.pin.block().pdsr.read().bits() & self.bitmask()) != 0).into()
    }

    #[inline]
    fn is_high(&self) -> bool {
        self.get_pin_level() == Level::High
    }

    #[inline]
    fn is_low(&self) -> bool {
        self.get_pin_level() == Level::Low
    }

}

pub struct Input<'d, T: PeripheralType> {
    pin: Flex<'d, T>,
}
impl<'d, T: Pin> Input<'d, T> {
    pub fn new(pin: Peri<'d, T>, pull: Pull) -> Self {
        let mut pin = Flex::new(pin);
        pin.disable_output();
        pin.set_pull(pull);
        Self { pin }
    }

    #[inline]
    pub fn is_high(&self) -> bool {
        self.pin.is_high()
    }

    #[inline]
    pub fn is_low(&self) -> bool {
        self.pin.is_low()
    }
}
pub struct Output<'d, T: PeripheralType> {
    pin: Flex<'d, T>,
}
impl<'d, T: Pin> Output<'d, T> {
    #[inline]
    pub fn new(pin: Peri<'d, T>, initial_output: Level) -> Self {
        let mut pin = Flex::new(pin);
        pin.set_level(initial_output);
        pin.enable_output();
        Self { pin }
    }

    #[inline]
    pub fn set_high(&mut self) {
        self.pin.set_high()
    }

    #[inline]
    pub fn set_low(&mut self) {
        self.pin.set_low()
    }

    #[inline]
    pub fn set_level(&mut self, level: Level) {
        self.pin.set_level(level)
    }

    #[inline]
    pub fn get_output_level(&self) -> Level {
        self.pin.get_output_level()
    }

    #[inline]
    pub fn is_set_high(&self) -> bool {
        self.pin.is_set_high()
    }

    #[inline]
    pub fn is_set_low(&self) -> bool {
        self.pin.is_set_low()
    }
}


// Embedded Hal Implementations
use core::convert::Infallible;
use core::hint::unreachable_unchecked;
impl<'d, T: Pin> embedded_hal::digital::v2::InputPin for Input<'d, T> {
    type Error = Infallible;

    fn is_high(&self) -> Result<bool, Self::Error> {
        Ok(self.is_high())
    }

    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(self.is_low())
    }
}


impl<'d, T: Pin> embedded_hal::digital::v2::OutputPin for Output<'d, T> {
    type Error = Infallible;

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.set_high();
        Ok(())
    }

    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.set_low();
        Ok(())
    }
}

impl<'d, T: Pin> embedded_hal::digital::v2::StatefulOutputPin for Output<'d, T> {
    fn is_set_high(&self) -> Result<bool, Self::Error> {
        Ok(self.pin.is_set_high())
    }

    fn is_set_low(&self) -> Result<bool, Self::Error> {
        Ok(self.pin.is_set_low())
    }
}

impl<'d, T: Pin> toggleable::Default for Output<'d, T> {}

impl<'d, T: Pin> embedded_hal::digital::v2::InputPin for Flex<'d, T> {
    type Error = Infallible;

    fn is_high(&self) -> Result<bool, Self::Error> {
        Ok(self.is_high())
    }

    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(self.is_low())
    }
}

impl<'d, T: Pin> embedded_hal::digital::v2::OutputPin for Flex<'d, T> {
    type Error = Infallible;

    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(self.set_high())
    }

    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(self.set_low())
    }
}

impl<'d, T: Pin> embedded_hal::digital::v2::StatefulOutputPin for Flex<'d, T> {
    fn is_set_high(&self) -> Result<bool, Self::Error> {
        Ok(self.is_set_high())
    }

    fn is_set_low(&self) -> Result<bool, Self::Error> {
        Ok(self.is_set_low())
    }
}

impl<'d, T: Pin> toggleable::Default for Flex<'d, T> {}

// #[cfg(feature = "nightly")]
// impl<'d, T: Pin> embedded_hal_async::digital::Wait for Flex<'d, T> {
//     async fn wait_for_high(&mut self) -> Result<(), Self::Error> {
//         self.wait_for_high().await;
//         Ok(())
//     }

//     async fn wait_for_low(&mut self) -> Result<(), Self::Error> {
//         self.wait_for_low().await;
//         Ok(())
//     }

//     async fn wait_for_rising_edge(&mut self) -> Result<(), Self::Error> {
//         self.wait_for_rising_edge().await;
//         Ok(())
//     }

//     async fn wait_for_falling_edge(&mut self) -> Result<(), Self::Error> {
//         self.wait_for_falling_edge().await;
//         Ok(())
//     }

//     async fn wait_for_any_edge(&mut self) -> Result<(), Self::Error> {
//         self.wait_for_any_edge().await;
//         Ok(())
//     }
// }

// #[cfg(feature = "nightly")]
// impl<'d, T: Pin> embedded_hal_async::digital::Wait for Input<'d, T> {
//     async fn wait_for_high(&mut self) -> Result<(), Self::Error> {
//         self.wait_for_high().await;
//         Ok(())
//     }

//     async fn wait_for_low(&mut self) -> Result<(), Self::Error> {
//         self.wait_for_low().await;
//         Ok(())
//     }

//     async fn wait_for_rising_edge(&mut self) -> Result<(), Self::Error> {
//         self.wait_for_rising_edge().await;
//         Ok(())
//     }

//     async fn wait_for_falling_edge(&mut self) -> Result<(), Self::Error> {
//         self.wait_for_falling_edge().await;
//         Ok(())
//     }

//     async fn wait_for_any_edge(&mut self) -> Result<(), Self::Error> {
//         self.wait_for_any_edge().await;
//         Ok(())
//     }
// }

// #[cfg(feature = "nightly")]
// impl<'d, T: Pin> embedded_hal_async::digital::Wait for OutputOpenDrain<'d, T> {
//     async fn wait_for_high(&mut self) -> Result<(), Self::Error> {
//         self.wait_for_high().await;
//         Ok(())
//     }

//     async fn wait_for_low(&mut self) -> Result<(), Self::Error> {
//         self.wait_for_low().await;
//         Ok(())
//     }

//     async fn wait_for_rising_edge(&mut self) -> Result<(), Self::Error> {
//         self.wait_for_rising_edge().await;
//         Ok(())
//     }

//     async fn wait_for_falling_edge(&mut self) -> Result<(), Self::Error> {
//         self.wait_for_falling_edge().await;
//         Ok(())
//     }

//     async fn wait_for_any_edge(&mut self) -> Result<(), Self::Error> {
//         self.wait_for_any_edge().await;
//         Ok(())
//     }
// }
