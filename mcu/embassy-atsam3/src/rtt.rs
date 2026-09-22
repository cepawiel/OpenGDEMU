
// use crate::{interrupt::{self, typelevel::ID_RTT}, pac};

// use defmt::{debug};
// use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
// use embassy_sync::blocking_mutex::Mutex;
// use core::{marker::PhantomData, cell::Cell};
// use embassy_hal_internal::PeripheralRef;
// use portable_atomic::Ordering;


// pub struct Callbacks {
//     alarm_callback: Option<(fn(*mut ()), *mut ())>,
//     tick_callback: Option<(fn(*mut ()), *mut ())>,
// }

// pub struct RealTimeTimer<'d> {
//     _phantom: PhantomData<&'d ()>,
//     cb: Mutex<CriticalSectionRawMutex, Callbacks>,
// }
// unsafe impl Send for Callbacks {}

// impl<'d> RealTimeTimer<'d> {
//     pub const fn new() -> Self {

//         let cb = Mutex::const_new(
//             CriticalSectionRawMutex::new(),
//             Callbacks {
//                 alarm_callback: None,
//                 tick_callback: None,
//             }
//         );
//         Self {
//             _phantom: PhantomData,
//             cb: cb,
//         }
//     }

//     pub fn init(
//         &self,
//         _irq: impl interrupt::typelevel::Binding<ID_RTT, InterruptHandler> + 'd,
//         tps: u16,
//     ) {
//         rtt().mr.write(|w| unsafe {
//             w.rtpres().bits(32_000 / tps)
//         });
//     }

//     pub fn set_tick_callback(&mut self, callback: fn(*mut ()), ctx: *mut ()) {
//         self.cb.get_mut().tick_callback = Some((callback, ctx));
//         // self.cb.lock(|d| {
//         //     d.tick_callback = Some((callback, ctx));
//         // });
//     }

//     pub fn enable_tick_callback(&self) {
//         rtt().mr.write(|w| w.rttincien().set_bit());
//     }

//     pub fn set_alarm_callback(&mut self, callback: fn(*mut ()), ctx: *mut ()) {
//         self.cb.get_mut().alarm_callback = Some((callback, ctx));
//         // self.cb.lock(|d| {
//         //     d.alarm_callback = Some((callback, ctx));
//         // });
//     }

//     pub fn set_alarm(&self, timestamp: u32) {
//         rtt().ar.write(|w| unsafe { w.bits(timestamp) });
//         rtt().mr.write(|w| w.almien().set_bit());
//     }

//     pub fn cancel_alarm(&self) {
//         rtt().mr.write(|w| w.almien().clear_bit());
//     }
// }





