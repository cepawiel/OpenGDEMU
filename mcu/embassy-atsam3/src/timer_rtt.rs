

use atsam3u4e_pac::RTT;
use embassy_time::Timer;

use crate::interrupt::InterruptExt;
use crate::{interrupt, pac};
use crate::bind_interrupts;

use core::cell::Cell;
use core::sync::atomic::AtomicBool;
use portable_atomic::{AtomicU8, AtomicU64, Ordering, AtomicU32};

use defmt::{debug, panic};
use embassy_time::driver::{AlarmHandle, Driver};
use critical_section::CriticalSection;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;



struct AlarmState {
    in_use: AtomicBool,
    timestamp: Cell<u64>,
    callback: Cell<Option<(fn(*mut ()), *mut ())>>,
}
unsafe impl Send for AlarmState {}

struct TimerDriver {
    alarm: Mutex<CriticalSectionRawMutex, AlarmState>,

    current_tick: AtomicU64,
}
embassy_time::time_driver_impl!(static DRIVER: TimerDriver = TimerDriver {
    alarm: Mutex::const_new(CriticalSectionRawMutex::new(), AlarmState{
        in_use: AtomicBool::new(false),
        callback: Cell::new(None),
        timestamp: Cell::new(0xFFFF_FFFF_FFFF_FFFF),
    }),
    
    current_tick: AtomicU64::new(0),
});

#[inline]
fn rtt() -> &'static pac::rtt::RegisterBlock {
    unsafe { &*pac::RTT::ptr() }
}

impl Driver for TimerDriver {
    fn now(&self) -> u64 {
        let a = DRIVER.current_tick.load(Ordering::Relaxed);
        // debug!("now {}", a);
        a
    }

    unsafe fn allocate_alarm(&self) -> Option<AlarmHandle> {
        // debug!("allocate alarm");
        self.alarm.lock(|a| {
            let s = a.in_use.fetch_update(Ordering::Relaxed, Ordering::Relaxed, |b| {
                if b {
                    None
                } else {
                    Some(true)
                }
            });

            match s {
                Ok(_) => {
                    // debug!("allocate alarm ok");
                    Some(AlarmHandle::new(0))
                }
                Err(_) => {
                    debug!("allocate alarm err");
                    None
                }
            }
        })
    }

    fn set_alarm_callback(&self, alarm: AlarmHandle, callback: fn(*mut ()), ctx: *mut ()) {
        assert_eq!(alarm.id(), 0);
        self.alarm.lock(|a| {
            a.callback.set(Some((callback, ctx)));
        });
    }

    fn set_alarm(&self, alarm: AlarmHandle, timestamp: u64) -> bool {
        assert_eq!(alarm.id(), 0);
        if timestamp <= self.now() {
            false
        } else {
            // ms to match systick
            let prescaler : u16  = 32_768 / 1_000;
            self.alarm.lock(|a| {
                a.timestamp.set(timestamp);
            });
            rtt().ar.write(|w| unsafe { w.bits( timestamp.try_into().unwrap()) });
            rtt().mr.write(|w| 
                w
                    .rtpres().bits(prescaler)
                    .almien().set_bit()
                    .rttrst().set_bit()
            );
            true
        }
    }
}

impl TimerDriver {
    fn now32() -> u32 {
        rtt().vr.read().bits()
    }
}

use crate::interrupt::typelevel::ID_RTT;

pub unsafe fn init() {
    // set mode
    rtt().mr.write(|w| { 
        w.rtpres().bits(32_768 / 1_000)
        // w.rtpres().bits(0x4000)
        //  .almien().set_bit()
        // .rttincien().set_bit()
        .rttrst().set_bit()
    });

    debug!("Clear RTT IRQ Bits {}", rtt().sr.read().bits());

    interrupt::ID_RTT.enable();
}

/// Interrupt handler.
pub struct InterruptHandler {}

impl interrupt::typelevel::Handler<ID_RTT> for InterruptHandler {
    unsafe fn on_interrupt() {
        let sr = rtt().sr.read().bits();
        debug!("RTT IRQ Bits {}", sr);
        // cortex_m::interrupt::free(|_| {
        //     if sr & 0x02 == 0x02 {
        //         let a = DRIVER.current_tick.fetch_add(1, Ordering::Relaxed);
        //         debug!("Tick: {}", a);
        //     }
        //     if sr & 0x01 == 0x01 {
        //         debug!("ALARM");
        //         rtt().mr.write(|w| w.almien().clear_bit());
        //         DRIVER.alarm.lock(|a| {
        //             a.callback.get().map(|cb| cb.0(cb.1));
        //         });
        //     }
        // });    
    }
}

bind_interrupts!(struct Irqs {
    ID_RTT => InterruptHandler;
});


