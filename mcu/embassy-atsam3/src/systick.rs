

use crate::{pac, config::SystickConfig};
use cortex_m::peripheral::syst::SystClkSource;
use cortex_m_rt::exception;
use core::cell::Cell;

use critical_section::Mutex;
use portable_atomic::{AtomicU8, AtomicUsize, Ordering, AtomicU32};
use defmt::debug;

/// Milliseconds since boot.
///
/// A `Cell` under a critical section rather than an `AtomicU64`. This part
/// has 32-bit atomics but not 64-bit ones, so `portable_atomic::AtomicU64`
/// falls back to a seqlock -- and that lock table is **2144 bytes of `.bss`**
/// on a device with 32 KB of RAM and about two kilobytes of stack. The
/// critical section costs a few cycles in a 1 kHz interrupt and one on each
/// read, which is the better trade by a wide margin.
static SYSTICK: Mutex<Cell<u64>> = Mutex::new(Cell::new(0));

/// Optional callback run from the SysTick exception every millisecond,
/// after the tick counter has advanced. Stored as a plain address so it can
/// be installed at runtime without locking; 0 = none.
static TICK_HOOK: AtomicUsize = AtomicUsize::new(0);

/// Install `f` as the per-millisecond hook (see [`TICK_HOOK`]). It runs in
/// interrupt context, so it must be short and must not block.
pub fn set_tick_hook(f: fn()) {
    TICK_HOOK.store(f as usize, Ordering::Release);
}

pub(crate) unsafe fn init(config : SystickConfig) {
    debug!("Using {} for 1ms systick", config.div);

    // TODO: probably don't do this?
    let s = &mut pac::CorePeripherals::steal().SYST;

    s.clear_current();
    s.set_clock_source(SystClkSource::Core);
    s.enable_interrupt();
    s.set_reload(config.div);

    s.enable_counter();
}

pub fn now() -> u64 {
    critical_section::with(|cs| SYSTICK.borrow(cs).get())
}

#[exception]
fn SysTick() {
    critical_section::with(|cs| {
        let t = SYSTICK.borrow(cs);
        t.set(t.get().wrapping_add(1));
    });
    #[cfg(feature = "time-driver")]
    crate::timer::on_tick();
    let hook = TICK_HOOK.load(Ordering::Acquire);
    if hook != 0 {
        // SAFETY: only ever stored from a `fn()` by `set_tick_hook`.
        let f: fn() = unsafe { core::mem::transmute(hook) };
        f();
    }
}
