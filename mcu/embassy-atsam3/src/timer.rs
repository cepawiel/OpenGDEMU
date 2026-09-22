use core::cell::RefCell;
use core::task::Waker;

use critical_section::Mutex;
use embassy_time_driver::Driver;
use embassy_time_queue_utils::Queue;

use crate::systick;

struct TimerDriver {
    queue: Mutex<RefCell<Queue>>,
}

embassy_time_driver::time_driver_impl!(static DRIVER: TimerDriver = TimerDriver {
    queue: Mutex::new(RefCell::new(Queue::new())),
});

impl Driver for TimerDriver {
    fn now(&self) -> u64 {
        systick::now()
    }

    fn schedule_wake(&self, at: u64, waker: &Waker) {
        critical_section::with(|cs| {
            self.queue.borrow(cs).borrow_mut().schedule_wake(at, waker);
        });
    }
}

/// Drain the queue, waking any wakers whose deadline has passed.
/// Called from the SysTick interrupt handler at the systick tick rate (1 kHz).
pub(crate) fn on_tick() {
    let now = systick::now();
    critical_section::with(|cs| {
        DRIVER.queue.borrow(cs).borrow_mut().next_expiration(now);
    });
}
