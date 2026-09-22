
use pac::WDT;

use crate::pac;

#[inline]
const fn wdt() -> &'static pac::wdt::RegisterBlock {
    unsafe { &*WDT::ptr() }
}

pub fn disable() {
    wdt().mr.write(|w| {
        w.wddis().set_bit()
    });
}