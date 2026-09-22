
use pac::{EFC0, EFC1};

use crate::pac;

#[inline]
const fn efc0() -> &'static pac::efc0::RegisterBlock{
    unsafe { &*EFC0::ptr() }
}

#[inline]
const fn efc1() -> &'static pac::efc1::RegisterBlock{
    unsafe { &*EFC1::ptr() }
}


pub unsafe fn init() {
    efc0().fmr.write(|w| unsafe {
        w.fws().bits(4)
    });
    efc1().fmr.write(|w| unsafe {
        w.fws().bits(4)
    });
}