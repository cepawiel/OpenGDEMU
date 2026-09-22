
use defmt::debug;
use pac::pwm::clk;

use crate::{pac::PMC, config::{self, ClockConfig}};
use crate::pac;

#[inline]
const fn pmc() -> &'static pac::pmc::RegisterBlock {
    unsafe { &*PMC::ptr() }
}

pub struct PowerManagementController {}

impl PowerManagementController {
    pub unsafe fn init(config: ClockConfig) {
        Self::disable_wp();

        pmc().ckgr_mor.modify(| _, w| { 
            // register "password"
            w.key().bits(0x37)   
            // startup cycles * 8
             .moscxtst().bits(8)
            // main osc enable
             .moscxten().set_bit()
            // on chip rc enable
            // .moscrcen().set_bit()

            // disable main xosc bypass
            //  .moscxtby().clear_bit()
            
        });

        // wait for osc status bit to clear
        // debug!("Waiting for Main OSC");
        while pmc().pmc_sr.read().moscxts().bit_is_clear() {}

        // switch clock src
        pmc().ckgr_mor.write_with_zero(|w| {
            // register "password"
            w.key().bits(0x37)
            // startup cycles * 8
             .moscxtst().bits(8)
             // main osc enable
            .moscxten().set_bit()            
             // on chip rc enable
            //  .moscrcen().set_bit()

            // use main osc
            .moscsel().set_bit()
        });

        // wait for master clock ready
        // debug!("Waiting for MCK");
        while pmc().pmc_sr.read().moscsels().bit_is_clear() {}

        pmc().ckgr_pllar.modify(|_, w| unsafe {
            // required 1
            w.one().set_bit()
             .mula().bits(config.plla.mul - 1)
             .pllacount().bits(0x3F)
             .diva().bits(config.plla.div)
        });

        // wait for pll A ready
        // debug!("Waiting for PLLA");
        while pmc().pmc_sr.read().locka().bit_is_clear() {}
        debug!("PLLA Locked");

        pmc().pmc_mckr.modify(|_, w| w.pres().clk_1() );
        while pmc().pmc_sr.read().mckrdy().bit_is_clear() {}
        pmc().pmc_mckr.modify(|_, w| w.css().plla_clk() );
        while pmc().pmc_sr.read().mckrdy().bit_is_clear() {}

        print_clock_info();
    }

    pub unsafe fn disable_wp() {
        if pmc().pmc_wpmr.read().wpen().bit_is_set() {
            debug!("Disabling PMC WP");
            pmc().pmc_wpmr.write_with_zero(|w| {
                w.wpkey().bits(0x504D43)
            });

            while pmc().pmc_wpmr.read().wpen().bit_is_set() {}
        }
    }

    pub unsafe fn enable_pck0() {
        // disable PCK
        pmc().pmc_scdr.write_with_zero(|w| {
            w.pck0().set_bit()
        });
        while pmc().pmc_scsr.read().pck0().bit_is_set() {}

        // Setup PCK Source & Divider — PCK0 drives the FPGA's CLK_48_MHz pin.
        // PLLA is 96 MHz, so /2 gives the expected 48 MHz.
        pmc().pmc_pck[0].modify(|_, w| {
            w.css().plla_clk().pres().clk_2()
        });
        while pmc().pmc_sr.read().pckrdy0().bit_is_clear() {}

        // Enable PCK
        pmc().pmc_scer.write_with_zero(|w| {
            w.pck0().set_bit()
        });
        while pmc().pmc_scsr.read().pck0().bit_is_clear() {}

        // Wait for PCK to be ready
        while pmc().pmc_sr.read().pckrdy0().bit_is_clear() {}
    }

}

fn print_clock_info() {
    // wait for MAINFRDY to be set
    while pmc().ckgr_mcfr.read().mainfrdy().bit_is_clear() {}

    let clk_per16slow = pmc().ckgr_mcfr.read().mainf().bits() as u32;
    let clk_per_sec = (clk_per16slow * 32_768) / 16;
    debug!("[{}] Main clock: {} Hz", clk_per16slow, clk_per_sec);

}