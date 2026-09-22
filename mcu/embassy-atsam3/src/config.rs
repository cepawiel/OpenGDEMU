

// Clock Configs
pub struct ClockConfig {
    pub xosc: XoscConfig,
    pub plla: PllAConfig,
}

pub struct XoscConfig {
    pub freq: u32,
}

pub struct PllAConfig {
    pub mul: u16,
    pub div: u8,
}

// Systick Config
pub struct SystickConfig {
    pub div: u32,
}

// Core Config
pub struct Config {
    pub clocks : ClockConfig,
    pub systick : SystickConfig,
}

impl Config {
    pub const fn new(
        xosc_freq : u32,
        plla_mult : u16,
        plla_div: u8,
    ) -> Self {
        let core_clock = (xosc_freq * plla_mult as u32) / plla_div as u32;

        Config {
            clocks: ClockConfig {
                xosc: XoscConfig { freq: xosc_freq },
                plla: PllAConfig { mul: plla_mult, div: plla_div },
            },
            systick: SystickConfig { div: core_clock / 1_000 },
        }
    }
}

  
