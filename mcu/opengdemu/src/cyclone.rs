
use cortex_m::asm;
use defmt::debug;
use embassy_atsam3::{Peri, PeripheralType, gpio::{Input, Output, Pin, Level, Pull}};
use embassy_time::{Timer, Duration};

pub struct PSLoader<'a, O1: PeripheralType, O2: PeripheralType, O3: PeripheralType, I1: PeripheralType, I2: PeripheralType> {
    nconfig: Output<'a, O1>,
    clk: Output<'a, O2>,
    data: Output<'a, O3>,
    nstatus: Input<'a, I1>,
    done: Input<'a, I2>,
}

impl<'a, O1: Pin, O2: Pin, O3: Pin, I1: Pin, I2: Pin> PSLoader<'a, O1, O2, O3, I1, I2> {
    pub fn new(
        nconfig: Peri<'a, O1>,
        clk: Peri<'a, O2>,
        data: Peri<'a, O3>,
        nstatus: Peri<'a, I1>,
        done: Peri<'a, I2>,
    ) -> Self {
        let nconfig = Output::new(nconfig, Level::Low);
        let clk = Output::new(clk, Level::Low);
        let data = Output::new(data, Level::Low);
        let nstatus = Input::new(nstatus, Pull::Up);
        let done = Input::new(done, Pull::Up);

        Self {
            nconfig,
            clk,
            data,
            nstatus,
            done,
        }
    }

    async fn delay() {
        for _ in 0..5 {
            asm::nop();
        }
        // Timer::after(Duration::from_millis(1)).await;
    }

    #[inline]
    async fn send_bit(&mut self, bit: bool) {
        self.clk.set_low();
        Self::delay().await;

        self.data.set_level(bit.into());
        Self::delay().await;

        self.clk.set_high();
        Self::delay().await;
        Self::delay().await;
    }

    #[inline]
    async fn send_byte(&mut self, byte: u8) {
        for i in 0..8 {
            let bit = (byte & (1 << i)) != 0;
            self.send_bit(bit).await;
        }
    }

    pub async fn load(&mut self, bitstream: &[u8]) {
        self.load_from(bitstream.iter().copied()).await
    }

    /// Clock configuration bytes in as they are produced.
    ///
    /// Taking an iterator rather than a slice is what lets the bitstream stay
    /// compressed in flash: the LZMA decoder hands over one byte at a time
    /// and the 72 KB decompressed form never exists in memory.
    pub async fn load_from<I: Iterator<Item = u8>>(&mut self, bitstream: I) {
        // Cyclone II PS-config timing: hold nconfig low for ≥500 ns, then
        // raise it and wait for nstatus to go high (the FPGA signaling it's
        // ready to clock in data, max ~230 µs per Altera AN-114). The
        // earlier 1-second Timers here were a debug holdover and dominated
        // total power-on-to-FPGA-ready time — when our board is powered
        // by the DC, every saved millisecond is one less millisecond the
        // DC's south bridge sees a dead GD-ROM bus.
        Timer::after(Duration::from_millis(1)).await;
        self.nconfig.set_high();
        while self.nstatus.is_low() {
            asm::nop();
        }

        debug!("Starting Bitsream Upload");
        for byte in bitstream {
            if self.done.is_high() {
                debug!("FPGA Done High!");
                return;
            }
            if self.nstatus.is_low() {
                panic!("FPGA Upload Error");
            }

            self.send_byte(byte).await;
        }

        debug!("Bitstream Uploaded! Starting Padding");
        // send 0s until done
        while self.done.is_low() {
            self.send_bit(false).await;
        }
    }
    
}

