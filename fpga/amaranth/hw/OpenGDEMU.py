from amaranth import *

from .cdda import CDDAOutput
from .ide_device import IDEDevice


class OpenGDEMUCore(Elaboratable):
    """OpenGDEMU FPGA core.

    Bridges the SAM3U Static Memory Controller (MCU bus) to the Dreamcast G1
    parallel ATA interface. The Quartus top owns the PLL, tristate I/O, and
    converts the SAM3U's active-low strobes to the active-high signals this
    core consumes.

    Phase 2: two `IDEDevice` instances (primary GD-ROM, secondary HDD) share
    the IDE bus. The DEV bit (Device register bit 4) selects which one drives
    `io_IDE_DATA_OUT` and `io_IDE_INTRQ` on the IDE side. Each device gets
    its own 128-byte MCU window:

      0x6300_0000..0x6300_007F   primary device (MCU_ADDR[7]=0)
      0x6300_0080..0x6300_00FF   secondary device (MCU_ADDR[7]=1)

    CDDA: `CDDAOutput` serialises MCU-supplied PCM onto SCK/SDAT/LRCK in the
    `audio` clock domain (CLK_11_2896_MHz, fed in on `io_AUDIO_CLK`). Its
    MCU registers sit at slots 29..31 of the primary window.
    """

    def __init__(self):
        # MCU SMC bus.
        self.io_MCU_CS = Signal()
        self.io_MCU_BS = Signal(2)
        self.io_MCU_RD = Signal()
        self.io_MCU_WR = Signal()
        self.io_MCU_ADDR = Signal(8)
        self.io_MCU_DATA_IN = Signal(16)
        self.io_MCU_DATA_OUT = Signal(16)
        self.io_MCU_IRQ = Signal()

        # Dreamcast CDDA audio. CDCLK is driven by the wrapper's PLL;
        # SCK/SDAT/LRCK come from the serialiser clocked by io_AUDIO_CLK.
        self.io_AUDIO_CLK = Signal()
        self.io_DC_CDCLK = Signal()
        self.io_DC_SCK = Signal()
        self.io_DC_SDAT = Signal()
        self.io_DC_LRCK = Signal()
        self.io_DC_EMPH = Signal()

        # Dreamcast G1 IDE bus.
        self.io_IDE_RSTn = Signal()
        self.io_IDE_CSn = Signal(2)
        self.io_IDE_WR = Signal()
        self.io_IDE_RD = Signal()
        self.io_IDE_RD_H = Signal()
        self.io_IDE_WR_H = Signal()
        self.io_IDE_ADDR = Signal(3)
        self.io_IDE_DATA_IN = Signal(16)
        self.io_IDE_DATA_OUT = Signal(16)
        self.io_IDE_DMARQ = Signal()
        self.io_IDE_DMACKn = Signal()
        self.io_IDE_IORDY = Signal()
        self.io_IDE_INTRQ = Signal()

        # Audio-xtal heartbeat from the wrapper (a divided-down toggle of
        # CLK_11_2896_MHz). Lets each IDEDevice's diag count audio activity.
        self.io_audio_tick = Signal()

        # Built here rather than in elaborate() so testbenches can reach the
        # per-device debug ports.
        self.dev0 = IDEDevice(primary=True)
        self.dev1 = IDEDevice(primary=False)

    def ports(self):
        return [
            self.io_MCU_CS, self.io_MCU_BS, self.io_MCU_RD, self.io_MCU_WR,
            self.io_MCU_ADDR, self.io_MCU_DATA_IN, self.io_MCU_DATA_OUT,
            self.io_MCU_IRQ,
            self.io_DC_CDCLK, self.io_DC_SCK, self.io_DC_SDAT,
            self.io_DC_LRCK, self.io_DC_EMPH,
            self.io_IDE_RSTn, self.io_IDE_CSn, self.io_IDE_WR, self.io_IDE_RD,
            self.io_IDE_RD_H,
            self.io_IDE_WR_H,
            self.io_IDE_ADDR, self.io_IDE_DATA_IN, self.io_IDE_DATA_OUT,
            self.io_IDE_DMARQ, self.io_IDE_DMACKn, self.io_IDE_IORDY,
            self.io_IDE_INTRQ,
            self.io_audio_tick,
            self.io_AUDIO_CLK,
        ]

    def elaborate(self, platform):
        m = Module()

        m.submodules.dev0 = dev0 = self.dev0
        m.submodules.dev1 = dev1 = self.dev1

        # Audio xtal domain for the CDDA serialiser. Reset-less: it only
        # holds free-running counters, and the sync-side control state
        # (enable, FIFO) is what actually resets.
        m.domains.audio = cd_audio = ClockDomain("audio", reset_less=True)
        m.d.comb += cd_audio.clk.eq(self.io_AUDIO_CLK)
        m.submodules.cdda = cdda = CDDAOutput()

        # IDE bus is shared by both devices.
        for dev in (dev0, dev1):
            m.d.comb += [
                dev.ide_rstn.eq(self.io_IDE_RSTn),
                dev.ide_csn.eq(self.io_IDE_CSn),
                dev.ide_rd.eq(self.io_IDE_RD),
                dev.ide_rd_h.eq(self.io_IDE_RD_H),
                dev.ide_wr_h.eq(self.io_IDE_WR_H),
                dev.ide_wr.eq(self.io_IDE_WR),
                dev.ide_addr.eq(self.io_IDE_ADDR),
                dev.ide_data_in.eq(self.io_IDE_DATA_IN),
                dev.ide_dmackn.eq(self.io_IDE_DMACKn),
                dev.audio_tick.eq(self.io_audio_tick),
            ]

        # Only the selected device drives the IDE data bus and INTRQ; same
        # for DMARQ since DMA cycles are inherently per-device. IORDY is
        # AND'd: each device's iordy defaults to 1 (ready) and only the
        # active device pulls it low to pace its own DMA bursts.
        m.d.comb += [
            self.io_IDE_DATA_OUT.eq(
                Mux(dev1.ide_data_out_en, dev1.ide_data_out, dev0.ide_data_out)
            ),
            self.io_IDE_INTRQ.eq(dev0.ide_intrq | dev1.ide_intrq),
            self.io_IDE_DMARQ.eq(dev0.ide_dmarq | dev1.ide_dmarq),
            self.io_IDE_IORDY.eq(dev0.ide_iordy & dev1.ide_iordy),
        ]

        # MCU bus: high address bit picks the device window.
        mcu_dev_sel = self.io_MCU_ADDR[7]

        for dev in (dev0, dev1):
            m.d.comb += [
                dev.mcu_rd.eq(self.io_MCU_RD),
                dev.mcu_wr.eq(self.io_MCU_WR),
                dev.mcu_bs.eq(self.io_MCU_BS),
                dev.mcu_addr.eq(self.io_MCU_ADDR[:7]),
                dev.mcu_data_in.eq(self.io_MCU_DATA_IN),
            ]
        m.d.comb += [
            dev0.mcu_cs.eq(self.io_MCU_CS & ~mcu_dev_sel),
            dev1.mcu_cs.eq(self.io_MCU_CS & mcu_dev_sel),
            self.io_MCU_DATA_OUT.eq(
                Mux(mcu_dev_sel, dev1.mcu_data_out, dev0.mcu_data_out)
            ),
            self.io_MCU_IRQ.eq(dev0.mcu_irq | dev1.mcu_irq),

            # CDDA block: registers through the primary device's window,
            # serial audio straight out. CDCLK is the wrapper's business.
            # EMPH follows iceGDROM: high only for pre-emphasis tracks,
            # which no GD-ROM title uses, so it stays low.
            cdda.ctrl_we.eq(dev0.aux_ctrl_we),
            cdda.ctrl_wdata.eq(dev0.aux_wdata),
            cdda.push.eq(dev0.aux_push),
            cdda.push_data.eq(dev0.aux_wdata),
            dev0.aux_ctrl_rd.eq(cdda.ctrl_rdata),
            dev0.aux_level.eq(cdda.level),
            self.io_DC_CDCLK.eq(0),
            self.io_DC_SCK.eq(cdda.sck),
            self.io_DC_SDAT.eq(cdda.sdat),
            self.io_DC_LRCK.eq(cdda.lrck),
            self.io_DC_EMPH.eq(0),
        ]

        return m
