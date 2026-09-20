from amaranth import *


class IDEAddressDecode(Elaboratable):
    """Decode `CSn[1:0]` + `ADDR[2:0]` into one-hot ATA register selects.

    Reference: ATA-2 spec p.63-74.

    Command block (CSn = 0b10, i.e. CS1n=1 deasserted, CS0n=0 asserted):
      0  Data            (RW)
      1  Error           (R) / Features (W)
      2  Sector Count    (RW)
      3  LBA Low         (RW)
      4  LBA Mid         (RW)
      5  LBA High        (RW)
      6  Device          (RW)
      7  Status          (R) / Command (W)

    Control block (CSn = 0b01):
      6  Alt Status      (R) / Device Control (W)
      7  Debug test pad  (RW, custom)
    """

    def __init__(self):
        self.csn = Signal(2)
        self.addr = Signal(3)

        self.data = Signal()
        self.error = Signal()
        self.features = Signal()
        self.sector_count = Signal()
        self.lba_low = Signal()
        self.lba_mid = Signal()
        self.lba_high = Signal()
        self.device = Signal()
        self.status = Signal()
        self.command = Signal()
        self.alt_status = Signal()
        self.device_control = Signal()
        self.debug_testpad = Signal()

    def elaborate(self, platform):
        m = Module()

        cmd_block = (self.csn == 0b10)
        ctrl_block = (self.csn == 0b01)

        m.d.comb += [
            self.data.eq(cmd_block & (self.addr == 0)),
            self.error.eq(cmd_block & (self.addr == 1)),
            self.features.eq(cmd_block & (self.addr == 1)),
            self.sector_count.eq(cmd_block & (self.addr == 2)),
            self.lba_low.eq(cmd_block & (self.addr == 3)),
            self.lba_mid.eq(cmd_block & (self.addr == 4)),
            self.lba_high.eq(cmd_block & (self.addr == 5)),
            self.device.eq(cmd_block & (self.addr == 6)),
            self.status.eq(cmd_block & (self.addr == 7)),
            self.command.eq(cmd_block & (self.addr == 7)),
            self.alt_status.eq(ctrl_block & (self.addr == 6)),
            self.device_control.eq(ctrl_block & (self.addr == 6)),
            self.debug_testpad.eq(ctrl_block & (self.addr == 7)),
        ]
        return m
