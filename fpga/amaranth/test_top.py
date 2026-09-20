#!/usr/bin/env python3
"""Two-device tests against the real top level (hw/OpenGDEMU.py).

test.py drives one IDEDevice in isolation, which cannot see anything that
depends on how the two devices share the bus -- who drives IDE_DATA_OUT, how
the DEV bit routes a command, whether one device's state leaks into the
other's register reads. Those are exactly the bugs that show up on hardware as
"the MCU says 0x58 and the Dreamcast reads 0xd8".

Run:  amaranth_env/bin/python3 test_top.py
"""

import os
import sys

from amaranth.sim import Simulator

from hw.OpenGDEMU import OpenGDEMUCore


ADDR_DATA = 0
ADDR_SECTOR_COUNT = 2
ADDR_LBA_LOW = 3
ADDR_LBA_MID = 4
ADDR_LBA_HIGH = 5
ADDR_DEVICE = 6
ADDR_COMMAND = 7
ADDR_ALT_STATUS = 6      # control block

CSN_CMD = 0b10
CSN_CTRL = 0b01
CSN_IDLE = 0b11

# MCU window: bit 7 of the address picks the device.
MCU_STATUS = 1
MCU_HW_INFO = 0
MCU_DMA_MODE = 19
SECONDARY = 0x80


async def ide_idle(ctx, dut):
    ctx.set(dut.io_IDE_CSn, CSN_IDLE)
    ctx.set(dut.io_IDE_RD, 0)
    ctx.set(dut.io_IDE_RD_H, 0)
    ctx.set(dut.io_IDE_WR, 0); ctx.set(dut.io_IDE_WR_H, 0)
    await ctx.tick()


async def release_reset(ctx, dut):
    ctx.set(dut.io_IDE_CSn, CSN_IDLE)
    ctx.set(dut.io_IDE_RD, 0)
    ctx.set(dut.io_IDE_RD_H, 0)
    ctx.set(dut.io_IDE_WR, 0); ctx.set(dut.io_IDE_WR_H, 0)
    ctx.set(dut.io_IDE_DMACKn, 1)
    ctx.set(dut.io_IDE_RSTn, 0)
    for _ in range(4):
        await ctx.tick()
    ctx.set(dut.io_IDE_RSTn, 1)
    for _ in range(60):
        await ctx.tick()


async def ide_write(ctx, dut, csn, addr, data):
    ctx.set(dut.io_IDE_CSn, csn)
    ctx.set(dut.io_IDE_ADDR, addr)
    ctx.set(dut.io_IDE_DATA_IN, data)
    await ctx.tick()
    ctx.set(dut.io_IDE_WR, 1); ctx.set(dut.io_IDE_WR_H, 1)
    for _ in range(5):
        await ctx.tick()
    ctx.set(dut.io_IDE_WR, 0); ctx.set(dut.io_IDE_WR_H, 0)
    for _ in range(5):
        await ctx.tick()
    await ide_idle(ctx, dut)


async def ide_read(ctx, dut, csn, addr):
    ctx.set(dut.io_IDE_CSn, csn)
    ctx.set(dut.io_IDE_ADDR, addr)
    await ctx.tick()
    ctx.set(dut.io_IDE_RD, 1)
    ctx.set(dut.io_IDE_RD_H, 1)
    for _ in range(4):
        await ctx.tick()
    value = ctx.get(dut.io_IDE_DATA_OUT)
    ctx.set(dut.io_IDE_RD, 0)
    ctx.set(dut.io_IDE_RD_H, 0)
    for _ in range(4):
        await ctx.tick()
    await ide_idle(ctx, dut)
    return value


async def mcu_idle(ctx, dut):
    ctx.set(dut.io_MCU_CS, 0)
    ctx.set(dut.io_MCU_RD, 0)
    ctx.set(dut.io_MCU_WR, 0)
    await ctx.tick()


async def mcu_write(ctx, dut, addr, data):
    ctx.set(dut.io_MCU_CS, 1)
    ctx.set(dut.io_MCU_ADDR, addr)
    ctx.set(dut.io_MCU_DATA_IN, data)
    ctx.set(dut.io_MCU_WR, 1)
    await ctx.tick()
    await ctx.tick()
    ctx.set(dut.io_MCU_WR, 0)
    await ctx.tick()
    await mcu_idle(ctx, dut)


async def mcu_read(ctx, dut, addr):
    ctx.set(dut.io_MCU_CS, 1)
    ctx.set(dut.io_MCU_ADDR, addr)
    ctx.set(dut.io_MCU_RD, 1)
    await ctx.tick()
    await ctx.tick()
    value = ctx.get(dut.io_MCU_DATA_OUT)
    ctx.set(dut.io_MCU_RD, 0)
    await ctx.tick()
    await mcu_idle(ctx, dut)
    return value


# -- testbenches ------------------------------------------------------------

async def tb_secondary_status_not_masked_by_primary(ctx, dut):
    """With DEV=1, an ALTSTATUS read must return the secondary's status.

    On hardware the MCU read the secondary's status register back as 0x58
    (BSY clear, DRQ set) while the Dreamcast read 0xd8 on the same bus --
    0x58 with the primary's BSY OR'd in. Both devices see every IDE strobe;
    only `device_selected` keeps the deselected one off the bus.
    """
    await release_reset(ctx, dut)

    # Select device 1 and give the secondary a command, so it is the one in
    # a data-out phase while the primary sits in its post-reset state.
    await ide_write(ctx, dut, CSN_CMD, ADDR_DEVICE, 0x10)
    await ide_write(ctx, dut, CSN_CMD, ADDR_COMMAND, 0x30)   # WRITE SECTORS

    # MCU answers on the secondary's window: DRQ up, BSY down, then ack.
    await mcu_write(ctx, dut, SECONDARY | MCU_DMA_MODE, 4)   # arm write flow control
    await mcu_write(ctx, dut, SECONDARY | MCU_STATUS, 0x58)
    await mcu_write(ctx, dut, SECONDARY | MCU_HW_INFO, 0)
    for _ in range(4):
        await ctx.tick()

    mcu_view = await mcu_read(ctx, dut, SECONDARY | MCU_STATUS) & 0xFF
    ide_view = await ide_read(ctx, dut, CSN_CTRL, ADDR_ALT_STATUS) & 0xFF

    assert mcu_view == 0x58, f"MCU should see 0x58, got {mcu_view:#04x}"
    assert ide_view == mcu_view, (
        f"IDE-side ALTSTATUS should match the selected device: "
        f"MCU sees {mcu_view:#04x}, IDE sees {ide_view:#04x}"
    )


async def tb_primary_status_when_dev0(ctx, dut):
    """The mirror case: with DEV=0 the primary answers, not the secondary."""
    await release_reset(ctx, dut)
    await ide_write(ctx, dut, CSN_CMD, ADDR_DEVICE, 0x00)

    # Put a distinctive status on each device so a mix-up is visible.
    await mcu_write(ctx, dut, MCU_STATUS, 0x50)
    await mcu_write(ctx, dut, SECONDARY | MCU_STATUS, 0xD8)
    for _ in range(4):
        await ctx.tick()

    ide_view = await ide_read(ctx, dut, CSN_CTRL, ADDR_ALT_STATUS) & 0xFF
    assert ide_view == 0x50, (
        f"with DEV=0 the primary (0x50) must answer, got {ide_view:#04x}"
    )


async def tb_write_command_routes_to_selected_device(ctx, dut):
    """A command written with DEV=1 must land on the secondary only."""
    await release_reset(ctx, dut)
    await ide_write(ctx, dut, CSN_CMD, ADDR_DEVICE, 0x10)
    await ide_write(ctx, dut, CSN_CMD, ADDR_COMMAND, 0x30)
    for _ in range(4):
        await ctx.tick()

    prim = await mcu_read(ctx, dut, MCU_HW_INFO)
    sec = await mcu_read(ctx, dut, SECONDARY | MCU_HW_INFO)
    assert (sec >> 1) & 1, "secondary should have COMMAND_PEND set"
    assert not ((prim >> 1) & 1), "primary must not see a DEV=1 command"



async def tb_read_fifo_word_order(ctx, dut):
    """Words pushed by the MCU must come out of the IDE data register in order.

    The read path now goes memory -> FIFO output register -> skid -> pins, to
    keep the host from ever sampling the memory output directly. Every stage is
    a chance to present the wrong word, and on hardware that costs a 20-minute
    build-and-flash to notice, so check the ordering here.
    """
    await release_reset(ctx, dut)
    await ide_write(ctx, dut, CSN_CMD, ADDR_DEVICE, 0x10)   # select device 1
    await ide_write(ctx, dut, CSN_CMD, ADDR_COMMAND, 0x20)  # READ SECTORS

    N = 12
    for i in range(N):
        await mcu_write(ctx, dut, SECONDARY | 9, 0x1000 + i)
    # Let the prefetch settle.
    for _ in range(8):
        await ctx.tick()

    # Arm PIO read flow control + DRQ, as the firmware does.
    await mcu_write(ctx, dut, SECONDARY | MCU_DMA_MODE, 8)
    await mcu_write(ctx, dut, SECONDARY | MCU_STATUS, 0x58)
    await mcu_write(ctx, dut, SECONDARY | MCU_HW_INFO, 0)
    for _ in range(4):
        await ctx.tick()

    got = []
    for _ in range(N):
        got.append(await ide_read(ctx, dut, CSN_CMD, ADDR_DATA) & 0xFFFF)
    want = [0x1000 + i for i in range(N)]
    assert got == want, f"read FIFO order wrong:\n  got  {[hex(v) for v in got]}\n  want {[hex(v) for v in want]}"



async def ide_dma_read(ctx, dut):
    """One DMA data read: DMACK asserted, CSn idle, RD pulsed."""
    ctx.set(dut.io_IDE_CSn, CSN_IDLE)
    ctx.set(dut.io_IDE_DMACKn, 0)
    await ctx.tick()
    ctx.set(dut.io_IDE_RD, 1)
    ctx.set(dut.io_IDE_RD_H, 1)
    for _ in range(3):
        await ctx.tick()
    value = ctx.get(dut.io_IDE_DATA_OUT)
    ctx.set(dut.io_IDE_RD, 0)
    ctx.set(dut.io_IDE_RD_H, 0)
    for _ in range(3):
        await ctx.tick()
    return value


async def tb_dma_read_word_order(ctx, dut):
    """DMA reads must deliver the staged words in order, with none repeated.

    The earlier ordering test only drove PIO. A skid-register attempt passed
    that and still shipped garbage over DMA on hardware, because the DMA pop is
    a different condition (DMACK held, address decode inactive) and was never
    simulated. Cover it here, where a mistake costs seconds rather than a
    twenty-minute build-flash-run.
    """
    await release_reset(ctx, dut)
    await ide_write(ctx, dut, CSN_CMD, ADDR_DEVICE, 0x10)
    await ide_write(ctx, dut, CSN_CMD, ADDR_COMMAND, 0xC8)   # READ DMA

    N = 16
    for i in range(N):
        await mcu_write(ctx, dut, SECONDARY | 9, 0x2000 + i)
    await mcu_write(ctx, dut, SECONDARY | MCU_DMA_MODE, 1)   # DMA to host
    await mcu_write(ctx, dut, SECONDARY | MCU_STATUS, 0x50)
    await mcu_write(ctx, dut, SECONDARY | MCU_HW_INFO, 0)
    for _ in range(8):
        await ctx.tick()

    got = [await ide_dma_read(ctx, dut) & 0xFFFF for _ in range(N)]
    ctx.set(dut.io_IDE_DMACKn, 1)
    want = [0x2000 + i for i in range(N)]
    assert got == want, (
        "DMA read order wrong:\n"
        f"  got  {[hex(v) for v in got]}\n  want {[hex(v) for v in want]}"
    )


TESTBENCHES = [
    ("dma_read_word_order", tb_dma_read_word_order),
    ("read_fifo_word_order", tb_read_fifo_word_order),
    ("secondary_status_not_masked_by_primary", tb_secondary_status_not_masked_by_primary),
    ("primary_status_when_dev0", tb_primary_status_when_dev0),
    ("write_command_routes_to_selected_device", tb_write_command_routes_to_selected_device),
]


def run_one(name, tb):
    dut = OpenGDEMUCore()
    sim = Simulator(dut)
    sim.add_clock(1e-7)

    async def wrapper(ctx):
        await tb(ctx, dut)

    sim.add_testbench(wrapper)
    os.makedirs("vcd", exist_ok=True)
    with sim.write_vcd(f"vcd/top_{name}.vcd"):
        sim.run()


def main() -> None:
    failures = 0
    for name, tb in TESTBENCHES:
        try:
            run_one(name, tb)
            print(f"  ok    {name}")
        except AssertionError as e:
            print(f"  FAIL  {name}: {e}")
            failures += 1
    print()
    print(f"all {len(TESTBENCHES)} test(s) passed" if not failures
          else f"{failures} test(s) failed")
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
