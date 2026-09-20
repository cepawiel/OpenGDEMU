#!/usr/bin/env python3
"""Simulation tests for the CDDA serialiser (hw/cdda.py).

Run from the fpga/amaranth directory:

    amaranth_env/bin/python3 test_cdda.py

Two clocks: `sync` at 48 MHz for the MCU-facing FIFO, `audio` at
11.2896 MHz for the serialiser. The receiver model below samples SDAT on
each SCK rising edge, exactly as the AICA would, and reassembles frames.
"""

import os
import sys

from amaranth import *
from amaranth.sim import Simulator

from hw.cdda import CDDAOutput
from hw.OpenGDEMU import OpenGDEMUCore


class Harness(Elaboratable):
    """CDDAOutput with the `audio` domain created for it."""

    def __init__(self):
        self.dut = CDDAOutput(depth=16)

    def elaborate(self, platform):
        m = Module()
        m.domains.audio = ClockDomain("audio", reset_less=True)
        m.submodules.dut = self.dut
        return m


async def ctrl_write(ctx, dut, value):
    ctx.set(dut.ctrl_wdata, value)
    ctx.set(dut.ctrl_we, 1)
    await ctx.tick()
    await ctx.tick()
    ctx.set(dut.ctrl_we, 0)
    await ctx.tick()


async def push(ctx, dut, word):
    ctx.set(dut.push_data, word)
    ctx.set(dut.push, 1)
    await ctx.tick()
    ctx.set(dut.push, 0)
    await ctx.tick()


async def capture_frame(ctx, dut):
    """Wait for the next LRCK rising edge, then collect 64 SDAT bits
    (sampled on SCK rising edges). Returns (left, right, lrck_bits)."""
    # Align to the start of a frame: LRCK low -> high.
    while ctx.get(dut.lrck):
        await ctx.tick("audio")
    while not ctx.get(dut.lrck):
        await ctx.tick("audio")
    bits = []
    lrcks = []
    prev_sck = ctx.get(dut.sck)
    while len(bits) < 64:
        await ctx.tick("audio")
        sck = ctx.get(dut.sck)
        if sck and not prev_sck:
            bits.append(ctx.get(dut.sdat))
            lrcks.append(ctx.get(dut.lrck))
        prev_sck = sck
    # Frame is [16 zero][16 left][16 zero][16 right], MSB first.
    assert bits[0:16] == [0] * 16, f"left pad not zero: {bits[0:16]}"
    assert bits[32:48] == [0] * 16, f"right pad not zero: {bits[32:48]}"
    left = int("".join(str(b) for b in bits[16:32]), 2)
    right = int("".join(str(b) for b in bits[48:64]), 2)
    return left, right, lrcks


async def tb_serialiser(ctx, dut):
    # Disabled: clocks run, data is silence.
    l, r, lrcks = await capture_frame(ctx, dut)
    assert (l, r) == (0, 0), f"silence expected while disabled, got {l:#x} {r:#x}"
    assert lrcks[:32] == [1] * 32 and lrcks[32:] == [0] * 32, "LRCK: high for left, low for right"

    # Enable, queue two stereo pairs.
    await ctrl_write(ctx, dut, 0x01)
    for w in (0x1234, 0xABCD, 0x8000, 0x7FFF):
        await push(ctx, dut, w)
    assert ctx.get(dut.level) in (2, 3, 4)  # the prefetch may already have taken a pair

    frames = []
    for _ in range(4):
        l, r, _ = await capture_frame(ctx, dut)
        frames.append((l, r))
    # The first captured frame may still be the silent one that was in
    # flight when we pushed; the two data pairs must follow in order.
    i = frames.index((0x1234, 0xABCD))
    assert frames[i + 1] == (0x8000, 0x7FFF), f"frames: {[(hex(a), hex(b)) for a, b in frames]}"
    # Then the FIFO is dry: silence, and the underflow flag/counter say so.
    assert frames[i + 2] == (0, 0), f"frames: {[(hex(a), hex(b)) for a, b in frames]}"
    await ctx.tick()
    st = ctx.get(dut.ctrl_rdata)
    assert st & 0x03 == 0x03, f"expected enable|underflow, got {st:#x}"
    assert (st >> 8) >= 1, f"underflow count missing: {st:#x}"
    assert ctx.get(dut.level) == 0

    # Clearing the flag resets the counter; disabling silences.
    await ctrl_write(ctx, dut, 0x03)
    await ctx.tick()
    assert ctx.get(dut.ctrl_rdata) == 0x0001, f"got {ctx.get(dut.ctrl_rdata):#x}"
    await push(ctx, dut, 0x0101)
    await push(ctx, dut, 0x0202)
    await ctrl_write(ctx, dut, 0x04)  # flush: enable bit written 0 too
    await ctx.tick()
    assert ctx.get(dut.level) == 0, "flush should empty the FIFO"
    assert ctx.get(dut.ctrl_rdata) & 1 == 0


async def tb_odd_word_waits_for_pair(ctx, dut):
    """A lone left word must not be sent until its right word arrives."""
    await ctrl_write(ctx, dut, 0x01)
    await push(ctx, dut, 0x4444)
    l, r, _ = await capture_frame(ctx, dut)
    l2, r2, _ = await capture_frame(ctx, dut)
    assert (l, r) == (0, 0) and (l2, r2) == (0, 0)
    assert ctx.get(dut.level) == 1
    await push(ctx, dut, 0x5555)
    got = [await capture_frame(ctx, dut) for _ in range(3)]
    assert any(g[:2] == (0x4444, 0x5555) for g in got), got


# -- Top-level register decode ------------------------------------------------

async def top_mcu_write(ctx, core, addr, data):
    ctx.set(core.io_MCU_CS, 1)
    ctx.set(core.io_MCU_ADDR, addr)
    ctx.set(core.io_MCU_DATA_IN, data)
    ctx.set(core.io_MCU_WR, 1)
    await ctx.tick()
    await ctx.tick()
    ctx.set(core.io_MCU_WR, 0)
    await ctx.tick()
    ctx.set(core.io_MCU_CS, 0)
    await ctx.tick()


async def top_mcu_read(ctx, core, addr):
    ctx.set(core.io_MCU_CS, 1)
    ctx.set(core.io_MCU_ADDR, addr)
    ctx.set(core.io_MCU_RD, 1)
    await ctx.tick()
    await ctx.tick()
    value = ctx.get(core.io_MCU_DATA_OUT)
    ctx.set(core.io_MCU_RD, 0)
    await ctx.tick()
    ctx.set(core.io_MCU_CS, 0)
    await ctx.tick()
    return value


async def tb_top_level_slots(ctx, core):
    ctx.set(core.io_MCU_BS, 0b11)
    ctx.set(core.io_IDE_CSn, 0b11)
    # Let the post-reset counters settle (they don't matter here).
    for _ in range(40):
        await ctx.tick()
    assert await top_mcu_read(ctx, core, 29) == 0
    assert await top_mcu_read(ctx, core, 30) == 0
    # Pushes land in the audio FIFO, not the IDE read FIFO.
    for w in (1, 2, 3, 4):
        await top_mcu_write(ctx, core, 31, w)
    assert await top_mcu_read(ctx, core, 30) == 4, "audio level"
    assert await top_mcu_read(ctx, core, 10) == 0, "IDE read FIFO must be untouched"
    # Secondary window (addr bit 7) does not reach the audio block.
    await top_mcu_write(ctx, core, 0x80 | 31, 9)
    assert await top_mcu_read(ctx, core, 30) == 4
    await top_mcu_write(ctx, core, 29, 0x01)
    assert await top_mcu_read(ctx, core, 29) & 1 == 1
    # Prefetch takes one pair as soon as enabled.
    for _ in range(4):
        await ctx.tick()
    assert await top_mcu_read(ctx, core, 30) == 2
    await top_mcu_write(ctx, core, 29, 0x04)
    assert await top_mcu_read(ctx, core, 30) == 0


def run_harness(name, tb):
    h = Harness()
    sim = Simulator(h)
    sim.add_clock(1 / 48e6, domain="sync")
    sim.add_clock(1 / 11.2896e6, domain="audio")

    async def wrapper(ctx):
        await tb(ctx, h.dut)

    sim.add_testbench(wrapper)
    os.makedirs("vcd", exist_ok=True)
    with sim.write_vcd(f"vcd/{name}.vcd"):
        sim.run()


def run_top(name, tb):
    core = OpenGDEMUCore()
    sim = Simulator(core)
    sim.add_clock(1 / 48e6, domain="sync")

    # The core's audio domain is clocked from a port, so toggle that.
    async def audio_clock(ctx):
        while True:
            ctx.set(core.io_AUDIO_CLK, 1)
            await ctx.delay(0.5 / 11.2896e6)
            ctx.set(core.io_AUDIO_CLK, 0)
            await ctx.delay(0.5 / 11.2896e6)

    sim.add_process(audio_clock)

    async def wrapper(ctx):
        await tb(ctx, core)

    sim.add_testbench(wrapper)
    os.makedirs("vcd", exist_ok=True)
    with sim.write_vcd(f"vcd/{name}.vcd"):
        sim.run()


TESTS = [
    ("cdda_serialiser", run_harness, tb_serialiser),
    ("cdda_odd_word_waits_for_pair", run_harness, tb_odd_word_waits_for_pair),
    ("cdda_top_level_slots", run_top, tb_top_level_slots),
]


def main() -> None:
    failures = 0
    for name, runner, tb in TESTS:
        try:
            runner(name, tb)
            print(f"  ok    {name}")
        except AssertionError as e:
            print(f"  FAIL  {name}: {e}")
            failures += 1
    if failures:
        print(f"\n{failures} test(s) failed")
        sys.exit(1)
    print(f"\nall {len(TESTS)} test(s) passed")


if __name__ == "__main__":
    main()
