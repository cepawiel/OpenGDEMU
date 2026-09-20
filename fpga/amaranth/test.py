#!/usr/bin/env python3
"""Simulation tests for the IDE register file (hw/ide_device.py).

Run from the fpga/amaranth directory:

    amaranth_env/bin/python3 test.py

Generates `vcd/test_ide_device.vcd` for waveform inspection.
"""

import os
import sys

from amaranth.sim import Simulator

from hw.ide_device import IDEDevice


# ATA register address constants (within the command block, CSn=0b10).
ADDR_DATA = 0
ADDR_ERROR = 1
ADDR_FEATURES = 1
ADDR_SECTOR_COUNT = 2
ADDR_LBA_LOW = 3
ADDR_LBA_MID = 4
ADDR_LBA_HIGH = 5
ADDR_DEVICE = 6
ADDR_STATUS = 7
ADDR_COMMAND = 7
# Control block (CSn=0b01).
ADDR_DEVICE_CONTROL = 6
ADDR_ALT_STATUS = 6

CSN_CMD = 0b10
CSN_CTRL = 0b01
CSN_IDLE = 0b11

# MCU window addresses.
MCU_HW_INFO = 0
MCU_STATUS = 1
MCU_DEVICE = 2
MCU_ERROR = 3
MCU_CMD_FEAT = 4
MCU_SECTOR_COUNT = 5
MCU_LBA_LOW_MID = 6
MCU_LBA_HIGH_LOWP = 7
MCU_LBA_MIDP_HIGHP = 8
MCU_READ_FIFO = 9
MCU_READ_FIFO_LEVEL = 10
MCU_WRITE_FIFO_LEVEL = 11
MCU_INTRQ_PENDING = 13
MCU_DEBUG = 12
MCU_DMA_MODE = 19


# -- Bus helpers ------------------------------------------------------------

async def ide_idle(ctx, dut):
    ctx.set(dut.ide_csn, CSN_IDLE)
    ctx.set(dut.ide_rd, 0); ctx.set(dut.ide_rd_h, 0)
    ctx.set(dut.ide_wr, 0); ctx.set(dut.ide_wr_h, 0)
    await ctx.tick()


async def release_reset(ctx, dut):
    """Hold ide_rstn low briefly, then deassert and wait for the post-reset
    BSY counter to expire. The few cycles of asserted reset are needed so
    the in-reset state machine fires at least once and loads the ATAPI
    signature; otherwise we'd transition straight from "boot" to "idle"
    without the signature ever getting written.

    Tests run with `post_reset_count_bits=4` (16-cycle counter) so the
    deassert wait settles in well under 50 cycles."""
    ctx.set(dut.ide_csn, CSN_IDLE)
    ctx.set(dut.ide_rd, 0); ctx.set(dut.ide_rd_h, 0)
    ctx.set(dut.ide_wr, 0); ctx.set(dut.ide_wr_h, 0)
    ctx.set(dut.ide_rstn, 0)
    for _ in range(4):
        await ctx.tick()
    ctx.set(dut.ide_rstn, 1)
    for _ in range(50):
        await ctx.tick()


async def ide_write(ctx, dut, csn, addr, data):
    ctx.set(dut.ide_csn, csn)
    ctx.set(dut.ide_addr, addr)
    ctx.set(dut.ide_data_in, data)
    await ctx.tick()
    ctx.set(dut.ide_wr, 1); ctx.set(dut.ide_wr_h, 1)
    # Hold the strobe long enough for the 2-FF synchronizer (3-cycle
    # latency) to register the rise and latch data + decode signals.
    for _ in range(5):
        await ctx.tick()
    ctx.set(dut.ide_wr, 0); ctx.set(dut.ide_wr_h, 0)
    # Hold idle long enough for the falling-edge to propagate through
    # sync and fire the write logic / FIFO push.
    for _ in range(5):
        await ctx.tick()
    await ide_idle(ctx, dut)


async def ide_read(ctx, dut, csn, addr):
    ctx.set(dut.ide_csn, csn)
    ctx.set(dut.ide_addr, addr)
    await ctx.tick()
    ctx.set(dut.ide_rd, 1); ctx.set(dut.ide_rd_h, 1)
    # Hold the strobe long enough for the 2-FF synchronizer (3-cycle
    # latency) to settle, then sample.
    for _ in range(4):
        await ctx.tick()
    value = ctx.get(dut.ide_data_out)
    ctx.set(dut.ide_rd, 0); ctx.set(dut.ide_rd_h, 0)
    # Hold idle long enough for the falling-edge to propagate through
    # sync and trigger the FIFO pop.
    for _ in range(4):
        await ctx.tick()
    await ide_idle(ctx, dut)
    return value


async def ide_dma_read(ctx, dut):
    """Simulate one host DMA word read: deassert CSn (DMA cycles bypass
    the CSn-decoded address path), pull DMACKn low, pulse RD."""
    ctx.set(dut.ide_csn, CSN_IDLE)
    ctx.set(dut.ide_dmackn, 0)
    await ctx.tick()
    ctx.set(dut.ide_rd, 1); ctx.set(dut.ide_rd_h, 1)
    for _ in range(4):
        await ctx.tick()
    value = ctx.get(dut.ide_data_out)
    ctx.set(dut.ide_rd, 0); ctx.set(dut.ide_rd_h, 0)
    for _ in range(4):
        await ctx.tick()
    ctx.set(dut.ide_dmackn, 1)
    await ctx.tick()
    return value


async def mcu_idle(ctx, dut):
    ctx.set(dut.mcu_cs, 0)
    ctx.set(dut.mcu_rd, 0)
    ctx.set(dut.mcu_wr, 0)
    await ctx.tick()


async def mcu_write(ctx, dut, addr, data):
    ctx.set(dut.mcu_cs, 1)
    ctx.set(dut.mcu_addr, addr)
    ctx.set(dut.mcu_data_in, data)
    ctx.set(dut.mcu_wr, 1)
    await ctx.tick()
    await ctx.tick()
    ctx.set(dut.mcu_wr, 0)
    await ctx.tick()
    await mcu_idle(ctx, dut)


async def mcu_read(ctx, dut, addr):
    ctx.set(dut.mcu_cs, 1)
    ctx.set(dut.mcu_addr, addr)
    ctx.set(dut.mcu_rd, 1)
    await ctx.tick()
    await ctx.tick()
    value = ctx.get(dut.mcu_data_out)
    ctx.set(dut.mcu_rd, 0)
    await ctx.tick()
    await mcu_idle(ctx, dut)
    return value


# -- Testbenches ------------------------------------------------------------

async def tb_default_state(ctx, dut):
    # While ide_rstn is asserted (low), the device sits in the post-reset
    # phase: BSY=1, ATAPI signature loaded, command_pend cleared.
    await ctx.tick()
    await ctx.tick()
    status = await mcu_read(ctx, dut, MCU_STATUS)
    assert status & 0x80, f"BSY should be set in reset, got {status:#06x}"

    debug = await mcu_read(ctx, dut, MCU_DEBUG)
    assert debug == 0xC0DE, f"debug should be 0xC0DE, got {debug:#06x}"

    hw_info = await mcu_read(ctx, dut, MCU_HW_INFO)
    assert (hw_info & 1) == 0, f"primary should report bit 0 = 0, got {hw_info:#06x}"
    assert (hw_info >> 1) & 1 == 0, f"COMMAND_PEND should be 0, got {hw_info:#06x}"


async def tb_post_reset_idle(ctx, dut):
    # After ide_rstn is released and the post-reset counter expires, the
    # device transitions to idle: BSY=0, DRDY=1, DSC=1; ATAPI signature
    # 01/14/EB stays in LBA_LOW/MID/HIGH for the host to read back.
    await release_reset(ctx, dut)

    status = await mcu_read(ctx, dut, MCU_STATUS)
    # BSY clear, DRDY+DSC set → 0x50.
    assert (status & 0xFF) == 0x50, (
        f"post-reset status should be 0x50 (DRDY|DSC), got {status:#06x}"
    )

    # IDE-side ALTSTATUS should match (no IRQ side-effects).
    ide_status = await ide_read(ctx, dut, CSN_CTRL, ADDR_ALT_STATUS)
    assert (ide_status & 0xFF) == 0x50, (
        f"IDE alt-status should be 0x50, got {ide_status:#06x}"
    )

    # Sega GD-ROM signature: IDE-side reads of LBA_LOW/MID/HIGH should
    # return 0x81 / 0x14 / 0xEB. LBA_LOW is the Sega SECNR (disk type
    # 0x80 GD-ROM | status 0x01 ready) — the BIOS gives up at disc
    # validation if it's set to "paused" (0x82) instead.
    lo = await ide_read(ctx, dut, CSN_CMD, ADDR_LBA_LOW)
    mid = await ide_read(ctx, dut, CSN_CMD, ADDR_LBA_MID)
    hi = await ide_read(ctx, dut, CSN_CMD, ADDR_LBA_HIGH)
    assert (lo & 0xFF) == 0x81, f"LBA_LOW post-reset = 0x81, got {lo:#06x}"
    assert (mid & 0xFF) == 0x14, f"LBA_MID post-reset = 0x14, got {mid:#06x}"
    assert (hi & 0xFF) == 0xEB, f"LBA_HIGH post-reset = 0xEB, got {hi:#06x}"


async def tb_ide_write_lba(ctx, dut):
    await release_reset(ctx, dut)
    # IDE writes LBA Low/Mid/High; MCU reads them back.
    await ide_write(ctx, dut, CSN_CMD, ADDR_LBA_LOW, 0x12)
    await ide_write(ctx, dut, CSN_CMD, ADDR_LBA_MID, 0x34)
    await ide_write(ctx, dut, CSN_CMD, ADDR_LBA_HIGH, 0x56)

    # MCU reads LBA Low / Mid (split low/high byte in one 16-bit word).
    low_mid = await mcu_read(ctx, dut, MCU_LBA_LOW_MID)
    assert low_mid == 0x3412, f"LBA Low/Mid should be 0x3412, got {low_mid:#06x}"

    # MCU reads LBA High in the next word (low byte).
    high = await mcu_read(ctx, dut, MCU_LBA_HIGH_LOWP)
    assert (high & 0xFF) == 0x56, f"LBA High should be 0x56, got {high:#06x}"


async def tb_ide_writes_command_sets_pend(ctx, dut):
    await release_reset(ctx, dut)
    # IDE writes a Command — should latch BSY=1, COMMAND_PEND=1.
    await ide_write(ctx, dut, CSN_CMD, ADDR_COMMAND, 0x00EC)  # IDENTIFY DEVICE

    # MCU sees COMMAND_PEND in HW_INFO.
    hw_info = await mcu_read(ctx, dut, MCU_HW_INFO)
    assert (hw_info >> 1) & 1, f"COMMAND_PEND should be 1, got {hw_info:#06x}"

    # MCU sees Command in the cmd/features word.
    cmd_feat = await mcu_read(ctx, dut, MCU_CMD_FEAT)
    assert (cmd_feat & 0xFF) == 0xEC, f"Command should be 0xEC, got {cmd_feat:#06x}"

    # MCU IRQ should be asserted (after the 4-cycle shift register).
    for _ in range(8):
        await ctx.tick()
    assert ctx.get(dut.mcu_irq) == 1, "mcu_irq should be high while COMMAND_PEND"

    # MCU clears COMMAND_PEND by writing HW_INFO bit 1 = 0.
    await mcu_write(ctx, dut, MCU_HW_INFO, 0x0000)
    hw_info = await mcu_read(ctx, dut, MCU_HW_INFO)
    assert (hw_info >> 1) & 1 == 0, f"COMMAND_PEND should be cleared, got {hw_info:#06x}"

    # IRQ should go away after the shift register propagates.
    for _ in range(8):
        await ctx.tick()
    assert ctx.get(dut.mcu_irq) == 0, "mcu_irq should drop once COMMAND_PEND clears"


async def tb_ide_reads_status(ctx, dut):
    await release_reset(ctx, dut)
    # MCU sets a known status pattern.
    # Bit layout: BSY[7] DRDY[6] DF[5] DRQ[3] ERR[0]
    pattern = (1 << 6) | (1 << 3)  # DRDY + DRQ
    await mcu_write(ctx, dut, MCU_STATUS, pattern)

    # IDE reads the Status register — must match.
    status = await ide_read(ctx, dut, CSN_CMD, ADDR_STATUS)
    assert (status & 0xFF) == pattern, (
        f"IDE Status read should be {pattern:#04x}, got {status:#06x}"
    )

    # Alt Status (control block) should match.
    alt = await ide_read(ctx, dut, CSN_CTRL, ADDR_ALT_STATUS)
    assert (alt & 0xFF) == pattern, (
        f"IDE Alt Status read should be {pattern:#04x}, got {alt:#06x}"
    )


async def tb_nien_masks_intrq(ctx, dut):
    await release_reset(ctx, dut)
    # Ensure clean state: no pending command.
    await mcu_write(ctx, dut, MCU_HW_INFO, 0x0000)
    for _ in range(8):
        await ctx.tick()

    # IDE writes Device Control with nIEN=1 (bit 1).
    await ide_write(ctx, dut, CSN_CTRL, ADDR_DEVICE_CONTROL, 0x02)

    # MCU flags an interrupt — would normally raise INTRQ.
    await mcu_write(ctx, dut, MCU_INTRQ_PENDING, 0x0001)
    for _ in range(4):
        await ctx.tick()
    assert ctx.get(dut.ide_intrq) == 0, "INTRQ should be masked by nIEN"

    # Clear nIEN, INTRQ should now be asserted.
    await ide_write(ctx, dut, CSN_CTRL, ADDR_DEVICE_CONTROL, 0x00)
    for _ in range(4):
        await ctx.tick()
    assert ctx.get(dut.ide_intrq) == 1, "INTRQ should be high once nIEN=0"

    # And a Status read clears it.
    await ide_read(ctx, dut, CSN_CMD, ADDR_STATUS)
    assert ctx.get(dut.ide_intrq) == 0, "Status read should clear INTRQ"


async def tb_command_write_does_not_intrq(ctx, dut):
    """Latching a command must not assert INTRQ: the device is BSY and ATA
    forbids INTRQ while BSY=1. For PACKET that also means no IRQ for the
    CDB phase -- a GD-ROM sets DRQ for the CDB silently. Only the MCU's
    explicit pulse interrupts the host."""
    await release_reset(ctx, dut)
    await mcu_write(ctx, dut, MCU_HW_INFO, 0x0000)
    await ide_write(ctx, dut, CSN_CMD, ADDR_COMMAND, 0x00A0)
    for _ in range(8):
        await ctx.tick()
    assert (await mcu_read(ctx, dut, MCU_HW_INFO)) & 0x2, "command should be pending"
    assert ctx.get(dut.ide_intrq) == 0, "INTRQ must stay low on a command write"

    # MCU opens the CDB phase (DRQ=1, ack) -- still no IRQ.
    await mcu_write(ctx, dut, MCU_STATUS, 0x58)
    await mcu_write(ctx, dut, MCU_HW_INFO, 0x0000)
    for _ in range(4):
        await ctx.tick()
    assert ctx.get(dut.ide_intrq) == 0, "INTRQ must stay low for the CDB phase"

    await mcu_write(ctx, dut, MCU_INTRQ_PENDING, 0x0001)
    for _ in range(4):
        await ctx.tick()
    assert ctx.get(dut.ide_intrq) == 1, "MCU pulse should assert INTRQ"


async def tb_status_write_asserts_intrq(ctx, dut):
    """Bit 8 of a Status write raises INTRQ in the same clock as the new
    status. Posting them separately (status, then slot 13) let a host that
    polls Alt Status see BSY drop, read Status and write its next command
    before INTRQ rose; the IRQ then hit that command as a spurious one and
    Quake III's WinCE driver slept through the real DRQ interrupt. The
    ide_intrq must never be observable high while the old (BSY) status is."""
    await release_reset(ctx, dut)
    await mcu_write(ctx, dut, MCU_HW_INFO, 0x0000)
    # Plain status write: no IRQ side effect.
    await mcu_write(ctx, dut, MCU_STATUS, 0x00D0)
    for _ in range(4):
        await ctx.tick()
    assert ctx.get(dut.ide_intrq) == 0, "status write without bit 8 must not IRQ"

    # Status + IRQ in one write; watch every clock for a cycle where INTRQ
    # is high but the host-visible status still shows BSY.
    ctx.set(dut.mcu_cs, 1)
    ctx.set(dut.mcu_addr, MCU_STATUS)
    ctx.set(dut.mcu_data_in, 0x0150)
    ctx.set(dut.mcu_wr, 1)
    ctx.set(dut.ide_csn, CSN_CTRL)
    ctx.set(dut.ide_addr, ADDR_ALT_STATUS)
    for _ in range(6):
        await ctx.tick()
        bsy = (ctx.get(dut.ide_data_out) >> 7) & 1
        if ctx.get(dut.ide_intrq):
            assert bsy == 0, "INTRQ rose while status still showed BSY"
    ctx.set(dut.mcu_wr, 0)
    await ctx.tick()
    await mcu_idle(ctx, dut)
    await ide_idle(ctx, dut)
    assert ctx.get(dut.ide_intrq) == 1, "bit 8 should assert INTRQ"
    assert (await mcu_read(ctx, dut, MCU_STATUS)) & 0xFF == 0x50, "status bits must land too"

    # A Status read still clears it, as before.
    await ide_read(ctx, dut, CSN_CMD, ADDR_STATUS)
    for _ in range(4):
        await ctx.tick()
    assert ctx.get(dut.ide_intrq) == 0, "Status read must clear INTRQ"


async def tb_command_write_clears_intrq(ctx, dut):
    """A completion IRQ the host never acknowledged with a Status read must
    not survive the next Command write (ATA-5 DI1). The BIOS's DMA path
    reads Status before we post the completion, so without this the stale
    INTRQ rides into the next command and an interrupt-driven driver
    unmasking G1 there sees a spurious IRQ."""
    await release_reset(ctx, dut)
    await mcu_write(ctx, dut, MCU_HW_INFO, 0x0000)
    await mcu_write(ctx, dut, MCU_INTRQ_PENDING, 0x0001)
    for _ in range(4):
        await ctx.tick()
    assert ctx.get(dut.ide_intrq) == 1, "MCU pulse should assert INTRQ"

    await ide_write(ctx, dut, CSN_CMD, ADDR_COMMAND, 0x00A0)
    for _ in range(8):
        await ctx.tick()
    assert ctx.get(dut.ide_intrq) == 0, "Command write must negate INTRQ"
    assert (await mcu_read(ctx, dut, MCU_INTRQ_PENDING)) & 1 == 0, \
        "intrq_pending must be cleared by the Command write"
    assert (await mcu_read(ctx, dut, MCU_HW_INFO)) & 0x2, "command should still be pending"


async def tb_packet_write_opens_cdb_phase(ctx, dut):
    """Writing 0xA0 must show DRQ=1/BSY=0 with IRR=0x01 within a few core
    clocks, with no MCU involvement. Sega Rally 2's WinCE driver polls Alt
    Status a handful of times right after the Command write; if it sees
    BSY it sleeps waiting for an interrupt that a CDB phase never raises.
    Firmware latency (~100 us) is far too slow for that; iceGDROM does it
    in its command IRQ, a real drive in hardware."""
    await release_reset(ctx, dut)
    await mcu_write(ctx, dut, MCU_HW_INFO, 0x0000)
    # Leave a stale error and IRR=0x03 from a "previous" command.
    await mcu_write(ctx, dut, 3, 0x24)
    await mcu_write(ctx, dut, 5, 0x0003)
    await mcu_write(ctx, dut, MCU_STATUS, 0x51)
    await ide_write(ctx, dut, CSN_CMD, ADDR_DEVICE, 0x00)
    await ide_write(ctx, dut, CSN_CMD, ADDR_COMMAND, 0x00A0)
    for _ in range(3):
        await ctx.tick()
    st = await ide_read(ctx, dut, CSN_CTRL, ADDR_ALT_STATUS) & 0xFF
    assert st == 0x58, f"Alt Status right after 0xA0 should be 0x58, got {st:#04x}"
    irr = await ide_read(ctx, dut, CSN_CMD, ADDR_SECTOR_COUNT) & 0xFF
    assert irr == 0x01, f"IRR should be C/D=1, got {irr:#04x}"
    er = await ide_read(ctx, dut, CSN_CMD, ADDR_ERROR) & 0xFF
    assert er == 0x00, f"Error should be cleared, got {er:#04x}"
    assert (await mcu_read(ctx, dut, MCU_HW_INFO)) & 0x2, "command should be pending for the MCU"
    assert ctx.get(dut.ide_intrq) == 0, "no INTRQ for the CDB phase"

    # A non-PACKET command still just goes BSY.
    await ide_write(ctx, dut, CSN_CMD, ADDR_COMMAND, 0x00EC)
    for _ in range(3):
        await ctx.tick()
    st = await ide_read(ctx, dut, CSN_CTRL, ADDR_ALT_STATUS) & 0xFF
    assert st & 0x80 and not (st & 0x08), f"IDENTIFY should show BSY, got {st:#04x}"


async def tb_srst_resets_device(ctx, dut):
    """SRST in Device Control behaves like a hard reset: BSY while set,
    the transfer in flight is dropped (FIFOs, DMA mode, packet phase,
    pending IRQ), the signature is reloaded, and the device comes back
    idle once SRST clears and the BSY countdown expires. The MCU sees a
    sticky reset flag it clears by writing HW_INFO bit 2."""
    await release_reset(ctx, dut)
    await mcu_write(ctx, dut, MCU_HW_INFO, 0x0004)   # clear the boot reset
    assert not ((await mcu_read(ctx, dut, MCU_HW_INFO)) & 0x4), "reset flag should clear"

    # Mid-transfer: data staged for the host, DMA armed, IRQ pending.
    await ide_write(ctx, dut, CSN_CMD, ADDR_COMMAND, 0x00A0)
    await mcu_write(ctx, dut, MCU_HW_INFO, 0x0000)
    for w in (0x1111, 0x2222, 0x3333):
        await mcu_write(ctx, dut, MCU_READ_FIFO, w)
    await mcu_write(ctx, dut, MCU_DMA_MODE, 0x01)
    await mcu_write(ctx, dut, MCU_STATUS, 0x58)
    await mcu_write(ctx, dut, MCU_INTRQ_PENDING, 0x0001)
    for _ in range(4):
        await ctx.tick()
    assert ctx.get(dut.ide_intrq) == 1

    # Host sets SRST.
    await ide_write(ctx, dut, CSN_CTRL, ADDR_DEVICE_CONTROL, 0x04)
    for _ in range(4):
        await ctx.tick()
    st = await ide_read(ctx, dut, CSN_CTRL, ADDR_ALT_STATUS) & 0xFF
    assert st & 0x80 and not (st & 0x08), f"SRST should give BSY=1 DRQ=0, got {st:#04x}"
    assert ctx.get(dut.ide_intrq) == 0, "reset must drop the pending IRQ"
    hw = await mcu_read(ctx, dut, MCU_HW_INFO)
    assert hw & 0x4, f"reset_seen should be set, got {hw:#06x}"
    assert hw & 0x10, f"in_reset should read live, got {hw:#06x}"
    assert not ((await mcu_read(ctx, dut, MCU_DMA_MODE)) & 0x3), "DMA mode should clear"
    assert (await mcu_read(ctx, dut, MCU_READ_FIFO_LEVEL)) == 0, "read FIFO should be empty"

    # Host clears SRST: BSY persists through the countdown, then idle with
    # the signature.
    await ide_write(ctx, dut, CSN_CTRL, ADDR_DEVICE_CONTROL, 0x00)
    st = await ide_read(ctx, dut, CSN_CTRL, ADDR_ALT_STATUS) & 0xFF
    assert st & 0x80, f"BSY should hold right after SRST clears, got {st:#04x}"
    for _ in range(40):
        await ctx.tick()
    st = await ide_read(ctx, dut, CSN_CMD, ADDR_STATUS) & 0xFF
    assert st == 0x50, f"device should be idle after reset, got {st:#04x}"
    assert (await ide_read(ctx, dut, CSN_CMD, ADDR_LBA_MID)) & 0xFF == 0x14
    assert (await ide_read(ctx, dut, CSN_CMD, ADDR_LBA_HIGH)) & 0xFF == 0xEB
    # Sticky flag survives until the MCU clears it.
    assert (await mcu_read(ctx, dut, MCU_HW_INFO)) & 0x4
    await mcu_write(ctx, dut, MCU_HW_INFO, 0x0004)
    assert not ((await mcu_read(ctx, dut, MCU_HW_INFO)) & 0x4)


async def tb_sector_count_sliding(ctx, dut):
    await release_reset(ctx, dut)
    # IDE writes 0xAA, then 0xBB. The implementation slides high <- low,
    # low <- new (LBA48 HOB pattern). After two writes: high = 0xAA, low = 0xBB.
    await ide_write(ctx, dut, CSN_CMD, ADDR_SECTOR_COUNT, 0xAA)
    await ide_write(ctx, dut, CSN_CMD, ADDR_SECTOR_COUNT, 0xBB)

    # MCU reads the full 16-bit register.
    sc = await mcu_read(ctx, dut, MCU_SECTOR_COUNT)
    assert sc == 0xAABB, f"SectorCount sliding window: expected 0xAABB, got {sc:#06x}"


async def tb_primary_ignored_when_dev_set(ctx, dut):
    await release_reset(ctx, dut)
    # Primary device should NOT capture writes (other than Device) when
    # DEV=1. First, write Device with DEV=1 — both devices' DEVICE register
    # would update in the real system; in this single-device test, we just
    # write the local Device.
    await ide_write(ctx, dut, CSN_CMD, ADDR_DEVICE, 0x10)  # DEV=1
    # Now try to write LBA Low — primary should drop it.
    await ide_write(ctx, dut, CSN_CMD, ADDR_LBA_LOW, 0x77)
    # MCU reads LBALow — should hold the post-reset Sega SECNR byte
    # (0x81), not the dropped 0x77 from the IDE-side write.
    val = await mcu_read(ctx, dut, MCU_LBA_LOW_MID)
    assert (val & 0xFF) == 0x81, (
        f"primary should ignore LBALow write while DEV=1, got {val:#06x}"
    )


async def tb_data_out_en_gated_by_dev(ctx, dut):
    await release_reset(ctx, dut)
    # Set DEV=0 — primary selected. ide_data_out_en should rise on a read.
    await ide_write(ctx, dut, CSN_CMD, ADDR_DEVICE, 0x00)
    ctx.set(dut.ide_csn, CSN_CMD)
    ctx.set(dut.ide_addr, ADDR_STATUS)
    ctx.set(dut.ide_rd, 1); ctx.set(dut.ide_rd_h, 1)
    await ctx.tick()
    await ctx.tick()
    assert ctx.get(dut.ide_data_out_en) == 1, (
        "primary device should drive bus when DEV=0 and IDE reads"
    )
    ctx.set(dut.ide_rd, 0); ctx.set(dut.ide_rd_h, 0)
    await ide_idle(ctx, dut)

    # Set DEV=1 — primary deselected. ide_data_out_en should stay low.
    await ide_write(ctx, dut, CSN_CMD, ADDR_DEVICE, 0x10)
    ctx.set(dut.ide_csn, CSN_CMD)
    ctx.set(dut.ide_addr, ADDR_STATUS)
    ctx.set(dut.ide_rd, 1); ctx.set(dut.ide_rd_h, 1)
    await ctx.tick()
    await ctx.tick()
    assert ctx.get(dut.ide_data_out_en) == 0, (
        "primary device should NOT drive bus when DEV=1"
    )
    ctx.set(dut.ide_rd, 0); ctx.set(dut.ide_rd_h, 0)
    await ide_idle(ctx, dut)


async def tb_fifo_mcu_to_ide(ctx, dut):
    await release_reset(ctx, dut)
    # MCU stages 4 words via addr 9; IDE reads 4 data words back.
    # First make sure DEV=0 so this primary device responds on the bus.
    await ide_write(ctx, dut, CSN_CMD, ADDR_DEVICE, 0x00)

    payload = [0x1111, 0x2222, 0x3333, 0x4444]
    for w in payload:
        await mcu_write(ctx, dut, 9, w)

    occupancy = await mcu_read(ctx, dut, 10)
    assert occupancy == 4, f"read_fifo level should be 4 after 4 pushes, got {occupancy}"

    for expected in payload:
        got = await ide_read(ctx, dut, CSN_CMD, ADDR_DATA)
        assert got == expected, (
            f"IDE Data read should be {expected:#06x}, got {got:#06x}"
        )

    occupancy = await mcu_read(ctx, dut, 10)
    assert occupancy == 0, f"read_fifo level should be 0 after 4 pops, got {occupancy}"


async def tb_fifo_ide_to_mcu(ctx, dut):
    await release_reset(ctx, dut)
    await ide_write(ctx, dut, CSN_CMD, ADDR_DEVICE, 0x00)

    payload = [0xAAAA, 0xBBBB, 0xCCCC]
    for w in payload:
        await ide_write(ctx, dut, CSN_CMD, ADDR_DATA, w)

    occupancy = await mcu_read(ctx, dut, 11)
    assert occupancy == 3, f"write_fifo level should be 3 after 3 IDE writes, got {occupancy}"

    for expected in payload:
        got = await mcu_read(ctx, dut, 9)
        assert got == expected, (
            f"MCU read of write_fifo should be {expected:#06x}, got {got:#06x}"
        )

    occupancy = await mcu_read(ctx, dut, 11)
    assert occupancy == 0, f"write_fifo level should drain to 0, got {occupancy}"


async def tb_secondary_responds_when_dev_set(ctx, dut):
    await release_reset(ctx, dut)
    # `dut` for this case is a secondary device (primary=False).
    await ide_write(ctx, dut, CSN_CMD, ADDR_DEVICE, 0x10)  # DEV=1
    await ide_write(ctx, dut, CSN_CMD, ADDR_LBA_LOW, 0x99)
    val = await mcu_read(ctx, dut, MCU_LBA_LOW_MID)
    assert (val & 0xFF) == 0x99, (
        f"secondary should capture LBALow write while DEV=1, got {val:#06x}"
    )

    # Now flip DEV back to 0; secondary should ignore further LBA writes.
    await ide_write(ctx, dut, CSN_CMD, ADDR_DEVICE, 0x00)
    await ide_write(ctx, dut, CSN_CMD, ADDR_LBA_LOW, 0xAA)
    val = await mcu_read(ctx, dut, MCU_LBA_LOW_MID)
    assert (val & 0xFF) == 0x99, (
        f"secondary should ignore LBALow write while DEV=0, got {val:#06x}"
    )


async def tb_dma_read_basic(ctx, dut):
    """Stage one burst via MCU push, arm dma_mode, simulate the host's
    DMA-read handshake: DMARQ must rise once data is available, the
    host's DMACK+RD pulses must pop the FIFO, and DMARQ must drop
    when the FIFO empties."""
    await release_reset(ctx, dut)
    await ide_write(ctx, dut, CSN_CMD, ADDR_DEVICE, 0x00)

    # Park bus in idle with DMACKn high, RD low.
    ctx.set(dut.ide_dmackn, 1)
    ctx.set(dut.ide_rd, 0); ctx.set(dut.ide_rd_h, 0)
    ctx.set(dut.ide_wr, 0); ctx.set(dut.ide_wr_h, 0)
    await ctx.tick()

    # One full Holly burst: DMARQ is only granted at >= 16 words
    # (`rd_has_burst`), see dma_pop_survives_burst_end.
    payload = [0xCAFE, 0xBABE, 0xDEAD, 0xBEEF] + [0x1000 + i for i in range(12)]
    for w in payload:
        await mcu_write(ctx, dut, 9, w)

    # FIFO has data but dma_mode is still 0 → DMARQ must stay low.
    assert ctx.get(dut.ide_dmarq) == 0, "DMARQ should be 0 with dma_mode=0"

    # Arm DMA mode via MCU register 19.
    await mcu_write(ctx, dut, MCU_DMA_MODE, 1)
    # One settle tick for the comb path.
    await ctx.tick()
    assert ctx.get(dut.ide_dmarq) == 1, (
        "DMARQ should rise once dma_mode=1 with FIFO data and selected"
    )

    for expected in payload:
        got = await ide_dma_read(ctx, dut)
        assert got == expected, (
            f"DMA read should be {expected:#06x}, got {got:#06x}"
        )

    # Drained — DMARQ falls.
    for _ in range(4):
        await ctx.tick()
    assert ctx.get(dut.ide_dmarq) == 0, (
        "DMARQ should drop after FIFO drained"
    )

    # Disarm dma_mode.
    await mcu_write(ctx, dut, MCU_DMA_MODE, 0)


async def tb_dma_dmarq_not_gated_by_nien(ctx, dut):
    """nIEN must NOT mask DMARQ.

    ATA/ATAPI-5 5.2.8 qualifies DMARQ on device selection only; nIEN is
    scoped to INTRQ (5.2.9, 7.9). Hosts set nIEN=1 to poll status rather
    than take interrupts, so masking DMARQ with it drops the request
    mid-burst and the host's DMA stalls for good with data still queued.
    This test previously asserted the opposite and locked in that bug."""
    await release_reset(ctx, dut)
    await ide_write(ctx, dut, CSN_CMD, ADDR_DEVICE, 0x00)
    # Set nIEN=1 in DEVCTL.
    await ide_write(ctx, dut, CSN_CTRL, ADDR_DEVICE_CONTROL, 0x02)

    ctx.set(dut.ide_dmackn, 1)
    await ctx.tick()
    for i in range(16):  # a full burst, the DMARQ grant threshold
        await mcu_write(ctx, dut, 9, 0x1234 + i)
    await mcu_write(ctx, dut, MCU_DMA_MODE, 1)
    await ctx.tick()
    assert ctx.get(dut.ide_dmarq) == 1, (
        "DMARQ must stay asserted while nIEN=1 -- nIEN gates INTRQ only "
        "(ATA/ATAPI-5 5.2.8/5.2.9)"
    )



async def tb_dma_narrow_gap_still_pops(ctx, dut):
    """A DIOR- gap too short to appear in the posedge sample stream must
    still produce a second pop.

    Multiword DMA mode 2 allows tKR = 25 ns between strobes, only 1.2
    periods of the 48 MHz core clock, so the deassertion can fall entirely
    between two rising edges. Here ide_rd stays high across the boundary
    while ide_rd_h dips -- i.e. the gap existed but only the negedge sample
    caught it. Before two-phase sampling this merged into one falling edge
    and the FIFO popped once, leaving the host a word ahead of the device
    and every later word shifted."""
    await release_reset(ctx, dut)
    await ide_write(ctx, dut, CSN_CMD, ADDR_DEVICE, 0x00)
    ctx.set(dut.ide_dmackn, 1)
    ctx.set(dut.ide_rd, 0); ctx.set(dut.ide_rd_h, 0)
    ctx.set(dut.ide_wr, 0); ctx.set(dut.ide_wr_h, 0)
    await ctx.tick()

    for w in (0x1111, 0x2222):
        await mcu_write(ctx, dut, 9, w)
    await mcu_write(ctx, dut, MCU_DMA_MODE, 1)
    await ctx.tick()

    # First strobe, then a gap seen only on the negedge, then a second
    # strobe -- all without ide_rd ever reading low on a rising edge.
    ctx.set(dut.ide_dmackn, 0)
    ctx.set(dut.ide_rd, 1); ctx.set(dut.ide_rd_h, 1)
    for _ in range(3):
        await ctx.tick()
    first = ctx.get(dut.ide_data_out)

    # The narrow gap: posedge sample still 1, half-phase sample 0.
    ctx.set(dut.ide_rd_h, 0)
    await ctx.tick()
    ctx.set(dut.ide_rd_h, 1)
    for _ in range(3):
        await ctx.tick()
    second = ctx.get(dut.ide_data_out)

    ctx.set(dut.ide_rd, 0); ctx.set(dut.ide_rd_h, 0)
    ctx.set(dut.ide_dmackn, 1)
    await ctx.tick()

    assert first == 0x1111, f"first DMA word should be 0x1111, got {first:#06x}"
    assert second == 0x2222, (
        f"a gap visible only in the half-phase sample must still pop the FIFO: "
        f"expected 0x2222, got {second:#06x}"
    )
    await mcu_write(ctx, dut, MCU_DMA_MODE, 0)



async def tb_dma_pop_survives_burst_end(ctx, dut):
    """DMACK released right after the last read of a burst must not eat the pop.

    Holly moves DMA data in 32-byte bursts and drops DMACK immediately after
    the final read of each one. rd_fall reports an edge 1-1.5 cycles late, so
    gating the pop on the *present* dmack_active dropped it -- the word stayed
    in the FIFO and the next burst read it again, duplicating one word per
    burst boundary. Here DMACK is released in the same cycle the strobe falls;
    the second read must still return the second word."""
    await release_reset(ctx, dut)
    await ide_write(ctx, dut, CSN_CMD, ADDR_DEVICE, 0x00)
    ctx.set(dut.ide_dmackn, 1)
    ctx.set(dut.ide_rd, 0); ctx.set(dut.ide_rd_h, 0)
    ctx.set(dut.ide_wr, 0); ctx.set(dut.ide_wr_h, 0)
    await ctx.tick()

    for w in (0xAAAA, 0xBBBB):
        await mcu_write(ctx, dut, 9, w)
    await mcu_write(ctx, dut, MCU_DMA_MODE, 1)
    await ctx.tick()

    # Burst 1: one read, then DMACK drops immediately with the strobe.
    ctx.set(dut.ide_dmackn, 0)
    ctx.set(dut.ide_rd, 1); ctx.set(dut.ide_rd_h, 1)
    for _ in range(4):
        await ctx.tick()
    first = ctx.get(dut.ide_data_out)
    ctx.set(dut.ide_rd, 0); ctx.set(dut.ide_rd_h, 0)
    ctx.set(dut.ide_dmackn, 1)          # released before our sync sees the fall
    for _ in range(4):
        await ctx.tick()

    # Burst 2.
    ctx.set(dut.ide_dmackn, 0)
    ctx.set(dut.ide_rd, 1); ctx.set(dut.ide_rd_h, 1)
    for _ in range(4):
        await ctx.tick()
    second = ctx.get(dut.ide_data_out)
    ctx.set(dut.ide_rd, 0); ctx.set(dut.ide_rd_h, 0)
    ctx.set(dut.ide_dmackn, 1)
    await ctx.tick()

    assert first == 0xAAAA, f"first burst word should be 0xAAAA, got {first:#06x}"
    assert second == 0xBBBB, (
        f"DMACK released at the burst boundary must not suppress the pop: "
        f"expected 0xBBBB, got {second:#06x}"
    )
    await mcu_write(ctx, dut, MCU_DMA_MODE, 0)



async def tb_secondary_reports_ata_signature(ctx, dut_sec):
    """The secondary device is a hard disk, not a packet device.

    ATA/ATAPI-5 9.1: a non-packet device reports 0x01/0x00/0x00 in
    LBA_LOW/MID/HIGH with sector count 1, where the GD-ROM reports
    0x81/0x14/0xEB. The host uses this to tell the two apart on a shared
    bus, so DreamShell / KOS will only see an IDE disk if the secondary
    stops claiming to be ATAPI."""
    await release_reset(ctx, dut_sec)
    # Select device 1 so the secondary answers.
    await ide_write(ctx, dut_sec, CSN_CMD, ADDR_DEVICE, 0x10)

    lo = await ide_read(ctx, dut_sec, CSN_CMD, ADDR_LBA_LOW)
    mid = await ide_read(ctx, dut_sec, CSN_CMD, ADDR_LBA_MID)
    hi = await ide_read(ctx, dut_sec, CSN_CMD, ADDR_LBA_HIGH)
    assert (lo & 0xFF) == 0x01, f"secondary LBA_LOW should be 0x01, got {lo:#06x}"
    assert (mid & 0xFF) == 0x00, f"secondary LBA_MID should be 0x00, got {mid:#06x}"
    assert (hi & 0xFF) == 0x00, f"secondary LBA_HIGH should be 0x00, got {hi:#06x}"



async def tb_dma_write_fills_write_fifo(ctx, dut_sec):
    """DMA host->device: DMARQ must follow write-FIFO *room*, and DMACK+WR
    cycles must land words in the write FIFO.

    Reading, DMARQ means "I have data"; writing, it means "I have room", so
    the direction has to be part of dma_mode rather than a single flag. DMA
    cycles deassert CSn, so the push is gated on DMACK rather than the address
    decoder."""
    await release_reset(ctx, dut_sec)
    await ide_write(ctx, dut_sec, CSN_CMD, ADDR_DEVICE, 0x10)
    ctx.set(dut_sec.ide_dmackn, 1)
    ctx.set(dut_sec.ide_rd, 0); ctx.set(dut_sec.ide_rd_h, 0)
    ctx.set(dut_sec.ide_wr, 0); ctx.set(dut_sec.ide_wr_h, 0)
    await ctx.tick()

    # dma_mode = 2 (host -> device). The write FIFO is empty, so there is room
    # and DMARQ should rise.
    await mcu_write(ctx, dut_sec, MCU_DMA_MODE, 2)
    for _ in range(4):
        await ctx.tick()
    assert ctx.get(dut_sec.ide_dmarq) == 1, (
        "DMARQ should rise for a host->device DMA when the write FIFO has room"
    )

    # Two DMA write cycles: DMACK low, pulse WR with data on the bus.
    for word in (0x1234, 0x5678):
        ctx.set(dut_sec.ide_csn, CSN_IDLE)
        ctx.set(dut_sec.ide_dmackn, 0)
        ctx.set(dut_sec.ide_data_in, word)
        await ctx.tick()
        ctx.set(dut_sec.ide_wr, 1); ctx.set(dut_sec.ide_wr_h, 1)
        for _ in range(5):
            await ctx.tick()
        ctx.set(dut_sec.ide_wr, 0); ctx.set(dut_sec.ide_wr_h, 0)
        for _ in range(5):
            await ctx.tick()
        ctx.set(dut_sec.ide_dmackn, 1)
        await ctx.tick()

    level = await mcu_read(ctx, dut_sec, MCU_WRITE_FIFO_LEVEL)
    assert level == 2, f"write FIFO should hold 2 words, got {level}"
    a = await mcu_read(ctx, dut_sec, 9)
    b = await mcu_read(ctx, dut_sec, 9)
    assert a == 0x1234, f"first DMA-written word should be 0x1234, got {a:#06x}"
    assert b == 0x5678, f"second DMA-written word should be 0x5678, got {b:#06x}"
    await mcu_write(ctx, dut_sec, MCU_DMA_MODE, 0)



async def tb_stray_wr_does_not_replay_command(ctx, dut):
    """A write strobe with nothing decoded must not re-run the last command.

    decode_command_lat is latched while WR is high and was never cleared, so
    it stayed set after a command write. Any later write edge then re-entered
    the command path: BSY back on, command_pend back on, and fifo_clear
    wiping the write FIFO. On hardware that stalled every WRITE SECTORS --
    the MCU armed DRQ, read status back as 0x58, and moments later both it
    and the host saw 0xd8 with the write FIFO emptied."""
    await release_reset(ctx, dut)
    await ide_write(ctx, dut, CSN_CMD, ADDR_DEVICE, 0x00)
    await ide_write(ctx, dut, CSN_CMD, ADDR_COMMAND, 0x30)

    # MCU answers: DRQ up, BSY down, ack.
    await mcu_write(ctx, dut, MCU_STATUS, 0x58)
    await mcu_write(ctx, dut, 0, 0)          # ack_command
    for _ in range(4):
        await ctx.tick()
    before = await mcu_read(ctx, dut, MCU_STATUS) & 0xFF
    assert before == 0x58, f"status should be 0x58 after arming, got {before:#04x}"

    # A bare WR pulse with the bus idle -- no CSn, no address decode.
    ctx.set(dut.ide_csn, CSN_IDLE)
    await ctx.tick()
    ctx.set(dut.ide_wr, 1); ctx.set(dut.ide_wr_h, 1)
    for _ in range(5):
        await ctx.tick()
    ctx.set(dut.ide_wr, 0); ctx.set(dut.ide_wr_h, 0)
    for _ in range(8):
        await ctx.tick()

    after = await mcu_read(ctx, dut, MCU_STATUS) & 0xFF
    assert after == before, (
        f"a stray write strobe re-ran the command: status {before:#04x} -> {after:#04x}"
    )



async def tb_command_write_clears_drq(ctx, dut):
    """A new command must clear DRQ left over from the previous one.

    The command write set BSY and command_pend but left DRQ alone, so between
    the command reaching the device and the MCU setting up the new transfer,
    the host could still see DRQ from the *previous* command. A host that
    polls "BSY clear and DRQ set" right after writing the command then dumps a
    sector into a FIFO that fifo_clear immediately wipes, and the device waits
    forever for data that was already sent. Observed on hardware as
    status=0xd8 with the write FIFO empty and the MCU stuck for 10 s."""
    await release_reset(ctx, dut)
    await ide_write(ctx, dut, CSN_CMD, ADDR_DEVICE, 0x00)

    # Leave the device in a data phase: DRQ asserted by the MCU.
    await mcu_write(ctx, dut, MCU_STATUS, 0x58)
    for _ in range(4):
        await ctx.tick()
    assert (await mcu_read(ctx, dut, MCU_STATUS) & 0x08), "setup: DRQ should be set"

    # Now the host writes a new command.
    await ide_write(ctx, dut, CSN_CMD, ADDR_COMMAND, 0x30)
    for _ in range(4):
        await ctx.tick()

    st = await mcu_read(ctx, dut, MCU_STATUS) & 0xFF
    assert st & 0x80, f"command write should set BSY, got {st:#04x}"
    assert not (st & 0x08), (
        f"command write must clear stale DRQ, got {st:#04x}"
    )



async def tb_pio_write_data_reaches_fifo(ctx, dut):
    """The full PIO WRITE SECTORS sequence, as the hardware runs it.

    Command write -> MCU arms DRQ with write flow control -> host writes a
    sector into the Data register -> the words must appear in the write FIFO.
    On hardware the host saw DRQ come up (status 0x58) and sent 256 words, and
    the MCU then sat with wfifo=0 for ten seconds, so the data was going
    somewhere other than the FIFO."""
    await release_reset(ctx, dut)
    await ide_write(ctx, dut, CSN_CMD, ADDR_DEVICE, 0x00)
    await ide_write(ctx, dut, CSN_CMD, ADDR_COMMAND, 0x30)

    # MCU: arm write flow control, DRQ up, ack -- the order handle_disk_write
    # uses.
    await mcu_write(ctx, dut, MCU_DMA_MODE, 4)
    await mcu_write(ctx, dut, MCU_STATUS, 0x58)
    await mcu_write(ctx, dut, 0, 0)
    for _ in range(4):
        await ctx.tick()

    st = await mcu_read(ctx, dut, MCU_STATUS) & 0xFF
    assert st == 0x58, f"device should be armed at 0x58, got {st:#04x}"

    for i in range(8):
        await ide_write(ctx, dut, CSN_CMD, ADDR_DATA, 0x1000 + i)

    level = await mcu_read(ctx, dut, MCU_WRITE_FIFO_LEVEL)
    assert level == 8, f"write FIFO should hold 8 words, got {level}"
    first = await mcu_read(ctx, dut, 9)
    assert first == 0x1000, f"first word should be 0x1000, got {first:#06x}"


async def tb_packet_cdb_raises_bsy(ctx, dut):
    """The last CDB word of a PACKET command must raise BSY and drop DRQ.

    ATA-5 §10.8.6 gives the device 400 ns after the sixth CDB word to do
    this; firmware can't. Without it the host saw DRQ still set after the
    CDB and treated the command phase as the data phase -- fatcheck's
    REQUEST SENSE did exactly that and stalled 10 s on every run."""
    await release_reset(ctx, dut)
    await ide_write(ctx, dut, CSN_CMD, ADDR_DEVICE, 0x00)
    await ide_write(ctx, dut, CSN_CMD, ADDR_COMMAND, 0xA0)

    # The Command write itself opens the CDB phase; the MCU only acks.
    for _ in range(4):
        await ctx.tick()
    assert (await mcu_read(ctx, dut, MCU_DMA_MODE)) & 0x10, "pkt_phase should be set"
    await mcu_write(ctx, dut, 0, 0)

    for i in range(5):
        await ide_write(ctx, dut, CSN_CMD, ADDR_DATA, 0x2000 + i)
        st = await mcu_read(ctx, dut, MCU_STATUS) & 0xFF
        assert st == 0x58, f"DRQ must stay up through CDB word {i}, got {st:#04x}"

    await ide_write(ctx, dut, CSN_CMD, ADDR_DATA, 0x2005)
    for _ in range(4):
        await ctx.tick()
    st = await mcu_read(ctx, dut, MCU_STATUS) & 0xFF
    assert st & 0x80 and not (st & 0x08), (
        f"last CDB word should give BSY=1 DRQ=0, got {st:#04x}"
    )
    assert not ((await mcu_read(ctx, dut, MCU_DMA_MODE)) & 0x10), (
        "pkt_phase should self-clear"
    )
    level = await mcu_read(ctx, dut, MCU_WRITE_FIFO_LEVEL)
    assert level == 6, f"write FIFO should hold the 6 CDB words, got {level}"

    # A following data-out block must count 256 words again, not 6.
    await mcu_write(ctx, dut, MCU_DMA_MODE, 4)
    await mcu_write(ctx, dut, MCU_STATUS, 0x58)
    for _ in range(4):
        await ctx.tick()
    for i in range(6):
        await ide_write(ctx, dut, CSN_CMD, ADDR_DATA, 0x3000 + i)
    st = await mcu_read(ctx, dut, MCU_STATUS) & 0xFF
    assert st == 0x58, f"data phase must not end after 6 words, got {st:#04x}"


async def tb_hob_selects_prev(ctx, dut):
    await release_reset(ctx, dut)
    # MCU primes LBA Low and LBA Low Prev with distinct values.
    await mcu_write(ctx, dut, MCU_LBA_LOW_MID, 0x0011)  # LBALow=0x11
    await mcu_write(ctx, dut, MCU_LBA_HIGH_LOWP, 0xAA00)  # LBALowPrev=0xAA, LBAHigh=0

    # HOB=0 → IDE reads LBA Low = 0x11.
    await ide_write(ctx, dut, CSN_CTRL, ADDR_DEVICE_CONTROL, 0x00)
    val = await ide_read(ctx, dut, CSN_CMD, ADDR_LBA_LOW)
    assert (val & 0xFF) == 0x11, f"HOB=0 should read LBALow 0x11, got {val:#06x}"

    # HOB=1 → IDE reads LBA Low Prev = 0xAA.
    await ide_write(ctx, dut, CSN_CTRL, ADDR_DEVICE_CONTROL, 0x80)
    val = await ide_read(ctx, dut, CSN_CMD, ADDR_LBA_LOW)
    assert (val & 0xFF) == 0xAA, f"HOB=1 should read LBALowPrev 0xAA, got {val:#06x}"


# -- Runner -----------------------------------------------------------------

async def ide_read_short_hold(ctx, dut, csn, addr):
    """A host read that releases CSn/address in the same cycle it releases
    DIOR-: ATA t9 allows 10 ns of hold, less than one core cycle, and the
    synchronized falling edge arrives 1-1.5 cycles later than that."""
    ctx.set(dut.ide_csn, csn)
    ctx.set(dut.ide_addr, addr)
    await ctx.tick()
    ctx.set(dut.ide_rd, 1); ctx.set(dut.ide_rd_h, 1)
    for _ in range(4):
        await ctx.tick()
    value = ctx.get(dut.ide_data_out)
    ctx.set(dut.ide_rd, 0); ctx.set(dut.ide_rd_h, 0)
    ctx.set(dut.ide_csn, CSN_IDLE)
    for _ in range(4):
        await ctx.tick()
    await ide_idle(ctx, dut)
    return value


async def tb_pio_read_pops_with_short_hold(ctx, dut):
    """Stock-BIOS GET_TOC: 204 words staged, the host read all of them, but
    only 200 pops happened -- the pop was judged on the live address decode
    at rd_fall, by which time Holly had already released CS. The host then
    saw duplicated words and a DRQ that never dropped."""
    await release_reset(ctx, dut)
    await ide_write(ctx, dut, CSN_CMD, ADDR_DEVICE, 0x00)
    payload = [0x1111, 0x2222, 0x3333, 0x4444]
    for w in payload:
        await mcu_write(ctx, dut, 9, w)
    await mcu_write(ctx, dut, 13, 1)
    for expected in payload:
        got = await ide_read_short_hold(ctx, dut, CSN_CMD, ADDR_DATA)
        assert got == expected, f"expected {expected:#06x}, got {got:#06x}"
    assert await mcu_read(ctx, dut, 10) == 0, "read_fifo not drained"
    assert await mcu_read(ctx, dut, 25) == 4, "data-read counter missed a read"
    # The Status read's INTRQ clear must survive the same short hold.
    assert ctx.get(dut.ide_intrq) == 1
    await ide_read_short_hold(ctx, dut, CSN_CMD, ADDR_STATUS)
    assert ctx.get(dut.ide_intrq) == 0, "Status read with short hold did not clear INTRQ"
    assert (await mcu_read(ctx, dut, 24) & 0xFF) == 1


async def tb_pio_read_block_end_shows_bsy(ctx, dut):
    """ATAPI PIO data-in in DRQ blocks: once the host pops the last word of
    a block the device must show BSY (not 0x50) until the MCU has the next
    block staged. With read flow control armed the FPGA does it on the pop
    itself; the MCU is far too slow to beat the host's next poll."""
    await release_reset(ctx, dut)
    await ide_write(ctx, dut, CSN_CMD, ADDR_DEVICE, 0x00)
    await mcu_write(ctx, dut, MCU_DMA_MODE, 0x08)          # pio_rd_fc
    for w in (0xAAAA, 0x5555):
        await mcu_write(ctx, dut, 9, w)
    await mcu_write(ctx, dut, MCU_STATUS, 0x58)
    assert (await ide_read(ctx, dut, CSN_CTRL, ADDR_ALT_STATUS) & 0xFF) == 0x58
    assert await ide_read(ctx, dut, CSN_CMD, ADDR_DATA) == 0xAAAA
    assert (await ide_read(ctx, dut, CSN_CTRL, ADDR_ALT_STATUS) & 0xFF) == 0x58
    assert await ide_read(ctx, dut, CSN_CMD, ADDR_DATA) == 0x5555
    # Block drained: BSY=1, DRQ=0 with no MCU involvement.
    assert (await ide_read(ctx, dut, CSN_CTRL, ADDR_ALT_STATUS) & 0xFF) == 0xD0
    # Next block staged -> host sees the MCU's DRQ status again.
    await mcu_write(ctx, dut, 9, 0x1234)
    assert (await ide_read(ctx, dut, CSN_CTRL, ADDR_ALT_STATUS) & 0xFF) == 0x58
    assert await ide_read(ctx, dut, CSN_CMD, ADDR_DATA) == 0x1234
    assert (await ide_read(ctx, dut, CSN_CTRL, ADDR_ALT_STATUS) & 0xFF) == 0xD0
    # Completion status written by the MCU is not masked by the empty FIFO.
    await mcu_write(ctx, dut, MCU_STATUS, 0x50)
    assert (await ide_read(ctx, dut, CSN_CTRL, ADDR_ALT_STATUS) & 0xFF) == 0x50
    # Flow control off: an empty FIFO no longer implies BSY.
    await mcu_write(ctx, dut, MCU_STATUS, 0x58)
    await mcu_write(ctx, dut, MCU_DMA_MODE, 0x00)
    assert (await ide_read(ctx, dut, CSN_CTRL, ADDR_ALT_STATUS) & 0xFF) == 0x58


TESTBENCHES = [
    # (name, testbench, primary?)
    ("default_state", tb_default_state, True),
    ("post_reset_idle", tb_post_reset_idle, True),
    ("ide_write_lba", tb_ide_write_lba, True),
    ("ide_writes_command_sets_pend", tb_ide_writes_command_sets_pend, True),
    ("ide_reads_status", tb_ide_reads_status, True),
    ("nien_masks_intrq", tb_nien_masks_intrq, True),
    ("command_write_does_not_intrq", tb_command_write_does_not_intrq, True),
    ("command_write_clears_intrq", tb_command_write_clears_intrq, True),
    ("status_write_asserts_intrq", tb_status_write_asserts_intrq, True),
    ("packet_write_opens_cdb_phase", tb_packet_write_opens_cdb_phase, True),
    ("srst_resets_device", tb_srst_resets_device, True),
    ("sector_count_sliding", tb_sector_count_sliding, True),
    ("hob_selects_prev", tb_hob_selects_prev, True),
    ("primary_ignored_when_dev_set", tb_primary_ignored_when_dev_set, True),
    ("data_out_en_gated_by_dev", tb_data_out_en_gated_by_dev, True),
    ("fifo_mcu_to_ide", tb_fifo_mcu_to_ide, True),
    ("fifo_ide_to_mcu", tb_fifo_ide_to_mcu, True),
    ("pio_read_pops_with_short_hold", tb_pio_read_pops_with_short_hold, True),
    ("pio_read_block_end_shows_bsy", tb_pio_read_block_end_shows_bsy, True),
    ("secondary_responds_when_dev_set", tb_secondary_responds_when_dev_set, False),
    ("dma_read_basic", tb_dma_read_basic, True),
    ("dma_dmarq_not_gated_by_nien", tb_dma_dmarq_not_gated_by_nien, True),
    ("dma_narrow_gap_still_pops", tb_dma_narrow_gap_still_pops, True),
    ("stray_wr_does_not_replay_command", tb_stray_wr_does_not_replay_command, True),
    ("command_write_clears_drq", tb_command_write_clears_drq, True),
    ("pio_write_data_reaches_fifo", tb_pio_write_data_reaches_fifo, True),
    ("packet_cdb_raises_bsy", tb_packet_cdb_raises_bsy, True),
    ("dma_pop_survives_burst_end", tb_dma_pop_survives_burst_end, True),
    ("secondary_reports_ata_signature", tb_secondary_reports_ata_signature, False),
    ("dma_write_fills_write_fifo", tb_dma_write_fills_write_fifo, False),
]


def run_one(name, tb, primary):
    # post_reset_count_bits=4 → the reset BSY counter expires in 16 cycles,
    # so `release_reset()` finishes before the first real assertion in each
    # test. Hardware uses the default 24-bit counter via Quartus build.
    dut = IDEDevice(primary=primary, post_reset_count_bits=4)
    sim = Simulator(dut)
    sim.add_clock(1e-7)  # 10 MHz — fine for combinational/sync verification.

    async def wrapper(ctx):
        await tb(ctx, dut)

    sim.add_testbench(wrapper)
    os.makedirs("vcd", exist_ok=True)
    with sim.write_vcd(f"vcd/{name}.vcd"):
        sim.run()


def main() -> None:
    failures = 0
    for name, tb, primary in TESTBENCHES:
        try:
            run_one(name, tb, primary)
            print(f"  ok    {name}")
        except AssertionError as e:
            print(f"  FAIL  {name}: {e}")
            failures += 1
    if failures:
        print(f"\n{failures} test(s) failed")
        sys.exit(1)
    print(f"\nall {len(TESTBENCHES)} test(s) passed")


if __name__ == "__main__":
    main()
