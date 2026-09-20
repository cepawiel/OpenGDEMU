from amaranth import *
from amaranth.hdl.xfrm import ResetInserter
from amaranth.lib.fifo import SyncFIFO
from amaranth.lib.memory import Memory

from .ide_decode import IDEAddressDecode


# Default read FIFO depth (MCU→IDE) in 16-bit words. The primary device
# (GD-ROM) needs a much deeper FIFO than the secondary so the host's
# DMA burst can be absorbed before the MCU push catches up — IP.BIN
# CD_READ is 14336 bytes = 7168 words, and underrunning anywhere in
# that window makes the host's DMA channel terminate early. A 4096-word
# (8 KB) FIFO covers ~57% of that transfer with pre-prime.
READ_FIFO_DEPTH = 4096
# Four sectors, not one. At one sector exactly the FIFO was empty at every
# sector boundary of a multi-sector read, with the host still pulling, so the
# refill handshake had to be right 64 times per 32 KB command -- and about
# 0.7% of the time it was not, the host read a word the FIFO never serviced
# and everything after it in the transfer shifted by one. It also left no room
# for the push path's 4-word headroom, so a whole 256-word sector could not be
# staged in one go.
SECONDARY_READ_FIFO_DEPTH = 1024
# Write FIFO depth (IDE→MCU) for the packet device: only the 12-byte ATAPI
# CDB plus small SET_MODE payloads, so 32 words is plenty.
WRITE_FIFO_DEPTH = 32
# The disk takes whole 512-byte sectors from the host, which is 256 words --
# a 32-word FIFO cannot hold one at all, so WRITE SECTORS was impossible
# before this. 512 gives a sector of headroom so the MCU can be draining one
# while the host writes the next.
SECONDARY_WRITE_FIFO_DEPTH = 512
# Compatibility alias for places that still reference the old name.
FIFO_DEPTH = READ_FIFO_DEPTH


class IDEDevice(Elaboratable):
    """ATA register file for one emulated device.

    Owns the per-device ATA state: command/status/error/device/LBA/sector
    count plus the LBA48 "previous" shadow registers and the device-control
    bits (HOB/SRST/nIEN). Exposes two bus interfaces sharing the same
    storage:

    - **IDE bus** (driven by the Dreamcast G1 connector): address-decoded
      via `IDEAddressDecode`. Reads drive `ide_data_out` combinationally;
      writes are level-triggered against `ide_wr` on the FPGA core clock.
    - **MCU bus** (driven by the SAM3U SMC): a flat window of 16-bit words.
      Layout matches the prior design's `docs/AddressMapMCU.md` so the eventual
      Rust driver doesn't have to relearn it. FIFO entries (addr 9..11) are
      stubbed for now; phase 3 will add the data FIFOs.

    Phase 2 scope: two devices share the IDE bus, master/slave selection
    via the DEV bit (bit 4 of the Device register). The Device register
    *write* path is unconditional — both devices on the bus update so they
    agree on who is currently selected. Every other IDE write and every
    IDE read is gated on `device_selected`. The instance only drives the
    IDE bus while it is selected; `ide_data_out_en` lets the top-level
    pick the right output. Data register reads return 0xFFFF (FIFOs are
    phase 3).
    """

    def __init__(self, *, primary: bool = True, post_reset_count_bits: int = 24,
                 read_fifo_depth: int | None = None,
                 write_fifo_depth: int | None = None,
                 capture: bool = False):
        self.primary = primary
        # In-fabric signal capture on MCU slots 20/21. Off by default: it is a
        # debug instrument, and adding it moved the fitter enough to break
        # WRITE DMA on hardware (12/12 -> 11/12) even though timing still
        # closed comfortably. This design has 56 unconstrained input ports, so
        # placement changes can alter real bus behaviour without static timing
        # noticing. Turn it on deliberately, for one investigation at a time.
        self.capture = capture
        # Primary device gets the full 4 KB depth by default; secondary
        # uses a small FIFO since HDD emulation rarely benefits from
        # deep queueing and we need the block RAM elsewhere.
        if read_fifo_depth is None:
            read_fifo_depth = READ_FIFO_DEPTH if primary else SECONDARY_READ_FIFO_DEPTH
        self.read_fifo_depth = read_fifo_depth
        if write_fifo_depth is None:
            write_fifo_depth = WRITE_FIFO_DEPTH if primary else SECONDARY_WRITE_FIFO_DEPTH
        self.write_fifo_depth = write_fifo_depth
        # Width of the post-reset BSY counter. Default 24 bits at the FPGA
        # core clock gives ~0.4 s @ 48 MHz / ~1.5 s @ 11 MHz of busy time
        # after reset deasserts — enough for the DC's GD-ROM driver to see
        # a clean idle transition. Sim tests override this to ~4 bits so
        # the counter expires in a handful of cycles.
        self.post_reset_count_bits = post_reset_count_bits

        # IDE bus (active-high inside the core; the Quartus top inverts the
        # active-low DC strobes before feeding them in here).
        self.ide_rstn = Signal()
        self.ide_csn = Signal(2)
        self.ide_rd = Signal()
        self.ide_wr = Signal()
        self.ide_addr = Signal(3)
        self.ide_data_in = Signal(16)
        self.ide_data_out = Signal(16)
        self.ide_data_out_en = Signal()
        self.ide_intrq = Signal()
        # DMA handshake. ide_dmackn is the active-low DMA acknowledge from
        # the host (passed straight from the connector); ide_dmarq is the
        # device's DMA-request output, driven high when dma_mode is armed
        # by the MCU and the read FIFO has data ready.
        self.ide_dmackn = Signal()
        # Second sample of the read strobe, captured by the wrapper on the
        # *negative* clock edge. Multiword DMA mode 2 (ATA/ATAPI-5 table 50)
        # allows only tKR = 25 ns between consecutive DIOR- pulses, which is
        # 1.2 periods of the 48 MHz core clock -- too little margin to see the
        # gap reliably. Missing it merges two strobes into one falling edge and
        # the read FIFO pops once instead of twice, so the host's DMA counter
        # runs ahead of what we actually handed it and every later word is
        # shifted. Sampling on both clock edges halves the effective period to
        # 10.4 ns, giving >=2 samples inside a 25 ns gap.
        self.ide_rd_h = Signal()
        # ...and the same for the write strobe. A PIO data-out phase is a
        # burst of back-to-back 16-bit writes, and sampling them on one clock
        # edge has the same margin problem as the read side: miss the gap
        # between two strobes and they merge into one, so words never reach
        # the write FIFO. Observed as the host sending a full sector while the
        # MCU saw wfifo=0.
        self.ide_wr_h = Signal()
        self.ide_dmarq = Signal()
        # IORDY is the device's "I'm ready, you can complete this cycle"
        # output. Real ATA holds IORDY low to extend a host transfer
        # cycle until the device has data ready. We use it to pace DMA
        # bursts when the FIFO underruns: low while the FIFO is empty
        # in DMA mode, high otherwise. Defaults to 1 so PIO + idle bus
        # are unaffected.
        self.ide_iordy = Signal(reset=1)

        # Slow heartbeat derived from CLK_11_2896_MHz. The wrapper feeds
        # in a divided-down toggle of the audio xtal so the core can count
        # edges in its own domain and expose "is the audio clock alive"
        # to the MCU at diag slot 18. Synchronized internally.
        self.audio_tick = Signal()

        # MCU bus. The address is the per-device 7-bit window — top-level
        # uses MCU_ADDR[7] to pick which device the access lands on.
        self.mcu_cs = Signal()
        self.mcu_rd = Signal()
        self.mcu_wr = Signal()
        self.mcu_bs = Signal(2)
        self.mcu_addr = Signal(7)
        self.mcu_data_in = Signal(16)
        self.mcu_data_out = Signal(16)
        self.mcu_irq = Signal()

        # CDDA pass-through (primary window only). Slots 29..31 of the MCU
        # window belong to the audio block at the top level; this device
        # just decodes them. `aux_ctrl_rd`/`aux_level` are what the MCU
        # reads back from slots 29/30, `aux_ctrl_we` is high while the MCU
        # writes slot 29, `aux_push` pulses once per write to slot 31.
        self.aux_ctrl_rd = Signal(16)
        self.aux_level = Signal(16)
        self.aux_ctrl_we = Signal()
        self.aux_push = Signal()
        self.aux_wdata = Signal(16)

        # Simulation visibility only: read FIFO occupancy as a port, so a
        # testbench can log it against the host's strobes. Unused in synthesis.
        self.dbg_rfifo_level = Signal(range(self.read_fifo_depth + 1))

    def elaborate(self, platform):
        m = Module()

        # ----- ATA register state ------------------------------------------
        device = Signal(8)
        features = Signal(8)
        # Sega GD-ROM signature loaded at FPGA power-on (reset values) and
        # also reasserted by the in-reset state machine below whenever the
        # DC pulses ide_rstn. The DC's BIOS reads LBA_LOW/MID/HIGH to
        # identify the device class without sending a command.
        #
        #   LBA_LOW (= Sega SECNR): high nibble = disk type
        #     0x00 CDDA, 0x10 CD-ROM, 0x20 CD-ROM XA, 0x80 GD-ROM.
        #   LBA_LOW low nibble = drive status: 0x2 paused, 0x3 playing,
        #     0x6 no-disc. We start mounted+paused → 0x82.
        #   LBA_MID / LBA_HIGH = 0x14 / 0xEB: standard ATAPI signature
        #     bytes; some BIOS code paths still cross-check these.
        sector_count = Signal(16, reset=0x0001)
        # LBA_LOW (Sega SECNR): high nibble = disk type (0x80 GD-ROM),
        # low nibble = drive status (0x01 ready, 0x02 paused, 0x03
        # playing, 0x06 no-disc). Set to 0x81 (ready) to match what
        # both lxdream-nitro and a stock GDEMU advertise at reset —
        # 0x82 (paused) made the BIOS think the drive needed waking up.
        #
        # The secondary device is a plain ATA disk, not a packet device, and
        # must report the non-packet signature instead: ATA/ATAPI-5 9.1 gives
        # 0x01/0x00/0x00 with sector count 1. That is how the host tells a
        # hard disk from the GD-ROM sharing the same bus.
        if self.primary:
            sig_low, sig_mid, sig_high = 0x81, 0x14, 0xEB
        else:
            sig_low, sig_mid, sig_high = 0x01, 0x00, 0x00
        self.sig_low, self.sig_mid, self.sig_high = sig_low, sig_mid, sig_high

        lba_low = Signal(8, reset=sig_low)
        lba_mid = Signal(8, reset=sig_mid)
        lba_high = Signal(8, reset=sig_high)
        lba_low_prev = Signal(8)
        lba_mid_prev = Signal(8)
        lba_high_prev = Signal(8)
        command = Signal(8)
        # Status bits.
        bsy = Signal(reset=1)
        drdy = Signal()
        dsc = Signal()
        df = Signal()
        drq = Signal()
        err = Signal()
        # Error register bits. ATAPI ERROR layout: bit 2 = ABRT, bits 4..7
        # = sense key. The bytes between (bits 0,1,3) are NM/EOM/MCR which
        # we don't currently model (always read 0).
        abrt = Signal()
        sense_key = Signal(4)
        # Device Control bits.
        hob = Signal()
        srst = Signal()
        n_ien = Signal()
        # Set when the IDE side writes the Command register; cleared by MCU.
        command_pend = Signal()
        # A host-visible reset is either the ATA hard reset line or the SRST
        # bit of Device Control (ATA/ATAPI-5 9.2): both put the device in
        # the post-reset state, abort whatever it was doing and hold BSY
        # until it is ready. `reset_seen` is the sticky MCU-side record.
        in_reset = ~self.ide_rstn | srst
        reset_seen = Signal()

        # ----- Data FIFOs --------------------------------------------------
        # `read_fifo`: MCU pushes (addr 9 write), IDE pops on Data reads.
        # `write_fifo`: IDE pushes on Data writes, MCU pops (addr 9 read).
        # Both wrapped in a ResetInserter so a 1-cycle pulse on
        # `fifo_clear` empties them — used on every IDE command write so
        # leftover bytes from a previous command can't bleed into the
        # next one.
        fifo_clear = Signal()
        m.submodules.read_fifo = read_fifo = ResetInserter(fifo_clear)(
            SyncFIFO(width=16, depth=self.read_fifo_depth)
        )
        m.submodules.write_fifo = write_fifo = ResetInserter(fifo_clear)(
            SyncFIFO(width=16, depth=self.write_fifo_depth)
        )
        # Defaults each cycle — overridden below when an actual push/pop
        # event fires.
        m.d.comb += [
            read_fifo.w_en.eq(0),
            read_fifo.r_en.eq(0),
            write_fifo.w_en.eq(0),
            write_fifo.r_en.eq(0),
            self.dbg_rfifo_level.eq(read_fifo.level),
        ]

        # ----- IDE bus address decode --------------------------------------
        m.submodules.decode = decode = IDEAddressDecode()
        m.d.comb += [
            decode.csn.eq(self.ide_csn),
            decode.addr.eq(self.ide_addr),
        ]

        # `device_selected` reflects the DEV bit relative to this instance's
        # role: primary responds when DEV=0, secondary when DEV=1.
        device_selected = Signal()
        if self.primary:
            m.d.comb += device_selected.eq(~device[4])
        else:
            m.d.comb += device_selected.eq(device[4])

        # ----- IDE bus edge detection --------------------------------------
        # `ide_wr` / `ide_rd` are asynchronous to the FPGA clock (the DC
        # drives them, we sample on our 48 MHz core clock). Run them
        # through 2-FF synchronizers before edge-detecting so a DC bus
        # cycle that's only a few FPGA cycles long is reliably caught
        # without metastability.
        wr_sync = Signal(3, reset=0b111)  # idle = de-asserted (active high inside core)
        rd_sync = Signal(3, reset=0b000)
        m.d.sync += [
            wr_sync.eq(Cat(self.ide_wr, wr_sync[:-1])),
            rd_sync.eq(Cat(self.ide_rd, rd_sync[:-1])),
        ]
        # `wr_clean` is the synchronized (metastability-resolved) version
        # of ide_wr; `wr_clean_prev` is its prior cycle. Falling-edge =
        # the host has just released the WR strobe, data still valid.
        wr_clean_prev = wr_sync[2]
        wr_clean = wr_sync[1]
        rd_clean_prev = rd_sync[2]
        rd_clean = rd_sync[1]
        wr_h_sync = Signal(2, reset=0b11)
        m.d.sync += wr_h_sync.eq(Cat(self.ide_wr_h, wr_h_sync[:-1]))
        wr_half = wr_h_sync[1]
        # Chronological order across a cycle boundary, as for the read strobe:
        # wr_sync[2] -> wr_h_sync[1] -> wr_sync[1].
        wr = (wr_clean_prev & ~wr_half) | (wr_half & ~wr_clean)

        # Half-phase sample stream for the read strobe, synchronized with the
        # same latency so the two interleave cleanly. At any core edge n:
        #   rd_sync[2]   is the input at t = n-2
        #   rd_h_sync[1] is the input at t = n-1.5   (captured on the negedge)
        #   rd_sync[1]   is the input at t = n-1
        # so the chronological order is rd_sync[2] -> rd_h_sync[1] ->
        # rd_sync[1], and a falling edge in either half counts. Consecutive
        # strobes are >=120 ns apart (t0), i.e. ~6 core cycles, so at most one
        # edge lands in a single cycle and one pop per cycle stays correct.
        rd_h_sync = Signal(2, reset=0b00)
        m.d.sync += rd_h_sync.eq(Cat(self.ide_rd_h, rd_h_sync[:-1]))
        rd_half = rd_h_sync[1]

        rd_fall = (rd_clean_prev & ~rd_half) | (rd_half & ~rd_clean)

        # Latch `ide_data_in` continuously while WR is asserted so we
        # don't depend on it being valid on the precise FPGA cycle the
        # synchronized falling edge fires (host's data hold time may be
        # shorter than our sync latency).
        ide_data_latch = Signal(16)
        # ...and a one-cycle-older copy, which is what the write path actually
        # consumes. The latch tracks the *raw* strobe while `wr` is a
        # synchronized falling edge reported ~2 cycles later, so with only
        # tKW = 25 ns between consecutive DMA write cycles (1.2 core cycles at
        # 48 MHz) the next cycle can re-latch before we have consumed the
        # previous word -- pushing word n+1 in place of word n. Observed as a
        # single wrong word per DMA write, at a different offset each run,
        # always holding its successor's value. PIO writes are unaffected
        # either way; their data is stable for many cycles.
        ide_data_latch_d = Signal(16)
        m.d.sync += ide_data_latch_d.eq(ide_data_latch)
        with m.If(self.ide_wr):
            m.d.sync += ide_data_latch.eq(self.ide_data_in)

        # Capture the address decode at WR-rising too — by the time
        # `wr` (falling edge) fires the host's address has potentially
        # already moved on.
        decode_data_lat = Signal()
        decode_device_lat = Signal()
        decode_command_lat = Signal()
        decode_features_lat = Signal()
        decode_sector_count_lat = Signal()
        decode_lba_low_lat = Signal()
        decode_lba_mid_lat = Signal()
        decode_lba_high_lat = Signal()
        decode_device_control_lat = Signal()
        with m.If(self.ide_wr):
            m.d.sync += [
                decode_data_lat.eq(decode.data),
                decode_device_lat.eq(decode.device),
                decode_command_lat.eq(decode.command),
                decode_features_lat.eq(decode.features),
                decode_sector_count_lat.eq(decode.sector_count),
                decode_lba_low_lat.eq(decode.lba_low),
                decode_lba_mid_lat.eq(decode.lba_mid),
                decode_lba_high_lat.eq(decode.lba_high),
                decode_device_control_lat.eq(decode.device_control),
            ]
        # Same for reads: `rd_fall` fires 1-1.5 core cycles after the host
        # released DIOR-, and ATA only guarantees 10 ns of address/CS hold
        # after that (t9). Judging the read by the *live* decode at rd_fall
        # silently dropped the FIFO pop (and the INTRQ clear) whenever Holly
        # let go of CS quickly -- the host then read the same word twice, the
        # FIFO kept a few leftovers, DRQ never dropped and the BIOS waited
        # forever for a PIO data-in it had actually finished (GET_TOC: 204
        # words asked, 200 pops seen, 4 left). Latch the decode while the
        # strobe is active, exactly like dmack_held above for DMA.
        decode_rd_data_lat = Signal()
        decode_rd_status_lat = Signal()
        decode_rd_alt_status_lat = Signal()
        with m.If(self.ide_rd):
            m.d.sync += [
                decode_rd_data_lat.eq(decode.data),
                decode_rd_status_lat.eq(decode.status),
                decode_rd_alt_status_lat.eq(decode.alt_status),
            ]

        # ----- DMA handshake -----------------------------------------------
        # `dma_mode` is armed by the MCU at slot 19 right before staging the
        # data-in payload of an ATAPI DMA-mode PACKET. While armed and the
        # device is selected with interrupts enabled (~n_ien), DMARQ goes
        # high whenever the read FIFO has at least one word — that tells
        # the host's DMA controller "data is ready to grab". On a DMA RD
        # cycle the host pulls DMACKn low and toggles RDn; we pop the FIFO
        # on the synchronized falling edge of RD just as for PIO data
        # reads, but address-decoded via DMACK rather than CSn/ADDR.
        # 0 = off, 1 = device->host (read), 2 = host->device (write). The
        # direction matters because DMARQ means opposite things each way:
        # reading, it says "I have data"; writing, "I have room".
        dma_mode = Signal(2)
        dma_rd_mode = Signal()
        dma_wr_mode = Signal()
        # Slot 19 bit 2: gate the host-visible DRQ on write-FIFO room during a
        # PIO data-out phase. The MCU cannot do this itself -- it polls at
        # 100 us while the host fills the 512-word FIFO in ~60 us, so a
        # multi-sector WRITE SECTORS overran it and lost everything past the
        # second sector while DRQ was still asserted. Backpressure has to be
        # in hardware; this is the standard mechanism for it.
        pio_wr_fc = Signal()
        # Words the host has pushed into the current PIO data-out block.
        BLOCK_WORDS = 256
        pio_wr_words = Signal(range(BLOCK_WORDS))
        # Bit 3 is the read-direction dual: gate DRQ on the read FIFO actually
        # having data. Same reason -- lowering DRQ from firmware between
        # sectors is far too slow, and the host reads the empty FIFO before we
        # notice, getting the previous sector again.
        pio_rd_fc = Signal()
        # Bit 4: ATAPI packet phase. The MCU sets it before raising DRQ for a
        # PACKET command's CDB; the data-out counter then ends the "block"
        # after PACKET_WORDS instead of BLOCK_WORDS and raises BSY / drops
        # DRQ on the last CDB word (ATA-5 §10.8.6: within 400 ns). Without
        # it the host saw DRQ still set after the CDB, treated it as the
        # data phase and pushed/pulled words that belonged to nothing;
        # fatcheck's REQUEST SENSE hit that and stalled 10 s. Self-clears
        # on the last CDB word and on any new command.
        PACKET_WORDS = 6
        pkt_phase = Signal()
        m.d.comb += [
            dma_rd_mode.eq(dma_mode == 1),
            dma_wr_mode.eq(dma_mode == 2),
        ]
        dma_any = dma_rd_mode | dma_wr_mode

        dmackn_sync = Signal(2, reset=0b11)
        m.d.sync += dmackn_sync.eq(Cat(self.ide_dmackn, dmackn_sync[:-1]))
        dmackn_clean = dmackn_sync[1]
        dmack_active = ~dmackn_clean

        # DMACK as it was *during* the cycle, not as it is now. rd_fall / wr
        # report an edge 1-1.5 core cycles late, and Holly releases DMACK
        # immediately after the last transfer of each 32-byte burst, so gating
        # on the present value silently drops the FIFO operation at every
        # burst boundary.
        dmack_held = Signal()
        with m.If(rd_clean | wr_clean):
            m.d.sync += dmack_held.eq(dmack_active)

        # NB: nIEN must NOT gate DMARQ. ATA/ATAPI-5 5.2.8 qualifies DMARQ on
        # device selection only; nIEN is scoped to INTRQ alone (5.2.9, 7.9:
        # "nIEN is the enable bit for the device interrupt to the host ...
        # the INTRQ signal shall be in a high impedance state"). Hosts
        # routinely set nIEN=1 to poll status instead of taking interrupts,
        # and gating DMARQ on it dropped the request mid-burst: the host's
        # DMA then stalled permanently with data still queued in the FIFO,
        # delivering a variable 9-14 KB of a 14 KB CD_READ depending on when
        # the BIOS happened to write Device Control.
        # DMARQ is registered and only allowed to change while no read strobe
        # is in flight. ATA/ATAPI-5 figure 47 bounds when the device may
        # negate DMARQ relative to the current DIOR- assertion (tL); driving
        # it combinationally from r_rdy dropped it the instant the FIFO went
        # empty, which can land in the middle of a host read and corrupt the
        # word in flight. Holding it until the strobe is idle lets the current
        # word complete, which is exactly what the figure requires.
        #
        # Observed before this: exactly one scrambled word about a dozen reads
        # after each MCU chunk refill, where the FIFO runs dry and refills
        # under a host that is already pulling.
        #
        # Holly moves 32 bytes -- 16 words -- per grant and does not stop for
        # DMARQ negating mid-burst (on the write side a grant with three words
        # free still delivered sixteen; the other thirteen were lost). So a
        # grant is a promise of a whole burst, in both directions.
        #
        # Reading, that promise was broken every time the FIFO ran dry and
        # refilled under a waiting host: DMARQ went up on the first word of the
        # refill, and the host then pulled 16 words at 120 ns each while the
        # MCU's DMAC was supplying them at one per 125 ns SMC cycle. Which word
        # the host overtook the refill on depended on a few nanoseconds of
        # phase, so the failure moved with FPGA placement -- one word per few
        # megabytes on one build, word 12 of every sector on another -- and
        # the value read was whatever that FIFO slot held a full wrap earlier.
        # test_timing.py reproduces it at t0 = 110 ns. Asking for a whole burst
        # before granting one removes the race entirely.
        #
        # This cannot deadlock a transfer on the Dreamcast: Holly's DMA length
        # is in 32-byte units, so the host only ever asks for whole bursts, and
        # the MCU stages whole sectors (512 or 2048 bytes). Whatever is left
        # in the FIFO with nothing more coming is therefore a multiple of 16
        # words -- and if that ever stops being true, the fix is to pad on the
        # MCU side, not to lower this.
        #
        # But the threshold is only the *grant* condition. Holly, reading,
        # does react to DMARQ negating mid-burst: it ends the burst, keeps only
        # the whole 32-byte chunks and re-requests the rest. Negating at
        # level < 16 therefore cut the final chunk of every READ DMA short at
        # its second word -- the words were already popped, so the re-request
        # never got a grant and the host sat at "done=480 of 512" forever.
        # Hence `rd_burst_left`: a grant hands out 16 words of credit and
        # DMARQ stays up until they are consumed (data is guaranteed to be
        # there, the FIFO only gets fuller), and only then re-asks for a
        # full burst. `rd_has_burst` is OR'd in so a back-to-back grant does
        # not glitch DMARQ low for the cycle the counter takes to reload.
        #
        # Writing, the mirror image: `w_rdy` only drops when the FIFO is
        # completely full, so demand a burst of headroom before granting. On a
        # 64-sector WRITE DMA the lack of this surfaced as the MCU receiving 41
        # of 64 sectors while Holly reported all 32 KB sent.
        DMA_BURST_WORDS = 16
        _depth = self.write_fifo_depth
        _room = (_depth - 2 * DMA_BURST_WORDS) if _depth > 4 * DMA_BURST_WORDS \
                else _depth // 2
        wr_has_room = Signal()
        m.d.comb += wr_has_room.eq(write_fifo.level <= _room)
        rd_has_burst = Signal()
        m.d.comb += rd_has_burst.eq(read_fifo.level >= DMA_BURST_WORDS)

        rd_burst_left = Signal(range(DMA_BURST_WORDS + 1))
        rd_dma_pop = Signal()
        with m.If(rd_dma_pop):
            # A pop with no credit means the grant came from `rd_has_burst`
            # the cycle before the counter reloaded: count it as word 1.
            m.d.sync += rd_burst_left.eq(
                Mux(rd_burst_left == 0, DMA_BURST_WORDS - 1, rd_burst_left - 1)
            )
        with m.Elif((rd_burst_left == 0) & rd_has_burst & dma_rd_mode):
            m.d.sync += rd_burst_left.eq(DMA_BURST_WORDS)
        with m.If(~dma_rd_mode):
            m.d.sync += rd_burst_left.eq(0)
        rd_dmarq = rd_has_burst | ((rd_burst_left != 0) & read_fifo.r_rdy)

        dmarq_r = Signal()
        strobe_idle = ~rd_clean & ~rd_half & ~wr_clean
        dmarq_want = device_selected & (
            (dma_rd_mode & rd_dmarq) | (dma_wr_mode & wr_has_room)
        )
        with m.If(strobe_idle):
            m.d.sync += dmarq_r.eq(dmarq_want)
        m.d.comb += self.ide_dmarq.eq(dmarq_r)
        # IORDY pacing: while DMA is armed on this (selected) device,
        # follow the read FIFO's r_rdy signal — IORDY high = data is
        # available, IORDY low = pause the host's DMA cycle while the
        # MCU catches up refilling. Outside of DMA-active contexts
        # leave IORDY high so PIO register reads complete promptly.
        m.d.comb += self.ide_iordy.eq(
            Mux(dma_rd_mode & device_selected, read_fifo.r_rdy, 1)
        )

        # Data FIFO push/pop on the IDE side. PIO reads fire on RD-fall +
        # DATA-decode + selected; DMA reads fire on RD-fall + DMACK active
        # + dma_mode + selected (DMA cycles deassert CSn, so address decode
        # doesn't apply there).
        with m.If(wr & decode_data_lat & device_selected & write_fifo.w_rdy):
            m.d.comb += [
                write_fifo.w_data.eq(ide_data_latch_d),
                write_fifo.w_en.eq(1),
            ]
            # ATA-5 §9.7 (PIO data-out): the device asserts BSY as soon as the
            # host writes the *last* word of a block. Nothing did, so the host
            # finished a sector, saw BSY clear, concluded the write was done
            # and immediately wrote the next command -- whose `fifo_clear`
            # wiped the sector it had just delivered. The MCU's wait loop
            # sleeps 1 ms, far longer than the ~256-word burst takes, so it
            # only ever woke to an empty FIFO and timed out after 10 s.
            # Counting words here is what makes the host wait for us.
            block_last = Mux(pkt_phase, PACKET_WORDS - 1, BLOCK_WORDS - 1)
            with m.If(pio_wr_words == block_last):
                m.d.sync += pio_wr_words.eq(0)
                with m.If(pio_wr_fc | pkt_phase):
                    m.d.sync += [bsy.eq(1), drq.eq(0)]
                m.d.sync += pkt_phase.eq(0)
            with m.Else():
                m.d.sync += pio_wr_words.eq(pio_wr_words + 1)
        with m.If(wr & dmack_held & dma_wr_mode & device_selected):
            m.d.comb += [
                write_fifo.w_data.eq(ide_data_latch_d),
                write_fifo.w_en.eq(1),
            ]
        with m.If(rd_fall & decode_rd_data_lat & device_selected):
            m.d.comb += read_fifo.r_en.eq(1)
        # Pop on the falling edge, but gated on whether DMACK was asserted
        # *during* the cycle rather than on its value now.
        #
        # rd_fall reports an edge that physically happened 1-1.5 core cycles
        # earlier, while dmack_active is the present state. The Dreamcast's
        # Holly DMA engine moves data in 32-byte bursts (its own progress
        # counter is masked to 32-byte granularity) and releases DMACK
        # immediately after the last read of each burst -- so by the time our
        # synchronized falling edge arrived, DMACK had already gone away and
        # the pop was silently suppressed. The word stayed in the FIFO and the
        # next burst read it again, duplicating one word per burst boundary
        # and shifting everything after it. Measured before this fix: slips at
        # word 0x090, 0x1a0, 0x280, 0x400, 0x420 ... every one a multiple of
        # 16 words = 32 bytes.
        with m.If(rd_fall & dmack_held & dma_rd_mode & device_selected):
            m.d.comb += [read_fifo.r_en.eq(1), rd_dma_pop.eq(1)]

        # The Device register write is unconditional — both devices need to
        # know who is selected after a Device write completes. Use the
        # latched data + latched decode so a too-short WR strobe still
        # gets captured cleanly.
        with m.If(wr & decode_device_lat):
            m.d.sync += device.eq(ide_data_latch[:8])

        # Every other IDE write is gated on `device_selected`.
        sel_wr = wr & device_selected
        with m.If(sel_wr & decode_command_lat):
            m.d.sync += [
                command.eq(ide_data_latch[:8]),
                bsy.eq(1),
                command_pend.eq(1),
                # A new command invalidates the previous transfer's status.
                # Leaving DRQ set let a host that polls immediately after the
                # command write see leftover DRQ, send a sector into a FIFO
                # that fifo_clear then wiped, and wait forever for data the
                # device never received -- status 0xd8 with an empty write
                # FIFO. ATA has the device clear DRQ and ERR when it takes a
                # command; only BSY is meaningful until it responds.
                drq.eq(0),
                err.eq(0),
                pio_wr_words.eq(0),
                pkt_phase.eq(0),
            ]
            # PACKET: open the CDB phase right here (IRR C/D=1, DRQ=1,
            # BSY=0, the 6-word count armed) instead of leaving BSY up for
            # the ~100 us the MCU needs to take its interrupt and get round
            # to it. A real drive (and iceGDROM, inside its command IRQ)
            # shows DRQ within microseconds; Sega Rally 2's WinCE driver
            # polls Alt Status a handful of times straight after the
            # Command write, sees BSY, and goes to sleep waiting for an
            # interrupt a CDB phase never generates.
            if self.primary:
                with m.If(ide_data_latch[:8] == 0xA0):
                    m.d.sync += [
                        bsy.eq(0),
                        drq.eq(1),
                        pkt_phase.eq(1),
                        sector_count[:8].eq(0x01),
                        abrt.eq(0),
                        sense_key.eq(0),
                    ]
            # Reset both FIFOs on every new IDE command — a fresh command
            # always starts a fresh transfer, and stale bytes from a
            # previous response/packet would otherwise corrupt it.
            m.d.comb += fifo_clear.eq(1)
        with m.If(sel_wr & decode_features_lat):
            m.d.sync += features.eq(ide_data_latch[:8])
        with m.If(sel_wr & decode_sector_count_lat):
            # LBA48 sliding window: current → prev, new → current.
            m.d.sync += [
                sector_count[8:].eq(sector_count[:8]),
                sector_count[:8].eq(ide_data_latch[:8]),
            ]
        with m.If(sel_wr & decode_lba_low_lat):
            m.d.sync += [
                lba_low_prev.eq(lba_low),
                lba_low.eq(ide_data_latch[:8]),
            ]
        with m.If(sel_wr & decode_lba_mid_lat):
            m.d.sync += [
                lba_mid_prev.eq(lba_mid),
                lba_mid.eq(ide_data_latch[:8]),
            ]
        with m.If(sel_wr & decode_lba_high_lat):
            m.d.sync += [
                lba_high_prev.eq(lba_high),
                lba_high.eq(ide_data_latch[:8]),
            ]
        # Device Control is written to both devices regardless of selection
        # (ATA/ATAPI-5 7.9): SRST resets the pair, and each keeps its own
        # nIEN for when it is selected.
        with m.If(wr & decode_device_control_lat):
            m.d.sync += [
                hob.eq(ide_data_latch[7]),
                srst.eq(ide_data_latch[2]),
                n_ien.eq(ide_data_latch[1]),
            ]
        # A reset drops any transfer in flight along with its FIFO contents.
        with m.If(in_reset):
            m.d.comb += fifo_clear.eq(1)

        # ----- IDE reads (combinational) -----------------------------------
        # Default to 0xFFFF for unmapped addresses. During a DMA read cycle
        # the host deasserts CSn (so the address decoder sees nothing), but
        # we still need to drive the FIFO data on DC_DATA — gate that path
        # directly on dmack_active rather than the decode lines.
        # DRQ as the host sees it, not as the MCU set it: while PIO flow
        # control is armed and the relevant FIFO cannot service a transfer,
        # drop it so the host waits. Declared unconditionally so it is a real
        # signal everywhere, not only inside the status branch of the mux.
        drq_out = Signal()
        m.d.comb += drq_out.eq(
            drq
            & ~(pio_wr_fc & ~write_fifo.w_rdy)
            & ~(pio_rd_fc & ~read_fifo.r_rdy)
        )
        # A data-in DRQ block ends the instant the host pops the last word,
        # and ATAPI (and iceGDROM's IDE_IOTARGET) then shows BSY until the
        # device has the next block or the completion status ready. Showing
        # 0x50 there instead reads as "command complete" to a host polling
        # right after its last Data read -- the MCU cannot raise BSY within
        # the microsecond that leaves. Read flow control is only armed for
        # the duration of a data-in phase, so an empty read FIFO then means
        # exactly "between blocks".
        bsy_out = Signal()
        m.d.comb += bsy_out.eq(bsy | (drq & pio_rd_fc & ~read_fifo.r_rdy))

        ide_out = Signal(16, reset=0xFFFF)
        with m.If(dmack_active & dma_rd_mode):
            m.d.comb += ide_out.eq(read_fifo.r_data)
        with m.Elif(decode.data):
            m.d.comb += ide_out.eq(read_fifo.r_data)
        with m.Elif(decode.error):
            # ERROR layout: [_, _, ABRT, _, sk0, sk1, sk2, sk3]
            m.d.comb += ide_out[:8].eq(Cat(C(0, 2), abrt, C(0, 1), sense_key))
        with m.Elif(decode.sector_count):
            m.d.comb += ide_out[:8].eq(Mux(hob, sector_count[8:], sector_count[:8]))
        with m.Elif(decode.lba_low):
            m.d.comb += ide_out[:8].eq(Mux(hob, lba_low_prev, lba_low))
        with m.Elif(decode.lba_mid):
            m.d.comb += ide_out[:8].eq(Mux(hob, lba_mid_prev, lba_mid))
        with m.Elif(decode.lba_high):
            m.d.comb += ide_out[:8].eq(Mux(hob, lba_high_prev, lba_high))
        with m.Elif(decode.device):
            m.d.comb += ide_out[:8].eq(device)
        with m.Elif(decode.status | decode.alt_status):
            # Bit layout: BSY[7] DRDY[6] DF[5] DSC[4] DRQ[3] -[2] -[1] ERR[0]
            m.d.comb += ide_out[:8].eq(Cat(err, C(0, 2), drq_out, dsc, df, drdy, bsy_out))
        m.d.comb += self.ide_data_out.eq(ide_out)

        # ----- optional signal capture (slots 20/21) -----------------------
        # One sample per IDE write strobe, triggered by the MCU arming slot
        # 20, read back through slot 21 (write index, read value). This is
        # what showed that a PIO data-out phase really does deliver all 256
        # words into the write FIFO, after three wrong hypotheses about why it
        # appeared not to.
        cap_arm = Signal()
        cap_run = Signal()
        CAP_DEPTH = 512
        cap_wptr = Signal(range(CAP_DEPTH + 1))
        cap_rptr = Signal(range(CAP_DEPTH))
        cap_rdata = Signal(16)
        if self.capture:
            cap_mem = Memory(shape=unsigned(16), depth=CAP_DEPTH, init=[])
            m.submodules.cap_mem = cap_mem
            cap_wr = cap_mem.write_port()
            cap_rd = cap_mem.read_port(domain="comb")
            # Aimed at the IDE write stream: one sample per write strobe
            # with the byte, the raw address/chip-select lines and whether the
            # decoder called it a command. That is what showed a *real*
            # command write arriving in the middle of a PIO data-out burst,
            # rather than a data word being misdecoded as one.
            cap_sample = Cat(
                ide_data_latch_d[:8],   # 0..7   the byte written
                self.ide_addr,          # 8..10  raw address lines
                self.ide_csn,           # 11..12 raw chip selects
                decode_command_lat,     # 13     decoded as a COMMAND write?
                bsy,                    # 14
                drq,                    # 15
            )
            cap_trig = sel_wr
            with m.If(cap_arm & cap_trig & ~cap_run):
                m.d.sync += [cap_run.eq(1), cap_arm.eq(0)]
            with m.If(cap_run & cap_trig):
                m.d.comb += [
                    cap_wr.addr.eq(cap_wptr),
                    cap_wr.data.eq(cap_sample),
                    cap_wr.en.eq(1),
                ]
                m.d.sync += cap_wptr.eq(cap_wptr + 1)
                with m.If(cap_wptr == CAP_DEPTH - 1):
                    m.d.sync += cap_run.eq(0)
            m.d.comb += [cap_rd.addr.eq(cap_rptr), cap_rdata.eq(cap_rd.data)]

        # Drive `ide_data_out_en` only when this device is selected and the
        # IDE bus is reading a register we own. Top-level uses this to mux
        # between primary and secondary on the shared IDE_DATA_OUT line.
        any_read_decode = (
            decode.data | decode.error | decode.sector_count | decode.lba_low
            | decode.lba_mid | decode.lba_high | decode.device | decode.status
            | decode.alt_status | decode.debug_testpad
        )
        m.d.comb += self.ide_data_out_en.eq(
            device_selected & self.ide_rd
            & (any_read_decode | (dmack_active & dma_rd_mode))
        )

        # IDE INTRQ logic. Real ATAPI semantics: INTRQ is asserted when
        # the device wants the host's attention (transition into DRQ=1
        # for data-in, command-done) and is cleared by the host reading
        # STATUS. The only source is the `intrq_pending` flop the MCU
        # pulses high after staging a response or completing a command;
        # an IDE-side read of the STATUS (or COMMAND) register clears it,
        # just like real hardware.
        #
        # `command_pend` used to be OR'd in as well, which asserted INTRQ
        # from the Command write until the MCU acknowledged it -- i.e.
        # while BSY=1 and, for PACKET, across the CDB phase. ATA forbids
        # INTRQ while BSY=1 and a GD-ROM does not interrupt for the CDB
        # (iceGDROM sets DRQ for it without an IRQ). The BIOS shrugged it
        # off; an interrupt-driven kernel (WinCE titles) gets a spurious
        # IRQ with BSY set before every packet.
        intrq_pending = Signal()
        m.d.comb += self.ide_intrq.eq(
            device_selected & ~n_ien & intrq_pending
        )
        # Clear `intrq_pending` when the host reads the Status register.
        # Use the synchronized falling-edge of ide_rd to match the FIFO
        # pop semantics already wired up above for the Data register.
        with m.If(rd_fall & decode_rd_status_lat & device_selected):
            m.d.sync += intrq_pending.eq(0)
        # ...and when it writes the Command register (ATA-5 DI1: "when the
        # Command register is written, the device shall exit the interrupt
        # pending state"). Otherwise a completion IRQ the host never acked
        # by a Status read -- the BIOS's DMA path reads Status before we
        # post it -- stays asserted into the next command, and a driver
        # that only then unmasks G1 starts with a spurious interrupt.
        with m.If(sel_wr & decode_command_lat):
            m.d.sync += intrq_pending.eq(0)

        # ----- host access counters (slots 24..26) -------------------------
        # What the host did since the last Command write. When a PIO data-in
        # phase stalls ("host never drained FIFO") these say whether it is
        # polling Status, only peeking at Alt Status, reading the Data
        # register at all, or sitting idle waiting for an interrupt. Byte
        # counters saturate, the data counter wraps.
        host_status_rds = Signal(8)
        host_altstat_rds = Signal(8)
        host_data_rds = Signal(16)
        devctl_last = Signal(8)
        devctl_writes = Signal(8)
        # Frozen copies of the previous command's counts, so the firmware
        # can report them after the host has already written the next
        # command (a fast host clears the live ones before the MCU looks).
        prev_status_rds = Signal(8)
        prev_altstat_rds = Signal(8)
        prev_data_rds = Signal(16)
        host_rd = rd_fall & device_selected
        with m.If(sel_wr & decode_command_lat):
            m.d.sync += [
                prev_status_rds.eq(host_status_rds),
                prev_altstat_rds.eq(host_altstat_rds),
                prev_data_rds.eq(host_data_rds),
                host_status_rds.eq(0),
                host_altstat_rds.eq(0),
                host_data_rds.eq(0),
            ]
        with m.Else():
            with m.If(host_rd & decode_rd_status_lat & (host_status_rds != 0xFF)):
                m.d.sync += host_status_rds.eq(host_status_rds + 1)
            with m.If(host_rd & decode_rd_alt_status_lat & (host_altstat_rds != 0xFF)):
                m.d.sync += host_altstat_rds.eq(host_altstat_rds + 1)
            with m.If(host_rd & decode_rd_data_lat):
                m.d.sync += host_data_rds.eq(host_data_rds + 1)
        with m.If(wr & decode_device_control_lat):
            m.d.sync += [
                devctl_last.eq(ide_data_latch[:8]),
                devctl_writes.eq(devctl_writes + 1),
            ]

        # ----- MCU window --------------------------------------------------
        # Layout (word addresses; matches docs/AddressMapMCU.md):
        #   0  HW info      bit 0 = !primary, bit 1 = COMMAND_PEND
        #   1  Status       ERR/DRQ/DF/DRDY/BSY
        #   2  Device       8b
        #   3  Error        bit 2 = ABRT
        #   4  Command lo / Features hi
        #   5  Sector Count 16b
        #   6  LBA Low / LBA Mid
        #   7  LBA High / LBA Low Prev
        #   8  LBA Mid Prev / LBA High Prev
        #   9  Data FIFO    (read = pop write_fifo; write = push read_fifo)
        #  10  Read FIFO occupancy
        #  11  Write FIFO occupancy
        #  12  Debug test   reads 0xC0DE
        # The SMC read is a pure combinational cone: MCU_ADDR -> decode -> a
        # 22-way mux -> the data pins, with nothing registered and no timing
        # constraint on any of it (56 unconstrained input ports). Its budget is
        # the SMC access -- 1 MCK setup + 8 MCK pulse at 96 MHz, so ~90 ns
        # minus SAM3U setup and board delay -- on a Cyclone II C8. smc.rs
        # already records that shortening the pulse garbled write-FIFO reads,
        # so this path was known to be marginal, and every case added to the
        # mux makes it worse.
        #
        # Register the mux output so the SMC samples a flop instead. The
        # address is stable for the whole access and an FPGA cycle is 21 ns, so
        # the extra cycle costs nothing.
        mcu_out = Signal(16)
        mcu_out_r = Signal(16)
        lvl_at_read = Signal(16)
        lvl_read_count = Signal(16)
        m.d.sync += mcu_out_r.eq(mcu_out)
        m.d.comb += self.mcu_data_out.eq(mcu_out_r)

        wmcu = self.mcu_cs & self.mcu_wr

        # Diagnostic counters (slots 14..17). They answer, in order:
        #   "is the FPGA core clocked?" (free_run delta)
        #   "is the host strobing IDE?" (ide_strobes)
        #   "did the DC issue a bus reset?" (rstn edge counters)
        #   "what does the IDE bus look like right now?" (live snapshot)
        # The MCU polls all of these from its diag loop; values survive
        # DC power cycles because nothing here depends on the DC being up.
        diag_free_run = Signal(16)
        m.d.sync += diag_free_run.eq(diag_free_run + 1)

        diag_ide_strobes = Signal(16)
        with m.If(wr | rd_fall):
            m.d.sync += diag_ide_strobes.eq(diag_ide_strobes + 1)

        # ide_rstn edge counters. If the DC never asserts/deasserts RSTn,
        # neither counter ever moves — meaning the DC's south-bridge is
        # not issuing a GD-ROM reset cycle, which is itself diagnostic.
        rstn_sync = Signal(2, reset=0b11)
        m.d.sync += rstn_sync.eq(Cat(self.ide_rstn, rstn_sync[:-1]))
        rstn_clean = rstn_sync[1]
        rstn_clean_prev = Signal(reset=1)
        m.d.sync += rstn_clean_prev.eq(rstn_clean)
        rstn_fall_evt = rstn_clean_prev & ~rstn_clean
        rstn_rise_evt = ~rstn_clean_prev & rstn_clean

        diag_rstn_falls = Signal(8)
        diag_rstn_rises = Signal(8)
        with m.If(rstn_fall_evt):
            m.d.sync += diag_rstn_falls.eq(diag_rstn_falls + 1)
        with m.If(rstn_rise_evt):
            m.d.sync += diag_rstn_rises.eq(diag_rstn_rises + 1)

        # Live IDE bus snapshot (combinational) — captures what the DC
        # is actually putting on the cable RIGHT NOW. If everything reads
        # back as 1's the DC isn't driving anything; if RSTn is stuck
        # low the DC is holding us in reset; etc.
        diag_bus_state = Signal(8)
        m.d.comb += diag_bus_state.eq(Cat(
            self.ide_rstn,            # bit 0: 1 = out of reset
            self.ide_csn[0],          # bit 1: 1 = CS0 deasserted
            self.ide_csn[1],          # bit 2: 1 = CS1 deasserted
            self.ide_rd,              # bit 3: 1 = host reading
            self.ide_wr,              # bit 4: 1 = host writing
            Const(0, 3),
        ))

        # audio_tick: cross-clock heartbeat from CLK_11_2896_MHz. Counting
        # both edges of a divider bit gives a steady tick rate proportional
        # to the audio-xtal frequency. If diag_audio_alive doesn't change
        # between MCU reads, the audio crystal is dead or unrouted.
        audio_sync = Signal(2)
        m.d.sync += audio_sync.eq(Cat(self.audio_tick, audio_sync[:-1]))
        audio_clean = audio_sync[1]
        audio_clean_prev = Signal()
        m.d.sync += audio_clean_prev.eq(audio_clean)
        audio_edge = audio_clean ^ audio_clean_prev
        diag_audio_alive = Signal(16)
        with m.If(audio_edge):
            m.d.sync += diag_audio_alive.eq(diag_audio_alive + 1)

        # MCU bus edge detection — FIFO push/pop must fire exactly once per
        # access, but the SAM3U SMC strobe is many FPGA cycles wide.
        mcu_wr_active = self.mcu_cs & self.mcu_wr
        mcu_rd_active = self.mcu_cs & self.mcu_rd
        mcu_wr_active_prev = Signal()
        mcu_rd_active_prev = Signal()
        m.d.sync += [
            mcu_wr_active_prev.eq(mcu_wr_active),
            mcu_rd_active_prev.eq(mcu_rd_active),
        ]
        mcu_wr_edge = mcu_wr_active_prev & ~mcu_wr_active
        mcu_rd_edge = mcu_rd_active_prev & ~mcu_rd_active

        with m.Switch(self.mcu_addr):
            with m.Case(0):
                # bit 2: sticky "a reset (hard or SRST) happened". Write 1
                # to clear. The firmware uses it to abandon a transfer the
                # host reset out from under it instead of waiting 10 s for
                # a drain that will never come.
                # bit 3: live nIEN, bit 4: live in-reset (diagnostics).
                m.d.comb += [
                    mcu_out[0].eq(0 if self.primary else 1),
                    mcu_out[1].eq(command_pend),
                    mcu_out[2].eq(reset_seen),
                    mcu_out[3].eq(n_ien),
                    mcu_out[4].eq(in_reset),
                ]
                with m.If(wmcu):
                    m.d.sync += command_pend.eq(self.mcu_data_in[1])
                    with m.If(self.mcu_data_in[2]):
                        m.d.sync += reset_seen.eq(0)
            with m.Case(1):
                m.d.comb += mcu_out[:8].eq(
                    Cat(err, C(0, 2), drq, dsc, df, drdy, bsy)
                )
                # Bit 8: assert INTRQ in the same clock as the new status.
                # Posting them as two MCU writes (status, then slot 13) left
                # a window where a host polling Alt Status saw BSY drop,
                # read Status, and wrote its next command before INTRQ
                # rose -- so the IRQ landed on the *next* command as a
                # spurious one. Quake III's WinCE driver then went to
                # sleep on the real DRQ interrupt and never woke: its
                # small PIO reads timed out and it fell back to the BIOS
                # menu. ATA has the device raise INTRQ together with the
                # BSY-to-0 / DRQ transition, which is what this bit does.
                with m.If(wmcu):
                    m.d.sync += [
                        err.eq(self.mcu_data_in[0]),
                        drq.eq(self.mcu_data_in[3]),
                        dsc.eq(self.mcu_data_in[4]),
                        df.eq(self.mcu_data_in[5]),
                        drdy.eq(self.mcu_data_in[6]),
                        bsy.eq(self.mcu_data_in[7]),
                    ]
                    with m.If(self.mcu_data_in[8]):
                        m.d.sync += intrq_pending.eq(1)
            with m.Case(2):
                m.d.comb += mcu_out[:8].eq(device)
                with m.If(wmcu):
                    m.d.sync += device.eq(self.mcu_data_in[:8])
            with m.Case(3):
                # Mirrors the IDE-side ERROR register: bit 2 = ABRT,
                # bits 4..7 = sense key. MCU writes a full ERROR-register
                # byte; the FPGA captures the bits we model.
                m.d.comb += [
                    mcu_out[2].eq(abrt),
                    mcu_out[4:8].eq(sense_key),
                ]
                with m.If(wmcu):
                    m.d.sync += [
                        abrt.eq(self.mcu_data_in[2]),
                        sense_key.eq(self.mcu_data_in[4:8]),
                    ]
            with m.Case(4):
                m.d.comb += [
                    mcu_out[:8].eq(command),
                    mcu_out[8:].eq(features),
                ]
                with m.If(wmcu):
                    m.d.sync += [
                        command.eq(self.mcu_data_in[:8]),
                        features.eq(self.mcu_data_in[8:]),
                    ]
            with m.Case(5):
                m.d.comb += mcu_out.eq(sector_count)
                with m.If(wmcu):
                    m.d.sync += sector_count.eq(self.mcu_data_in)
            with m.Case(6):
                m.d.comb += [
                    mcu_out[:8].eq(lba_low),
                    mcu_out[8:].eq(lba_mid),
                ]
                with m.If(wmcu):
                    m.d.sync += [
                        lba_low.eq(self.mcu_data_in[:8]),
                        lba_mid.eq(self.mcu_data_in[8:]),
                    ]
            with m.Case(7):
                m.d.comb += [
                    mcu_out[:8].eq(lba_high),
                    mcu_out[8:].eq(lba_low_prev),
                ]
                with m.If(wmcu):
                    m.d.sync += [
                        lba_high.eq(self.mcu_data_in[:8]),
                        lba_low_prev.eq(self.mcu_data_in[8:]),
                    ]
            with m.Case(8):
                m.d.comb += [
                    mcu_out[:8].eq(lba_mid_prev),
                    mcu_out[8:].eq(lba_high_prev),
                ]
                with m.If(wmcu):
                    m.d.sync += [
                        lba_mid_prev.eq(self.mcu_data_in[:8]),
                        lba_high_prev.eq(self.mcu_data_in[8:]),
                    ]
            with m.Case(9):
                # Reads pop the write_fifo (data the IDE side has written
                # for the MCU to consume). Writes push into the read_fifo
                # (data the MCU is staging for the IDE side to read).
                m.d.comb += mcu_out.eq(write_fifo.r_data)
                with m.If(mcu_rd_edge):
                    m.d.comb += write_fifo.r_en.eq(1)
                with m.If(mcu_wr_edge):
                    m.d.comb += [
                        read_fifo.w_data.eq(self.mcu_data_in),
                        read_fifo.w_en.eq(1),
                    ]
            with m.Case(10):
                m.d.comb += mcu_out.eq(read_fifo.level)
            with m.Case(11):
                m.d.comb += mcu_out.eq(write_fifo.level)
                # Latch what the level actually was during this read, and count
                # the reads. Reading that back through slot 22 separates the
                # two remaining explanations for the MCU seeing 0 while the
                # in-fabric capture shows 256: either this read path is
                # returning the wrong thing, or the two are not looking at the
                # same FIFO.
                with m.If(self.mcu_cs & self.mcu_rd):
                    m.d.sync += [
                        lvl_at_read.eq(write_fifo.level),
                        lvl_read_count.eq(lvl_read_count + 1),
                    ]
            with m.Case(22):
                m.d.comb += mcu_out.eq(lvl_at_read)
            with m.Case(23):
                m.d.comb += mcu_out.eq(lvl_read_count)
            with m.Case(24):
                # Host Status reads (low) / Alt Status reads (high) since
                # the last Command write.
                m.d.comb += mcu_out.eq(Cat(host_status_rds, host_altstat_rds))
            with m.Case(25):
                # Host Data-register reads since the last Command write.
                m.d.comb += mcu_out.eq(host_data_rds)
            with m.Case(26):
                # Last Device Control byte written (low) / write count (high).
                m.d.comb += mcu_out.eq(Cat(devctl_last, devctl_writes))
            with m.Case(27):
                # Slot 24 as it stood when the current command was written.
                m.d.comb += mcu_out.eq(Cat(prev_status_rds, prev_altstat_rds))
            with m.Case(28):
                # Slot 25 as it stood when the current command was written.
                m.d.comb += mcu_out.eq(prev_data_rds)
            with m.Case(12):
                m.d.comb += mcu_out.eq(0xC0DE)
            with m.Case(13):
                # IDE INTRQ pending. MCU writes 1 to assert (host will
                # see INTRQ high until it reads STATUS); reads the
                # current state. Bit 0 is the only data bit — high byte
                # is reserved zero for now.
                m.d.comb += mcu_out[0].eq(intrq_pending)
                with m.If(wmcu):
                    m.d.sync += intrq_pending.eq(self.mcu_data_in[0])
            with m.Case(14):
                # Free-running counter on the core clock. If this changes
                # between successive MCU reads the FPGA core is clocked.
                m.d.comb += mcu_out.eq(diag_free_run)
            with m.Case(15):
                # Counts completed IDE strobes (read or write) from the host.
                # Stays at 0 if the DC is not touching the GD-ROM bus.
                m.d.comb += mcu_out.eq(diag_ide_strobes)
            with m.Case(16):
                # ide_rstn edges: low byte = rising edges (DC released
                # reset), high byte = falling edges (DC asserted reset).
                m.d.comb += mcu_out.eq(Cat(diag_rstn_rises, diag_rstn_falls))
            with m.Case(17):
                # Live IDE bus snapshot. See diag_bus_state above for layout.
                m.d.comb += mcu_out[:8].eq(diag_bus_state)
            with m.Case(18):
                # Audio-xtal alive counter. Nonzero delta proves the
                # 11.2896 MHz crystal is reaching the FPGA and routing
                # through fabric to where we'd drive DC_CDCLK from.
                m.d.comb += mcu_out.eq(diag_audio_alive)
            with m.Case(20):
                # Capture control: write 1 to arm. Reads back
                # [0]=armed [1]=running [2]=done, [15:8]=events captured >> 1.
                m.d.comb += [
                    mcu_out[0].eq(cap_arm),
                    mcu_out[1].eq(cap_run),
                    mcu_out[2].eq((cap_wptr == CAP_DEPTH) & ~cap_run),
                    mcu_out[8:].eq(cap_wptr[1:]),
                ]
                with m.If(wmcu):
                    with m.If(self.mcu_data_in[0]):
                        m.d.sync += [
                            cap_arm.eq(1), cap_run.eq(0),
                            cap_wptr.eq(0), cap_rptr.eq(0),
                        ]
                    with m.Else():
                        m.d.sync += [cap_arm.eq(0), cap_run.eq(0)]
            with m.Case(21):
                # Write the index, then read the value. No auto-increment: an
                # SMC read is a multi-cycle level, not a pulse.
                m.d.comb += mcu_out.eq(cap_rdata)
                with m.If(wmcu):
                    m.d.sync += cap_rptr.eq(self.mcu_data_in[:9])
            with m.Case(19):
                # DMA mode. MCU writes 1 to arm DMA before staging the
                # data-in payload of an ATAPI DMA-mode PACKET; writes 0
                # after the host has drained the transfer. While 1, the
                # FPGA asserts DMARQ whenever the read FIFO has data.
                m.d.comb += [
                    mcu_out[:2].eq(dma_mode),
                    mcu_out[2].eq(pio_wr_fc),
                    mcu_out[3].eq(pio_rd_fc),
                    mcu_out[4].eq(pkt_phase),
                ]
                with m.If(wmcu):
                    m.d.sync += [
                        dma_mode.eq(self.mcu_data_in[:2]),
                        pio_wr_fc.eq(self.mcu_data_in[2]),
                        pio_rd_fc.eq(self.mcu_data_in[3]),
                        pkt_phase.eq(self.mcu_data_in[4]),
                    ]
            with m.Case(29):
                # CDDA control/status (see hw/cdda.py for the bit layout).
                m.d.comb += mcu_out.eq(self.aux_ctrl_rd)
                m.d.comb += self.aux_ctrl_we.eq(wmcu)
            with m.Case(30):
                # CDDA sample FIFO level in 16-bit words.
                m.d.comb += mcu_out.eq(self.aux_level)
            with m.Case(31):
                # CDDA sample push: one 16-bit PCM word per write, L then R.
                m.d.comb += self.aux_push.eq(mcu_wr_edge)
        m.d.comb += self.aux_wdata.eq(self.mcu_data_in)

        # MCU IRQ — assert a few cycles after COMMAND_PEND rises so the
        # write completes on the IDE side before the MCU sees the request.
        irq_delay = Signal(4)
        m.d.sync += irq_delay.eq(Cat(command_pend, irq_delay[:-1]))
        m.d.comb += self.mcu_irq.eq(irq_delay[-1])

        # ----- Post-reset state machine ------------------------------------
        # A reset (DC asserts ide_rstn low, or sets SRST in Device Control)
        # brings the device back to a known state: BSY held while the
        # "drive" comes ready, then a one-shot transition into idle (BSY=0,
        # DRDY=1, DSC=1) with the ATAPI device signature loaded into
        # LBA_LOW/MID/HIGH so the host can identify the device class without
        # sending a command. `in_reset` is defined with the control bits.
        #
        # Counter width comes from `post_reset_count_bits` (default 24 for
        # hardware, ~0.4 s @ 48 MHz). Putting this block at the bottom of
        # elaborate() means its sync writes win over the IDE-side and
        # MCU-side writes above whenever the conditions below fire
        # (Amaranth uses last-assignment-wins ordering for `m.d.sync +=`
        # to the same signal).
        post_reset_count = Signal(self.post_reset_count_bits)
        post_reset_done = post_reset_count[-1]
        post_reset_done_prev = Signal()
        m.d.sync += post_reset_done_prev.eq(post_reset_done)
        post_reset_complete = post_reset_done & ~post_reset_done_prev

        with m.If(in_reset):
            m.d.sync += [
                post_reset_count.eq(0),
                post_reset_done_prev.eq(0),
                # Force the ATA register file into the ATAPI post-reset state.
                bsy.eq(1),
                drdy.eq(0),
                dsc.eq(0),
                drq.eq(0),
                df.eq(0),
                err.eq(0),
                abrt.eq(0),
                sense_key.eq(0),
                command_pend.eq(0),
                reset_seen.eq(1),
                # Whatever transfer was in progress is over: no DMA, no
                # PIO flow control, no half-received CDB, no stale IRQ.
                dma_mode.eq(0),
                pio_wr_fc.eq(0),
                pio_rd_fc.eq(0),
                pkt_phase.eq(0),
                pio_wr_words.eq(0),
                intrq_pending.eq(0),
                # Sega GD-ROM signature: SECTOR_COUNT=01, LBA_LOW=81
                # (disk_type=0x80 GD-ROM | status=0x01 ready), LBA_MID=14,
                # LBA_HIGH=EB. See note above the signal `reset=` values.
                sector_count.eq(0x0001),
                lba_low.eq(sig_low),
                lba_mid.eq(sig_mid),
                lba_high.eq(sig_high),
            ]
        with m.Elif(~post_reset_done):
            m.d.sync += post_reset_count.eq(post_reset_count + 1)
        with m.Elif(post_reset_complete):
            # One-shot rising edge of post_reset_done: the drive is now idle
            # and ready. Subsequent BSY/DRDY/DSC assertions are driven by
            # command writes (IDE side) or the MCU's status writes.
            m.d.sync += [
                bsy.eq(0),
                drdy.eq(1),
                dsc.eq(1),
            ]

        return m
