"""CDDA digital audio output.

The GD-ROM hands the AICA a serial PCM stream on three pins: SCK (bit
clock), LRCK (channel select) and SDAT (data), with CDCLK (33.8688 MHz)
alongside as the audio master clock. The bit format below is the one the
iceGDROM project reverse-engineered from a real drive and ships in
`digital_sound_output.v`:

  * SCK = 64 x 44.1 kHz = 2.8224 MHz, i.e. the 11.2896 MHz audio xtal / 4
  * one 64-bit frame per stereo sample pair; LRCK is high for the first 32
    bits (left) and low for the last 32 (right)
  * each 32-bit half is 16 zero bits followed by the 16-bit sample, MSB
    first (right-justified in the slot)
  * SDAT and LRCK change on the falling edge of SCK

Samples arrive from the MCU as 16-bit words in a small FIFO in the 48 MHz
`sync` domain, in the order they sit in a raw 2352-byte audio sector: left,
right, left, right ... (little-endian words, so the SMC halfword write
carries a whole sample). The serialiser lives in the `audio` domain
(CLK_11_2896_MHz); a toggle handshake carries one stereo pair across per
frame. When the FIFO cannot supply a pair, the frame is sent as silence and
a sticky underflow flag (plus a counter) is raised for the firmware to log.

MCU window (primary device, decoded in ide_device.py):

  slot 29  control/status
             write: bit 0 = enable output, bit 1 = clear underflow flag and
                    counter, bit 2 = flush the FIFO
             read:  bit 0 = enable, bit 1 = underflow (sticky),
                    bits 15:8 = silent frames inserted since last clear
                    (saturating)
  slot 30  FIFO level in words (read only)
  slot 31  sample push (write only): one PCM word per write, L then R

SCK and LRCK free-run regardless of enable, like a real drive; with the
output disabled SDAT carries zeros.
"""

from amaranth import *
from amaranth.hdl.xfrm import ResetInserter
from amaranth.lib.fifo import SyncFIFO


# 3 M4Ks worth of 16-bit words (768 x 16 = 12288 bits): 384 stereo pairs,
# 8.7 ms of audio. The rest of the block RAM is taken by the IDE FIFOs; the
# firmware keeps this topped up from a SysTick hook so the deadline is soft.
AUDIO_FIFO_DEPTH = 768


class CDDAOutput(Elaboratable):
    def __init__(self, depth: int = AUDIO_FIFO_DEPTH):
        self.depth = depth

        # MCU side (sync domain).
        self.ctrl_we = Signal()
        self.ctrl_wdata = Signal(16)
        self.ctrl_rdata = Signal(16)
        self.level = Signal(16)
        self.push = Signal()
        self.push_data = Signal(16)

        # Dreamcast side (audio domain).
        self.sck = Signal()
        self.sdat = Signal()
        self.lrck = Signal()

        # Simulation visibility: frames handed to the serialiser.
        self.frames = Signal(16)

    def elaborate(self, platform):
        m = Module()

        # ----- MCU-facing state (sync) ------------------------------------
        enable = Signal()
        underflow = Signal()
        uf_count = Signal(8)
        clear = Signal()

        m.submodules.fifo = fifo = ResetInserter(clear)(
            SyncFIFO(width=16, depth=self.depth)
        )
        m.d.comb += [
            fifo.w_en.eq(self.push),
            fifo.w_data.eq(self.push_data),
            fifo.r_en.eq(0),
            self.level.eq(fifo.level),
            self.ctrl_rdata[0].eq(enable),
            self.ctrl_rdata[1].eq(underflow),
            self.ctrl_rdata[8:].eq(uf_count),
            clear.eq(self.ctrl_we & self.ctrl_wdata[2]),
        ]
        with m.If(self.ctrl_we):
            m.d.sync += enable.eq(self.ctrl_wdata[0])
            with m.If(self.ctrl_wdata[1]):
                m.d.sync += [underflow.eq(0), uf_count.eq(0)]

        # Next stereo pair, prefetched from the FIFO so it is ready the
        # moment the serialiser asks. Low half = left, high half = right.
        pair = Signal(32)
        pair_valid = Signal()
        fetch_second = Signal()
        # Register the serialiser reads at frame start. It is rewritten
        # within a few sync cycles of the request and then sits still for
        # the remaining ~22 us of the frame, so the audio side always
        # samples a long-settled value.
        pair_out = Signal(32)

        # Frame-start request from the audio domain: a toggle, synchronised.
        req_a = Signal()
        req_sync = Signal(3)
        m.d.sync += req_sync.eq(Cat(req_a, req_sync[:-1]))
        req_edge = req_sync[1] ^ req_sync[2]

        with m.If(req_edge):
            m.d.sync += [
                pair_out.eq(Mux(pair_valid, pair, 0)),
                pair_valid.eq(0),
                fetch_second.eq(0),
                self.frames.eq(self.frames + 1),
            ]
            with m.If(enable & ~pair_valid):
                m.d.sync += underflow.eq(1)
                with m.If(uf_count != 0xFF):
                    m.d.sync += uf_count.eq(uf_count + 1)
        with m.Elif(enable & ~pair_valid):
            with m.If(~fetch_second):
                # Only start once both words are there so left/right never
                # get out of step across an underflow.
                with m.If(fifo.level >= 2):
                    m.d.comb += fifo.r_en.eq(1)
                    m.d.sync += [
                        pair[:16].eq(fifo.r_data),
                        fetch_second.eq(1),
                    ]
            with m.Else():
                m.d.comb += fifo.r_en.eq(1)
                m.d.sync += [
                    pair[16:].eq(fifo.r_data),
                    pair_valid.eq(1),
                    fetch_second.eq(0),
                ]
        with m.If(~enable):
            m.d.sync += [pair_valid.eq(0), fetch_second.eq(0)]

        # ----- Serialiser (audio) -----------------------------------------
        # phase[1:0] divides the xtal by 4 for SCK; phase[7:2] is the bit
        # index within the 64-bit frame.
        # Starts at -1 so the very first edge is a proper frame start.
        phase = Signal(8, init=0xFF)
        nxt = Signal(8)
        shift = Signal(32)
        m.d.comb += nxt.eq(phase + 1)
        m.d.audio += [
            phase.eq(nxt),
            self.sck.eq(nxt[1]),
        ]
        bit = nxt[2:8]
        # Everything below lands on the same edge SCK falls on.
        with m.If(nxt[:2] == 0):
            m.d.audio += self.lrck.eq(~bit[5])
            with m.If(bit == 0):
                # Frame start: latch the pair (left goes out first, so it
                # takes the top of the shift register) and ask for the next.
                m.d.audio += [
                    shift.eq(Cat(pair_out[16:], pair_out[:16])),
                    req_a.eq(~req_a),
                    self.sdat.eq(0),
                ]
            with m.Elif(bit[4]):
                # Bits 16..31 of each half: the sample, MSB first.
                m.d.audio += [
                    self.sdat.eq(shift[31]),
                    shift.eq(Cat(C(0, 1), shift[:31])),
                ]
            with m.Else():
                m.d.audio += self.sdat.eq(0)

        return m
