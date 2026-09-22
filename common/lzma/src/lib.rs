//! A pull-style LZMA1 decoder small enough to sit in the boot path.
//!
//! Scope is deliberately narrow: raw LZMA1 streams (no .xz/.lzma container,
//! no properties byte) encoded with `lc=0 lp=0 pb=0` and a 4 KiB dictionary,
//! which is exactly what `build.rs` produces for the FPGA bitstream. Fixing
//! those three parameters at compile time is what keeps this small — the
//! literal coder collapses to a single 0x300-entry table instead of
//! `0x300 << lc`, and every `posState` index disappears.
//!
//! The API is a byte-at-a-time pull (`next_byte`) rather than
//! decompress-into-a-buffer, because the caller bit-bangs each byte into the
//! FPGA as it appears and never has room to hold the 72 KiB result.
//!
//! Follows the reference decoder in LzmaSpec.c; the variable names are kept
//! close to it so the two can be read side by side.

#![no_std]

/// Dictionary size. Must match the `dict=` the encoder was given, and must be
/// a power of two (the ring buffer indexes with a mask).
pub const DICT_SIZE: usize = 4096;
const DICT_MASK: usize = DICT_SIZE - 1;

const NUM_STATES: usize = 12;
const PROB_BITS: u32 = 11;
const PROB_INIT: u16 = (1 << PROB_BITS) / 2;
const MOVE_BITS: u32 = 5;
const TOP: u32 = 1 << 24;

const MATCH_MIN_LEN: u32 = 2;
const END_POS_MODEL_INDEX: u32 = 14;
const NUM_FULL_DISTANCES: u32 = 1 << (END_POS_MODEL_INDEX >> 1);
const NUM_ALIGN_BITS: u32 = 4;
const NUM_LEN_TO_POS_STATES: u32 = 4;

/// A length coder: 2 choice bits, then one of three bit trees. `pb=0` means
/// the low and mid coders have a single posState each rather than 16.
const LEN_CODER_SIZE: usize = 2 + 8 + 8 + 256;
const LEN_CHOICE: usize = 0;
const LEN_CHOICE2: usize = 1;
const LEN_LOW: usize = 2;
const LEN_MID: usize = 2 + 8;
const LEN_HIGH: usize = 2 + 8 + 8;

// Probability array layout. One flat array keeps `Scratch` a single
// allocation the caller can carve out of an existing buffer.
const OFF_IS_MATCH: usize = 0;
const OFF_IS_REP: usize = OFF_IS_MATCH + NUM_STATES;
const OFF_IS_REP_G0: usize = OFF_IS_REP + NUM_STATES;
const OFF_IS_REP_G1: usize = OFF_IS_REP_G0 + NUM_STATES;
const OFF_IS_REP_G2: usize = OFF_IS_REP_G1 + NUM_STATES;
const OFF_IS_REP0_LONG: usize = OFF_IS_REP_G2 + NUM_STATES;
const OFF_POS_SLOT: usize = OFF_IS_REP0_LONG + NUM_STATES;
const OFF_SPEC_POS: usize = OFF_POS_SLOT + (NUM_LEN_TO_POS_STATES as usize * 64);
const OFF_ALIGN: usize = OFF_SPEC_POS + (1 + NUM_FULL_DISTANCES as usize - END_POS_MODEL_INDEX as usize);
const OFF_LEN: usize = OFF_ALIGN + (1 << NUM_ALIGN_BITS);
const OFF_REP_LEN: usize = OFF_LEN + LEN_CODER_SIZE;
const OFF_LITERAL: usize = OFF_REP_LEN + LEN_CODER_SIZE;

/// Total probability slots. 1775 u16s = 3550 bytes.
pub const NUM_PROBS: usize = OFF_LITERAL + 0x300;

/// Working memory for a decode: the probability model plus the dictionary
/// window. Kept out of the `Decoder` itself so the caller can place it
/// wherever the RAM is — on this firmware it is carved out of a buffer that
/// is not live yet, so the decode costs no additional static RAM.
#[repr(C, align(4))]
pub struct Scratch {
    probs: [u16; NUM_PROBS],
    dict: [u8; DICT_SIZE],
}

/// Bytes needed to hold a [`Scratch`].
pub const SCRATCH_BYTES: usize = core::mem::size_of::<Scratch>();

impl Scratch {
    /// Reinterpret the head of `buf` as scratch space.
    ///
    /// The start is rounded up to 4-byte alignment rather than demanded,
    /// so callers can hand over any borrowed buffer without arranging its
    /// alignment; `buf` therefore needs [`SCRATCH_BYTES`] plus up to 3 bytes
    /// of slack. Returns `None` if it is too short.
    ///
    /// The contents are fully initialised by [`Decoder::new`], so whatever
    /// was in `buf` is irrelevant — but it *is* destroyed.
    pub fn from_buf(buf: &mut [u8]) -> Option<&mut Scratch> {
        let pad = buf.as_ptr().align_offset(core::mem::align_of::<Scratch>());
        let buf = buf.get_mut(pad..)?;
        if buf.len() < SCRATCH_BYTES {
            return None;
        }
        // SAFETY: length and alignment checked above, and `Scratch` is a
        // `repr(C)` aggregate of integer arrays, so every bit pattern is a
        // valid value. The borrow is reborrowed from `buf`, so the caller
        // cannot use `buf` again while the scratch lives.
        Some(unsafe { &mut *(buf.as_mut_ptr() as *mut Scratch) })
    }
}

/// Why a decode stopped early.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    /// The stream ended before `out_len` bytes had been produced.
    Truncated,
    /// A range-coder or distance check failed: the stream is not a valid
    /// LZMA1 stream with these parameters.
    Corrupt,
}

pub struct Decoder<'a> {
    input: &'a [u8],
    in_pos: usize,
    range: u32,
    code: u32,
    s: &'a mut Scratch,
    dict_pos: usize,
    total_out: usize,
    out_len: usize,
    state: u32,
    reps: [u32; 4],
    /// Bytes still owed by the match currently being emitted.
    rem_len: u32,
    err: Option<Error>,
    done: bool,
}

impl<'a> Decoder<'a> {
    /// Start decoding `input`, which must expand to exactly `out_len` bytes.
    ///
    /// `out_len` is what terminates the decode: raw LZMA1 carries no length,
    /// and relying on the end-of-stream marker alone would mean trusting the
    /// stream to contain one.
    pub fn new(input: &'a [u8], out_len: usize, s: &'a mut Scratch) -> Self {
        for p in s.probs.iter_mut() {
            *p = PROB_INIT;
        }
        let mut d = Decoder {
            input,
            in_pos: 0,
            range: u32::MAX,
            code: 0,
            s,
            dict_pos: 0,
            total_out: 0,
            out_len,
            state: 0,
            reps: [0; 4],
            rem_len: 0,
            err: None,
            done: false,
        };
        // Range-coder init: the first byte is always zero and ignored, then
        // four big-endian bytes prime `code`.
        if d.input.len() < 5 {
            d.fail(Error::Truncated);
            return d;
        }
        d.in_pos = 1;
        for _ in 0..4 {
            let b = d.next_in();
            d.code = (d.code << 8) | b as u32;
        }
        d
    }

    /// The error that ended the decode, if any. Check this after the last
    /// `next_byte()` — a decode that ran clean returns `None`.
    pub fn error(&self) -> Option<Error> {
        self.err
    }

    /// Bytes produced so far.
    pub fn produced(&self) -> usize {
        self.total_out
    }

    fn fail(&mut self, e: Error) {
        if self.err.is_none() {
            self.err = Some(e);
        }
        self.done = true;
    }

    #[inline(always)]
    fn next_in(&mut self) -> u8 {
        match self.input.get(self.in_pos) {
            Some(&b) => {
                self.in_pos += 1;
                b
            }
            None => {
                self.fail(Error::Truncated);
                0
            }
        }
    }

    #[inline(always)]
    fn normalize(&mut self) {
        if self.range < TOP {
            self.range <<= 8;
            let b = self.next_in();
            self.code = (self.code << 8) | b as u32;
        }
    }

    #[inline(always)]
    fn decode_bit(&mut self, i: usize) -> u32 {
        let v = self.s.probs[i] as u32;
        let bound = (self.range >> PROB_BITS) * v;
        let sym;
        if self.code < bound {
            self.range = bound;
            self.s.probs[i] = (v + (((1 << PROB_BITS) - v) >> MOVE_BITS)) as u16;
            sym = 0;
        } else {
            self.range -= bound;
            self.code -= bound;
            self.s.probs[i] = (v - (v >> MOVE_BITS)) as u16;
            sym = 1;
        }
        self.normalize();
        sym
    }

    #[inline(always)]
    fn bittree(&mut self, base: usize, num_bits: u32) -> u32 {
        let mut m = 1u32;
        for _ in 0..num_bits {
            m = (m << 1) + self.decode_bit(base + m as usize);
        }
        m - (1 << num_bits)
    }

    #[inline(always)]
    fn bittree_reverse(&mut self, base: usize, num_bits: u32) -> u32 {
        let mut m = 1u32;
        let mut sym = 0u32;
        for i in 0..num_bits {
            let bit = self.decode_bit(base + m as usize);
            m = (m << 1) + bit;
            sym |= bit << i;
        }
        sym
    }

    fn direct_bits(&mut self, num_bits: u32) -> u32 {
        let mut res = 0u32;
        for _ in 0..num_bits {
            self.range >>= 1;
            self.code = self.code.wrapping_sub(self.range);
            let t = 0u32.wrapping_sub(self.code >> 31);
            self.code = self.code.wrapping_add(self.range & t);
            if self.code == self.range {
                self.fail(Error::Corrupt);
            }
            self.normalize();
            res = (res << 1).wrapping_add(t).wrapping_add(1);
        }
        res
    }

    /// Decode a length symbol. Returns the raw value; callers add
    /// `MATCH_MIN_LEN`. `pb=0` collapses the posState index to 0.
    fn decode_len(&mut self, base: usize) -> u32 {
        if self.decode_bit(base + LEN_CHOICE) == 0 {
            return self.bittree(base + LEN_LOW, 3);
        }
        if self.decode_bit(base + LEN_CHOICE2) == 0 {
            return 8 + self.bittree(base + LEN_MID, 3);
        }
        16 + self.bittree(base + LEN_HIGH, 8)
    }

    fn decode_distance(&mut self, len: u32) -> u32 {
        let len_state = if len < NUM_LEN_TO_POS_STATES {
            len
        } else {
            NUM_LEN_TO_POS_STATES - 1
        };
        let pos_slot = self.bittree(OFF_POS_SLOT + (len_state as usize * 64), 6);
        if pos_slot < 4 {
            return pos_slot;
        }
        let num_direct = (pos_slot >> 1) - 1;
        let mut dist = (2 | (pos_slot & 1)) << num_direct;
        if pos_slot < END_POS_MODEL_INDEX {
            // `OFF_SPEC_POS + dist - pos_slot` mirrors LzmaSpec's
            // `PosDecoders + dist - posSlot`: the sub-trees for the
            // different slots overlap in one 115-entry array.
            let base = OFF_SPEC_POS + dist as usize - pos_slot as usize;
            dist += self.bittree_reverse(base, num_direct);
        } else {
            dist += self.direct_bits(num_direct - NUM_ALIGN_BITS) << NUM_ALIGN_BITS;
            dist += self.bittree_reverse(OFF_ALIGN, NUM_ALIGN_BITS);
        }
        dist
    }

    #[inline(always)]
    fn byte_at_dist(&self, dist: u32) -> u8 {
        self.s.dict[(self.dict_pos.wrapping_sub(dist as usize + 1)) & DICT_MASK]
    }

    #[inline(always)]
    fn put_byte(&mut self, b: u8) -> u8 {
        self.s.dict[self.dict_pos] = b;
        self.dict_pos = (self.dict_pos + 1) & DICT_MASK;
        self.total_out += 1;
        b
    }

    fn decode_literal(&mut self) -> u8 {
        let mut symbol = 1u32;
        // States 7..11 mean the previous symbol was a match, and the literal
        // is then coded against the byte at the last used distance.
        if self.state >= 7 {
            let mut match_byte = self.byte_at_dist(self.reps[0]);
            loop {
                let match_bit = ((match_byte >> 7) & 1) as u32;
                match_byte <<= 1;
                let bit = self.decode_bit(
                    OFF_LITERAL + (((1 + match_bit) << 8) + symbol) as usize,
                );
                symbol = (symbol << 1) | bit;
                if match_bit != bit || symbol >= 0x100 {
                    break;
                }
            }
        }
        while symbol < 0x100 {
            let bit = self.decode_bit(OFF_LITERAL + symbol as usize);
            symbol = (symbol << 1) | bit;
        }
        self.put_byte((symbol - 0x100) as u8)
    }

    /// Produce the next decompressed byte, or `None` once `out_len` bytes
    /// have been produced or the stream failed.
    pub fn next_byte(&mut self) -> Option<u8> {
        if self.done || self.total_out >= self.out_len {
            return None;
        }

        // Still paying out a match from a previous call.
        if self.rem_len > 0 {
            self.rem_len -= 1;
            let b = self.byte_at_dist(self.reps[0]);
            return Some(self.put_byte(b));
        }

        let state = self.state as usize;
        if self.decode_bit(OFF_IS_MATCH + state) == 0 {
            let b = self.decode_literal();
            self.state = if self.state < 4 {
                0
            } else if self.state < 10 {
                self.state - 3
            } else {
                self.state - 6
            };
            return if self.done { None } else { Some(b) };
        }

        let len;
        if self.decode_bit(OFF_IS_REP + state) != 0 {
            // A repeated distance. Nothing has been emitted yet, so there is
            // no previous distance to repeat.
            if self.total_out == 0 {
                self.fail(Error::Corrupt);
                return None;
            }
            if self.decode_bit(OFF_IS_REP_G0 + state) == 0 {
                if self.decode_bit(OFF_IS_REP0_LONG + state) == 0 {
                    // Short rep: exactly one byte at reps[0].
                    self.state = if self.state < 7 { 9 } else { 11 };
                    let b = self.byte_at_dist(self.reps[0]);
                    return if self.done { None } else { Some(self.put_byte(b)) };
                }
            } else {
                let dist;
                if self.decode_bit(OFF_IS_REP_G1 + state) == 0 {
                    dist = self.reps[1];
                } else {
                    if self.decode_bit(OFF_IS_REP_G2 + state) == 0 {
                        dist = self.reps[2];
                    } else {
                        dist = self.reps[3];
                        self.reps[3] = self.reps[2];
                    }
                    self.reps[2] = self.reps[1];
                }
                self.reps[1] = self.reps[0];
                self.reps[0] = dist;
            }
            len = self.decode_len(OFF_REP_LEN) + MATCH_MIN_LEN;
            self.state = if self.state < 7 { 8 } else { 11 };
        } else {
            self.reps[3] = self.reps[2];
            self.reps[2] = self.reps[1];
            self.reps[1] = self.reps[0];
            let len_raw = self.decode_len(OFF_LEN);
            self.state = if self.state < 7 { 7 } else { 10 };
            let dist = self.decode_distance(len_raw);
            if dist == u32::MAX {
                // End-of-stream marker. Legitimate only once the expected
                // output has been produced, which the caller checks.
                self.done = true;
                return None;
            }
            self.reps[0] = dist;
            // A distance may not reach past what has been emitted, nor
            // outside the window.
            if dist as usize >= DICT_SIZE || dist as usize >= self.total_out {
                self.fail(Error::Corrupt);
                return None;
            }
            len = len_raw + MATCH_MIN_LEN;
        }

        if self.done {
            return None;
        }
        self.rem_len = len - 1;
        let b = self.byte_at_dist(self.reps[0]);
        Some(self.put_byte(b))
    }
}

impl<'a> Iterator for Decoder<'a> {
    type Item = u8;
    #[inline]
    fn next(&mut self) -> Option<u8> {
        self.next_byte()
    }
}
