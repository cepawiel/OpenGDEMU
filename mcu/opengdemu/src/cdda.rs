//! CDDA playback: streams raw audio sectors from the card into the FPGA's
//! serialiser (hw/cdda.py) in real time.
//!
//! Three parties share a 4 KB byte ring in SRAM0:
//!
//!  * `cdda_task` (producer) fills it with whole 512-byte SD blocks read
//!    straight off the card by CMD18 through the track's extent map -- no
//!    filesystem code in the loop. It also drives the play/repeat state.
//!  * the SysTick hook (consumer, interrupt context, once per ms) tops the
//!    FPGA's 768-word FIFO up from the ring with DMAC channel 2. Audio runs
//!    at 176.4 KB/s, so a tick moves ~90 words; the ring holds 23 ms, the
//!    FPGA FIFO another 8.7 ms, comfortably more than the longest stretch
//!    `ide_task` spends inside a blocking SD read.
//!  * `ide_task` starts, pauses and stops playback and reads the position.
//!
//! Positions in the ring are byte counters into a "stream" that is reset
//! for every *run*: a contiguous stretch of one track file. Block reads
//! land at any 512-byte boundary, so a run's first valid byte usually sits
//! partway into its first block (`TAIL` starts there) and its last block is
//! only partly wanted (`END` clips it). Between runs -- the next track of a
//! multi-track play, or the next lap of a repeating one -- the producer
//! waits for the ring to drain, resets the counters and starts over; that
//! costs a couple of milliseconds of silence on top of whatever pregap the
//! disc already has, which a real drive's seek would dwarf.
//!
//! Sector addresses everywhere in here are FADs (LBA + 150), as the host
//! speaks them.

use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use critical_section::Mutex;
use defmt::{info, warn, Format};
use embassy_atsam3::{dmac, hsmci, smc, systick};
use embassy_time::{Duration, Timer};

/// FPGA MCU-window slots (primary device), see hw/cdda.py.
const REG_CDDA_CTRL: usize = 29;
const REG_CDDA_LEVEL: usize = 30;
const REG_CDDA_PUSH: usize = 31;
const CTRL_ENABLE: u16 = 0x01;
const CTRL_CLEAR_UF: u16 = 0x02;
const CTRL_FLUSH: u16 = 0x04;
/// Depth of the FPGA sample FIFO in 16-bit words, minus slack for the
/// serialiser's prefetch and the level read being a cycle stale.
const FPGA_FILL_TARGET: u32 = 768 - 8;

pub const SECTOR_BYTES: u32 = 2352;
const RING_BYTES: usize = 4096;
const BLOCK: u32 = 512;
/// Most blocks fetched per producer round; keeps each blocking read short.
const MAX_FETCH_BLOCKS: u32 = 8;

#[repr(C, align(4))]
struct Ring([u8; RING_BYTES]);
/// On a 2E this moves out of SRAM0 and into the NAND flash controller's SRAM,
/// which nothing else in this firmware uses and which is 4,224 bytes -- just
/// enough. That section is NOLOAD, so `init` has to zero it by hand.
#[cfg_attr(feature = "sam3u2e", link_section = ".nfc")]
static mut RING: Ring = Ring([0; RING_BYTES]);

/// Stream byte counters for the current run (see module docs). HEAD is
/// written by the producer, TAIL by the consumer; END by the producer
/// before it lets the consumer loose on a run.
static HEAD: AtomicU32 = AtomicU32::new(0);
static TAIL: AtomicU32 = AtomicU32::new(0);
static END: AtomicU32 = AtomicU32::new(0);
/// Consumer on/off switch; the tick hook does nothing while this is clear.
static FEED: AtomicBool = AtomicBool::new(false);
/// Turn the FPGA output on as soon as the consumer has landed a push in
/// its FIFO. Enabling any earlier plays up to a tick of silence (measured
/// 16-24 frames per start) before the first samples arrive.
static ARM: AtomicBool = AtomicBool::new(false);
/// Bytes handed to the DMAC on the previous tick and not yet credited to
/// TAIL. Touched only from the tick hook.
static mut PENDING: u32 = 0;

#[derive(Clone, Copy, PartialEq, Eq, Format)]
pub enum Status {
    /// Nothing queued; also what a CD_SEEK leaves behind.
    Standby,
    Playing,
    Paused,
    /// Reached the end without a repeat left. Reports as PAUSE with the
    /// position held at the last sector.
    Finished,
    /// A read failed mid-play.
    Error,
}

#[derive(Clone, Copy, Format)]
pub struct State {
    pub status: Status,
    pub start_fad: u32,
    pub end_fad: u32,
    /// Repeats still owed after the current lap; 15 = forever.
    pub repeat: u8,
    /// Where the next run starts (producer bookkeeping; a resumed pause
    /// carries on from the run it was in, this only matters between runs).
    next_fad: u32,
    /// First FAD of the run currently in the ring and the stream position
    /// of its first byte, for turning TAIL into a playing position.
    run_fad: u32,
    run_head_off: u32,
    /// Incremented by every `play`/`stop` so the producer notices a restart
    /// even if the fields happen to match.
    seq: u32,
}

const IDLE: State = State {
    status: Status::Standby,
    start_fad: 0,
    end_fad: 0,
    repeat: 0,
    next_fad: 0,
    run_fad: 0,
    run_head_off: 0,
    seq: 0,
};

static STATE: Mutex<RefCell<State>> = Mutex::new(RefCell::new(IDLE));

fn with_state<R>(f: impl FnOnce(&mut State) -> R) -> R {
    critical_section::with(|cs| f(&mut STATE.borrow(cs).borrow_mut()))
}

fn reg_write(slot: usize, v: u16) {
    unsafe { core::ptr::write_volatile(smc::fpga_window().add(slot), v) }
}

fn reg_read(slot: usize) -> u16 {
    unsafe { core::ptr::read_volatile(smc::fpga_window().add(slot)) }
}

/// Install the tick hook. Call once, after `dmac::init()`.
pub fn init() {
    // `.nfc` is NOLOAD: nothing has zeroed the ring at startup. Harmless for
    // the audio itself (the producer always fills ahead of the consumer) but
    // not worth leaving to that argument.
    #[cfg(feature = "sam3u2e")]
    unsafe {
        core::ptr::write_bytes(core::ptr::addr_of_mut!(RING.0).cast::<u8>(), 0, RING_BYTES);
    }
    systick::set_tick_hook(on_tick);
    reg_write(REG_CDDA_CTRL, CTRL_CLEAR_UF | CTRL_FLUSH);
}

/// Stop the consumer and wait for any push it has in flight, so the ring
/// and the FPGA FIFO may be reset underneath it.
fn quiesce_feed() {
    FEED.store(false, Ordering::Release);
    ARM.store(false, Ordering::Release);
    dmac::wait_done_ch2();
    // The hook only touches PENDING while FEED is set, and it either ran
    // to completion before the store above or bailed on seeing it, so this
    // cannot race. Whatever it had queued was pushed but not credited;
    // dropping it makes a resume replay those few ms, which is what we want
    // after the FIFO has been flushed.
    unsafe { PENDING = 0 };
}

/// Log and clear the FPGA's silent-frame counter, if it counted any.
fn report_underflow(when: &str) {
    let st = reg_read(REG_CDDA_CTRL);
    let frames = st >> 8;
    if st & 0x02 != 0 {
        warn!("cdda: underflow, {} silent frames (saturating) by {}", frames, when);
        reg_write(REG_CDDA_CTRL, (st & CTRL_ENABLE) | CTRL_CLEAR_UF);
    }
}

/// Start playing `[start_fad, end_fad)`, `repeat` extra times (15 = loop
/// forever). Fails if the start sector is not on a mapped track.
pub fn play(start_fad: u32, end_fad: u32, repeat: u8) -> Result<(), ()> {
    let gdi = crate::ide::gdi_ref().ok_or(())?;
    let track = gdi.track_for_lba(start_fad.saturating_sub(150)).ok_or(())?;
    if track.ext_len == 0 || end_fad <= start_fad {
        return Err(());
    }
    quiesce_feed();
    // Flush whatever the previous play left queued so the first frames of
    // this one are its own. Output stays off until the producer has the
    // first block in, else the wait would count as an underflow.
    reg_write(REG_CDDA_CTRL, CTRL_CLEAR_UF | CTRL_FLUSH);
    END.store(0, Ordering::Release);
    HEAD.store(0, Ordering::Release);
    TAIL.store(0, Ordering::Release);
    with_state(|s| {
        *s = State {
            status: Status::Playing,
            start_fad,
            end_fad,
            repeat,
            next_fad: start_fad,
            run_fad: start_fad,
            run_head_off: 0,
            seq: s.seq.wrapping_add(1),
        };
    });
    info!("cdda: play FAD {}..{} repeat={} (track {})", start_fad, end_fad, repeat, track.number);
    Ok(())
}

/// Resume a paused play from where it stopped. No-op unless paused.
pub fn resume() -> bool {
    let resumed = with_state(|s| {
        if s.status == Status::Paused {
            s.status = Status::Playing;
            true
        } else {
            false
        }
    });
    if resumed {
        reg_write(REG_CDDA_CTRL, CTRL_CLEAR_UF);
        ARM.store(true, Ordering::Release);
        FEED.store(true, Ordering::Release);
        info!("cdda: resume");
    }
    resumed
}

/// Freeze playback, keeping the position.
pub fn pause() {
    let was_playing = with_state(|s| {
        if s.status == Status::Playing {
            s.status = Status::Paused;
            true
        } else {
            false
        }
    });
    if was_playing {
        quiesce_feed();
        report_underflow("pause");
        reg_write(REG_CDDA_CTRL, CTRL_FLUSH);
        info!("cdda: pause at FAD {}", snapshot().fad);
    }
}

/// Stop playback and drop the queue (CD_SEEK, eject, reset).
pub fn stop() {
    let was_active = with_state(|s| {
        let active = s.status != Status::Standby;
        s.status = Status::Standby;
        s.seq = s.seq.wrapping_add(1);
        active
    });
    if was_active {
        quiesce_feed();
        report_underflow("stop");
        reg_write(REG_CDDA_CTRL, CTRL_CLEAR_UF | CTRL_FLUSH);
        info!("cdda: stop");
    }
}

pub struct Snapshot {
    pub status: Status,
    pub repeat: u8,
    /// Sector currently going out (best effort: the FPGA FIFO adds < 9 ms).
    pub fad: u32,
}

pub fn snapshot() -> Snapshot {
    let s = with_state(|s| *s);
    let tail = TAIL.load(Ordering::Relaxed);
    let fad = match s.status {
        Status::Standby => 0,
        _ => {
            let played = tail.saturating_sub(s.run_head_off) / SECTOR_BYTES;
            (s.run_fad + played).min(s.end_fad.saturating_sub(1))
        }
    };
    Snapshot { status: s.status, repeat: s.repeat, fad }
}

/// SysTick hook: credit the previous push, then queue the next one.
fn on_tick() {
    if !FEED.load(Ordering::Acquire) {
        return;
    }
    // SAFETY: PENDING is only touched here, in one interrupt handler.
    let pending = unsafe { PENDING };
    if pending != 0 {
        if dmac::busy_ch2() {
            return;
        }
        TAIL.fetch_add(pending, Ordering::Release);
        unsafe { PENDING = 0 };
        // That push is in the FIFO now: safe to start draining it.
        if ARM.swap(false, Ordering::AcqRel) {
            reg_write(REG_CDDA_CTRL, CTRL_ENABLE);
        }
    }
    let tail = TAIL.load(Ordering::Relaxed);
    let valid = HEAD.load(Ordering::Acquire).min(END.load(Ordering::Acquire));
    let avail = valid.saturating_sub(tail);
    if avail < 2 {
        return;
    }
    let level = reg_read(REG_CDDA_LEVEL) as u32;
    let room = FPGA_FILL_TARGET.saturating_sub(level) * 2;
    let idx = (tail as usize) % RING_BYTES;
    let n = avail.min(room).min((RING_BYTES - idx) as u32) & !1;
    if n == 0 {
        return;
    }
    unsafe {
        let src = core::ptr::addr_of!(RING.0).cast::<u8>().add(idx).cast::<u16>();
        dmac::push_halfwords_ch2(src, smc::fpga_window().add(REG_CDDA_PUSH), n / 2);
        PENDING = n;
    }
}

/// Producer-side description of the run being fetched.
struct Run {
    track: &'static crate::gdi::Track,
    /// FAD the run ends at (exclusive).
    end_fad: u32,
    /// File byte offset of the next block to fetch.
    fetch_byte: u32,
    blocks_left: u32,
}

/// Prepare the ring for a run starting at `fad`. Returns `None` if the
/// sector is not on a mapped track.
fn start_run(gdi: &'static crate::gdi::Gdi, fad: u32, end_fad: u32) -> Option<Run> {
    let lba = fad.checked_sub(150)?;
    let track = gdi.track_for_lba(lba)?;
    let track_end_fad = track.start_lba + track.sector_count + 150;
    let run_end = end_fad.min(track_end_fad);
    if run_end <= fad {
        return None;
    }
    let byte0 = track.byte_offset_of(lba)?;
    let head_off = byte0 % BLOCK;
    let bytes = (run_end - fad) * SECTOR_BYTES.min(track.sector_size as u32);
    let stream_end = head_off + bytes;
    // Order matters against the tick hook: nothing is valid until END is
    // raised again at the bottom.
    END.store(0, Ordering::Release);
    HEAD.store(0, Ordering::Release);
    TAIL.store(head_off, Ordering::Release);
    with_state(|s| {
        s.run_fad = fad;
        s.run_head_off = head_off;
    });
    END.store(stream_end & !1, Ordering::Release);
    FEED.store(true, Ordering::Release);
    Some(Run {
        track,
        end_fad: run_end,
        fetch_byte: byte0 - head_off,
        blocks_left: (stream_end + BLOCK - 1) / BLOCK,
    })
}

fn finish(status: Status) {
    quiesce_feed();
    report_underflow("end of play");
    with_state(|s| s.status = status);
    // Leave the output enabled: the FIFO drains its last frames on its own.
    let dev = crate::ide::IdeDevice::primary();
    dev.set_secnr(crate::ide::secnr(crate::ide::GD_PAUSE));
    info!("cdda: {:?}", status);
}

#[embassy_executor::task]
pub async fn cdda_task() {
    let mut seq = 0u32;
    let mut run: Option<Run> = None;
    loop {
        let s = with_state(|s| *s);
        if s.seq != seq {
            // A new play or a stop: whatever run was in progress is void.
            seq = s.seq;
            run = None;
        }
        if s.status != Status::Playing {
            Timer::after(Duration::from_millis(2)).await;
            continue;
        }
        let Some(gdi) = crate::ide::gdi_ref() else {
            finish(Status::Error);
            continue;
        };

        let Some(r) = run.as_mut() else {
            // Between runs: pick the next one, or wrap / finish the lap.
            let next = s.next_fad;
            if next >= s.end_fad {
                let again = with_state(|s| {
                    if s.repeat == 0 {
                        false
                    } else {
                        if s.repeat != 15 {
                            s.repeat -= 1;
                        }
                        s.next_fad = s.start_fad;
                        true
                    }
                });
                if !again {
                    finish(Status::Finished);
                }
                continue;
            }
            match start_run(gdi, next, s.end_fad) {
                Some(r) => {
                    info!("cdda: run FAD {}..{} track {}", next, r.end_fad, r.track.number);
                    run = Some(r);
                }
                None => {
                    warn!("cdda: FAD {} not playable", next);
                    finish(Status::Error);
                }
            }
            continue;
        };

        if r.blocks_left == 0 {
            // Fetched everything; hand over once the consumer has too.
            if TAIL.load(Ordering::Acquire) >= END.load(Ordering::Acquire) {
                quiesce_feed();
                let end = r.end_fad;
                with_state(|s| s.next_fad = end);
                run = None;
            } else {
                Timer::after(Duration::from_millis(1)).await;
            }
            continue;
        }

        let head = HEAD.load(Ordering::Relaxed);
        let used = head.wrapping_sub(TAIL.load(Ordering::Acquire)) as i32;
        let free_blocks = (RING_BYTES as i32 - used.max(0)) as u32 / BLOCK;
        if free_blocks == 0 {
            Timer::after(Duration::from_millis(1)).await;
            continue;
        }
        let idx = (head as usize) % RING_BYTES;
        let Some((block, _, contig)) = r.track.locate_byte(r.fetch_byte) else {
            warn!("cdda: file offset {} unmapped", r.fetch_byte);
            finish(Status::Error);
            continue;
        };
        let n = free_blocks
            .min((RING_BYTES - idx) as u32 / BLOCK)
            .min(r.blocks_left)
            .min(contig)
            .min(MAX_FETCH_BLOCKS);
        let dst = unsafe {
            core::slice::from_raw_parts_mut(
                core::ptr::addr_of_mut!(RING.0).cast::<u8>().add(idx),
                (n * BLOCK) as usize,
            )
        };
        if let Err(e) = unsafe { hsmci::read_blocks_dma(block, dst) } {
            warn!("cdda: SD read {} blocks at {} failed: {:?}", n, block, e);
            finish(Status::Error);
            continue;
        }
        HEAD.fetch_add(n * BLOCK, Ordering::Release);
        if head == 0 {
            // First block of a play: output comes on once it's in the FIFO.
            ARM.store(true, Ordering::Release);
        }
        r.fetch_byte += n * BLOCK;
        r.blocks_left -= n;
    }
}
