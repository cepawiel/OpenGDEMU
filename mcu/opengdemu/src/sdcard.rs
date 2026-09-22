//! SD card init state machine and partition-table inspection.
//!
//! Polled, 1-bit, no DMA, intentionally minimal. Drives an SD card from
//! IDLE all the way to TRANSFER state, reads the CSD to compute the card
//! capacity, and dumps the first sector or two so we can see what's
//! actually on the card.

use defmt::{debug, info, warn};

use embassy_atsam3::hsmci::{self, CmdError, Response};
use embassy_time::{Duration, Timer};

const ACMD: u8 = 0xC0; // marker — not really an opcode, just used here for logging
                       // clarity (real ACMDs use the same numeric encoding as their
                       // CMD counterparts; the difference is the preceding CMD55).

/// Capacity class returned by ACMD41.
#[derive(Debug, Copy, Clone, PartialEq, Eq, defmt::Format)]
pub enum CardCapacity {
    /// SDSC v1 / v2: byte-addressed, capacity ≤ 2 GB (≤ 4 GB for some).
    StandardCapacity,
    /// SDHC / SDXC: block-addressed, capacity 4 GB – 2 TB.
    HighCapacity,
}

#[derive(Debug, Copy, Clone, defmt::Format)]
pub struct CardInfo {
    pub capacity: CardCapacity,
    pub rca: u16,
    pub csd: [u32; 4],
    pub size_blocks: u32,
}

#[derive(Debug, Copy, Clone, defmt::Format)]
pub enum InitError {
    Cmd { cmd: u8, err: CmdError },
    NoOcrReady,
}

fn cmd(cmd: u8, arg: u32, resp: Response, open_drain: bool, ignore_crc: bool) -> Result<u32, InitError> {
    unsafe { hsmci::send_command(cmd, arg, resp, open_drain, ignore_crc) }
        .map_err(|e| InitError::Cmd { cmd, err: e })
}

/// Like `cmd`, but retries on `ResponseTimeout` up to `attempts` times with
/// a small settle between tries. Used during init when this board's bus
/// timing is intermittent — it's normal here for an early CMD to time out
/// and succeed on retry.
async fn cmd_retry(
    n: u8,
    arg: u32,
    resp: Response,
    open_drain: bool,
    ignore_crc: bool,
    attempts: u32,
) -> Result<u32, InitError> {
    for i in 0..attempts {
        match cmd(n, arg, resp, open_drain, ignore_crc) {
            Ok(v) => return Ok(v),
            Err(InitError::Cmd { err: CmdError::ResponseTimeout, .. }) if i + 1 < attempts => {
                debug!("CMD{} retry {}", n, i + 1);
                Timer::after(Duration::from_millis(2)).await;
                continue;
            }
            Err(e) => return Err(e),
        }
    }
    unreachable!()
}

/// Drive a freshly powered SD card from POWER_ON through to TRANSFER state.
/// Caller is responsible for the HSMCI peripheral being initialized; this
/// function handles the 74 init clocks, CMD0, CMD8, ACMD41, CMD2/3/9/7/16.
pub async fn init() -> Result<CardInfo, InitError> {
    // Re-send 74+ init clocks every entry so a partially-initialized card
    // (e.g. from a prior failed attempt) gets a fresh start.
    unsafe { hsmci::send_init_clocks(); }

    // CMD0: GO_IDLE_STATE. No response. open-drain because no card has an
    // RCA yet.
    let _ = cmd(0, 0, Response::None, true, false)?;

    // CMD8: SEND_IF_COND with check pattern 0xAA + 2.7-3.6V (0x1) =
    // 0x0000_01AA. open-drain.
    let _ = cmd_retry(8, 0x0000_01AA, Response::Short, true, false, 5).await?;

    // ACMD41: SD_SEND_OP_COND. Skip the probe-with-arg=0 pattern — some
    // cards interpret it as "host supports no voltage" and go inactive.
    // Send the real voltage mask + HCS directly; up to ~2 s for the busy
    // bit to flip on slow cards.
    let init_arg = 0x4000_0000 | 0x00FF_8000;

    let mut ocr = 0u32;
    let mut ready = false;
    for _ in 0..400 {
        // CMD55 → ACMD41 must be sent as a pair every iteration. If CMD41
        // times out, retrying CMD41 alone won't work — APP_CMD only
        // persists for one command. Treat CMD41 timeout as "still busy"
        // and let the outer loop send another CMD55+CMD41 pair.
        let cmd55 = cmd_retry(55, 0, Response::Short, true, false, 5).await?;
        let _ = cmd55;
        match cmd(41, init_arg, Response::Short, true, true) {
            Ok(v) => ocr = v,
            Err(InitError::Cmd { err: CmdError::ResponseTimeout, .. }) => {
                Timer::after(Duration::from_millis(5)).await;
                continue;
            }
            Err(e) => return Err(e),
        }
        if ocr & (1 << 31) != 0 {
            ready = true;
            break;
        }
        Timer::after(Duration::from_millis(5)).await;
    }
    if !ready {
        warn!("ACMD41 stuck on OCR={:#010x} (busy bit never set)", ocr);
        return Err(InitError::NoOcrReady);
    }
    debug!("ACMD41 ready, OCR={:#010x}", ocr);

    let capacity = if ocr & (1 << 30) != 0 {
        CardCapacity::HighCapacity
    } else {
        CardCapacity::StandardCapacity
    };

    // CMD2: ALL_SEND_CID — long R2. Still in identification phase, so
    // open-drain.
    let _ = cmd_retry(2, 0, Response::Long, true, false, 5).await?;
    let cid = unsafe { hsmci::long_response() };
    debug!("CID = {:08x} {:08x} {:08x} {:08x}", cid[0], cid[1], cid[2], cid[3]);

    Timer::after(Duration::from_millis(10)).await;

    // CMD3: SEND_RELATIVE_ADDR. Card returns a published RCA in the upper
    // 16 bits of the R6 response. Push-pull — though ASF's table flags
    // CMD3 as open-drain, on this board CMD3 only succeeds in push-pull.
    let r6 = cmd_retry(3, 0, Response::Short, false, false, 5).await?;
    let rca = ((r6 >> 16) & 0xFFFF) as u16;
    debug!("CMD3 RCA = {:#06x}", rca);
    Timer::after(Duration::from_millis(10)).await;

    // CMD9: SEND_CSD — long R2, CSD is 128 bits across rspr[0..4].
    let _ = cmd_retry(9, (rca as u32) << 16, Response::Long, false, false, 5).await?;
    let csd = unsafe { hsmci::long_response() };
    debug!("CSD = {:08x} {:08x} {:08x} {:08x}", csd[0], csd[1], csd[2], csd[3]);
    Timer::after(Duration::from_millis(10)).await;

    // CMD7: SELECT_CARD → puts the addressed card in TRANSFER state.
    // Response is R1b (busy).
    let _ = cmd_retry(7, (rca as u32) << 16, Response::Short, false, false, 5).await?;
    Timer::after(Duration::from_millis(10)).await;

    // SD spec: cards default to 512-byte block length. CMD16 enforces it
    // for SDSC; SDHC/SDXC silently ignore it but the call is harmless.
    let _ = cmd_retry(16, 512, Response::Short, false, false, 5).await?;

    // Leave identification mode behind: widen to 4 data lines and raise the
    // clock off the 400 kHz init rate. Until this runs the card is doing
    // ~46 KB/s, which is ~250x too slow to feed a G1 DMA burst and is why
    // CD_READ has to pre-cache everything up front.
    //
    // Order matters: ACMD6 switches the *card*, then the host controller
    // follows. If the host widens first, the card is still driving one line
    // and every data block comes back as a CRC error.
    match set_bus_width_4bit(rca).await {
        Ok(()) => {
            unsafe { hsmci::set_bus_width_4() };
            // 24 MHz: the SD default-speed ceiling is 25 MHz, and we have not
            // done the CMD6 high-speed switch that would allow 50.
            let hz = unsafe { hsmci::set_clock_hz(24_000_000) };
            info!("SD link: 4-bit @ {} Hz", hz);
        }
        Err(e) => {
            // Not fatal — a card that refuses ACMD6 still works at 1-bit,
            // just slowly. Better to boot degraded than not at all.
            warn!("ACMD6 (4-bit) failed, staying 1-bit @ 400 kHz: {:?}", e);
        }
    }

    let size_blocks = csd_size_blocks(&csd, capacity);

    Ok(CardInfo { capacity, rca, csd, size_blocks })
}

/// ACMD6 SET_BUS_WIDTH with arg=2 (4-bit). Must be preceded by CMD55 carrying
/// the card's RCA, and the card must already be selected (CMD7 / TRANSFER
/// state) for the app-command pair to be accepted.
async fn set_bus_width_4bit(rca: u16) -> Result<(), InitError> {
    let _ = cmd_retry(55, (rca as u32) << 16, Response::Short, false, false, 5).await?;
    let _ = cmd_retry(6, 2, Response::Short, false, false, 5).await?;
    Timer::after(Duration::from_millis(2)).await;
    Ok(())
}

/// Measure raw sequential `read_block` throughput and report it over UART.
///
/// The transfer size escalates until one run takes long enough to time
/// accurately against embassy's 1 kHz tick, so the same routine stays
/// meaningful both at the 400 kHz identification clock (~50 KB/s) and on a
/// 4-bit high-speed link (several MB/s). Reads from deep in the card to
/// avoid measuring anything but the link itself.
#[cfg(feature = "sd-bench")]
pub async fn bench_read(start_block: u32) {
    bench_one("pio", start_block, false).await;
    bench_one("dma", start_block, true).await;
    // 4 and 8 blocks: a 2352-byte GD-ROM sector is ~5 blocks, so this is the
    // run length the streaming path would actually use. Bigger chunks would
    // score better but need RAM we do not have.
    bench_multi(start_block, 4).await;
    bench_multi(start_block, 8).await;
    verify_multi(start_block).await;
}

/// Check CMD18 against the single-block path byte for byte. A fast read that
/// returns the wrong data is worse than a slow one, and nothing else in the
/// benchmark would notice.
#[cfg(feature = "sd-bench")]
async fn verify_multi(start_block: u32) {
    const N: usize = SCRATCH_BLOCKS;
    let multi = unsafe { &mut SCRATCH[..] };

    if unsafe { hsmci::read_blocks_dma(start_block, multi) }.is_err() {
        info!("SD verify: CMD18 read failed");
        return;
    }
    let mut single = [0u8; 512];
    for i in 0..N {
        if unsafe { hsmci::read_block(start_block + i as u32, &mut single) }.is_err() {
            info!("SD verify: single-block read failed");
            return;
        }
        let lhs = &multi[i * 512..(i + 1) * 512];
        if lhs != &single[..] {
            let bad = (0..512).find(|&b| lhs[b] != single[b]).unwrap_or(0);
            info!(
                "SD verify: MISMATCH block {} byte {} (cmd18={:#04x} cmd17={:#04x})",
                start_block + i as u32, bad, lhs[bad], single[bad]
            );
            return;
        }
    }
    info!("SD verify: CMD18 == CMD17 over {} blocks, OK", N);
}

/// Longest CMD18 run the benchmark exercises, in 512-byte blocks.
#[cfg(feature = "sd-bench")]
const SCRATCH_BLOCKS: usize = 8;

/// One scratch buffer shared by every multi-block routine here. RAM is tight
/// (32 KB total, ~14 KB of it already CD_READ_BUF) and the stack lives in
/// what is left, so a second buffer this size overflows it — which shows up
/// as a HardFault/BusFault on instruction prefetch, not as a clean error.
#[cfg(feature = "sd-bench")]
static mut SCRATCH: [u8; SCRATCH_BLOCKS * 512] = [0; SCRATCH_BLOCKS * 512];

/// Same measurement for CMD18 multi-block reads, at a given run length.
#[cfg(feature = "sd-bench")]
async fn bench_multi(start_block: u32, chunk_blocks: usize) {
    let buf = unsafe { &mut SCRATCH[..chunk_blocks * 512] };

    for &blocks in [256u32, 4096].iter() {
        let chunks = blocks / chunk_blocks as u32;
        let t0 = Instant::now();
        for c in 0..chunks {
            let lba = start_block + c * chunk_blocks as u32;
            if unsafe { hsmci::read_blocks_dma(lba, buf) }.is_err() {
                info!(
                    "SD bench[m{}]: read failed at block {}", chunk_blocks, lba
                );
                return;
            }
        }
        let us = t0.elapsed().as_micros().max(1);
        let bytes = chunks as u64 * chunk_blocks as u64 * 512;
        info!(
            "SD bench[m{}]: {} blocks ({} KB) in {} us -> {} KB/s",
            chunk_blocks, chunks * chunk_blocks as u32, bytes / 1024, us,
            (bytes * 1_000_000) / (us * 1024)
        );
        if us >= 300_000 {
            break;
        }
    }
}

#[cfg(feature = "sd-bench")]
async fn bench_one(what: &str, start_block: u32, use_dma: bool) {
    let mut buf = [0u8; 512];
    for &blocks in [32u32, 256, 4096].iter() {
        let t0 = Instant::now();
        for i in 0..blocks {
            let r = if use_dma {
                unsafe { hsmci::read_block_dma(start_block + i, &mut buf) }
            } else {
                unsafe { hsmci::read_block(start_block + i, &mut buf) }
            };
            if r.is_err() {
                info!(
                    "SD bench[{}]: read failed at block {}", what, start_block + i
                );
                return;
            }
        }
        let us = t0.elapsed().as_micros().max(1);
        let bytes = blocks as u64 * 512;
        let kbps = (bytes * 1_000_000) / (us * 1024);
        info!(
            "SD bench[{}]: {} blocks ({} KB) in {} us -> {} KB/s",
            what, blocks, bytes / 1024, us, kbps
        );
        // Long enough for the 1 ms tick to be accurate; no need to escalate.
        if us >= 300_000 {
            break;
        }
    }
}

/// Decode the card capacity (in 512-byte blocks) from the CSD.
fn csd_size_blocks(csd: &[u32; 4], capacity: CardCapacity) -> u32 {
    // The 128-bit CSD is laid out MSB-first: bit 127 is the top of csd[0],
    // bit 0 is the bottom of csd[3]. CSD_STRUCTURE = csd[0][31:30].
    match capacity {
        CardCapacity::HighCapacity => {
            // CSD v2 (SDHC/SDXC). C_SIZE = bits [69:48] (22 bits).
            //   csd[1][5:0]   = C_SIZE[21:16]
            //   csd[2][31:16] = C_SIZE[15:0]
            let c_size_hi = csd[1] & 0x3F;
            let c_size_lo = (csd[2] >> 16) & 0xFFFF;
            let c_size = (c_size_hi << 16) | c_size_lo;
            // Capacity = (C_SIZE + 1) * 512 KB = (C_SIZE + 1) * 1024 blocks.
            (c_size + 1) * 1024
        }
        CardCapacity::StandardCapacity => {
            // CSD v1 (SDSC). C_SIZE = bits [73:62] (12 bits), C_SIZE_MULT
            // = bits [49:47] (3 bits), READ_BL_LEN = bits [83:80] (4 bits).
            //   csd[1][9:0]   = C_SIZE[11:2]
            //   csd[2][31:30] = C_SIZE[1:0]
            //   csd[2][9:7]   = C_SIZE_MULT
            //   csd[1][19:16] = READ_BL_LEN
            let c_size_hi = csd[1] & 0x3FF;
            let c_size_lo = (csd[2] >> 30) & 0x3;
            let c_size = (c_size_hi << 2) | c_size_lo;
            let c_size_mult = (csd[2] >> 7) & 0x7;
            let read_bl_len = (csd[1] >> 16) & 0xF;
            let mult = 1u32 << (c_size_mult + 2);
            let block_len = 1u32 << read_bl_len;
            // Capacity in bytes = (c_size + 1) * mult * block_len.
            // Convert to 512-byte blocks.
            (c_size + 1) * mult * (block_len / 512)
        }
    }
}

/// Pretty-print a sector to defmt logs as 32 lines × 16 bytes hex + ascii.
// Bring-up helpers: no callers today, kept because the next card that
// refuses to mount is the reason they exist.
#[allow(dead_code)]
pub fn dump_sector(label: &str, sector: &[u8; 512]) {
    info!("--- {} ---", label);
    for row in 0..32 {
        let off = row * 16;
        let line = &sector[off..off + 16];
        info!(
            "{:#06x}  {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x}  {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x}",
            off,
            line[0], line[1], line[2], line[3], line[4], line[5], line[6], line[7],
            line[8], line[9], line[10], line[11], line[12], line[13], line[14], line[15],
        );
    }
}

/// Inspect sector 0 (LBA 0): if it has the MBR boot signature `0x55AA` at
/// offset 510, walk the four primary partition entries. If the first one
/// has type 0xEE (GPT protective MBR), the card is GPT-formatted and the
/// real partition table starts at LBA 1.
///
/// Returns true if the card looks like a GPT disk (so caller can read the
/// GPT header + entries on the next call).
// Bring-up helpers: no callers today, kept because the next card that
// refuses to mount is the reason they exist.
#[allow(dead_code)]
pub fn parse_mbr(sector: &[u8; 512]) -> bool {
    if sector[510] != 0x55 || sector[511] != 0xAA {
        warn!("LBA 0 has no MBR signature ({:02x} {:02x}, expected 55 AA)",
              sector[510], sector[511]);
        return false;
    }
    info!("MBR signature OK (55 AA)");
    let mut is_gpt = false;
    for (i, off) in [0x1BE, 0x1CE, 0x1DE, 0x1EE].iter().enumerate() {
        let entry = &sector[*off..*off + 16];
        let ptype = entry[4];
        let lba_start = u32::from_le_bytes([entry[8], entry[9], entry[10], entry[11]]);
        let lba_count = u32::from_le_bytes([entry[12], entry[13], entry[14], entry[15]]);
        if ptype == 0 && lba_count == 0 {
            info!("  partition {}: empty", i);
        } else {
            info!(
                "  partition {}: type=0x{:02x} start_lba={} count={} (~{} MB)",
                i,
                ptype,
                lba_start,
                lba_count,
                (lba_count as u64 * 512 / 1_048_576) as u32,
            );
            if ptype == 0xEE {
                is_gpt = true;
            }
        }
    }
    if is_gpt {
        info!("MBR partition 0 is type 0xEE — GPT-protective MBR. Real table at LBA 1.");
    }
    is_gpt
}

/// Decode the GPT header at LBA 1 and walk a few entries from the partition
/// entry array (typically at LBA 2). Caller passes both 512-byte buffers.
// Bring-up helpers: no callers today, kept because the next card that
// refuses to mount is the reason they exist.
#[allow(dead_code)]
pub fn parse_gpt(header: &[u8; 512], first_entries: &[u8; 512]) {
    let sig = &header[0..8];
    if sig != b"EFI PART" {
        warn!("GPT signature mismatch at LBA 1: {:?}", sig);
        return;
    }
    let revision = u32::from_le_bytes([header[8], header[9], header[10], header[11]]);
    let first_usable = u64::from_le_bytes([
        header[40], header[41], header[42], header[43],
        header[44], header[45], header[46], header[47],
    ]);
    let last_usable = u64::from_le_bytes([
        header[48], header[49], header[50], header[51],
        header[52], header[53], header[54], header[55],
    ]);
    let entries_lba = u64::from_le_bytes([
        header[72], header[73], header[74], header[75],
        header[76], header[77], header[78], header[79],
    ]);
    let n_entries = u32::from_le_bytes([header[80], header[81], header[82], header[83]]);
    let entry_size = u32::from_le_bytes([header[84], header[85], header[86], header[87]]);

    info!("GPT header: rev={:#010x}, entries_lba={}, n_entries={}, entry_size={}",
          revision, entries_lba, n_entries, entry_size);
    info!("GPT usable LBAs: {} .. {} ({} sectors)",
          first_usable, last_usable, last_usable - first_usable + 1);

    // Walk as many entries as fit in the 512-byte buffer (typically 4 at
    // 128 bytes each).
    if entry_size != 128 {
        warn!("GPT entry size {} != 128 — walker assumes 128-byte entries", entry_size);
        return;
    }
    for i in 0..(512 / 128) {
        let off = i * 128;
        let entry = &first_entries[off..off + 128];
        let type_guid = &entry[0..16];
        // A zero type GUID means "unused entry".
        if type_guid.iter().all(|&b| b == 0) {
            continue;
        }
        let first_lba = u64::from_le_bytes([
            entry[32], entry[33], entry[34], entry[35],
            entry[36], entry[37], entry[38], entry[39],
        ]);
        let last_lba = u64::from_le_bytes([
            entry[40], entry[41], entry[42], entry[43],
            entry[44], entry[45], entry[46], entry[47],
        ]);
        info!(
            "  entry {}: type GUID = {:02x}{:02x}{:02x}{:02x}-{:02x}{:02x}-{:02x}{:02x}-{:02x}{:02x}-{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}, lba {} .. {} (~{} MB)",
            i,
            // First three fields are little-endian on disk.
            type_guid[3], type_guid[2], type_guid[1], type_guid[0],
            type_guid[5], type_guid[4],
            type_guid[7], type_guid[6],
            type_guid[8], type_guid[9],
            type_guid[10], type_guid[11], type_guid[12], type_guid[13], type_guid[14], type_guid[15],
            first_lba, last_lba,
            ((last_lba - first_lba + 1) * 512 / 1_048_576),
        );
    }
}

// Suppress dead-code warning in case the constant goes unused depending on
// what the test code reaches for.
#[allow(dead_code)]
const _: u8 = ACMD;
