//! Minimal FAT32 reader for building block-extent maps of the track files.
//!
//! The data path wants to read raw sectors straight off the SD card with
//! multi-block CMD18 reads into whichever buffer it likes -- the CD_READ
//! staging buffer, or the CDDA ring that must keep up with real time. The
//! `embedded_sdmmc` path we use for everything else re-opens the volume,
//! re-walks the directory and reads one 512-byte block at a time through a
//! bounce buffer, which cost ~55 ms per CD_READ chunk. So at image load we
//! walk the FAT once and remember each track file as a short list of
//! contiguous block runs (`Extent`); after that a disc LBA maps to an SD
//! block with integer arithmetic and no filesystem code at all.
//!
//! Only FAT32 on a 512-byte-sector volume is handled (with or without an
//! MBR in front). Anything else -- FAT16, exFAT, a file with more runs than
//! the pool can hold -- leaves the track unmapped and the slow path stays in
//! charge, so this is strictly an accelerator.

use defmt::{info, warn, Format};
use embassy_atsam3::hsmci;
use embedded_sdmmc::ShortFileName;
use heapless::Vec;

/// One contiguous run of a file on the card.
#[derive(Clone, Copy, Format)]
pub struct Extent {
    /// Byte offset within the file where this run starts.
    pub file_off: u32,
    /// First SD block of the run.
    pub lba: u32,
    /// Length of the run in 512-byte blocks.
    pub blocks: u32,
}

/// Shared pool: every mapped track file owns a slice of it (see
/// `Track::ext_start` / `ext_len`). A freshly formatted card writes each
/// file in one or two runs; 96 leaves room for a moderately fragmented
/// 20-track image before we give up on the fast path.
pub const MAX_EXTENTS: usize = 96;

/// Volume geometry, everything in 512-byte blocks.
#[derive(Clone, Copy, Format)]
pub struct Geometry {
    fat_lba: u32,
    sectors_per_fat: u32,
    data_lba: u32,
    blocks_per_cluster: u32,
    pub root_cluster: u32,
}

impl Geometry {
    pub fn cluster_lba(&self, cluster: u32) -> u32 {
        self.data_lba + (cluster - 2) * self.blocks_per_cluster
    }

    /// Bytes per cluster.
    pub fn cluster_bytes(&self) -> u32 {
        self.blocks_per_cluster * 512
    }
}

/// A directory entry we care about.
#[derive(Clone, Copy, Format)]
pub struct Entry {
    pub first_cluster: u32,
    pub size: u32,
    pub is_dir: bool,
}

fn rd16(b: &[u8], off: usize) -> u32 {
    u16::from_le_bytes([b[off], b[off + 1]]) as u32
}

fn rd32(b: &[u8], off: usize) -> u32 {
    u32::from_le_bytes([b[off], b[off + 1], b[off + 2], b[off + 3]])
}

fn read_block(lba: u32, buf: &mut [u8; 512]) -> Result<(), ()> {
    unsafe { hsmci::read_block(lba, buf) }.map_err(|_| ())
}

fn looks_like_bpb(s: &[u8; 512]) -> bool {
    (s[0] == 0xEB || s[0] == 0xE9) && rd16(s, 0x0B) == 512 && s[0x0D] != 0
}

/// Locate the first FAT32 volume on the card.
pub fn probe() -> Option<Geometry> {
    let mut sec = [0u8; 512];
    read_block(0, &mut sec).ok()?;
    if sec[510] != 0x55 || sec[511] != 0xAA {
        return None;
    }
    // Same choice embedded-sdmmc makes for VolumeIdx(0): partition entry 0
    // of an MBR, or the whole card if LBA 0 already is a BPB.
    let base = if looks_like_bpb(&sec) {
        0
    } else {
        let start = rd32(&sec, 0x1BE + 8);
        if start == 0 {
            return None;
        }
        read_block(start, &mut sec).ok()?;
        if !looks_like_bpb(&sec) || sec[510] != 0x55 || sec[511] != 0xAA {
            return None;
        }
        start
    };
    let blocks_per_cluster = sec[0x0D] as u32;
    let reserved = rd16(&sec, 0x0E);
    let num_fats = sec[0x10] as u32;
    let root_entries = rd16(&sec, 0x11);
    let spf16 = rd16(&sec, 0x16);
    let spf32 = rd32(&sec, 0x24);
    let root_cluster = rd32(&sec, 0x2C);
    if root_entries != 0 || spf16 != 0 || spf32 == 0 || num_fats == 0 || root_cluster < 2 {
        warn!("fat: volume at {} is not FAT32", base);
        return None;
    }
    let g = Geometry {
        fat_lba: base + reserved,
        sectors_per_fat: spf32,
        data_lba: base + reserved + num_fats * spf32,
        blocks_per_cluster,
        root_cluster,
    };
    info!("fat: FAT32 at {}, {} blocks/cluster, fat@{} data@{}",
          base, blocks_per_cluster, g.fat_lba, g.data_lba);
    Some(g)
}

/// Next cluster in the chain, or `None` at end-of-chain / bad entry.
/// `cache` holds one FAT sector so a chain walk mostly costs no reads.
struct FatCursor {
    sector: u32,
    buf: [u8; 512],
}

impl FatCursor {
    fn new() -> Self {
        Self { sector: u32::MAX, buf: [0; 512] }
    }

    fn next(&mut self, g: &Geometry, cluster: u32) -> Result<Option<u32>, ()> {
        let sector = cluster / 128;
        if sector >= g.sectors_per_fat {
            return Err(());
        }
        if sector != self.sector {
            read_block(g.fat_lba + sector, &mut self.buf)?;
            self.sector = sector;
        }
        let v = rd32(&self.buf, (cluster % 128) as usize * 4) & 0x0FFF_FFFF;
        if v >= 0x0FFF_FFF8 {
            Ok(None)
        } else if v < 2 || v == 0x0FFF_FFF7 {
            Err(())
        } else {
            Ok(Some(v))
        }
    }
}

/// Compare an on-disk 11-byte name with a `ShortFileName`.
fn name_matches(raw: &[u8], name: &ShortFileName) -> bool {
    let trim = |s: &[u8]| -> usize {
        let mut n = s.len();
        while n > 0 && s[n - 1] == b' ' {
            n -= 1;
        }
        n
    };
    let base = &raw[..trim(&raw[..8])];
    let ext = &raw[8..8 + trim(&raw[8..11])];
    base.eq_ignore_ascii_case(name.base_name()) && ext.eq_ignore_ascii_case(name.extension())
}

/// Look `name` up in the directory starting at `dir_cluster`.
pub fn find_in_dir(g: &Geometry, dir_cluster: u32, name: &ShortFileName) -> Option<Entry> {
    let mut fat = FatCursor::new();
    let mut buf = [0u8; 512];
    let mut cluster = dir_cluster;
    loop {
        let base = g.cluster_lba(cluster);
        for b in 0..g.blocks_per_cluster {
            read_block(base + b, &mut buf).ok()?;
            for e in buf.chunks_exact(32) {
                match e[0] {
                    0x00 => return None,
                    0xE5 => continue,
                    _ => {}
                }
                let attr = e[11];
                if attr & 0x0F == 0x0F || attr & 0x08 != 0 {
                    continue; // LFN piece or volume label
                }
                if name_matches(&e[..11], name) {
                    return Some(Entry {
                        first_cluster: (rd16(e, 0x14) << 16) | rd16(e, 0x1A),
                        size: rd32(e, 0x1C),
                        is_dir: attr & 0x10 != 0,
                    });
                }
            }
        }
        cluster = fat.next(g, cluster).ok()??;
    }
}

/// Walk `entry`'s cluster chain and append its runs to `out`. Returns the
/// number of extents added, or `None` if the pool ran out or the chain is
/// broken (the caller then leaves the file on the slow path).
pub fn map_extents<const N: usize>(
    g: &Geometry,
    entry: &Entry,
    out: &mut Vec<Extent, N>,
) -> Option<usize> {
    if entry.first_cluster < 2 || entry.size == 0 {
        return None;
    }
    let start_len = out.len();
    let mut fat = FatCursor::new();
    let mut cluster = entry.first_cluster;
    let cluster_bytes = g.cluster_bytes();
    // Blocks the file actually occupies; the last cluster is usually partial.
    let mut blocks_left = (entry.size + 511) / 512;
    let mut file_off = 0u32;
    let mut run = Extent { file_off: 0, lba: g.cluster_lba(cluster), blocks: 0 };
    loop {
        let n = g.blocks_per_cluster.min(blocks_left);
        run.blocks += n;
        blocks_left -= n;
        file_off = file_off.wrapping_add(cluster_bytes);
        if blocks_left == 0 {
            break;
        }
        let next = match fat.next(g, cluster) {
            Ok(Some(c)) => c,
            _ => {
                warn!("fat: chain ends early ({} blocks short)", blocks_left);
                out.truncate(start_len);
                return None;
            }
        };
        if next != cluster + 1 {
            if out.push(run).is_err() {
                warn!("fat: extent pool full");
                out.truncate(start_len);
                return None;
            }
            run = Extent { file_off, lba: g.cluster_lba(next), blocks: 0 };
        }
        cluster = next;
    }
    if out.push(run).is_err() {
        warn!("fat: extent pool full");
        out.truncate(start_len);
        return None;
    }
    Some(out.len() - start_len)
}

/// Resolve a byte offset within a mapped file to `(sd_block, offset within
/// that block, contiguous blocks available from there)`.
pub fn locate(extents: &[Extent], byte_off: u32) -> Option<(u32, u32, u32)> {
    let e = extents
        .iter()
        .rfind(|e| e.file_off <= byte_off)?;
    let rel = (byte_off - e.file_off) / 512;
    if rel >= e.blocks {
        return None;
    }
    Some((e.lba + rel, byte_off % 512, e.blocks - rel))
}
