//! CDI (DiscJuggler) parser.
//!
//! CDI stores a disc back to front: raw track data starts at file offset 0
//! with no header at all, a descriptor is appended at the end, and the final
//! four bytes give the descriptor's length. Everything is little-endian and
//! packed.
//!
//! Descriptor layout, confirmed against a real image (dcload-ser 1.0.6) and
//! against the writer in mkdcdisc (`src/disc_image/formats/cdi.c`):
//!
//! ```text
//!   u8   session_count
//!   per session:
//!     u8   padding
//!     u16  track_count
//!     u32  unknown
//!     per track:
//!       track header   56 + filename_length bytes
//!       track data     180 bytes, or 188 for the last track of a session
//!   terminator: one empty session header, one track header, disc info
//!   u32  descriptor length, counted back from EOF
//! ```
//!
//! The filename is *variable length* — mkdcdisc always writes 32 bytes, but
//! DiscJuggler writes the full source path (56 bytes in the image above), so
//! a fixed 32 cannot be assumed; getting this wrong silently misaligns every
//! field after it.
//!
//! Track data is laid out in the file back to back in descriptor order, each
//! track occupying `total_length` sectors (pregap included) of
//! `sector_size(read_mode)` bytes. A track's *data* therefore begins at the
//! running base plus `pregap * sector_size`. For the reference image that
//! arithmetic accounts for the file exactly: 17025*2352 + 16725*2352 +
//! 312653*2336 = 809,737,408, and the file is 809,738,626 — the 1218-byte
//! descriptor.

use defmt::info;
use embedded_sdmmc::{Directory, ShortFileName};

use crate::blockdev::HsmciBlockDevice;
use crate::gdi::{GdiError, Track};

type Dir<'a> = Directory<'a, HsmciBlockDevice, crate::blockdev::DummyClock, 4, 4, 1>;

/// Bytes of a track header before the variable-length filename:
/// two 10-byte markers, 3 settings, total_tracks, filename_length.
const HDR_BEFORE_NAME: u32 = 25;
/// Bytes of a track header after the filename: 11 unknown, three u32,
/// max_cd_length, one more u32.
const HDR_AFTER_NAME: u32 = 31;
/// A track descriptor, when it is the last of its session. Non-final tracks
/// omit the trailing 8 bytes.
const TRACK_DATA_LEN: u32 = 188;
const SESSION_HDR_LEN: u32 = 7;

/// Field offsets within the track descriptor. Verified against a real image:
/// index_count u16 @0, pregap @2, sector_count @6, unknown[6] @10,
/// track_mode @16, unknown @20, session @24, track_number @28, start_lba @32,
/// total_length @36, unknown[16] @40, read_mode @56, control @60.
const OFF_PREGAP: usize = 2;
const OFF_SECTOR_COUNT: usize = 6;
const OFF_START_LBA: usize = 32;
const OFF_TOTAL_LENGTH: usize = 36;
const OFF_READ_MODE: usize = 56;
const OFF_CONTROL: usize = 60;

fn rd_u16(b: &[u8], at: usize) -> u16 {
    u16::from_le_bytes([b[at], b[at + 1]])
}
fn rd_u32(b: &[u8], at: usize) -> u32 {
    u32::from_le_bytes([b[at], b[at + 1], b[at + 2], b[at + 3]])
}

/// Stored bytes per sector for a CDI `read_mode`.
fn sector_size(read_mode: u32) -> Option<u16> {
    match read_mode {
        0 => Some(2048),
        1 => Some(2336),
        2 => Some(2352),
        _ => None,
    }
}

/// Find the first `*.CDI` in `dir`.
pub fn find_cdi(dir: &Dir<'_>) -> Result<ShortFileName, GdiError> {
    let mut found: Option<ShortFileName> = None;
    let _ = dir.iterate_dir(|entry| {
        if found.is_none()
            && !entry.attributes.is_directory()
            && entry.name.extension().eq_ignore_ascii_case(b"CDI")
        {
            found = Some(entry.name.clone());
        }
    });
    found.ok_or(GdiError::NoGdiFile)
}

/// Parse the descriptor of `name` in `dir` into a track map.
///
/// Walks the descriptor with small seeks rather than buffering it whole: a
/// many-track image runs to several KB and this part has 52 KB of SRAM to
/// live in, most of it already spoken for by the CD_READ staging buffer.
/// Fills the shared track pool rather than returning the tracks: 99 of them
/// is about 4 KB, and this part has roughly 2 KB of stack. See
/// [`crate::gdi::MAX_TRACKS`].
/// Returns the number of sessions the descriptor declares. A MIL-CD has two
/// (audio, then the bootable data track), and the host is told that count via
/// REQ_SES(0) -- it is how the BIOS learns session 2 exists at all.
pub fn parse(dir: &Dir<'_>, name: &ShortFileName) -> Result<u8, GdiError> {
    let f = dir
        .open_file_in_dir(name, embedded_sdmmc::Mode::ReadOnly)
        .map_err(|_| GdiError::OpenGdiFile)?;
    let file_len = f.length();
    if file_len < 8 {
        return Err(GdiError::BadCdi);
    }

    let read_at = |off: u32, buf: &mut [u8]| -> Result<(), GdiError> {
        f.seek_from_start(off).map_err(|_| GdiError::ReadGdiFile)?;
        let mut done = 0usize;
        while done < buf.len() {
            let n = f.read(&mut buf[done..]).map_err(|_| GdiError::ReadGdiFile)?;
            if n == 0 {
                return Err(GdiError::ReadGdiFile);
            }
            done += n;
        }
        Ok(())
    };

    let mut w4 = [0u8; 4];
    read_at(file_len - 4, &mut w4)?;
    let desc_len = rd_u32(&w4, 0);
    if desc_len < 16 || desc_len > file_len {
        return Err(GdiError::BadCdi);
    }
    let desc_start = file_len - desc_len;

    let mut one = [0u8; 1];
    read_at(desc_start, &mut one)?;
    let session_count = one[0];
    info!("cdi: descriptor {} bytes, {} session(s)", desc_len, session_count);

    // Cleared first, so a parse that fails part way cannot leave the
    // previous image's tail behind this one's.
    let tracks = crate::gdi::tracks_mut();
    tracks.clear();
    // Running base of the next track's stored data within the file.
    let mut base: u32 = 0;
    let mut cursor = desc_start + 1;
    let mut number: u8 = 1;

    for _ in 0..session_count {
        let mut sh = [0u8; SESSION_HDR_LEN as usize];
        read_at(cursor, &mut sh)?;
        let track_count = rd_u16(&sh, 1);
        cursor += SESSION_HDR_LEN;

        for ti in 0..track_count {
            // Header: read up to the length byte, then skip the name.
            let mut head = [0u8; HDR_BEFORE_NAME as usize];
            read_at(cursor, &mut head)?;
            let name_len = head[HDR_BEFORE_NAME as usize - 1] as u32;
            let td_at = cursor + HDR_BEFORE_NAME + name_len + HDR_AFTER_NAME;

            let mut td = [0u8; TRACK_DATA_LEN as usize];
            read_at(td_at, &mut td)?;

            let pregap = rd_u32(&td, OFF_PREGAP);
            let sector_count = rd_u32(&td, OFF_SECTOR_COUNT);
            let start_lba = rd_u32(&td, OFF_START_LBA);
            let total_length = rd_u32(&td, OFF_TOTAL_LENGTH);
            let read_mode = rd_u32(&td, OFF_READ_MODE);
            let control = rd_u32(&td, OFF_CONTROL);

            let ssize = sector_size(read_mode).ok_or(GdiError::BadCdi)?;

            // Data begins past this track's own pregap.
            let file_offset = base
                .checked_add(pregap.checked_mul(ssize as u32).ok_or(GdiError::BadCdi)?)
                .ok_or(GdiError::BadCdi)?;

            if tracks
                .push(Track {
                    number,
                    start_lba,
                    ctrl: (control & 0x0F) as u8,
                    sector_size: ssize,
                    sector_count,
                    filename: name.clone(),
                    file_offset,
                    ext_start: 0,
                    ext_len: 0,
                })
                .is_err()
            {
                return Err(GdiError::BadCdi);
            }
            info!(
                "cdi: track {} lba={} sectors={} ssize={} ctrl={:#04x} off={}",
                number, start_lba, sector_count, ssize, (control & 0x0F) as u8, file_offset
            );

            base = base
                .checked_add(total_length.checked_mul(ssize as u32).ok_or(GdiError::BadCdi)?)
                .ok_or(GdiError::BadCdi)?;
            number = number.wrapping_add(1);

            // The last track of a session carries 8 extra bytes.
            let is_last = ti + 1 == track_count;
            cursor = td_at + if is_last { TRACK_DATA_LEN } else { TRACK_DATA_LEN - 8 };
        }
    }

    if tracks.is_empty() {
        return Err(GdiError::BadCdi);
    }
    Ok(session_count)
}
