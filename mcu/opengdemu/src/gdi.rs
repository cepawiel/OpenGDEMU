//! GDI parser + LBA-to-file map.
//!
//! GDI ("GD-ROM Image") is the de-facto Dreamcast disc-image format. The
//! `.gdi` text file at the root of the image directory describes the
//! tracks; each track points at a binary file holding raw sectors.
//!
//! Layout of a typical 4-track GD-ROM image:
//!
//! ```text
//! 4
//! 1 0 4 2352 track01.bin 0
//! 2 600 0 2352 track02.raw 0
//! 3 45000 4 2352 track03.bin 0
//! 4 549150 0 2352 track04.raw 0
//! ```
//!
//! Columns: track-number, start-LBA, ctrl-bits, sector-size, filename,
//! pregap (we ignore pregap for now). Tracks 1-2 live in the SDA (Single
//! Density Area, LBAs 0..11400), track 3+ live in the HDA (High Density
//! Area, starts at LBA 45000 by Sega convention).

use defmt::{info, warn, Format};
use embedded_sdmmc::{Directory, ShortFileName, VolumeManager};
use heapless::{String, Vec};

use crate::blockdev::{BlockError, HsmciBlockDevice};

/// Maximum tracks we ever expect. Red Book technically caps at 99 but
/// Red Book's limit, and the real one.
///
/// This was 32, justified by "real Dreamcast GDIs are <=22 tracks". That is
/// not true: seventeen of the 298 US retail discs exceed 32, and Tom Clancy's
/// Rainbow Six - Rogue Spear has 98. They are the CDDA-heavy ones, so the
/// more music a game shipped with the less likely it was to mount at all --
/// `TooManyTracks`, surfacing to the console as an ILLEGAL REQUEST on MOUNT.
///
/// The 32 did buy something real, though: a `Vec<Track, 99>` is about 4 KB
/// and this part has roughly 2 KB of stack, so the tracks cannot live in a
/// `Gdi` that is built as a local and returned by value. Hence [`TRACKS`].
pub const MAX_TRACKS: usize = 99;
/// Max GDI file size we'll bother reading. A 4-track GDI is ~80 bytes.
const MAX_GDI_BYTES: usize = 4096;

#[derive(Clone, Format)]
pub struct Track {
    pub number: u8,
    pub start_lba: u32,
    pub ctrl: u8,
    pub sector_size: u16,
    /// Length in sectors. Derived from the track file's size on disk
    /// (`file_size / sector_size`). Used to compute the lead-out FAD
    /// reported in the TOC.
    pub sector_count: u32,
    /// 8.3 short file name as embedded-sdmmc reports it. Used to open
    /// the track binary at read time.
    pub filename: ShortFileName,
    /// Byte offset within `filename` at which this track's `start_lba`
    /// sector begins. GDI keeps one file per track so this is always 0;
    /// CDI concatenates every track into a single file, so each track
    /// sits at its own base offset (past the preceding tracks, and past
    /// its own pregap).
    pub file_offset: u32,
    /// Slice of the shared extent pool (`extents()`) describing where
    /// `filename` sits on the card, or `ext_len == 0` when the file could
    /// not be mapped and reads must go through the filesystem instead.
    pub ext_start: u16,
    pub ext_len: u16,
}

/// Block-run map of every mapped track file, filled by `load_image`.
static mut EXTENTS: Vec<crate::fat::Extent, { crate::fat::MAX_EXTENTS }> = Vec::new();

/// The mounted image's tracks.
///
/// A static pool rather than a field of [`Gdi`], because 99 of them is about
/// 4 KB and this part has about 2 KB of stack -- as a field they would be
/// built as a local in [`parse`] and moved through two more frames on the way
/// to `ide::GDI`, which is the deepest, most stack-fragile path there is.
///
/// One pool, because one image is mounted at a time. A load that fails part
/// way leaves it holding rubbish, which is safe only because a failed mount
/// already clears the mounted image (`ide.rs`, `SUB_MOUNT`): the console is
/// told the media is gone rather than being served half of the last disc.
static mut TRACKS: Vec<Track, MAX_TRACKS> = Vec::new();

/// Everything currently in the pool.
fn tracks() -> &'static [Track] {
    unsafe { &*core::ptr::addr_of!(TRACKS) }
}

pub(crate) fn tracks_mut() -> &'static mut Vec<Track, MAX_TRACKS> {
    unsafe { &mut *core::ptr::addr_of_mut!(TRACKS) }
}

impl Track {
    /// The track file's block runs, if it was mapped at load time.
    pub fn extents(&self) -> Option<&'static [crate::fat::Extent]> {
        if self.ext_len == 0 {
            return None;
        }
        let pool = unsafe { &*core::ptr::addr_of!(EXTENTS) };
        pool.get(self.ext_start as usize..(self.ext_start + self.ext_len) as usize)
    }

    /// Byte offset within `filename` of disc LBA `lba`.
    pub fn byte_offset_of(&self, lba: u32) -> Option<u32> {
        lba.checked_sub(self.start_lba)?
            .checked_mul(self.sector_size as u32)?
            .checked_add(self.file_offset)
    }

    /// SD block holding file byte `byte_off` of this track's file:
    /// `(block, offset within the block, contiguous blocks from there)`.
    /// `None` if the track is unmapped or the offset lies past its runs.
    pub fn locate_byte(&self, byte_off: u32) -> Option<(u32, u32, u32)> {
        crate::fat::locate(self.extents()?, byte_off)
    }

    /// Byte offset of the 2048-byte user data within one stored sector.
    /// 2352 is a raw sector (12 sync + 4 header) and 2336 is the CDI
    /// data form, which drops sync and header but keeps the 8-byte
    /// Mode 2 subheader. 2048 is already cooked.
    pub fn user_data_offset(&self) -> usize {
        match self.sector_size {
            2352 => 16,
            2336 => 8,
            _ => 0,
        }
    }

    pub fn is_data(&self) -> bool {
        // Ctrl bit 2 set → data track (CD-ROM Mode 1/2). Audio = bit 2 = 0.
        (self.ctrl & 0x04) != 0
    }
}

#[derive(Format)]
pub struct Gdi {
    /// LBA where the bootable area starts (typically 45000).
    ///
    /// Named for what it is on both kinds of image rather than for the
    /// GD-ROM's name for it: on a GDI this is the start of the High Density
    /// Area, and on a CDI it is the start of session 2 -- a MIL-CD has no
    /// high-density area at all. Both are the same thing to everything that
    /// reads this: where IP.BIN lives, which tracks sit above the split, and
    /// which track REQ_SES calls the second session's first.
    pub boot_area_lba: u32,
    /// Path from the card root to the folder holding the .gdi file and the
    /// per-track binaries. Stored so the IDE-side data path can re-open it
    /// on every CD_READ without re-walking the volume.
    pub image_dir: ImagePath,
    /// SD card block count, captured at parse time so the IDE task can
    /// rebuild a VolumeManager on demand without re-querying CSD.
    pub num_blocks: u32,
    /// True when this image came from a `.cdi` rather than a `.gdi`.
    ///
    /// A CDI here means a MIL-CD, and it is presented as what it is: a CD,
    /// not a GD-ROM. That changes three things, all of which have to agree or
    /// the BIOS abandons the boot -- the disc format reported in SECNR
    /// (CD-ROM XA, see `ide::disc_format`), the TOC (every track in the
    /// single-density area, since a CD has no high-density one), and the
    /// lead-out (the truth, because a CDI descriptor carries a real sector
    /// count per track, where a trimmed GDI must be padded out to a pressed
    /// disc's capacity). See [`Gdi::leadout_lba`] and [`Gdi::build_toc`].
    pub is_cdi: bool,
    /// How many sessions the disc has.
    ///
    /// Reported to the host as REQ_SES(0)'s track byte, which is where the
    /// BIOS reads the session *count* -- not, as was assumed here, the first
    /// track number. The two are indistinguishable on a GD-ROM, which stock
    /// GDEMU reports as a single session whose first track is also 1; they
    /// differ on a MIL-CD, which has two sessions and keeps everything
    /// bootable in the second. Saying 1 there tells the BIOS session 2 does
    /// not exist, and it stops looking. Verified against flycast
    /// `core/imgread/common.h` (`GetSessionInfo`: `to[2] = sessions.size()`).
    pub sessions: u8,
}

impl Gdi {
    /// This image's tracks.
    ///
    /// They live in [`TRACKS`], not in here: see that static for why. The
    /// consequence to know is that a `Gdi` kept across a mount describes the
    /// *new* image's tracks, so there is no point holding one -- ask
    /// `ide::gdi_ref()` each time, which everything already does.
    pub fn tracks(&self) -> &'static [Track] {
        tracks()
    }

    /// Track containing `lba`, if any. Linear scan since N ≤ 99.
    pub fn track_for_lba(&self, lba: u32) -> Option<&Track> {
        self.tracks().iter().rfind(|t| t.start_lba <= lba)
    }

    /// `(filename, byte_offset)` for reading `lba`. Caller opens the
    /// file by name and seeks to the offset.
    pub fn locate(&self, lba: u32) -> Option<(ShortFileName, u32)> {
        let t = self.track_for_lba(lba)?;
        let sector = lba - t.start_lba;
        let offset = sector
            .checked_mul(t.sector_size as u32)?
            .checked_add(t.file_offset)?;
        Some((t.filename.clone(), offset))
    }

    /// First and last track numbers (used by READ_TOC).
    pub fn track_bounds(&self) -> Option<(u8, u8)> {
        let first = self.tracks().first()?.number;
        let last = self.tracks().last()?.number;
        Some((first, last))
    }

    /// Total disc length in LBAs, used as the lead-out address.
    ///
    /// A GDI carries no track lengths, and a pressed GD-ROM's high-density
    /// area always runs out to LBA 549150, so that is what a GDI reports --
    /// the conservative upper bound the BIOS expects for a GD-ROM rather
    /// than a CD-ROM, and correct even for a trimmed dump.
    ///
    /// A CDI is different, and reporting 549150 for one is a lie the BIOS
    /// catches. The DiscJuggler descriptor gives an exact sector count per
    /// track, so the real end of the disc is known; a MIL-CD's data session
    /// ends where its last track ends, nowhere near a pressed disc's
    /// capacity. Claiming otherwise describes a GD-ROM whose high-density
    /// area runs ~192000 sectors past its own last track, and the BIOS
    /// abandons the boot and starts over rather than load anything -- it
    /// never reads past the ISO9660 root directory.
    pub fn leadout_lba(&self) -> u32 {
        if self.is_cdi {
            if let Some(last) = self.tracks().last() {
                return last.start_lba + last.sector_count;
            }
        }
        549150
    }

    /// Build a 408-byte Sega GD-ROM TOC response. `area` selects which
    /// session of the disc to describe:
    ///   0 = Single Density Area (tracks with LBA < 45000)
    ///   1 = High  Density Area (tracks with LBA >= 45000)
    ///
    /// Layout: 99 4-byte entries (tracks 1..99), then first-track,
    /// last-track, and lead-out entries — also 4 bytes each. Each
    /// entry is `[CTRL_ADR, FAD_hi, FAD_mid, FAD_lo]` where CTRL_ADR
    /// = `(ctl << 4) | 1` (CTL high nibble, ADR=1 low nibble — the
    /// Sega/iceGDROM convention, opposite to what the standard ATAPI
    /// READ_TOC uses). Unused track slots are 0xFF-filled.
    pub fn build_toc(&self, area: u8, out: &mut [u8; 408]) {
        out.fill(0xFF);

        // A MIL-CD has no high-density area. Every track on it is an ordinary
        // CD track in the single-density area, and once the host is told the
        // disc is CD-ROM XA it stops asking for area 1 altogether -- so
        // splitting a CDI at `boot_area_lba` hides its data track from the
        // only area the host still looks at. The BIOS then sees two audio
        // tracks with nothing bootable and never issues a read at all.
        let in_area = |t: &Track| -> bool {
            if self.is_cdi {
                area == 0
            } else if area == 0 {
                t.start_lba < self.boot_area_lba
            } else {
                t.start_lba >= self.boot_area_lba
            }
        };
        // Sega TOC layout (per iceGDROM tools/imgfile.c:165): CTL in
        // the upper nibble, ADR=1 in the lower. Opposite of the
        // standard CD-ROM TOC, but the DC BIOS does this byte-swap
        // and expects this convention.
        let ctrl_adr = |ctl: u8| -> u8 { (ctl << 4) | 0x01 };

        let mut first: Option<u8> = None;
        let mut last: Option<u8> = None;
        let mut last_track_ref: Option<&Track> = None;

        for t in self.tracks().iter() {
            if !in_area(t) {
                continue;
            }
            if t.number == 0 || t.number > 99 {
                continue;
            }
            let idx = (t.number - 1) as usize * 4;
            let fad = t.start_lba + 150;
            out[idx] = ctrl_adr(t.ctrl);
            out[idx + 1] = (fad >> 16) as u8;
            out[idx + 2] = (fad >> 8) as u8;
            out[idx + 3] = fad as u8;

            first.get_or_insert(t.number);
            last = Some(t.number);
            last_track_ref = Some(t);
        }

        if let (Some(f), Some(l), Some(lt)) = (first, last, last_track_ref) {
            // First track entry (offset 99*4 = 396).
            let first_ctrl = self.tracks().iter()
                .find(|t| t.number == f)
                .map(|t| ctrl_adr(t.ctrl))
                .unwrap_or(0x01);
            out[396] = first_ctrl;
            out[397] = f;
            out[398] = 0;
            out[399] = 0;
            // Last track entry (offset 100*4 = 400).
            out[400] = ctrl_adr(lt.ctrl);
            out[401] = l;
            out[402] = 0;
            out[403] = 0;
            // Lead-out entry (offset 101*4 = 404). FAD = first sector
            // after the last track (matches iceGDROM tools/imgfile.c
            // line 172: start_sector + data_count).
            // A GDI carries no track lengths; a pressed GD-ROM's high-density
            // area always runs out to LBA 549150, so report that for the HD
            // lead-out even when the last track file is trimmed short
            // (homebrew GDIs, "trimmed" dumps). The low-density area is only
            // as long as its files.
            //
            // A CDI is exempt: its descriptor gives exact per-track sector
            // counts, so `lt` already ends where the disc really ends, and
            // padding a MIL-CD out to a pressed disc's capacity is what made
            // the BIOS refuse to boot one.
            let mut leadout_fad = (lt.start_lba + lt.sector_count) + 150;
            if area == 1 && !self.is_cdi {
                leadout_fad = leadout_fad.max(self.leadout_lba() + 150);
            }
            out[404] = ctrl_adr(lt.ctrl);
            out[405] = (leadout_fad >> 16) as u8;
            out[406] = (leadout_fad >> 8) as u8;
            out[407] = leadout_fad as u8;
        }
    }
}

/// How deep an image may sit below the card root: the game's own folder,
/// plus room for a couple of levels above it.
pub const MAX_PATH_PARTS: usize = 3;

/// Where an image's folder is, as the components to walk from the root.
///
/// Not a single name. The host names an image by path -- `us/Crazy Taxi …`
/// -- because a card holding several hundred discs wants them grouped, and
/// nothing here needs to know what the grouping means: it walks whatever it
/// is handed. A path of one component is the old behaviour exactly.
pub type ImagePath = Vec<ShortFileName, MAX_PATH_PARTS>;

/// Walk `path` from `dir`, which becomes the image's directory.
///
/// `change_dir` rather than `open_dir` because the latter returns a
/// directory borrowing the one it came from, which cannot be walked in a
/// loop -- and because descending in place holds one directory slot rather
/// than one per level.
fn descend<D, T, const A: usize, const B: usize, const C: usize>(
    dir: &mut embedded_sdmmc::Directory<'_, D, T, A, B, C>,
    path: &ImagePath,
) -> Result<(), GdiError>
where
    D: embedded_sdmmc::BlockDevice,
    T: embedded_sdmmc::TimeSource,
{
    for part in path.iter() {
        dir.change_dir(part.clone())
            .map_err(|_| GdiError::OpenImageDir)?;
    }
    Ok(())
}

/// One selectable disc image: the directory holding it and the descriptor
/// file inside, plus which format that file is.
#[derive(Clone, Format)]
pub struct ImageRef {
    pub dir: ImagePath,
    pub file: ShortFileName,
    pub is_cdi: bool,
    /// The directory's long name when it has one, else its 8.3 name. This is
    /// what the config matches on and what the host is shown.
    ///
    /// Matching on the 8.3 alias would be fragile: `SEGARA~1` is whatever the
    /// tool that created the directory happened to pick, and a different
    /// collision order on another machine yields `SEGARA~2` for the same disc,
    /// silently breaking the config. Files are still *opened* by short name --
    /// embedded-sdmmc reads long names but does not look up by them.
    pub name: String<MAX_NAME_CHARS>,
}

/// Longest image name we keep.
///
/// A name is a path now, and the folder at the end of it is a full retail
/// title -- "Alone in the Dark - The New Nightmare v1.000 (2001)(Infogrames)
/// (US)(Disc 2 of 2)[!]" is 83 characters before any shelf in front of it.
/// At 64 those names truncated, and two discs of the same game truncated to
/// the *same* string, so a mount picked whichever came first.
pub const MAX_NAME_CHARS: usize = gamecfg::MAX_NEXT;

/// Scratch for reading long file names. Shared by the two places that walk
/// a directory -- they never run at the same time, and this part has about
/// two kilobytes of stack to its name, so a second copy is not free.
static mut LFN_STORE: [u8; 4 * MAX_NAME_CHARS] = [0; 4 * MAX_NAME_CHARS];


/// Config file at the root of the card. 8.3 only -- the DC-side tools build
/// FatFs with FF_USE_LFN=0, and "OPENGDEMU.CFG" has a nine-character base
/// name, so it cannot even be opened there.
const CONFIG_NAME: &str = "GDEMU.CFG";
const MAX_CONFIG_BYTES: usize = 512;

/// How a directory entry is named for matching and display.
///
/// The long filename when there is one, else the 8.3 name. A directory whose
/// name already fits 8.3 and needs no case preservation -- LAUNCH, MENU --
/// has no long-filename entry at all, so `long` is `None` for it. Deriving
/// this in one place is the point: `enumerate_images` had the fallback and
/// `load_by_name` did not, which made those folders listable but impossible
/// to mount by name (`NoGdiFile`, since no directory ever matched).
fn entry_name(entry: &embedded_sdmmc::DirEntry, long: Option<&str>) -> String<MAX_NAME_CHARS> {
    let mut name: String<MAX_NAME_CHARS> = String::new();
    match long {
        Some(l) if !l.is_empty() => {
            for c in l.chars() {
                if name.push(c).is_err() {
                    break;
                }
            }
        }
        _ => {
            for b in entry.name.base_name() {
                let _ = name.push(*b as char);
            }
            if !entry.name.extension().is_empty() {
                let _ = name.push('.');
                for b in entry.name.extension() {
                    let _ = name.push(*b as char);
                }
            }
        }
    }
    name
}


/// Read a folder's `GAME.CFG`, or the defaults if it has none.
///
/// A folder without one is an ordinary single disc, which is what
/// `Meta::default` means -- so a card that predates this file keeps working
/// exactly as it did.
fn read_game_cfg<D, T, const A: usize, const B: usize, const C: usize>(
    dir: &embedded_sdmmc::Directory<'_, D, T, A, B, C>,
) -> gamecfg::Meta
where
    D: embedded_sdmmc::BlockDevice,
    T: embedded_sdmmc::TimeSource,
{
    let Ok(f) = dir.open_file_in_dir(gamecfg::FILENAME, embedded_sdmmc::Mode::ReadOnly) else {
        return gamecfg::Meta::default();
    };
    // Static rather than stack: this runs in the same frame that later holds
    // the GDI text buffer, and the two together used to overflow the task.
    //
    // Small on purpose. These files hold a handful of keys, and this part has
    // about two kilobytes of RAM to spare in total -- a 1 KB buffer here was
    // a quarter of it.
    static mut CFG_BUF: [u8; 256] = [0; 256];
    let buf = unsafe { &mut *core::ptr::addr_of_mut!(CFG_BUF) };
    let got = f.read(buf).unwrap_or(0);
    gamecfg::Meta::parse(core::str::from_utf8(&buf[..got]).unwrap_or(""))
}

/// The image named by `GDEMU.CFG` at the root of the card, if it names one.
///
/// Returns the *name*, not an index into anything. Mounting is by name all
/// the way down -- `load_by_name` finds the one directory it needs -- so
/// there is nothing here for an index to be an index into, and resolving one
/// would mean walking the whole card first.
pub fn configured_name(num_blocks: u32) -> Option<String<MAX_NAME_CHARS>> {
    let dev = HsmciBlockDevice::new();
    dev.set_num_blocks(num_blocks);
    let mgr = VolumeManager::new(dev, crate::blockdev::DummyClock);
    let volume = mgr.open_volume(embedded_sdmmc::VolumeIdx(0)).ok()?;
    let root = volume.open_root_dir().ok()?;
    let name = ShortFileName::create_from_str(CONFIG_NAME).ok()?;

    static mut CFG: [u8; MAX_CONFIG_BYTES] = [0; MAX_CONFIG_BYTES];
    let buf = unsafe { &mut *core::ptr::addr_of_mut!(CFG) };
    let len = read_file_to_buf(&root, &name, buf).ok()?;

    for line in buf[..len].split(|c| *c == b'\n') {
        let line = trim_ascii(line);
        if line.is_empty() || line[0] == b'#' {
            continue;
        }
        let mut parts = line.splitn(2, |c| *c == b'=');
        let key = trim_ascii(parts.next()?);
        let val = trim_ascii(parts.next()?);
        if !key.eq_ignore_ascii_case(b"image") {
            continue;
        }
        let mut out: String<MAX_NAME_CHARS> = String::new();
        for b in val {
            if out.push(*b as char).is_err() {
                break;
            }
        }
        info!("gdi: config selects '{}'", out.as_str());
        return Some(out);
    }
    None
}

fn trim_ascii(mut s: &[u8]) -> &[u8] {
    while let Some((f, rest)) = s.split_first() {
        if f.is_ascii_whitespace() { s = rest; } else { break; }
    }
    while let Some((l, rest)) = s.split_last() {
        if l.is_ascii_whitespace() { s = rest; } else { break; }
    }
    s
}

/// The enumerated images, and which one is mounted.
///
/// The selection lives in RAM only, on purpose. `/OPENGDEMU.CFG` supplies the
/// boot default; swapping discs at runtime changes this and nothing else, so a
/// disc change never costs the card a write. Power-cycling returns to the
/// configured default.
///
/// It is held as a *name* rather than an index because the list is rebuilt on
/// every eject/load cycle -- that is the whole point of the eject window, the
/// host may have added or removed images while it was open -- and an index
/// would silently come to mean a different disc.
static mut SELECTED_DIR: Option<ImagePath> = None;
/// The mounted image under the name a host would use for it, so a remount can
/// go back through `load_by_name` without a catalog to look an index up in.
static mut SELECTED_NAME: Option<String<MAX_NAME_CHARS>> = None;

fn remember(dir: ImagePath, name: &str) {
    let mut n: String<MAX_NAME_CHARS> = String::new();
    for c in name.chars() {
        if n.push(c).is_err() {
            break;
        }
    }
    unsafe {
        SELECTED_DIR = Some(dir);
        SELECTED_NAME = Some(n);
    }
}





/// Load whichever image should be mounted now: the runtime choice if there is
/// one, else whatever `GDEMU.CFG` names. A card with neither comes up empty.
///
/// Both are resolved by name, so mounting opens one directory. Nothing here
/// walks the card or builds a list of images: which disc to serve is the
/// host's business, and the host picks it by name over 0xE0.
pub fn load_default(num_blocks: u32) -> Result<Gdi, GdiError> {
    if let Some(name) = unsafe { &*core::ptr::addr_of!(SELECTED_NAME) }.clone() {
        match load_by_name(num_blocks, name.as_str()) {
            Ok(gdi) => return Ok(gdi),
            // The card may have changed under an open tray; fall through
            // rather than refusing to mount anything at all.
            Err(e) => warn!("gdi: selected image '{}' is gone ({:?})", name.as_str(), e),
        }
    }

    if let Some(name) = configured_name(num_blocks) {
        match load_by_name(num_blocks, name.as_str()) {
            Ok(gdi) => return Ok(gdi),
            Err(e) => warn!(
                "gdi: config names '{}', which is not on the card ({:?})",
                name.as_str(),
                e
            ),
        }
    }

    // Nothing named, so nothing to mount. The firmware does not go looking:
    // finding "some image" means walking the card, and the answer to an
    // unconfigured card is to configure it, not to guess. The drive comes up
    // empty and the host sees no disc.
    warn!("gdi: no image selected and no usable GDEMU.CFG -- coming up empty");
    Err(GdiError::NoGdiFile)
}


/// The folder the mounted image's `GAME.CFG` names as `next`, read now
/// rather than remembered.
///
/// Opening the card for one file is cheap, and this runs from the button --
/// the shallowest caller there is. Keeping it instead would mean recording
/// it during a mount, which is the deep path this whole arrangement exists
/// to keep light.
pub fn next_of_mounted(num_blocks: u32) -> Option<String<{ gamecfg::MAX_NEXT }>> {
    let dir = unsafe { &*core::ptr::addr_of!(SELECTED_DIR) }.clone()?;
    let dev = HsmciBlockDevice::new();
    dev.set_num_blocks(num_blocks);
    let mgr = VolumeManager::new(dev, crate::blockdev::DummyClock);
    let volume = mgr.open_volume(embedded_sdmmc::VolumeIdx(0)).ok()?;
    let mut d = volume.open_root_dir().ok()?;
    descend(&mut d, &dir).ok()?;
    read_game_cfg(&d).next
}

/// Load the image in the folder called `want`, **without enumerating**.
///
/// The card is opened, the one directory is found by name, and its
/// descriptor is parsed. That is all a mount needs: which disc to serve is
/// the menu's business, and the menu can read the card itself.
///
/// The distinction matters more than it looks. `load_selected` calls
/// `enumerate_images`, which returns its sixteen-entry vector *by value* --
/// roughly 1.5 KB of stack on a part with about 2 KB of it in total. That
/// fits from `main_task` at boot and just fits from the host's MOUNT, but
/// from the button, deeper again and under a running game, it wedged the
/// firmware hard enough to take the debug port with it.
pub fn load_by_name(num_blocks: u32, want: &str) -> Result<Gdi, GdiError> {
    let dev = HsmciBlockDevice::new();
    dev.set_num_blocks(num_blocks);
    let mgr = VolumeManager::new(dev, crate::blockdev::DummyClock);
    let volume = mgr
        .open_volume(embedded_sdmmc::VolumeIdx(0))
        .map_err(|_| GdiError::OpenVolume)?;
    let mut d = volume.open_root_dir().map_err(|_| GdiError::OpenRoot)?;

    // Resolve the path one component at a time, matching each against the
    // long names in the directory above it. A name with no separator walks
    // exactly one level, which is what a card without shelves has.
    let mut path = ImagePath::new();
    for part in want.split('/').filter(|p| !p.is_empty()) {
        let mut lfn =
            embedded_sdmmc::LfnBuffer::new(unsafe { &mut *core::ptr::addr_of_mut!(LFN_STORE) });
        let mut hit: Option<ShortFileName> = None;
        let _ = d.iterate_dir_lfn(&mut lfn, |entry, long| {
            if hit.is_some()
                || !entry.attributes.is_directory()
                || entry.name.base_name() == b"."
                || entry.name.base_name() == b".."
            {
                return;
            }
            let name = entry_name(entry, long);
            if gamecfg::resolve(core::iter::once(name.as_str()), part).is_some() {
                hit = Some(entry.name.clone());
            }
        });
        let Some(short) = hit else {
            return Err(GdiError::NoGdiFile);
        };
        if path.push(short.clone()).is_err() {
            warn!("gdi: '{}' is deeper than {} levels", want, MAX_PATH_PARTS);
            return Err(GdiError::NoGdiFile);
        }
        d.change_dir(short).map_err(|_| GdiError::OpenImageDir)?;
    }
    if path.is_empty() {
        return Err(GdiError::NoGdiFile);
    }

    let found = match find_gdi(&d) {
        Ok(n) => Some((n, false)),
        Err(_) => crate::cdi::find_cdi(&d).ok().map(|n| (n, true)),
    };
    let Some((file, is_cdi)) = found else {
        return Err(GdiError::NoGdiFile);
    };
    let mut name: String<MAX_NAME_CHARS> = String::new();
    for c in want.chars() {
        if name.push(c).is_err() {
            break;
        }
    }
    let which = ImageRef {
        dir: path.clone(),
        file,
        is_cdi,
        name,
    };
    // Remember it, so a later remount keeps this disc.
    remember(path, want);
    load_image(num_blocks, &which)
}

/// Forget the runtime choice: the next load goes back to the configured
/// default (or the first image), exactly as a power cycle would.
pub fn select_default() {
    unsafe {
        SELECTED_DIR = None;
        SELECTED_NAME = None;
    }
    info!("gdi: selection reset to the configured default");
}

/// Load the image `which` names, ready to serve: descriptor parsed and every
/// track file mapped to its SD block runs.
pub fn load_image(num_blocks: u32, which: &ImageRef) -> Result<Gdi, GdiError> {
    let mut gdi = parse_image(num_blocks, which)?;
    map_track_extents(&mut gdi);
    Ok(gdi)
}

/// Parse `which` into a track table without touching the extent pool, so it
/// can be done for an image other than the mounted one (metadata lookups).
/// Borrows the CD_READ staging buffer for the descriptor text.
pub fn parse_image(num_blocks: u32, which: &ImageRef) -> Result<Gdi, GdiError> {
    let dev = HsmciBlockDevice::new();
    dev.set_num_blocks(num_blocks);

    let mgr = VolumeManager::new(dev, crate::blockdev::DummyClock);
    let volume = mgr
        .open_volume(embedded_sdmmc::VolumeIdx(0))
        .map_err(|_| GdiError::OpenVolume)?;
    let root = volume.open_root_dir().map_err(|_| GdiError::OpenRoot)?;

    let image_path = which.dir.clone();
    let image_file = which.file.clone();
    let is_cdi = which.is_cdi;
    info!("gdi: image '{}' file '{}'", which.name.as_str(), show_sfn(&image_file));
    let mut image_dir = root;
    descend(&mut image_dir, &image_path)?;

    // Prefer a .gdi; fall back to a .cdi. CDI keeps every track in one file
    // with the track map appended at the end, so it needs no companion track
    // binaries and its per-track sector counts come out of the descriptor
    // rather than from file sizes.
    let from_cdi = is_cdi;
    let mut gdi = if !is_cdi {
        {
            // Borrow the CD_READ staging buffer for the text: nothing is
            // streaming while an image is being (re)loaded, and it saves
            // keeping a 4 KB buffer around for a file read once per boot.
            let scratch = crate::ide::loader_scratch();
            let buf = &mut scratch[..MAX_GDI_BYTES];
            let len = read_file_to_buf(&image_dir, &image_file, buf)?;
            parse(&buf[..len])?
        }
    } else {
        {
            let cdi_name = image_file.clone();
            info!("cdi: parsing '{}'", show_sfn(&cdi_name));
            let sessions = crate::cdi::parse(&image_dir, &cdi_name)?;
            // Same rule as the GDI path: the High Density Area begins at the
            // first track numbered 3 or above (45000 on a Sega layout, which
            // is exactly where a GD-ROM-shaped CDI puts its second session).
            let hda = tracks()
                .iter()
                .find(|t| t.number >= 3)
                .map(|t| t.start_lba)
                .unwrap_or(45000);
            Gdi {
                boot_area_lba: hda,
                image_dir: image_path.clone(),
                num_blocks,
                is_cdi: true,
                sessions,
            }
        }
    };
    gdi.image_dir = image_path;
    gdi.num_blocks = num_blocks;

    // Walk the image dir and fill in each track's sector_count from the
    // on-disk file size. The closure runs inside iterate_dir's scope and
    // needs the tracks mutably while comparing their filenames, so it goes
    // through a raw pointer to the pool rather than a borrow of it.
    if !from_cdi {
        let tracks_ptr = tracks_mut().as_mut_ptr();
        let tracks_len = tracks().len();
        let _ = image_dir.iterate_dir(|entry| {
            if entry.attributes.is_directory() {
                return;
            }
            for i in 0..tracks_len {
                let t = unsafe { &mut *tracks_ptr.add(i) };
                if entry.name.base_name() == t.filename.base_name()
                    && entry.name.extension() == t.filename.extension()
                {
                    t.sector_count = entry.size / t.sector_size as u32;
                    break;
                }
            }
        });
    }
    // `image_dir` is the root directory, descended in place, so dropping it
    // releases the one slot both used to hold.
    drop(image_dir);
    drop(volume);
    Ok(gdi)
}

/// Bytes of the IP.BIN header the host is given: hardware ID through the
/// 128-byte title (offsets 0x00..0x100 of the boot track's first sector).
pub const IP_HEADER_BYTES: usize = 256;

/// Read the IP.BIN header of `which` without mounting it. Returns the track
/// number the header came from and whether it carries the `SEGA SEGAKATANA`
/// signature.
///
/// The BIOS boots from the first track of the last session: track 3 on a
/// GD-ROM layout, track 2 on a selfboot CDI (whose second session starts
/// at 11702). Rather than reconstruct sessions, try every data track in
/// that order and take the first that is signed; a disc with no signed
/// track yields the first data track's sector unsigned, and `NoIpBin` only
/// when nothing at all could be read.
pub fn read_ip_header(
    num_blocks: u32,
    which: &ImageRef,
    out: &mut [u8; IP_HEADER_BYTES],
) -> Result<(u8, bool), GdiError> {
    let gdi = parse_image(num_blocks, which)?;
    // Static for the same stack reason as `enumerate_images`: the Gdi above
    // is already several KB of this frame.
    static mut RAW: [u8; 16 + IP_HEADER_BYTES] = [0; 16 + IP_HEADER_BYTES];
    let raw = unsafe { &mut *core::ptr::addr_of_mut!(RAW) };
    let mut fallback: Option<u8> = None;
    for pass in 0..2 {
        for t in tracks().iter() {
            let later_session = t.start_lba >= 11700;
            if !t.is_data() || later_session != (pass == 0) {
                continue;
            }
            let off = t.user_data_offset();
            let want = off + IP_HEADER_BYTES;
            match read_at_lba(&gdi, t.start_lba, &mut raw[..want]) {
                Ok(n) if n == want => {}
                _ => continue,
            }
            if &raw[off..off + 16] == b"SEGA SEGAKATANA " {
                out.copy_from_slice(&raw[off..want]);
                return Ok((t.number, true));
            }
            if fallback.is_none() {
                out.copy_from_slice(&raw[off..want]);
                fallback = Some(t.number);
            }
        }
    }
    fallback.map(|n| (n, false)).ok_or(GdiError::NoIpBin)
}

/// Build the block-run map for every track file so the data path can read
/// the card directly. Best effort: a track that cannot be mapped keeps
/// `ext_len == 0` and is served through the filesystem as before.
fn map_track_extents(gdi: &mut Gdi) {
    let pool = unsafe { &mut *core::ptr::addr_of_mut!(EXTENTS) };
    pool.clear();
    for t in tracks_mut().iter_mut() {
        t.ext_start = 0;
        t.ext_len = 0;
    }
    let Some(geom) = crate::fat::probe() else {
        warn!("gdi: no FAT32 geometry; track reads stay on the slow path");
        return;
    };
    let mut dir_cluster = geom.root_cluster;
    for part in gdi.image_dir.iter() {
        let Some(e) = crate::fat::find_in_dir(&geom, dir_cluster, part).filter(|e| e.is_dir) else {
            warn!("gdi: image dir not found in FAT; slow path");
            return;
        };
        // A `..` entry pointing at the root stores cluster 0; the root's own
        // cluster is what that means.
        dir_cluster = if e.first_cluster < 2 { geom.root_cluster } else { e.first_cluster };
    }

    // Read through `tracks()` and write through a *fresh, short-lived*
    // `tracks_mut()` each time. Two `tracks_mut()` calls live at once are two
    // `&mut` to the same static, which is UB -- and which I wrote here first,
    // by rewriting the field accesses mechanically. It compiled and the
    // mapping came out wrong, which costs every read the slow path.
    let n = tracks().len();
    for i in 0..n {
        // CDI: every track shares one file, so reuse its map.
        let shared = (0..i).find(|&j| {
            let t = tracks();
            t[j].ext_len != 0 && t[j].filename == t[i].filename
        });
        if let Some(j) = shared {
            let (start, len) = (tracks()[j].ext_start, tracks()[j].ext_len);
            let t = &mut tracks_mut()[i];
            t.ext_start = start;
            t.ext_len = len;
            continue;
        }

        let filename = tracks()[i].filename.clone();
        let number = tracks()[i].number;
        let Some(entry) = crate::fat::find_in_dir(&geom, dir_cluster, &filename) else {
            warn!("gdi: track {} file missing from FAT", number);
            continue;
        };
        let start = pool.len();
        match crate::fat::map_extents(&geom, &entry, pool) {
            Some(len) => {
                let first = pool[start].lba;
                let t = &mut tracks_mut()[i];
                t.ext_start = start as u16;
                t.ext_len = len as u16;
                info!("gdi: track {} -> {} extent(s) from block {}", number, len, first);
            }
            None => warn!("gdi: track {} unmapped; slow path", number),
        }
    }
}

/// Read `buf.len()` bytes starting at `byte_offset` within the track
/// file for the given LBA. Re-opens the FAT volume on each call —
/// fine for short single-sector probes but too slow for game data.
pub fn read_at_lba(gdi: &Gdi, lba: u32, buf: &mut [u8]) -> Result<usize, GdiError> {
    let (track_filename, byte_offset) = gdi.locate(lba).ok_or(GdiError::LbaOutOfRange)?;

    let dev = HsmciBlockDevice::new();
    dev.set_num_blocks(gdi.num_blocks);
    let mgr = VolumeManager::new(dev, crate::blockdev::DummyClock);
    let volume = mgr
        .open_volume(embedded_sdmmc::VolumeIdx(0))
        .map_err(|_| GdiError::OpenVolume)?;
    let root = volume.open_root_dir().map_err(|_| GdiError::OpenRoot)?;
    let mut dir = root;
    descend(&mut dir, &gdi.image_dir)?;
    let f = dir
        .open_file_in_dir(&track_filename, embedded_sdmmc::Mode::ReadOnly)
        .map_err(|_| GdiError::OpenGdiFile)?;
    f.seek_from_start(byte_offset).map_err(|_| GdiError::ReadGdiFile)?;
    let mut total = 0usize;
    while total < buf.len() {
        let n = f.read(&mut buf[total..]).map_err(|_| GdiError::ReadGdiFile)?;
        if n == 0 {
            break;
        }
        total += n;
    }
    Ok(total)
}



fn find_gdi(dir: &Directory<'_, HsmciBlockDevice, crate::blockdev::DummyClock, 4, 4, 1>)
    -> Result<ShortFileName, GdiError>
{
    let mut found: Option<ShortFileName> = None;
    let _ = dir.iterate_dir(|entry| {
        if found.is_none() && !entry.attributes.is_directory()
            && extension_eq(&entry.name, b"GDI")
        {
            found = Some(entry.name.clone());
        }
    });
    found.ok_or(GdiError::NoGdiFile)
}

fn extension_eq(name: &ShortFileName, ext: &[u8]) -> bool {
    let e = name.extension();
    e.len() == ext.len() && e.iter().zip(ext.iter()).all(|(a, b)| a.eq_ignore_ascii_case(b))
}

fn read_file_to_buf(
    dir: &Directory<'_, HsmciBlockDevice, crate::blockdev::DummyClock, 4, 4, 1>,
    name: &ShortFileName,
    buf: &mut [u8],
) -> Result<usize, GdiError> {
    let f = dir.open_file_in_dir(name, embedded_sdmmc::Mode::ReadOnly)
        .map_err(|_| GdiError::OpenGdiFile)?;
    let mut total = 0usize;
    while total < buf.len() {
        let n = f.read(&mut buf[total..]).map_err(|_| GdiError::ReadGdiFile)?;
        if n == 0 { break; }
        total += n;
    }
    Ok(total)
}

fn parse(data: &[u8]) -> Result<Gdi, GdiError> {
    // Straight into the pool: 99 of these will not fit on the stack, which
    // is the whole reason the pool exists. Cleared first, so a failed parse
    // cannot leave the previous image's tail behind this one's.
    let tracks = tracks_mut();
    tracks.clear();
    let mut lines = SplitLines::new(data);

    // Line 1: number of tracks.
    let header = lines.next().ok_or(GdiError::Parse)?;
    let _expected: u32 = parse_u32(header)?;

    while let Some(line) = lines.next() {
        let trimmed = trim(line);
        if trimmed.is_empty() {
            continue;
        }
        let mut fields = SplitWS::new(trimmed);
        let number = parse_u32(fields.next().ok_or(GdiError::Parse)?)? as u8;
        let start_lba = parse_u32(fields.next().ok_or(GdiError::Parse)?)?;
        let ctrl = parse_u32(fields.next().ok_or(GdiError::Parse)?)? as u8;
        let sector_size = parse_u32(fields.next().ok_or(GdiError::Parse)?)? as u16;
        let filename_raw = fields.next().ok_or(GdiError::Parse)?;
        // The pregap field (last) we ignore.
        let _ = fields.next();

        let filename = sfn_from_bytes(filename_raw)?;
        // sector_count gets filled in after we've parsed the GDI text
        // by walking the image dir and matching each filename to its
        // FAT-reported file size.
        tracks
            .push(Track { number, start_lba, ctrl, sector_size, sector_count: 0, filename,
                          file_offset: 0, ext_start: 0, ext_len: 0 })
            .map_err(|_| GdiError::TooManyTracks)?;
    }

    let boot_area_lba = tracks.iter()
        .find(|t| t.number >= 3)
        .map(|t| t.start_lba)
        .unwrap_or(45000);

    let gdi = Gdi {
        boot_area_lba,
        // Filled in by the caller -- `parse` does not know where the image
        // came from. Empty means "the card root", which is also what a path
        // with no components walks to.
        image_dir: ImagePath::new(),
        num_blocks: 0,
        // This is the `.gdi` text parser; the CDI path builds its own `Gdi`.
        is_cdi: false,
        // One session, which is what stock GDEMU reports even for a
        // dual-area GD-ROM -- and the value this used to send unconditionally.
        sessions: 1,
    };
    info!("gdi: parsed {} tracks, HDA starts at LBA {}", tracks.len(), gdi.boot_area_lba);
    for t in tracks.iter() {
        info!("  track {} lba={} ctrl=0x{:02x} sec={} file={}",
            t.number, t.start_lba, t.ctrl, t.sector_size, show_sfn(&t.filename));
    }
    Ok(gdi)
}

fn parse_u32(s: &[u8]) -> Result<u32, GdiError> {
    let mut n: u32 = 0;
    let mut any = false;
    for &b in s {
        if !b.is_ascii_digit() {
            if any { break; }
            return Err(GdiError::Parse);
        }
        n = n.checked_mul(10).and_then(|x| x.checked_add((b - b'0') as u32))
            .ok_or(GdiError::Parse)?;
        any = true;
    }
    if !any { return Err(GdiError::Parse); }
    Ok(n)
}

fn sfn_from_bytes(raw: &[u8]) -> Result<ShortFileName, GdiError> {
    // Build a trimmed copy in-place. embedded-sdmmc's ShortFileName::create
    // wants a regular filename string.
    let mut buf: String<32> = String::new();
    for &b in raw {
        if b == b'"' { continue; }  // GDI sometimes quotes the filename
        buf.push(b as char).map_err(|_| GdiError::Parse)?;
    }
    ShortFileName::create_from_str(buf.as_str()).map_err(|_| GdiError::BadFilename)
}

fn trim(s: &[u8]) -> &[u8] {
    let mut start = 0;
    while start < s.len() && (s[start] == b' ' || s[start] == b'\t' || s[start] == b'\r') {
        start += 1;
    }
    let mut end = s.len();
    while end > start && (s[end - 1] == b' ' || s[end - 1] == b'\t' || s[end - 1] == b'\r') {
        end -= 1;
    }
    &s[start..end]
}

struct SplitLines<'a> { data: &'a [u8], i: usize }
impl<'a> SplitLines<'a> {
    fn new(data: &'a [u8]) -> Self { Self { data, i: 0 } }
    fn next(&mut self) -> Option<&'a [u8]> {
        if self.i >= self.data.len() { return None; }
        let start = self.i;
        while self.i < self.data.len() && self.data[self.i] != b'\n' {
            self.i += 1;
        }
        let line = &self.data[start..self.i];
        if self.i < self.data.len() { self.i += 1; }
        Some(line)
    }
}

struct SplitWS<'a> { data: &'a [u8], i: usize }
impl<'a> SplitWS<'a> {
    fn new(data: &'a [u8]) -> Self { Self { data, i: 0 } }
    fn next(&mut self) -> Option<&'a [u8]> {
        while self.i < self.data.len() && (self.data[self.i] == b' ' || self.data[self.i] == b'\t') {
            self.i += 1;
        }
        if self.i >= self.data.len() { return None; }
        let start = self.i;
        while self.i < self.data.len() && self.data[self.i] != b' ' && self.data[self.i] != b'\t' {
            self.i += 1;
        }
        Some(&self.data[start..self.i])
    }
}

/// Format a short filename as `BASE.EXT` into a fixed-size buffer
/// suitable for `defmt::info!("{=str}", ..)`.
fn show_sfn(name: &ShortFileName) -> SfnStr {
    let mut out = [b' '; 12];
    let mut i = 0;
    for &b in name.base_name() {
        if i < 8 {
            out[i] = b;
            i += 1;
        }
    }
    let ext = name.extension();
    if !ext.is_empty() {
        out[8] = b'.';
        let n = ext.len().min(3);
        out[9..9 + n].copy_from_slice(&ext[..n]);
    }
    SfnStr(out)
}

struct SfnStr([u8; 12]);

impl Format for SfnStr {
    fn format(&self, f: defmt::Formatter) {
        // Render as a regular ASCII string. ShortFileName bytes are
        // already in the printable range, but be defensive.
        let s = core::str::from_utf8(&self.0).unwrap_or("?");
        defmt::write!(f, "{}", s)
    }
}

#[derive(Debug, Format)]
pub enum GdiError {
    OpenVolume,
    OpenRoot,
    OpenImageDir,
    OpenGdiFile,
    ReadGdiFile,
    LbaOutOfRange,
    NoImageDir,
    NoGdiFile,
    BadCdi,
    Parse,
    BadFilename,
    TooManyTracks,
    NoIpBin,
    Block(BlockError),
}

impl From<BlockError> for GdiError {
    fn from(e: BlockError) -> Self { GdiError::Block(e) }
}
