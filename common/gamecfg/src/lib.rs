#![no_std]

//! Per-folder game metadata: `GAME.CFG`.
//!
//! Shared between the console and the firmware, which both need to read
//! it and must agree on what it means. The console shows titles and
//! launches programs; the firmware needs `disc` and `set` to know what
//! the front-panel button should swap to, and it needs that while a game
//! is running and the menu is long gone.
//!
//! One of these sits beside the disc images or the program in each folder and
//! says what the entry is. Everything in it is optional -- a folder with a
//! `.gdi` and no `GAME.CFG` keeps working exactly as before, which is the
//! point: the file adds information, it is not a new requirement.
//!
//! ```text
//! title = "Skies of Arcadia (Disc 1)"
//! next  = "Skies of Arcadia Disc 2"
//!
//! # or, for a program rather than a disc:
//! type = "elf"
//! boot = "prog.elf"
//! ```
//!
//! `next` names the folder the front-panel button swaps to. A link rather
//! than a numbering: any folder can point at any other, the two discs of a
//! game point at each other, and nothing has to agree about an ordering or
//! about which set a folder belongs to.
//!
//! **The syntax is a subset of TOML**, so a file written here is valid TOML
//! and an editor will highlight it, but the parser is a few hundred lines
//! rather than a dependency: `key = value`, `#` comments, quoted strings,
//! bare integers, and `[sections]` skipped. YAML was the alternative and is
//! much harder to hand-parse correctly -- its indentation and implicit typing
//! rules are exactly the kind of thing that half-works.
//!
//! **The name is `GAME.CFG` because of 8.3.** `GAME.TOML` has a four
//! character extension, so it only exists on the card under a generated alias
//! like `GAME~1.TOM`, and which alias depends on the tool that wrote the
//! card. The extension has to be three characters for the name to be the same
//! everywhere.

use heapless::String;

/// The file this module looks for, in every image folder.
pub const FILENAME: &str = "GAME.CFG";

/// Longest title kept. Matches the firmware's own limit, so a title that
/// fits one fits the other.
pub const MAX_TITLE: usize = 64;
/// Longest `boot` filename and `next` folder name.
pub const MAX_BOOT: usize = 32;
/// Matches `gdrom::catalog::NAME_LEN`: `next` names another folder, and a
/// value too short to hold one cannot be matched against the catalog.
pub const MAX_NEXT: usize = 96;

/// What kind of entry a folder is.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Kind {
    /// A disc image: the firmware mounts it and the BIOS boots it. What a
    /// folder is when it says nothing.
    #[default]
    Disc,
    /// A program the console loads and runs itself.
    Elf,
}

/// A parsed `GAME.CFG`. Every field is optional; [`Meta::default`] is what a
/// folder without the file means.
#[derive(Debug, Clone, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Meta {
    /// Display name. Falls back to the folder's own name when absent.
    pub title: Option<String<MAX_TITLE>>,
    /// Folder the front-panel button swaps to, by name.
    ///
    /// A link, not a position: disc one points at disc two and disc two
    /// points back, so the button walks the pair without anything needing to
    /// know how many discs there are or what order they go in. Anything may
    /// point at anything.
    pub next: Option<String<MAX_NEXT>>,
    pub kind: Kind,
    /// For `type = "elf"`, the file to load. When absent the loader takes
    /// the only `.elf` in the folder.
    pub boot: Option<String<MAX_BOOT>>,
}

impl Meta {
    /// Parse the contents of a `GAME.CFG`.
    ///
    /// Never fails. A line that does not parse is skipped: the file is
    /// hand-edited on a PC, and refusing to show a game because someone
    /// fat-fingered a quote would be the wrong trade. Anything genuinely
    /// needed has a fallback.
    pub fn parse(text: &str) -> Self {
        let mut out = Meta::default();
        for line in text.lines() {
            let line = strip_comment(line).trim();
            if line.is_empty() || line.starts_with('[') {
                continue;
            }
            let Some((key, value)) = line.split_once('=') else {
                continue;
            };
            let key = key.trim();
            let value = value.trim();

            match key {
                "title" => out.title = unquote(value).and_then(fit),
                "boot" => out.boot = unquote(value).and_then(fit),
                // `next_disk` accepted too: it is the obvious other name for
                // this and costs one line to not be wrong about.
                "next" | "next_disk" | "next_disc" => out.next = unquote(value).and_then(fit),
                "type" => {
                    if let Some(v) = unquote(value) {
                        out.kind = match v {
                            "elf" | "bin" | "prog" => Kind::Elf,
                            _ => Kind::Disc,
                        };
                    }
                }
                _ => {}
            }
        }
        out
    }
}

/// Drop a trailing `# comment`, but not a `#` inside quotes.
fn strip_comment(line: &str) -> &str {
    let mut in_quotes = false;
    for (i, c) in line.char_indices() {
        match c {
            '"' => in_quotes = !in_quotes,
            '#' if !in_quotes => return &line[..i],
            _ => {}
        }
    }
    line
}

/// Take the inside of a quoted string, or a bare word as-is.
fn unquote(v: &str) -> Option<&str> {
    let v = v.trim();
    if v.len() >= 2 && v.starts_with('"') && v.ends_with('"') {
        Some(&v[1..v.len() - 1])
    } else if v.is_empty() || v.starts_with('"') {
        // An unterminated quote is a typo, not a value.
        None
    } else {
        Some(v)
    }
}

/// Truncate rather than reject: a title too long for the buffer is still a
/// better label than the folder's 8.3 name.
fn fit<const N: usize>(s: &str) -> Option<String<N>> {
    let mut out: String<N> = String::new();
    for c in s.chars() {
        if out.push(c).is_err() {
            break;
        }
    }
    Some(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reads_the_documented_example() {
        let m = Meta::parse(
            "# a game
title = \"Skies of Arcadia (Disc 1)\"
next = \"Skies of Arcadia Disc 2\"
",
        );
        assert_eq!(m.title.as_deref(), Some("Skies of Arcadia (Disc 1)"));
        assert_eq!(m.next.as_deref(), Some("Skies of Arcadia Disc 2"));
        assert_eq!(m.kind, Kind::Disc);
    }

    #[test]
    fn reads_a_program_entry() {
        let m = Meta::parse("type = \"elf\"\nboot = \"prog.elf\"\n");
        assert_eq!(m.kind, Kind::Elf);
        assert_eq!(m.boot.as_deref(), Some("prog.elf"));
    }

    #[test]
    fn a_missing_file_is_a_plain_disc() {
        let m = Meta::default();
        assert_eq!(m.kind, Kind::Disc);
        assert!(m.title.is_none());
        assert!(m.next.is_none());
    }

    #[test]
    fn survives_junk() {
        let m = Meta::parse("[meta]\nnonsense\ntitle = \"ok\"\nnext = \n= 3\n");
        assert_eq!(m.title.as_deref(), Some("ok"));
        assert_eq!(m.next, None);
    }

    #[test]
    fn hash_inside_quotes_is_not_a_comment() {
        let m = Meta::parse("title = \"Hash # Game\" # real comment\n");
        assert_eq!(m.title.as_deref(), Some("Hash # Game"));
    }
}

/// Resolve a `next` name to an index in the catalog.
///
/// `names` yields each entry's folder name in catalog order. Matching is
/// case-insensitive and ignores a trailing slash, because the name is typed
/// by hand on a PC and "Skies of Arcadia Disc 2/" is the same folder.
///
/// Returns `None` when nothing matches, which is the right answer for a
/// typo: the button does nothing rather than swapping to a surprise.
pub fn resolve<'a>(names: impl Iterator<Item = &'a str>, want: &str) -> Option<usize> {
    let want = want.trim().trim_end_matches('/');
    names.enumerate().find_map(|(i, n)| {
        (n.len() == want.len()
            && n.chars()
                .zip(want.chars())
                .all(|(a, b)| a.eq_ignore_ascii_case(&b)))
        .then_some(i)
    })
}

#[cfg(test)]
mod link_tests {
    use super::*;

    const NAMES: &[&str] = &["LAUNCH", "Skies of Arcadia", "Skies of Arcadia Disc 2"];

    #[test]
    fn finds_a_folder_by_name() {
        assert_eq!(resolve(NAMES.iter().copied(), "Skies of Arcadia Disc 2"), Some(2));
    }

    #[test]
    fn ignores_case_and_a_trailing_slash() {
        assert_eq!(resolve(NAMES.iter().copied(), "skies of arcadia"), Some(1));
        assert_eq!(resolve(NAMES.iter().copied(), "LAUNCH/"), Some(0));
    }

    #[test]
    fn a_typo_resolves_to_nothing() {
        assert_eq!(resolve(NAMES.iter().copied(), "Skies of Arcadia Disc 3"), None);
        // A prefix is not a match: it would link to the wrong disc.
        assert_eq!(resolve(NAMES.iter().copied(), "Skies of Arcadia D"), None);
    }

    #[test]
    fn parses_the_link_and_its_aliases() {
        for key in ["next", "next_disk", "next_disc"] {
            let m = Meta::parse(&alloc_line(key));
            assert_eq!(m.next.as_deref(), Some("Disc 2"), "{key}");
        }
    }

    fn alloc_line(key: &str) -> heapless::String<64> {
        let mut s: heapless::String<64> = heapless::String::new();
        s.push_str(key).unwrap();
        s.push_str(" = \"Disc 2\"").unwrap();
        s
    }
}
