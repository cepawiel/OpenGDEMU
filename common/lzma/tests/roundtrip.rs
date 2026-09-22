//! Round-trip the decoder against the real encoder.
//!
//! There is no encoder in this crate — `build.rs` on the firmware side shells
//! out to `xz`, so the tests do the same and check that what `xz` produces is
//! what this decoder consumes. A hand-written range decoder is exactly the
//! kind of code that is subtly wrong on the paths a small input never
//! reaches, so the headline case is the actual 72 KiB FPGA bitstream.

use std::io::Write;
use std::process::{Command, Stdio};

/// Must match `build.rs` and `lzma::DICT_SIZE`.
const XZ_FILTER: &str = "preset=9e,dict=4KiB,lc=0,lp=0,pb=0";

fn compress(data: &[u8]) -> Vec<u8> {
    let mut child = Command::new("xz")
        .args(["--format=raw", &format!("--lzma1={XZ_FILTER}"), "-c"])
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::inherit())
        .spawn()
        .expect("xz must be installed to run these tests");
    child
        .stdin
        .take()
        .unwrap()
        .write_all(data)
        .expect("write to xz");
    let out = child.wait_with_output().expect("xz");
    assert!(out.status.success(), "xz failed");
    out.stdout
}

fn check(original: &[u8]) {
    let packed = compress(original);
    let mut buf = vec![0u8; lzma::SCRATCH_BYTES];
    let scratch = lzma::Scratch::from_buf(&mut buf).expect("scratch");
    let mut dec = lzma::Decoder::new(&packed, original.len(), scratch);

    let got: Vec<u8> = dec.by_ref().collect();
    assert_eq!(dec.error(), None, "decoder reported an error");
    assert_eq!(got.len(), original.len(), "length mismatch");
    if got != original {
        let at = got.iter().zip(original).position(|(a, b)| a != b).unwrap();
        panic!("first mismatch at byte {at}: got {:#04x} want {:#04x}", got[at], original[at]);
    }
}

#[test]
fn empty_and_tiny() {
    check(b"");
    check(b"a");
    check(b"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa");
}

#[test]
fn repetitive() {
    // Long runs and repeats exercise the rep-distance and short-rep paths.
    let mut v = Vec::new();
    for i in 0..4000 {
        v.extend_from_slice(b"the quick brown fox ");
        v.push((i % 251) as u8);
        v.extend_from_slice(&[0u8; 37]);
    }
    check(&v);
}

#[test]
fn incompressible() {
    // A cheap LCG: no structure, so almost every symbol is a literal and the
    // encoder leans on direct bits.
    let mut x: u32 = 0x1234_5678;
    let v: Vec<u8> = (0..20000)
        .map(|_| {
            x = x.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            (x >> 24) as u8
        })
        .collect();
    check(&v);
}

#[test]
fn spans_the_dictionary() {
    // Matches at distances near, at, and past the 4 KiB window boundary.
    let mut x: u32 = 0xC0DE_1234;
    let mut v: Vec<u8> = (0..DICT * 3)
        .map(|_| {
            x = x.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            (x >> 28) as u8
        })
        .collect();
    let head: Vec<u8> = v[..DICT].to_vec();
    v.extend_from_slice(&head);
    check(&v);
}
const DICT: usize = lzma::DICT_SIZE;

#[test]
fn the_actual_bitstream() {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../fpga/Quartus/output_files/OpenGDEMU.rbf"
    );
    let Ok(rbf) = std::fs::read(path) else {
        eprintln!("skipping: {path} not built");
        return;
    };
    check(&rbf);
}
