//! `embedded_sdmmc::BlockDevice` adapters.
//!
//! `HsmciBlockDevice` lifts the global HSMCI peripheral up to the
//! `BlockDevice` trait so embedded-sdmmc can read sectors through it.
//!
//! `LbaOffset` wraps any `BlockDevice` and adds a fixed LBA offset to every
//! access. Useful when we know the partition's start LBA out-of-band (e.g.
//! after parsing a GPT) and want embedded-sdmmc to treat the partition as
//! the whole disk. For simple MBR + FAT32 cards we don't need it —
//! embedded-sdmmc parses the MBR directly.

use core::cell::Cell;

use defmt::Format;
use embassy_atsam3::hsmci::{self, CmdError};
use embedded_sdmmc::{Block, BlockCount, BlockDevice, BlockIdx};

#[derive(Debug, Format)]
pub enum BlockError {
    Hsmci(CmdError),
}

impl From<CmdError> for BlockError {
    fn from(value: CmdError) -> Self {
        BlockError::Hsmci(value)
    }
}

/// `BlockDevice` over the SAM3U HSMCI peripheral. Reads happen via
/// [`hsmci::read_block`]; writes are not implemented yet — `write()`
/// returns an error.
///
/// `num_blocks` is reported via [`Cell`] so the embedded-sdmmc trait's
/// `&self` interface can update it from the SD init result. Set with
/// [`HsmciBlockDevice::set_num_blocks`] after the card has been
/// identified.
pub struct HsmciBlockDevice {
    num_blocks: Cell<u32>,
}

impl HsmciBlockDevice {
    pub const fn new() -> Self {
        Self { num_blocks: Cell::new(0) }
    }

    pub fn set_num_blocks(&self, n: u32) {
        self.num_blocks.set(n);
    }
}

impl BlockDevice for HsmciBlockDevice {
    type Error = BlockError;

    fn read(&self, blocks: &mut [Block], start_block_idx: BlockIdx) -> Result<(), Self::Error> {
        for (i, block) in blocks.iter_mut().enumerate() {
            let lba = start_block_idx.0.wrapping_add(i as u32);
            unsafe { hsmci::read_block(lba, &mut block.contents) }?;
        }
        Ok(())
    }

    fn write(&self, _blocks: &[Block], _start_block_idx: BlockIdx) -> Result<(), Self::Error> {
        // CMD24 (WRITE_BLOCK) not implemented yet — read-only for the
        // GD-ROM emulation use case.
        Err(BlockError::Hsmci(CmdError::BadResponse))
    }

    fn num_blocks(&self) -> Result<BlockCount, Self::Error> {
        Ok(BlockCount(self.num_blocks.get()))
    }
}

/// Adapter that adds a fixed LBA offset to every access on the inner
/// device. With this we can hand embedded-sdmmc a "device" that maps
/// LBA 0 to the start of a partition, without embedded-sdmmc doing any
/// partition parsing of its own. Needed for GPT disks (embedded-sdmmc
/// only knows about MBR).
pub struct LbaOffset<B: BlockDevice> {
    inner: B,
    start: u32,
    length: u32,
}

impl<B: BlockDevice> LbaOffset<B> {
    pub fn new(inner: B, start: u32, length: u32) -> Self {
        Self { inner, start, length }
    }
}

impl<B: BlockDevice> BlockDevice for LbaOffset<B> {
    type Error = B::Error;

    fn read(&self, blocks: &mut [Block], start_block_idx: BlockIdx) -> Result<(), Self::Error> {
        let shifted = BlockIdx(self.start.wrapping_add(start_block_idx.0));
        self.inner.read(blocks, shifted)
    }

    fn write(&self, blocks: &[Block], start_block_idx: BlockIdx) -> Result<(), Self::Error> {
        let shifted = BlockIdx(self.start.wrapping_add(start_block_idx.0));
        self.inner.write(blocks, shifted)
    }

    fn num_blocks(&self) -> Result<BlockCount, Self::Error> {
        Ok(BlockCount(self.length))
    }
}

/// Minimal embedded-sdmmc `TimeSource`. We don't have an RTC, so every
/// timestamp is the same fixed value. embedded-sdmmc only uses this when
/// creating/modifying files, which we don't do yet.
pub struct DummyClock;

impl embedded_sdmmc::TimeSource for DummyClock {
    fn get_timestamp(&self) -> embedded_sdmmc::Timestamp {
        embedded_sdmmc::Timestamp {
            year_since_1970: 55, // 2025
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}
