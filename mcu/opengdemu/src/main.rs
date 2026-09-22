#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::{Spawner, Executor};
use embassy_time::{Duration, Timer};
// use gpio::{Level, Output};
use defmt_rtt as _;
use panic_probe as _;

use embassy_atsam3::{config::Config, gpio::{Output, Level}, smc, UserPeripherals};
use core::ptr::read_volatile;

use cortex_m_rt::{entry, exception};
use static_cell::StaticCell;

use embedded_hal::digital::v2::ToggleableOutputPin;

mod blockdev;
mod button;
mod cdda;
mod cyclone;
mod cdi;
mod fat;
mod fwupdate;
mod gdi;
mod ide;
mod sdcard;

static EXECUTOR0: StaticCell<Executor> = StaticCell::new();

#[exception]
unsafe fn DefaultHandler(irqn: i16) {
    cortex_m::asm::bkpt();
    defmt::panic!("Unhandled exception (IRQn = {})", irqn);
    loop{}
}

#[entry]
fn main() -> ! {
    info!("CPU Booted!!!\n");

    let config = Config::new(12_000_000, 8, 1);

    let p = embassy_atsam3::init(config);
   
    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(|spawner| {
        spawner.spawn(main_task(spawner, p).unwrap());
    })
    
}

/// FPGA bitstream, LZMA-compressed by `build.rs` and parked in the second
/// flash plane so it does not crowd the code out of the first. Read as
/// ordinary memory-mapped flash at run time and unpacked straight into the
/// passive-serial loader — see `load_fpga`.
#[link_section = ".flash1"]
static BITSTREAM_LZMA: [u8; include_bytes!(concat!(env!("OUT_DIR"), "/bitstream.lzma")).len()] =
    *include_bytes!(concat!(env!("OUT_DIR"), "/bitstream.lzma"));

// `BITSTREAM_RAW_LEN`: the decompressed length, which the raw LZMA1 stream
// does not carry.
include!(concat!(env!("OUT_DIR"), "/bitstream_len.rs"));

#[embassy_executor::task]
async fn main_task(spawner: Spawner, p: UserPeripherals) {
    // Load the FPGA first. When the GDEMU board is powered by the DC
    // every millisecond between MCU power-on and an alive FPGA is one
    // more millisecond the DC sees a dead GD-ROM bus and CDCLK silent.
    // Don't introduce *anything* (LED setup, log banner, timers) before
    // this point that we don't strictly need.
    let t_fpga = embassy_time::Instant::now();
    let mut fpga = cyclone::PSLoader::new(
        p.nconfig,
        p.clk,
        p.data,
        p.nstatus,
        p.done,
    );
    // The decoder's working memory (a 4 KiB window plus the probability
    // model, ~7.6 KB) is carved out of the CD_READ staging buffer. That
    // buffer is 14 KB of SRAM1 that nothing touches until the first CD_READ,
    // which cannot arrive before the FPGA is even configured — so the
    // compression costs no static RAM at all.
    let scratch = lzma::Scratch::from_buf(ide::loader_scratch())
        .expect("CD_READ_BUF too small for the LZMA scratch");
    let mut dec = lzma::Decoder::new(&BITSTREAM_LZMA, BITSTREAM_RAW_LEN, scratch);
    fpga.load_from(&mut dec).await;
    // The PS loader stops early once CONF_DONE rises, so a short decode is
    // only a problem if the decoder itself failed.
    if dec.error().is_some() {
        defmt::panic!(
            "bitstream decode failed after {} of {} bytes",
            dec.produced(),
            BITSTREAM_RAW_LEN
        );
    }

    let mut led = Output::new(p.led, Level::High);
    // Worth watching: LZMA decoding is interleaved with the bit-bang, so it
    // adds directly to the window where the DC sees a dead GD-ROM bus.
    info!("FPGA Loaded! ({} ms)", t_fpga.elapsed().as_millis());

    // Cyclone II runs internal init after CONF_DONE rises before user
    // logic is live. Without this slack, the post-load probe floats and
    // returns 0xffff.
    Timer::after(Duration::from_millis(10)).await;

    // Quick FPGA bus sanity check — FPGA[12] should read 0xC0DE on both
    // devices. Skipping the verbose register/FIFO sweep we used to do here
    // (regression-tested in Amaranth sim).
    let fpga = smc::fpga_window();
    unsafe {
        let p = read_volatile(fpga.add(12));
        let s = read_volatile(fpga.add(0x80 + 12));
        info!("FPGA debug: primary={:#06x} secondary={:#06x}", p, s);
    }

    // SD card init + FAT32 + GDI parse first, so the IDE task can
    // serve real game data on its first command. We delay the IDE task
    // spawn until the GDI is ready so it can take a `&'static Gdi`.
    match sdcard::init().await {
        Ok(card) => {
            let mb = (card.size_blocks as u64) * 512 / 1_048_576;
            let gb = mb / 1024;
            info!(
                "Card up: {:?}, RCA={:#06x}, {} blocks (~{} GB / {} MB) [CSD-derived]",
                card.capacity, card.rca, card.size_blocks, gb, mb
            );

            #[cfg(feature = "sd-bench")]
            sdcard::bench_read(1_000_000).await;

            // The emulated disk on the secondary exposes the whole card,
            // MBR included, so the host sees a normal partitioned disk and
            // can reach the same filesystem that holds the GD-ROM images.
            ide::size_disk_from_mbr(card.size_blocks);

            let dev = blockdev::HsmciBlockDevice::new();
            dev.set_num_blocks(card.size_blocks);
            list_root_dir(dev);

            match gdi::load_default(card.size_blocks) {
                Ok(g) => ide::mount_gdi(g),
                Err(e) => warn!("gdi load failed: {:?}", e),
            }
            // Pre-populate the CD_READ buffer with IP.BIN before the BIOS
            // can issue its first CD_READ. Otherwise the BIOS triggers its
            // DMA channel ~50ms before our SD read finishes and the host
            // times out around 10-11 KB into the 14 KB transfer.
            ide::prefetch_mounted().await;
            // PA22 is the FPGA's MCU_IRQn line. Configure it for a
            // falling-edge IRQ then spawn the (single) IDE task that
            // services both emulated devices when the line drops.
            unsafe {
                embassy_atsam3::gpio::enable_pioa_falling_irq(22);
            }
            // CDDA: the SysTick hook feeds the FPGA from a ring the task
            // below keeps filled from the card.
            cdda::init();
            spawner.spawn(cdda::cdda_task().unwrap());
            // The front-panel button: swaps to the next disc of a
            // multi-disc game, which only the firmware can do once the game
            // has replaced the menu.
            button::init();
            spawner.spawn(button::button_task().unwrap());
            spawner.spawn(ide::ide_task(
                ide::IdeDevice::primary(),
                ide::IdeDevice::secondary(),
            ).unwrap());
        }
        Err(e) => {
            warn!("SD init failed: {:?}", e);
            unsafe {
                embassy_atsam3::gpio::enable_pioa_falling_irq(22);
            }
            spawner.spawn(ide::ide_task(
                ide::IdeDevice::primary(),
                ide::IdeDevice::secondary(),
            ).unwrap());
        }
    }

    let fpga = smc::fpga_window();
    // Only read by the `fpga-diag` log below, so they are only declared when
    // it is: without the cfg they are four "assigned but never used" warnings
    // in every ordinary build, and deleting them instead would quietly break
    // the diagnostic's deltas.
    #[cfg(feature = "fpga-diag")]
    let (mut prev_free, mut prev_strobes, mut prev_rstn_edges, mut prev_audio) =
        (0u16, 0u16, 0u16, 0u16);
    loop {
        Timer::after(Duration::from_secs(1)).await;
        led.toggle().unwrap();
        let (_free_run, _strobes, _rstn_edges, _bus_state, _audio_alive) = unsafe {
            (
                read_volatile(fpga.add(14)),
                read_volatile(fpga.add(15)),
                read_volatile(fpga.add(16)),
                read_volatile(fpga.add(17)),
                read_volatile(fpga.add(18)),
            )
        };
        // free_run delta = nonzero proves FPGA core clock is alive.
        // strobes delta = nonzero proves DC is touching the IDE bus.
        // rstn_edges nonzero = DC has issued at least one reset cycle.
        // bus_state bit 0 = ide_rstn level (1 = out of reset).
        // audio_alive delta = nonzero proves the 11.2896 MHz xtal is
        // running and routes into the FPGA fabric (≈344 edges/sec
        // expected at audio_div[15]).
        //
        // Off by default. info! is unsynchronized, so this once-a-
        // second line interleaves mid-word with the IDE task's per-command
        // tracing and shreds both. Enable `fpga-diag` only when the point of
        // the run is FPGA liveness rather than ATAPI behaviour.
        #[cfg(feature = "fpga-diag")]
        info!(
            "diag: free_run=0x{:04x} (d={}) strobes={} (d={}) rstn_edges=0x{:04x} (d=0x{:04x}) bus=0x{:02x} audio=0x{:04x} (d={})",
            free_run,
            free_run.wrapping_sub(prev_free),
            strobes,
            strobes.wrapping_sub(prev_strobes),
            rstn_edges,
            rstn_edges.wrapping_sub(prev_rstn_edges),
            bus_state & 0xFF,
            audio_alive,
            audio_alive.wrapping_sub(prev_audio),
        );
        #[cfg(feature = "fpga-diag")]
        {
            prev_free = free_run;
            prev_strobes = strobes;
            prev_rstn_edges = rstn_edges;
            prev_audio = audio_alive;
        }
    }
}

/// Open MBR partition 0, walk the root directory, log every entry.
/// Lives in its own fn so embedded-sdmmc's `Volume<'_, ...>` borrow of the
/// VolumeManager doesn't leak into `main_task`'s drop scope.
fn list_root_dir(dev: blockdev::HsmciBlockDevice) {
    let mgr = embedded_sdmmc::VolumeManager::new(dev, blockdev::DummyClock);
    let volume = match mgr.open_volume(embedded_sdmmc::VolumeIdx(0)) {
        Ok(v) => v,
        Err(e) => {
            warn!("open_volume(0) failed: {:?}", defmt::Debug2Format(&e));
            return;
        }
    };
    info!("opened MBR volume 0");

    let root = match volume.open_root_dir() {
        Ok(r) => r,
        Err(e) => {
            warn!("open_root_dir failed: {:?}", defmt::Debug2Format(&e));
            return;
        }
    };
    info!("--- / ---");
    // First pass: log every entry and remember the first sub-directory's
    // ShortFileName so we can recurse after `iterate_dir` releases its lock.
    let mut subdir: Option<embedded_sdmmc::ShortFileName> = None;
    let _ = root.iterate_dir(|entry| {
        log_entry("/", entry);
        if subdir.is_none()
            && entry.attributes.is_directory()
            && entry.name.base_name() != b"."
            && entry.name.base_name() != b".."
        {
            subdir = Some(entry.name.clone());
        }
    });

    if let Some(name) = subdir {
        match root.open_dir(&name) {
            Ok(sub) => {
                let label = sfn_label(&name);
                info!("--- /{}/ ---", core::str::from_utf8(&label).unwrap_or("?"));
                let _ = sub.iterate_dir(|child| {
                    log_entry(core::str::from_utf8(&label).unwrap_or("?"), child);
                });
            }
            Err(e) => warn!("open_dir failed: {:?}", defmt::Debug2Format(&e)),
        }
    }
}

fn sfn_label(name: &embedded_sdmmc::ShortFileName) -> [u8; 12] {
    let mut out = [b' '; 12];
    let raw = name.base_name();
    let n = raw.len().min(8);
    out[..n].copy_from_slice(&raw[..n]);
    out
}

fn log_entry(parent: &str, entry: &embedded_sdmmc::DirEntry) {
    let mut name = [b' '; 12];
    let raw = entry.name.base_name();
    let n = raw.len().min(8);
    name[..n].copy_from_slice(&raw[..n]);
    let ext = entry.name.extension();
    if !ext.is_empty() {
        name[8] = b'.';
        let m = ext.len().min(3);
        name[9..9 + m].copy_from_slice(&ext[..m]);
    }
    let name_str = core::str::from_utf8(&name).unwrap_or("?");
    info!(
        "{}/{}  {} bytes  dir={}",
        parent,
        name_str,
        entry.size,
        entry.attributes.is_directory()
    );
}