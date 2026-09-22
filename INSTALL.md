# INSTALL — SWD probe setup and flashing

This guide gets a stock GDEMU clone board flashed with OpenGDEMU using a
Raspberry Pi Pico as the SWD probe. Nothing here builds from source: you need a
prebuilt firmware `.elf`, a host flashing tool, and a prepared SD card with
`.gdi` images on it.

> **Warning:** flashing overwrites the stock GDEMU firmware and there is no way
> back — the original image is not dumpable. Only do this on a board you are
> willing to lose.

---

## 1. What you need

**Hardware**

| Item | Notes |
|------|-------|
| GDEMU clone board (v5.5 or v5.15b) | v5.5 is preferred — the v5.15b clones drop the H4 UART header, and UART is where the logs come from |
| Raspberry Pi Pico (or Pico 2 / official Debug Probe) | A bare $4 Pico is fine; the official Debug Probe already ships the right firmware |
| Micro-USB cable | Pico → host PC |
| 5–7 jumper wires, female-to-female | To H1 (SWD) and H4 (UART) on the GDEMU board |
| Dreamcast (or a real 5 V supply) | **The board must be powered by the Dreamcast, not by the probe** — see §4 |
| SD card, FAT32 | Where the disc images live |

**Files** (all binaries — nothing is compiled here)

| File | Where from |
|------|------------|
| `debugprobe.uf2` | <https://github.com/raspberrypi/debugprobe/releases> — the **`debugprobe.uf2`** asset for your board (`debugprobe_on_pico.uf2` for a plain Pico, `_on_pico2` for a Pico 2). Version **2.2.0 or newer.** |
| `opengdemu.elf` | The OpenGDEMU firmware ELF. The FPGA bitstream is compressed **inside** this ELF, so there is no separate `.rbf` to program. |
| `*.gdi` / `*.cdi` disc images | Yours. One folder per disc on the card. |

---

## 2. Flash Picoprobe firmware onto the Pico

1. Unplug the Pico.
2. Hold **BOOTSEL** and plug it into USB. A drive named `RPI-RP2` appears.
3. Drag `debugprobe.uf2` onto that drive. The Pico reboots automatically.
4. Confirm it enumerated as a CMSIS-DAP probe:

   ```sh
   lsusb | grep -i 'Debugprobe\|CMSIS'
   ls /dev/serial/by-id/ | grep -i debugprobe
   ```

   The probe is one USB device with two interfaces: interface 0 is the
   CMSIS-DAP debug port (no `/dev` node of its own — openocd and probe-rs talk
   to it directly over USB), interface 1 is a USB-serial adapter for the
   target's UART. So `/dev/serial/by-id/` shows exactly one entry, ending in
   `-if01`, and it is a symlink to `/dev/ttyACM0` on a machine with no other
   ACM devices. That node is the UART in §8.

> **The legacy `picoprobe.uf2` will not work.** It reports a pre-2.2.0
> CMSIS-DAP version and `probe-rs` refuses it with *"the firmware on the probe
> is outdated"*. Use `debugprobe.uf2` from the link above.

---

## 3. Wiring

The GDEMU board's headers have a **white dot next to pin 1**. See
[`docs/PCB.md`](docs/PCB.md) for the photos and the full pinout.

### H1 — SWD (7-pin header)

| H1 pin | Signal | Pico pin (plain Pico running debugprobe) |
|--------|--------|------------------------------------------|
| 1 | 3V3 | **leave unconnected** (see §4) — but it is a handy 3.3 V source for the erase step, §6 |
| 2 | TDI | not used for SWD |
| 3 | TDO / TRACESWO | optional — not used |
| 4 | **TMS / SWDIO** | GP3 (physical pin 5) |
| 5 | **TCK / SWCLK** | GP2 (physical pin 4) |
| 6 | NRST | optional; only needed if you want hardware reset |
| 7 | **GND** | any GND, e.g. physical pin 3 |

Three wires (SWDIO, SWCLK, GND) are the minimum.

### H4 — UART, for firmware logs (3-pin header, v5.5 boards only)

| H4 pin | Signal | Pico pin |
|--------|--------|----------|
| 1 | PA11 — board **RX** | GP4 = probe UART TX (physical pin 6) |
| 2 | PA12 — board **TX** | GP5 = probe UART RX (physical pin 7) |
| 3 | GND | shared with the SWD GND |

Only H4 pin 2 (board TX) is strictly needed — the firmware only prints.

On the **official Raspberry Pi Debug Probe**, use the labelled 3-pin SWD cable
(SWCLK / GND / SWDIO) and the separate UART cable (its TX to H4 pin 1, its RX
to H4 pin 2, GND to H4 pin 3) instead of the GPIO numbers above.

---

## 4. Power — read this before plugging anything in

**Power the GDEMU board from the Dreamcast** (or another solid 5 V rail) and
leave H1 pin 1 (3V3) disconnected from the Pico.

The probe's USB rail is enough to run the MCU and issue SD card *commands*, but
not enough for SD *data* transfers — reads hang with HSMCI's `DTIP` never
asserting, which looks exactly like a broken driver. If SD access starts
hanging, check power first.

Consequence: **SWD is only alive while the Dreamcast is on.** A dead probe
connection usually just means the console is off.

Sequence: connect the probe wires with everything off → plug the Pico into the
host → power on the Dreamcast → flash.

---

## 5. Host tools

Pick one flashing path. `probe-rs` is the shorter one.

### probe-rs (recommended)

Prebuilt binaries, no Rust toolchain needed:

```sh
curl -LsSf https://github.com/probe-rs/probe-rs/releases/latest/download/probe-rs-tools-installer.sh | sh
```

or grab the `probe-rs-tools-*` archive for your platform from
<https://github.com/probe-rs/probe-rs/releases> and put `probe-rs` and
`defmt-print` on your `PATH`.

On Linux, add the udev rules so you don't need `sudo`:

```sh
sudo curl -L https://probe.rs/files/69-probe-rs.rules -o /etc/udev/rules.d/69-probe-rs.rules
sudo udevadm control --reload && sudo udevadm trigger
```

Check the probe and target are both seen:

```sh
probe-rs list
probe-rs read --chip ATSAM3U4E --protocol swd b32 0x400e0740 1
```

The second command reads the SAM3U's CHIPID register. A live, unlocked board
answers with an ID in the `0x281xxxxx` family — `0x28100961` is an ATSAM3U4E
rev 1:

```
400e0740: 28100961
```

> **Don't use `probe-rs info` as the "is it working" test.** It only ever
> auto-detects the target from its chip registry, and it cannot auto-detect
> Atmel/Microchip parts — it ignores `--chip` and fails with *"The connected
> chip could not automatically be determined"* even when SWD is perfectly
> healthy. `probe-rs info --verbose` does work, but it prints two messages that
> look like failures and are not: *"The probe does not support the JTAG
> protocol"* (it tries JTAG first; CMSIS-DAP probes are SWD-only, then it falls
> through to SWD) and *"Debug port version DPv1 does not support SWD
> multidrop. Stopping here"* (the normal end of a **successful** SWD scan).

### OpenOCD (alternative)

Install your distro's `openocd` package (0.11 or newer — it needs the
`cmsis-dap` driver and the `at91sam3u4e` target script, both of which ship with
openocd itself). No GDB needed for flashing.

Check the probe and target come up:

```sh
openocd \
  -f interface/cmsis-dap.cfg \
  -c "transport select swd" \
  -c "adapter speed 1000" \
  -f target/at91sam3u4e.cfg \
  -c "init; targets; exit"
```

You want a line reporting the target `halted` or `running`. Ctrl-C if it doesn't
exit on its own.

---

## 6. Unlock the MCU — erase the stock firmware

**A stock board will not talk to the probe.** The factory GDEMU firmware sets
the SAM3U's flash security bit, which disconnects the debug port entirely.
Until you clear it, every tool fails at the same place:

```
$ probe-rs read --chip ATSAM3U4E --protocol swd b32 0x400e0740 1
Error: The target is not responding / device is locked
```

openocd says much the same — `Cannot halt, device is secured`, or it finds the
DAP but no core. This is not bad wiring; it is the security bit doing its job.

The security bit can only be cleared from *outside* the debug interface, by the
SAM3U's hardware **ERASE** function: hold the ERASE input high while the chip is
powered, and its flash controller wipes the entire flash array and clears the
security bit along with it. The clone boards bring ERASE out as a **bare test
pad next to the MCU** (it is not on any header).

> **This is destructive and irreversible.** It erases the stock GDEMU firmware
> completely, and nobody has a dump of it. Once you do this the board runs
> OpenGDEMU or nothing.

### Procedure

1. Power the board down completely (Dreamcast off).
2. Find the ERASE test pad next to the SAM3U. Tack a wire to it, or hold a
   probe tip on it — it only has to make contact for a moment.
3. Get a 3.3 V source. **H1 pin 1 is 3V3** and is the convenient one; a jumper
   from H1 pin 1 to the pad is the whole circuit. (A ~1 kΩ series resistor is
   cheap insurance against slipping onto a neighbouring pin.)
4. Power the board on (Dreamcast on) **with the pad held at 3.3 V**.
5. Hold it there for **at least half a second** — the datasheet minimum is
   220 ms of high level after reset is released. A couple of seconds is fine.
6. **Remove the wire from the pad**, then power cycle the board.

Step 6 matters: ERASE has an internal pull-down and is sampled at every
power-up, so a wire left attached re-erases the chip on every boot and your
freshly flashed firmware disappears. If a flash "succeeds" but the board comes
up blank after a power cycle, check that the pad is floating.

### Confirm it worked

With the pad disconnected and the board powered:

```sh
probe-rs read --chip ATSAM3U4E --protocol swd b32 0x400e0740 1
```

You should now get a CHIPID back (`400e0740: 28100961`) instead of a lock
error. The flash is
empty at this point — the MCU is running nothing, which is expected. Go to §7.

Do this once per board. Re-flashing later needs no erase: OpenGDEMU does not
set the security bit, so the debug port stays open.

---

## 7. Flash the firmware

### With probe-rs

```sh
probe-rs download --chip ATSAM3U4E --protocol swd --binary-format elf opengdemu.elf
probe-rs verify   --chip ATSAM3U4E --protocol swd opengdemu.elf   # optional
probe-rs reset    --chip ATSAM3U4E --protocol swd
```

`--chip ATSAM3U4E` is required on every one of these: probe-rs cannot work the
part out on its own. Do **not** add `--connect-under-reset` unless you actually
wired H1 pin 6 (NRST) — without it, that flag just times out.

If you run the optional `verify`, run it **between** `download` and `reset`, in
exactly that order. The ELF's `.data` segment is loaded from flash but lives at
`0x20000000` in SRAM, and `verify` compares it at that SRAM address — so once
the firmware is running and has touched its own variables, `verify` reports
*"contents do not match"* on a board that is perfectly programmed.

That is the whole install. The FPGA bitstream is embedded in the ELF and the
MCU configures the Cyclone II over its passive-serial pins at every boot — there
is no separate FPGA programming step and no bitstream file on the SD card.

### With OpenOCD

One command, start to finish — flash, verify, reset, exit:

```sh
openocd \
  -f interface/cmsis-dap.cfg \
  -c "transport select swd" \
  -c "adapter speed 1000" \
  -f target/at91sam3u4e.cfg \
  -c "program opengdemu.elf verify reset exit"
```

If you'd rather drive it step by step, run openocd without the `program` line:

```sh
openocd -f interface/cmsis-dap.cfg -c "transport select swd" \
        -c "adapter speed 1000" -f target/at91sam3u4e.cfg
```

then in a second terminal `telnet localhost 4444` (or `nc localhost 4444`) and
issue:

```
reset halt
flash write_image erase opengdemu.elf
verify_image opengdemu.elf
reset run
```

Notes:

- `adapter speed 1000` (1 MHz) is a conservative value that works over jumper
  wires. Raise it if you want, back it off if you get SWD errors.
- If openocd stops with an IDCODE / TAPID mismatch, add
  `-c "set CPUTAPID 0"` before the target script to skip the check.
- `program … reset exit` leaves the MCU running the new firmware; you do not
  need to power cycle.

---

## 8. Watch the logs

The firmware logs with `defmt` over UART at **115200 8-N-1**, through the
probe's serial bridge. Decoding needs the same ELF you flashed (the strings live
in it, not on the wire):

```sh
defmt-print -e opengdemu.elf serial --path /dev/ttyACM0 --baud 115200
```

If you have other USB-serial devices attached, `/dev/ttyACM0` may be one of
them — use the stable name instead:

```sh
defmt-print -e opengdemu.elf serial \
  --path /dev/serial/by-id/usb-Raspberry_Pi_Debug_Probe_CMSIS-DAP_*-if01 \
  --baud 115200
```

Leave this running in its own terminal — it survives Dreamcast power
cycles and probe disconnects, which is why logging is on UART rather than RTT.

A plain terminal (`picocom`, `minicom`) on the same port shows the boot banner in
readable ASCII; everything after it is encoded defmt frames and needs the tool
above.

---

## 9. Prepare the SD card

1. Format the card **FAT32** (one partition, MBR).
2. One folder per disc at the root, each holding that disc's `.gdi` and its
   track files (or a `.cdi`):

   ```
   /SONICADV/  sonicadv.gdi  track01.bin  track02.raw  track03.bin  …
   /MENU/      disc.gdi      …
   ```

3. Optional `/GDEMU.CFG` at the root picks which disc is mounted at power-on, by
   folder name:

   ```
   image=SONICADV
   ```

   Lines starting with `#` are comments; the key is case-insensitive. Without
   this file the firmware mounts whatever it finds first, and a card with
   nothing usable comes up empty. The selection made at runtime lives in RAM
   only — a power cycle returns to what `GDEMU.CFG` names.

Note the file name is `GDEMU.CFG`, 8.3 — not `OPENGDEMU.CFG`.

---

## 10. Install the board and verify

1. Power everything off. Seat the GDEMU board on the Dreamcast's G1 connector in
   place of the GD-ROM drive, with the probe wires still attached (route them out
   of the shell).
2. Insert the SD card, power on the Dreamcast.
3. In the `defmt-print` terminal you should see the boot banner, the FPGA
   bitstream load, SD card init, and the image enumeration.
4. The Dreamcast should reach the BIOS and boot the selected disc.

---

## 11. Troubleshooting

| Symptom | Likely cause |
|---------|--------------|
| Target locked / secured / not responding, on a board never flashed before | Stock firmware's security bit. Erase the chip via the ERASE pad (§6). |
| Board goes blank after every power cycle despite a successful flash | ERASE wire still attached to the pad (§6). |
| `probe-rs` : *"firmware on the probe is outdated"* | Legacy `picoprobe.uf2` on the Pico. Flash `debugprobe.uf2` ≥ 2.2.0 (§2). |
| No probe listed at all | udev rules not installed, or the Pico is still in BOOTSEL mode. |
| `probe-rs info` : *"the connected chip could not automatically be determined"* | Not a fault. `info` can't auto-detect Atmel parts and ignores `--chip`. Check the link with the CHIPID read in §5 instead. |
| `probe-rs info --verbose` : *"probe does not support the JTAG protocol"* / *"DPv1 does not support SWD multidrop"* | Both benign. The first is `info` trying JTAG before SWD on an SWD-only probe; the second is the normal end of a successful SWD scan. |
| *"Timeout while attaching to target under reset"* | `--connect-under-reset` used without NRST wired (H1 pin 6). Drop the flag. |
| `probe-rs verify` : *"contents do not match"* after a successful flash | `verify` was run after `reset`. It checks `.data` at its SRAM address, which the running firmware has already overwritten. Verify between `download` and `reset` (§7). |
| Probe found, target not found / SWD errors | Dreamcast is off — the MCU is powered from the DC's 5 V rail (§4). Also check SWDIO/SWCLK aren't swapped and GND is shared. |
| Nothing on the UART | v5.15b board (H4 unpopulated), TX/RX swapped, or you're on the wrong `ttyACM` node. |
| Garbage on the UART | Expected — defmt frames. Decode with `defmt-print -e <the exact ELF you flashed>`. |
| Logs stop mid-word / wrong strings | The ELF passed to `defmt-print` isn't the one on the board. |
| SD commands work but reads hang | Powering the board from the probe's USB rail (§4). |
| Dreamcast boots to a black screen | If the console has a dual-BIOS mod set to dcload, that's the BIOS, not the GDEMU. Switch to stock. |

---

## See also

- [`docs/PCB.md`](docs/PCB.md) — full reverse-engineered pinout of the clone boards
- [`docs/Debugging.md`](docs/Debugging.md) — debugging notes
