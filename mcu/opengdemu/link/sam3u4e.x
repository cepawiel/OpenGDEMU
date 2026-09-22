MEMORY
{
	FLASH : ORIGIN = 0x00080000, LENGTH = 0x00020000 /* Flash0, 128K */
	flash1 (W!RX) : ORIGIN = 0x00100000, LENGTH = 0x00020000 /* Flash1, 128K */
	RAM   : ORIGIN = 0x20000000, LENGTH = 0x00008000 /* Sram0, 32K */
	sram1  (W!RX) : ORIGIN = 0x20080000, LENGTH = 0x00004000 /* Sram1, 16K */
	nfc    (W!RX) : ORIGIN = 0x20100000, LENGTH = 0x00001080 /* NFC SRAM, 4224 */
}

/* SRAM1 is a second, non-contiguous 16K bank (SRAM0 ends at 0x20008000,
 * SRAM1 starts at 0x20080000), so the RAM region cannot simply be made
 * longer. Giving it its own output section lets the big staging buffers move
 * off SRAM0, where they were crowding the stack: statics had grown to 19,904
 * of the 32K bank, leaving under 13K for the stack, and a 4K buffer in a deep
 * FAT call chain was enough to run off the end of it -- which shows up as a
 * HardFault on an undefined instruction inside memset.
 *
 * NOLOAD: nothing zero-initialises this, so only put buffers here that are
 * always written before they are read.
 */
SECTIONS
{
  /* The FPGA bitstream is ~64K of a 128K flash bank, and the bank is not
   * where the code has to live: SAM3U4E has a second 128K plane at 0x00100000
   * that nothing used, so parking the bitstream there roughly halves flash
   * pressure on the code. Not contiguous with FLASH (which ends at 0x000A0000)
   * so, as with SRAM1, it needs its own region rather than a longer one. */
  .flash1 : ALIGN(4)
  {
    KEEP(*(.flash1 .flash1.*));
    . = ALIGN(4);
  } > flash1

  .sram1 (NOLOAD) : ALIGN(4)
  {
    *(.sram1 .sram1.*);
    . = ALIGN(4);
  } > sram1

  /* The NAND flash controller's SRAM, unused by this firmware. Empty on 4E,
   * where SRAM0 is not tight enough to be worth the NOLOAD caveat; it is
   * declared here only so both linker scripts stay the same shape. */
  .nfc (NOLOAD) : ALIGN(4)
  {
    *(.nfc .nfc.*);
    . = ALIGN(4);
  } > nfc
} INSERT AFTER .bss;
