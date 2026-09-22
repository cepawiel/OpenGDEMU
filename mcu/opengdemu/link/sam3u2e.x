MEMORY
{
	/* SAM3U2E has a single 128K flash plane -- there is no 0x00100000
	 * plane to park the bitstream in, so it shares FLASH with the code.
	 * That is the whole reason this part is tight: 4E spends 128K on code
	 * and a second 128K on the bitstream; 2E has to fit both in one bank. */
	FLASH : ORIGIN = 0x00080000, LENGTH = 0x00020000 /* Flash0, 128K */
	RAM   : ORIGIN = 0x20000000, LENGTH = 0x00004000 /* Sram0, 16K */
	sram1 (W!RX) : ORIGIN = 0x20080000, LENGTH = 0x00004000 /* Sram1, 16K */
	nfc   (W!RX) : ORIGIN = 0x20100000, LENGTH = 0x00001080 /* NFC SRAM, 4224 */
}

/* See link/sam3u4e.x for what each of these sections is for; the only
 * difference here is that .flash1 has no second plane to go to. */
SECTIONS
{
  .flash1 : ALIGN(4)
  {
    KEEP(*(.flash1 .flash1.*));
    . = ALIGN(4);
  } > FLASH

  .sram1 (NOLOAD) : ALIGN(4)
  {
    *(.sram1 .sram1.*);
    . = ALIGN(4);
  } > sram1

  /* The NAND flash controller's SRAM. Nothing in this firmware drives the
   * NFC, so on 2E its 4,224 bytes are the one block of RAM available for
   * free -- which is exactly enough for cdda::RING. NOLOAD, so anything
   * put here must be written before it is read (cdda::init zeroes it). */
  .nfc (NOLOAD) : ALIGN(4)
  {
    *(.nfc .nfc.*);
    . = ALIGN(4);
  } > nfc
} INSERT AFTER .bss;
