/* The updater is linked to run from the base of SRAM1 and emitted as a raw
 * binary (--oformat=binary), so the firmware can copy the blob there verbatim
 * and jump to offset 0.
 *
 * SRAM1 rather than SRAM0 because SRAM0 holds the running firmware's .data,
 * .bss and stack -- the updater borrows the caller's stack, so that has to
 * stay intact. SRAM1 holds only ide::CD_READ_BUF, which is dead the moment a
 * firmware update starts.
 *
 * Everything lands in one output section: the blob has to be position-exact
 * and gap-free, since it is copied as a flat byte run. .bss is folded in as
 * ordinary content rather than NOBITS so that zeroed statics are actually
 * present in the image -- nothing zeroes them at load time.
 */
ENTRY(_start)

SECTIONS
{
  . = 0x20080000;

  .updater : ALIGN(4)
  {
    /* _start must be at offset 0: the firmware jumps to the blob's base. */
    KEEP(*(.entry));
    *(.text .text.*);
    *(.rodata .rodata.*);
    *(.data .data.*);
    *(.bss .bss.*);
    *(COMMON);
    . = ALIGN(4);
  }

  /DISCARD/ :
  {
    *(.ARM.exidx*);
    *(.ARM.attributes);
    *(.comment);
    *(.debug*);
  }
}
