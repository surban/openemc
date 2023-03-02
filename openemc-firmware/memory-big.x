__flash_page_size = 0x400;

MEMORY
{
  FLASH : ORIGIN = 0x08013000, LENGTH = 0xe000
  RAM   : ORIGIN = 0x20000000, LENGTH = 20K
}

SECTIONS
{
  /* Bootloader info block at start of RAM. */
  .boot_info ORIGIN(RAM) (NOLOAD) : ALIGN(4)
  {
    KEEP(*(.boot_info .boot_info.*));
    . = ALIGN(4);
  } > RAM

  /* Log buffer. */
  .defmt_log (NOLOAD) : ALIGN(4)
  {
    KEEP(*(.defmt_log .defmt_log.*));
    . = ALIGN(4);
  } > RAM
}
INSERT BEFORE .data;
