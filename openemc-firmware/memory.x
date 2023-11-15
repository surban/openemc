
MEMORY
{
  FLASH : ORIGIN = 0x08000000 + __bootloader_max_size, LENGTH = __flash_size - __bootloader_max_size - __cfg_size
  RAM   : ORIGIN = 0x20000000, LENGTH = __ram_size
}

SECTIONS
{
  /* Bootloader info block at start of RAM. */
  .boot_info ORIGIN(RAM) (NOLOAD) : ALIGN(8)
  {
    KEEP(*(.boot_info .boot_info.*));
    . = ALIGN(8);
  } > RAM

  /* Bootloader log buffer. */
  .defmt_boot_log ORIGIN(RAM) + 0x90 (NOLOAD) : ALIGN(8)
  {
    KEEP(*(.defmt_boot_log .defmt_boot_log.*));
    . = ALIGN(8);
  } > RAM

  /* Log buffer. */
  .defmt_log (NOLOAD) : ALIGN(8)
  {
    KEEP(*(.defmt_log .defmt_log.*));
    . = ALIGN(8);
  } > RAM
}
INSERT BEFORE .data;

SECTIONS
{
  .program_end (NOLOAD) : ALIGN(4)
  {
    __flash_program_end = .;
  } > FLASH
}
INSERT AFTER .gnu.sgstubs;
