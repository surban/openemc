SECTIONS
{
  /* Flash space for user program. */
  .user_flash ORIGIN(FLASH) + 0x13000 : ALIGN(__flash_page_size)
  {
    __user_flash_start = .;
  } > FLASH
}
INSERT AFTER .gnu.sgstubs;

SECTIONS
{
  /* Bootloader info block at start of RAM for providing
     information to user program. */
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

__user_flash_end = ORIGIN(FLASH) + LENGTH(FLASH);

