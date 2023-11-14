/* Normal memory configuration with bootloader size 12 kB */

/* Flash size is determined dynamically at runtime;
   the value provided here is just a lower limit.
   The RAM size specified here only applies to the
   bootloader; the user program can use more RAM.   */

MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 16K
  RAM   : ORIGIN = 0x20000000, LENGTH = 10K
}

SECTIONS
{
  /* Flash space for user program. */
  .user_flash ORIGIN(FLASH) + 0x3000 : ALIGN(0x800)
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

