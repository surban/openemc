/* Flash size is determined dynamically at runtime;
   the value provided here is just an upper limit.
   The RAM size specified here only applies to the
   bootloader; the user program can use more RAM.   */

MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 128K
  RAM   : ORIGIN = 0x20000000, LENGTH = 10K
}

SECTIONS
{
  /* Pad unused bootloader flash with zeros. */
  .padding :
  {
    FILL(0);
    . = ORIGIN(FLASH) + __bootloader_max_size - 1;
    BYTE(0);
  } > FLASH

  /* Flash space for user program. */
  .user_flash ORIGIN(FLASH) + __bootloader_max_size : ALIGN(0x800)
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
