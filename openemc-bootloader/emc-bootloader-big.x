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
  .boot_info ORIGIN(RAM) (NOLOAD) : ALIGN(4)
  {
    KEEP(*(.boot_info .boot_info.*));
    . = ALIGN(4);
  } > RAM
}
INSERT BEFORE .data;

__user_flash_end = ORIGIN(FLASH) + LENGTH(FLASH);

