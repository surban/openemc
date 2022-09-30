MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 64K
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
}
INSERT BEFORE .data;
