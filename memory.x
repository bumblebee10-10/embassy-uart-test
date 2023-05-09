MEMORY
{
  BOOT2 : ORIGIN = 0x10000000, LENGTH = 0x100
  FLASH(rx) : ORIGIN = 0x10000100, LENGTH = 2048k - 0x100
  RAM(rwx) : ORIGIN =  0x20000000, LENGTH = 264k
}
SECTIONS {
  .boot_loader ORIGIN(BOOT2) :
  {
    KEEP(*(.boot_loader*));
  } > BOOT2

} INSERT BEFORE .text;

