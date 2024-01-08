MEMORY
{
  FLASH         : ORIGIN = 0x08000000, LENGTH = 0x4000
  APP           : ORIGIN = 0x08004000, LENGTH = 0xc000
  PERSIST (rwx) : ORIGIN = 0x20000000, LENGTH = 0x4
  RAM (rwx)     : ORIGIN = 0x20000004, LENGTH = 0x4ffc
}

__persist = ORIGIN(PERSIST);
__app_start = ORIGIN(APP);
__app_end = ORIGIN(APP) + LENGTH(APP);
