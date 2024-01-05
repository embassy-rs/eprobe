MEMORY
{
  FLASH         : ORIGIN = 0x08000000, LENGTH = 0x3000
  APP           : ORIGIN = 0x08003000, LENGTH = 0xd000
  PERSIST (rwx) : ORIGIN = 0x20000000, LENGTH = 0x4
  RAM (rwx)     : ORIGIN = 0x20000004, LENGTH = 0x4ffc
}

__persist = ORIGIN(PERSIST);
__app_start = ORIGIN(APP);
__app_end = ORIGIN(APP) + LENGTH(APP);
