MEMORY
{
  /* NOTE 1 K = 1 KiBi = 1024 bytes */

  /* Default for the STM32F407ZGT6 */
  /* FLASH : ORIGIN = 0x08000000, LENGTH = 1M */
  /* RAM : ORIGIN = 0x20000000, LENGTH = 112K */

  /* Where the bootloader lives, region1, sectors 0..=3 */
  BOOTLOADER                        : ORIGIN = 0x08000000, LENGTH = 64K
  /* State, region2, sector 4 */
  BOOTLOADER_STATE                  : ORIGIN = 0x08010000, LENGTH = 64K
  /* Active/DFU use the remaining sectors in region3, sectors 5..=11 */
  FLASH                             : ORIGIN = 0x08020000, LENGTH = 384K
  DFU                               : ORIGIN = 0x08080000, LENGTH = 512K
  RAM                         (rwx) : ORIGIN = 0x20000000, LENGTH = 111K
  PANIC_MSG                   (rwx) : ORIGIN = 0x2001BC00, LENGTH = 1K
}

/* NOTE: these are relative to the start of the region they reside in! */
__bootloader_state_start = 0;
__bootloader_state_end = LENGTH(BOOTLOADER_STATE);

__bootloader_dfu_start = ORIGIN(DFU) - ORIGIN(FLASH);
__bootloader_dfu_end = ORIGIN(DFU) + LENGTH(DFU) - ORIGIN(FLASH);

_panic_dump_start = ORIGIN(PANIC_MSG);
_panic_dump_end   = ORIGIN(PANIC_MSG) + LENGTH(PANIC_MSG);
