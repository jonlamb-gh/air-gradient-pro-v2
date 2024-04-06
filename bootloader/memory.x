MEMORY
{
  /* NOTE 1 K = 1 KiBi = 1024 bytes */

  /* Default for the STM32F407ZGT6 */
  /* FLASH : ORIGIN = 0x08000000, LENGTH = 1M */
  /* RAM : ORIGIN = 0x20000000, LENGTH = 112K */

  /* Where the bootloader lives, region1, sectors 0..=3 */
  FLASH                             : ORIGIN = 0x08000000, LENGTH = 64K
  /* State, region2, sector 4 */
  BOOTLOADER_STATE                  : ORIGIN = 0x08010000, LENGTH = 64K
  /* Active/DFU use the remaining sectors in region3, sectors 5..=11 */
  ACTIVE                            : ORIGIN = 0x08020000, LENGTH = 384K
  DFU                               : ORIGIN = 0x08080000, LENGTH = 512K
  RAM                         (rwx) : ORIGIN = 0x20000000, LENGTH = 112K
}

/* NOTE: these are relative to the start of the region they reside in! */
__bootloader_state_start = 0;
__bootloader_state_end = LENGTH(BOOTLOADER_STATE);

__bootloader_active_start = 0;
__bootloader_active_end = LENGTH(ACTIVE);

__bootloader_dfu_start = ORIGIN(DFU) - ORIGIN(ACTIVE);
__bootloader_dfu_end = ORIGIN(DFU) + LENGTH(DFU) - ORIGIN(ACTIVE);
