#![no_std]
#![no_main]

#[cfg(feature = "debug")]
use defmt_rtt as _;

use core::cell::RefCell;
use cortex_m_rt::{entry, exception};
use embassy_boot_stm32::*;
use embassy_stm32::flash::{self, Flash, BANK1_REGION3};
use embassy_sync::blocking_mutex::Mutex;

macro_rules! debug {
      ($s:literal $(, $x:expr)* $(,)?) => {
          {
              #[cfg(feature = "debug")]
              ::defmt::debug!($s $(, $x)*);
              #[cfg(not(feature = "debug"))]
              let _ = ($( & $x ),*);
          }
      };
  }

#[entry]
fn main() -> ! {
    let p = embassy_stm32::init(Default::default());

    // Uncomment this if you are debugging the bootloader with debugger/RTT attached,
    // as it prevents a hard fault when accessing flash 'too early' after boot.
    #[cfg(feature = "debug")]
    {
        for _ in 0..10000000 {
            cortex_m::asm::nop();
        }
    }

    debug!("FLASH default layout: {}", flash::is_default_layout());

    let layout = Flash::new_blocking(p.FLASH).into_blocking_regions();
    let flash_region2 = Mutex::new(RefCell::new(layout.bank1_region2));
    let flash_region3 = Mutex::new(RefCell::new(layout.bank1_region3));

    let config =
        BootLoaderConfig::from_linkerfile_blocking(&flash_region3, &flash_region3, &flash_region2);

    let active_offset = config.active.offset();
    debug!("active_offset = 0x:{:X}", active_offset);

    let bl = BootLoader::prepare::<_, _, _, 1024>(config);

    unsafe { bl.load(BANK1_REGION3.base + active_offset) }

    // TODO just for testing without the firmware updater in the mix
    /*
    unsafe {
        let mut p = cortex_m::Peripherals::steal();
        p.SCB.invalidate_icache();
        p.SCB.vtor.write(BANK1_REGION3.base);
        cortex_m::asm::bootload(BANK1_REGION3.base as *const u32)
    }
    */
}

#[no_mangle]
#[cfg_attr(target_os = "none", link_section = ".HardFault.user")]
unsafe extern "C" fn HardFault() {
    #[cfg(feature = "debug")]
    {
        defmt::error!("HardFault");
    }
    cortex_m::peripheral::SCB::sys_reset();
}

#[exception]
unsafe fn DefaultHandler(_: i16) -> ! {
    const SCB_ICSR: *const u32 = 0xE000_ED04 as *const u32;
    let irqn = core::ptr::read_volatile(SCB_ICSR) as u8 as i16 - 16;

    panic!("DefaultHandler #{:?}", irqn);
}

#[allow(unused_variables)]
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    #[cfg(feature = "debug")]
    {
        defmt::error!("panic: {}", defmt::Display2Format(info));
    }
    cortex_m::asm::udf();
}
