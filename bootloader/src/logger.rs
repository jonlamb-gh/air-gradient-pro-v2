use core::cell::RefCell;
use cortex_m::interrupt::{self, Mutex};
use embassy_stm32::{
    mode::Blocking,
    peripherals::USART6,
    rcc,
    usart::{BasicInstance, UartTx},
};
use embedded_io::Write as IoWrite;
use log::{Metadata, Record};
use static_cell::StaticCell;

type Inner<T> = Mutex<RefCell<UartTx<'static, T, Blocking>>>;
struct Logger<T: BasicInstance>(Inner<T>);
type LoggerUsart6 = Logger<USART6>;

static LOGGER: StaticCell<LoggerUsart6> = StaticCell::new();

pub(crate) unsafe fn init_logger(tx: UartTx<'static, USART6, Blocking>) {
    let logger = LOGGER.init(Logger(Mutex::new(RefCell::new(tx))));
    log::set_logger(logger)
        .map(|_| log::set_max_level(log::LevelFilter::Trace))
        .unwrap();
}

// SAFETY: don't use any logging after this, should be called before
// jumping to firmware
pub(crate) unsafe fn flush_disable_logger() {
    let logger = log::logger();
    logger.flush();
    log::set_max_level(log::LevelFilter::Off);

    // Re-construct so we can call Drop and disable the peripheral
    // Disable the peripheral
    let uart = embassy_stm32::pac::USART6;
    uart.cr1().write(|w| {
        w.set_ue(false);
        w.set_te(false);
        w.set_re(false);
    });
    rcc::disable::<USART6>();
}

impl<T> log::Log for Logger<T>
where
    T: BasicInstance,
{
    fn enabled(&self, metadata: &Metadata) -> bool {
        metadata.level() <= log::max_level()
    }

    fn log(&self, record: &Record) {
        if self.enabled(record.metadata()) {
            let _ = interrupt::free(|cs| {
                writeln!(
                    self.0.borrow(cs).borrow_mut(),
                    "{}{}",
                    level_marker(record.level()),
                    record.args()
                )
            })
            .ok();
        }
    }

    fn flush(&self) {
        let _ = interrupt::free(|cs| self.0.borrow(cs).borrow_mut().blocking_flush()).ok();
    }
}

const fn level_marker(level: log::Level) -> &'static str {
    use log::Level::*;
    match level {
        Error => "[E] ",
        Warn => "[W] ",
        Info => "",
        Debug => "[D] ",
        Trace => "[T] ",
    }
}
