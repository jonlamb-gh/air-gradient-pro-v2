use core::fmt;
use embassy_stm32::pac::rcc::Rcc;

#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug, defmt::Format)]
pub enum ResetReason {
    /// Low-power management reset
    LowPowerReset,
    /// The window watchdog triggered
    WindowWatchdogReset,
    /// The independent watchdog triggered
    IndependentWatchdogReset,
    /// The software did a soft reset
    SoftwareReset,
    /// The mcu went from not having power to having power and resetting
    PowerOnReset,
    /// The reset pin was asserted
    PinReset,
    /// The brownout detector triggered
    BrownoutReset,
    /// The reason could not be determined, contains the raw CSR register value
    Unknown(u32),
}

impl ResetReason {
    pub fn read(rcc: &Rcc) -> Self {
        let reason = rcc.csr().read();
        if reason.lpwrrstf() {
            ResetReason::LowPowerReset
        } else if reason.wwdgrstf() {
            ResetReason::WindowWatchdogReset
        } else if reason.wdgrstf() {
            ResetReason::IndependentWatchdogReset
        } else if reason.sftrstf() {
            ResetReason::SoftwareReset
        } else if reason.porrstf() {
            ResetReason::PowerOnReset
        } else if reason.padrstf() {
            ResetReason::PinReset
        } else if reason.borrstf() {
            ResetReason::BrownoutReset
        } else {
            ResetReason::Unknown(reason.0)
        }
    }

    pub fn read_and_clear(rcc: &mut Rcc) -> Self {
        let reason = Self::read(rcc);
        rcc.csr().modify(|reg| reg.set_rmvf(true));
        reason
    }
}

impl fmt::Display for ResetReason {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ResetReason::LowPowerReset => f.write_str("Low-power management reset"),
            ResetReason::WindowWatchdogReset => f.write_str("WWDG reset"),
            ResetReason::IndependentWatchdogReset => f.write_str("IWDG reset"),
            ResetReason::SoftwareReset => f.write_str("Software reset"),
            ResetReason::PowerOnReset => f.write_str("Power-on reset"),
            ResetReason::PinReset => f.write_str("Pin reset (NRST)"),
            ResetReason::BrownoutReset => f.write_str("Brownout reset"),
            ResetReason::Unknown(rcc_csr) => write!(
                f,
                "Could not determine the cause. RCC CSR bits were 0x{:X}",
                rcc_csr
            ),
        }
    }
}
