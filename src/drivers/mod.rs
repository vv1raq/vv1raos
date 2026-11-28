//! Драйверы для Qualcomm SM6375 (Snapdragon 695)

pub mod gcc_sm6375;
pub mod geni_uart;
pub mod gpt;
pub mod msm_uart;
pub mod pinmux;
pub mod pmic_sm6375;
pub mod smmu;
pub mod spmi;
pub mod ufs;
pub mod ufs_phy_qmp_v4;
pub mod vibro;
pub mod clock_verify;

use crate::platform::{driver_register, DriverError};

/// Явная регистрация всех платформенных драйверов.
/// Вызывается из kernel_main до platform_init.
pub fn register_all_drivers() -> Result<(), DriverError> {
    gcc_sm6375::register_driver()?;
    spmi::register_driver()?;
    smmu::register_driver()?;
    pmic_sm6375::register_driver()?;
    msm_uart::register_driver()?;
    ufs::register_driver()?;
    Ok(())
}