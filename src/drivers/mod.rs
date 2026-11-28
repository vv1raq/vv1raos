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

/// Записать напрямую в PL011 (QEMU virt) для отладки
fn early_write(msg: &str) {
    const QEMU_PL011: usize = 0x9000000;
    const DR: usize = 0x00;
    const FR: usize = 0x18;
    unsafe {
        for b in msg.bytes() {
            while core::ptr::read_volatile((QEMU_PL011 + FR) as *const u32) & (1 << 5) != 0 {}
            core::ptr::write_volatile((QEMU_PL011 + DR) as *mut u32, b as u32);
        }
    }
}

/// Явная регистрация всех платформенных драйверов.
/// Вызывается из kernel_main до platform_init.
pub fn register_all_drivers() -> Result<(), DriverError> {
    early_write("[DRV_REG] start\n");
    gcc_sm6375::register_driver()?;
    early_write("[DRV_REG] gcc done\n");
    spmi::register_driver()?;
    early_write("[DRV_REG] spmi done\n");
    smmu::register_driver()?;
    early_write("[DRV_REG] smmu done\n");
    pmic_sm6375::register_driver()?;
    early_write("[DRV_REG] pmic done\n");
    msm_uart::register_driver()?;
    early_write("[DRV_REG] msm_uart done\n");
    ufs::register_driver()?;
    early_write("[DRV_REG] ufs done\n");
    Ok(())
}