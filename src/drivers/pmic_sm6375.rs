//! PMIC Driver для Qualcomm SM6375 (PMI8998 / PMI8999 / SMBchg)
//!
//! PMIC подключен по SPMI. Типичные SID (Slave ID):
//! - PMI8998 Primary: SID 0
//! - PMI8998 Secondary: SID 1
//! - SMB1380 Charger: SID 2-3
//!
//! Регистры PMIC (базовые адреса по SPMI):
//! - LDO/BUCK control: 0x1000-0x1FFF
//! - GPIO: 0x2000-0x2FFF
//! - Interrupts: 0x3000-0x3FFF
//! - ADC/Temp: 0x4000-0x4FFF
//! - Revision ID: 0x0000-0x00FF

use crate::drivers::spmi::{spmi_read_reg, spmi_write_reg, spmi_write_burst};
use crate::platform::{self, Driver, DriverError, PlatformDevice, SpinLock};
use core::sync::atomic::{AtomicBool, Ordering};

// PMIC SID (обычно 0 для primary PMIC)
static PMIC_SID: SpinLock<Option<u8>> = SpinLock::new(None);
static PMIC_INIT: AtomicBool = AtomicBool::new(false);

// PMIC регистры (общие для PMI8998/8999)
mod reg {
    // Revision / ID
    pub const REV1: u16 = 0x00;
    pub const REV2: u16 = 0x01;
    pub const REV3: u16 = 0x02;
    pub const REV4: u16 = 0x03;
    pub const SUBTYPE: u16 = 0x04;
    pub const TYPE: u16 = 0x05;

    // LDO/BUCK control base
    pub const LDO_BASE: u16 = 0x1000;
    pub const BUCK_BASE: u16 = 0x1200;
    pub const LDO_STRIDE: u16 = 0x10;
    pub const BUCK_STRIDE: u16 = 0x20;

    // LDO registers (per LDO)
    pub const LDO_EN_CTL: u16 = 0x00;    // enable control
    pub const LDO_VOLTAGE: u16 = 0x04;   // voltage select
    pub const LDO_MODE: u16 = 0x08;      // mode (HPM/LPM)
    pub const LDO_STATUS: u16 = 0x0C;    // status

    // BUCK registers (per BUCK)
    pub const BUCK_EN_CTL: u16 = 0x00;
    pub const BUCK_VOLTAGE: u16 = 0x04;
    pub const BUCK_TARGET: u16 = 0x08;
    pub const BUCK_MODE: u16 = 0x0C;
    pub const BUCK_STATUS: u16 = 0x10;

    // GPIO base
    pub const GPIO_BASE: u16 = 0x2000;
    pub const GPIO_STRIDE: u16 = 0x10;
    pub const GPIO_CTL: u16 = 0x00;
    pub const GPIO_DIR: u16 = 0x04;
    pub const GPIO_PULL: u16 = 0x08;
    pub const GPIO_DRV: u16 = 0x0C;

    // Interrupts
    pub const IRQ_BASE: u16 = 0x3000;
    pub const IRQ_STATUS: u16 = 0x00;
    pub const IRQ_MASK: u16 = 0x04;
    pub const IRQ_CLEAR: u16 = 0x08;
    pub const IRQ_TYPE: u16 = 0x0C;

    // ADC
    pub const ADC_BASE: u16 = 0x4000;
    pub const ADC_CTRL: u16 = 0x00;
    pub const ADC_RESULT: u16 = 0x04;
}

// LDO/BUCK enable биты
mod en {
    pub const EN: u8 = 0x01;
    pub const DIS: u8 = 0x00;
    pub const HPM: u8 = 0x01;  // High Power Mode
    pub const LPM: u8 = 0x00;  // Low Power Mode
}

/// Прочитать PMIC revision
fn pmic_read_rev(sid: u8) -> Result<(u8, u8, u8), DriverError> {
    let rev1 = spmi_read_reg(sid, reg::REV1)?;
    let rev2 = spmi_read_reg(sid, reg::REV2)?;
    let rev3 = spmi_read_reg(sid, reg::REV3)?;
    Ok((rev1, rev2, rev3))
}

/// Получить SID из SpinLock
fn pmic_sid() -> Result<u8, DriverError> {
    let guard = PMIC_SID.lock();
    guard.ok_or(DriverError::InvalidState)
}

/// Включить LDO
pub fn pmic_ldo_enable(ldo_id: u8, enable: bool) -> Result<(), DriverError> {
    let sid = pmic_sid()?;
    let addr = reg::LDO_BASE + (ldo_id as u16 * reg::LDO_STRIDE) + reg::LDO_EN_CTL;
    spmi_write_reg(sid, addr, if enable { en::EN } else { en::DIS })
}

/// Установить напряжение LDO (в мВ, если поддерживается таблицей)
pub fn pmic_ldo_set_voltage(ldo_id: u8, mv: u16) -> Result<(), DriverError> {
    let sid = pmic_sid()?;
    // Напряжение кодируется в register value (зависит от конкретного LDO)
    // Для упрощения: пишем напрямую в мВ / 50 (типичный шаг)
    let val = (mv / 50) as u8;
    let addr = reg::LDO_BASE + (ldo_id as u16 * reg::LDO_STRIDE) + reg::LDO_VOLTAGE;
    spmi_write_reg(sid, addr, val)
}

/// Включить BUCK
pub fn pmic_buck_enable(buck_id: u8, enable: bool) -> Result<(), DriverError> {
    let sid = pmic_sid()?;
    let addr = reg::BUCK_BASE + (buck_id as u16 * reg::BUCK_STRIDE) + reg::BUCK_EN_CTL;
    spmi_write_reg(sid, addr, if enable { en::EN } else { en::DIS })
}

/// Установить напряжение BUCK
pub fn pmic_buck_set_voltage(buck_id: u8, mv: u16) -> Result<(), DriverError> {
    let sid = pmic_sid()?;
    // BUCK voltage: register = (mv - 300) / 12.5 (примерно)
    let val = ((mv.saturating_sub(300)) / 12) as u8;
    let addr = reg::BUCK_BASE + (buck_id as u16 * reg::BUCK_STRIDE) + reg::BUCK_VOLTAGE;
    spmi_write_reg(sid, addr, val)
}

/// Настроить GPIO PMIC
pub fn pmic_gpio_config(gpio: u8, output: bool, pull_up: bool, drive: u8) -> Result<(), DriverError> {
    let sid = pmic_sid()?;
    let base = reg::GPIO_BASE + (gpio as u16 * reg::GPIO_STRIDE);

    // CTL: биты направления, pull, drive strength
    let mut ctl = if output { 0x1 } else { 0x0 };
    if pull_up { ctl |= 0x2; }
    ctl |= (drive & 0x7) << 4;
    spmi_write_reg(sid, base + reg::GPIO_CTL, ctl)?;

    Ok(())
}

/// Прочитать GPIO
pub fn pmic_gpio_read(gpio: u8) -> Result<bool, DriverError> {
    let sid = pmic_sid()?;
    let base = reg::GPIO_BASE + (gpio as u16 * reg::GPIO_STRIDE);
    let val = spmi_read_reg(sid, base + reg::GPIO_CTL)?;
    Ok(val & 0x1 != 0)
}

/// Записать GPIO
pub fn pmic_gpio_write(gpio: u8, high: bool) -> Result<(), DriverError> {
    let sid = pmic_sid()?;
    let base = reg::GPIO_BASE + (gpio as u16 * reg::GPIO_STRIDE);
    let val = if high { 0x1 } else { 0x0 };
    // CTL bit 0 = output value
    let mut cur = spmi_read_reg(sid, base + reg::GPIO_CTL)?;
    cur = (cur & !0x1) | val;
    spmi_write_reg(sid, base + reg::GPIO_CTL, cur)
}

/// Включить все критические реглюляторы для загрузки (early init)
fn pmic_enable_critical_rails() -> Result<(), DriverError> {
    // Типичные critical rails для SM6375 boot:
    // LDO_0..LDO_N, BUCK_0..BUCK_N — зависит от board
    // Здесь включаем базовый набор, реальные ID берутся из DT

    // Пример: LDO 1, 2, 3 (Vdd_aux, Vdd_dsp, Vdd_gpu)
    for ldo in [1, 2, 3] {
        let _ = pmic_ldo_enable(ldo, true);
    }
    // BUCK 0, 1 (Vdd_cpu, Vdd_mem)
    for buck in [0, 1] {
        let _ = pmic_buck_enable(buck, true);
    }

    Ok(())
}

/// Пробирование PMIC драйвера
fn pmic_probe(dev: &PlatformDevice) -> Result<(), DriverError> {
    if PMIC_INIT.swap(true, Ordering::AcqRel) {
        return Err(DriverError::AlreadyExists);
    }

    // SID берется из reg property или power-domains
    let sid = dev.get_resource_mem(0).map(|(b, _)| b as u8).unwrap_or(0);
    *PMIC_SID.lock() = Some(sid);

    // Читаем revision
    match pmic_read_rev(sid) {
        Ok((r1, r2, r3)) => {
            kprintln!("pmic: SID={} rev={}.{}.{}", sid, r1, r2, r3);
        }
        Err(e) => {
            kprintln!("pmic: read rev failed: {}", e);
            return Err(e);
        }
    }

    // Включаем критические rails
    pmic_enable_critical_rails()?;

    // Инициализируем вибромотор (GPIO 13 на PMI8998 — подстроить под board)
    crate::drivers::vibro::vibro_init(13);

    kprintln!("pmic: probe ok");
    Ok(())
}

static PMIC_DRIVER: Driver = Driver {
    name: "pmic-sm6375",
    compatible: ["qcom,pmi8998", "qcom,pmi8999", "qcom,pmic-sm6375", "", "", "", "", ""],
    compatible_n: 3,
    probe: pmic_probe,
    remove: None,
};

/// Регистрация драйвера (явный вызов вместо .init_array)
pub fn register_driver() -> Result<(), DriverError> {
    platform::driver_register(&PMIC_DRIVER)
}

// pub use pmic_ldo_enable;
// pub use pmic_ldo_set_voltage;
// pub use pmic_buck_enable;
// pub use pmic_buck_set_voltage;
// pub use pmic_gpio_config;
// pub use pmic_gpio_read;
// pub use pmic_gpio_write;