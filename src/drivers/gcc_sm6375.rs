//! GCC SM6375 (Blair) — Global Clock Controller для Snapdragon 695 (SM6375)
//!
//! Базовый адрес берётся из DT (reg property): обычно 0x1400000 (blair-gccsyscon)
//! Регистры: CLK_ENABLE, CLK_DISABLE, CLK_STATUS, CLK_FREQ, CLK_DIV, PLL_MODE, BCMD
//!
//! Источник: Linux drivers/clk/qcom/gcc-sm6375.c + Technical Reference Manual

use crate::platform::{self, Driver, DriverError, PlatformDevice, ProbeStatus, Resource};
use crate::sync::SpinLock;
use core::sync::atomic::{AtomicBool, Ordering};

// GCC базовый адрес будет взят из DT при probe
static GCC_BASE: SpinLock<Option<usize>> = SpinLock::new(None);

// Регистры GCC (offset от GCC_BASE)
mod reg {
    // Clock branch control
    pub const CLK_ENA: usize = 0x0000;     // CLK_ENABLE
    pub const CLK_DIS: usize = 0x0004;     // CLK_DISABLE
    pub const CLK_STS: usize = 0x0008;     // CLK_STATUS
    pub const CLK_FREQ: usize = 0x000C;    // CLK_FREQ (read current freq)
    pub const CLK_DIV: usize = 0x0010;     // CLK_DIV (set divider)
    pub const CLK_CMD_RCGR: usize = 0x0014; // CLK_CMD_RCGR
    pub const CLK_CMD_RCGR_VAL: usize = 0x0018;
    pub const CLK_MND: usize = 0x001C;     // MND counter
    pub const PLL_MODE: usize = 0x0020;    // PLL_MODE
    pub const PLL_STATUS: usize = 0x0024;  // PLL_STATUS
    pub const PLL_USER_CTL: usize = 0x0028;
    pub const PLL_CONFIG_CTL: usize = 0x002C;
    pub const PLL_TEST_CTL: usize = 0x0030;

    // Block command (BCMD) для batch операций
    pub const BCMD: usize = 0x1000;        // BCMD
    pub const BCMD_RCGR: usize = 0x1004;

    // Reset control
    pub const RESET: usize = 0x2000;       // BLOCK_RESET
    pub const RESET_STATUS: usize = 0x2004;

    // PLL базовые регистры (начинаются с 0x3000)
    pub const PLL_BASE: usize = 0x3000;
    pub const PLL_STRIDE: usize = 0x40;    // шаг между PLL
}

// Часто встречающиеся clock IDs (см. Linux gcc-sm6375.h)
#[allow(dead_code)]
mod clk_id {
    // GPU clocks
    pub const GPU_CX: u32 = 0x1000;
    pub const GPU_GX: u32 = 0x1001;

    // Memory clocks
    pub const DDRSS: u32 = 0x2000;
    pub const DDRSS_AXI: u32 = 0x2001;

    // BLSP (UART, SPI, I2C)
    pub const BLSP1_APPS: u32 = 0x3000;
    pub const BLSP1_AHB: u32 = 0x3001;
    pub const BLSP2_APPS: u32 = 0x3002;
    pub const BLSP2_AHB: u32 = 0x3003;

    // USB
    pub const USB3_PRIM_PHY: u32 = 0x4000;
    pub const USB3_SEC_PHY: u32 = 0x4001;

    // UFS
    pub const UFS_PHY: u32 = 0x5000;
    pub const UFS_AXI: u32 = 0x5001;

    // SPMI
    pub const SPMI: u32 = 0x6000;

    // GPLL0
    pub const GPLL0: u32 = 0x7000;
    pub const GPLL0_OUT_MAIN: u32 = 0x7001;
    pub const GPLL0_OUT_AUX: u32 = 0x7002;
}

// PLL IDs
#[allow(dead_code)]
mod pll_id {
    pub const GPLL0: u32 = 0;
    pub const GPLL1: u32 = 1;
    pub const GPLL2: u32 = 2;
    pub const GPLL3: u32 = 3;
    pub const GPLL4: u32 = 4;
    pub const GPLL5: u32 = 5;
    pub const GPLL6: u32 = 6;
    pub const GPLL7: u32 = 7;
    pub const GPLL8: u32 = 8;
    pub const GPLL9: u32 = 9;
}

static GCC_INIT: AtomicBool = AtomicBool::new(false);
static GCC_MAPPED: SpinLock<Option<usize>> = SpinLock::new(None);

/// Получить базовый адрес GCC
fn gcc_base() -> usize {
    let guard = GCC_MAPPED.lock();
    guard.expect("GCC not mapped")
}

/// Безопасное чтение 32-битного регистра GCC
#[inline]
fn gcc_read(offset: usize) -> u32 {
    let base = gcc_base();
    unsafe { core::ptr::read_volatile((base + offset) as *const u32) }
}

/// Безопасная запись 32-битного регистра GCC
#[inline]
fn gcc_write(offset: usize, val: u32) {
    let base = gcc_base();
    unsafe { core::ptr::write_volatile((base + offset) as *mut u32, val) }
}

/// Установка битов
#[inline]
fn gcc_set_bits(offset: usize, mask: u32) {
    let val = gcc_read(offset) | mask;
    gcc_write(offset, val);
}

/// Очистка битов
#[inline]
fn gcc_clear_bits(offset: usize, mask: u32) {
    let val = gcc_read(offset) & !mask;
    gcc_write(offset, val);
}

/// Включить clock branch
pub fn clk_enable(clk_id: u32) -> Result<(), DriverError> {
    let reg = reg::CLK_ENA + (clk_id as usize * 4);
    gcc_set_bits(reg, 1);
    // Ждем установки бита в CLK_STATUS
    for _ in 0..1000 {
        if gcc_read(reg::CLK_STS + (clk_id as usize * 4)) & 1 != 0 {
            return Ok(());
        }
    }
    Err(DriverError::Timeout)
}

/// Выключить clock branch
pub fn clk_disable(clk_id: u32) -> Result<(), DriverError> {
    let reg = reg::CLK_DIS + (clk_id as usize * 4);
    gcc_set_bits(reg, 1);
    for _ in 0..1000 {
        if gcc_read(reg::CLK_STS + (clk_id as usize * 4)) & 1 == 0 {
            return Ok(());
        }
    }
    Err(DriverError::Timeout)
}

/// Проверить, включен ли clock
pub fn clk_is_enabled(clk_id: u32) -> bool {
    gcc_read(reg::CLK_STS + (clk_id as usize * 4)) & 1 != 0
}

/// Установить частоту clock branch через MND/RCGR
pub fn clk_set_rate(clk_id: u32, rate: u64, parent_rate: u64) -> Result<(), DriverError> {
    // Для простоты: используем RCGR с простым делителем
    // Реальная реализация: вычисление M/N/D, программирование PLL_MODE, CLK_MND, CLK_CMD_RCGR
    let div = if rate == 0 { 1 } else { (parent_rate / rate).max(1) as u32 };
    let mnd = (div - 1) & 0xFFFF; // упрощенно

    let rcgr_reg = reg::CLK_CMD_RCGR + (clk_id as usize * 0x40);
    let mnd_reg = reg::CLK_MND + (clk_id as usize * 0x40);

    gcc_write(mnd_reg, mnd);
    gcc_write(rcgr_reg, 0x1); // trigger update

    for _ in 0..1000 {
        if gcc_read(rcgr_reg) & 1 == 0 {
            return Ok(());
        }
    }
    Err(DriverError::Timeout)
}

/// Получить текущую частоту clock
pub fn clk_get_rate(clk_id: u32) -> u64 {
    gcc_read(reg::CLK_FREQ + (clk_id as usize * 4)) as u64
}

/// Сброс блока (block reset)
pub fn block_reset(clk_id: u32, assert: bool) -> Result<(), DriverError> {
    let reg = reg::RESET + (clk_id as usize * 4);
    if assert {
        gcc_set_bits(reg, 1);
    } else {
        gcc_clear_bits(reg, 1);
    }
    Ok(())
}

/// Пробирование GCC драйвера
fn gcc_probe(dev: &PlatformDevice) -> Result<(), DriverError> {
    if GCC_INIT.swap(true, Ordering::AcqRel) {
        return Err(DriverError::AlreadyExists);
    }

    // Находим память
    let (base, size) = dev.get_resource_mem(0).ok_or(DriverError::NoMemory)?;

    // Маппим GCC в виртуальную память (identity mapping уже есть в PMM)
    *GCC_MAPPED.lock() = Some(base);

    // Запоминаем базу для clock_verify
    crate::drivers::clock_verify::clock_set_gcc_base(base);

    kprintln!("gcc-sm6375: mapped at {:#x}, size={:#x}", base, size);

    // Базовая проверка: читаем какой-нибудь регистр
    let _ = gcc_read(0x0);
    kprintln!("gcc-sm6375: probe ok");
    crate::drivers::vibro::indicate(crate::drivers::vibro::BootStage::GccProbe);

    Ok(())
}

/// Драйвер GCC (Blair / SM6375)
static GCC_DRIVER: Driver = Driver {
    name: "gcc-blair",
    compatible: ["qcom,blair-gccsyscon", "qcom,gcc-sm6375", "", "", "", "", "", ""],
    compatible_n: 2,
    probe: gcc_probe,
    remove: None,
};

/// Регистрация драйвера (явный вызов вместо .init_array)
pub fn register_driver() -> Result<(), DriverError> {
    platform::driver_register(&GCC_DRIVER)
}

// Публичный API для других драйверов — wrapper функции с ожидаемыми именами
pub fn clk_enable_by_id(clk_id: u32) -> Result<(), DriverError> {
    clk_enable(clk_id)
}
pub fn clk_disable_by_id(clk_id: u32) -> Result<(), DriverError> {
    clk_disable(clk_id)
}