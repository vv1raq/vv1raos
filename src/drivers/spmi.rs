//! SPMI (System Power Management Interface) Master Driver для SM6375
//!
//! SPMI контроллер находится в BLSP (Bus Low Speed Peripheral) блоке.
//! SM6375 имеет несколько BLSP: BLSP1, BLSP2, BLSP3.
//! SPMI обычно на BLSP1 или BLSP2.
//!
//! Базовые адреса (из DT):
//! - BLSP1: 0x0880_0000 + offset (обычно 0x0)
//! - BLSP2: 0x0880_0000 + offset
//!
//! SPMI Master регистры (относительно BLSP base + SPMI offset):
//! SPMI_GENI_BASE = 0x0
//! SPMI_PIC_BASE = 0x2000
//! SPMI_OBS_BASE = 0x3000

use crate::platform::{self, Driver, DriverError, PlatformDevice};
use crate::sync::SpinLock;
use core::sync::atomic::{AtomicBool, Ordering};

// SPMI регистры (GENI-based)
mod reg {
    // GENI registers
    pub const GENI_CTRL: usize = 0x00;
    pub const GENI_STATUS: usize = 0x04;
    pub const GENI_ERR: usize = 0x08;
    pub const GENI_M_CMD0: usize = 0x0C;
    pub const GENI_M_CMD1: usize = 0x10;
    pub const GENI_M_RD_CMD0: usize = 0x14;
    pub const GENI_M_RD_CMD1: usize = 0x18;
    pub const GENI_TX_FIFO: usize = 0x20;
    pub const GENI_RX_FIFO: usize = 0x24;
    pub const GENI_FIFO_STATUS: usize = 0x28;
    pub const GENI_FIFO_WATERMARK: usize = 0x2C;

    // SPMI PIC (Peripheral Interface Controller)
    pub const PIC_CTRL: usize = 0x2000;
    pub const PIC_ACC_CMD: usize = 0x2004;
    pub const PIC_ACC_ADDR: usize = 0x2008;
    pub const PIC_ACC_DATA: usize = 0x200C;
    pub const PIC_ACC_STATUS: usize = 0x2010;
    pub const PIC_OWNER_ACC: usize = 0x2014;
    pub const PIC_OWNER_DATA: usize = 0x2018;
    pub const PIC_OWNER_STATUS: usize = 0x201C;

    // SPMI OBS (Observer)
    pub const OBS_CTRL: usize = 0x3000;
}

// SPMI команды
mod cmd {
    pub const EXT_READ_LONG: u8 = 0x10;
    pub const EXT_WRITE_LONG: u8 = 0x11;
    pub const READ_CMD: u8 = 0x00;
    pub const WRITE_CMD: u8 = 0x01;
    pub const RESET_CMD: u8 = 0x07;
    pub const SLEEP_CMD: u8 = 0x08;
    pub const SHUTDOWN_CMD: u8 = 0x09;
    pub const WAKE_CMD: u8 = 0x0A;
    pub const AUTHENTICATE_CMD: u8 = 0x0B;
}

static SPMI_INIT: AtomicBool = AtomicBool::new(false);
static SPMI_BASE: SpinLock<Option<usize>> = SpinLock::new(None);

fn spmi_base() -> usize {
    let guard = SPMI_BASE.lock();
    guard.expect("SPMI not mapped")
}

#[inline]
fn spmi_read(offset: usize) -> u32 {
    let base = spmi_base();
    unsafe { core::ptr::read_volatile((base + offset) as *const u32) }
}

#[inline]
fn spmi_write(offset: usize, val: u32) {
    let base = spmi_base();
    unsafe { core::ptr::write_volatile((base + offset) as *mut u32, val) }
}

/// Инициализация SPMI Master
fn spmi_init(base: usize) -> Result<(), DriverError> {
    // Сброс GENI
    spmi_write(reg::GENI_CTRL, 0x1); // SW reset
    for _ in 0..1000 {
        if spmi_read(reg::GENI_CTRL) & 1 == 0 {
            break;
        }
    }

    // Настройка GENI для SPMI Master mode
    // Master mode, SPMI protocol
    spmi_write(reg::GENI_CTRL, 0x100); // M_MST_EN | M_SPI_MODE (SPMI)

    // FIFO watermarks
    spmi_write(reg::GENI_FIFO_WATERMARK, 0x1010); // RX/TX watermark = 16

    // PIC init
    spmi_write(reg::PIC_CTRL, 0x1); // Enable PIC

    kprintln!("spmi: initialized at {:#x}", base);
    Ok(())
}

/// SPMI Extended Read Long (чтение регистра PMIC)
/// sid: Slave ID (0-15 для PMIC), addr: 16-bit адрес регистра
pub fn spmi_read_reg(sid: u8, addr: u16) -> Result<u8, DriverError> {
    let _base = SPMI_BASE.lock().expect("SPMI not mapped");

    // Формируем команду через PIC
    // PIC_ACC_CMD: [31:24]=opcode, [23:16]=sid, [15:0]=addr
    let cmd = ((cmd::EXT_READ_LONG as u32) << 24) | ((sid as u32) << 16) | (addr as u32);
    spmi_write(reg::PIC_ACC_CMD, cmd);

    // Запуск транзакции
    spmi_write(reg::PIC_ACC_STATUS, 0x1); // GO

    // Ждем завершения
    for _ in 0..10000 {
        let status = spmi_read(reg::PIC_ACC_STATUS);
        if status & 1 == 0 {
            break;
        }
    }

    let status = spmi_read(reg::PIC_ACC_STATUS);
    if status & 0xE != 0 {
        return Err(DriverError::InvalidState);
    }

    let data = spmi_read(reg::PIC_ACC_DATA) as u8;
    Ok(data)
}

/// SPMI Extended Write Long (запись регистра PMIC)
pub fn spmi_write_reg(sid: u8, addr: u16, val: u8) -> Result<(), DriverError> {
    let _base = SPMI_BASE.lock().expect("SPMI not mapped");

    // Данные в PIC_ACC_DATA
    spmi_write(reg::PIC_ACC_DATA, val as u32);

    // Команда
    let cmd = ((cmd::EXT_WRITE_LONG as u32) << 24) | ((sid as u32) << 16) | (addr as u32);
    spmi_write(reg::PIC_ACC_CMD, cmd);

    // Запуск
    spmi_write(reg::PIC_ACC_STATUS, 0x1);

    for _ in 0..10000 {
        let status = spmi_read(reg::PIC_ACC_STATUS);
        if status & 1 == 0 {
            break;
        }
    }

    let status = spmi_read(reg::PIC_ACC_STATUS);
    if status & 0xE != 0 {
        return Err(DriverError::InvalidState);
    }
    Ok(())
}

/// SPMI Broadcast Write (к нескольким sids)
pub fn spmi_broadcast_write(addr: u16, val: u8) -> Result<(), DriverError> {
    // SID = 0xFF для broadcast
    spmi_write_reg(0xFF, addr, val)
}

/// Прочитать несколько байт (burst read)
pub fn spmi_read_burst(sid: u8, addr: u16, buf: &mut [u8]) -> Result<(), DriverError> {
    for (i, b) in buf.iter_mut().enumerate() {
        *b = spmi_read_reg(sid, addr + i as u16)?;
    }
    Ok(())
}

/// Записать несколько байт (burst write)
pub fn spmi_write_burst(sid: u8, addr: u16, buf: &[u8]) -> Result<(), DriverError> {
    for (i, &b) in buf.iter().enumerate() {
        spmi_write_reg(sid, addr + i as u16, b)?;
    }
    Ok(())
}

/// Пробирование SPMI драйвера
fn spmi_probe(dev: &PlatformDevice) -> Result<(), DriverError> {
    if SPMI_INIT.swap(true, Ordering::AcqRel) {
        return Err(DriverError::AlreadyExists);
    }

    let (base, size) = dev.get_resource_mem(0).ok_or(DriverError::NoMemory)?;

    *SPMI_BASE.lock() = Some(base);

    // Включаем clock для SPMI через GCC
    if let Some(clk_id) = dev.get_resource_clock("bus") {
        crate::drivers::gcc_sm6375::clk_enable_by_id(clk_id as u32)?;
    }
    if let Some(clk_id) = dev.get_resource_clock("iface") {
        crate::drivers::gcc_sm6375::clk_enable_by_id(clk_id as u32)?;
    }

    spmi_init(base)?;
    kprintln!("spmi: probe ok, base={:#x}, size={:#x}", base, size);
    crate::drivers::vibro::indicate(crate::drivers::vibro::BootStage::SpmiProbe);
    Ok(())
}

static SPMI_DRIVER: Driver = Driver {
    name: "spmi-pmic-arb",
    compatible: ["qcom,spmi-pmic-arb", "qcom,spmi-geni", "", "", "", "", "", ""],
    compatible_n: 2,
    probe: spmi_probe,
    remove: None,
};

/// Регистрация драйвера (явный вызов вместо .init_array)
pub fn register_driver() -> Result<(), DriverError> {
    platform::driver_register(&SPMI_DRIVER)
}

// pub use spmi_read_reg;
// pub use spmi_write_reg;
// pub use spmi_broadcast_write;
// pub use spmi_read_burst;
// pub use spmi_write_burst;