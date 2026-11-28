//! MSM UART (BLSP UART) Driver для Qualcomm SM6375
//!
//! BLSP (Bus Low Speed Peripheral) содержит UART, SPI, I2C.
//! UART регистры совместимы с 16550 но с расширениями Qualcomm.
//!
//! Базовые адреса (из DT):
//! - BLSP1_UART: 0x0880_0000 + 0x0 (или 0x10000)
//! - BLSP2_UART: 0x0880_0000 + 0x20000
//!
//! Регистры (offset от UART base):
//! 0x000: RBR/THR/DLL (RX Buffer / TX Holding / Divisor Latch Low)
//! 0x004: IER/DLH (Interrupt Enable / Divisor Latch High)
//! 0x008: IIR/FCR (Interrupt ID / FIFO Control)
//! 0x00C: LCR (Line Control)
//! 0x010: MCR (Modem Control)
//! 0x014: LSR (Line Status)
//! 0x018: MSR (Modem Status)
//! 0x01C: SCR (Scratch)
//! 0x020: SRBR_STHR (Shadow RX/TX)
//! 0x028: IP_VERSION
//! 0x030: UART_DM (DM Enable)
//! 0x034: UART_DM_RX (DM RX)
//! 0x038: UART_DM_TX (DM TX)
//! 0x040: RX_TOTAL_SNAP
//! 0x044: RX_FIFO_SNAP
//! 0x048: TX_FIFO_SNAP
//! 0x050: DM_RXFIFO
//! 0x060: RX_FIFO
//! 0x064: TX_FIFO
//! 0x070: CLOCKS
//! 0x074: BAUD_RATE
//! 0x078: WATERMARK
//! 0x07C: IRDA
//! 0x080: STALE_TIMER
//! 0x084: RX_STALE

use crate::drivers::gcc_sm6375::{clk_enable_by_id, clk_disable_by_id};
use crate::platform::{self, Driver, DriverError, PlatformDevice, ProbeStatus, Resource};
use crate::sync::SpinLock;
use core::fmt::Write;
use core::sync::atomic::{AtomicBool, Ordering};

// UART регистры
mod reg {
    pub const RBR: usize = 0x00;
    pub const THR: usize = 0x00;
    pub const DLL: usize = 0x00;
    pub const IER: usize = 0x04;
    pub const DLH: usize = 0x04;
    pub const IIR: usize = 0x08;
    pub const FCR: usize = 0x08;
    pub const LCR: usize = 0x0C;
    pub const MCR: usize = 0x10;
    pub const LSR: usize = 0x14;
    pub const MSR: usize = 0x18;
    pub const SCR: usize = 0x1C;
    pub const SRBR_STHR: usize = 0x20;
    pub const IP_VERSION: usize = 0x28;
    pub const DM_EN: usize = 0x30;
    pub const DM_RX: usize = 0x34;
    pub const DM_TX: usize = 0x38;
    pub const RX_TOTAL_SNAP: usize = 0x40;
    pub const RX_FIFO_SNAP: usize = 0x44;
    pub const TX_FIFO_SNAP: usize = 0x48;
    pub const DM_RXFIFO: usize = 0x50;
    pub const RX_FIFO: usize = 0x60;
    pub const TX_FIFO: usize = 0x64;
    pub const CLK: usize = 0x70;
    pub const BAUD: usize = 0x74;
    pub const WATERMARK: usize = 0x78;
    pub const IRDA: usize = 0x7C;
    pub const STALE_TIMER: usize = 0x80;
    pub const RX_STALE: usize = 0x84;
}

// LSR биты
mod lsr {
    pub const DR: u32 = 0x01;   // Data Ready
    pub const OE: u32 = 0x02;   // Overrun Error
    pub const PE: u32 = 0x04;   // Parity Error
    pub const FE: u32 = 0x08;   // Framing Error
    pub const BI: u32 = 0x10;   // Break Interrupt
    pub const THRE: u32 = 0x20; // THR Empty
    pub const TEMT: u32 = 0x40; // Transmitter Empty
    pub const FIFOE: u32 = 0x80; // FIFO Error
}

// FCR биты
mod fcr {
    pub const FIFO_EN: u32 = 0x01;
    pub const RX_RESET: u32 = 0x02;
    pub const TX_RESET: u32 = 0x04;
    pub const DMA_MODE: u32 = 0x08;
    pub const RX_TRIG_1: u32 = 0x00;
    pub const RX_TRIG_4: u32 = 0x40;
    pub const RX_TRIG_8: u32 = 0x80;
    pub const RX_TRIG_14: u32 = 0xC0;
}

// LCR биты
mod lcr {
    pub const DLAB: u32 = 0x80; // Divisor Latch Access Bit
    pub const BREAK: u32 = 0x40;
    pub const PARITY_EN: u32 = 0x08;
    pub const EVEN_PARITY: u32 = 0x10;
    pub const TWO_STOP: u32 = 0x04;
    pub const WORD_LEN_8: u32 = 0x03;
}

// MCR биты
mod mcr {
    pub const DTR: u32 = 0x01;
    pub const RTS: u32 = 0x02;
    pub const LOOP: u32 = 0x10;
    pub const AFE: u32 = 0x20; // Auto Flow Enable
}

// DM (Debug Messenger) регистры
mod dm {
    pub const DM_EN: u32 = 0x1;
    pub const RX_EN: u32 = 0x2;
    pub const TX_EN: u32 = 0x4;
    pub const RX_FLUSH: u32 = 0x10;
    pub const TX_FLUSH: u32 = 0x20;
}

static UART_EARLY: SpinLock<Option<MsmUart>> = SpinLock::new(None);
static UART_LATE: SpinLock<Option<MsmUart>> = SpinLock::new(None);
static UART_INIT: AtomicBool = AtomicBool::new(false);

/// Структура UART
#[derive(Clone, Copy)]
pub struct MsmUart {
    base: usize,
    clk_uart: Option<u32>,
    clk_pclk: Option<u32>,
    baud: u32,
}

impl MsmUart {
    #[inline]
    fn read(&self, offset: usize) -> u32 {
        unsafe { core::ptr::read_volatile((self.base + offset) as *const u32) }
    }

    #[inline]
    fn write(&self, offset: usize, val: u32) {
        unsafe { core::ptr::write_volatile((self.base + offset) as *mut u32, val) }
    }

    /// Инициализация UART (early или late)
    fn init_hw(&self) {
        // Disable interrupts
        self.write(reg::IER, 0);

        // Enable FIFO, reset RX/TX
        self.write(reg::FCR, fcr::FIFO_EN | fcr::RX_RESET | fcr::TX_RESET);
        // Watermark: RX trigger at 8 chars
        self.write(reg::FCR, fcr::FIFO_EN | fcr::RX_TRIG_8);

        // Line control: 8N1, no parity
        self.write(reg::LCR, lcr::WORD_LEN_8);

        // Modem control: RTS/DTR high, no loopback
        self.write(reg::MCR, mcr::DTR | mcr::RTS);

        // Set baud rate
        self.set_baud(self.baud);

        // Enable DM (Debug Messenger) для earlycon
        self.write(reg::DM_EN, dm::DM_EN | dm::RX_EN | dm::TX_EN);
    }

    /// Установить baud rate
    fn set_baud(&self, baud: u32) {
        // UART clock обычно 7.3728 MHz или 24 MHz
        // DL = uart_clk / (16 * baud)
        // Для SM6375: uart_clk = 7.3728 MHz (от GPLL0)
        const UART_CLK: u32 = 7_372_800;
        let div = UART_CLK / (16 * baud);

        // Set DLAB
        let lcr = self.read(reg::LCR);
        self.write(reg::LCR, lcr | lcr::DLAB);

        // Write divisor
        self.write(reg::DLL, (div & 0xFF) as u32);
        self.write(reg::DLH, ((div >> 8) & 0xFF) as u32);

        // Clear DLAB
        self.write(reg::LCR, lcr & !lcr::DLAB);

        // Also write to BAUD register (Qualcomm extension)
        self.write(reg::BAUD, div);
    }

    /// Проверка, готов ли TX
    fn tx_ready(&self) -> bool {
        self.read(reg::LSR) & lsr::THRE != 0
    }

    /// Проверка, есть ли данные в RX
    fn rx_ready(&self) -> bool {
        self.read(reg::LSR) & lsr::DR != 0
    }

    /// Отправить байт (polling)
    fn putc(&self, c: u8) {
        while !self.tx_ready() {
            core::hint::spin_loop();
        }
        self.write(reg::THR, c as u32);
    }

    /// Получить байт (polling)
    fn getc(&self) -> Option<u8> {
        if self.rx_ready() {
            Some(self.read(reg::RBR) as u8)
        } else {
            None
        }
    }
}

impl Write for MsmUart {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        for b in s.bytes() {
            self.putc(b);
        }
        Ok(())
    }
}

/// Earlycon: инициализация до полной platform init
pub fn earlycon_init(dtb: usize) -> Result<(), DriverError> {
    // Парсим DTB вручную для поиска earlycon узла
    // Для простоты: ищем первый compatible "qcom,msm-uart" или "qcom,msm-serial"
    // и используем его как earlycon

    // В реальности: parse chosen/stdout-path или aliases/serial0
    // Здесь просто используем первый найденный

    // Заглушка: earlycon будет инициализирован в kernel_main после platform_init
    Ok(())
}

/// Получить earlycon для печати до полной инициализации
pub fn earlycon_putc(c: u8) {
    if let Some(uart) = UART_EARLY.lock().as_ref() {
        uart.putc(c);
    }
}

/// Получить earlycon для печати строк
pub fn earlycon_write_str(s: &str) {
    if let Some(mut uart) = UART_EARLY.lock().as_mut() {
        let _ = core::fmt::write(&mut uart, format_args!("{}", s));
    }
}

/// Late init: полноценный UART после platform_drivers_init
fn msm_uart_probe(dev: &PlatformDevice) -> Result<(), DriverError> {
    let (base, size) = dev.get_resource_mem(0).ok_or(DriverError::NoMemory)?;

    // Clock IDs
    let clk_uart = dev.get_resource_clock("uart").map(|x| x as u32);
    let clk_pclk = dev.get_resource_clock("pclk").map(|x| x as u32);

    // Включаем clocks
    if let Some(id) = clk_uart {
        clk_enable_by_id(id)?;
    }
    if let Some(id) = clk_pclk {
        clk_enable_by_id(id)?;
    }

    let uart = MsmUart {
        base,
        clk_uart,
        clk_pclk,
        baud: 115200,
    };

    uart.init_hw();

    // Если earlycon не был установлен — используем этот
    if UART_EARLY.lock().is_none() {
        *UART_EARLY.lock() = Some(uart);
        kprintln!("msm-uart: earlycon at {:#x}", base);
    } else {
        *UART_LATE.lock() = Some(uart);
        kprintln!("msm-uart: late console at {:#x}", base);
    }

    kprintln!("msm-uart: probe ok, base={:#x}, size={:#x}", base, size);
    Ok(())
}

/// Публичная функция для вывода из kernel_main (после probe)
pub fn msm_uart_late_putc(c: u8) {
    if let Some(uart) = UART_LATE.lock().as_ref() {
        uart.putc(c);
    } else if let Some(uart) = UART_EARLY.lock().as_ref() {
        uart.putc(c);
    }
}

pub fn msm_uart_late_getc() -> Option<u8> {
    if let Some(uart) = UART_LATE.lock().as_ref() {
        uart.getc()
    } else if let Some(uart) = UART_EARLY.lock().as_ref() {
        uart.getc()
    } else {
        None
    }
}

static MSM_UART_DRIVER: Driver = Driver {
    name: "msm-geni-serial",
    compatible: ["qcom,msm-geni-serial-hs", "qcom,msm-uart", "qcom,msm-serial", "", "", "", "", ""],
    compatible_n: 3,
    probe: msm_uart_probe,
    remove: None,
};

/// Регистрация драйвера (явный вызов вместо .init_array)
pub fn register_driver() -> Result<(), DriverError> {
    platform::driver_register(&MSM_UART_DRIVER)
}

// pub use earlycon_init;
// pub use earlycon_putc;
// pub use earlycon_write_str;
// pub use msm_uart_late_putc;
// pub use msm_uart_late_getc;