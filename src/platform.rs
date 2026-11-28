use core::{
    fmt,
    marker::PhantomData,
    sync::atomic::{AtomicBool, AtomicUsize, Ordering},
};

/// Тип прерывания (как в DT: GIC_SPI, GIC_PPI, GIC_IPI)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum IrqType {
    Spi = 0,
    Ppi = 1,
    Ipi = 2,
}

/// Описание прерывания из DT
#[derive(Debug, Clone, Copy)]
pub struct IrqSpec {
    pub irq_type: IrqType,
    pub number: u32,      // SPI/PPI номер
    pub trigger: u32,     // flags: 1=low-to-high, 2=high-to-low, 4=level-high, 8=level-low
}

/// Ресурс устройства (память, IRQ, clock)
#[derive(Debug, Clone, Copy)]
pub enum Resource {
    Mem { base: usize, size: usize },
    Irq(IrqSpec),
    Clock { name: &'static str, id: usize },
    PowerDomain { name: &'static str, id: usize },
    /// IOMMU/SMMU: phandle контроллера, SID (Stream ID), mask (опционально)
    Iommu { phandle: usize, sid: u32, mask: u32 },
}

/// Статус пробирования
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProbeStatus {
    NotProbed,
    Probing,
    Ok,
    Failed,
}

/// Функция пробирования драйвера
pub type ProbeFn = fn(&PlatformDevice) -> Result<(), DriverError>;

/// Ошибки драйвера
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DriverError {
    NotSupported,
    NoMemory,
    NoIrq,
    NoClock,
    AlreadyExists,
    Busy,
    Timeout,
    InvalidState,
    NotReady,       // драйвер не инициализирован
    InvalidArg,     // неверный аргумент
    IoError,        // ошибка ввода-вывода
}

impl fmt::Display for DriverError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{:?}", self)
    }
}

/// Platform устройство (экземпляр из DT)
/// Ресурсы и compatible хранятся inline для избежания проблем с lifetime
#[derive(Clone, Copy)]
pub struct PlatformDevice {
    pub name: &'static str,
    pub compatible: [&'static str; 8],  // inline storage
    pub compatible_n: usize,
    pub resources: [Resource; 8],  // inline storage
    pub resources_n: usize,
    pub data: usize,           // приватные данные драйвера после probe (адрес)
    pub status: ProbeStatus,
    pub driver_data: usize,    // опционально: указатель на драйвер-специфичную структуру (адрес)
}

impl PlatformDevice {
    pub fn get_resource_mem(&self, index: usize) -> Option<(usize, usize)> {
        let mut found = 0;
        for r in &self.resources[..self.resources_n] {
            if let Resource::Mem { base, size } = r {
                if found == index {
                    return Some((*base, *size));
                }
                found += 1;
            }
        }
        None
    }

    pub fn get_resource_irq(&self, index: usize) -> Option<IrqSpec> {
        let mut found = 0;
        for r in &self.resources[..self.resources_n] {
            if let Resource::Irq(irq) = r {
                if found == index {
                    return Some(*irq);
                }
                found += 1;
            }
        }
        None
    }

    pub fn get_resource_clock(&self, name: &str) -> Option<usize> {
        for r in &self.resources[..self.resources_n] {
            if let Resource::Clock { name: n, id } = r {
                if *n == name {
                    return Some(*id);
                }
            }
        }
        None
    }

    /// Получить SID (Stream ID) из iommus ресурса (первый найденный)
    pub fn get_resource_sid(&self) -> Option<u32> {
        for r in &self.resources[..self.resources_n] {
            if let Resource::Iommu { sid, .. } = r {
                return Some(*sid);
            }
        }
        None
    }
}

/// Драйвер platform устройства
#[derive(Clone, Copy)]
pub struct Driver {
    pub name: &'static str,
    pub compatible: [&'static str; 8],
    pub compatible_n: usize,
    pub probe: ProbeFn,
    pub remove: Option<fn(&PlatformDevice)>,
}

/// Глобальный реестр драйверов и устройств
const MAX_DRIVERS: usize = 32;
const MAX_DEVICES: usize = 64;

static DRIVERS: SpinLock<[Option<&'static Driver>; MAX_DRIVERS]> = SpinLock::new([None; MAX_DRIVERS]);
static DEVICES: SpinLock<[Option<PlatformDevice>; MAX_DEVICES]> = SpinLock::new([None; MAX_DEVICES]);
static INIT_DONE: AtomicBool = AtomicBool::new(false);

/// PL011 base для раннего лога (QEMU virt = 0x9000000)
static EARLY_UART: AtomicUsize = AtomicUsize::new(0);

/// Установить базу PL011 для раннего лога (вызывается из kernel_main после парсинга DTB)
pub fn set_early_uart(base: usize) {
    EARLY_UART.store(base, Ordering::Release);
}

/// Ранний лог напрямую в PL011 (работает до console::init)
fn early_log(msg: &str) {
    let base = EARLY_UART.load(Ordering::Acquire);
    if base == 0 {
        return;
    }
    const DR: usize = 0x00;
    const FR: usize = 0x18;
    unsafe {
        for b in msg.bytes() {
            // Ждём пока TX не свободен
            while core::ptr::read_volatile((base + FR) as *const u32) & (1 << 5) != 0 {}
            core::ptr::write_volatile((base + DR) as *mut u32, b as u32);
        }
    }
}

/// Зарегистрировать драйвер (вызывается из конструкторов драйверов)
pub fn driver_register(drv: &'static Driver) -> Result<(), DriverError> {
    let mut drivers = DRIVERS.lock();
    for slot in drivers.iter_mut() {
        if slot.is_none() {
            *slot = Some(drv);
            let mut compats = [""; 8];
            let mut n = 0;
            for c in &drv.compatible[..drv.compatible_n] {
                if !c.is_empty() && n < 8 {
                    compats[n] = *c;
                    n += 1;
                }
            }
            // Ранний лог напрямую в UART (только если UART уже установлен)
            early_log("[DRV] registered\n");
            // Также пишем в PL011 по фиксированному адресу QEMU (0x9000000) как fallback
            const QEMU_PL011: usize = 0x9000000;
            const DR: usize = 0x00;
            const FR: usize = 0x18;
            unsafe {
                for b in b"[DRV] registered\n" {
                    while core::ptr::read_volatile((QEMU_PL011 + FR) as *const u32) & (1 << 5) != 0 {}
                    core::ptr::write_volatile((QEMU_PL011 + DR) as *mut u32, *b as u32);
                }
            }
            return Ok(());
        }
    }
    Err(DriverError::NoMemory)
}

/// Создать platform device из DT узла (вызывается из platform_init)
pub fn device_register(dev: PlatformDevice) -> Result<(), DriverError> {
    let mut devices = DEVICES.lock();
    for slot in devices.iter_mut() {
        if slot.is_none() {
            *slot = Some(dev);
            return Ok(());
        }
    }
    Err(DriverError::NoMemory)
}

/// Запуск пробирования всех зарегистрированных устройств
pub fn platform_probe_all() -> usize {
    let mut probed = 0;

    // Сначала собираем список устройств для пробирования (индексы)
    let mut to_probe = [0usize; MAX_DEVICES];
    let mut to_probe_n = 0;

    {
        let devices = DEVICES.lock();
        let drivers = DRIVERS.lock();
        for (i, dev_slot) in devices.iter().enumerate() {
            if let Some(dev) = dev_slot {
                if dev.status == ProbeStatus::NotProbed {
                    for drv_opt in drivers.iter() {
                        if let Some(drv) = drv_opt {
                            if drv.compatible[..drv.compatible_n].iter().any(|c| dev.compatible[..dev.compatible_n].contains(c)) {
                                if to_probe_n < MAX_DEVICES {
                                    to_probe[to_probe_n] = i;
                                    to_probe_n += 1;
                                }
                                kprintln!("platform: will probe device {} ({}) with driver {}", i, dev.name, drv.name);
                                break;
                            }
                        }
                    }
                }
            }
        }
    }

    if to_probe_n == 0 {
        kprintln!("platform: no devices to probe (no matching drivers)");
    }

    // Теперь пробуем каждое устройство
    for idx in &to_probe[..to_probe_n] {
        let idx = *idx;

        // Копируем устройство под локом, затем отпускаем лок
        let (dev, drv) = {
            let mut devices = DEVICES.lock();
            let dev_slot = devices.get_mut(idx);
            let Some(dev_slot) = dev_slot else { continue };
            let Some(dev) = dev_slot else { continue };
            if dev.status != ProbeStatus::NotProbed { continue; }

            // Ищем драйвер
            let drivers = DRIVERS.lock();
            let drv = drivers.iter().find_map(|drv_opt| {
                drv_opt.and_then(|drv| {
                    if drv.compatible[..drv.compatible_n].iter().any(|c| dev.compatible[..dev.compatible_n].contains(c)) {
                        Some(*drv)
                    } else { None }
                })
            });

            // Копируем dev (теперь Copy) и отпускаем локи
            (*dev, drv)
        };

        let Some(drv) = drv else { continue };

        // MMIO-ресурсы устройства должны быть в identity-map ДО probe:
        // на реальном железе регионы (GCC/SPMI/UFS...) не входят в стартовый
        // mmio-список main.rs, и обращение к ним без маппинга = data abort.
        for i in 0..dev.resources_n {
            if let Resource::Mem { base, size } = dev.resources[i] {
                if size != 0 {
                    crate::mmu::map_device_identity(base, base + size);
                }
            }
        }

        // Обновляем статус под локом
        {
            let mut devices = DEVICES.lock();
            if let Some(dev_slot) = devices.get_mut(idx) {
                if let Some(dev) = dev_slot {
                    dev.status = ProbeStatus::Probing;
                }
            }
        }

        match (drv.probe)(&dev) {
            Ok(()) => {
                let mut devices = DEVICES.lock();
                if let Some(dev_slot) = devices.get_mut(idx) {
                    if let Some(dev) = dev_slot {
                        dev.status = ProbeStatus::Ok;
                        probed += 1;
                    }
                }
            }
            Err(e) => {
                let mut devices = DEVICES.lock();
                if let Some(dev_slot) = devices.get_mut(idx) {
                    if let Some(dev) = dev_slot {
                        dev.status = ProbeStatus::Failed;
                        kprintln!("platform: probe failed for {}: {}", dev.name, e);
                    }
                }
            }
        }
    }
    probed
}

/// Получить устройство по имени (для поиска clock/PMIC/UART)
pub fn device_find_by_name(name: &str) -> Option<PlatformDevice> {
    let devices = DEVICES.lock();
    for dev_opt in devices.iter() {
        if let Some(dev) = dev_opt {
            if dev.name == name && dev.status == ProbeStatus::Ok {
                return Some(*dev);
            }
        }
    }
    None
}

/// Получить устройство по compatible
pub fn device_find_by_compatible(compat: &str) -> Option<PlatformDevice> {
    let devices = DEVICES.lock();
    for dev_opt in devices.iter() {
        if let Some(dev) = dev_opt {
            if dev.compatible[..dev.compatible_n].contains(&compat) && dev.status == ProbeStatus::Ok {
                return Some(*dev);
            }
        }
    }
    None
}

/// Инициализация подсистемы platform (вызывается рано в kernel_main)
pub fn platform_init(fdt_addr: usize) {
    if INIT_DONE.swap(true, Ordering::AcqRel) {
        return;
    }
    // Парсинг DT и регистрация устройств будет в fdt.rs
    crate::fdt::platform_populate(fdt_addr);
}

/// Есть ли среди зарегистрированных драйверов поддерживающий хотя бы один
/// из данных compatible. platform_populate использует это как фильтр:
/// в реальном DTB SM6375 сотни узлов с compatible, а реестр устройств
/// ограничен (MAX_DEVICES) — без фильтра до GCC/SPMI/UFS очередь не дошла бы.
pub fn driver_interested(compat: &[&str], n: usize) -> bool {
    if n == 0 {
        return false;
    }
    let drivers = DRIVERS.lock();
    drivers
        .iter()
        .flatten()
        .any(|drv| {
            drv.compatible[..drv.compatible_n]
                .iter()
                .any(|c| !c.is_empty() && compat[..n].contains(c))
        })
}

/// Инициализация драйверов (после platform_init, когда PMIC/GCC доступны)
pub fn platform_drivers_init() {
    let n = platform_probe_all();
    kprintln!("platform: probed {} devices", n);
}

/// Макрос для регистрации драйвера при инициализации (ctor)
#[macro_export]
macro_rules! platform_driver {
    ($name:ident, $drv:expr) => {
        #[used]
        #[link_section = ".init_array"]
        static $name: fn() = || {
            let _ = $crate::platform::driver_register(&$drv);
        };
    };
}

pub use platform_driver;

// re-export sync primitives for drivers
pub use crate::sync::{SpinLock, SpinGuard, Global};

// Внешние зависимости для макроса
#[allow(unused_imports)]
use crate::kprintln;