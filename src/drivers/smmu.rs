//! Qualcomm Apps SMMU (SMMUv3 / qsmmu-v500) — bypass для UFS DMA
//!
//! На SM6375 (Blair): /soc/apps-smmu@0c600000, compatible "qcom,qsmmu-v500"
//! UFS использует SID 0x60 (96) за Apps SMMU.
//!
//! Стратегия: найти контекстный банк (TBU), обслуживающий SID 0x60,
//! и настроить его в режим flat mapping (identity) или полностью отключить SMMU
//! для этого SID. В Linux это делается через CBAR/TCR/MAIR контекстного банка.
//!
//! Регистры SMMUv3 (ARM DDI 0570E):
//!   SMMU_S_IDR0/1/2/3/4/5/6/7         0x0000-0x001C  — ID registers
//!   SMMU_S_CR0                        0x0020         — Control register 0
//!   SMMU_S_CR0ACK                     0x0028         — CR0 acknowledge
//!   SMMU_S_GFSR/GFSYNR                0x0030/0x0038  — Global fault
//!   SMMU_S_TLBCR                      0x0040         — TLB control
//!   SMMU_S_TLBSTATUS                  0x0048         — TLB status
//!   SMMU_S_TLBIALLH                   0x0050         — TLB invalidate all H
//!   ...
//!   SMMU_S_CB_BASE(n)                 0x8000 + n*0x2000  — Context bank n base
//!
//! Context Bank регистры (offset от CB_BASE):
//!   CBn_TCR          0x00  — Translation Control Register
//!   CBn_CBAR         0x08  — Context Bank Attribute Register (root table)
//!   CBn_MAIR0/1      0x10/0x18 — Memory Attribute Indirection
//!   CBn_ACTLR        0x20  — Auxiliary Control
//!   CBn_SCTLR        0x28  — Stream Table Control (SMMUv3.1)
//!   CBn_REVIDR       0x30  — Revision ID
//!
//! TBU (Translation Buffer Unit) config регистры at 0xC702000:
//!   TBU_n_CFG        0x00  — TBU configuration (SID range, CB index)

use crate::platform::{self, Driver, DriverError, PlatformDevice};
use crate::sync::SpinLock;
use core::sync::atomic::{AtomicBool, Ordering};

/// SMMUv3 базовый адрес (устанавливается при probe)
static SMMU_BASE: SpinLock<Option<usize>> = SpinLock::new(None);
static SMMU_INIT: AtomicBool = AtomicBool::new(false);

// SMMUv3 регистры (offset от SMMU_BASE)
mod smmu_reg {
    pub const S_IDR0: usize = 0x0000;
    pub const S_CR0: usize = 0x0020;
    pub const S_CR0ACK: usize = 0x0028;
    pub const S_GFSR: usize = 0x0030;
    pub const S_GFSYNR: usize = 0x0038;
    pub const S_TLBCR: usize = 0x0040;
    pub const S_TLBIALLH: usize = 0x0050;
    pub const S_CB_BASE: usize = 0x8000;  // Context Bank base array
    pub const S_CB_STRIDE: usize = 0x2000;
}

// Context Bank регистры (offset от CB_BASE)
mod cb_reg {
    pub const CB_TCR: usize = 0x00;
    pub const CB_CBAR: usize = 0x08;
    pub const CB_MAIR0: usize = 0x10;
    pub const CB_MAIR1: usize = 0x18;
    pub const CB_ACTLR: usize = 0x20;
    pub const CB_SCTLR: usize = 0x28;
}

/// Чтение 64-битного регистра SMMU
fn smmu_read64(off: usize) -> u64 {
    if let Some(base) = *SMMU_BASE.lock() {
        unsafe { core::ptr::read_volatile((base + off) as *const u64) }
    } else { 0 }
}

/// Запись 64-битного регистра SMMU
fn smmu_write64(off: usize, val: u64) {
    if let Some(base) = *SMMU_BASE.lock() {
        unsafe { core::ptr::write_volatile((base + off) as *mut u64, val) }
    }
}

/// Чтение 32-битного регистра SMMU
fn smmu_read32(off: usize) -> u32 {
    if let Some(base) = *SMMU_BASE.lock() {
        unsafe { core::ptr::read_volatile((base + off) as *const u32) }
    } else { 0 }
}

/// Запись 32-битного регистра SMMU
fn smmu_write32(off: usize, val: u32) {
    if let Some(base) = *SMMU_BASE.lock() {
        unsafe { core::ptr::write_volatile((base + off) as *mut u32, val) }
    }
}

/// Задержка
fn udelay(us: u32) {
    for _ in 0..us * 100 {
        core::hint::spin_loop();
    }
}

/// Найти Apps SMMU базовый адрес через platform device
fn get_smmu_base() -> Option<usize> {
    platform::device_find_by_compatible("qcom,qsmmu-v500")
        .and_then(|dev| dev.get_resource_mem(0))
        .map(|(base, _)| base)
}

/// Настроить контекстный банк для flat mapping (identity) для данного SID
/// Возвращает true если успешно
fn configure_cb_for_sid(sid: u32) -> bool {
    let base = get_smmu_base();
    let Some(base) = base else { 
        kprintln!("smmu: SMMU not probed yet");
        return false; 
    };

    kprintln!("smmu: SMMU base {:#x}, trying to bypass SID {}", base, sid);

    // Читаем IDR0 для проверки версии
    let idr0 = smmu_read64(smmu_reg::S_IDR0);
    kprintln!("smmu: S_IDR0 = {:#x}", idr0);

    // Проверяем, что SMMU включён
    let cr0 = smmu_read32(smmu_reg::S_CR0);
    kprintln!("smmu: S_CR0 = {:#x}", cr0);

    // Для SID 0x60 (UFS) пытаемся найти подходящий TBU
    // TBU_BASE = SMMU_BASE + 0x102000 (0xC702000 - 0xC600000)
    let tbu_base = base + 0x102000;
    
    // Читаем конфигурацию TBU (4 entry, по 8 байт каждый: SID_start, SID_count, CB_index, flags)
    for i in 0..4 {
        let cfg = unsafe { core::ptr::read_volatile((tbu_base + i * 8) as *const u64) };
        if cfg != 0 {
            let sid_start = (cfg & 0xFFFF) as u32;
            let sid_count = ((cfg >> 16) & 0xFFFF) as u32;
            let cb_idx = ((cfg >> 32) & 0xFF) as u32;
            let flags = ((cfg >> 40) & 0xFF) as u32;
            kprintln!("smmu: TBU[{}] sid={}..{} cb={} flags={:#x}", 
                     i, sid_start, sid_start.saturating_add(sid_count.saturating_sub(1)), cb_idx, flags);
            
            if sid >= sid_start && sid < sid_start + sid_count {
                kprintln!("smmu: SID {} maps to CB {}", sid, cb_idx);
                // Настраиваем этот контекстный банк
                let cb_off = smmu_reg::S_CB_BASE + (cb_idx as usize * smmu_reg::S_CB_STRIDE);
                
                // TCR: 4KB pages, 48-bit VA, 48-bit PA, TTBR0/1
                let tcr: u64 = (1 << 31) | (1 << 30) | (0x10 << 16) | (0x10 << 0) | (1 << 8) | (1 << 7);
                smmu_write64(cb_off + 0x00, tcr); // CB_TCR
                
                // CB_ACTLR: Force Bypass бит 0 (для SMMUv3)
                smmu_write64(cb_off + 0x20, 1); // CB_ACTLR bit 0 = FB
                
                // Инвалидируем TLB для этого CB
                smmu_write32(smmu_reg::S_TLBIALLH, 1 << cb_idx);
                udelay(100);
                
                kprintln!("smmu: CB {} configured for bypass", cb_idx);
                return true;
            }
        }
    }
    
    false
}

/// Probe SMMU драйвера
fn smmu_probe(dev: &PlatformDevice) -> Result<(), DriverError> {
    if SMMU_INIT.swap(true, Ordering::AcqRel) {
        return Err(DriverError::AlreadyExists);
    }

    let (base, size) = dev.get_resource_mem(0).ok_or(DriverError::NoMemory)?;
    *SMMU_BASE.lock() = Some(base);

    kprintln!("smmu: Apps SMMU (qsmmu-v500) at {:#x}, size={:#x}", base, size);

    // Включаем SMMU если не включён
    let cr0 = smmu_read32(0x0020); // S_CR0
    if (cr0 & 1) == 0 {
        smmu_write32(0x0020, cr0 | 1);
        udelay(100);
        kprintln!("smmu: enabled");
    }

    // Временно: пробуем настроить bypass для UFS SID 0x60
    // Реальная реализация должна это делать по требованию UFS драйвера
    let _ = configure_cb_for_sid(0x60);

    kprintln!("smmu: probe ok");
    Ok(())
}

/// SMMU драйвер
static SMMU_DRIVER: Driver = Driver {
    name: "smmu-qcom-v500",
    compatible: ["qcom,qsmmu-v500", "", "", "", "", "", "", ""],
    compatible_n: 1,
    probe: smmu_probe,
    remove: None,
};

/// Регистрация драйвера (явный вызов вместо .init_array)
pub fn register_driver() -> Result<(), DriverError> {
    platform::driver_register(&SMMU_DRIVER)
}

// Публичная функция для UFS драйвера — запросить bypass для своего SID
pub fn smmu_request_bypass(sid: u32) -> bool {
    if !SMMU_INIT.load(Ordering::Acquire) {
        return false;
    }
    configure_cb_for_sid(sid)
}