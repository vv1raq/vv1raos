//! UFS Host Controller (UFSHCI 3.x) для Qualcomm SM6375 (Blair).
//!
//! Режимы запуска:
//! * takeover — загрузчик (ABL/XBL) уже поднял контроллер и линк (HCE=1 и
//!   HCS.DP=1). PHY, HCE и link startup не трогаем, только ставим свои списки
//!   запросов. Основной путь для `fastboot boot`.
//! * full init — HCE-цикл, PHY, DME_LINKSTARTUP. Последовательность PHY НЕ
//!   проверена (нужны таблицы QMP v4 из Linux phy-qcom-qmp-ufs), на железе
//!   вероятнее всего не заработает.
//!
//! Структуры (UTRD, UPIU, PRDT) собраны по UFSHCI 3.0 и Linux ufshci.h/ufs.h
//! по памяти. На реальном железе не проверялись.
//!
//! Допущения: идентичный маппинг (VA == PA) для статических буферов, ядро
//! замаплено Normal WB, поэтому вокруг DMA делается `dc civac`.

use crate::platform::{self, Driver, DriverError, PlatformDevice};
use crate::sync::SpinLock;
use crate::timer;
use core::arch::asm;
use core::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, AtomicUsize, Ordering};

/// Принудительно делать полный init (HCE + PHY + link startup), даже если
/// загрузчик уже поднял контроллер.
const FORCE_FULL_INIT: bool = false;
/// LUN, с которого читаем данные (GPT и т.п.).
const DEFAULT_LUN: u8 = 0;
const PHY_BASE_DEFAULT: usize = 0x4807000;

static HC_BASE: AtomicUsize = AtomicUsize::new(0);
static PHY_BASE: AtomicUsize = AtomicUsize::new(0);
static PROBED: AtomicBool = AtomicBool::new(false);
/// Выставляется только когда контроллер и устройство полностью готовы.
static READY: AtomicBool = AtomicBool::new(false);
static BLOCK_SIZE: AtomicU32 = AtomicU32::new(0);
static CAPACITY_BLOCKS: AtomicU64 = AtomicU64::new(0);

// ---- Регистры HC (UFSHCI) ----
const REG_IS: usize = 0x20;
const REG_IE: usize = 0x24;
const REG_HCS: usize = 0x30;
const REG_HCE: usize = 0x34;
const REG_UTRIACR: usize = 0x4C;
const REG_UTRLBA: usize = 0x50;
const REG_UTRLBAU: usize = 0x54;
const REG_UTRLDBR: usize = 0x58;
const REG_UTRLCLR: usize = 0x5C;
const REG_UTRLRSR: usize = 0x60;
const REG_UTMRLBA: usize = 0x70;
const REG_UTMRLBAU: usize = 0x74;
const REG_UTMRLRSR: usize = 0x80;
const REG_UICCMD: usize = 0x90;
const REG_UCMDARG1: usize = 0x94;
const REG_UCMDARG2: usize = 0x98;
const REG_UCMDARG3: usize = 0x9C;

// HCS
const HCS_DP: u32 = 1 << 0;
const HCS_UTRLRDY: u32 = 1 << 1;
const HCS_UTMRLRDY: u32 = 1 << 2;
const HCS_UCRDY: u32 = 1 << 3;

// IS / IE
const IS_UTRCS: u32 = 1 << 0;
const IS_UE: u32 = 1 << 2;
const IS_ULSS: u32 = 1 << 8;
const IS_UCCS: u32 = 1 << 10;
const IS_DFES: u32 = 1 << 11;
const IS_HCFES: u32 = 1 << 16;
const IS_SBFES: u32 = 1 << 17;
const IS_FATAL: u32 = IS_DFES | IS_HCFES | IS_SBFES;
const IE_MASK: u32 = IS_UTRCS | IS_UE | IS_ULSS | IS_UCCS | IS_FATAL;

const UIC_DME_LINKSTARTUP: u32 = 0x16;

// ---- UTRD / UPIU ----
const CT_SCSI: u32 = 0;
const CT_DEV_MANAGE: u32 = 2;
const UTRD_INTR: u32 = 1 << 24;
const UTRD_DD_H2D: u32 = 1 << 25;
const UTRD_DD_D2H: u32 = 1 << 26;
const OCS_INVALID: u32 = 0x0F;

const UPIU_NOP_OUT: u8 = 0x00;
const UPIU_COMMAND: u8 = 0x01;
const UPIU_QUERY_REQ: u8 = 0x16;
const UPIU_NOP_IN: u8 = 0x20;
const UPIU_RESPONSE: u8 = 0x21;
const UPIU_QUERY_RSP: u8 = 0x36;

const QUERY_READ_REQ: u8 = 0x01;
const QUERY_WRITE_REQ: u8 = 0x81;
const QUERY_OP_READ_FLAG: u8 = 0x05;
const QUERY_OP_SET_FLAG: u8 = 0x06;
const QUERY_IDN_FDEVICEINIT: u8 = 0x01;

const NUM_SLOTS: usize = 32;
const UCD_CMD_OFF: usize = 0;
const UCD_RSP_OFF: usize = 512;
const UCD_RSP_LEN: usize = 512;
const UCD_PRDT_OFF: usize = 1024;
const DATA_BUF_SIZE: usize = 16 * 1024;

const SCSI_TIMEOUT_US: u64 = 2_000_000;

/// UTP Transfer Request Descriptor: 8 dword (32 байта).
#[repr(C)]
#[derive(Clone, Copy)]
struct Utrd {
    dw: [u32; 8],
}

/// Запись PRDT: DW0/DW1 — адрес, DW2 — резерв, DW3 — (байт - 1).
#[repr(C)]
#[derive(Clone, Copy)]
struct Prdt {
    dw: [u32; 4],
}

/// UTP Command Descriptor: Command UPIU, Response UPIU, PRDT.
#[repr(C, align(128))]
struct Ucd {
    cmd: [u8; 512],
    rsp: [u8; 512],
    prdt: [Prdt; 4],
}

/// Всё, что видит DMA-движок контроллера.
#[repr(C, align(4096))]
struct Dma {
    utrl: [Utrd; NUM_SLOTS], // 1 КиБ, выравнивание 1 КиБ
    utmrl: [u8; 1024],       // список task management (не используем)
    ucd: Ucd,
    data: [u8; DATA_BUF_SIZE],
}

const _: () = assert!(core::mem::offset_of!(Dma, utmrl) % 1024 == 0);
const _: () = assert!(core::mem::offset_of!(Dma, ucd) % 128 == 0);
const _: () = assert!(core::mem::offset_of!(Dma, data) % 64 == 0);
const _: () = assert!(core::mem::offset_of!(Ucd, rsp) == UCD_RSP_OFF);
const _: () = assert!(core::mem::offset_of!(Ucd, prdt) == UCD_PRDT_OFF);

const UTRD_ZERO: Utrd = Utrd { dw: [0; 8] };
const PRDT_ZERO: Prdt = Prdt { dw: [0; 4] };

static DMA: SpinLock<Dma> = SpinLock::new(Dma {
    utrl: [UTRD_ZERO; NUM_SLOTS],
    utmrl: [0; 1024],
    ucd: Ucd {
        cmd: [0; 512],
        rsp: [0; 512],
        prdt: [PRDT_ZERO; 4],
    },
    data: [0; DATA_BUF_SIZE],
});

#[derive(Clone, Copy, PartialEq)]
enum Dir {
    None,
    In,
    Out,
}

// ---- Низкоуровневые помощники ----

fn rd(off: usize) -> u32 {
    let b = HC_BASE.load(Ordering::Relaxed);
    if b == 0 {
        return 0;
    }
    unsafe { core::ptr::read_volatile((b + off) as *const u32) }
}

fn wr(off: usize, val: u32) {
    let b = HC_BASE.load(Ordering::Relaxed);
    if b == 0 {
        return;
    }
    unsafe { core::ptr::write_volatile((b + off) as *mut u32, val) }
}

fn phy_wr(off: usize, val: u32) {
    let b = PHY_BASE.load(Ordering::Relaxed);
    if b == 0 {
        return;
    }
    unsafe { core::ptr::write_volatile((b + off) as *mut u32, val) }
}

fn counter_hz() -> u64 {
    let f = timer::freq();
    if f == 0 {
        19_200_000
    } else {
        f
    }
}

/// Задержка по системному счётчику (не по числу итераций цикла).
fn udelay(us: u64) {
    let ticks = us * counter_hz() / 1_000_000;
    let start = timer::counter();
    while timer::counter().wrapping_sub(start) < ticks {
        core::hint::spin_loop();
    }
}

/// Опрашивает условие до истечения таймаута (мкс).
fn wait_for<F: Fn() -> bool>(timeout_us: u64, cond: F) -> bool {
    let limit = timeout_us * counter_hz() / 1_000_000 + 1;
    let start = timer::counter();
    loop {
        if cond() {
            return true;
        }
        if timer::counter().wrapping_sub(start) > limit {
            return cond();
        }
        core::hint::spin_loop();
    }
}

/// Clean+invalidate по VA для DMA-буферов (ядро замаплено Normal WB).
fn dcache_flush(addr: usize, len: usize) {
    if len == 0 {
        return;
    }
    let ctr: u64;
    unsafe { asm!("mrs {}, ctr_el0", out(reg) ctr, options(nomem, nostack, preserves_flags)) };
    let line = 4usize << ((ctr >> 16) & 0xF);
    let end = addr + len;
    let mut p = addr & !(line - 1);
    unsafe { asm!("dsb sy", options(nostack)) };
    while p < end {
        unsafe { asm!("dc civac, {}", in(reg) p, options(nostack)) };
        p += line;
    }
    unsafe { asm!("dsb sy", options(nostack)) };
}

// ---- UIC ----

fn uic_cmd(cmd: u32, a1: u32, a2: u32, a3: u32) -> Result<u32, DriverError> {
    // UCRDY = 1 — контроллер готов принять UIC-команду.
    if !wait_for(1_000_000, || rd(REG_HCS) & HCS_UCRDY != 0) {
        return Err(DriverError::Busy);
    }
    wr(REG_IS, IS_UCCS);
    wr(REG_UCMDARG1, a1);
    wr(REG_UCMDARG2, a2);
    wr(REG_UCMDARG3, a3);
    wr(REG_UICCMD, cmd);
    if !wait_for(500_000, || rd(REG_IS) & IS_UCCS != 0) {
        return Err(DriverError::Timeout);
    }
    wr(REG_IS, IS_UCCS);
    // Код результата — младший байт UCMDARG2, 0 = успех.
    if rd(REG_UCMDARG2) & 0xFF != 0 {
        return Err(DriverError::IoError);
    }
    Ok(rd(REG_UCMDARG3))
}

fn link_startup() -> Result<(), DriverError> {
    for attempt in 0..3 {
        match uic_cmd(UIC_DME_LINKSTARTUP, 0, 0, 0) {
            Ok(_) => {
                wr(REG_IS, IS_ULSS);
                if rd(REG_HCS) & HCS_DP != 0 {
                    kprintln!("ufs: link startup complete");
                    return Ok(());
                }
                kprintln!("ufs: link is up but no device present");
                return Err(DriverError::NotReady);
            }
            Err(_) => kprintln!("ufs: link startup attempt {} failed", attempt),
        }
    }
    Err(DriverError::Timeout)
}

// ---- Инициализация контроллера ----

/// Контроллер уже включён и линк поднят загрузчиком.
fn hc_is_up() -> bool {
    rd(REG_HCE) & 1 != 0 && rd(REG_HCS) & HCS_DP != 0
}

/// Останавливает списки запросов, оставшиеся от загрузчика.
fn lists_stop() {
    if !wait_for(100_000, || rd(REG_UTRLDBR) == 0) {
        kprintln!("ufs: doorbell busy ({:#x}), clearing", rd(REG_UTRLDBR));
        wr(REG_UTRLCLR, 0); // 0 в бите = очистить слот
        let _ = wait_for(100_000, || rd(REG_UTRLDBR) == 0);
    }
    wr(REG_UTRLRSR, 0);
    wr(REG_UTMRLRSR, 0);
}

fn hce_cycle() -> Result<(), DriverError> {
    wr(REG_HCE, 0);
    if !wait_for(10_000, || rd(REG_HCE) & 1 == 0) {
        return Err(DriverError::Timeout);
    }
    udelay(1_000);
    wr(REG_HCE, 1);
    udelay(100);
    if !wait_for(50_000, || rd(REG_HCE) & 1 != 0) {
        return Err(DriverError::Timeout);
    }
    wr(REG_IE, IE_MASK);
    Ok(())
}

/// ВНИМАНИЕ: последовательность не проверена, реальные таблицы QMP v4 нужно
/// взять из Linux (phy-qcom-qmp-ufs). Нужна только в режиме full init.
fn phy_init_unverified() {
    let b = PHY_BASE.load(Ordering::Relaxed);
    if b == 0 {
        return;
    }
    // PHY не входит в ресурс HC (0x4804000 + 0x3000) — маппим явно.
    crate::mmu::map_device_identity(b & !0xfff, (b & !0xfff) + 0x1000);
    phy_wr(0x008, 1); // SW_RESET
    udelay(10);
    phy_wr(0x008, 0);
    udelay(10);
    phy_wr(0x004, 0); // POWER_DOWN_CONTROL
    udelay(10);
    phy_wr(0x044, 0x3); // TX_LANE_ENABLE
    phy_wr(0x084, 0x3); // RX_LANE_ENABLE
    phy_wr(0x010, 0x02); // PHY_MODE
    phy_wr(0x000, 0x1); // PHY_START
    udelay(100);
    kprintln!("ufs: PHY init done (unverified)");
}

fn full_init() -> Result<(), DriverError> {
    phy_init_unverified();
    hce_cycle()?;
    kprintln!("ufs: HC enabled");
    link_startup()
}

/// Отдаёт контроллеру наши списки запросов и запускает их.
fn make_operational(d: &Dma) -> Result<(), DriverError> {
    wr(REG_UTRIACR, 0); // без агрегации прерываний
    wr(REG_IE, IE_MASK);
    wr(REG_IS, rd(REG_IS)); // сбросить старые статусы (W1C)

    let utrl = d.utrl.as_ptr() as usize as u64;
    let utmrl = d.utmrl.as_ptr() as usize as u64;
    wr(REG_UTRLBA, utrl as u32);
    wr(REG_UTRLBAU, (utrl >> 32) as u32);
    wr(REG_UTMRLBA, utmrl as u32);
    wr(REG_UTMRLBAU, (utmrl >> 32) as u32);

    let both = HCS_UTRLRDY | HCS_UTMRLRDY;
    if !wait_for(100_000, || (rd(REG_HCS) & both) == both) {
        kprintln!("ufs: warning: lists not ready, HCS={:#x}", rd(REG_HCS));
    }
    wr(REG_UTMRLRSR, 1);
    wr(REG_UTRLRSR, 1);
    Ok(())
}

// ---- Выполнение запросов ----

fn rsp(d: &Dma, i: usize) -> u8 {
    unsafe { core::ptr::read_volatile(&d.ucd.rsp[i]) }
}

/// Кладёт запрос в слот 0 и ждёт завершения. Command UPIU (первые 32 байта
/// `d.ucd.cmd`) и данные для Dir::Out заполняет вызывающий.
fn exec(
    d: &mut Dma,
    cmd_type: u32,
    dir: Dir,
    data_len: usize,
    timeout_us: u64,
) -> Result<(), DriverError> {
    if data_len > DATA_BUF_SIZE {
        return Err(DriverError::InvalidArg);
    }
    if dir != Dir::None && (data_len == 0 || data_len % 4 != 0) {
        return Err(DriverError::InvalidArg);
    }

    d.ucd.rsp.fill(0);
    let ucd_pa = &d.ucd as *const Ucd as usize as u64;
    let data_pa = d.data.as_ptr() as usize as u64;

    let mut dw0 = (cmd_type << 28) | UTRD_INTR;
    match dir {
        Dir::In => dw0 |= UTRD_DD_D2H,
        Dir::Out => dw0 |= UTRD_DD_H2D,
        Dir::None => {}
    }
    let prdt_n: u32 = if dir == Dir::None { 0 } else { 1 };
    d.utrl[0] = Utrd {
        dw: [
            dw0,
            0,
            OCS_INVALID,
            0,
            ucd_pa as u32,
            (ucd_pa >> 32) as u32,
            // DW6: длина ответа (младшие 16) и смещение (старшие 16), в dword
            (((UCD_RSP_OFF / 4) as u32) << 16) | ((UCD_RSP_LEN / 4) as u32),
            // DW7: число записей PRDT (младшие 16) и смещение (старшие 16), в dword
            (((UCD_PRDT_OFF / 4) as u32) << 16) | prdt_n,
        ],
    };
    if dir != Dir::None {
        d.ucd.prdt[0] = Prdt {
            dw: [data_pa as u32, (data_pa >> 32) as u32, 0, (data_len as u32) - 1],
        };
    }

    dcache_flush(d.utrl.as_ptr() as usize, core::mem::size_of::<[Utrd; NUM_SLOTS]>());
    dcache_flush(&d.ucd as *const Ucd as usize, core::mem::size_of::<Ucd>());
    if dir != Dir::None {
        dcache_flush(data_pa as usize, data_len);
    }

    wr(REG_IS, IS_UTRCS);
    wr(REG_UTRLDBR, 1);
    if !wait_for(timeout_us, || rd(REG_UTRLDBR) & 1 == 0) {
        wr(REG_UTRLCLR, !1u32); // 0 в бите 0 = очистить слот 0
        return Err(DriverError::Timeout);
    }

    let is_val = rd(REG_IS);
    wr(REG_IS, is_val & IS_UTRCS);
    if is_val & IS_FATAL != 0 {
        kprintln!("ufs: fatal error, IS={:#x}", is_val);
        return Err(DriverError::IoError);
    }

    // Забрать то, что записал контроллер.
    dcache_flush(d.utrl.as_ptr() as usize, core::mem::size_of::<[Utrd; NUM_SLOTS]>());
    dcache_flush(&d.ucd.rsp as *const [u8; 512] as usize, UCD_RSP_LEN);
    if dir == Dir::In {
        dcache_flush(data_pa as usize, data_len);
    }
    let ocs = unsafe { core::ptr::read_volatile(&d.utrl[0].dw[2]) } & 0xFF;
    if ocs != 0 {
        kprintln!("ufs: request failed, OCS={:#x}", ocs);
        return Err(DriverError::IoError);
    }
    Ok(())
}

/// SCSI-команда. Возвращает SCSI status (0 = GOOD, 2 = CHECK CONDITION).
fn scsi_cmd(d: &mut Dma, lun: u8, cdb: &[u8], dir: Dir, data_len: usize) -> Result<u8, DriverError> {
    if cdb.is_empty() || cdb.len() > 16 {
        return Err(DriverError::InvalidArg);
    }
    d.ucd.cmd[UCD_CMD_OFF..UCD_CMD_OFF + 32].fill(0);
    d.ucd.cmd[0] = UPIU_COMMAND;
    d.ucd.cmd[1] = match dir {
        Dir::In => 0x40,  // READ
        Dir::Out => 0x20, // WRITE
        Dir::None => 0x00,
    };
    d.ucd.cmd[2] = lun;
    d.ucd.cmd[3] = 0; // task tag = слот 0
    d.ucd.cmd[12..16].copy_from_slice(&(data_len as u32).to_be_bytes());
    d.ucd.cmd[16..16 + cdb.len()].copy_from_slice(cdb);

    exec(d, CT_SCSI, dir, data_len, SCSI_TIMEOUT_US)?;

    if rsp(d, 0) & 0x3F != UPIU_RESPONSE || rsp(d, 6) != 0 {
        return Err(DriverError::IoError);
    }
    Ok(rsp(d, 7))
}

fn nop_out(d: &mut Dma) -> Result<(), DriverError> {
    d.ucd.cmd[UCD_CMD_OFF..UCD_CMD_OFF + 32].fill(0);
    d.ucd.cmd[0] = UPIU_NOP_OUT;
    exec(d, CT_DEV_MANAGE, Dir::None, 0, 100_000)?;
    if rsp(d, 0) & 0x3F != UPIU_NOP_IN {
        return Err(DriverError::IoError);
    }
    Ok(())
}

/// Query: READ_FLAG / SET_FLAG. Возвращает значение флага.
fn query_flag(d: &mut Dma, func: u8, opcode: u8, idn: u8) -> Result<u8, DriverError> {
    d.ucd.cmd[UCD_CMD_OFF..UCD_CMD_OFF + 32].fill(0);
    d.ucd.cmd[0] = UPIU_QUERY_REQ;
    d.ucd.cmd[5] = func;
    d.ucd.cmd[12] = opcode;
    d.ucd.cmd[13] = idn;
    exec(d, CT_DEV_MANAGE, Dir::None, 0, 1_500_000)?;
    if rsp(d, 0) & 0x3F != UPIU_QUERY_RSP || rsp(d, 6) != 0 {
        return Err(DriverError::IoError);
    }
    Ok(rsp(d, 23) & 1)
}

/// NOP OUT, затем fDeviceInit (как ufshcd_complete_dev_init в Linux).
fn dev_init(d: &mut Dma) -> Result<(), DriverError> {
    let mut ok = false;
    for _ in 0..10 {
        if nop_out(d).is_ok() {
            ok = true;
            break;
        }
        udelay(10_000);
    }
    if !ok {
        kprintln!("ufs: NOP OUT failed");
        return Err(DriverError::Timeout);
    }
    query_flag(d, QUERY_WRITE_REQ, QUERY_OP_SET_FLAG, QUERY_IDN_FDEVICEINIT)?;
    for _ in 0..300 {
        if query_flag(d, QUERY_READ_REQ, QUERY_OP_READ_FLAG, QUERY_IDN_FDEVICEINIT)? == 0 {
            return Ok(());
        }
        udelay(5_000);
    }
    kprintln!("ufs: fDeviceInit did not clear");
    Err(DriverError::Timeout)
}

/// TEST UNIT READY с повторами (первый ответ может быть UNIT ATTENTION).
fn unit_ready(d: &mut Dma, lun: u8) -> bool {
    for _ in 0..10 {
        if let Ok(0) = scsi_cmd(d, lun, &[0u8; 6], Dir::None, 0) {
            return true;
        }
        udelay(20_000);
    }
    false
}

/// READ CAPACITY(10): (число блоков, размер блока).
fn read_capacity(d: &mut Dma, lun: u8) -> Result<(u64, u32), DriverError> {
    let cdb = [0x25u8, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    if scsi_cmd(d, lun, &cdb, Dir::In, 8)? != 0 {
        return Err(DriverError::IoError);
    }
    let last = u32::from_be_bytes([d.data[0], d.data[1], d.data[2], d.data[3]]);
    let bs = u32::from_be_bytes([d.data[4], d.data[5], d.data[6], d.data[7]]);
    if bs == 0 || last == u32::MAX {
        return Err(DriverError::NotSupported);
    }
    Ok((last as u64 + 1, bs))
}

// ---- Probe ----

fn ufs_probe(dev: &PlatformDevice) -> Result<(), DriverError> {
    if PROBED.swap(true, Ordering::AcqRel) {
        return Err(DriverError::AlreadyExists);
    }

    let (hc_base, hc_size) = dev.get_resource_mem(0).ok_or(DriverError::NoMemory)?;
    HC_BASE.store(hc_base, Ordering::Release);
    let phy = dev
        .get_resource_mem(1)
        .map(|(b, _)| b)
        .unwrap_or(PHY_BASE_DEFAULT);
    PHY_BASE.store(phy, Ordering::Release);
    kprintln!("ufs: HC at {:#x}, size={:#x}, PHY at {:#x}", hc_base, hc_size, phy);

    // Такты: ошибки не фатальны — загрузчик мог уже всё включить.
    for name in ["core", "bus", "phy"] {
        if let Some(id) = dev.get_resource_clock(name) {
            if crate::drivers::gcc_sm6375::clk_enable_by_id(id as u32).is_err() {
                kprintln!("ufs: clock '{}' enable failed", name);
            }
        }
    }
    if crate::drivers::clock_verify::verify_clock_tree().is_err() {
        kprintln!("ufs: clock tree verify failed");
    }

    crate::drivers::vibro::indicate(crate::drivers::vibro::BootStage::UfsInit);

    // SMMU bypass для нашего SID — до любых DMA-запросов.
    if let Some(sid) = dev.get_resource_sid() {
        let _ = crate::drivers::smmu::smmu_request_bypass(sid);
    }

    let takeover = !FORCE_FULL_INIT && hc_is_up();
    if takeover {
        kprintln!("ufs: HC already up (HCE=1, DP=1), taking over");
        lists_stop();
    } else {
        kprintln!("ufs: full HC init");
        full_init()?;
    }

    {
        let mut guard = DMA.lock();
        let d: &mut Dma = &mut *guard;
        make_operational(d)?;

        // При takeover устройство обычно уже инициализировано загрузчиком.
        let skip_dev_init = takeover && unit_ready(d, DEFAULT_LUN);
        if !skip_dev_init {
            dev_init(d)?;
            if !unit_ready(d, DEFAULT_LUN) {
                kprintln!("ufs: LUN {} not ready", DEFAULT_LUN);
                return Err(DriverError::NotReady);
            }
        }

        let (blocks, bs) = read_capacity(d, DEFAULT_LUN)?;
        BLOCK_SIZE.store(bs, Ordering::Release);
        CAPACITY_BLOCKS.store(blocks, Ordering::Release);
        READY.store(true, Ordering::Release);
        kprintln!("ufs: LUN {}: {} blocks x {} bytes", DEFAULT_LUN, blocks, bs);
    } // лок DMA отпущен: gpt_init читает через ufs_read_sectors

    kprintln!("ufs: probe ok");
    let _ = crate::drivers::gpt::gpt_init();
    Ok(())
}

static UFS_DRIVER: Driver = Driver {
    name: "ufs-qcom",
    compatible: ["qcom,ufshc", "qcom,ufs-qcom", "", "", "", "", "", ""],
    compatible_n: 2,
    probe: ufs_probe,
    remove: None,
};

/// Регистрация драйвера (явный вызов вместо .init_array)
pub fn register_driver() -> Result<(), DriverError> {
    platform::driver_register(&UFS_DRIVER)
}

// ---- Block device API ----

/// Размер логического блока LUN 0 в байтах (обычно 4096); 0, если UFS не готов.
pub fn ufs_block_size() -> usize {
    BLOCK_SIZE.load(Ordering::Acquire) as usize
}

/// Ёмкость LUN 0 в логических блоках.
pub fn ufs_get_capacity() -> Result<u64, DriverError> {
    if !READY.load(Ordering::Acquire) {
        return Err(DriverError::NotReady);
    }
    Ok(CAPACITY_BLOCKS.load(Ordering::Acquire))
}

/// Читает `count` логических блоков LUN 0 начиная с `lba`. Размер блока — см.
/// `ufs_block_size()` (на UFS обычно 4096, а не 512). `buf` должен вмещать
/// `count * block_size` байт.
pub fn ufs_read_sectors(lba: u64, count: u16, buf: &mut [u8]) -> Result<usize, DriverError> {
    if !READY.load(Ordering::Acquire) {
        return Err(DriverError::NotReady);
    }
    let bs = ufs_block_size();
    if bs == 0 || count == 0 {
        return Err(DriverError::InvalidArg);
    }
    let max_blocks = DATA_BUF_SIZE / bs;
    if max_blocks == 0 {
        return Err(DriverError::NotSupported);
    }
    let total = count as usize * bs;
    if buf.len() < total {
        return Err(DriverError::InvalidArg);
    }
    let end = lba.checked_add(count as u64).ok_or(DriverError::InvalidArg)?;
    if end > CAPACITY_BLOCKS.load(Ordering::Acquire) {
        return Err(DriverError::InvalidArg);
    }

    let mut guard = DMA.lock();
    let d: &mut Dma = &mut *guard;
    let mut done = 0usize;
    while done < count as usize {
        let n = core::cmp::min(count as usize - done, max_blocks);
        let cur = lba + done as u64; // < end <= 2^32, влезает в u32
        let mut cdb = [0u8; 10];
        cdb[0] = 0x28; // READ(10)
        cdb[2..6].copy_from_slice(&(cur as u32).to_be_bytes());
        cdb[7..9].copy_from_slice(&(n as u16).to_be_bytes());
        let len = n * bs;
        if scsi_cmd(d, DEFAULT_LUN, &cdb, Dir::In, len)? != 0 {
            return Err(DriverError::IoError);
        }
        buf[done * bs..done * bs + len].copy_from_slice(&d.data[..len]);
        done += n;
    }
    Ok(total)
}
