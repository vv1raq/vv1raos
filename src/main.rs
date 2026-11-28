#![no_std]
#![no_main]

#[macro_use]
mod console;
mod aspace;
mod config;
mod drivers;
mod elf;
mod fdt;
mod gic;
mod initramfs;
mod ipc;
mod irq;
mod mem;
mod mmu;
mod platform;
mod pmm;
mod proc;
mod sched;
mod shm;
mod smp;
mod sync;
mod obj;
mod port;
mod syscall;
mod timer;
mod uart;
mod wdt;

use core::arch::{asm, global_asm};
use core::panic::PanicInfo;

global_asm!(include_str!("boot.S"));

fn halt() -> ! {
    loop {
        unsafe { asm!("wfe", options(nomem, nostack)) };
    }
}

fn current_el() -> u64 {
    let v: u64;
    unsafe { asm!("mrs {}, CurrentEL", out(reg) v, options(nomem, nostack, preserves_flags)) };
    v >> 2
}

fn mpidr() -> u64 {
    let v: u64;
    unsafe { asm!("mrs {}, mpidr_el1", out(reg) v, options(nomem, nostack, preserves_flags)) };
    v
}

fn must<T>(v: Option<T>, what: &str) -> T {
    match v {
        Some(x) => x,
        None => {
            kprintln!("setup failed: {}", what);
            halt()
        }
    }
}

fn pmm_selftest() {
    match (pmm::alloc_page(), pmm::alloc_page(), pmm::alloc_page()) {
        (Some(a), Some(b), Some(c)) => {
            let zeroed = unsafe { core::slice::from_raw_parts(a as *const u8, pmm::PAGE_SIZE) }
                .iter()
                .all(|&x| x == 0);
            unsafe { (b as *mut u64).write(0xdead_beef_cafe_f00d) };
            let readback = unsafe { (b as *const u64).read() } == 0xdead_beef_cafe_f00d;
            pmm::free_page(b);
            let d = pmm::alloc_page();
            let reused = d == Some(b);
            let rezeroed = unsafe { (b as *const u64).read() } == 0;
            kprintln!("PMM  pages {:#x} {:#x} {:#x}", a, b, c);
            kprintln!(
                "PMM  selftest: zero={} rw={} reuse={} rezero={}",
                zeroed, readback, reused, rezeroed
            );
            pmm::free_page(a);
            pmm::free_page(c);
            if let Some(d) = d {
                pmm::free_page(d);
            }
        }
        _ => kprintln!("PMM  selftest: out of memory"),
    }
}

/// Вызывает функции-конструкторы из .init_array — регистрацию драйверов
/// (см. #[link_section = ".init_array"] в src/drivers/*).
unsafe fn run_init_array() {
    extern "C" {
        static __init_array_start: u8;
        static __init_array_end: u8;
    }
    let start = &__init_array_start as *const u8 as usize;
    let end = &__init_array_end as *const u8 as usize;
    kprintln!("init_array: start={:#x} end={:#x} count={}", start, end, (end - start) / 8);
    let mut p = start;
    while p + 8 <= end {
        let f: fn() = core::mem::transmute((p as *const usize).read());
        kprintln!("init_array: calling {:#x}", p);
        f();
        p += 8;
    }
}

#[no_mangle]
pub extern "C" fn kernel_main(dtb: usize) -> ! {
    smp::init_boot_cpu();
    // Регистрация платформенных драйверов (gcc/spmi/pmic/uart/ufs) до
    // platform_init: platform_populate фильтрует узлы DT по зарегистрированным
    // драйверам.
    unsafe { run_init_array() };
    let info = fdt::parse(dtb);

    // Отключаем APSS watchdog до любых потенциально зависающих действий
    // (на QEMU virt узла нет — skip).
    if let Some(w) = info.as_ref().and_then(|i| i.wdt) {
        wdt::disable(w);
        crate::drivers::vibro::indicate(crate::drivers::vibro::BootStage::WdtOff);
    }

    let uart = info.as_ref().and_then(|i| i.pl011).map(|base| {
        let u = uart::Pl011::new(base);
        unsafe { u.init() };
        u
    });
    // GENI SE UART — консоль реального железа (SM6375), ранний вывод
    if let Some(g) = info.as_ref().and_then(|i| i.geni_uart) {
        crate::drivers::geni_uart::early_init(g);
    }

    console::init(uart, info.as_ref().and_then(|i| i.geni_uart));

    kprintln!("mk: stage 8 boot");
    kprintln!("EL{}  MPIDR={:#x}", current_el(), mpidr());
    let Some(info) = info else {
        console::emergency_write(b"[boot] DTB PARSE FAILED\n");
        kprintln!("DTB invalid, x0={:#x}", dtb);
        halt()
    };
    kprintln!("DTB  @ {:#x} ({} bytes)", info.dtb, info.dtb_size);
    for &(b, s) in &info.ram[..info.ram_n] {
        kprintln!("RAM  {:#x} + {:#x}", b, s);
    }
    kprintln!("no-map regions: {}, reserved total: {}", info.no_map_n, info.reserved_n);

    // ---- MMU ----
    let mut mmio = [(0usize, 0usize); 4 + fdt::MAX_VIRTIO_MMIO];
    let mut np = 0;
    for &m in [info.pl011].iter().flatten() {
        mmio[np] = (m, 0x1000);
        np += 1;
    }
    if let Some(g) = info.geni_uart {
        // GENI SE UART: узел в DTB обычно 0x4000, маппим столько же
        mmio[np] = (g, 0x4000);
        np += 1;
    }
    if let Some(g) = info.gic {
        mmio[np] = (g.dist, if g.dist_size != 0 { g.dist_size } else { 0x10000 });
        np += 1;
        mmio[np] = (g.redist, if g.redist_size != 0 { g.redist_size } else { 0x20000 });
        np += 1;
    }
    for i in 0..info.virtio_mmio_n {
        let (base, size, _irq) = info.virtio_mmio[i];
        let sz = if size != 0 { size } else { 0x200 };
        if np < mmio.len() {
            mmio[np] = (base, sz);
            np += 1;
        }
    }
    // ---- Физический аллокатор (нужен до MMU, т.к. таблицы страниц аллоцируются из PMM) ----
    let (ks, ke) = mmu::kernel_range();
    let mut res = [(0usize, 0usize); 128];
    let mut rn = 0;
    res[rn] = (ks, ke);
    rn += 1;
    res[rn] = (info.dtb, info.dtb + info.dtb_size);
    rn += 1;
    for &(b, s) in &info.no_map[..info.no_map_n] {
        if rn < res.len() {
            res[rn] = (b as usize, (b + s) as usize);
            rn += 1;
        }
    }
    // ВСЯ reserved-memory (не только no-map) — прошивка живёт и в обычных
    // резервах (smem, cont_splash, dma-pools на SM6375).
    for &(b, s) in &info.reserved[..info.reserved_n] {
        if rn < res.len() {
            res[rn] = (b as usize, (b + s) as usize);
            rn += 1;
        }
    }
    let ram = &info.ram[..info.ram_n];
    let main_range = ram
        .iter()
        .find(|r| (r.0 as usize) <= ks && ke <= (r.0 + r.1) as usize)
        .or(ram.first());
    let Some(&(rb, rs)) = main_range else {
        kprintln!("no RAM ranges in DTB");
        halt()
    };
    pmm::init(core::cmp::max(ke, rb as usize), (rb + rs) as usize, &res[..rn]);
    kprintln!(
        "PMM  kernel {:#x}..{:#x}, free ~{} MiB",
        ks,
        ke,
        pmm::free_pages_estimate() * pmm::PAGE_SIZE / (1024 * 1024)
    );

    // ---- MMU ----
    let (root, _kernel_asid) = unsafe {
        mmu::enable_kernel_mmu(&info.ram[..info.ram_n], &info.no_map[..info.no_map_n], &mmio[..np])
    };
    kprintln!("MMU  on, TTBR0={:#x}", root);
    pmm_selftest();

    // Инициализация cleanup-коллбэков для объектов (shm, ports, endpoints).
    shm::init();
    port::init();
    ipc::init();

    // ---- Platform device model (Qualcomm SM6375) ----
    // Парсинг DT и регистрация platform устройств
    platform::platform_init(info.dtb);
    // Пробирование драйверов: GCC -> SPMI -> PMIC -> UART -> UFS
    platform::platform_drivers_init();

    // Теперь UFS готов — загружаем initramfs (сначала UFS ESP, потом DTB fallback)
    initramfs::init(&info);
    crate::drivers::vibro::indicate(crate::drivers::vibro::BootStage::InitramfsOk);

    // Инициализация pinmux (TLMM) для UART пинов, если загрузчик не настроил
    if let Some(tlmm_base) = info.pinctrl {
        crate::drivers::pinmux::tlmm_init(tlmm_base);
        let _ = crate::drivers::pinmux::tlmm_setup_console_uart();
    }

    // Дамп статуса тактов для отладки
    crate::drivers::clock_verify::dump_clock_status();

    // ---- GICv3 ----
    let Some(g) = info.gic else {
        kprintln!("GIC  GICv3 node not found in DTB");
        halt()
    };
    if let Err(e) = unsafe { gic::init(g.dist, g.redist, g.redist_size) } {
        kprintln!("GIC  init failed: {}", e);
        halt()
    }
    kprintln!("GIC  v3 up: GICD={:#x} GICR={:#x}", g.dist, g.redist);

    // Самотест SGI: INTID 1 -> этот же CPU
    gic::enable_irq(1, 0x80);
    unsafe { asm!("msr daifclr, #2", options(nomem, nostack)) }; // размаскировать IRQ
    gic::send_sgi_self(1);
    let mut spins = 0u32;
    while irq::SGI_COUNT.load(core::sync::atomic::Ordering::Relaxed) == 0 && spins < 10_000_000 {
        spins += 1;
    }
    let sgi_ok = irq::SGI_COUNT.load(core::sync::atomic::Ordering::Relaxed) == 1;
    kprintln!("GIC  SGI selftest: {}", if sgi_ok { "OK" } else { "FAIL" });

    // ---- Generic Timer: тик 100 Гц ----
    let tid = info.timer_intid.unwrap_or(30);
    let f = timer::freq();
    kprintln!("TIMER cntfrq={} Hz, INTID={}", f, tid);
    if f == 0 {
        kprintln!("TIMER CNTFRQ_EL0 == 0 (firmware did not set it)");
        halt()
    }
    gic::enable_irq(tid, 0x80);
    // Планировщик и период таймера должны быть готовы до CPU_ON: вторичные
    // ядра сразу заводят свои idle-потоки и локальные таймеры.
    sched::init();
    timer::start_periodic(tid, 100);

    // ---- SMP: PSCI CPU_ON, per-CPU, IPI, спинлок ----
    smp::init(&info);
    smp::selftest();

    // Ядро завершает низкоуровневую инициализацию и передаёт управление
    // первой пользовательской программе — init (PID 1). Дальше ОС живёт в EL0:
    // init запускает сервисы/тесты/оболочку через sys_spawn и ждёт их через sys_wait.
    let init = must(proc::spawn("init", 6, 0), "init");
    kprintln!(
        "BOOT kernel ready: init pid={}, timer 100 Hz, CPUs online={}",
        init,
        smp::online_count()
    );
    // Передаём управление первому готовому потоку (кооперативный запуск).
    sched::start_first();
    // Сюда возвращается idle-поток CPU0, когда на него переключаются.
    sched::idle_loop(0);
}

#[no_mangle]
pub extern "C" fn exception_handler(idx: u64, esr: u64, elr: u64, far: u64, lr: u64, spsr: u64, fp: u64) -> ! {
    let pid = sched::current_pid();
    let tid = sched::current_tid();
    let sp_el0: u64;
    let sp_el1: u64;
    unsafe { core::arch::asm!("mrs {}, sp_el0", out(reg) sp_el0, options(nomem, nostack, preserves_flags)); }
    unsafe { core::arch::asm!("mov {}, sp", out(reg) sp_el1, options(nomem, nostack, preserves_flags)); }
    console::emergency_write(b"\n[EXC]\n");
    if !console::console_free() {
        console::emergency_write(b"[EXC] console locked, halting\n");
        halt();
    }
    kprintln!("EXCEPTION vec={} ESR={:#x} ELR={:#x} FAR={:#x} LR={:#x} SPSR={:#x} pid={} tid={} SP_EL0={:#x} SP_EL1={:#x} FP={:#x}", idx, esr, elr, far, lr, spsr, pid, tid, sp_el0, sp_el1, fp);
    // frame pointer walk
    let mut cur = fp as usize;
    for i in 0..16 {
        if cur < 0x4000_0000 || cur > 0x4800_0000 || cur & 7 != 0 { break; }
        let prev = unsafe { core::ptr::read_volatile(cur as *const usize) };
        let ret = unsafe { core::ptr::read_volatile((cur + 8) as *const usize) };
        kprintln!("  fp[{i}] fp={:#x} lr={:#x}", cur, ret);
        if prev == 0 || prev <= cur { break; }
        cur = prev;
    }
    // dump a few words from SP_EL1
    let sp1 = sp_el1 as usize;
    for i in 0..16 {
        let addr = sp1 + i * 8;
        let val = unsafe { core::ptr::read_volatile(addr as *const u64) };
        kprintln!("  [sp1+{:#x}] = {:#x}", i * 8, val);
    }
    // dump a few words from user stack
    let sp = sp_el0 as usize;
    for i in 0..8 {
        let addr = sp + i * 8;
        let val = if addr >= 0x4000_0000_0000 && addr < 0x4000_4000_0000 {
            unsafe { core::ptr::read_volatile(addr as *const u64) }
        } else { 0 };
        kprintln!("  [sp+{:#x}] = {:#x}", i * 8, val);
    }
    if let Some((root, asid)) = proc::root_of(pid) {
        let va = sp & !0xfff;
        kprintln!("  root={:#x} asid={} va={:#x}", root, asid, va);
        unsafe {
            let l0 = root as *const u64;
            let e0 = l0.add((va>>39)&0x1ff).read();
            kprintln!("  l0e={:#x}", e0);
            if e0 & 3 == 3 {
                let l1 = (e0 & 0x0000_FFFF_FFFF_F000u64) as *const u64;
                let e1 = l1.add((va>>30)&0x1ff).read();
                kprintln!("  l1e={:#x}", e1);
                if e1 & 3 == 3 {
                    let l2 = (e1 & 0x0000_FFFF_FFFF_F000u64) as *const u64;
                    let e2 = l2.add((va>>21)&0x1ff).read();
                    kprintln!("  l2e={:#x}", e2);
                    if e2 & 3 == 3 {
                        let l3 = (e2 & 0x0000_FFFF_FFFF_F000u64) as *const u64;
                        let e3 = l3.add((va>>12)&0x1ff).read();
                        kprintln!("  l3e={:#x}", e3);
                    }
                }
            }
        }
    }
    for i in 1..=6 {
        if let Some((r,a)) = proc::root_of(i) { kprintln!("  pid{} root={:#x} asid={} alive={:?}", i, r, a, proc::exit_status(i)); } else { kprintln!("  pid{} none", i); }
    }
    halt()
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    console::emergency_write(b"\nPANIC!\n");
    // kprintln! берёт лок консоли: если паника случилась под ним — не виснем.
    if console::console_free() {
        kprintln!("PANIC: {}", info);
    }
    halt()
}
