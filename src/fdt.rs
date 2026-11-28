//! Минимальный разбор Flattened Device Tree: RAM, PL011, GICv3, таймер, PSCI, virtio-mmio.

use crate::platform;

const FDT_MAGIC: u32 = 0xd00d_feed;
const TOK_BEGIN_NODE: u32 = 1;
const TOK_END_NODE: u32 = 2;
const TOK_PROP: u32 = 3;
const TOK_NOP: u32 = 4;
const TOK_END: u32 = 9;
const MAX_DEPTH: usize = 16;
const MAX_DTB_SIZE: usize = 8 * 1024 * 1024;
pub const MAX_RAM: usize = 16;
pub const MAX_NOMAP: usize = 64;
pub const MAX_CPUS: usize = 4;
pub const MAX_VIRTIO_MMIO: usize = 32;

#[derive(Clone, Copy)]
pub struct GicInfo {
    pub dist: usize,
    pub dist_size: usize,
    pub redist: usize,
    pub redist_size: usize,
}

/// PSCI: способ вызова (SMC/HVC) и function ID для CPU_ON.
#[derive(Clone, Copy)]
pub struct Psci {
    pub method: u8, // 0 = smc, 1 = hvc
    pub cpu_on: u64,
}

pub struct BootInfo {
    pub dtb: usize,
    pub dtb_size: usize,
    pub ram: [(u64, u64); MAX_RAM],
    pub ram_n: usize,
    /// Диапазоны из reserved-memory с `no-map` (их нельзя отображать в MMU).
    pub no_map: [(u64, u64); MAX_NOMAP],
    pub no_map_n: usize,
    /// ВСЕ диапазоны /reserved-memory (включая без no-map): Linux не выдаёт их
    /// аллокатору. На SM6375 там livetimer, cont_splash, dma-pools и др.
    pub reserved: [(u64, u64); MAX_NOMAP],
    pub reserved_n: usize,
    pub pl011: Option<usize>,
    pub gic: Option<GicInfo>,
    /// INTID физического таймера EL1 (из узла arm,armv8-timer).
    pub timer_intid: Option<u32>,
    /// Узел /psci (если есть).
    pub psci: Option<Psci>,
    /// (MPIDR, enable-method == "psci") для каждого /cpus/cpu@N.
    pub cpus: [(u64, bool); MAX_CPUS],
    pub cpus_n: usize,
    /// virtio-mmio узлы: (base, size, irq).
    pub virtio_mmio: [(usize, usize, u32); MAX_VIRTIO_MMIO],
    pub virtio_mmio_n: usize,
    /// Initramfs: физический адрес начала и конца (из /chosen/linux,initrd-start/end).
    pub initrd_start: Option<usize>,
    pub initrd_end: Option<usize>,
    /// GENI SE UART-консоль SM6375 (compatible "qcom,msm-geni-console", status ok).
    pub geni_uart: Option<usize>,
    /// APSS watchdog (compatible "qcom,msm-watchdog") — база для отключения.
    pub wdt: Option<usize>,
    /// TLMM pinctrl база (pinctrl@400000) для настройки пинов UART и др.
    pub pinctrl: Option<usize>,
}

struct Node<'a> {
    reg: Option<&'a [u8]>,
    is_pl011: bool,
    is_mem: bool,
    no_map: bool,
    is_gic3: bool,
    is_timer: bool,
    is_psci: bool,
    is_cpu: bool,
    is_virtio_mmio: bool,
    is_chosen: bool,
    is_geni_uart: bool,
    is_watchdog: bool,
    is_pinctrl: bool,
    /// Родитель — /reserved-memory
    is_reserved: bool,
    /// status == "disabled" — узел не трогаем.
    disabled: bool,
    enable_psci: bool,
    interrupts: Option<&'a [u8]>,
    method: Option<&'a [u8]>,
    cpu_on: Option<u64>,
}

impl<'a> Node<'a> {
    const fn new() -> Self {
        Self {
            reg: None,
            is_pl011: false,
            is_mem: false,
            no_map: false,
            is_gic3: false,
            is_timer: false,
            is_psci: false,
            is_cpu: false,
            is_virtio_mmio: false,
            is_chosen: false,
            is_geni_uart: false,
            is_watchdog: false,
            is_pinctrl: false,
            is_reserved: false,
            disabled: false,
            enable_psci: false,
            interrupts: None,
            method: None,
            cpu_on: None,
        }
    }
}

fn be32(b: &[u8], off: usize) -> u32 {
    u32::from_be_bytes([b[off], b[off + 1], b[off + 2], b[off + 3]])
}

fn cstr(b: &[u8], start: usize) -> &[u8] {
    if start >= b.len() {
        return &[];
    }
    let s = &b[start..];
    let n = s.iter().position(|&c| c == 0).unwrap_or(s.len());
    &s[..n]
}

fn has_compat(list: &[u8], want: &[u8]) -> bool {
    list.split(|&c| c == 0).any(|s| s == want)
}

fn read_cells(data: &[u8], first: usize, count: usize) -> u64 {
    let mut v = 0u64;
    for i in 0..count {
        v = (v << 32) | be32(data, (first + i) * 4) as u64;
    }
    v
}

fn read_u64_prop(data: &[u8]) -> u64 {
    if data.len() >= 8 {
        ((be32(data, 0) as u64) << 32) | be32(data, 4) as u64
    } else if data.len() >= 4 {
        be32(data, 0) as u64
    } else {
        0
    }
}

fn finalize(info: &mut BootInfo, n: &Node, (ac, sc): (usize, usize)) {
    // PSCI: method + cpu_on. Узел /psci не имеет reg.
    if n.is_psci && info.psci.is_none() {
        let method = match n.method {
            Some(m) if m.starts_with(b"hvc") => 1,
            _ => 0,
        };
        info.psci = Some(Psci {
            method,
            cpu_on: n.cpu_on.unwrap_or(0xC400_0003),
        });
    }
    // CPU-узлы: reg = MPIDR, enable-method = "psci" (QEMU virt).
    if n.is_cpu {
        if let Some(reg) = n.reg {
            if info.cpus_n < MAX_CPUS && ac >= 1 && reg.len() >= ac * 4 {
                let mpidr = read_cells(reg, 0, ac);
                info.cpus[info.cpus_n] = (mpidr, n.enable_psci);
                info.cpus_n += 1;
            }
        }
    }
    // Таймер: 4 тройки <тип номер флаги>; вторая — нон-секьюрный физический таймер EL1.
    if n.is_timer && info.timer_intid.is_none() {
        if let Some(i) = n.interrupts {
            if i.len() >= 24 {
                let ty = be32(i, 12);
                let num = be32(i, 16);
                info.timer_intid = Some(if ty == 1 { num + 16 } else { num + 32 });
            }
        }
    }
    let Some(reg) = n.reg else { return };
    let tuple = (ac + sc) * 4;
    if ac == 0 || ac > 2 || sc > 2 || reg.len() < tuple {
        return;
    }
    let count = reg.len() / tuple;
    let base = read_cells(reg, 0, ac);
    for i in 0..count {
        let b = read_cells(reg, i * (ac + sc), ac);
        let sz = read_cells(reg, i * (ac + sc) + ac, sc);
        if sz == 0 {
            continue;
        }
        if n.is_mem && info.ram_n < MAX_RAM {
            info.ram[info.ram_n] = (b, sz);
            info.ram_n += 1;
        }
        if n.is_reserved && info.reserved_n < MAX_NOMAP {
            info.reserved[info.reserved_n] = (b, sz);
            info.reserved_n += 1;
        }
        if n.no_map && info.no_map_n < MAX_NOMAP {
            info.no_map[info.no_map_n] = (b, sz);
            info.no_map_n += 1;
        }
    }
    if n.is_gic3 && info.gic.is_none() && count >= 2 {
        let w = ac + sc;
        info.gic = Some(GicInfo {
            dist: read_cells(reg, 0, ac) as usize,
            dist_size: read_cells(reg, ac, sc) as usize,
            redist: read_cells(reg, w, ac) as usize,
            redist_size: read_cells(reg, w + ac, sc) as usize,
        });
    }
    if n.is_pl011 && info.pl011.is_none() {
        info.pl011 = Some(base as usize);
    }
    if n.is_geni_uart && info.geni_uart.is_none() && !n.disabled {
        info.geni_uart = Some(base as usize);
    }
    if n.is_pinctrl && info.pinctrl.is_none() && !n.disabled {
        info.pinctrl = Some(base as usize);
    }
    if n.is_watchdog && info.wdt.is_none() && !n.disabled {
        info.wdt = Some(base as usize);
    }
    if n.is_virtio_mmio && info.virtio_mmio_n < MAX_VIRTIO_MMIO {
        let irq = n
            .interrupts
            .and_then(|i| {
                if i.len() >= 12 {
                    let ty = be32(i, 0);
                    let num = be32(i, 4);
                    Some(if ty == 1 { num + 16 } else { num + 32 })
                } else {
                    None
                }
            })
            .unwrap_or(0);
        let size = read_cells(reg, ac, sc) as usize;
        info.virtio_mmio[info.virtio_mmio_n] = (base as usize, size, irq);
        info.virtio_mmio_n += 1;
    }
}

/// Возвращает None, если `dtb` не указывает на корректный FDT.
pub fn parse(dtb: usize) -> Option<BootInfo> {
    if dtb == 0 || dtb & 3 != 0 {
        return None;
    }
    let head = unsafe { core::slice::from_raw_parts(dtb as *const u8, 40) };
    if be32(head, 0) != FDT_MAGIC {
        return None;
    }
    let total = be32(head, 4) as usize;
    if !(40..=MAX_DTB_SIZE).contains(&total) {
        return None;
    }
    let blob = unsafe { core::slice::from_raw_parts(dtb as *const u8, total) };
    let off_struct = be32(blob, 8) as usize;
    let off_strings = be32(blob, 12) as usize;
    let size_strings = be32(blob, 32) as usize;
    let size_struct = be32(blob, 36) as usize;
    if off_struct.checked_add(size_struct)? > total || off_strings.checked_add(size_strings)? > total {
        return None;
    }
    let strings = &blob[off_strings..off_strings + size_strings];
    let end = off_struct + size_struct;

    let mut info = BootInfo {
        dtb,
        dtb_size: total,
        ram: [(0, 0); MAX_RAM],
        ram_n: 0,
        no_map: [(0, 0); MAX_NOMAP],
        no_map_n: 0,
        reserved: [(0, 0); MAX_NOMAP],
        reserved_n: 0,
        pl011: None,
        gic: None,
        timer_intid: None,
        psci: None,
        cpus: [(0, false); MAX_CPUS],
        cpus_n: 0,
        virtio_mmio: [(0, 0, 0); MAX_VIRTIO_MMIO],
        virtio_mmio_n: 0,
        initrd_start: None,
        initrd_end: None,
        geni_uart: None,
        wdt: None,
        pinctrl: None,
    };
    let mut cells = [(2usize, 1usize); MAX_DEPTH + 1];
    let mut depth = 0usize;
    // Глубина узла /reserved-memory (если встретился)
    let mut reserved_at: Option<usize> = None;
    let mut node = Node::new();
    let mut pending = false;
    let mut p = off_struct;

    while p + 4 <= end {
        let tok = be32(blob, p);
        p += 4;
        match tok {
            TOK_BEGIN_NODE => {
                if pending {
                    finalize(&mut info, &node, cells[depth - 1]);
                }
                let start = p;
                while p < end && blob[p] != 0 {
                    p += 1;
                }
                if p >= end {
                    break;
                }
                let name = &blob[start..p];
                p = (p + 1 + 3) & !3;
                depth += 1;
                if depth > MAX_DEPTH {
                    break;
                }
                cells[depth] = (2, 1);
                node = Node::new();
                node.is_mem = name.starts_with(b"memory");
                node.is_psci = name == b"psci";
                node.is_cpu = name.starts_with(b"cpu@");
                node.is_chosen = name == b"chosen";
                // Дочерние узлы /reserved-memory резервируем целиком
                node.is_reserved = reserved_at == Some(depth.wrapping_sub(1));
                if name == b"reserved-memory" {
                    reserved_at = Some(depth);
                }
                pending = true;
            }
            TOK_END_NODE => {
                if pending {
                    finalize(&mut info, &node, cells[depth - 1]);
                    pending = false;
                }
                if Some(depth) == reserved_at {
                    reserved_at = None;
                }
                if depth == 0 {
                    break;
                }
                depth -= 1;
            }
            TOK_PROP => {
                if p + 8 > end {
                    break;
                }
                let len = be32(blob, p) as usize;
                let nameoff = be32(blob, p + 4) as usize;
                p += 8;
                if len > end - p {
                    break;
                }
                let data = &blob[p..p + len];
                p = (p + len + 3) & !3;
                if !pending {
                    continue;
                }
                match cstr(strings, nameoff) {
                    b"#address-cells" if len >= 4 => cells[depth].0 = be32(data, 0) as usize,
                    b"#size-cells" if len >= 4 => cells[depth].1 = be32(data, 0) as usize,
                    b"reg" => node.reg = Some(data),
                    b"no-map" => node.no_map = true,
                    b"interrupts" => node.interrupts = Some(data),
                    b"method" => node.method = Some(data),
                    b"cpu_on" => node.cpu_on = Some(read_u64_prop(data)),
                    b"enable-method" => node.enable_psci = data.starts_with(b"psci"),
                    b"status" => node.disabled = data.starts_with(b"disabled"),
                    b"device_type" => {
                        if data.starts_with(b"memory") {
                            node.is_mem = true;
                        }
                    }
                    b"compatible" => {
                        node.is_pl011 |= has_compat(data, b"arm,pl011");
                        node.is_gic3 |= has_compat(data, b"arm,gic-v3");
                        node.is_timer |= has_compat(data, b"arm,armv8-timer");
                        node.is_psci |= has_compat(data, b"arm,psci-1.0")
                            || has_compat(data, b"arm,psci-0.2")
                            || has_compat(data, b"arm,psci");
                        node.is_virtio_mmio |= has_compat(data, b"virtio,mmio");
                        node.is_geni_uart |= has_compat(data, b"qcom,msm-geni-console");
                        node.is_watchdog |= has_compat(data, b"qcom,msm-watchdog");
                        node.is_pinctrl |= has_compat(data, b"qcom,tlmm");
                    }
                    b"linux,initrd-start" if node.is_chosen && len >= 8 => {
                        info.initrd_start = Some(read_u64_prop(data) as usize);
                        kprintln!("fdt: initrd-start = {:#x}", info.initrd_start.unwrap());
                    }
                    b"linux,initrd-end" if node.is_chosen && len >= 8 => {
                        info.initrd_end = Some(read_u64_prop(data) as usize);
                        kprintln!("fdt: initrd-end = {:#x}", info.initrd_end.unwrap());
                    }
                    _ => {}
                }
            }
            TOK_NOP => {}
            TOK_END => break,
            _ => break,
        }
    }
    Some(info)
}

/// Полный парсинг DT для platform device model.
/// Регистрирует все узлы с compatible как PlatformDevice.
pub fn platform_populate(dtb: usize) {
    if dtb == 0 || dtb & 3 != 0 {
        return;
    }
    let head = unsafe { core::slice::from_raw_parts(dtb as *const u8, 40) };
    if be32(head, 0) != FDT_MAGIC {
        return;
    }
    let total = be32(head, 4) as usize;
    if !(40..=MAX_DTB_SIZE).contains(&total) {
        return;
    }
    let blob = unsafe { core::slice::from_raw_parts(dtb as *const u8, total) };
    let off_struct = be32(blob, 8) as usize;
    let off_strings = be32(blob, 12) as usize;
    let size_strings = be32(blob, 32) as usize;
    let size_struct = be32(blob, 36) as usize;
    if off_struct.checked_add(size_struct).is_none() || off_strings.checked_add(size_strings).is_none() {
        return;
    }
    if off_struct + size_struct > total || off_strings + size_strings > total {
        return;
    }
    let strings = &blob[off_strings..off_strings + size_strings];
    let end = off_struct + size_struct;

    let mut cells = [(2usize, 1usize); MAX_DEPTH + 1];
    let mut depth = 0usize;
    let mut node = PlatformNode::new();
    let mut pending = false;
    let mut p = off_struct;

    while p + 4 <= end {
        let tok = be32(blob, p);
        p += 4;
        match tok {
            TOK_BEGIN_NODE => {
                if pending {
                    finalize_platform_node(&node, &strings, &cells[depth - 1]);
                }
                let start = p;
                while p < end && blob[p] != 0 {
                    p += 1;
                }
                if p >= end {
                    break;
                }
                let name = &blob[start..p];
                p = (p + 1 + 3) & !3;
                depth += 1;
                if depth > MAX_DEPTH {
                    break;
                }
                cells[depth] = (2, 1);
                node = PlatformNode::new();
                node.name = name;
                pending = true;
            }
            TOK_END_NODE => {
                if pending {
                    finalize_platform_node(&node, &strings, &cells[depth - 1]);
                    pending = false;
                }
                if depth == 0 {
                    break;
                }
                depth -= 1;
            }
            TOK_PROP => {
                if p + 8 > end {
                    break;
                }
                let len = be32(blob, p) as usize;
                let nameoff = be32(blob, p + 4) as usize;
                p += 8;
                if len > end - p {
                    break;
                }
                let data = &blob[p..p + len];
                p = (p + len + 3) & !3;
                if !pending {
                    continue;
                }
                let prop_name = cstr(strings, nameoff);
                match prop_name {
                    b"#address-cells" if len >= 4 => cells[depth].0 = be32(data, 0) as usize,
                    b"#size-cells" if len >= 4 => cells[depth].1 = be32(data, 0) as usize,
                    b"reg" => node.reg = Some(data),
                    b"interrupts" => node.interrupts = Some(data),
                    b"compatible" => {
                        // Копируем compatible строки
                        let mut idx = 0;
                        let mut off = 0;
                        while off < data.len() && idx < MAX_COMPAT {
                            let s = cstr(data, off);
                            if !s.is_empty() {
                                node.compat[idx] = s;
                                idx += 1;
                            }
                            off += s.len() + 1;
                        }
                        node.compat_n = idx;
                    }
                    b"clocks" => node.clocks = Some(data),
                    b"clock-names" => node.clock_names = Some(data),
                    b"power-domains" => node.power_domains = Some(data),
                    b"iommus" => node.iommus = Some(data),
                    b"status" => node.status = Some(data),
                    _ => {}
                }
            }
            TOK_NOP => {}
            TOK_END => break,
            _ => break,
        }
    }
}

// Внутренние структуры для парсинга platform nodes
const MAX_COMPAT: usize = 8;
const MAX_RES: usize = 8;

struct PlatformNode<'a> {
    name: &'a [u8],
    reg: Option<&'a [u8]>,
    interrupts: Option<&'a [u8]>,
    compat: [&'a [u8]; MAX_COMPAT],
    compat_n: usize,
    clocks: Option<&'a [u8]>,
    clock_names: Option<&'a [u8]>,
    power_domains: Option<&'a [u8]>,
    status: Option<&'a [u8]>,
    iommus: Option<&'a [u8]>,
}

impl<'a> PlatformNode<'a> {
    const fn new() -> Self {
        Self {
            name: &[],
            reg: None,
            interrupts: None,
            compat: [&[]; MAX_COMPAT],
            compat_n: 0,
            clocks: None,
            clock_names: None,
            power_domains: None,
            status: None,
            iommus: None,
        }
    }
}

fn finalize_platform_node<'a>(node: &PlatformNode<'a>, strings: &'a [u8], ac_sc: &(usize, usize)) {
    // Проверяем status != "disabled"
    if let Some(st) = node.status {
        if st.starts_with(b"disabled") || st.starts_with(b"fail") {
            return;
        }
    }

    // Нужен хотя бы один compatible
    if node.compat_n == 0 {
        return;
    }

    // Извлекаем compatible строки
    // DTB живет вечно (в физической памяти), поэтому можно безопасно расширить lifetime до 'static
    let mut compat_strs: [&'static str; MAX_COMPAT] = [""; MAX_COMPAT];
    for i in 0..node.compat_n {
        let s = unsafe { core::str::from_utf8_unchecked(node.compat[i]) };
        compat_strs[i] = unsafe { core::mem::transmute::<&str, &'static str>(s) };
    }

    // Ресурсы
    let mut resources: [platform::Resource; MAX_RES] = [platform::Resource::Mem { base: 0, size: 0 }; MAX_RES];
    let mut res_n = 0;

    // Memory regions
    if let Some(reg) = node.reg {
        let (ac, sc) = *ac_sc;
        let tuple = (ac + sc) * 4;
        if ac > 0 && ac <= 2 && sc <= 2 && reg.len() >= tuple {
            let count = reg.len() / tuple;
            for i in 0..count {
                let base = read_cells(reg, i * (ac + sc), ac) as usize;
                let size = read_cells(reg, i * (ac + sc) + ac, sc) as usize;
                if size > 0 && res_n < MAX_RES {
                    resources[res_n] = platform::Resource::Mem { base, size };
                    res_n += 1;
                }
            }
        }
    }

    // Interrupts
    if let Some(ints) = node.interrupts {
        if ints.len() >= 12 {
            // Формат: <type number flags> (3 cells = 12 bytes)
            let count = ints.len() / 12;
            for i in 0..count {
                let ty = be32(ints, i * 12);
                let num = be32(ints, i * 12 + 4);
                let flags = be32(ints, i * 12 + 8);
                let irq_type = match ty {
                    0 => platform::IrqType::Spi,
                    1 => platform::IrqType::Ppi,
                    _ => platform::IrqType::Spi,
                };
                if res_n < MAX_RES {
                    resources[res_n] = platform::Resource::Irq(platform::IrqSpec {
                        irq_type,
                        number: num,
                        trigger: flags,
                    });
                    res_n += 1;
                }
            }
        }
    }

    // Clocks
    if let Some(clks) = node.clocks {
        // clock-names optional
        let names_data = node.clock_names.unwrap_or(&[]);
        let mut name_idx = 0;
        let mut name_off = 0;
        let mut clock_names: [&'a [u8]; 8] = [&[]; 8];
        let mut name_n = 0;
        while name_off < names_data.len() && name_n < 8 {
            let s = cstr(names_data, name_off);
            if !s.is_empty() {
                clock_names[name_n] = s;
                name_n += 1;
            }
            name_off += s.len() + 1;
        }

        // clocks: массив phandle (u32)
        if clks.len() % 4 == 0 {
            let count = clks.len() / 4;
            for i in 0..count {
                let phandle = be32(clks, i * 4);
                let name = if i < name_n {
                    let s = unsafe { core::str::from_utf8_unchecked(clock_names[i]) };
                    unsafe { core::mem::transmute::<&str, &'static str>(s) }
                } else {
                    ""
                };
                if res_n < MAX_RES {
                    resources[res_n] = platform::Resource::Clock { name, id: phandle as usize };
                    res_n += 1;
                }
            }
        }
    }

    // Power domains
    if let Some(pds) = node.power_domains {
        if pds.len() % 4 == 0 {
            let count = pds.len() / 4;
            for i in 0..count {
                let phandle = be32(pds, i * 4);
                if res_n < MAX_RES {
                    resources[res_n] = platform::Resource::PowerDomain { name: "", id: phandle as usize };
                    res_n += 1;
                }
            }
        }
    }

    // IOMMU/SMMU (iommus property: <phandle sid mask> per entry, 3 cells = 12 bytes)
    if let Some(ims) = node.iommus {
        // iommus: массив phandle + sid + mask (3 u32 per entry)
        if ims.len() % 12 == 0 {
            let count = ims.len() / 12;
            for i in 0..count {
                let phandle = be32(ims, i * 12) as usize;
                let sid = be32(ims, i * 12 + 4);
                let mask = be32(ims, i * 12 + 8);
                if res_n < MAX_RES {
                    resources[res_n] = platform::Resource::Iommu { phandle, sid, mask };
                    res_n += 1;
                }
            }
        }
    }

    // Имя устройства из имени узла
    let name_str = {
        let s = unsafe { core::str::from_utf8_unchecked(node.name) };
        unsafe { core::mem::transmute::<&str, &'static str>(s) }
    };

    // Создаем PlatformDevice
    let mut res_arr = [platform::Resource::Mem { base: 0, size: 0 }; 8];
    for i in 0..res_n {
        res_arr[i] = resources[i];
    }
    let mut compat_arr = [""; 8];
    for i in 0..node.compat_n {
        let s = unsafe { core::str::from_utf8_unchecked(node.compat[i]) };
        compat_arr[i] = unsafe { core::mem::transmute::<&str, &'static str>(s) };
    }

    // Фильтр: регистрируем только если драйвер уже зарегистрирован и
    // поддерживает узел. Иначе реестр переполняется на реальном DTB.
    if !platform::driver_interested(&compat_arr, node.compat_n) {
        // Логируем пропущенные узлы для диагностики
        if node.compat_n > 0 {
            let first_compat = unsafe { core::str::from_utf8_unchecked(node.compat[0]) };
            kprintln!("fdt: skip node '{}' (compat: {}) - no driver", name_str, first_compat);
        }
        return;
    }

    let dev = platform::PlatformDevice {
        name: name_str,
        compatible: compat_arr,
        compatible_n: node.compat_n,
        resources: res_arr,
        resources_n: res_n,
        data: 0,
        status: platform::ProbeStatus::NotProbed,
        driver_data: 0,
    };

    // Регистрируем
    let _ = platform::device_register(dev);
}
