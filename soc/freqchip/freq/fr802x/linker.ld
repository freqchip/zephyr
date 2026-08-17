/*
 * Copyright (c) 2026 Freqchip
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/devicetree.h>
#include <zephyr/linker/linker-tool.h>
#include <zephyr/linker/devicetree_regions.h>

#include <zephyr/arch/arm/cortex_m/scripts/linker.ld>

/* 保留给 boot 固件的 SRAM */
#define BOOT_SRAM_SKIP   DT_REG_SIZE(DT_NODELABEL(boot_ram))  /* boot 预留大小 (0x1818) */
#define SRAM_BASE        DT_REG_ADDR(DT_NODELABEL(sram0))     /* SRAM 起始地址 (0x20000000) */
#define SRAM_SIZE        DT_REG_SIZE(DT_NODELABEL(sram0))     /* SRAM 总大小 (96KB) */

MEMORY
{
    FLASH (rwx) : ORIGIN = DT_REG_ADDR(DT_NODELABEL(flash0)) + DT_REG_ADDR(DT_NODELABEL(slot0_partition)),  /* flash0 起点 + slot0 偏移 (0x10002000) */
                  LENGTH = DT_REG_SIZE(DT_NODELABEL(slot0_partition))                                       /* slot0 分区大小 (384KB) */
    RAM   (rw)  : ORIGIN = SRAM_BASE + BOOT_SRAM_SKIP,                                                      /* SRAM 起始 + boot 预留 (0x20001818) */
                  LENGTH = SRAM_SIZE - BOOT_SRAM_SKIP                                                       /* SRAM 总大小 - boot 预留 (89.8KB) */
}

/* BLE Controller ROM 符号表 */
INCLUDE linker_ble_rom_symbols.ld

/* ram_code 放在最后，之后重新定义 _end 确保 heap 不与其重叠 */
SECTIONS
{
    ram_code : ALIGN(8)
    {
        KEEP(*(ram_code))
        KEEP(*(ram_code.*))
    } > RAM AT > ROMABLE_REGION

    PROVIDE(__ram_code_end = .);
    PROVIDE(__ram_code_start = ADDR(ram_code));
    PROVIDE(__ram_code_load_start = LOADADDR(ram_code));

    /* 重新定义 _end，使 C 库 heap 从 ram_code 之后开始 */
    _end = .;
    PROVIDE(_end = .);
    __heap_start = .;
    PROVIDE(__heap_start = .);
    z_mapped_end = .;
    PROVIDE(z_mapped_end = .);
    _image_ram_end = .;
    PROVIDE(_image_ram_end = .);

    /* picolibc sbrk 需要的 heap 结束地址 */
    __heap_end = ORIGIN(RAM) + LENGTH(RAM);
    PROVIDE(__heap_end = __heap_end);
}

/*
        RAM 起始 (0x20001818)
               │
               ▼
        ┌─────────────┐
        │   .data     │  ← 已初始化数据
        ├─────────────┤
        │    .bss     │  ← 未初始化数据（清0）
        ├─────────────┤
        │   .noinit   │  ← 不初始化数据
        ├─────────────┤
        │  ram_code   │  ← 从 FLASH 拷贝到 RAM 执行
        ├─────────────┤
        │    HEAP     │  ← 从 ram_code 之后开始，到 SRAM 末尾
        └─────────────┘
        RAM 结束 (0x20010000)
*/