# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(${ZEPHYR_BASE}/drivers)
zephyr_sources(soc.c)
zephyr_sources(power.c)

# BLE exchange RAM is mapped in the Cortex-M Device memory region.  HCI
# packets start with a one-byte H4 indicator, so their headers are naturally
# unaligned.  Prevent GCC from folding fixed-size memcpy calls into unaligned
# halfword/word accesses, which are not permitted for Device memory.
zephyr_compile_options(-mno-unaligned-access)

zephyr_include_directories(.)

set(SOC_LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/linker.ld CACHE INTERNAL "")

# 链接器搜索路径: 让 linker.ld 的 INCLUDE 能找到同目录文件
zephyr_ld_options(-L${CMAKE_CURRENT_SOURCE_DIR})
