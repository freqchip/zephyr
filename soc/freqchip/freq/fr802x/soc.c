/*
 * Copyright (c) 2026 Freqchip
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/__assert.h>
#include <cmsis_core.h>
#include <errno.h>
#include <malloc.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <soc.h>
#include <system_fr802x.h>
#include <heap.h>

/* 链接器导出的 ram_code 拷贝符号 */
extern char __ram_code_start[];
extern char __ram_code_end[];
extern char __ram_code_load_start[];
extern char __heap_start[];
extern char __heap_end[];

#define FR802X_HEAP_MAGIC            0x46524850U
#define FR802X_MALLOC_ALIGNMENT      __alignof__(z_max_align_t)
#define FR802X_MALLOC_PAGE_ALIGNMENT 0x1000U

struct fr802x_heap_header {
	void *raw;
	size_t size;
	uint32_t magic;
};

static bool fr802x_heap_ready;

void btdm_controller_prepare(void);
void trim_cp_config(void);
void trim_ft_config(void);

static bool valid_alignment(size_t alignment)
{
	return alignment >= sizeof(void *) &&
	       (alignment % sizeof(void *)) == 0U &&
	       (alignment & (alignment - 1U)) == 0U;
}

static void *fr802x_heap_alloc(size_t size, size_t alignment)
{
	struct fr802x_heap_header *header;
	uintptr_t aligned;
	uintptr_t start;
	size_t alloc_size;
	void *raw;

	if (!fr802x_heap_ready || !valid_alignment(alignment)) {
		return NULL;
	}

	/* malloc(0) may return a unique pointer which can later be freed. */
	alloc_size = size == 0U ? 1U : size;
	if (alloc_size > UINT32_MAX - sizeof(*header) - (alignment - 1U)) {
		return NULL;
	}

	raw = heap_mem_alloc(HEAP_TYPE_SRAM_BLOCK,
			     (uint32_t)(alloc_size + sizeof(*header) + alignment - 1U));
	if (raw == NULL) {
		return NULL;
	}

	start = (uintptr_t)raw + sizeof(*header);
	aligned = (start + alignment - 1U) & ~(uintptr_t)(alignment - 1U);
	header = (struct fr802x_heap_header *)aligned - 1;
	header->raw = raw;
	header->size = size;
	header->magic = FR802X_HEAP_MAGIC;

	return (void *)aligned;
}

static struct fr802x_heap_header *fr802x_heap_header_get(void *ptr)
{
	struct fr802x_heap_header *header = (struct fr802x_heap_header *)ptr - 1;

	__ASSERT_NO_MSG(header->magic == FR802X_HEAP_MAGIC);
	return header->magic == FR802X_HEAP_MAGIC ? header : NULL;
}

void *malloc(size_t size)
{
	void *ptr = fr802x_heap_alloc(size, FR802X_MALLOC_ALIGNMENT);

	if (ptr == NULL && size != 0U) {
		errno = ENOMEM;
	}

	return ptr;
}

void free(void *ptr)
{
	struct fr802x_heap_header *header;
	void *raw;

	if (ptr == NULL) {
		return;
	}

	header = fr802x_heap_header_get(ptr);
	if (header == NULL) {
		return;
	}

	raw = header->raw;
	header->magic = 0U;
	heap_mem_free(raw);
}

void *calloc(size_t nmemb, size_t size)
{
	size_t total;
	void *ptr;

	if (nmemb != 0U && size > SIZE_MAX / nmemb) {
		errno = ENOMEM;
		return NULL;
	}

	total = nmemb * size;
	ptr = malloc(total);
	if (ptr != NULL) {
		memset(ptr, 0, total);
	}

	return ptr;
}

void *realloc(void *ptr, size_t size)
{
	struct fr802x_heap_header *header;
	void *new_ptr;
	size_t copy_size;

	if (ptr == NULL) {
		return malloc(size);
	}

	if (size == 0U) {
		free(ptr);
		return NULL;
	}

	header = fr802x_heap_header_get(ptr);
	if (header == NULL) {
		errno = EINVAL;
		return NULL;
	}

	new_ptr = malloc(size);
	if (new_ptr == NULL) {
		return NULL;
	}

	copy_size = header->size < size ? header->size : size;
	memcpy(new_ptr, ptr, copy_size);
	free(ptr);

	return new_ptr;
}

void *reallocarray(void *ptr, size_t nmemb, size_t size)
{
	if (nmemb != 0U && size > SIZE_MAX / nmemb) {
		errno = ENOMEM;
		return NULL;
	}

	return realloc(ptr, nmemb * size);
}

void *memalign(size_t alignment, size_t size)
{
	void *ptr = fr802x_heap_alloc(size, alignment);

	if (ptr == NULL) {
		errno = valid_alignment(alignment) ? ENOMEM : EINVAL;
	}

	return ptr;
}

void *aligned_alloc(size_t alignment, size_t size)
{
	if (!valid_alignment(alignment) || (size % alignment) != 0U) {
		errno = EINVAL;
		return NULL;
	}

	return memalign(alignment, size);
}

int posix_memalign(void **memptr, size_t alignment, size_t size)
{
	void *ptr;

	if (memptr == NULL || !valid_alignment(alignment)) {
		return EINVAL;
	}

	ptr = fr802x_heap_alloc(size, alignment);
	if (ptr == NULL) {
		return ENOMEM;
	}

	*memptr = ptr;
	return 0;
}

void *valloc(size_t size)
{
	return memalign(FR802X_MALLOC_PAGE_ALIGNMENT, size);
}

void *pvalloc(size_t size)
{
	size_t rounded;

	if (size > SIZE_MAX - (FR802X_MALLOC_PAGE_ALIGNMENT - 1U)) {
		errno = ENOMEM;
		return NULL;
	}

	rounded = (size + FR802X_MALLOC_PAGE_ALIGNMENT - 1U) &
		  ~(size_t)(FR802X_MALLOC_PAGE_ALIGNMENT - 1U);
	return valloc(rounded);
}

void cfree(void *ptr)
{
	free(ptr);
}

size_t malloc_usable_size(void *ptr)
{
	struct fr802x_heap_header *header;

	if (ptr == NULL) {
		return 0U;
	}

	header = fr802x_heap_header_get(ptr);
	return header == NULL ? 0U : header->size;
}

static void fr802x_heap_init(void)
{
	size_t heap_size = (size_t)(__heap_end - __heap_start);

	__ASSERT_NO_MSG(heap_size <= UINT32_MAX);
	heap_mem_init(HEAP_TYPE_SRAM_BLOCK, (uint8_t *)__heap_start,
		      (uint32_t)heap_size);
	fr802x_heap_ready = true;
}

static void soc_ram_code_copy(void)
{
	if (&__ram_code_end[0] > &__ram_code_start[0]) {
		size_t size = __ram_code_end - __ram_code_start;

		memcpy(__ram_code_start, __ram_code_load_start, size);

		/*
		 * memcpy 只刷了 D-Cache，I-Cache 里还是旧的。必须清 I-Cache，
		 * 否则 CPU 取指时 I-Cache 命中垃圾数据 → HardFault。
		 */
		__DSB();
		SCB->ICIALLU = 0U;          /* 写 0 即 Invalidate All I-Cache */
		__DSB();
		__ISB();
	}
}

static int FR8029D_soc_init(void)
{
	/* ram_code 段从 flash 拷贝到 RAM，之后才能调用 __RAM_CODE 函数 */
	soc_ram_code_copy();
	fr802x_heap_init();

	SystemInit();

	btdm_controller_prepare();

	trim_cp_config();
#ifndef FT_PROGRAMMER
    trim_ft_config();
#else
    trim_ft_param.version = 0xff;
#endif

	pmu_init();

	return 0;
}

SYS_INIT(FR8029D_soc_init, PRE_KERNEL_1, 0);
