/*
 * Copyright (c) 2016 Linaro Limited
 * Copyright (c) 2021 Yonatan Schachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file SoC configuration macros for the Raspberry Pi RP2040 family processors
 */

#ifndef _RPI_PICO_RP2040_SOC_H_
#define _RPI_PICO_RP2040_SOC_H_

#include <zephyr/sys/__assert.h>
#include <zephyr/kernel_structs.h>
#include <hardware/sync.h>

#define __VTOR_PRESENT CONFIG_CPU_CORTEX_M_HAS_VTOR
#define __MPU_PRESENT CONFIG_CPU_HAS_ARM_MPU

#ifdef __cplusplus
extern "C" {
#endif

#if defined(CONFIG_SMP)
static ALWAYS_INLINE uint32_t arch_proc_id(void)
{
	uint core_num = get_core_num();
    __ASSERT(core_num < CONFIG_MP_NUM_CPUS, "Invalid core number!");

	return (uint32_t)core_num;
}

static ALWAYS_INLINE _cpu_t *arch_curr_cpu(void)
{
	return &_kernel.cpus[arch_proc_id()];
}
#endif

#ifdef __cplusplus
}
#endif

#endif /* _RPI_PICO_RP2040_SOC_H_ */
