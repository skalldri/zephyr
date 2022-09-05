/*
 * Copyright 2022 IoT.bzh
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_ARCH_ARM_AARCH32_ARCH_INLINES_H
#define ZEPHYR_INCLUDE_ARCH_ARM_AARCH32_ARCH_INLINES_H

#include <zephyr/kernel_structs.h>

#ifndef CONFIG_SMP
static ALWAYS_INLINE _cpu_t *arch_curr_cpu(void)
{
	/* Dummy implementation always return the first cpu */
	return &_kernel.cpus[0];
}

static ALWAYS_INLINE uint32_t arch_proc_id(void)
{
	/*
	 * Placeholder implementation to be replaced with an architecture
	 * specific call to get processor ID
	 */
	return arch_curr_cpu()->id;
}
#else
// ARM32 doesn't have a generic, implementation-independent method for SMP-releated arch functions.
// Expect that these are declared in the soc.h file for this processor
#include <soc.h>
#endif

#endif /* ZEPHYR_INCLUDE_ARCH_ARM_AARCH32_ARCH_INLINES_H */
