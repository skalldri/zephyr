/*
 * Copyright (c) 2021 Stuart Alldritt
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief System/hardware module for Raspberry Pi RP2040 family processor
 *
 * This module provides routines to initialize and support Symmetric Multi-Processing (SMP)
 * on Raspberry Pi PR2040 family processors.
 */

#include <zephyr/sys/arch_interface.h>
#include <zephyr/kernel/thread_stack.h>

#include <pico/multicore.h>
#include <hardware/sync.h>

typedef struct {
    arch_cpustart_t fn;
    void* arg;
} rp2040_launch_config_t;

static bool cpus_active[CONFIG_MP_NUM_CPUS];
static rp2040_launch_config_t cpu_launch_config[CONFIG_MP_NUM_CPUS];

extern void *_vector_table[];

void _secondary_processor_trampoline_start()
{
    // RP2040 only has a single secondary processor. If this function is executing, we must be
    // the secondary processor.
    cpus_active[1] = true;
    cpu_launch_config[1].fn(cpu_launch_config[1].arg);
}

/**
 * @brief 
 * 
 * @param cpu_num 
 * @param stack 
 * @param sz 
 * @param fn 
 * @param arg 
 */
void arch_start_cpu(int cpu_num, k_thread_stack_t *stack, int sz, arch_cpustart_t fn, void *arg)
{
    __ASSERT(cpu_num == 1, "RP2040 only contains two CPUs");

    cpu_launch_config[cpu_num].fn = fn;
    cpu_launch_config[cpu_num].arg = arg;

    char* stack_real = Z_KERNEL_STACK_BUFFER(stack) + sz;

    // TODO: call the RP2040 HAL function to boot the secondary CPU
    multicore_reset_core1();
    multicore_launch_core1_raw(_secondary_processor_trampoline_start, (uint32_t*)stack_real, (uint32_t)_vector_table);
}

/**
 * @brief 
 * 
 * @param cpu_num 
 * @return true 
 * @return false 
 */
bool arch_cpu_active(int cpu_num)
{
    return cpus_active[cpu_num];
}
