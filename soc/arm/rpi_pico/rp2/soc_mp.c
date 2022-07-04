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

//#include <pico/multi

static bool cpus_active[CONFIG_MP_NUM_CPUS];

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

    // TODO: call the RP2040 HAL function to boot the secondary CPU


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
