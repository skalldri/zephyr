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
#include <zephyr/kernel.h>
#include <zephyr/init.h>

#include <pico/multicore.h>
#include <hardware/sync.h>
#include <hardware/regs/intctrl.h>

typedef struct {
    arch_cpustart_t fn;
    void* arg;
    char* stack_real;
} rp2040_launch_config_t;

static bool cpus_active[CONFIG_MP_NUM_CPUS];
static rp2040_launch_config_t cpu_launch_config[CONFIG_MP_NUM_CPUS];

extern void *_vector_table[];

extern void z_arm_secondary_core_reset(void);

ISR_DIRECT_DECLARE(ipi_0_isr) {
    // Drain the fifo
    multicore_fifo_drain();

    // Clear ISR
    multicore_fifo_clear_irq();

    // Signal Zephyr that we received an ISR
    z_sched_ipi();

    return 1;
}

ISR_DIRECT_DECLARE(ipi_1_isr) {
    // Drain the fifo
    multicore_fifo_drain();

    // Clear ISR
    multicore_fifo_clear_irq();

    // Signal Zephyr that we received an ISR
    z_sched_ipi();

    return 1;
}

void arch_sched_ipi() {
    multicore_fifo_push_blocking(0xDEADBEEF);
}

void z_arm_secondary_core_entry()
{
    //k_busy_wait(10 * USEC_PER_SEC);
    //__breakpoint();
    //printk("Starting run!\n");

    printk("*** Booting Zephyr OS - Secondary CPU %d ***\n", arch_proc_id());

    IRQ_DIRECT_CONNECT(SIO_IRQ_PROC1, 0, ipi_1_isr, 0);
    irq_enable(SIO_IRQ_PROC1);

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

    printk("Starting CPU %d with stack pointer %p\n", cpu_num, stack_real);

    multicore_reset_core1();
    multicore_launch_core1_raw(z_arm_secondary_core_reset, (uint32_t*)stack_real, (uint32_t)_vector_table);
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

static int rp2040_smp_init(const struct device *arg)
{
	uint32_t key;

	key = irq_lock();
	
    IRQ_DIRECT_CONNECT(SIO_IRQ_PROC0, 0, ipi_0_isr, 0);
    irq_enable(SIO_IRQ_PROC0);

	irq_unlock(key);

	return 0;
}

SYS_INIT(rp2040_smp_init, PRE_KERNEL_1, 1);