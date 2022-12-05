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
struct k_sem lockout_sem[CONFIG_MP_NUM_CPUS];

extern void *_vector_table[];

extern void z_arm_secondary_core_reset(void);

#define RP2040_FIFO_IPI 0xDEADBEEF
#define RP2040_FIFO_LOCKOUT_REQ 0xBADC0FEE
#define RP2040_FIFO_LOCKOUT_ACK 0xBADC00DE
#define RP2040_FIFO_UNLOCK 0xFEE12BAD

// Re-implement functions from the RP2040 SDK to _ensure_ they are always __ramfunc
__ramfunc bool mc_fifo_rvalid() {
    return !!(sio_hw->fifo_st & SIO_FIFO_ST_VLD_BITS);
}

__ramfunc uint32_t mc_fifo_pop_blocking() {
    // If nothing there yet, we wait for an event first,
    // to try and avoid too much busy waiting
    while (!mc_fifo_rvalid()) {
        __wfe();
    }

    return sio_hw->fifo_rd;
}

__ramfunc bool mc_fifo_wready(void) {
    return !!(sio_hw->fifo_st & SIO_FIFO_ST_RDY_BITS);
}

__ramfunc void mc_fifo_push_blocking(int32_t data) {
    // We wait for the fifo to have some space
    while (!mc_fifo_wready()) {}

    sio_hw->fifo_wr = data;

    // Fire off an event to the other core
    __sev();
}

__ramfunc void mc_fifo_clear_irq(void) {
    // Write any value to clear the error flags
    sio_hw->fifo_st = 0xff;
}

__ramfunc int ipi_isr(uint8_t isr) {
    bool lockout = false;

    while (lockout || mc_fifo_rvalid()) {
        uint32_t word = mc_fifo_pop_blocking();

        switch (word) {
            case RP2040_FIFO_IPI:
                z_sched_ipi();
                break;

            case RP2040_FIFO_LOCKOUT_REQ:
                // The other core has requested that we lock out.
                __ASSERT(lockout == false, "Core %d: Request to lockout while already locked!", arch_proc_id());
                //printk("Core %d entering RAMFUNC spinlock\n", arch_proc_id());
                lockout = true;
                mc_fifo_push_blocking(RP2040_FIFO_LOCKOUT_ACK);
                break;

            case RP2040_FIFO_LOCKOUT_ACK:
                //printk("Core %d received lockout ACK\n", arch_proc_id());
                // Give our semaphore to unblock rp2040_mp_lockout()
                k_sem_give(&lockout_sem[arch_proc_id()]);
                break;

            case RP2040_FIFO_UNLOCK:
                __ASSERT(lockout == true, "Core %d: Request to unlock while not locked!", arch_proc_id());
                //printk("Core %d received unlock, exiting spinlock\n", arch_proc_id());
                lockout = false;
                break;

            default:
                __ASSERT(false, "Unknown value received in multicore FIFO on core %d: %u", arch_proc_id(), word);
                break;
        }
    }

    // Clear IRQ
    mc_fifo_clear_irq();

    return 1;
}

ISR_DIRECT_DECLARE(ipi_0_isr) {
    return ipi_isr(0);
}

ISR_DIRECT_DECLARE(ipi_1_isr) {
    return ipi_isr(1);
}

void arch_sched_ipi() {
    multicore_fifo_push_blocking(RP2040_FIFO_IPI);
}

void rp2040_mp_lockout() {
    // Request the other core lock-out
    mc_fifo_push_blocking(RP2040_FIFO_LOCKOUT_REQ);

    // Wait for the core to reply via ISR
    k_sem_take(&lockout_sem[arch_proc_id()], K_FOREVER);
}

void rp2040_mp_unlock() {
    // Tell the other core it's unlocked
    // DO NOT reset the semaphore here: only the ISR is allowed to reset the semaphore
    mc_fifo_push_blocking(RP2040_FIFO_UNLOCK);
}

void z_arm_secondary_core_entry()
{
    //k_busy_wait(10 * USEC_PER_SEC);
    //__breakpoint();
    //printk("Starting run!\n");

    printk("*** Booting Zephyr OS - Secondary CPU %d ***\n", arch_proc_id());

    // Drain the FIFO and clear any pending IRQs from the FIFO before we enable the FIFO
    // ISR
    multicore_fifo_drain();
    multicore_fifo_clear_irq();

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

    // Drain our FIFO to ensure there are no stale messages from the bootloader process, and
    // clear any stale IRQs in the FIFO
    multicore_fifo_drain();
    multicore_fifo_clear_irq();

    // Now that the second core has launched, and our FIFO is drained, we can enable the IRQ for the FIFO
    irq_enable(SIO_IRQ_PROC0);
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

    // Don't enable the IRQ yet, since it's used by the PicoSDK during the bootloading process
    IRQ_DIRECT_CONNECT(SIO_IRQ_PROC0, 0, ipi_0_isr, 0);

	irq_unlock(key);

    for (int i = 0; i < CONFIG_MP_NUM_CPUS; i++) {
        // Initialize all semaphores to 0, with max value of 1
        // The semaphore contract is, while we are holding the semaphore represented by our Core ID,
        // the other core is locked out and will not execute
        k_sem_init(&lockout_sem[i], 0, 1);
    }

	return 0;
}

SYS_INIT(rp2040_smp_init, PRE_KERNEL_1, 1);