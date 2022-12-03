/*
 * Copyright (c) 2013-2015 Wind River Systems, Inc.
 * Copyright (c) 2020 Nordic Semiconductor ASA.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Populated vector table in ROM
 *
 * Vector table at the beginning of the image for starting system. The reset
 * vector is the system entry point, ie. the first instruction executed.
 *
 * The table is populated with all the system exception handlers. The NMI vector
 * must be populated with a valid handler since it can happen at any time. The
 * rest should not be triggered until the kernel is ready to handle them.
 */

#include <zephyr/toolchain.h>
#include <zephyr/linker/sections.h>
#include "vector_table.h"

_ASM_FILE_PROLOGUE
/*
 * Tell armclang that stack alignment are ensured.
 */
.eabi_attribute Tag_ABI_align_preserved, 1

GDATA(z_main_stack)

SECTION_SUBSEC_FUNC(exc_vector_table,_vector_table_section,_vector_table)

    /*
     * setting the _very_ early boot on the main stack allows to use memset
     * on the interrupt stack when CONFIG_INIT_STACKS is enabled before
     * switching to the interrupt stack for the rest of the early boot
     */
    .word z_main_stack + CONFIG_MAIN_STACK_SIZE

    .word z_arm_reset // IRQ 1: Reset vector
    .word z_arm_nmi   // IRQ 2: Non-maskable Interrupt

    .word z_arm_hard_fault // IRQ 3: Hard Fault
#if defined(CONFIG_ARMV6_M_ARMV8_M_BASELINE)
    .word 0 // IRQ 4: Reserved
    .word 0 // IRQ 5: Reserved
    .word 0 // IRQ 6: Reserved
    .word 0 // IRQ 7: Reserved
    .word 0 // IRQ 8: Reserved
    .word 0 // IRQ 9: Reserved
    .word 0 // IRQ 10: Reserved
    .word z_arm_svc // IRQ 11: SVCall
    .word 0 // IRQ 12: Reserved
#elif defined(CONFIG_ARMV7_M_ARMV8_M_MAINLINE)
    .word z_arm_mpu_fault // IRQ 4:
    .word z_arm_bus_fault // IRQ 5:
    .word z_arm_usage_fault // IRQ 6:
#if defined(CONFIG_ARMV8_M_SE)
#if defined(CONFIG_ARM_SECURE_FIRMWARE)
    .word z_arm_secure_fault // IRQ 7
#else
    .word z_arm_exc_spurious // IRQ 7
#endif /* CONFIG_ARM_SECURE_FIRMWARE */
#else
    .word 0 // IRQ 7
#endif /* CONFIG_ARMV8_M_SE */
    .word 0 // IRQ 8
    .word 0 // IRQ 9
    .word 0 // IRQ 10
    .word z_arm_svc // IRQ 11: SVCall
    .word z_arm_debug_monitor // IRQ 12: Debug Monitor
#else
#error Unknown ARM architecture
#endif /* CONFIG_ARMV6_M_ARMV8_M_BASELINE */
    .word 0 // IRQ 13: Reserved
#if defined(CONFIG_MULTITHREADING)
    .word z_arm_pendsv // IRQ 14: PendSV
#else
    .word z_arm_exc_spurious // IRQ 14: PendSV
#endif
#if defined(CONFIG_CPU_CORTEX_M_HAS_SYSTICK)
#if defined(CONFIG_SYS_CLOCK_EXISTS) && \
    defined(CONFIG_CORTEX_M_SYSTICK_INSTALL_ISR)
    .word sys_clock_isr // IRQ 15: SysTick timer, optional
#else
    .word z_arm_exc_spurious // IRQ 15: SysTick timer, optional
#endif /* CONFIG_SYS_CLOCK_EXISTS && CONFIG_CORTEX_M_SYSTICK_INSTALL_ISR */
#else
    .word 0 // IRQ 15: SysTick timer, optional
#endif /* CONFIG_CPU_CORTEX_M_HAS_SYSTICK */

