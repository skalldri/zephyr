/*
 * Copyright (c) 2013-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief ARM Cortex-M exception/interrupt exit API
 *
 * Provides functions for performing kernel handling when exiting exceptions or
 * interrupts that are installed directly in the vector table (i.e. that are not
 * wrapped around by _isr_wrapper()).
 */

#include <zephyr/toolchain.h>
#include <zephyr/linker/sections.h>
#include <offsets_short.h>
#include <zephyr/arch/cpu.h>

_ASM_FILE_PROLOGUE

GTEXT(z_arm_exc_exit)
GTEXT(z_arm_int_exit)
GDATA(_kernel)

/**
 *
 * @brief Kernel housekeeping when exiting interrupt handler installed
 * 		directly in vector table
 *
 * Kernel allows installing interrupt handlers (ISRs) directly into the vector
 * table to get the lowest interrupt latency possible. This allows the ISR to
 * be invoked directly without going through a software interrupt table.
 * However, upon exiting the ISR, some kernel work must still be performed,
 * namely possible context switching. While ISRs connected in the software
 * interrupt table do this automatically via a wrapper, ISRs connected directly
 * in the vector table must invoke z_arm_int_exit() as the *very last* action
 * before returning.
 *
 * e.g.
 *
 * void myISR(void)
 * {
 * 	printk("in %s\n", __FUNCTION__);
 * 	doStuff();
 * 	z_arm_int_exit();
 * }
 *
 */

SECTION_SUBSEC_FUNC(TEXT, _HandlerModeExit, z_arm_int_exit)

/* z_arm_int_exit falls through to z_arm_exc_exit (they are aliases of each
 * other)
 */

/**
 *
 * @brief Kernel housekeeping when exiting exception handler installed
 * 		directly in vector table
 *
 * See z_arm_int_exit().
 *
 */

SECTION_SUBSEC_FUNC(TEXT, _HandlerModeExit, z_arm_exc_exit)

#if defined(CONFIG_PREEMPT_ENABLED)
#if defined(CONFIG_USE_SWITCH)	

#ifdef CONFIG_SMP
    /* determine which CPU we're running on */
    push {lr}
    bl arm32_curr_cpu

    /*
    r0 is the return from arm32_curr_cpu(), contains the address
    of our struct _cpu for this core. 

    store _cpu for this core in r1
    */
    mov r1, r0

#if defined(CONFIG_ARMV6_M_ARMV8_M_BASELINE)
    pop {r3}
    mov lr, r3
#else
    pop {lr}
#endif /* CONFIG_ARMV6_M_ARMV8_M_BASELINE */
#else /* !CONFIG_SMP */
    /* load _kernel into r1 */
    ldr r1, =_kernel

#if (___kernel_t_cpus_OFFSET != 0)
    /* get offset to "cpus[0]" entry in _kernel */
    adds r1, r1, #___kernel_t_cpus_OFFSET
#endif
#endif /* CONFIG_SMP */

	/* r1 now contains the address of the correct "_cpu" entry for this core */

    /* load "_cpu[x].current" into r2 */
    ldr r2, [r1, #___cpu_t_current_OFFSET]

	/* 
	Call z_get_next_switch_handle() with a NULL-pointer since we haven't done any work 
	to save the thread context yet
	*/
	push {r1, r2, lr}
	movs r0, #0
	bl z_get_next_switch_handle

#if defined(CONFIG_ARMV6_M_ARMV8_M_BASELINE)
	pop {r1, r2, r3} 
	mov lr, r3
#else
	pop {r1, r2, lr}
#endif /* CONFIG_ARMV6_M_ARMV8_M_BASELINE */

	/* 
	R0 is the return value from z_get_next_switch_handle().
	We passed in NULL as the argument, so if there is no thread waiting for switching then
	we will get NULL as the return value in R0.
	In that event, jump to _EXIT_EXC to skip pending the PendSV exception
	*/
	cmp r0, #0
	beq _EXIT_EXC

	/* store the return value from z_get_next_switch_handle (r0) into the
	 (now previous) thread's "switch_to" member */
	movs r3, #_thread_offset_to_switch_to
	adds r3, r2, r3
	str r0, [r3]

	/* store the current thread (r2) in the current thread's switched_from member */
	movs r3, #_thread_offset_to_switched_from
	adds r3, r2, r3
	str r2, [r3]

	/* store the (now previous) thread into the
	 new thread's "switched_from" member */
	movs r3, #_thread_offset_to_switched_from
	adds r3, r0, r3
	str r2, [r3]
	
	/* store the new thread (r0) in the new thread's switch_to member */
	movs r3, #_thread_offset_to_switch_to
	adds r3, r0, r3
	str r0, [r3]

	/* context switch required, pend the PendSV exception */
	ldr r1, =_SCS_ICSR
	ldr r2, =_SCS_ICSR_PENDSV
	str r2, [r1]

#else /* !CONFIG_USE_SWITCH */
	ldr r3, =_kernel

	ldr r1, [r3, #_kernel_offset_to_current]
	ldr r0, [r3, #_kernel_offset_to_ready_q_cache]
	cmp r0, r1
	beq _EXIT_EXC

	/* context switch required, pend the PendSV exception */
	ldr r1, =_SCS_ICSR
	ldr r2, =_SCS_ICSR_PENDSV
	str r2, [r1]
#endif /* CONFIG_USE_SWITCH */

_ExcExitWithGdbStub:

_EXIT_EXC:
#endif /* CONFIG_PREEMPT_ENABLED */

#ifdef CONFIG_STACK_SENTINEL
	push {r0, lr}
	bl z_check_stack_sentinel
#if defined(CONFIG_ARMV6_M_ARMV8_M_BASELINE)
	pop {r0, r1}
	mov lr, r1
#else
	pop {r0, lr}
#endif /* CONFIG_ARMV6_M_ARMV8_M_BASELINE */
#endif /* CONFIG_STACK_SENTINEL */

	bx lr
