/*
 * Copyright (c) 2013-2014 Wind River Systems, Inc.
 * Copyright (c) 2017-2019 Nordic Semiconductor ASA.
 * Copyright (c) 2020 Stephanos Ioannidis <root@stephanos.io>
 * Copyright (c) 2022 Stuart Alldritt
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Thread context switching for ARM Cortex-M and Cortex-R
 *
 * This module implements the routines necessary for thread context switching
 * on ARM Cortex-A, Cortex-M and Cortex-R CPUs.
 */

#include <zephyr/toolchain.h>
#include <zephyr/linker/sections.h>
#include <offsets_short.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/syscall.h>
#include <zephyr/kernel.h>

// #if defined(CONFIG_CPU_CORTEX_M)
// #include <zephyr/arch/arm/aarch32/cortex_m/cpu.h>
// #endif

_ASM_FILE_PROLOGUE

GDATA(_kernel)

/*
 * Routine to handle context switches
 *
 * This function is directly called either by _isr_wrapper() in case of
 * preemption, or arch_switch() in case of cooperative switching.
 *
 * void z_arm_context_switch(struct k_thread *new, struct k_thread *old);
 */

GTEXT(z_arm_context_switch)
SECTION_FUNC(TEXT, z_arm_context_switch)

#ifdef CONFIG_INSTRUMENT_THREAD_SWITCHING
    /* Register the context switch */
    push {r0, lr}                              // Push r0 and lr onto the thread stack
    bl z_thread_mark_switched_out              // Mark the thread is switched out
#if defined(CONFIG_ARMV6_M_ARMV8_M_BASELINE)
    pop {r0, r1}                               // Restore R0 and R1 from the thread stack
    mov lr, r1                                 // Looks like on some ARM architectures, we cannot "pop" directly into the LR, so we 
                                               // "pop" into R1 first
                                               // TODO: doesn't this destroy the contents of R1?
#else
    pop {r0, lr}                               // Just restore R0 and LR that were previously saved
#endif /* CONFIG_ARMV6_M_ARMV8_M_BASELINE */
#endif /* CONFIG_INSTRUMENT_THREAD_SWITCHING */

    /* In the original ARM Swap code: */
    /* r0 == unused */
    /* r1 == _kernel */
    /* r2 == current / "old" k_thread */
    /* r3 == unused */

    /* Current state of the world: */
    /* r0 == arg0 == "new" k_thread */
    /* r1 == arg1 == current / "old" k_thread */
    /* r2 == unused */
    /* r3 == unused */

    /* Lets re-arrange some things to make it easier to re-use the old Swap code */
    /* Move the "new" k_thread into r3 for safekeeping, we need it later */
    mov r3, r0
    /* Move the "old" / current k_thread into r2 to match the original Swap code */
    mov r2, r1
    
    /* Load _kernel into r1. */
    ldr r1, =_kernel

    /* Final state of the world: */
    /* r0 == unused */
    /* r1 == _kernel */
    /* r2 == current / "old" k_thread */
    /* r3 == "new" k_thread */

#if defined(CONFIG_ARM_STORE_EXC_RETURN)
    strb lr, [r2, #_thread_offset_to_mode_exc_return] /* Store the LSB of LR (aks 'EXC_RETURN') to the thread's 'mode' word. */
#endif

    /* 
    ARM ABI requires that we save r4-r11 + psp whenever a context switch occurs.
    _callee_saved is an arch-defined struct that is large enough to store
    all these registers.
    Get the offset to this structure from the current thread pointer, and store in R0.
    Since the struct is inline with the k_thread, r0 now contains
    the address where we can start writing registers
    */
    ldr r0, =_thread_offset_to_callee_saved
    add r0, r2

    /* save callee-saved + psp in thread */
#if defined(CONFIG_CPU_CORTEX_M)
    mrs ip, PSP
#endif

#if defined(CONFIG_ARMV6_M_ARMV8_M_BASELINE)
    /* 
    Looks like some ARM archs can't "push" from all registers.
    We can break this up into two separate "push" operations.
    First, store the initial set of r4-r7 registers.
    Note that STMEA automatically increments the stack pointer (R0 in this case)
    so we don't need to worry about incrementing that value.
    /*
    stmea r0!, {r4-r7}

    /* Next, copy r8-r11 into r4-r7 */
    mov r4, r8
    mov r5, r9
    mov r6, r10
    mov r7, r11
    
    /* Now, store R4-R7 (which actually contains the values from r8-11) */
    stmea r0!, {r4-r7}

    /* Copy R12 (aka "ip") into R7 */
    mov r7, ip

    /* Store R7 (which actually contains R12) onto the stack as well */
    stmea r0!, {r7}

#elif defined(CONFIG_ARMV7_M_ARMV8_M_MAINLINE)
    /* 
    This ARM arch supports directly pushing all registers 
    into a stack, so do this in one instruction 
    */
    stmia r0, {v1-v8, ip}

#ifdef CONFIG_FPU_SHARING
    /* Assess whether switched-out thread had been using the FP registers. */
    tst lr, #_EXC_RETURN_FTYPE_Msk

    /* If no FP store is necessary, break to the "out_fp_endif" label */
    bne out_fp_endif

    /* FP context was active: set FP state and store callee-saved registers.
     * Note: if Lazy FP stacking is enabled, storing the callee-saved
     * registers will automatically trigger FP state preservation in
     * the thread's stack. This will also clear the FPCCR.LSPACT flag.
     */

    /*
    Since floating point registers are optional, they are stored in a different
    structure than other callee-saved registers. Calculate the new storage address
    needed for the floating point registers
    */
    add r0, r2, #_thread_offset_to_preempt_float

    /* Push the floating point registers into the storage location */
    vstmia r0, {s16-s31}

out_fp_endif:
    /* At this point FPCCR.LSPACT is guaranteed to be cleared,
     * regardless of whether the thread has an active FP context.
     */
#endif /* CONFIG_FPU_SHARING */

#elif defined(CONFIG_ARMV7_R) || defined(CONFIG_AARCH32_ARMV8_R) \
	|| defined(CONFIG_ARMV7_A)

    /* Store rest of process context */
    cps #MODE_SYS
    /* Push R4-R12 onto the stack indicated by R0 */
    stm r0, {r4-r11, sp}
    cps #MODE_SVC

#if defined(CONFIG_FPU_SHARING)
    /* 
    We need to test if this thread is using the FP registers. Get the offset
    to the user_options variable in the thread context, and store the final address
    of the user_options variable in r0
    */
    ldrb r0, [r2, #_thread_offset_to_user_options]

    /* Test if the K_FP_REGS flag is set in the user_options variable, pointed to by r0 */
    tst r0, #K_FP_REGS /* _current->base.user_options & K_FP_REGS */

    /* Jump to out_fp_inactive if FP is not enabled for this thread */
    beq out_fp_inactive

    /* ???? */
    mov ip, #FPEXC_EN
    vmsr fpexc, ip

    /*
     * If the float context pointer is not null, then the VFP has not been
     * used since this thread has used it.  Consequently, the caller-saved
     * float registers have not been saved away, so write them to the
     * exception stack frame.
     *
     * Load the address of the kernel->fp_ctx variable into R0
     */
    ldr r0, [r1, #_kernel_offset_to_fp_ctx]

    /* test if the kernel->fp_ctx variable is NULL */
    cmp r0, #0

    /* If the fp_ctx variable was NULL, jump to out_store_thread_context */
    beq out_store_thread_context

    /* 
    fp_ctx was not NULL. fp_ctx is a pointer to the memory location
    where we should store s0-s15.
    Push s0-s15 onto the provided memory address.
    Note the use of r0! to indicate that r0 is the address of a pointer.
    */    
    vstmia r0!, {s0-s15}

    /* 
    need to also push the FPCSR, Floating Point Status/Control Register.

    FPSCR cannot be pushed directly, we need to copy it into a 
    general purpose register first.

    we're out of available registers. Re-use R1 to store the FPSCR while we store it, then
    re-load the kernel base address into R1 when we're finished
    */
    vmrs r1, fpscr

    /* ????? store FPSCR on the stack ????? */
    stm r0, {r1, ip}

    /* Restore the kernel pointer in r1, we will need it again later */
    ldr r1, =_kernel

out_store_thread_context:
    /* Store s16-s31 to thread context */
    add r0, r2, #_thread_offset_to_preempt_float
    vstmia r0, {s16-s31}

    mov ip, #0
    vmsr fpexc, ip

out_fp_inactive:
    /*
     * The floating context has now been saved to the exception stack
     * frame, so zero out the global pointer to note this.
     */
    mov r0, #0
    str r0, [r1, #_kernel_offset_to_fp_ctx]
#endif /* CONFIG_FPU_SHARING */
#else
#error Unknown ARM architecture
#endif /* CONFIG_ARMV6_M_ARMV8_M_BASELINE */

    /* Protect the kernel state while we play with the thread lists */
#if defined(CONFIG_ARMV6_M_ARMV8_M_BASELINE)
    cpsid i
#elif defined(CONFIG_ARMV7_M_ARMV8_M_MAINLINE)
    movs.n r0, #_EXC_IRQ_DEFAULT_PRIO
    msr BASEPRI_MAX, r0
    isb /* Make the effect of disabling interrupts be realized immediately */
#elif defined(CONFIG_ARMV7_R) || defined(CONFIG_AARCH32_ARMV8_R) \
	|| defined(CONFIG_ARMV7_A)
    /*
     * Interrupts are still disabled from arch_swap so empty clause
     * here to avoid the preprocessor error below
     */
#else
#error Unknown ARM architecture
#endif /* CONFIG_ARMV6_M_ARMV8_M_BASELINE */

    /*
     * Prepare to clear PendSV with interrupts unlocked, but
     * don't clear it yet. PendSV must not be cleared until
     * the new thread is context-switched in since all decisions
     * to pend PendSV have been taken with the current kernel
     * state and this is what we're handling currently.
     */
#if defined(CONFIG_CPU_CORTEX_M)
    ldr v4, =_SCS_ICSR
    ldr v3, =_SCS_ICSR_UNPENDSV
#endif

    /* _kernel is still in r1 */

    /* the thread to run is in R3. Move it into R2 */
    mov r2, r3

    /* OLD CODE: */

    /* Fetch the "new" thread pointer from the kernel's "ready_q_cache" */
    /* ldr r2, [r1, #_kernel_offset_to_ready_q_cache] */
    /* Store the "new" thread pointer in the kernel's "current" variable */
    /* str r2, [r1, #_kernel_offset_to_current] */

    /*
     * Clear PendSV so that if another interrupt comes in and
     * decides, with the new kernel state based on the new thread
     * being context-switched in, that it needs to reschedule, it
     * will take, but that previously pended PendSVs do not take,
     * since they were based on the previous kernel state and this
     * has been handled.
     */

    /* _SCS_ICSR is still in v4 and _SCS_ICSR_UNPENDSV in v3 */
#if defined(CONFIG_CPU_CORTEX_M)
    str v3, [v4, #0]
#endif

#if defined(CONFIG_THREAD_LOCAL_STORAGE)
    /* Grab the TLS pointer */
    ldr r4, =_thread_offset_to_tls
    /* 
    R4 == address of the tls variable in the new thread 
    Note that the TLS variable is itself a pointer.
    */
    adds r4, r2, r4
    /* De-reference the TLS pointer into R0 */
    ldr r0, [r4]

#if defined(CONFIG_CPU_AARCH32_CORTEX_R) || defined(CONFIG_CPU_AARCH32_CORTEX_A)
    /* Store TLS pointer in the "Process ID" register.
     * This register is used as a base pointer to all
     * thread variables with offsets added by toolchain.
     */
    mcr 15, 0, r0, cr13, cr0, 3
#endif

#if defined(CONFIG_CPU_CORTEX_M)
    /* For Cortex-M, store TLS pointer in a global variable,
     * as it lacks the process ID or thread ID register
     * to be used by toolchain to access thread data.
     */
    ldr r4, =z_arm_tls_ptr
    str r0, [r4]
#endif

#endif

#if defined(CONFIG_ARM_STORE_EXC_RETURN)
    /* Restore EXC_RETURN value. */
    ldrsb lr, [r2, #_thread_offset_to_mode_exc_return]
#endif

    /* Restore previous interrupt disable state (irq_lock key)
     * (We clear the arch.basepri field after restoring state)
     */
#if defined(CONFIG_ARMV6_M_ARMV8_M_BASELINE) && (_thread_offset_to_basepri > 124)
    /* Doing it this way since the offset to thread->arch.basepri can in
     * some configurations be larger than the maximum of 124 for ldr/str
     * immediate offsets.
     */
    ldr r4, =_thread_offset_to_basepri
    adds r4, r2, r4

    ldr r0, [r4]
    movs.n r3, #0
    str r3, [r4]
#else
    ldr r0, [r2, #_thread_offset_to_basepri]
    movs r3, #0
    str r3, [r2, #_thread_offset_to_basepri]
#endif

#if defined(CONFIG_ARMV6_M_ARMV8_M_BASELINE)
    /* BASEPRI not available, previous interrupt disable state
     * maps to PRIMASK.
     *
     * Only enable interrupts if value is 0, meaning interrupts
     * were enabled before irq_lock was called.
     */
    cmp r0, #0
    bne _thread_irq_disabled
    cpsie i
_thread_irq_disabled:

#if defined(CONFIG_MPU_STACK_GUARD) || defined(CONFIG_USERSPACE)
    /* Re-program dynamic memory map */
    push {r2,lr}
    mov r0, r2
    bl z_arm_configure_dynamic_mpu_regions
    pop {r2,r3}
    mov lr, r3
#endif

#ifdef CONFIG_USERSPACE
    /* restore mode */
    ldr r3, =_thread_offset_to_mode
    adds r3, r2, r3
    ldr r0, [r3]
    mrs r3, CONTROL
    movs.n r1, #1
    bics r3, r1
    orrs r3, r0
    msr CONTROL, r3

    /* ISB is not strictly necessary here (stack pointer is not being
     * touched), but it's recommended to avoid executing pre-fetched
     * instructions with the previous privilege.
     */
    isb

#endif

    ldr r4, =_thread_offset_to_callee_saved
    adds r0, r2, r4

    /* restore r4-r12 for new thread */
    /* first restore r8-r12 located after r4-r7 (4*4bytes) */
    adds r0, #16
    ldmia r0!, {r3-r7}
    /* move to correct registers */
    mov r8, r3
    mov r9, r4
    mov r10, r5
    mov r11, r6
    mov ip, r7
    /* restore r4-r7, go back 9*4 bytes to the start of the stored block */
    subs r0, #36
    ldmia r0!, {r4-r7}
#elif defined(CONFIG_ARMV7_M_ARMV8_M_MAINLINE)
    /* restore BASEPRI for the incoming thread */
    msr BASEPRI, r0

#ifdef CONFIG_FPU_SHARING
    /* Assess whether switched-in thread had been using the FP registers. */
    tst lr, #_EXC_RETURN_FTYPE_Msk
    beq in_fp_active
    /* FP context inactive for swapped-in thread:
     * - reset FPSCR to 0
     * - set EXC_RETURN.F_Type (prevents FP frame un-stacking when returning
     *   from pendSV)
     */
    movs.n r3, #0
    vmsr fpscr, r3
    b in_fp_endif

in_fp_active:
    /* FP context active:
     * - clear EXC_RETURN.F_Type
     * - FPSCR and caller-saved registers will be restored automatically
     * - restore callee-saved FP registers
     */
    add r0, r2, #_thread_offset_to_preempt_float
    vldmia r0, {s16-s31}
in_fp_endif:
    /* Clear CONTROL.FPCA that may have been set by FP instructions */
    mrs r3, CONTROL
    bic r3, #_CONTROL_FPCA_Msk
    msr CONTROL, r3
    isb
#endif

#if defined(CONFIG_MPU_STACK_GUARD) || defined(CONFIG_USERSPACE)
    /* Re-program dynamic memory map */
    push {r2,lr}
    mov r0, r2 /* _current thread */
    bl z_arm_configure_dynamic_mpu_regions
    pop {r2,lr}
#endif

#ifdef CONFIG_USERSPACE
    /* restore mode */
    ldr r0, [r2, #_thread_offset_to_mode]
    mrs r3, CONTROL
    bic r3, #1
    orr r3, r0
    msr CONTROL, r3

    /* ISB is not strictly necessary here (stack pointer is not being
     * touched), but it's recommended to avoid executing pre-fetched
     * instructions with the previous privilege.
     */
    isb

#endif

    /* load callee-saved + psp from thread */
    add r0, r2, #_thread_offset_to_callee_saved
    ldmia r0, {v1-v8, ip}
#elif defined(CONFIG_ARMV7_R) || defined(CONFIG_AARCH32_ARMV8_R) \
	|| defined(CONFIG_ARMV7_A)
_thread_irq_disabled:
    /* load _kernel into r1 and current k_thread into r2 */
    ldr r1, =_kernel
    ldr r2, [r1, #_kernel_offset_to_current]

    /* addr of callee-saved regs in thread in r0 */
    ldr r0, =_thread_offset_to_callee_saved
    add r0, r2

    /* restore r4-r11 and sp for incoming thread */
    cps #MODE_SYS
    ldm r0, {r4-r11, sp}
    cps #MODE_SVC

#if defined(CONFIG_FPU_SHARING)
    ldrb r0, [r2, #_thread_offset_to_user_options]
    tst r0, #K_FP_REGS /* _current->base.user_options & K_FP_REGS */
    beq in_fp_inactive

    mov r3, #FPEXC_EN
    vmsr fpexc, r3

    /* Restore s16-s31 from thread context */
    add r0, r2, #_thread_offset_to_preempt_float
    vldmia r0, {s16-s31}

    mov r3, #0
    vmsr fpexc, r3

in_fp_inactive:
#endif /* CONFIG_FPU_SHARING */

#if defined (CONFIG_ARM_MPU)
    /* r2 contains k_thread */
    mov r0, r2
    /* Re-program dynamic memory map */
    push {r2, lr}
    bl z_arm_configure_dynamic_mpu_regions
    pop {r2, lr}
#endif
#else
#error Unknown ARM architecture
#endif /* CONFIG_ARMV6_M_ARMV8_M_BASELINE */

#if defined(CONFIG_CPU_CORTEX_M)
    msr PSP, ip
#endif

#ifdef CONFIG_BUILTIN_STACK_GUARD
    /* r2 contains k_thread */
    add r0, r2, #0
    push {r2, lr}
    bl configure_builtin_stack_guard
    pop {r2, lr}
#endif /* CONFIG_BUILTIN_STACK_GUARD */

#ifdef CONFIG_INSTRUMENT_THREAD_SWITCHING
    /* Register the context switch */
    push {r0, lr}
    bl z_thread_mark_switched_in
#if defined(CONFIG_ARMV6_M_ARMV8_M_BASELINE)
    pop {r0, r1}
    mov lr, r1
#else
    pop {r0, lr}
#endif
#endif /* CONFIG_INSTRUMENT_THREAD_SWITCHING */

    /*
     * Cortex-M: return from PendSV exception
     * Cortex-R: return to the caller (z_arm_{exc,int}_exit, or z_arm_svc)
     */
    bx lr