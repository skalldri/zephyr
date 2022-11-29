/*
 * Copyright (c) 2013-2016 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Private kernel definitions (ARM)
 *
 * This file contains private kernel function definitions and various
 * other definitions for the 32-bit ARM Cortex-A/R/M processor architecture
 * family.
 *
 * This file is also included by assembly language files which must #define
 * _ASMLANGUAGE before including this header file.  Note that kernel
 * assembly source files obtains structure offset values via "absolute symbols"
 * in the offsets.o module.
 */

#ifndef ZEPHYR_ARCH_ARM_INCLUDE_AARCH32_KERNEL_ARCH_FUNC_H_
#define ZEPHYR_ARCH_ARM_INCLUDE_AARCH32_KERNEL_ARCH_FUNC_H_

#include <kernel_arch_data.h>

#include <zephyr/sys/__assert.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef _ASMLANGUAGE
extern void z_arm_fault_init(void);
extern void z_arm_cpu_idle_init(void);
#ifdef CONFIG_ARM_MPU
extern void z_arm_configure_static_mpu_regions(void);
extern void z_arm_configure_dynamic_mpu_regions(struct k_thread *thread);
extern int z_arm_mpu_init(void);
#endif /* CONFIG_ARM_MPU */
#ifdef CONFIG_ARM_AARCH32_MMU
extern int z_arm_mmu_init(void);
#endif /* CONFIG_ARM_AARCH32_MMU */

static ALWAYS_INLINE void arch_kernel_init(void)
{
	z_arm_interrupt_stack_setup();
	z_arm_exc_setup();
	z_arm_fault_init();
	z_arm_cpu_idle_init();
	z_arm_clear_faults();
#if defined(CONFIG_ARM_MPU)
	z_arm_mpu_init();
	/* Configure static memory map. This will program MPU regions,
	 * to set up access permissions for fixed memory sections, such
	 * as Application Memory or No-Cacheable SRAM area.
	 *
	 * This function is invoked once, upon system initialization.
	 */
	z_arm_configure_static_mpu_regions();
#endif /* CONFIG_ARM_MPU */
#if defined(CONFIG_ARM_AARCH32_MMU)
	z_arm_mmu_init();
#endif /* CONFIG_ARM_AARCH32_MMU */
}

#if !defined(CONFIG_USE_SWITCH)
static ALWAYS_INLINE void
arch_thread_return_value_set(struct k_thread *thread, unsigned int value)
{
	thread->arch.swap_return_value = value;
}
#endif

#if !defined(CONFIG_MULTITHREADING) && defined(CONFIG_CPU_CORTEX_M)
extern FUNC_NORETURN void z_arm_switch_to_main_no_multithreading(
	k_thread_entry_t main_func,
	void *p1, void *p2, void *p3);

#define ARCH_SWITCH_TO_MAIN_NO_MULTITHREADING \
	z_arm_switch_to_main_no_multithreading

#endif /* !CONFIG_MULTITHREADING && CONFIG_CPU_CORTEX_M */

#if defined(CONFIG_USE_SWITCH)
static inline void arch_switch(void *switch_to, void **switched_from)
{
	__ASSERT(switch_to != NULL, "switch_to cannot be NULL");
	__ASSERT(switched_from != NULL, "switched_from cannot be NULL");

	struct k_thread *new = switch_to;
	struct k_thread *old = CONTAINER_OF(switched_from, struct k_thread,
					    switch_handle);

	__ASSERT(
		old->arch.switch_to == NULL && new->arch.switched_from == NULL, 
		"Context switch pending during call to arch_switch()\n"
		"old_thread: %p\n"
		"old_thread->switch_to: %p\n"
		"new_thread: %p\n"
		"new_thread->switched_from: %p\n",
		(void*)old,
		(void*)old->arch.switch_to,
		(void*)new,
		(void*)new->arch.switched_from);

	// Use this to pass data into our upcoming PendSV interrupt
	// PendSV interrupt is responsible for setting this back to NULL
	// after it has completed the context switch
	if ((old->base.thread_state & _THREAD_DUMMY) == 0) {
		old->arch.switch_to = new;
		new->arch.switched_from = old;
	} else {
		// Special case: during startup, we will be asked to switch from the dummy_thread 
		// to a real thread.
		// The dummy thread is allocated on the ISR stack, which will become corrupt when we
		// perform our initial PendSV (and we want to reclaim the entire stack anyway...).
		// We pass in a "NULL" switched_from into the PendSV handler to indicate to the context switching
		// code that we should skip saving the dummy_thread context, and in fact should not interract with
		// the dummy thread at all.
		new->arch.switched_from = NULL;
	}

	// printk("arch_switch(%d): old (%p) -> new (%p)\n", arch_proc_id(), (void*)old, (void*)new);

#if defined(CONFIG_CPU_CORTEX_M)
	/* set pending bit to make sure we will take a PendSV exception */
	SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;

	/* clear mask or enable all irqs to take a pendsv */
	irq_unlock(0);
#elif defined(CONFIG_CPU_AARCH32_CORTEX_R) || defined(CONFIG_CPU_AARCH32_CORTEX_A)
	z_arm_cortex_r_svc();
	irq_unlock(key);
#else
#error "This arch is not supported"
#endif // CONFIG_CPU_CORTEX_M
}
#endif // CONFIG_USE_SWITCH

extern FUNC_NORETURN void z_arm_userspace_enter(k_thread_entry_t user_entry,
					       void *p1, void *p2, void *p3,
					       uint32_t stack_end,
					       uint32_t stack_start);

extern void z_arm_fatal_error(unsigned int reason, const z_arch_esf_t *esf);

#endif /* _ASMLANGUAGE */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_ARCH_ARM_INCLUDE_AARCH32_KERNEL_ARCH_FUNC_H_ */
