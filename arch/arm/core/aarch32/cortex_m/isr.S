#include <zephyr/toolchain.h>
#include <zephyr/linker/sections.h>
#include <offsets_short.h>
#include <zephyr/arch/cpu.h>

_ASM_FILE_PROLOGUE

GTEXT(z_arm_sys_clock_wrapper)
SECTION_SUBSEC_FUNC(TEXT,_isr_handlers,z_arm_sys_clock_wrapper)
    push {lr}
    bl sys_clock_isr

#if defined(CONFIG_ARMV6_M_ARMV8_M_BASELINE)
	pop {r0}
	mov lr, r0
#else
	pop {lr}
#endif /* CONFIG_ARMV6_M_ARMV8_M_BASELINE */

    b z_arm_int_exit

GTEXT(z_arm_nmi_wrapper)
SECTION_SUBSEC_FUNC(TEXT,_isr_handlers,z_arm_nmi_wrapper)
    push {lr}
    bl z_arm_nmi

#if defined(CONFIG_ARMV6_M_ARMV8_M_BASELINE)
	pop {r0}
	mov lr, r0
#else
	pop {lr}
#endif /* CONFIG_ARMV6_M_ARMV8_M_BASELINE */

    b z_arm_int_exit