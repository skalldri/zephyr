#include <zephyr/sys/arch_interface.h>

// Need a non-inlined version of this function that can be called from ASM code
_cpu_t *arm32_curr_cpu(void) {
    return arch_curr_cpu();
}

LINKER_KEEP(arm32_curr_cpu);

