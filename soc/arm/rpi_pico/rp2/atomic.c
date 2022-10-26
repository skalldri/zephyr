
#include <hardware/sync.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/arch_interface.h>
#include <zephyr/init.h>

// Implementation heavily based on atomic_c.c

// Use "OS Spinlock 1" from the Pico SDK to implement atomic services for the Pico
#define ATOMIC_HW_SPINLOCK PICO_SPINLOCK_ID_OS1

#define ATOMIC_BOILERPLATE_START(_return_type, _default_return) \
    spin_lock_t* lock = spin_lock_instance(ATOMIC_HW_SPINLOCK); \
    _return_type ret = _default_return; \
    int key = arch_irq_lock(); \
    spin_lock_unsafe_blocking(lock);

#define ATOMIC_BOILERPLATE_END() \
    spin_unlock_unsafe(lock); \
    arch_irq_unlock(key); \
    return ret;

// Atomic compare-and-set
bool atomic_cas(atomic_t *target, atomic_val_t old_value,
             atomic_val_t new_value)
{
    ATOMIC_BOILERPLATE_START(bool, false);

	if (*target == old_value) {
		*target = new_value;
		ret = true;
	}

    ATOMIC_BOILERPLATE_END();
}

bool atomic_ptr_cas(atomic_ptr_t *target, atomic_ptr_val_t old_value,
                  atomic_ptr_val_t new_value)
{
    ATOMIC_BOILERPLATE_START(bool, false);

	if (*target == old_value) {
		*target = new_value;
		ret = true;
	}

    ATOMIC_BOILERPLATE_END();
}

atomic_val_t atomic_add(atomic_t *target, atomic_val_t value)
{
    ATOMIC_BOILERPLATE_START(atomic_val_t, value);

    ret = *target;
    *target += value;

    ATOMIC_BOILERPLATE_END();
}

atomic_val_t atomic_sub(atomic_t *target, atomic_val_t value)
{
    ATOMIC_BOILERPLATE_START(atomic_val_t, value);

    ret = *target;
    *target -= value;

    ATOMIC_BOILERPLATE_END();
}

atomic_val_t atomic_inc(atomic_t *target)
{
    return atomic_add(target, 1);
}

atomic_val_t atomic_dec(atomic_t *target)
{
    return atomic_sub(target, 1);
}

atomic_val_t atomic_get(const atomic_t *target)
{
    return *target;
}

atomic_ptr_val_t atomic_ptr_get(const atomic_ptr_t *target)
{
    return *target;
}

atomic_val_t atomic_set(atomic_t *target, atomic_val_t value)
{
    ATOMIC_BOILERPLATE_START(atomic_val_t, value);

    ret = *target;
    *target = value;

    ATOMIC_BOILERPLATE_END();
}

atomic_ptr_val_t atomic_ptr_set(atomic_ptr_t *target, atomic_ptr_val_t value)
{
    ATOMIC_BOILERPLATE_START(atomic_ptr_val_t, value);

    ret = *target;
    *target = value;

    ATOMIC_BOILERPLATE_END();
}

atomic_val_t atomic_clear(atomic_t *target)
{
    return atomic_set(target, 0);
}

atomic_ptr_val_t atomic_ptr_clear(atomic_ptr_t *target)
{
    return atomic_ptr_set(target, NULL);
}

atomic_val_t atomic_or(atomic_t *target, atomic_val_t value)
{
    ATOMIC_BOILERPLATE_START(atomic_val_t, value);

    ret = *target;
    *target |= value;

    ATOMIC_BOILERPLATE_END();
}

atomic_val_t atomic_xor(atomic_t *target, atomic_val_t value)
{
    ATOMIC_BOILERPLATE_START(atomic_val_t, value);

    ret = *target;
    *target ^= value;

    ATOMIC_BOILERPLATE_END();
}

atomic_val_t atomic_and(atomic_t *target, atomic_val_t value)
{
    ATOMIC_BOILERPLATE_START(atomic_val_t, value);

    ret = *target;
    *target &= value;

    ATOMIC_BOILERPLATE_END();
}

atomic_val_t atomic_nand(atomic_t *target, atomic_val_t value)
{
    ATOMIC_BOILERPLATE_START(atomic_val_t, value);

    ret = *target;
    *target = ~(*target & value);

    ATOMIC_BOILERPLATE_END();
}

void rp2040_atomic_init() {
    // Initialize our spinlock HW
    spin_lock_init(ATOMIC_HW_SPINLOCK);
}