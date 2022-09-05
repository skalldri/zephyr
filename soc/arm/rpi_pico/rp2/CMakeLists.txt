# Copyright (c) 2021 Yonatan Schachter
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(soc.c)

if(CONFIG_SMP)
    zephyr_library_sources(soc_mp.c)
endif()

if(CONFIG_ATOMIC_OPERATIONS_ARCH)
    zephyr_library_sources(atomic.c)
endif()