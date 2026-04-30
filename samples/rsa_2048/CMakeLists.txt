# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.20.0)

# MUST be before find_package
list(APPEND ZEPHYR_EXTRA_MODULES
    /home/deeptha/work/zephyr/modules/crypto/mbedtls
)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(rsa_2048)

target_sources(app PRIVATE src/main.c)

if(CONFIG_XIP)
    zephyr_code_relocate(FILES src/main.c LOCATION ROM_TEXT NOCOPY)
    zephyr_code_relocate(FILES src/main.c LOCATION RAM_DATA)
    zephyr_code_relocate(FILES src/main.c LOCATION RAM_BSS)
endif()