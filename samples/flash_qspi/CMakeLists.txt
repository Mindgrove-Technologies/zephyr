# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(flash_qspi)

target_sources(app PRIVATE src/main.c)

set(OPENOCD_EXECUTABLE "/usr/local/bin/mind-openocd")

# if(CONFIG_XIP)
#     zephyr_code_relocate(src/main.c TEXT NOCOPY)
#     zephyr_code_relocate(src/main.c ROM)
#     zephyr_code_relocate(src/main.c ROM_BSS)
# endif()