#
# Copyright (c) 2025 Mindgrove Technologies
#

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(coremark)

# Your application’s own main() and portme
target_sources(app PRIVATE
  src/main.c
)
