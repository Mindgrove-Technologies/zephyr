
if(CONFIG_SOC_RISCV64_SHAKTI)
    if (CONFIG_RISCV_GENERIC_TOOLCHAIN)
        zephyr_compile_options(-march=rv64imafdc -mabi=lp64d -O1 -mcmodel=medany)
    else()
        zephyr_compile_options(-g -march=rv64ima -mabi=lp64 -O)
    endif()
endif()

zephyr_sources()
# zephyr_sources_ifdef(CONFIG_BOARD_VISION_SOC soc.c)
