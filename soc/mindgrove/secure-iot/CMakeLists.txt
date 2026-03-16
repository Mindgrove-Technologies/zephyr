
if(CONFIG_SOC_SECURE_IOT)
    if (CONFIG_RISCV_GENERIC_TOOLCHAIN)
        zephyr_compile_options(-march=rv64imafdc_zicsr_zifencei -mabi=lp64d -mcmodel=medany -Os)
    else()
        zephyr_compile_options(-march=rv64imafdc_zicsr_zifencei -mabi=lp64d -mcmodel=medany -Os)
    endif()
endif()

zephyr_sources()
zephyr_include_directories(.)

set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/riscv/common/linker.ld CACHE INTERNAL "")
zephyr_sources(soc.c)