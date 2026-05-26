/*
 * Copyright (c) 2024 Mindgrove Technologies
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT mindgrove_pinctrl

#include <zephyr/drivers/pinctrl.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/device.h>
#include <pinctrl_soc.h>

#define LOG_LEVEL CONFIG_PINCTRL_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pinctrl_mindgrove);

#define MINDGROVE_MUX_COUNT  31U
#define MINDGROVE_MUX_STRIDE 4U

struct pinctrl_mindgrove_config {
    uintptr_t base;
};

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt,
                           uintptr_t reg)
{
    ARG_UNUSED(reg);
	// volatile uint32_t *marker = (volatile uint32_t *)0x80000000UL;
    // *marker = 0xDEADBEEF;
    /* Read base address at compile time — no device lookup needed */
    const uintptr_t base = DT_REG_ADDR(DT_NODELABEL(pinctrl));

    for (uint8_t i = 0U; i < pin_cnt; i++) {
        const uint8_t mux_idx = MINDGROVE_PIN_MUX_IDX(pins[i]);
        const uint8_t bit_val = MINDGROVE_PIN_BIT(pins[i]);

        if (mux_idx >= MINDGROVE_MUX_COUNT) {
            return -EINVAL;
        }

        const mem_addr_t mux_reg = base + (mux_idx * MINDGROVE_MUX_STRIDE);
        uint32_t val = sys_read32(mux_reg);

        if (bit_val) {
            val |= BIT(0);
        } else {
            val &= ~BIT(0);
        }

        sys_write32(val, mux_reg);
    }
    return 0;
}

static int mindgrove_pinctrl_init(const struct device *dev)
{
    /* nothing to initialise at the hardware level */
    return 0;
}

static const struct pinctrl_mindgrove_config pinctrl_mindgrove_cfg_0 = {
    .base = DT_INST_REG_ADDR(0),
};

