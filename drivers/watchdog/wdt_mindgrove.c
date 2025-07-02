/*
 * Copyright (c) Mindgrove Technologies Pvt. Ltd 2023. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Watchdog (WDT) Driver for mindgrove
 */

#define DT_DRV_COMPAT mindgrove_wdt

#include <zephyr/kernel.h>
#include <soc.h>
#include <zephyr/drivers/watchdog.h>

#define LOG_LEVEL CONFIG_WDT_LOG_LEVEL
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>


#define INTERRUPT_MODE			1
#define SOFT_RESET_MODE			7
#define RESET_MODE				3

#define WD_CYCLES_REG_ADDR		0x40400
#define CONTROL_REG_ADDR		0x40408
#define RESET_CYCLES_REG_ADDR	0x40410
#define WDT_MAX_CYCLES         0xFFFFFFFF  // 32-bit max

#define WDT_NODE DT_NODELABEL(watchdog0)

#define WDT_BASE_ADDR DT_REG_ADDR(WDT_NODE)
#define WDT_CLK_FREQ  DT_PROP(WDT_NODE, clock_frequency)
struct wdt_mindgrove_device_config {
    uintptr_t regs;
    uint32_t sys_clk_freq;
};

struct wdt_mindgrove_reg {
    volatile uint32_t wd_cycles;
     uint32_t reserved1;
    volatile uint16_t control_reg;
    uint16_t reserved2;
    uint32_t reserved3;
    volatile uint16_t reset_cycles;
};
// struct __attribute__((packed, aligned(4))) wdt_soc_reg {
//     volatile uint32_t wd_cycles;       // Offset 0x00
//     volatile uint32_t reserved1;       // Offset 0x04
//     volatile uint16_t control_reg;     // Offset 0x08
//     volatile uint16_t reserved2;       // Offset 0x0A
//     volatile uint32_t reserved3;       // Offset 0x0C
//     volatile uint16_t reset_cycles;    // Offset 0x10
// };
struct wdt_mindgrove_dev_data {
    wdt_callback_t cb;
    uint16_t mode;
    uint16_t rcycles;
    uint32_t wcycles;
    bool enable_cb;
    bool timeout_valid;
};

#define DEV_REG(dev) \
	((struct wdt_mindgrove_reg *) \
	 ((const struct wdt_mindgrove_device_config *const)(dev)->config)->regs)

static int wdt_mindgrove_disable(const struct device *dev)
{
    volatile struct wdt_mindgrove_reg *wdt = DEV_REG(dev);
	wdt->control_reg=wdt->control_reg && 0xFFFE;
    // wdt->control_reg =(wdt->control_reg && 0xFFFE);// Clear enable bit
    return 0;
}

static int wdt_mindgrove_setup(const struct device *dev, uint8_t options)
{
    volatile struct wdt_mindgrove_reg *wdt = DEV_REG(dev);
    struct wdt_mindgrove_dev_data *data = dev->data;

    if (!data->timeout_valid) {
        return -EINVAL;
    }
    wdt->wd_cycles     = data->wcycles;
    wdt->reset_cycles  = data->rcycles;
    wdt->control_reg   = options;
    return 0;
}

static int wdt_mindgrove_install_timeout(const struct device *dev,
                                   const struct wdt_timeout_cfg *cfg)
{
    struct wdt_mindgrove_dev_data *data = dev->data;
    const struct wdt_mindgrove_device_config *config = dev->config;

    uint32_t timeout_cycles;
    uint32_t reset_cycles;

    if (data->timeout_valid) {
        return -ENOMEM;
    }

    if (cfg->window.min != 0 || cfg->window.max == 0) {
        return -EINVAL;
    }

    timeout_cycles = ((uint64_t)cfg->window.max * config->sys_clk_freq) / 1000;

    if (timeout_cycles > WDT_MAX_CYCLES) {
        return -EINVAL;
    }

    reset_cycles = timeout_cycles / 4;

    data->wcycles = timeout_cycles;
    data->rcycles = reset_cycles;
    data->timeout_valid = true;

    return 0;
}

static int wdt_mindgrove_feed(const struct device *dev, int channel_id)
{
	wdt_mindgrove_disable(dev);
	wdt_mindgrove_setup(dev,3);
}

static const struct wdt_driver_api wdt_mindgrove_api = {
	.setup = wdt_mindgrove_setup,
	.disable = wdt_mindgrove_disable,
	.install_timeout = wdt_mindgrove_install_timeout,
	.feed = wdt_mindgrove_feed,
};



static int wdt_mindgrove_init(const struct device *dev)
{
	return 0;
}

static struct wdt_mindgrove_dev_data wdt_mindgrove_data;

static const struct wdt_mindgrove_device_config wdt_mindgrove_cfg = {
	.regs = WDT_BASE_ADDR,
    .sys_clk_freq = WDT_CLK_FREQ 

};

DEVICE_DT_INST_DEFINE(0, wdt_mindgrove_init, NULL,
		      &wdt_mindgrove_data, &wdt_mindgrove_cfg, PRE_KERNEL_1,
		      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &wdt_mindgrove_api);
