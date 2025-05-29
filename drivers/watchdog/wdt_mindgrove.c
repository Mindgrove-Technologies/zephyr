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
LOG_MODULE_REGISTER(wdt_sifive);

#define INTERRUPT_MODE			1
#define SOFT_RESET_MODE			7
#define RESET_MODE				3

#define WD_CYCLES_REG_ADDR		0x40400
#define CONTROL_REG_ADDR		0x40408
#define RESET_CYCLES_REG_ADDR	0x40410

volatile uint16_t *control_reg = (uint16_t*) (CONTROL_REG_ADDR);
	 volatile uint32_t *wd_cycles = (uint32_t*) (WD_CYCLES_REG_ADDR);
	 volatile uint16_t *reset_cycles = (uint16_t*) (RESET_CYCLES_REG_ADDR);


struct wdt_mindgrove_device_config {
	uintptr_t regs;
	uint16_t mode;
	uint16_t rcycles;
	uint32_t wcycles;

}obj;


struct wdt_mindgrove_dev_data {
	wdt_callback_t cb;
	bool enable_cb;
	bool timeout_valid;
};

static int wdt_mindgrove_disable(const struct device *dev)
{
	*control_reg=*control_reg && 0xFFFE;
	return 0;
}

static int wdt_mindgrove_setup(const struct device *dev, uint8_t options)
{
	*control_reg=options;
	*reset_cycles=obj.rcycles;
	*wd_cycles=obj.wcycles;
	return 0;
}


static int wdt_mindgrove_install_timeout(const struct device *dev,
				      const struct wdt_timeout_cfg *cfg)
{
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
	.feed = wdt_mindgrove_feed,
};



static int wdt_mindgrove_init(const struct device *dev)
{
	return 0;
}

static struct wdt_mindgrove_dev_data wdt_mindgrove_data;

static const struct wdt_mindgrove_device_config wdt_mindgrove_cfg = {
	.regs = DT_INST_REG_ADDR(0), 
	.rcycles = DT_INST_PROP(0, rcycles), 
	.wcycles = DT_INST_PROP(0, wcycles)
};

DEVICE_DT_INST_DEFINE(0, wdt_mindgrove_init, NULL,
		      &wdt_mindgrove_data, &wdt_mindgrove_cfg, PRE_KERNEL_1,
		      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &wdt_mindgrove_api);
