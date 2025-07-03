/* 
 * @copyright Copyright (c) Mindgrove Technologies Pvt. Ltd 2025. All rights reserved.
 * 
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <soc.h>
#include <zephyr/sys/util.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/interrupt_controller/riscv_plic.h>
#include <zephyr/sw_isr_table.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/drivers/gpio.h>
#include "gpio_mindgrove.h"

#define DT_DRV_COMPAT mindgrove_gpio

typedef void (*gpio_mindgrove_cfg_func_t)(void);

typedef struct gpio_mindgrove_regs_t
{
    uint32_t  direction;               /*! direction register */
    uint32_t  reserved0;                /*! reserved for future use */
    uint32_t  data;                    /*! data register */
    uint32_t  reserved1;                /*! reserved for future use */
    uint32_t  set;                 /*! set register */
    uint32_t  reserved2;                /*! reserved for future use */
    uint32_t  clear;                   /*! clear register */
    uint32_t  reserved3;                /*! reserved for future use */
    uint32_t  toggle;               /*! toggle register */
    uint32_t  reserved4;                /*! reserved for future use */
    uint8_t  qualification;    /*! qualification register */
    uint8_t  reserved5;                /*! reserved for future use */
    uint16_t  reserved6;              /*! reserved for future use */
    uint32_t  reserved12;              /*! reserved for future use */
    uint32_t  intr_config;     /*! interrupt configuration register */
    uint32_t  reserved7;              /*! reserved for future use */
};


struct gpio_mindgrove_config{
    struct gpio_driver_config common;
    uintptr_t gpio_base_addr;
    uint32_t gpio_irq_base;
    gpio_mindgrove_cfg_func_t gpio_cfg_func;
    uint32_t gpio_mode;
};

struct gpio_mindgrove_data {
    struct gpio_driver_data common;
    sys_slist_t cb;

};

/* Helper Macros for GPIO */
#define DEV_GPIO_CFG(dev)						\
	((const struct gpio_mindgrove_config * const)(dev)->config)
#define DEV_GPIO(dev)							\
	((volatile struct gpio_mindgrove_regs_t *)(DEV_GPIO_CFG(dev))->gpio_base_addr)
#define DEV_GPIO_DATA(dev)				\
	((struct gpio_mindgrove_data *)(dev)->data)

int gpio_mindgrove_init(const struct device *dev){    
    volatile struct gpio_mindgrove_regs_t *gpio = DEV_GPIO(dev);
    const struct gpio_mindgrove_config *cfg = DEV_GPIO_CFG(dev);
    return 0;
}

int gpio_mindgrove_pin_configure (const struct device *dev, 
                        gpio_pin_t pin, 
                        gpio_flags_t flags){
    volatile struct gpio_mindgrove_regs_t *gpio = DEV_GPIO(dev);
    const struct gpio_mindgrove_config *cfg = DEV_GPIO_CFG(dev);
    if(flags & GPIO_OUTPUT){
        gpio->direction |= (1 << pin);
    }
    else{
        gpio->direction &= ~(1 << pin);
    }
    return 0;
}

int gpio_mindgrove_pin_get_raw(const struct device *dev,
                    gpio_port_value_t *pin)
{
    volatile struct gpio_mindgrove_regs_t *gpio = DEV_GPIO(dev);
    return gpio->data;
}

int gpio_mindgrove_pin_set_raw(const struct device *dev,
                    gpio_port_value_t pin)
{
    volatile struct gpio_mindgrove_regs_t *gpio = DEV_GPIO(dev);   
    const struct gpio_mindgrove_config *cfg = DEV_GPIO_CFG(dev);
    gpio ->set = pin;
    return 0;
}

int gpio_mindgrove_pin_toggle(const struct device *dev,
                    gpio_port_value_t pin)
{
    volatile struct gpio_mindgrove_regs_t *gpio = DEV_GPIO(dev);
    gpio ->toggle = pin;
    return 0;
}

int gpio_mindgrove_pin_clear_raw(const struct device *dev,
                    gpio_port_value_t pin)
{
    volatile struct gpio_mindgrove_regs_t *gpio = DEV_GPIO(dev);   
    gpio ->clear = pin;
    return 0;
}

//-------Function WIP----------
static inline unsigned int gpio_mindgrove_pin_irq(unsigned int base_irq, int pin)
{
    unsigned int level = irq_get_level(base_irq);
    volatile unsigned int pin_irq = 0;

    pin_irq = base_irq + pin;

    return pin_irq;
}

static int gpio_mindgrove_irq_handler(const struct device *dev)
{
    struct gpio_mindgrove_data *data = DEV_GPIO_DATA(dev);
    volatile struct gpio_mindgrove_regs_t *gpio_reg = DEV_GPIO(dev);
    const struct gpio_mindgrove_config *cfg = DEV_GPIO_CFG(dev); 

    uint8_t pin = ((uint8_t)(cfg->gpio_irq_base >> CONFIG_1ST_LEVEL_INTERRUPT_BITS) - 1) - 1 ; // This logic needs fixing
    
    gpio_reg->intr_config &= ~(BIT(pin));

    gpio_fire_callbacks(&data->cb, dev, BIT(pin));
    
    return 0;
}

static int gpio_mindgrove_isr(const struct device *dev)
{
    printf("Entered GPIO ISR()\n");
}

int gpio_mindgrove_pin_interrupt_configure(const struct device *dev, 
                                                gpio_pin_t pin, 
                                                enum gpio_int_mode mode,
					       enum gpio_int_trig trig)
{
    volatile struct gpio_mindgrove_regs_t *gpio_reg = DEV_GPIO(dev);
    const struct gpio_mindgrove_config *cfg = DEV_GPIO_CFG(dev);

    // Initially disable interrupt for all 32 GPIOs
    gpio_reg->intr_config &= ~(0xFFFFFFFF); 
    
    if(trig == 1)
    {
        gpio_reg->intr_config &= ~(1 << pin);
    }
    else
    {
        gpio_reg->intr_config |= (1 << pin);
    }

    irq_enable(gpio_mindgrove_pin_irq(cfg->gpio_irq_base, pin));

    return 0;
}

static const struct gpio_driver_api gpio_mindgrove_driver = {
    .pin_configure              = gpio_mindgrove_pin_configure,
    .port_get_raw               = gpio_mindgrove_pin_get_raw,   
    .port_set_bits_raw          = gpio_mindgrove_pin_set_raw,    
    .port_clear_bits_raw        = gpio_mindgrove_pin_clear_raw,
    .port_toggle_bits           = gpio_mindgrove_pin_toggle, 
    .pin_interrupt_configure    = gpio_mindgrove_pin_interrupt_configure,  
};

static void gpio_mindgrove_cfg(void){
    uint32_t gpio_pin;
    gpio_pin = (1 << gpio_pin);
}

static const struct gpio_mindgrove_config gpio_mindgrove_config0 ={
    .gpio_base_addr     = GPIO_START,
    .gpio_irq_base      = GPIO_IRQ_BASE,
    .gpio_cfg_func      = gpio_mindgrove_cfg,
    .gpio_mode          = DT_PROP(DT_NODELABEL(gpio0), config_gpio)
};

static struct gpio_mindgrove_data gpio_mindgrove_data0;

#define GPIO_INIT(inst)	\
DEVICE_DT_INST_DEFINE(inst, \
                gpio_mindgrove_init,  \
                NULL, \
                &gpio_mindgrove_data0, &gpio_mindgrove_config0, \
                PRE_KERNEL_1, CONFIG_GPIO_INIT_PRIORITY, \
                &gpio_mindgrove_driver); 

DT_INST_FOREACH_STATUS_OKAY(GPIO_INIT)