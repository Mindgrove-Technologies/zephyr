/* 
 * @copyright Copyright (c) Mindgrove Technologies Pvt. Ltd 2025. All rights reserved.
 * 
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h> // Provides irq_connect_dynamic

LOG_MODULE_REGISTER(gpio_mindgrove, CONFIG_GPIO_LOG_LEVEL);

#define DT_DRV_COMPAT mindgrove_gpio

/* Register Offsets (relative to 0x40200 base address) */
#define OFF_DIRECTION      0x00 // GPIO_DIRECTION_CTRL_REG
#define OFF_DATA           0x08 // GPIO_DATA_REG
#define OFF_SET            0x10 // GPIO_SET_REG
#define OFF_CLEAR          0x18 // GPIO_CLEAR_REG
#define OFF_TOGGLE         0x20 // GPIO_TOGGLE_REG
// #define OFF_INTR_CONFIG    0x30 // GPIO_INTR_REG

// Placeholder for GPIO register offsets (in bytes)
#define OFF_DATA_INPUT          0x00 // Common location for input/data register
#define OFF_INTR_ENABLE         0x30 // To enable/disable the pin interrupt (Used for Write)
#define OFF_INTR_STATUS_REG     0x4C // Read status (trial: try 0x4C)

/* Configuration Structure */
struct gpio_mindgrove_config {
    struct gpio_driver_config common;
    uintptr_t base;
    /* Interrupt properties */
    int irq_num;
    int irq_priority;
};

/* Data Structure */
struct gpio_mindgrove_data {
    struct gpio_driver_data common;
    sys_slist_t cb;
};

/* Helper Macros */
#define DEV_CFG(dev) ((const struct gpio_mindgrove_config *)(dev)->config)
#define DEV_DATA(dev) ((struct gpio_mindgrove_data *)(dev)->data)
#define DEV_REGS(dev) ((volatile uint32_t *)((const struct gpio_mindgrove_config *)(dev)->config)->base)

static int gpio_mindgrove_pin_configure(const struct device *dev,
                                        gpio_pin_t pin,
                                        gpio_flags_t flags)
{
    volatile uint32_t *base = DEV_REGS(dev);
    
    if (DEV_CFG(dev)->base == 0) {
        LOG_ERR("Invalid base address for %s", dev->name);
        return -EINVAL;
    }

    // 1. Configure Direction (Input/Output)
    if (flags & GPIO_OUTPUT) {
        // Set bit for output (1)
        base[OFF_DIRECTION / 4] |= (1u << pin);
    } else {
        // Clear bit for input (0)
        base[OFF_DIRECTION / 4] &= ~(1u << pin);
    }

    // 2. Clear output latch if set, based on initial state flags (optional)
    if (flags & GPIO_OUTPUT_INIT_HIGH) {
        base[OFF_SET / 4] = (1u << pin);
    } else if (flags & GPIO_OUTPUT_INIT_LOW) {
        base[OFF_CLEAR / 4] = (1u << pin);
    }
    
    return 0;
}

static int gpio_mindgrove_port_get_raw(const struct device *dev,
                                       gpio_port_value_t *value)
{
    volatile uint32_t *base = DEV_REGS(dev);
    *value = base[OFF_DATA / 4];
    return 0;
}

static int gpio_mindgrove_port_set_bits_raw(const struct device *dev,
                                            gpio_port_pins_t pins)
{
    volatile uint32_t *base = DEV_REGS(dev);
    base[OFF_SET / 4] = pins;
    return 0;
}

static int gpio_mindgrove_port_clear_bits_raw(const struct device *dev,
                                              gpio_port_pins_t pins)
{
    volatile uint32_t *base = DEV_REGS(dev);
    base[OFF_CLEAR / 4] = pins;
    return 0;
}

static int gpio_mindgrove_port_toggle_bits(const struct device *dev,
                                           gpio_port_pins_t pins)
{
    volatile uint32_t *base = DEV_REGS(dev);
    base[OFF_TOGGLE / 4] = pins;
    return 0;
}

static int gpio_mindgrove_pin_interrupt_configure(const struct device *dev,
                                                  gpio_pin_t pin,
                                                  enum gpio_int_mode mode,
                                                  enum gpio_int_trig trig)
{
    volatile uint32_t *base = DEV_REGS(dev);
    uint32_t mask = (1u << pin);
    uint32_t intr_reg = base[OFF_INTR_ENABLE / 4];

    // 1. Disable interrupt: Clear the enable bit for the pin
    intr_reg &= ~mask;

    if (mode != GPIO_INT_MODE_DISABLED) {
        // 2. Configure mode/trigger
        if (trig == GPIO_INT_TRIG_HIGH) { // Assuming High/Rising Edge
            // Set bit (assuming high/rising edge is '1')
            intr_reg |= mask;
        } else if (trig == GPIO_INT_TRIG_LOW) { // Assuming Low/Falling Edge
            // Clear bit (assuming low/falling edge is '0')
            intr_reg &= ~mask;
        }
        
        // 3. Enable interrupt bit in the local GPIO register
        intr_reg |= mask; 
    }

    base[OFF_INTR_ENABLE / 4] = intr_reg;
    return 0;
}

/* Common ISR for Direct Interrupts (Single IRQ line for all GPIOs) */
static void gpio_mindgrove_isr(const void *arg)
{
    const struct device *dev = (const struct device *)arg;
    struct gpio_mindgrove_data *data = DEV_DATA(dev);
    
    volatile uint32_t *base = DEV_REGS(dev);
    uint32_t status = base[0x40 / 4]; 

    if (status) {
        // Clear pending interrupts
        base[0x40 / 4] = status; 
        
        gpio_fire_callbacks(&data->cb, dev, status);
    }
}

static int gpio_mindgrove_manage_callback(const struct device *dev,
                                          struct gpio_callback *callback, bool set)
{
    struct gpio_mindgrove_data *data = DEV_DATA(dev);
    return gpio_manage_callback(&data->cb, callback, set);
}


static int gpio_mindgrove_init(const struct device *dev)
{
    const struct gpio_mindgrove_config *cfg = DEV_CFG(dev);
    struct gpio_mindgrove_data *data = DEV_DATA(dev);

    if (cfg->base == 0) {
        LOG_ERR("Base address is zero. Check 'reg' property in DTS.");
        return -ENODEV;
    }

    sys_slist_init(&data->cb);

    // Direct Interrupt Configuration (if IRQ is present in DT)
    if (cfg->irq_num > 0) {
        // standard Zephyr function irq_connect_dynamic
        if (irq_connect_dynamic(cfg->irq_num, 
                                       cfg->irq_priority, 
                                       gpio_mindgrove_isr,
                                       (void *)dev, 
                                       0) < 0) {
            LOG_ERR("Failed to connect IRQ %d", cfg->irq_num);
            return -EIO;
        }
        
        irq_enable(cfg->irq_num);
        LOG_DBG("GPIO %s IRQ %d connected and enabled.", dev->name, cfg->irq_num);
    } else {
        LOG_DBG("GPIO %s configured without IRQ.", dev->name);
    }

    return 0;
}

static const struct gpio_driver_api gpio_mindgrove_api = {
    .pin_configure = gpio_mindgrove_pin_configure,
    .port_get_raw = gpio_mindgrove_port_get_raw,
    .port_set_bits_raw = gpio_mindgrove_port_set_bits_raw,
    .port_clear_bits_raw = gpio_mindgrove_port_clear_bits_raw,
    .port_toggle_bits = gpio_mindgrove_port_toggle_bits,
    .pin_interrupt_configure = gpio_mindgrove_pin_interrupt_configure,
    .manage_callback = gpio_mindgrove_manage_callback,
};

/* Device Instantiation Macros */
#define GPIO_INIT(n) \
    static const struct gpio_mindgrove_config gpio_mindgrove_config_##n = { \
        .common = { \
            .port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(n), \
        }, \
        .base = DT_INST_REG_ADDR(n), \
        .irq_num = DT_INST_IRQN(n), \
        .irq_priority = DT_INST_IRQ_BY_IDX(n, 0, priority), \
    }; \
    static struct gpio_mindgrove_data gpio_mindgrove_data_##n; \
    DEVICE_DT_INST_DEFINE(n, \
                          gpio_mindgrove_init, \
                          NULL, \
                          &gpio_mindgrove_data_##n, \
                          &gpio_mindgrove_config_##n, \
                          POST_KERNEL, \
                          CONFIG_GPIO_INIT_PRIORITY, \
                          &gpio_mindgrove_api);

DT_INST_FOREACH_STATUS_OKAY(GPIO_INIT)
