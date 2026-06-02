/*
 * Mindgrove GPIO Driver
 *
 * Hardware facts (confirmed from GDB testing):
 * - GPIO_INTR_REG bit=1 → interrupt fires when pin is LOW
 * - No edge detection, no high-level trigger in hardware
 * - Level-triggered: ISR must clear INTR_ENABLE + PLIC enable before EOI
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/drivers/interrupt_controller/riscv_plic.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>

LOG_MODULE_REGISTER(gpio_mindgrove, CONFIG_GPIO_LOG_LEVEL);

#define DT_DRV_COMPAT mindgrove_gpio

#define OFF_DIRECTION   0x00
#define OFF_DATA        0x08
#define OFF_SET         0x10
#define OFF_CLEAR       0x18
#define OFF_TOGGLE      0x20
#define OFF_INTR_ENABLE 0x30

struct gpio_mindgrove_config {
    struct gpio_driver_config common;
    uintptr_t base;
    uint32_t irq_num;
    uint32_t irq_priority;
};

struct gpio_mindgrove_data {
    struct gpio_driver_data common;
    sys_slist_t cb;
};

#define DEV_CFG(dev)  ((const struct gpio_mindgrove_config *)(dev)->config)
#define DEV_DATA(dev) ((struct gpio_mindgrove_data *)(dev)->data)

static int gpio_mindgrove_pin_configure(const struct device *dev,
                                        gpio_pin_t pin, gpio_flags_t flags)
{
    const struct gpio_mindgrove_config *cfg = DEV_CFG(dev);

    if (flags & GPIO_OUTPUT) {
        sys_write32(sys_read32(cfg->base + OFF_DIRECTION) | BIT(pin),
                    cfg->base + OFF_DIRECTION);
    } else {
        sys_write32(sys_read32(cfg->base + OFF_DIRECTION) & ~BIT(pin),
                    cfg->base + OFF_DIRECTION);
    }
    if (flags & GPIO_OUTPUT_INIT_HIGH) {
        sys_write32(BIT(pin), cfg->base + OFF_SET);
    } else if (flags & GPIO_OUTPUT_INIT_LOW) {
        sys_write32(BIT(pin), cfg->base + OFF_CLEAR);
    }
    return 0;
}

static int gpio_mindgrove_port_get_raw(const struct device *dev,
                                       gpio_port_value_t *value)
{
    *value = sys_read32(DEV_CFG(dev)->base + OFF_DATA);
    return 0;
}

static int gpio_mindgrove_port_set_bits_raw(const struct device *dev,
                                            gpio_port_pins_t pins)
{
    sys_write32(pins, DEV_CFG(dev)->base + OFF_SET);
    return 0;
}

static int gpio_mindgrove_port_clear_bits_raw(const struct device *dev,
                                              gpio_port_pins_t pins)
{
    sys_write32(pins, DEV_CFG(dev)->base + OFF_CLEAR);
    return 0;
}

static int gpio_mindgrove_port_toggle_bits(const struct device *dev,
                                           gpio_port_pins_t pins)
{
    sys_write32(pins, DEV_CFG(dev)->base + OFF_TOGGLE);
    return 0;
}

static int gpio_mindgrove_pin_interrupt_configure(const struct device *dev,
                                                  gpio_pin_t pin,
                                                  enum gpio_int_mode mode,
                                                  enum gpio_int_trig trig)
{
    const struct gpio_mindgrove_config *cfg = DEV_CFG(dev);
    uint32_t intr = sys_read32(cfg->base + OFF_INTR_ENABLE);

    printk("[GPIO] pin=%u mode=%d trig=%d intr_before=0x%08x\n",
           pin, mode, trig, intr);

    /* Disable */
    if (mode == GPIO_INT_MODE_DISABLED) {
        // intr &= ~BIT(pin);
        sys_write32(intr, cfg->base + OFF_INTR_ENABLE);

        if (intr == 0U) {
            irq_disable(cfg->irq_num);
        }
        return 0;
    }

    /* Configure polarity */
    if (trig == GPIO_INT_TRIG_LOW) {
        /* 0 = LOW trigger */
        intr &= ~BIT(pin);
        printk("[GPIO] configured LOW trigger\n");
    } else if (trig == GPIO_INT_TRIG_HIGH) {
        /* 1 = HIGH trigger */
        intr |= BIT(pin);
        printk("[GPIO] configured HIGH trigger\n");
    } else {
        printk("[GPIO] ERROR: Only LEVEL HIGH/LOW supported\n");
        return -ENOTSUP;
    }

    sys_write32(intr, cfg->base + OFF_INTR_ENABLE);

    irq_enable(cfg->irq_num);

    printk("[GPIO] enabled pin=%u intr=0x%08x\n", pin, intr);

    return 0;
}

static void gpio_mindgrove_isr(const void *arg)
{
    const struct device *dev = arg;
    const struct gpio_mindgrove_config *cfg = DEV_CFG(dev);
    struct gpio_mindgrove_data *data = DEV_DATA(dev);

    /*
     * Step 0: Read current pin states
     *
     * IMPORTANT:
     * - This does NOT clear the interrupt (hardware is level-triggered)
     * - It only tells us which pins are currently HIGH/LOW
     */
    uint32_t pins = sys_read32(cfg->base + OFF_DATA);

    printk("[GPIO ISR] Entry: dev=%p irq_num=%u pins=0x%08x\n",
           dev, cfg->irq_num, pins);

    /*
     * Step 1: Temporarily disable interrupt delivery at PLIC
     *
     * WHY:
     * - This prevents re-entry while we are handling the current interrupt
     * - Since the interrupt is LEVEL-triggered, it can retrigger immediately
     *
     * NOTE:
     * - We are NOT disabling GPIO hardware
     * - Only stopping CPU from receiving more interrupts temporarily
     */
    riscv_plic_irq_disable(cfg->irq_num);

    /*
     * Step 2: Fire registered GPIO callbacks
     *
     * - Zephyr will internally filter only the pins of interest
     * - Application logic runs here
     */
    gpio_fire_callbacks(&data->cb, dev, pins);

    /*
     * Step 3: Re-enable interrupt delivery at PLIC
     *
     * IMPORTANT:
     * - If the pin condition is still active (e.g., still LOW),
     *   interrupt will fire again immediately (expected behavior)
     *
     * - This is how level-triggered interrupts work
     */
    riscv_plic_irq_enable(cfg->irq_num);

    printk("[GPIO ISR] Exit\n");
}

static int gpio_mindgrove_manage_callback(const struct device *dev,
                                          struct gpio_callback *callback,
                                          bool set)
{
    return gpio_manage_callback(&DEV_DATA(dev)->cb, callback, set);
}

static int gpio_mindgrove_init(const struct device *dev)
{
    const struct gpio_mindgrove_config *cfg = DEV_CFG(dev);
    struct gpio_mindgrove_data *data = DEV_DATA(dev);

    sys_slist_init(&data->cb);

    printk("[GPIO] init: base=0x%lx irq=%u prio=%u\n",
           (unsigned long)cfg->base, cfg->irq_num, cfg->irq_priority);

    irq_connect_dynamic(cfg->irq_num, cfg->irq_priority,
                        gpio_mindgrove_isr, (void *)dev, 0);
    riscv_plic_set_priority(cfg->irq_num, 2U);

    printk("[GPIO] ISR connected, priority set. PLIC NOT enabled yet.\n");
    return 0;
}

static const struct gpio_driver_api gpio_mindgrove_api = {
    .pin_configure           = gpio_mindgrove_pin_configure,
    .port_get_raw            = gpio_mindgrove_port_get_raw,
    .port_set_bits_raw       = gpio_mindgrove_port_set_bits_raw,
    .port_clear_bits_raw     = gpio_mindgrove_port_clear_bits_raw,
    .port_toggle_bits        = gpio_mindgrove_port_toggle_bits,
    .pin_interrupt_configure = gpio_mindgrove_pin_interrupt_configure,
    .manage_callback         = gpio_mindgrove_manage_callback,
};

#define GPIO_INIT(n) \
static const struct gpio_mindgrove_config gpio_mindgrove_config_##n = { \
    .common = { \
        .port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(n), \
    }, \
    .base         = DT_INST_REG_ADDR(n), \
    .irq_num      = DT_INST_IRQ_BY_IDX(0, 0, irq) + CONFIG_2ND_LVL_ISR_TBL_OFFSET, \
    .irq_priority = DT_INST_IRQ_BY_IDX(n, 0, priority), \
}; \
static struct gpio_mindgrove_data gpio_mindgrove_data_##n; \
DEVICE_DT_INST_DEFINE(n, gpio_mindgrove_init, NULL, \
                      &gpio_mindgrove_data_##n, \
                      &gpio_mindgrove_config_##n, \
                      POST_KERNEL, CONFIG_GPIO_INIT_PRIORITY, \
                      &gpio_mindgrove_api);

DT_INST_FOREACH_STATUS_OKAY(GPIO_INIT)