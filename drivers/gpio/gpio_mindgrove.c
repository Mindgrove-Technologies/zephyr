/*  
 * 
 * @copyright Copyright (c) Mindgrove Technologies Pvt. Ltd 2025. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Hardware register map (confirmed from baremetal driver gpio_driver.c):
 *
 *  0x00  DIRECTION   — bit=1 output, bit=0 input
 *  0x08  DATA        — current pin states (read-only)
 *  0x10  SET         — write BIT(pin) to drive HIGH
 *  0x18  CLEAR       — write BIT(pin) to drive LOW
 *  0x20  TOGGLE      — write BIT(pin) to toggle
 *  0x30  INTR_POL    — interrupt polarity register
 *                      bit=1 → fires when pin is LOW   (TRIG_LOW)
 *                      bit=0 → fires when pin is HIGH  (TRIG_HIGH)
 *  0x40  INTR_STATUS — write-back to clear fired bits
 *
 * Fired-pin detection:
 *   fired when pin_high != trig_high
 *   LOW  trigger (pol=1): trig_high=1, fires when pin=LOW  (0!=1) ✓
 *   HIGH trigger (pol=0): trig_high=0, fires when pin=HIGH (1!=0) ✓
 *
 * PLIC priority:
 *   plic_init() in the Mindgrove PLIC driver zeros ALL source priorities
 *   at boot. IRQ_CONNECT() does not set priority. A source with priority=0
 *   is never delivered by the PLIC regardless of the enable bit.
 *   The GPIO driver must call riscv_plic_set_priority() for each pin's
 *   PLIC source during irq_config_func — this is the driver's responsibility,
 *   not the application's.
 *
 * Re-arm rule:
 *   irq_enable() MUST be called from a work queue (thread context), never
 *   from inside the ISR callback. Calling it before the PLIC completes
 *   the current claim/complete cycle causes immediate re-delivery →
 *   stack overflow.
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/interrupt_controller/riscv_plic.h>

LOG_MODULE_REGISTER(gpio_mindgrove, CONFIG_GPIO_LOG_LEVEL);

#define DT_DRV_COMPAT mindgrove_gpio

#define OFF_DIRECTION   0x00
#define OFF_DATA        0x08
#define OFF_SET         0x10
#define OFF_CLEAR       0x18
#define OFF_TOGGLE      0x20
#define OFF_INTR_POL    0x30
#define OFF_INTR_STATUS 0x40

#define GPIO_NUM_PINS        32U
#define GPIO_PLIC_PRIORITY   2U   /* must be > 0 and > PLIC threshold (0) */

struct gpio_mindgrove_config {
    struct gpio_driver_config common;
    uintptr_t base;
    const uint32_t *irq_nums;
    void (*irq_config_func)(const struct device *dev);
};

struct gpio_mindgrove_data {
    struct gpio_driver_data common;
    sys_slist_t cb;
    uint32_t armed_pins;
};

#define DEV_CFG(dev)  ((const struct gpio_mindgrove_config *)(dev)->config)
#define DEV_DATA(dev) ((struct gpio_mindgrove_data *)(dev)->data)
#define DEV_REGS(dev) ((volatile uint32_t *)DEV_CFG(dev)->base)

/* ------------------------------------------------------------------ */
/* ISR                                                                 */
/* ------------------------------------------------------------------ */

static void gpio_mindgrove_isr(const void *arg)
{
    const struct device *dev = (const struct device *)arg;
    const struct gpio_mindgrove_config *cfg = DEV_CFG(dev);
    struct gpio_mindgrove_data *data = DEV_DATA(dev);
    volatile uint32_t *base = DEV_REGS(dev);

    uint32_t data_val = base[OFF_DATA / 4];
    uint32_t pol_val  = base[OFF_INTR_POL / 4];

uint32_t asserting_pins = (data_val ^ pol_val) & data->armed_pins;


    if (asserting_pins == 0U) {
        return;
    }

    uint32_t pins = asserting_pins;

    while (pins) {

        uint32_t pin = find_lsb_set(pins) - 1U;

        /* Disable this GPIO interrupt source in PLIC */
        //irq_disable(cfg->irq_nums[pin]);

        /* Software mask */
        data->armed_pins &= ~BIT(pin);

        pins &= ~BIT(pin);
    }

    /* Clear hardware pending bits */
    base[OFF_INTR_STATUS / 4] = asserting_pins;

    gpio_fire_callbacks(&data->cb, dev, asserting_pins);
}

// static void gpio_mindgrove_isr(const void *arg)
// {
//     const struct device *dev = (const struct device *)arg;
//     const struct gpio_mindgrove_config *cfg = DEV_CFG(dev);
//     struct gpio_mindgrove_data *data = DEV_DATA(dev);
//     volatile uint32_t *base = DEV_REGS(dev);

//     uint32_t data_val = base[OFF_DATA / 4];
//     uint32_t pol_val  = base[OFF_INTR_POL / 4];

//     uint32_t asserting_pins = 0;

//     for (int pin = 0; pin < GPIO_NUM_PINS; pin++) {
//         if (!(data->armed_pins & BIT(pin))) {
//             continue;
//         }
//         bool pin_high  = (data_val & BIT(pin)) != 0;
//         bool trig_high = (pol_val  & BIT(pin)) != 0;

//         if (pin_high != trig_high) {
//             asserting_pins |= BIT(pin);
//         }
//     }

//     if (asserting_pins == 0U) {
//         return;
//     }

//     uint32_t pins = asserting_pins;
//     while (pins) {
//         uint32_t pin = find_lsb_set(pins) - 1U;
//         irq_disable(cfg->irq_nums[pin]);
//         data->armed_pins &= ~BIT(pin);
//         irq_enable(cfg->irq_nums[pin]);
//         pins &= ~BIT(pin);
//     }
//     base[OFF_INTR_STATUS / 4] = asserting_pins;
//     gpio_fire_callbacks(&data->cb, dev, asserting_pins);
// }

/* ------------------------------------------------------------------ */
/* GPIO API                                                            */
/* ------------------------------------------------------------------ */

static int gpio_mindgrove_pin_configure(const struct device *dev,
                                        gpio_pin_t pin,
                                        gpio_flags_t flags)
{
    volatile uint32_t *base = DEV_REGS(dev);

    if (pin >= GPIO_NUM_PINS) {
        return -EINVAL;
    }

    if (flags & GPIO_OUTPUT) {
        base[OFF_DIRECTION / 4] |= BIT(pin);
    } else {
        base[OFF_DIRECTION / 4] &= ~BIT(pin);
    }

    if (flags & GPIO_OUTPUT_INIT_HIGH) {
        base[OFF_SET / 4] = BIT(pin);
    } else if (flags & GPIO_OUTPUT_INIT_LOW) {
        base[OFF_CLEAR / 4] = BIT(pin);
    }

    return 0;
}

static int gpio_mindgrove_port_get_raw(const struct device *dev,
                                       gpio_port_value_t *value)
{
    *value = DEV_REGS(dev)[OFF_DATA / 4];
    return 0;
}

static int gpio_mindgrove_port_set_bits_raw(const struct device *dev,
                                            gpio_port_pins_t pins)
{
    DEV_REGS(dev)[OFF_SET / 4] = pins;
    return 0;
}

static int gpio_mindgrove_port_clear_bits_raw(const struct device *dev,
                                              gpio_port_pins_t pins)
{
    DEV_REGS(dev)[OFF_CLEAR / 4] = pins;
    return 0;
}

static int gpio_mindgrove_port_toggle_bits(const struct device *dev,
                                           gpio_port_pins_t pins)
{
    DEV_REGS(dev)[OFF_TOGGLE / 4] = pins;
    return 0;
}

                                                  enum gpio_int_mode mode,
                                                  enum gpio_int_trig trig)
{
    const struct gpio_mindgrove_config *cfg = DEV_CFG(dev);
    struct gpio_mindgrove_data *data = DEV_DATA(dev);
    volatile uint32_t *base = DEV_REGS(dev);

    if (pin >= GPIO_NUM_PINS) {
        return -EINVAL;
    }

    if (mode == GPIO_INT_MODE_DISABLED) {
        irq_disable(cfg->irq_nums[pin]);
        data->armed_pins &= ~BIT(pin);
        return 0;
    }

    if (mode == GPIO_INT_MODE_EDGE) {
        return -ENOTSUP;
    }

    /* 
 * Zephyr passes combined flags for convenience macros like:
 *
 * GPIO_INT_LEVEL_HIGH
 * GPIO_INT_LEVEL_LOW
 *
 * so we must test trigger bits using bitmasks, not equality.
 *
 * Hardware polarity:
 *   bit=1 -> LOW trigger
 *   bit=0 -> HIGH trigger
 */
if ((trig & GPIO_INT_TRIG_LOW) != 0U) {

    /* LOW trigger */
    base[OFF_INTR_POL / 4] |= BIT(pin);

} else if ((trig & GPIO_INT_TRIG_HIGH) != 0U) {

    /* HIGH trigger */
    base[OFF_INTR_POL / 4] &= ~BIT(pin);

} else {
    return -ENOTSUP;
}

    /*
     * Do NOT clear INTR_STATUS here.
     * If the pin is already at the trigger level, the hardware has
     * already asserted the status bit. Clearing it would cause the
     * interrupt to be missed. Status is only cleared inside the ISR.
     */
    irq_enable(cfg->irq_nums[pin]);
    data->armed_pins |= BIT(pin);

    return 0;
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
    volatile uint32_t *base = DEV_REGS(dev);

    if (cfg->base == 0U) {
        LOG_ERR("Base address is zero");
        return -ENODEV;
    }

    sys_slist_init(&DEV_DATA(dev)->cb);
    DEV_DATA(dev)->armed_pins = 0;

    /* Mirror current pin state into POL so no pin starts in triggered
     * condition. PLIC line is driven directly by (DATA != POL) with no
     * INTR_STATUS gating — any mismatch causes a storm at boot. */
    // base[OFF_INTR_POL / 4] = base[OFF_DATA / 4];

    cfg->irq_config_func(dev);
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

/* ------------------------------------------------------------------ */
/* Per-instance macros                                                 */
/* ------------------------------------------------------------------ */

#define GPIO_IRQ_NUM_ENTRY(i, n)  DT_INST_IRQN_BY_IDX(n, i),

#define GPIO_IRQ_NUMS_DECLARE(n)                                            \
    static const uint32_t gpio_irq_nums_##n[] = {                          \
        LISTIFY(DT_INST_NUM_IRQS(n), GPIO_IRQ_NUM_ENTRY, (), n)             \
    }

#define GPIO_IRQ_CONNECT_ONE(i, n)                                          \
    IRQ_CONNECT(DT_INST_IRQN_BY_IDX(n, i),                                 \
                DT_INST_IRQ_BY_IDX(n, i, priority),                        \
                gpio_mindgrove_isr,                                         \
                DEVICE_DT_INST_GET(n),                                      \
                0)

/*
 * GPIO_SET_PLIC_PRIORITY_ONE: set PLIC priority for pin i's IRQ source.
 *
 * riscv_plic_set_priority() takes the multilevel-encoded IRQ number and
 * sets the PLIC hardware priority register for that source.
 *
 * This MUST be called after IRQ_CONNECT and before irq_enable, because
 * plic_init() zeros all priorities at boot. Without this, the source
 * priority stays at 0 and the PLIC never delivers the interrupt even
 * when the source is enabled.
 *
 * GPIO_PLIC_PRIORITY must be > 0 and > the PLIC threshold (default 0).
 */
#define GPIO_SET_PLIC_PRIORITY_ONE(i, n)                                    \
    riscv_plic_set_priority(DT_INST_IRQN_BY_IDX(n, i), GPIO_PLIC_PRIORITY)

#define GPIO_IRQ_CONFIG_FUNC(n)                                             \
    static void gpio_mindgrove_irq_config_##n(const struct device *dev)     \
    {                                                                       \
        ARG_UNUSED(dev);                                                    \
        LISTIFY(DT_INST_NUM_IRQS(n), GPIO_IRQ_CONNECT_ONE, (;), n);        \
        LISTIFY(DT_INST_NUM_IRQS(n), GPIO_SET_PLIC_PRIORITY_ONE, (;), n);  \
    }

#define GPIO_INIT(n)                                                        \
    GPIO_IRQ_CONFIG_FUNC(n);                                                \
    GPIO_IRQ_NUMS_DECLARE(n);                                               \
                                                                            \
    static const struct gpio_mindgrove_config gpio_mindgrove_config_##n = { \
        .common = {                                                         \
            .port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(n),            \
        },                                                                  \
        .base            = DT_INST_REG_ADDR(n),                             \
        .irq_nums        = gpio_irq_nums_##n,                               \
        .irq_config_func = gpio_mindgrove_irq_config_##n,                   \
    };                                                                      \
                                                                            \
    static struct gpio_mindgrove_data gpio_mindgrove_data_##n;              \
                                                                            \
    DEVICE_DT_INST_DEFINE(n,                                                \
                          gpio_mindgrove_init,                              \
                          NULL,                                             \
                          &gpio_mindgrove_data_##n,                         \
                          &gpio_mindgrove_config_##n,                       \
                          POST_KERNEL,                                      \
                          CONFIG_GPIO_INIT_PRIORITY,                        \
                          &gpio_mindgrove_api);

DT_INST_FOREACH_STATUS_OKAY(GPIO_INIT)