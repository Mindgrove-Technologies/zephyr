/*
 * GPIO Interrupt Test — Mindgrove Secure IoT SoC
 *
 * Tests GPIO pins 0 and 1 simultaneously.
 * Each pin has its own work item so re-arm is independent.
 *
 * Re-arm strategy:
 *   Re-arm immediately from the work queue — do NOT wait for pin to go
 *   HIGH first. If the pin is still LOW when re-armed, the ISR fires
 *   again (correct level-triggered behavior). The REARM_MS delay between
 *   re-arms prevents flooding while a pin is held LOW.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(gpio_test, LOG_LEVEL_INF);

#define GPIO_NODE       DT_NODELABEL(gpio0)
#define TEST_PIN_COUNT  2        /* test pin 0 and pin 1 */
#define REARM_MS        50       /* delay between re-arms while held LOW */

static const struct device *gpio_dev;

/* Per-pin state */
struct pin_state {
    struct gpio_callback        cb;
    struct k_work_delayable     rearm_work;
    volatile uint32_t           count;
    uint8_t                     pin;
};

static struct pin_state pins[TEST_PIN_COUNT];

/* ------------------------------------------------------------------ */
/* Work handler — re-arms the interrupt for its specific pin          */
/* ------------------------------------------------------------------ */

static void rearm_work_handler(struct k_work *work)
{
    struct pin_state *ps = CONTAINER_OF(work, struct pin_state,
                                        rearm_work.work);

    /*
     * Re-arm immediately. If pin is still LOW the ISR fires again —
     * that is correct for level-triggered hardware. The REARM_MS delay
     * above prevents a tight loop.
     */
    int ret = gpio_pin_interrupt_configure(gpio_dev, ps->pin,
                                           GPIO_INT_LEVEL_LOW);
    if (ret < 0) {
        printk("[REARM] pin=%u ERROR: %d\n", ps->pin, ret);
    } else {
        int val = gpio_pin_get_raw(gpio_dev, ps->pin);
        printk("[REARM] pin=%u re-armed (val=%d)\n", ps->pin, val);
    }
}

/* ------------------------------------------------------------------ */
/* Callback — one per pin, identified by the pins bitmask             */
/* ------------------------------------------------------------------ */

static void gpio_cb_handler(const struct device *dev,
                             struct gpio_callback *cb,
                             uint32_t fired_pins)
{
    struct pin_state *ps = CONTAINER_OF(cb, struct pin_state, cb);

    ps->count++;
    int val = gpio_pin_get_raw(dev, ps->pin);

    printk("[CB] pin=%u count=%u val=%d\n", ps->pin, ps->count, val);

    /*
     * Schedule re-arm after REARM_MS. Must NOT call
     * gpio_pin_interrupt_configure() here — the PLIC has not yet
     * completed this interrupt's claim/complete cycle. Calling
     * irq_enable() now causes immediate re-delivery → stack overflow.
     */
    k_work_reschedule(&ps->rearm_work, K_MSEC(REARM_MS));
}

/* ------------------------------------------------------------------ */
/* main                                                                */
/* ------------------------------------------------------------------ */

int main(void)
{
    printk("\n========================================\n");
    printk("  GPIO Interrupt Test — Mindgrove SoC\n");
    printk("  Testing pins 0 and 1\n");
    printk("  Connect any pin to GND to trigger\n");
    printk("========================================\n\n");

    gpio_dev = DEVICE_DT_GET(GPIO_NODE);
    if (!device_is_ready(gpio_dev)) {
        printk("[MAIN] ERROR: GPIO device not ready\n");
        return -1;
    }
    printk("[MAIN] GPIO device: %s\n\n", gpio_dev->name);

    for (int i = 0; i < TEST_PIN_COUNT; i++) {
        uint8_t pin = (uint8_t)i;
        pins[i].pin = pin;

        /* Configure as input with pull-up */
        int ret = gpio_pin_configure(gpio_dev, pin,
                                     GPIO_INPUT | GPIO_PULL_UP);
        if (ret < 0) {
            printk("[MAIN] ERROR: pin %u configure failed: %d\n", pin, ret);
            return -1;
        }

        int val = gpio_pin_get_raw(gpio_dev, pin);
        printk("[MAIN] Pin %u: initial value = %d (%s)\n",
               pin, val, val ? "HIGH" : "LOW");

        /* Init per-pin work item */
        k_work_init_delayable(&pins[i].rearm_work, rearm_work_handler);

        /* Register per-pin callback for exactly this pin's bit */
        gpio_init_callback(&pins[i].cb, gpio_cb_handler, BIT(pin));
        gpio_add_callback(gpio_dev, &pins[i].cb);

        /* Arm interrupt */
        ret = gpio_pin_interrupt_configure(gpio_dev, pin,
                                           GPIO_INT_LEVEL_LOW);
        if (ret < 0) {
            printk("[MAIN] ERROR: pin %u interrupt configure failed: %d\n",
                   pin, ret);
            return -1;
        }

        printk("[MAIN] Pin %u armed for LEVEL_LOW\n", pin);
    }

    printk("\n[MAIN] Ready. Connect pin 0 or pin 1 to GND...\n\n");

    uint32_t last_total = 0;

    while (1) {
        uint32_t total = pins[0].count + pins[1].count;

        if (total != last_total) {
            printk("[MAIN] counts: pin0=%u  pin1=%u\n\n",
                   pins[0].count, pins[1].count);
            last_total = total;
        }

        k_sleep(K_MSEC(100));
    }

    return 0;
}