#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/interrupt_controller/riscv_plic.h>

LOG_MODULE_REGISTER(gpio_test, LOG_LEVEL_INF);

#define GPIO_NODE  DT_NODELABEL(gpio0)
#define TEST_PIN   1

/* PLIC / GPIO register addresses */
#define PLIC_PEND  0x0C001000U
#define PLIC_EN    0x0C002000U
#define PLIC_CLAIM 0x0C200004U
#define GPIO_INTR  0x40230U
#define GPIO_DATA  0x40208U

static struct gpio_callback gpio_cb;
static volatile uint32_t isr_count;

static void gpio_interrupt_cb(const struct device *dev,
                              struct gpio_callback *cb, uint32_t pins)
{
    isr_count++;

    /* Complete PLIC claim first */
    uint32_t claim = sys_read32(PLIC_CLAIM);
    if (claim != 0) {
        sys_write32(claim, PLIC_CLAIM);
        __asm__ volatile("fence iorw, iorw" ::: "memory");
    }

    /* Re-enable PLIC source if needed */
    riscv_plic_irq_enable(43);
}

int main(void)
{
    const struct device *gpio_dev = DEVICE_DT_GET(GPIO_NODE);
    uint32_t last_count = 0;

    printk("=== Mindgrove GPIO interrupt auto-rearm test ===\n");
    printk("TEST_PIN=%d\n", TEST_PIN);

    if (!device_is_ready(gpio_dev)) {
        printk("ERROR: GPIO device not ready\n");
        return 0;
    }

    printk("[MAIN] GPIO device ready\n");

    /* Configure pin as input with pull-up */
    gpio_pin_configure(gpio_dev, TEST_PIN, GPIO_INPUT | GPIO_PULL_UP);

    /* Register ISR */
    gpio_init_callback(&gpio_cb, gpio_interrupt_cb, BIT(TEST_PIN));
    gpio_add_callback(gpio_dev, &gpio_cb);

    /* Configure GPIO interrupt (level-low) */
   gpio_pin_interrupt_configure(gpio_dev, TEST_PIN,
                             GPIO_INT_LEVEL_LOW);

    /* Enable PLIC source */
    riscv_plic_irq_enable(43);

    printk("[MAIN] interrupt configured. Monitoring pin...\n");

    /* --- Polling loop: only print on new ISR events --- */
    while (1) {
        if (isr_count != last_count) {
            printk("[EVENT] New ISR triggered! ISR_count=%u, pin_val=%d\n",
                   isr_count, gpio_pin_get_raw(gpio_dev, TEST_PIN));
            last_count = isr_count;
        }

        k_sleep(K_MSEC(100));  /* check every 100ms */
    }

    return 0;
}