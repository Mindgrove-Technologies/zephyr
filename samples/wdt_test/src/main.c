/*
 * Simple Mindgrove Watchdog Test Application
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/sys/printk.h>

#define WDT_NODE DT_NODELABEL(watchdog0)

/*
 * Vendor-specific flag for soft reset.
 * Bit 3 is free — does not conflict with Zephyr's WDT_FLAG_RESET_SOC (bit 0)
 * or WDT_FLAG_RESET_CPU_CORE (bit 1).
 * Must match the definition in the driver.
 */
#define WDT_FLAG_MINDGROVE_SOFT_RESET  BIT(3)

/*
 * Select mode:
 *
 * 1 -> HARD RESET after 5 seconds
 * 0 -> IMMEDIATE SOFT RESET
 */
#define TEST_HARD_RESET  0  /* Set to 0 for Soft Reset, 1 for Hard Reset */

int main(void)
{
    const struct device *wdt;
    struct wdt_timeout_cfg cfg;
    int channel_id;
    int ret;

    wdt = DEVICE_DT_GET(WDT_NODE);
    if (!device_is_ready(wdt)) {
        printk("Watchdog device not ready\n");
        return 0;
    }

    printk("Mindgrove WDT test\n");

    cfg.callback   = NULL;
    cfg.window.min = 0;

#if TEST_HARD_RESET
    /*
     * HARD RESET after 5 seconds.
     * WDT_CYCLES and WDT_ACTIVE are loaded by the driver;
     * system resets when the counter expires.
     */
    cfg.window.max = 5000;              /* 5000 ms */
    cfg.flags      = WDT_FLAG_RESET_SOC;
    printk("Configuring HARD RESET mode (5 second timeout)\n");
#else
    /*
     * SOFT RESET — fires immediately on wdt_setup().
     * The hardware only needs CTRL written; WDT_CYCLES and
     * WDT_ACTIVE are not used in this path (matches baremetal driver).
     * window.max is irrelevant but set to 0 to make that explicit.
     */
    cfg.window.max = 1;
    cfg.flags      = WDT_FLAG_MINDGROVE_SOFT_RESET;
    printk("Configuring SOFT RESET mode (immediate)\n");
#endif

    channel_id = wdt_install_timeout(wdt, &cfg);
    if (channel_id < 0) {
        printk("wdt_install_timeout failed: %d\n", channel_id);
        return 0;
    }

    ret = wdt_setup(wdt, 0);
    if (ret < 0) {
        printk("wdt_setup failed: %d\n", ret);
        return 0;
    }

#if TEST_HARD_RESET
    printk("System will HARD RESET in 5 seconds — not feeding watchdog\n");
    while (1) {
        printk("Waiting for watchdog reset...\n");
        k_sleep(K_SECONDS(1));
    }
#else
    /*
     * Soft reset fires synchronously inside wdt_setup() by writing
     * the CTRL register. Execution should never reach here.
     * The nop loop is a fallback to confirm if reset did NOT occur —
     * if you see "SOFT RESET did not trigger", the CTRL write failed.
     */
    printk("SOFT RESET should have triggered inside wdt_setup()\n");
    printk("If you see this line, soft reset did NOT occur — check driver\n");
    while (1) {
        __asm__ volatile("nop");
    }
#endif

    return 0;
}