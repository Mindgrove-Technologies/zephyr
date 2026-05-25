#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/sys/printk.h>

/* Fetch the watchdog device node from devicetree */
#define WDT_NODE DT_NODELABEL(watchdog0)

#if !DT_NODE_HAS_STATUS_OKAY(WDT_NODE)
#error "Watchdog node watchdog0 is disabled or not defined in devicetree!"
#endif

/* Custom configuration matching your driver's soft reset handler */
#define SOFT_RESET_MODE_FLAG 7

void main(void)
{
    const struct device *wdt_dev = DEVICE_DT_GET(WDT_NODE);
    struct wdt_timeout_cfg wdt_config;
    int wdt_channel_id;
    int ret;

    printk("\n--- Mindgrove WDTimer Sample Application Startup ---\n");

    if (!device_is_ready(wdt_dev)) {
        printk("Error: Watchdog device is not ready\n");
        return;
    }

    /* * =================================================================
     * SCENARIO 1: Testing Normal Watchdog Operation (Feeding)
     * =================================================================
     */
    printk("\n[Step 1] Configuring Watchdog for a 3-second window...\n");
    
    wdt_config.window.min = 0U;
    wdt_config.window.max = 3000U; /* 3000 milliseconds */
    wdt_config.callback = NULL;    /* No interrupt callback needed for pure reset */
    wdt_config.flags = WDT_FLAG_RESET_SOC; /* Starts as hardware reset */

    wdt_channel_id = wdt_install_timeout(wdt_dev, &wdt_config);
    if (wdt_channel_id < 0) {
        printk("Failed to install watchdog timeout: %d\n", wdt_channel_id);
        return;
    }

    ret = wdt_setup(wdt_dev, 0);
    if (ret < 0) {
        printk("Failed to setup watchdog: %d\n", ret);
        return;
    }

    printk("Watchdog armed. Feeding every 1 second for 5 seconds total...\n");
    for (int i = 1; i <= 5; i++) {
        k_msleep(1000);
        wdt_feed(wdt_dev, wdt_channel_id);
        printk("   Fed watchdog... (Pass %d/5)\n", i);
    }

    /* Disable the watchdog cleanly to reconfigure it */
    printk("Disabling watchdog to switch reset modes.\n");
    wdt_disable(wdt_dev);

    /* * =================================================================
     * SCENARIO 2: Demonstrating SOFT RESET vs HARD RESET
     * =================================================================
     * Toggle the comment flags below to test either scenario.
     * When triggered, the SoC will reset and execution will jump back to main.
     * =================================================================
     */
    
    #if 1 
        /* --- TEST SOFT RESET --- */
        printk("\n[Step 2] Triggering a SOFT RESET in 2 seconds...\n");
        wdt_config.window.max = 2000U;
        wdt_config.flags = SOFT_RESET_MODE_FLAG; /* Maps to MODE_SOFT_RESET */
    #else
        /* --- TEST HARD RESET --- */
        printk("\n[Step 2] Triggering a HARD RESET in 2 seconds...\n");
        wdt_config.window.max = 2000U;
        wdt_config.flags = WDT_FLAG_RESET_SOC;   /* Maps to MODE_RESET */
    #endif

    /* Re-install and setup with new configs */
    wdt_channel_id = wdt_install_timeout(wdt_dev, &wdt_config);
    wdt_setup(wdt_dev, 0);

    printk("Watchdog re-armed. Letting it expire now...\n");
    
    /* Sit back and wait for the reset block to cleanly fire */
    while (1) {
        printk("Waiting for system reset...\n");
        k_msleep(500);
    }
}