
#include <stdio.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/sys/printk.h>
#include <stdbool.h>

int main()
{
	// const struct device *wdt = DEVICE_DT_GET(DT_ALIAS(watchdog));
    int tr;
    printk("Hello \n");
    k_busy_wait(100);   
    const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(watchdog0));
    // tr = wdt_disable(dev);
    struct wdt_timeout_cfg cfg = {
    .window.min = 0,
    .window.max = 1000, // ms
    
    };

    wdt_install_timeout(dev,&cfg);
    wdt_setup(dev,3);
    return 0;
}