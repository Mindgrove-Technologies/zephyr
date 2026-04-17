#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/entropy.h>
#include <zephyr/sys/printk.h>

#define SAMPLES_TO_COLLECT 3
#define BUFFER_SIZE 16

int main(void)
{
    /* 1. Get the device binding from the 'chosen' node we set up earlier */
    const struct device *const entropy_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_entropy));
    uint8_t entropy_buffer[BUFFER_SIZE];
    int ret;

    printk("\n--- Mindgrove TRNG Hardware Test ---\n");

    /* 2. Check if the driver initialized successfully */
    if (!device_is_ready(entropy_dev)) {
        printk("Error: TRNG device not ready. Check your init priority!\n");
        return -EIO;
    }

    printk("Device %s is ready. Starting data collection...\n\n", entropy_dev->name);

    for (int s = 1; s <= SAMPLES_TO_COLLECT; s++) {
        /* 3. Call the standard API */
        /* This eventually calls your vtrng_generate function */
        ret = entropy_get_entropy(entropy_dev, entropy_buffer, BUFFER_SIZE);

        if (ret < 0) {
            printk("Sample %d: Failed to get entropy (Error: %d)\n", s, ret);
        } else {
            printk("Sample %02d [%db]: ", s, BUFFER_SIZE);
            for (int i = 0; i < BUFFER_SIZE; i++) {
                printk("%02x ", entropy_buffer[i]);
            }
            printk("\n");
        }

        /* Small delay to see it happen in real-time on console */
        k_msleep(500);
    }

    return 0;
}