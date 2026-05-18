#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/entropy.h>
#include <zephyr/sys/printk.h>

#define SAMPLES_TO_COLLECT 10
#define BUFFER_SIZE 16

int main(void)
{
    /* 1. Get the device binding from the 'chosen' node */
    const struct device *const entropy_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_entropy));
    uint8_t entropy_buffer[BUFFER_SIZE];
    int ret;

    printk("\n\r--- Mindgrove TRNG Hardware Test ---\n\r");

    /* 2. Check if the driver initialized successfully */
    if (!device_is_ready(entropy_dev)) {
        printk("Error: TRNG device not ready. Check your init priority!\n\r");
        return -EIO;
    }

    printk("Device %s is ready. Starting data collection...\n\r", entropy_dev->name);

    for (int s = 1; s <= SAMPLES_TO_COLLECT; s++) {
        /* 3. Call the standard API */
        ret = entropy_get_entropy(entropy_dev, entropy_buffer, BUFFER_SIZE);

        if (ret < 0) {
            printk("Sample %d: Failed to get entropy (Error: %d)\n\r", s, ret);
        } else {
            printk("Sample %02d [%db]: ", s, BUFFER_SIZE);
            for (int i = 0; i < BUFFER_SIZE; i++) {
                printk("%02x ", entropy_buffer[i]);
            }
            printk("\n\r");
        }

    }

    return 0;
}
