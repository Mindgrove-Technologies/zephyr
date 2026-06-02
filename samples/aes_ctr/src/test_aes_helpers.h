#ifndef TEST_AES_HELPERS_H
#define TEST_AES_HELPERS_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/crypto/crypto.h>
#include <zephyr/crypto/cipher.h>
#include <zephyr/sys/printk.h>
#include <string.h>

static inline const struct device *aes_get_dev(void)
{
    const struct device *dev = DEVICE_DT_GET_ONE(mindgrove_aes);
    if (!device_is_ready(dev)) {
        printk("AES device not ready\n");
        return NULL;
    }
    return dev;
}

#endif /* TEST_AES_HELPERS_H */