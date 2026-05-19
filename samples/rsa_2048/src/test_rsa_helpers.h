#ifndef TEST_RSA_HELPERS_H
#define TEST_RSA_HELPERS_H

#include <stdint.h>
#include <zephyr/sys/printk.h>

static inline void rsa_print_hex(const char *label,
                                  const uint8_t *data, uint16_t len)
{
    printk("%s: ", label);
    for (uint16_t i = 0; i < len; i++)
        printk("%02x", data[i]);
    printk("\n");
}

/* Converts bit length to byte length, rounding up */
static inline uint16_t bits_to_bytes(uint16_t bits)
{
    return (bits + 7) / 8;
}

#endif /* TEST_RSA_HELPERS_H */