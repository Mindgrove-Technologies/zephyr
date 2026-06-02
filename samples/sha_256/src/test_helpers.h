#ifndef TEST_HELPERS_H
#define TEST_HELPERS_H

#include <stdint.h>
#include <stddef.h>

#define SHA256_HASH_LEN     32
#define SHA256_DIGEST_SIZE  32

static inline void print_hex(const char *label, const uint8_t *data, size_t len)
{
    printk("%s: ", label);
    for (size_t i = 0; i < len; i++) {
        printk("%02x", data[i]);
        if ((i + 1) % 32 == 0 && i + 1 < len) printk("\n          ");
    }
    printk("\n");
}

static inline void print_digest(const uint8_t *digest)
{
    for (int i = 0; i < SHA256_DIGEST_SIZE; i++) {
        printk("%02x", digest[i]);
    }
}

#endif /* TEST_HELPERS_H */