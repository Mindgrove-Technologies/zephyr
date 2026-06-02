#ifndef SHA256_TEST_H
#define SHA256_TEST_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/crypto/crypto.h>
#include <zephyr/crypto/hash.h>
#include <zephyr/sys/printk.h>

#include <string.h>
#include <stdbool.h>

#include "rsp_files.h"

#define SHA256_HASH_LEN 32
#define SHA256_DIGEST_SIZE 32

void print_digest(const uint8_t *digest);

/* Single block KAT */
int test_sha256_single_block_kat(void);

/* Multi block tests */
int test_sha256_mmt_01(void);
int test_sha256_mmt_02(void);
int test_sha256_mmt_03(void);
int test_sha256_mmt_04(void);
int test_sha256_mmt_05(void);
int test_sha256_mmt_06(void);

/* Monte */
// int test_sha_monte_carlo(void);

#endif