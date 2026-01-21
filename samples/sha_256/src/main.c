/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/crypto/crypto.h>
#include <zephyr/sys/printk.h>
#include <string.h>

/* Test input */
static const uint8_t sha_input[] =
    "The quick brown fox jumps over the lazy dog";

/* Expected SHA-256 hash */
static const uint8_t expected_sha256[32] = {
    0xd7, 0xa8, 0xfb, 0xb3, 0x07, 0xd7, 0x60, 0x82,
    0x16, 0x3d, 0x29, 0x4f, 0x1f, 0x0f, 0x6f, 0x00,
    0x6b, 0x7a, 0xa1, 0x38, 0x37, 0x3b, 0x8a, 0x8b,
    0x6f, 0xd1, 0x4e, 0x0c, 0x3f, 0x9f, 0x00, 0x0f
};

static void print_hex(const uint8_t *buf, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        printk("%02x ", buf[i]);
    }
    printk("\n");
}

void main(void)
{
    printk("SHA256 sanity test start\n");

    /* Get MindGrove SHA device (same style as AES) */
    const struct device *dev = DEVICE_DT_GET_ONE(mindgrove_sha256);
    if (!device_is_ready(dev)) {
        printk("SHA device not ready\n");
        return;
    }

    printk("SHA device ready\n");

    struct hash_ctx ctx;
    struct hash_pkt pkt;
    uint8_t hash_out[32];
    int ret;

    memset(&ctx, 0, sizeof(ctx));
    memset(&pkt, 0, sizeof(pkt));
    memset(hash_out, 0, sizeof(hash_out));

    /* Begin SHA session */
    ret = hash_begin_session(dev, &ctx, CRYPTO_HASH_ALGO_SHA256);
    if (ret) {
        printk("hash_begin_session failed: %d\n", ret);
        return;
    }

    /* Prepare packet */
    pkt.in_buf  = (uint8_t *)sha_input;
    pkt.in_len  = strlen((const char *)sha_input);
    pkt.out_buf = hash_out;

    printk("SHA input (%d bytes):\n", pkt.in_len);
    print_hex(pkt.in_buf, pkt.in_len);

    /* Compute hash */
    ret = hash_compute(&ctx, &pkt);
    if (ret) {
        printk("hash_compute failed: %d\n", ret);
        hash_free_session(dev, &ctx);
        return;
    }

    hash_free_session(dev, &ctx);

    printk("SHA256 output:\n");
    print_hex(hash_out, sizeof(hash_out));

    if (memcmp(hash_out, expected_sha256, sizeof(expected_sha256)) == 0) {
        printk("SHA256 PASS\n");
    } else {
        printk("SHA256 FAIL\n");
    }
}
