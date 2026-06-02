#include <zephyr/kernel.h>
#include <zephyr/crypto/crypto.h>
#include <zephyr/crypto/hash.h>
#include <zephyr/sys/printk.h>
#include <string.h>

/* Each file includes ONLY its own header — no redefinition clash */
#include "sha256_mmt_test_vectors_04.h"   /* defines test_vectors, mmt_04_kat_vectors_count,
                                            sha256_multi_kat_04 */

#define SHA256_HASH_LEN 32

int test_sha256_mmt_04(void)
{
    const struct device *dev = DEVICE_DT_GET_ONE(mindgrove_sha256);
    if (!device_is_ready(dev)) {
        printk("SHA256 device not ready\n");
        return -ENODEV;
    }

    int total_pass = 0;
    int count = (int)mmt_04_kat_vectors_count;

    for (int v_idx = 0; v_idx < count; v_idx++) {
        const sha256_multi_kat_04 *tv = &test_vectors[v_idx];
        uint8_t output[SHA256_HASH_LEN] = {0};
        struct hash_ctx ctx;
        struct hash_pkt pkt;

        printk("\n=== MMT04 Vector %d (%u bits) ===\n",
               v_idx, tv->input_text_len);

        if (hash_begin_session(dev, &ctx, CRYPTO_HASH_ALGO_SHA256)) {
            printk("Session begin failed\n");
            return -EIO;
        }

        uint32_t total_bits = tv->input_text_len;
        uint32_t sent_bits  = 0;

        while ((total_bits - sent_bits) > 512) {
            uint32_t byte_offset = sent_bits / 8;
            pkt.in_buf  = (uint8_t *)&tv->input_text[byte_offset];
            pkt.in_len  = 64;
            pkt.out_buf = NULL;
            if (hash_update(&ctx, &pkt)) {
                hash_free_session(dev, &ctx);
                return -EIO;
            }
            sent_bits += 512;
        }

        uint32_t remaining_bytes = (total_bits - sent_bits + 7) / 8;
        uint32_t byte_offset     = sent_bits / 8;
        pkt.in_buf  = remaining_bytes ? (uint8_t *)&tv->input_text[byte_offset] : NULL;
        pkt.in_len  = remaining_bytes;
        pkt.out_buf = output;

        if (hash_compute(&ctx, &pkt)) {
            hash_free_session(dev, &ctx);
            return -EIO;
        }
        hash_free_session(dev, &ctx);

        if (memcmp(output, tv->output_hash, SHA256_HASH_LEN) == 0) {
            printk("MMT04 Vector %d: PASS\n", v_idx);
            total_pass++;
        } else {
            printk("MMT04 Vector %d: FAIL\nExpected: ", v_idx);
            for (int i = 0; i < 32; i++) printk("%02x", tv->output_hash[i]);
            printk("\nActual:   ");
            for (int i = 0; i < 32; i++) printk("%02x", output[i]);
            printk("\n");
            return -EFAULT;
        }
    }

    printk("\nMMT04: %d/%d vectors passed\n", total_pass, count);
    return 0;
}