

#include <zephyr/kernel.h>
#include <zephyr/crypto/crypto.h>
#include <zephyr/crypto/hash.h>
#include <zephyr/sys/printk.h>
#include <string.h>
#include "sha256_kat_testvectors.h"
#include "test_helpers.h"

int test_sha256_single_block_kat(void)
{
    extern const sha256_kat test_vectors[];
    extern const uint16_t kat_vectors_count;

    const struct device *dev =
        DEVICE_DT_GET_ONE(mindgrove_sha256);

    if (!device_is_ready(dev)) {
        printk("SHA256 device not ready\n");
        return -ENODEV;
    }

    for (uint16_t i = 0; i < kat_vectors_count; i++) {

        const sha256_kat *tv = &test_vectors[i];

        uint8_t output[SHA256_HASH_LEN] = {0};

        struct hash_ctx ctx;
        struct hash_pkt pkt;

        if (hash_begin_session(dev,
                               &ctx,
                               CRYPTO_HASH_ALGO_SHA256)) {

            printk("hash_begin_session failed\n");
            return -EIO;
        }

        uint32_t total_len_bits =
            tv->input_text_len;

        uint32_t total_len_bytes =
            total_len_bits / 8;

        if (total_len_bits == 512) {

            pkt.in_buf =
                (uint8_t *)tv->input_text;

            pkt.in_len = 64;
            pkt.out_buf = output;

            if (hash_compute(&ctx, &pkt)) {

                hash_free_session(dev, &ctx);
                return -EIO;
            }

        } else {

            uint32_t sent_bytes = 0;

            while ((total_len_bytes - sent_bytes) > 64) {

                pkt.in_buf =
                    (uint8_t *)&tv->input_text[sent_bytes];

                pkt.in_len = 64;
                pkt.out_buf = NULL;

                if (hash_update(&ctx, &pkt)) {

                    hash_free_session(dev, &ctx);
                    return -EIO;
                }

                sent_bytes += 64;
            }

            pkt.in_buf =
                (uint8_t *)&tv->input_text[sent_bytes];

            pkt.in_len =
                total_len_bytes - sent_bytes;

            pkt.out_buf = output;

            if (hash_compute(&ctx, &pkt)) {

                hash_free_session(dev, &ctx);
                return -EIO;
            }
        }

        hash_free_session(dev, &ctx);

        if (memcmp(output,
                   tv->output_hash,
                   SHA256_HASH_LEN) == 0) {

            printk("KAT #%d PASS\n", i + 1);

        } else {

            printk("KAT #%d FAIL\n", i + 1);
            return -EFAULT;
        }
    }

    return 0;
}