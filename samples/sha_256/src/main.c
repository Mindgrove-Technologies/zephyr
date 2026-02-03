#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/crypto/crypto.h>
#include <zephyr/crypto/hash.h>
#include <zephyr/sys/printk.h>
#include <string.h>
#include "rsp_files.h"

#define SHA256_HASH_LEN 32

static void print_hex(const uint8_t *buf, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        printk("%02x ", buf[i]);
    }
    printk("\n");
}

static bool check_hash(const uint8_t *got, const uint8_t *expected, size_t len)
{
    return memcmp(got, expected, len) == 0;
}

/* ============================= */
/* Single block test             */
/* ============================= */
static int test_single_block(void)
{
    const struct device *dev = DEVICE_DT_GET_ONE(mindgrove_sha256);
    if (!device_is_ready(dev)) {
        printk("SHA256 device not ready\n");
        return 1;
    }

    uint8_t output[SHA256_HASH_LEN];
    const char *input = "SDK-Unit-Testing";
    struct hash_ctx ctx;
    struct hash_pkt pkt;

    uint8_t expected[SHA256_HASH_LEN] = {
        0xd0, 0xc6, 0x2a, 0x1a, 0x24, 0x0c, 0xa6, 0xfe,
        0xcc, 0x2e, 0x36, 0xd5, 0x6f, 0xd7, 0x4a, 0x65,
        0xea, 0x70, 0xae, 0x00, 0x0a, 0x78, 0x90, 0x51,
        0x24, 0x14, 0xd4, 0x67, 0x91, 0x4e, 0xb7, 0xbd
    };

    hash_begin_session(dev, &ctx, CRYPTO_HASH_ALGO_SHA256);

    pkt.in_buf = (uint8_t *)input;
    pkt.in_len = strlen(input);
    pkt.out_buf = output;

    if (hash_compute(&ctx, &pkt)) {
        printk("hash_compute failed\n");
        hash_free_session(dev, &ctx);
        return 1;
    }

    hash_free_session(dev, &ctx);

    printk("\n[SHA256] Single block test\n");
    printk("Output   : "); print_hex(output, SHA256_HASH_LEN);
    printk("Expected : "); print_hex(expected, SHA256_HASH_LEN);
    printk(check_hash(output, expected, SHA256_HASH_LEN) ? "PASS\n" : "FAIL\n");
    return 0;
}

/* ============================= */
/* Multi-shot (streaming) test   */
/* ============================= */

static int test_multi_shot(void)
{
    const struct device *dev = DEVICE_DT_GET_ONE(mindgrove_sha256);
    if (!device_is_ready(dev)) {
        printk("SHA256 device not ready\n");
        return 1;
    }

    uint8_t output[SHA256_HASH_LEN];
    const char *input = "The quick brown fox jumps over the lazy dog";

    struct hash_ctx ctx;
    struct hash_pkt pkt;
    long iterated = 0;
    long total_len = strlen(input);
    const long chunk_size = 32; /* arbitrary chunk size for streaming */

    uint8_t expected[SHA256_HASH_LEN] = {
        0xd7, 0xa8, 0xfb, 0xb3, 0x07, 0xd7, 0x60, 0x82,
        0x16, 0x3d, 0x29, 0x4f, 0x1f, 0x0f, 0x6f, 0x00,
        0x6b, 0x7a, 0xa1, 0x38, 0x37, 0x3b, 0x8a, 0x8b,
        0x6f, 0xd1, 0x4e, 0x0c, 0x3f, 0x9f, 0x00, 0x0f
    };

    /* Begin hashing session */
    if (hash_begin_session(dev, &ctx, CRYPTO_HASH_ALGO_SHA256)) {
        printk("hash_begin_session failed\n");
        return 1;
    }

    /* Feed input in chunks, leaving the last chunk for finalization */
    while ((total_len - iterated) > chunk_size) {
        pkt.in_buf = (uint8_t *)(input + iterated);
        pkt.in_len = chunk_size;
        pkt.out_buf = NULL;

        if (hash_update(&ctx, &pkt)) {
            printk("hash_update failed\n");
            hash_free_session(dev, &ctx);
            return 1;
        }

        iterated += chunk_size;
    }

    /* Final block for hash_compute */
    pkt.in_buf = (uint8_t *)(input + iterated);
    pkt.in_len = total_len - iterated; /* remaining bytes */
    pkt.out_buf = output;

    if (hash_compute(&ctx, &pkt)) {
        printk("hash_compute (finalize) failed\n");
        hash_free_session(dev, &ctx);
        return 1;
    }

    hash_free_session(dev, &ctx);

    printk("\n[SHA256] Multi-shot test\n");
    printk("Output   : "); print_hex(output, SHA256_HASH_LEN);
    printk("Expected : "); print_hex(expected, SHA256_HASH_LEN);
    printk(check_hash(output, expected, SHA256_HASH_LEN) ? "PASS\n" : "FAIL\n");

    return 0;
}

#define SHA256_HASH_LEN 32

/* Single-block KAT test for Zephyr */
static int test_sha256_single_block_kat(void)
{
    extern const sha256_kat_t kat_vectors[];
    extern const uint16_t kat_vectors_count;

    const struct device *dev = DEVICE_DT_GET_ONE(mindgrove_sha256);
    if (!device_is_ready(dev)) {
        printk("SHA256 device not ready\n");
        return -ENODEV;
    }

    for (uint16_t i = 0; i < kat_vectors_count; i++) {
        const sha256_kat_t *tv = &kat_vectors[i];
        uint8_t output[SHA256_HASH_LEN] = {0};

        struct hash_ctx ctx;
        struct hash_pkt pkt;
        long int iterated_bits = 0;

        /* Begin session */
        if (hash_begin_session(dev, &ctx, CRYPTO_HASH_ALGO_SHA256)) {
            printk("hash_begin_session failed\n");
            return -EIO;
        }

        long int remaining_bits = tv->input_text_len;

        if (remaining_bits == 512) {
            // Special case: exactly one full block
            pkt.in_buf = (uint8_t *)tv->input_text;
            pkt.in_len = 64;  // 512 bits
            pkt.out_buf = NULL;
            if (hash_update(&ctx, &pkt)) {
                printk("hash_update failed\n");
                hash_free_session(dev, &ctx);
                return -EIO;
            }

            // Finalize with mode 1
            uint8_t dummy = 0;
            pkt.in_buf = &dummy;
            pkt.in_len = 0;
            pkt.out_buf = output;
            if (hash_compute(&ctx, &pkt)) {
                printk("hash_compute failed\n");
                hash_free_session(dev, &ctx);
                return -EIO;
            }
        } else {
            // Use original logic for < 512 bits or > 512 bits
            while ((remaining_bits - iterated_bits) > 512) {
                pkt.in_buf = (uint8_t *)(tv->input_text + iterated_bits / 8);
                pkt.in_len = 64;
                pkt.out_buf = NULL;
                if (hash_update(&ctx, &pkt)) {
                    printk("hash_update failed\n");
                    hash_free_session(dev, &ctx);
                    return -EIO;
                }
                iterated_bits += 512;
            }

            long int last_bits = remaining_bits - iterated_bits;
            pkt.in_buf = (uint8_t *)(tv->input_text + iterated_bits / 8);
            pkt.in_len = (last_bits + 7) / 8;
            pkt.out_buf = output;
            if (hash_compute(&ctx, &pkt)) {
                printk("hash_compute failed\n");
                hash_free_session(dev, &ctx);
                return -EIO;
            }
        }

        hash_free_session(dev, &ctx);

        /* Validate output */
        bool pass = true;
        for (uint8_t j = 0; j < SHA256_HASH_LEN; j++) {
            if (output[j] != tv->output_hash[j]) {
                printk("Single-block KAT #%d FAILED at byte %d, Expected: %02x Got: %02x\n",
                       i + 1, j, tv->output_hash[j], output[j]);
                pass = false;
                break;
            }
        }

        if (pass) {
            printk("Single-block KAT #%d PASS\n", i + 1);
        } else {
            return -EFAULT;
        }
    }

    return 0;
}



/* Multi-block KAT test for Zephyr */
static int test_sha256_multi_block_kat(void)
{
    extern const sha256_multi_kat_t multi_kat_vectors[];
    extern const uint16_t multi_kat_vectors_count;

    const struct device *dev = DEVICE_DT_GET_ONE(mindgrove_sha256);
    if (!device_is_ready(dev)) {
        printk("SHA256 device not ready\n");
        return -ENODEV;
    }

    for (uint16_t i = 0; i < multi_kat_vectors_count; i++) {
        const sha256_multi_kat_t *tv = &multi_kat_vectors[i];
        uint8_t output[SHA256_HASH_LEN] = {0};
        long int iterated_bits = 0;
        const long int block_bits = 512; // SHA-256 block size

        struct hash_ctx ctx;
        struct hash_pkt pkt;

        printk("KAT Multi-block #%d: Input length = %d bits\n", i + 1, tv->input_text_len);

        /* Begin session */
        if (hash_begin_session(dev, &ctx, CRYPTO_HASH_ALGO_SHA256)) {
            printk("hash_begin_session failed\n");
            return -EIO;
        }

        /* Feed all full 512-bit blocks via hash_update */
        while ((tv->input_text_len - iterated_bits) > block_bits) {
            pkt.in_buf = (uint8_t *)(tv->input_text + (iterated_bits / 8));
            pkt.in_len = block_bits / 8;  // full block in bytes
            pkt.out_buf = NULL;           // intermediate update

            if (hash_update(&ctx, &pkt)) {
                printk("hash_update failed\n");
                hash_free_session(dev, &ctx);
                return -EIO;
            }

            iterated_bits += block_bits;
        }

        /* Remaining bits go through hash_compute (final block) */
        long int remaining_bits = tv->input_text_len - iterated_bits;
        pkt.in_buf = (uint8_t *)(tv->input_text + (iterated_bits / 8));
        pkt.in_len = (remaining_bits + 7) / 8;  // round up last partial byte
        pkt.out_buf = output;

        if (hash_compute(&ctx, &pkt)) {
            printk("hash_compute failed\n");
            hash_free_session(dev, &ctx);
            return -EIO;
        }

        hash_free_session(dev, &ctx);

        /* Validate output */
        bool pass = true;
        for (uint8_t j = 0; j < SHA256_HASH_LEN; j++) {
            if (output[j] != tv->output_hash[j]) {
                printk("Multi-block KAT #%d FAILED at byte %d, Expected: %02x Got: %02x\n",
                       i + 1, j, tv->output_hash[j], output[j]);
                pass = false;
                break;
            }
        }

        if (pass) {
            printk("Multi-block KAT #%d PASS\n", i + 1);
        } else {
            return -EFAULT;
        }
    }

    return 0;
}




void main(void)
{
    printk("\n===== Mindgrove SHA256 Tests =====\n");
    // if(test_single_block()){
    //     printk("SINGLE SHOT FAILED\n");
    // }
    // else{
    //     printk("SINGLE SHOT PASSED\n");
    // }
    // if(test_multi_shot()){
    //     printk("MULTI SHOT FAILED\n");
    // }
    // else{
    //     printk("MULTI SHOT PASSED\n");
    // }
    if(test_sha256_single_block_kat()){
        printk("KAT SINGLE BLOCK FAILED\n");
    }
    else{
        printk("KAT SINGLE BLOCK PASSED\n");
    }
    // if(test_sha256_multi_block_kat()){
    //     printk("KAT MULTI BLOCK FAILED\n");
    // }
    // else{
    //     printk("KAT MULTI BLOCK PASSED\n");
    // }

    printk("\n===== SHA256 Tests Complete =====\n");
}
