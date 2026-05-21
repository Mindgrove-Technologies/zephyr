#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/crypto/crypto.h>
#include <zephyr/crypto/cipher.h>
#include <string.h>
#include "rsp_files.h"

#define ASSERT_OK(expr, msg) \
    do { int _ret = (expr); if (_ret != 0) { printk("FAIL: %s (%d)\n", msg, _ret); return -1; } } while (0)

#define ASSERT_MEM_EQ(a,b,len,msg) \
    do { if (memcmp(a,b,len) != 0) { printk("FAIL: %s\n", msg); return -1; } } while (0)

static const struct device *aes_dev;

static int aes_init(void)
{
    aes_dev = DEVICE_DT_GET_ONE(mindgrove_aes);
    if (!device_is_ready(aes_dev)) {
        printk("AES device not ready\n");
        return -1;
    }
    return 0;
}

static int aes_kat_multimode(void)
{
    printk("=== AES ECB KAT START ===\n");

    extern const aes_kat_t kat_vectors[];
    extern const uint32_t kat_vectors_count;

    for (uint32_t i = 0; i < kat_vectors_count; i++) {
        const aes_kat_t *tv = &kat_vectors[i];

        /* Only execute if the vector is explicitly configured for ECB */
        if (tv->mode != AES_ECB) {
            continue;
        }

        printk("\n==============================\n");
        printk("Vector %u | ECB Mode | Key %u-bit\n", i, tv->key_len * 8);
        printk("==============================\n");

        uint8_t cipher_text[256] = {0};
        uint8_t decrypted_text[256] = {0};

        /* ================= ENCRYPT ================= */
        struct cipher_ctx ctx = {0};
        ctx.key.bit_stream = tv->key;
        ctx.keylen = tv->key_len;
        ctx.flags = CAP_RAW_KEY | CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS;

        ASSERT_OK(cipher_begin_session(aes_dev, &ctx,
                                       CRYPTO_CIPHER_ALGO_AES,
                                       CRYPTO_CIPHER_MODE_ECB,
                                       CRYPTO_CIPHER_OP_ENCRYPT),
                  "Encrypt begin failed");

        for (size_t off = 0; off < tv->pt_len; off += 16) {
            struct cipher_pkt pkt = {
                .in_buf = tv->plaintext + off,
                .in_len = 16,
                .out_buf = cipher_text + off,
                .out_buf_max = 16,
            };

            ASSERT_OK(cipher_block_op(&ctx, &pkt), "Encrypt block failed");
        }
        cipher_free_session(aes_dev, &ctx);

        ASSERT_MEM_EQ(cipher_text, tv->ciphertext, tv->ct_len, "Encryption mismatch");
        printk("Encryption PASS\n");

        /* ================= DECRYPT ================= */
        struct cipher_ctx ctx_dec = {0};
        ctx_dec.key.bit_stream = tv->key;
        ctx_dec.keylen = tv->key_len;
        ctx_dec.flags = CAP_RAW_KEY | CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS;

        ASSERT_OK(cipher_begin_session(aes_dev, &ctx_dec,
                                       CRYPTO_CIPHER_ALGO_AES,
                                       CRYPTO_CIPHER_MODE_ECB,
                                       CRYPTO_CIPHER_OP_DECRYPT),
                  "Decrypt begin failed");

        for (size_t off = 0; off < tv->ct_len; off += 16) {
            struct cipher_pkt pkt = {
                .in_buf = cipher_text + off,
                .in_len = 16,
                .out_buf = decrypted_text + off,
                .out_buf_max = 16,
            };

            ASSERT_OK(cipher_block_op(&ctx_dec, &pkt), "Decrypt block failed");
        }
        cipher_free_session(aes_dev, &ctx_dec);

        ASSERT_MEM_EQ(decrypted_text, tv->plaintext, tv->pt_len, "Decryption mismatch");
        printk("Decryption PASS\n");
    }

    printk("=== AES ECB KAT COMPLETE ===\n");
    return 0;
}

static int aes_mmt_multimode(void)
{
    printk("=== AES MMT ECB Test START ===\n");

    extern const aes_kat_t mmt_vectors[];
    extern const uint32_t mmt_vectors_count;

    uint8_t cipher_text[256];
    uint8_t decrypted_text[256];

    for (uint32_t i = 0; i < mmt_vectors_count; i++) {
        const aes_kat_t *tv = &mmt_vectors[i];

        if (tv->mode != AES_ECB) {
            continue;
        }

        printk("\n====================================\n");
        printk("MMT Vector %u | ECB Mode | Key %u-bit\n", i, tv->key_len * 8);
        printk("====================================\n");

        /* Safety limits and strict block checking */
        if (tv->pt_len > sizeof(cipher_text) || tv->ct_len > sizeof(cipher_text)) {
            printk("Vector size breaks static constraints\n");
            return -ENOMEM;
        }

        if ((tv->pt_len % 16) != 0 || (tv->ct_len % 16) != 0) {
            printk("ECB size must be 16-byte aligned\n");
            return -EINVAL;
        }

        /* ================= ENCRYPT ======================== */
        memset(cipher_text, 0, sizeof(cipher_text));

        struct cipher_ctx ctx_enc = {0};
        ctx_enc.key.bit_stream = tv->key;
        ctx_enc.keylen         = tv->key_len;
        ctx_enc.flags          = CAP_RAW_KEY | CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS;

        ASSERT_OK(cipher_begin_session(aes_dev, &ctx_enc,
                                       CRYPTO_CIPHER_ALGO_AES,
                                       CRYPTO_CIPHER_MODE_ECB,
                                       CRYPTO_CIPHER_OP_ENCRYPT),
                  "Encrypt begin failed");

        for (size_t off = 0; off < tv->pt_len; off += 16) {
            struct cipher_pkt pkt = {
                .in_buf      = tv->plaintext + off,
                .in_len      = 16,
                .out_buf     = cipher_text + off,
                .out_buf_max = 16,
            };

            ASSERT_OK(cipher_block_op(&ctx_enc, &pkt), "Encrypt block failed");
        }
        cipher_free_session(aes_dev, &ctx_enc);

        ASSERT_MEM_EQ(cipher_text, tv->ciphertext, tv->ct_len, "Encryption mismatch");
        printk("Encryption PASS\n");

        /* ================= DECRYPT ======================== */
        memset(decrypted_text, 0, sizeof(decrypted_text));

        struct cipher_ctx ctx_dec = {0};
        ctx_dec.key.bit_stream = tv->key;
        ctx_dec.keylen         = tv->key_len;
        ctx_dec.flags          = CAP_RAW_KEY | CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS;

        ASSERT_OK(cipher_begin_session(aes_dev, &ctx_dec,
                                       CRYPTO_CIPHER_ALGO_AES,
                                       CRYPTO_CIPHER_MODE_ECB,
                                       CRYPTO_CIPHER_OP_DECRYPT),
                  "Decrypt begin failed");

        for (size_t off = 0; off < tv->ct_len; off += 16) {
            struct cipher_pkt pkt = {
                .in_buf      = tv->ciphertext + off,
                .in_len      = 16,
                .out_buf     = decrypted_text + off,
                .out_buf_max = 16,
            };

            ASSERT_OK(cipher_block_op(&ctx_dec, &pkt), "Decrypt block failed");
        }
        cipher_free_session(aes_dev, &ctx_dec);

        ASSERT_MEM_EQ(decrypted_text, tv->plaintext, tv->pt_len, "Decryption mismatch");
        printk("Decryption PASS\n");
    }

    printk("=== AES MMT ECB Test COMPLETE ===\n");
    return 0;
}

void main(void)
{
    printk("=== AES SANITY TEST START ===\n");

    if (aes_init()) {
        goto fail;
    }

    if (aes_kat_multimode()) {
        printk("AES ECB SINGLE KAT FAILED\n");
        goto fail;
    } else {
        printk("AES ECB SINGLE KAT PASSED\n");
    }

    if (aes_mmt_multimode()) {
        printk("AES ECB MMT KAT FAILED\n");
        goto fail;
    } else {
        printk("AES ECB MMT KAT PASSED\n");
    }

    for (;;) {
        k_sleep(K_FOREVER);
    }

fail:
    printk("=== AES TEST FAILED ===\n");
    for (;;) {
        k_sleep(K_FOREVER);
    }
}