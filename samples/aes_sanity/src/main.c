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

struct cipher_ctx ctx = {0};

uint8_t key[16] = { 0x2B,0x7E,0x15,0x16,0x28,0xAE,0xD2,0xA6,
                    0xAB,0xF7,0x15,0x88,0x09,0xCF,0x4F,0x3C };

uint8_t iv[16] = {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,
                   0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F };

 uint8_t plaintext[2 * 16] = {
        // 0x6B, 0xC1, 0xBE, 0xE2, 0x2E, 0x40, 0x9F, 0x96,
        // 0xE9, 0x3D, 0x7E, 0x11, 0x73, 0x93, 0x17, 0x2A,

        0xAE, 0x2D, 0x8A, 0x57, 0x1E, 0x03, 0xAC, 0x9C,
        0x9E, 0xB7, 0x6F, 0xAC, 0x45, 0xAF, 0x8E, 0x51,

        0x30, 0xC8, 0x1C, 0x46, 0xA3, 0x5C, 0xE4, 0x11,
        0xE5, 0xFB, 0xC1, 0x19, 0x1A, 0x0A, 0x52, 0xEF};

uint8_t ciphertext[3 * 16];
uint8_t exp_cipher_text[3 * 16] = {
        0x76, 0x49, 0xAB, 0xAC, 0x81, 0x19, 0xB2, 0x46,
        0xCE, 0xE9, 0x8E, 0x9B, 0x12, 0xE9, 0x19, 0x7D,

        0x74, 0xfb, 0x75, 0x20, 0xc2, 0xcf, 0x7b, 0x6c,
        0x30, 0xab, 0x9b, 0x1f, 0x17, 0x0d, 0x9a, 0xf5,

        0x34, 0xf8, 0xa2, 0x3d, 0xc9, 0x17, 0x3e, 0x62,
        0x77, 0xa9, 0xbd, 0x7d, 0xd5, 0xf4, 0xdf, 0xc9};

static int aes_init(void)
{
    aes_dev = DEVICE_DT_GET_ONE(mindgrove_aes);
    if (!device_is_ready(aes_dev)) {
        printk("AES device not ready\n");
        return -1;
    }
    return 0;
}

#define ASSERT_OK(expr, msg) \
    do { int _ret = (expr); if (_ret != 0) { printk("FAIL: %s (%d)\n", msg, _ret); return -1; } } while (0)

#define ASSERT_MEM_EQ(a,b,len,msg) \
    do { if (memcmp(a,b,len) != 0) { printk("FAIL: %s\n", msg); return -1; } } while (0)

static void print_bytes(const char *label, const uint8_t *data, size_t len)
{
    printk("%s: ", label);
    for (size_t i = 0; i < len; i++) {
        printk("%02x ", data[i]);
    }
    printk("\n");
}


static void dump_hex(const char *label, const uint8_t *buf, size_t len)
{
    printk("%s (%u bytes):\n", label, len);
    for (size_t i = 0; i < len; i++) {
        printk("%02x ", buf[i]);
        if ((i + 1) % 16 == 0) {
            printk("\n");
        }
    }
    printk("\n");
}

static int aes_ecb_multirun_zephyr(void)
{
    printk("=== AES ECB MULTI-RUN (ZEPHYR) START ===\n");

    uint8_t key[16] = {
        0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6,
        0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C
    };

    uint8_t plaintext[3 * 16] = {
        0x6B,0xC1,0xBE,0xE2,0x2E,0x40,0x9F,0x96,
        0xE9,0x3D,0x7E,0x11,0x73,0x93,0x17,0x2A,

        0xAE,0x2D,0x8A,0x57,0x1E,0x03,0xAC,0x9C,
        0x9E,0xB7,0x6F,0xAC,0x45,0xAF,0x8E,0x51,

        0x30,0xC8,0x1C,0x46,0xA3,0x5C,0xE4,0x11,
        0xE5,0xFB,0xC1,0x19,0x1A,0x0A,0x52,0xEF
    };

    uint8_t expected[3 * 16] = {
        0x3A,0xD7,0x7B,0xB4,0x0D,0x7A,0x36,0x60,
        0xA8,0x9E,0xCA,0xF3,0x24,0x66,0xEF,0x97,

        0xF5,0xD3,0xD5,0x85,0x03,0xB9,0x69,0x9D,
        0xE7,0x85,0x89,0x5A,0x96,0xFD,0xBA,0xAF,

        0x43,0xB1,0xCD,0x7F,0x59,0x8E,0xCE,0x23,
        0x88,0x1B,0x00,0xE3,0xED,0x03,0x06,0x88
    };

    uint8_t ciphertext[3 * 16] = {0};

    struct cipher_ctx ctx = {0};
    ctx.key.bit_stream = key;
    ctx.keylen = sizeof(key);
    ctx.flags = CAP_RAW_KEY | CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS;

    ASSERT_OK(cipher_begin_session(aes_dev, &ctx,
                                   CRYPTO_CIPHER_ALGO_AES,
                                   CRYPTO_CIPHER_MODE_ECB,
                                   CRYPTO_CIPHER_OP_ENCRYPT),
              "ECB begin failed");

    /* MULTI-RUN: one block per call */
    for (uint8_t block = 0; block < 3; block++) {
        struct cipher_pkt pkt = {
            .in_buf = plaintext + block * 16,
            .in_len = 16,
            .out_buf = ciphertext + block * 16,
            .out_buf_max = 16,
        };

        printk("Encrypt block %u\n", block);
        print_inputs("IN ", pkt.in_buf, 16);

        ASSERT_OK(cipher_block_op(&ctx, &pkt),
                  "ECB multirun encrypt failed");

        print_inputs("OUT", pkt.out_buf, 16);
    }

    cipher_free_session(aes_dev, &ctx);

    ASSERT_MEM_EQ(ciphertext, expected, sizeof(expected),
                  "ECB multi-run ciphertext mismatch");

    printk("=== AES ECB MULTI-RUN (ZEPHYR) PASS ===\n");
    return 0;
}

static void print_cipher_comparison(const char *label,
                                    const uint8_t *cipher,
                                    const uint8_t *expected,
                                    size_t len)
{
    printk("%s comparison:\n", label);
    printk("Block | Ciphertext                     | Expected\n");
    printk("------------------------------------------------------\n");

    for (size_t i = 0; i < len; i += 16) {
        printk("%3zu   | ", i / 16);

        // Print actual ciphertext
        for (size_t j = 0; j < 16 && i + j < len; j++) {
            printk("%02X ", cipher[i + j]);
        }

        printk("| ");

        // Print expected
        for (size_t j = 0; j < 16 && i + j < len; j++) {
            printk("%02X ", expected[i + j]);
        }

        printk("\n");
    }
    printk("\n");
}

/* AES CBC MULTI-RUN (ZEPHYR, single IV) */
static int aes_cbc_multirun_zephyr()
{
    uint8_t key[16] = {
        0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6,
        0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C
    };

    uint8_t iv[16] = {
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
        0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F
    };

    uint8_t plaintext[3 * 16] = {
        0x6B, 0xC1, 0xBE, 0xE2, 0x2E, 0x40, 0x9F, 0x96,
        0xE9, 0x3D, 0x7E, 0x11, 0x73, 0x93, 0x17, 0x2A,
        0xAE, 0x2D, 0x8A, 0x57, 0x1E, 0x03, 0xAC, 0x9C,
        0x9E, 0xB7, 0x6F, 0xAC, 0x45, 0xAF, 0x8E, 0x51,
        0x30, 0xC8, 0x1C, 0x46, 0xA3, 0x5C, 0xE4, 0x11,
        0xE5, 0xFB, 0xC1, 0x19, 0x1A, 0x0A, 0x52, 0xEF
    };


    uint8_t expected[3 * 16] = {
        0x76, 0x49, 0xAB, 0xAC, 0x81, 0x19, 0xB2, 0x46,
        0xCE, 0xE9, 0x8E, 0x9B, 0x12, 0xE9, 0x19, 0x7D,
        0x50, 0x86, 0xCB, 0x9B, 0x50, 0x72, 0x19, 0xEE,
        0x95, 0xDB, 0x11, 0x3A, 0x91, 0x76, 0x78, 0xB2,
        0x73, 0xBE, 0xD6, 0xB8, 0xE3, 0xC1, 0x74, 0x3B,
        0x71, 0x16, 0xE6, 0x9E, 0x22, 0x22, 0x95, 0x16
    };

    uint8_t ciphertext[3 * 16] = {0};

    struct cipher_ctx ctx = {0};
    ctx.key.bit_stream = key;
    ctx.keylen = sizeof(key);
    ctx.flags = CAP_RAW_KEY | CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS;

    ASSERT_OK(cipher_begin_session(aes_dev, &ctx,
                                   CRYPTO_CIPHER_ALGO_AES,
                                   CRYPTO_CIPHER_MODE_CBC,
                                   CRYPTO_CIPHER_OP_ENCRYPT),
              "CBC begin failed");

    /* MULTI-RUN: feed one block at a time, IV only once */
    for (uint8_t block = 0; block < 3; block++) {
        struct cipher_pkt pkt = {
            .in_buf = plaintext + block * 16,
            .in_len = 16,
            .out_buf = ciphertext + block * 16,
            .out_buf_max = 16
        };

        if (block == 0) {
            /* Only first block passes IV */
            ASSERT_OK(cipher_cbc_op(&ctx, &pkt, iv),
                      "CBC first block encrypt failed");
        } else {
            /* Subsequent blocks: hardware keeps chaining */
            ASSERT_OK(cipher_cbc_op(&ctx, &pkt, NULL),
                      "CBC subsequent block encrypt failed");
        }
    }

    cipher_free_session(aes_dev, &ctx);

    ASSERT_MEM_EQ(ciphertext, expected, sizeof(expected),
                  "CBC multi-run ciphertext mismatch");

    print_cipher_comparison("CBC multi-run", ciphertext, expected, 48); // 3 blocks × 16 bytes


    printk("=== AES CBC MULTI-RUN (ZEPHYR) PASS ===\n");
    return 0;
}
/* AES-128 CTR – Multi-block test (Zephyr, hardware CTR) */
static int aes_ctr_multirun_zephyr(void)
{
    printk("=== AES CTR MULTI-RUN (ZEPHYR, hardware CTR) START ===\n");

    uint8_t key[16] = {
        0x2B,0x7E,0x15,0x16,0x28,0xAE,0xD2,0xA6,
        0xAB,0xF7,0x15,0x88,0x09,0xCF,0x4F,0x3C
    };

    uint8_t iv[16] = {
        0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,
        0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F
    };

    uint8_t plaintext[48] = {
        0x6B,0xC1,0xBE,0xE2,0x2E,0x40,0x9F,0x96,
        0xE9,0x3D,0x7E,0x11,0x73,0x93,0x17,0x2A,
        0xAE,0x2D,0x8A,0x57,0x1E,0x03,0xAC,0x9C,
        0x9E,0xB7,0x6F,0xAC,0x45,0xAF,0x8E,0x51,
        0x30,0xC8,0x1C,0x46,0xA3,0x5C,0xE4,0x11,
        0xE5,0xFB,0xC1,0x19,0x1A,0x0A,0x52,0xEF
    };

    uint8_t expected[48] = {
        0x3B,0x3F,0xD9,0x2E,0xB7,0x2D,0xAD,0x20,
        0x33,0x34,0x49,0xF8,0xE8,0x3C,0xFB,0x4A,
        0x01,0x0C,0x04,0x19,0x99,0xE0,0x3F,0x36,
        0x44,0x86,0x24,0x48,0x3E,0x58,0x2D,0x0E,
        0xA6,0x22,0x93,0xCF,0xA6,0xDF,0x74,0x53,
        0x5C,0x35,0x41,0x81,0x16,0x87,0x74,0xDF
    };

    uint8_t ciphertext[48] = {0};

    struct cipher_ctx ctx = {0};
    ctx.key.bit_stream = key;
    ctx.keylen = sizeof(key);
    ctx.flags = CAP_RAW_KEY | CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS;

    /* Single session for all blocks */
    ASSERT_OK(cipher_begin_session(aes_dev, &ctx,
                                   CRYPTO_CIPHER_ALGO_AES,
                                   CRYPTO_CIPHER_MODE_CTR,
                                   CRYPTO_CIPHER_OP_ENCRYPT),
              "CTR session begin failed");

    /* Feed one block at a time; only first block passes IV */
    for (uint8_t block = 0; block < 3; block++) {
        struct cipher_pkt pkt = {
            .in_buf = plaintext + block * 16,
            .in_len = 16,
            .out_buf = ciphertext + block * 16,
            .out_buf_max = 16
        };

        uint8_t *iv_ptr = (block == 0) ? iv : NULL;

        ASSERT_OK(cipher_ctr_op(&ctx, &pkt, iv_ptr),
                  "CTR multi-block encrypt failed");

        printk("Encrypted CTR block %u\n", block);
    }

    cipher_free_session(aes_dev, &ctx);

    print_cipher_comparison("CTR multi-run", ciphertext, expected, sizeof(expected));
    ASSERT_MEM_EQ(ciphertext, expected, sizeof(expected),
                  "CTR multi-run ciphertext mismatch");

    printk("=== AES CTR MULTI-RUN (ZEPHYR) PASS ===\n");
    return 0;
}


// static int aes_ctr_multirun_zephyr(void)
// {
//     printk("=== AES CTR NIST KAT (ZEPHYR, hardware CTR) START ===\n");

//     uint8_t key[16] = {
//         0x2b,0x7e,0x15,0x16,0x28,0xae,0xd2,0xa6,
//         0xab,0xf7,0x15,0x88,0x09,0xcf,0x4f,0x3c
//     };

//     uint8_t iv[16] = {
//         0xf0,0xf1,0xf2,0xf3,0xf4,0xf5,0xf6,0xf7,
//         0xf8,0xf9,0xfa,0xfb,0xfc,0xfd,0xfe,0xff
//     };

//     uint8_t plaintext[64] = {
//         0x6b,0xc1,0xbe,0xe2,0x2e,0x40,0x9f,0x96,
//         0xe9,0x3d,0x7e,0x11,0x73,0x93,0x17,0x2a,
//         0xae,0x2d,0x8a,0x57,0x1e,0x03,0xac,0x9c,
//         0x9e,0xb7,0x6f,0xac,0x45,0xaf,0x8e,0x51,
//         0x30,0xc8,0x1c,0x46,0xa3,0x5c,0xe4,0x11,
//         0xe5,0xfb,0xc1,0x19,0x1a,0x0a,0x52,0xef,
//         0xf6,0x9f,0x24,0x45,0xdf,0x4f,0x9b,0x17,
//         0xad,0x2b,0x41,0x7b,0xe6,0x6c,0x37,0x10
//     };

//     uint8_t expected[64] = {
//         0x87,0x4d,0x61,0x91,0xb6,0x20,0xe3,0x26,
//         0x1b,0xef,0x68,0x64,0x99,0x0d,0xb6,0xce,
//         0x98,0x06,0xf6,0x6b,0x79,0x70,0xfd,0xff,
//         0x86,0x17,0x18,0x7b,0xb9,0xff,0xfd,0xff,
//         0x5a,0xe4,0xdf,0x3e,0xdb,0xd5,0xd3,0x5e,
//         0x5b,0x4f,0x09,0x02,0x0d,0xb0,0x3e,0xab,
//         0x1e,0x03,0x1d,0xda,0x2f,0xbe,0x03,0xd1,
//         0x79,0x21,0x70,0xa0,0xf3,0x00,0x9c,0xee
//     };

//     uint8_t ciphertext[64] = {0};

//     struct cipher_ctx ctx = {0};
//     ctx.key.bit_stream = key;
//     ctx.keylen = sizeof(key);
//     ctx.flags = CAP_RAW_KEY | CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS;

//     /* Single session for all blocks (hardware handles CTR increment) */
//     ASSERT_OK(cipher_begin_session(aes_dev, &ctx,
//                                    CRYPTO_CIPHER_ALGO_AES,
//                                    CRYPTO_CIPHER_MODE_CTR,
//                                    CRYPTO_CIPHER_OP_ENCRYPT),
//               "CTR session begin failed");

//     struct cipher_pkt pkt = {
//         .in_buf = plaintext,
//         .in_len = sizeof(plaintext),
//         .out_buf = ciphertext,
//         .out_buf_max = sizeof(ciphertext)
//     };

//     ASSERT_OK(cipher_ctr_op(&ctx, &pkt, iv),
//               "CTR encrypt failed");

//     cipher_free_session(aes_dev, &ctx);

//     print_cipher_comparison("CTR NIST KAT", ciphertext, expected, sizeof(expected));
//     ASSERT_MEM_EQ(ciphertext, expected, sizeof(expected),
//                   "CTR NIST KAT ciphertext mismatch");

//     printk("=== AES CTR NIST KAT (ZEPHYR, hardware CTR) PASS ===\n");
//     return 0;
// }



static int aes_test_single_ecb(void)
{
    uint8_t out[16] = {0};

    printk("=== AES ECB SINGLE-RUN TEST ===\n");

    struct cipher_ctx ctx = {
        .key.bit_stream = key,
        .keylen = sizeof(key),
        .flags = CAP_RAW_KEY | CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS,
    };

    // uint8_t expected[16] = {
    // 0x3a, 0xd7, 0x7b, 0xb4,
    // 0x0d, 0x7a, 0x36, 0x60,
    // 0xa8, 0x9e, 0xca, 0xf3,
    // 0x24, 0x66, 0xef, 0x97
    // };
    uint8_t expected[16] = {
    0xF5, 0xD3, 0xD5, 0x85, 0x03, 0xB9, 0x69, 0x9D,
    0xE7, 0x85, 0x89, 0x5A, 0x96, 0xFD, 0xBA, 0xAF
    };


    ASSERT_OK(cipher_begin_session(aes_dev, &ctx,
                                   CRYPTO_CIPHER_ALGO_AES,
                                   CRYPTO_CIPHER_MODE_ECB,
                                   CRYPTO_CIPHER_OP_ENCRYPT),
              "ECB begin failed");

    struct cipher_pkt pkt = {
        .in_buf = plaintext,     /* first 16 bytes only */
        .in_len = 16,
        .out_buf = out,
        .out_buf_max = 16,
    };

    printk("ECB: before cipher_block_op\n");
    ASSERT_OK(cipher_block_op(&ctx, &pkt), "ECB encrypt failed");
    printk("ECB: after cipher_block_op\n");

    cipher_free_session(aes_dev, &ctx);

    print_bytes("ECB output", out, 16);
    print_bytes("ECB expected", expected, 16);

    ASSERT_MEM_EQ(out, expected, 16, "ECB single-run mismatch");

    printk("PASS: AES ECB single-run\n");
    return 0;
}

static int aes_ecb_multivector(void)
{
       printk("=== ECB Multi-vector START Test  ===\n");

    extern const aes_kat_t kat_vectors[];
    extern const uint32_t kat_vectors_count;

    for (uint32_t i = 0; i < kat_vectors_count; i++) {
        const aes_kat_t *tv = &kat_vectors[i];

        /* Step 1: isolate ECB */
        if (tv->mode != AES_ECB)
            continue;

        printk("\n==============================\n");
        printk("ECB Test Vector %u\n", i);
        printk("==============================\n");

        print_inputs("Key", tv->key, tv->key_len);
        print_inputs("Plaintext", tv->plaintext, tv->pt_len);
        print_inputs("Expected CT", tv->ciphertext, tv->ct_len);

        uint8_t cipher_text[64] = {0};
        uint8_t decrypted_text[64] = {0};

        /* Step 2: fresh context per vector */
        struct cipher_ctx ctx = {0};
        ctx.key.bit_stream = tv->key;
        ctx.keylen = tv->key_len;
        ctx.flags = CAP_RAW_KEY | CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS;

        ASSERT_OK(cipher_begin_session(aes_dev, &ctx,
                                       CRYPTO_CIPHER_ALGO_AES,
                                       CRYPTO_CIPHER_MODE_ECB,
                                       CRYPTO_CIPHER_OP_ENCRYPT),
                  "ECB begin failed");

        /* Step 3: encrypt ONE BLOCK AT A TIME */
        for (size_t off = 0; off < tv->pt_len; off += 16) {
            struct cipher_pkt pkt = {
                .in_buf = tv->plaintext + off,
                .in_len = 16,
                .out_buf = cipher_text + off,
                .out_buf_max = 16,
            };

            printk("Encrypt block %zu\n", off / 16);
            print_inputs("IN ", pkt.in_buf, 16);

            ASSERT_OK(cipher_block_op(&ctx, &pkt),
                      "ECB encrypt failed");

            print_inputs("OUT", pkt.out_buf, 16);
        }

        cipher_free_session(aes_dev, &ctx);

        /* Step 4: verify ciphertext */
        ASSERT_MEM_EQ(cipher_text, tv->ciphertext, tv->ct_len,
                      "ECB ciphertext mismatch");

        printk("ECB encryption PASS\n");

        /* Step 5: decryption (same rules) */
        struct cipher_ctx ctx_dec = {0};
        ctx_dec.key.bit_stream = tv->key;
        ctx_dec.keylen = tv->key_len;
        ctx_dec.flags = CAP_RAW_KEY | CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS;

        ASSERT_OK(cipher_begin_session(aes_dev, &ctx_dec,
                                       CRYPTO_CIPHER_ALGO_AES,
                                       CRYPTO_CIPHER_MODE_ECB,
                                       CRYPTO_CIPHER_OP_DECRYPT),
                  "ECB decrypt begin failed");

        for (size_t off = 0; off < tv->ct_len; off += 16) {
            struct cipher_pkt pkt = {
                .in_buf = cipher_text + off,
                .in_len = 16,
                .out_buf = decrypted_text + off,
                .out_buf_max = 16,
            };

            ASSERT_OK(cipher_block_op(&ctx_dec, &pkt),
                      "ECB decrypt failed");
        }

        cipher_free_session(aes_dev, &ctx_dec);

        ASSERT_MEM_EQ(decrypted_text, tv->plaintext, tv->pt_len,
                      "ECB decrypt mismatch");

        printk("ECB decryption PASS\n");
    }

    printk("=== ECB Multi-vector Debug Test Complete ===\n");
    return 0;
}

static int aes_cbc_multivector(void)
{
    printk("=== CBC Multi-vector START Test ===\n");

    extern const aes_kat_t kat_vectors[];
    extern const uint32_t kat_vectors_count;

    for (uint32_t i = 0; i < kat_vectors_count; i++) {
        const aes_kat_t *tv = &kat_vectors[i];

        if (tv->mode != AES_CBC)
            continue;

        printk("\n==============================\n");
        printk("CBC Test Vector %u\n", i);
        printk("==============================\n");

        print_inputs("Key", tv->key, tv->key_len);
        print_inputs("IV ", tv->iv, tv->iv_len);
        print_inputs("PT ", tv->plaintext, tv->pt_len);
        print_inputs("EXP", tv->ciphertext, tv->ct_len);

        uint8_t cipher_text[64] = {0};
        uint8_t decrypted_text[64] = {0};

        /* ---------- ENCRYPT ---------- */

        struct cipher_ctx ctx = {0};
        ctx.key.bit_stream = tv->key;
        ctx.keylen = tv->key_len;
        ctx.flags = CAP_RAW_KEY | CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS;

        uint8_t iv_enc[16];
        memcpy(iv_enc, tv->iv, tv->iv_len);

        ASSERT_OK(cipher_begin_session(aes_dev, &ctx,
                                       CRYPTO_CIPHER_ALGO_AES,
                                       CRYPTO_CIPHER_MODE_CBC,
                                       CRYPTO_CIPHER_OP_ENCRYPT),
                  "CBC encrypt begin failed");

        for (size_t off = 0; off < tv->pt_len; off += 16) {
            struct cipher_pkt pkt = {
                .in_buf = tv->plaintext + off,
                .in_len = 16,
                .out_buf = cipher_text + off,
                .out_buf_max = 16,
            };

            printk("Encrypt block %zu\n", off / 16);
            print_inputs("IN ", pkt.in_buf, 16);
            print_inputs("IV ", iv_enc, 16);

            ASSERT_OK(cipher_cbc_op(&ctx, &pkt, iv_enc),
                      "CBC encrypt block failed");

            print_inputs("OUT", pkt.out_buf, 16);
        }

        cipher_free_session(aes_dev, &ctx);

        ASSERT_MEM_EQ(cipher_text, tv->ciphertext, tv->ct_len,
                      "CBC encryption mismatch");

        printk("CBC encryption PASS\n");

        /* ---------- DECRYPT ---------- */

        struct cipher_ctx ctx_dec = {0};
        ctx_dec.key.bit_stream = tv->key;
        ctx_dec.keylen = tv->key_len;
        ctx_dec.flags = CAP_RAW_KEY | CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS;

        uint8_t iv_dec[16];
        memcpy(iv_dec, tv->iv, tv->iv_len);

        ASSERT_OK(cipher_begin_session(aes_dev, &ctx_dec,
                                       CRYPTO_CIPHER_ALGO_AES,
                                       CRYPTO_CIPHER_MODE_CBC,
                                       CRYPTO_CIPHER_OP_DECRYPT),
                  "CBC decrypt begin failed");

        for (size_t off = 0; off < tv->ct_len; off += 16) {
            struct cipher_pkt pkt = {
                .in_buf = cipher_text + off,
                .in_len = 16,
                .out_buf = decrypted_text + off,
                .out_buf_max = 16,
            };

            printk("Decrypt block %zu\n", off / 16);
            print_inputs("IN ", pkt.in_buf, 16);
            print_inputs("IV ", iv_dec, 16);

            ASSERT_OK(cipher_cbc_op(&ctx_dec, &pkt, iv_dec),
                      "CBC decrypt block failed");

            print_inputs("OUT", pkt.out_buf, 16);
        }

        cipher_free_session(aes_dev, &ctx_dec);

        ASSERT_MEM_EQ(decrypted_text, tv->plaintext, tv->pt_len,
                      "CBC decrypt mismatch");

        printk("CBC decryption PASS\n");
    }

    printk("=== CBC Multi-vector Test Complete ===\n");
    return 0;
}

static int aes_ctr_multivector(void)
{
    printk("=== CTR Multi-vector START Test ===\n");

    extern const aes_kat_t kat_vectors[];
    extern const uint32_t kat_vectors_count;

    for (uint32_t i = 0; i < kat_vectors_count; i++) {
        const aes_kat_t *tv = &kat_vectors[i];

        if (tv->mode != AES_CTR)
            continue;

        printk("\n==============================\n");
        printk("CTR Test Vector %u\n", i);
        printk("==============================\n");

        print_inputs("Key", tv->key, tv->key_len);
        print_inputs("CTR", tv->iv, tv->iv_len);
        print_inputs("PT ", tv->plaintext, tv->pt_len);
        print_inputs("EXP", tv->ciphertext, tv->ct_len);

        uint8_t cipher_text[64] = {0};
        uint8_t decrypted_text[64] = {0};

        /* ---------- ENCRYPT ---------- */

        struct cipher_ctx ctx = {0};
        ctx.key.bit_stream = tv->key;
        ctx.keylen = tv->key_len;
        ctx.flags = CAP_RAW_KEY | CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS;

        uint8_t ctr_enc[16];
        memcpy(ctr_enc, tv->iv, tv->iv_len);

        ASSERT_OK(cipher_begin_session(aes_dev, &ctx,
                                       CRYPTO_CIPHER_ALGO_AES,
                                       CRYPTO_CIPHER_MODE_CTR,
                                       CRYPTO_CIPHER_OP_ENCRYPT),
                  "CTR encrypt begin failed");

        for (size_t off = 0; off < tv->pt_len; off += 16) {
            size_t blk_len = MIN(16, tv->pt_len - off);

            struct cipher_pkt pkt = {
                .in_buf = tv->plaintext + off,
                .in_len = blk_len,
                .out_buf = cipher_text + off,
                .out_buf_max = blk_len,
            };

            printk("Encrypt block %zu\n", off / 16);
            print_inputs("IN ", pkt.in_buf, blk_len);
            print_inputs("CTR", ctr_enc, 16);

            ASSERT_OK(cipher_ctr_op(&ctx, &pkt, ctr_enc),
                      "CTR encrypt block failed");

            print_inputs("OUT", pkt.out_buf, blk_len);
        }

        cipher_free_session(aes_dev, &ctx);

        ASSERT_MEM_EQ(cipher_text, tv->ciphertext, tv->ct_len,
                      "CTR encryption mismatch");

        printk("CTR encryption PASS\n");

        /* ---------- DECRYPT (same operation) ---------- */

        struct cipher_ctx ctx_dec = {0};
        ctx_dec.key.bit_stream = tv->key;
        ctx_dec.keylen = tv->key_len;
        ctx_dec.flags = CAP_RAW_KEY | CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS;

        uint8_t ctr_dec[16];
        memcpy(ctr_dec, tv->iv, tv->iv_len);

        ASSERT_OK(cipher_begin_session(aes_dev, &ctx_dec,
                                       CRYPTO_CIPHER_ALGO_AES,
                                       CRYPTO_CIPHER_MODE_CTR,
                                       CRYPTO_CIPHER_OP_DECRYPT),
                  "CTR decrypt begin failed");

        for (size_t off = 0; off < tv->ct_len; off += 16) {
            size_t blk_len = MIN(16, tv->ct_len - off);

            struct cipher_pkt pkt = {
                .in_buf = cipher_text + off,
                .in_len = blk_len,
                .out_buf = decrypted_text + off,
                .out_buf_max = blk_len,
            };

            printk("Decrypt block %zu\n", off / 16);
            print_inputs("IN ", pkt.in_buf, blk_len);
            print_inputs("CTR", ctr_dec, 16);

            ASSERT_OK(cipher_ctr_op(&ctx_dec, &pkt, ctr_dec),
                      "CTR decrypt block failed");

            print_inputs("OUT", pkt.out_buf, blk_len);
        }

        cipher_free_session(aes_dev, &ctx_dec);

        ASSERT_MEM_EQ(decrypted_text, tv->plaintext, tv->pt_len,
                      "CTR decrypt mismatch");

        printk("CTR decryption PASS\n");
    }

    printk("=== CTR Multi-vector Test Complete ===\n");
    return 0;
}


void main(void)
{
    printk("=== AES SANITY TEST START ===\n");

    if (aes_init()) {
        goto fail;
    }
    // if (aes_ecb_multivector()) {
    //     printk("AES KAT FAILED\n");
    // } else {
    //     printk("AES KAT PASSED\n");
    // }

    //  if (aes_cbc_multivector()) {
    //     printk("AES CBC KAT FAILED\n");
    // } else {
    //     printk("AES CBC KAT PASSED\n");
    // }

    //  if (aes_ctr_multivector()) {
    //     printk("AES CTR KAT FAILED\n");
    // } else {
    //     printk("AES CTR KAT PASSED\n");
    // } 

    // if (aes_ecb_multirun_zephyr()) {
    //     printk("AES Multi ECB KAT FAILED\n");
    // } else {
    //     printk("AES Multi ECB KAT PASSED\n");
    // }

    // if (aes_cbc_multirun_zephyr()) {
    //     printk("AES Multi CBC KAT FAILED\n");
    // } else {
    //     printk("AES Multi CBC KAT PASSED\n");
    // }

    if (aes_ctr_multirun_zephyr()) {
        printk("AES Multi CTR KAT FAILED\n");
    } else {
        printk("AES Multi CTR KAT PASSED\n");
    }

    // if (aes_test_single_ecb()) {
    //     goto fail;
    // }
    // else{
    //     printk("=== AES ECB SINGLE-RUN PASSED ===\n");
    // }


    for (;;) {
        k_sleep(K_FOREVER);
    }

fail:
    printk("=== AES TEST FAILED ===\n");
    for (;;) {
        k_sleep(K_FOREVER);
    }
}

