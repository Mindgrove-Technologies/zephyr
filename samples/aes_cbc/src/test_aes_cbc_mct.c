#include "test_aes_helpers.h"

#define MC_OUTER_LOOPS 100U
#define MC_INNER_LOOPS 1000U
#define AES_BLOCK_SIZE 16U

static void xor_buf(uint8_t *dst, const uint8_t *a, const uint8_t *b,
                    size_t len)
{
    for (size_t i = 0; i < len; i++) {
        dst[i] = a[i] ^ b[i];
    }
}

static int run_cbc_block(const struct device *dev,
                          uint8_t *out, const uint8_t *in,
                          const uint8_t *key, uint16_t key_len,
                          uint8_t *iv,
                          enum cipher_op op)
{
    struct cipher_ctx ctx = {0};
    ctx.key.bit_stream = (uint8_t *)key;
    ctx.keylen = key_len;
    ctx.flags = CAP_RAW_KEY | CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS;

    if (cipher_begin_session(dev, &ctx,
            CRYPTO_CIPHER_ALGO_AES,
            CRYPTO_CIPHER_MODE_CBC,
            op)) {
        return -EIO;
    }

    struct cipher_pkt pkt = {
        .in_buf      = (uint8_t *)in,
        .in_len      = AES_BLOCK_SIZE,
        .out_buf     = out,
        .out_buf_max = AES_BLOCK_SIZE,
    };

    int ret = cipher_cbc_op(&ctx, &pkt, iv);
    cipher_free_session(dev, &ctx);
    return ret;
}

/* ============================================================
 * CBC MCT Encrypt 128
 * ============================================================ */
static int test_cbc_mct_encrypt_128(const struct device *dev)
{
    uint8_t key[16] = {
        0x88, 0x09, 0xe7, 0xdd, 0x3a, 0x95, 0x9e, 0xe5,
        0xd8, 0xdb, 0xb1, 0x3f, 0x50, 0x1f, 0x22, 0x74
    };
    uint8_t pt[16] = {
        0x1f, 0xd4, 0xee, 0x65, 0x60, 0x3e, 0x61, 0x30,
        0xcf, 0xc2, 0xa8, 0x2a, 0xb3, 0xd5, 0x6c, 0x24
    };
    uint8_t iv[16] = {
        0xe5, 0xc0, 0xbb, 0x53, 0x5d, 0x7d, 0x54, 0x57,
        0x2a, 0xd0, 0x6d, 0x17, 0x0a, 0x0e, 0x58, 0xae
    };
    const uint8_t expected[16] = {
        0x7b, 0xed, 0x76, 0x71, 0xc8, 0x91, 0x3a, 0xa1,
        0x33, 0x0f, 0x19, 0x37, 0x61, 0x52, 0x3e, 0x67
    };

    uint8_t ct[16] = {0};
    uint8_t prev_ct[16] = {0};
    uint8_t iv_orig[16];

    for (uint8_t i = 0; i < MC_OUTER_LOOPS; i++) {
        memcpy(iv_orig, iv, 16);

        for (uint16_t j = 0; j < MC_INNER_LOOPS; j++) {
            memcpy(prev_ct, ct, 16);

            if (run_cbc_block(dev, ct, pt, key, 16, iv,
                              CRYPTO_CIPHER_OP_ENCRYPT)) {
                printk("CBC MCT ENC 128 FAIL at outer=%d inner=%d\n",
                       i, j);
                return -EIO;
            }

            memcpy(iv, ct, 16);

            if (j == 0) {
                memcpy(pt, iv_orig, 16);
            } else {
                memcpy(pt, prev_ct, 16);
            }
        }

        xor_buf(key, key, ct, 16);
        memcpy(pt, prev_ct, 16);
    }

    if (memcmp(ct, expected, 16) == 0) {
        printk("CBC MCT ENC 128: PASS\n");
        return 0;
    }
    printk("CBC MCT ENC 128: FAIL\n");
    return -EFAULT;
}

/* ============================================================
 * CBC MCT Encrypt 192
 * ============================================================ */
static int test_cbc_mct_encrypt_192(const struct device *dev)
{
    uint8_t key[24] = {
        0xde, 0xa6, 0x4f, 0x83, 0xcf, 0xe6, 0xa0, 0xa1,
        0x83, 0xdd, 0xbe, 0x86, 0x5c, 0xfc, 0xa0, 0x59,
        0xb3, 0xc6, 0x15, 0xc1, 0x62, 0x3d, 0x63, 0xfc
    };
    uint8_t pt[16] = {
        0xcd, 0x0b, 0x8c, 0x8a, 0x81, 0x79, 0xec, 0xb1,
        0x71, 0xb6, 0x4c, 0x89, 0x4a, 0x4d, 0x60, 0xfd
    };
    uint8_t iv[16] = {
        0x42, 0x6f, 0xbc, 0x08, 0x7b, 0x50, 0xb3, 0x95,
        0xc0, 0xfc, 0x81, 0xef, 0x9f, 0xd6, 0xd1, 0xaa
    };
    const uint8_t expected[16] = {
        0xe6, 0x45, 0x7b, 0xfc, 0x34, 0x33, 0xe8, 0x02,
        0x99, 0xc5, 0x2b, 0x2b, 0xe4, 0x18, 0xf5, 0x82
    };

    uint8_t ct[16] = {0};
    uint8_t prev_ct[16] = {0};
    uint8_t iv_orig[16];
    uint8_t tmp[24];

    for (uint8_t i = 0; i < MC_OUTER_LOOPS; i++) {
        memcpy(iv_orig, iv, 16);

        for (uint16_t j = 0; j < MC_INNER_LOOPS; j++) {
            memcpy(prev_ct, ct, 16);

            if (run_cbc_block(dev, ct, pt, key, 24, iv,
                              CRYPTO_CIPHER_OP_ENCRYPT)) {
                printk("CBC MCT ENC 192 FAIL at outer=%d inner=%d\n",
                       i, j);
                return -EIO;
            }

            memcpy(iv, ct, 16);

            if (j == 0) {
                memcpy(pt, iv_orig, 16);
            } else {
                memcpy(pt, prev_ct, 16);
            }
        }

        /* Key update for 192-bit: XOR with prev_ct[8..15] || ct[0..15] */
        memcpy(tmp,     prev_ct + 8, 8);
        memcpy(tmp + 8, ct,          16);
        xor_buf(key, key, tmp, 24);
        memcpy(pt, prev_ct, 16);
    }

    if (memcmp(ct, expected, 16) == 0) {
        printk("CBC MCT ENC 192: PASS\n");
        return 0;
    }
    printk("CBC MCT ENC 192: FAIL\n");
    return -EFAULT;
}

/* ============================================================
 * CBC MCT Encrypt 256
 * ============================================================ */
static int test_cbc_mct_encrypt_256(const struct device *dev)
{
    uint8_t key[32] = {
        0x63, 0x2b, 0xac, 0x4f, 0xe4, 0xdb, 0x44, 0xcf,
        0xcf, 0x18, 0xcf, 0xa9, 0x0b, 0x43, 0xf8, 0x6f,
        0x37, 0x86, 0x11, 0xb8, 0xd9, 0x68, 0x59, 0x5e,
        0xb8, 0x9e, 0x7a, 0xe9, 0x86, 0x24, 0x56, 0x4a
    };
    uint8_t pt[16] = {
        0x90, 0xed, 0x17, 0x47, 0x5f, 0x0a, 0x62, 0xbc,
        0x38, 0x1b, 0xa1, 0xf3, 0xff, 0xbf, 0xff, 0x33
    };
    uint8_t iv[16] = {
        0xff, 0x81, 0x27, 0x62, 0x1b, 0xe6, 0x16, 0x80,
        0x3e, 0x3f, 0x00, 0x23, 0x77, 0x73, 0x01, 0x85
    };
    const uint8_t expected[16] = {
        0xba, 0xde, 0x16, 0x67, 0xb4, 0x2f, 0x53, 0x7f,
        0x0c, 0xb3, 0xf5, 0x57, 0x3a, 0x94, 0x9a, 0xaa
    };

    uint8_t ct[16] = {0};
    uint8_t prev_ct[16] = {0};
    uint8_t iv_orig[16];
    uint8_t tmp[32];

    for (uint8_t i = 0; i < MC_OUTER_LOOPS; i++) {
        memcpy(iv_orig, iv, 16);

        for (uint16_t j = 0; j < MC_INNER_LOOPS; j++) {
            memcpy(prev_ct, ct, 16);

            if (run_cbc_block(dev, ct, pt, key, 32, iv,
                              CRYPTO_CIPHER_OP_ENCRYPT)) {
                printk("CBC MCT ENC 256 FAIL at outer=%d inner=%d\n",
                       i, j);
                return -EIO;
            }

            memcpy(iv, ct, 16);

            if (j == 0) {
                memcpy(pt, iv_orig, 16);
            } else {
                memcpy(pt, prev_ct, 16);
            }
        }

        /* Key update for 256-bit: XOR with prev_ct || ct */
        memcpy(tmp,      prev_ct, 16);
        memcpy(tmp + 16, ct,      16);
        xor_buf(key, key, tmp, 32);
        memcpy(pt, prev_ct, 16);
    }

    if (memcmp(ct, expected, 16) == 0) {
        printk("CBC MCT ENC 256: PASS\n");
        return 0;
    }
    printk("CBC MCT ENC 256: FAIL\n");
    return -EFAULT;
}

/* ============================================================
 * CBC MCT Decrypt 128
 * ============================================================ */
static int test_cbc_mct_decrypt_128(const struct device *dev)
{
    uint8_t key[16] = {
        0x28, 0x7b, 0x07, 0xc7, 0x8f, 0x8e, 0x3e, 0x1b,
        0xe7, 0xc4, 0x1b, 0x3d, 0x96, 0xc0, 0x4e, 0x6e
    };
    uint8_t ct[16] = {
        0x7c, 0x54, 0x92, 0x3b, 0x04, 0x90, 0xa9, 0xd4,
        0xde, 0x4e, 0xc1, 0xce, 0x67, 0x90, 0xaa, 0x4d
    };
    uint8_t iv[16] = {
        0x41, 0xb4, 0x61, 0xf9, 0x46, 0x4f, 0xd5, 0x15,
        0xd2, 0x54, 0x13, 0xb4, 0x24, 0x10, 0x02, 0xb8
    };
    const uint8_t expected[16] = {
        0x47, 0x69, 0x31, 0x7b, 0x05, 0x62, 0xc4, 0x59,
        0x49, 0xc1, 0x8b, 0x38, 0x55, 0xf8, 0xbf, 0x4a
    };

    uint8_t pt[16] = {0};
    uint8_t prev_pt[16] = {0};
    uint8_t iv_orig[16];
    uint8_t ct_snap[16];

    for (uint8_t i = 0; i < MC_OUTER_LOOPS; i++) {
        memcpy(iv_orig, iv, 16);

        for (uint16_t j = 0; j < MC_INNER_LOOPS; j++) {
            memcpy(prev_pt, pt, 16);
            memcpy(ct_snap, ct, 16);

            if (run_cbc_block(dev, pt, ct, key, 16, iv,
                              CRYPTO_CIPHER_OP_DECRYPT)) {
                printk("CBC MCT DEC 128 FAIL at outer=%d inner=%d\n",
                       i, j);
                return -EIO;
            }

            memcpy(iv, ct_snap, 16);

            if (j == 0) {
                memcpy(ct, iv_orig, 16);
            } else {
                memcpy(ct, prev_pt, 16);
            }
        }

        xor_buf(key, key, pt, 16);
        memcpy(iv, pt, 16);
        memcpy(ct, prev_pt, 16);
    }

    if (memcmp(pt, expected, 16) == 0) {
        printk("CBC MCT DEC 128: PASS\n");
        return 0;
    }
    printk("CBC MCT DEC 128: FAIL\n");
    return -EFAULT;
}

/* ============================================================
 * CBC MCT Decrypt 192
 * ============================================================ */
static int test_cbc_mct_decrypt_192(const struct device *dev)
{
    uint8_t key[24] = {
        0xa2, 0x4e, 0xbd, 0x4d, 0x7a, 0x08, 0x0c, 0x28,
        0xca, 0xae, 0x98, 0x4b, 0x50, 0x98, 0xa9, 0xea,
        0x38, 0xcf, 0x72, 0x80, 0xe2, 0xc5, 0xf1, 0x22
    };
    uint8_t ct[16] = {
        0xeb, 0x2c, 0x4e, 0x27, 0x12, 0x59, 0x1f, 0xf1,
        0x3b, 0x8a, 0xc7, 0x87, 0x0c, 0x9c, 0x40, 0x4c
    };
    uint8_t iv[16] = {
        0xc5, 0xae, 0xb9, 0xb5, 0x1a, 0xd5, 0x10, 0x83,
        0x71, 0xc5, 0x9d, 0x0b, 0x90, 0x81, 0x63, 0x10
    };
    const uint8_t expected[16] = {
        0x83, 0x64, 0x24, 0xea, 0xdf, 0x81, 0x55, 0xaa,
        0xf9, 0xa9, 0xa5, 0x13, 0x91, 0xa1, 0xcf, 0x7e
    };

    uint8_t pt[16] = {0};
    uint8_t prev_pt[16] = {0};
    uint8_t iv_orig[16];
    uint8_t ct_snap[16];
    uint8_t tmp[24];

    for (uint8_t i = 0; i < MC_OUTER_LOOPS; i++) {
        memcpy(iv_orig, iv, 16);

        for (uint16_t j = 0; j < MC_INNER_LOOPS; j++) {
            memcpy(prev_pt, pt, 16);
            memcpy(ct_snap, ct, 16);

            if (run_cbc_block(dev, pt, ct, key, 24, iv,
                              CRYPTO_CIPHER_OP_DECRYPT)) {
                printk("CBC MCT DEC 192 FAIL at outer=%d inner=%d\n",
                       i, j);
                return -EIO;
            }

            memcpy(iv, ct_snap, 16);

            if (j == 0) {
                memcpy(ct, iv_orig, 16);
            } else {
                memcpy(ct, prev_pt, 16);
            }
        }

        /* Key update for 192-bit */
        memcpy(tmp,     prev_pt + 8, 8);
        memcpy(tmp + 8, pt,          16);
        xor_buf(key, key, tmp, 24);
        memcpy(iv, pt, 16);
        memcpy(ct, prev_pt, 16);
    }

    if (memcmp(pt, expected, 16) == 0) {
        printk("CBC MCT DEC 192: PASS\n");
        return 0;
    }
    printk("CBC MCT DEC 192: FAIL\n");
    return -EFAULT;
}

/* ============================================================
 * CBC MCT Decrypt 256
 * ============================================================ */
static int test_cbc_mct_decrypt_256(const struct device *dev)
{
    uint8_t key[32] = {
        0x31, 0x39, 0x7a, 0xd8, 0xcc, 0x79, 0xc5, 0x19,
        0xe0, 0xf4, 0x6e, 0x0f, 0x70, 0x30, 0x35, 0x87,
        0xe3, 0x89, 0x58, 0xd7, 0x07, 0x23, 0xb7, 0x71,
        0x55, 0x23, 0x36, 0xb7, 0x77, 0x1f, 0x63, 0x11
    };
    uint8_t ct[16] = {
        0x27, 0xa1, 0xd5, 0xc1, 0x0f, 0xe4, 0x5b, 0x80,
        0x1d, 0x15, 0xf5, 0x6e, 0x65, 0x4a, 0x70, 0xf0
    };
    uint8_t iv[16] = {
        0x41, 0x39, 0xcb, 0x54, 0xee, 0xac, 0x3f, 0xcf,
        0x36, 0xed, 0x72, 0x94, 0x11, 0x22, 0xc4, 0x0f
    };
    const uint8_t expected[16] = {
        0x9b, 0xe8, 0x31, 0x79, 0x9a, 0x79, 0xb0, 0x95,
        0x52, 0x41, 0xf3, 0x08, 0xf0, 0xd5, 0xb2, 0xe1
    };

    uint8_t pt[16] = {0};
    uint8_t prev_pt[16] = {0};
    uint8_t iv_orig[16];
    uint8_t ct_snap[16];
    uint8_t tmp[32];

    for (uint8_t i = 0; i < MC_OUTER_LOOPS; i++) {
        memcpy(iv_orig, iv, 16);

        for (uint16_t j = 0; j < MC_INNER_LOOPS; j++) {
            memcpy(prev_pt, pt, 16);
            memcpy(ct_snap, ct, 16);

            if (run_cbc_block(dev, pt, ct, key, 32, iv,
                              CRYPTO_CIPHER_OP_DECRYPT)) {
                printk("CBC MCT DEC 256 FAIL at outer=%d inner=%d\n",
                       i, j);
                return -EIO;
            }

            memcpy(iv, ct_snap, 16);

            if (j == 0) {
                memcpy(ct, iv_orig, 16);
            } else {
                memcpy(ct, prev_pt, 16);
            }
        }

        /* Key update for 256-bit */
        memcpy(tmp,      prev_pt, 16);
        memcpy(tmp + 16, pt,      16);
        xor_buf(key, key, tmp, 32);
        memcpy(iv, pt, 16);
        memcpy(ct, prev_pt, 16);
    }

    if (memcmp(pt, expected, 16) == 0) {
        printk("CBC MCT DEC 256: PASS\n");
        return 0;
    }
    printk("CBC MCT DEC 256: FAIL\n");
    return -EFAULT;
}

/* ============================================================
 * Entry point
 * ============================================================ */
int test_aes_cbc_mct(void)
{
    const struct device *dev = aes_get_dev();
    if (!dev) return -ENODEV;

    int ret = 0;
    ret |= test_cbc_mct_encrypt_128(dev);
    ret |= test_cbc_mct_encrypt_192(dev);
    ret |= test_cbc_mct_encrypt_256(dev);
    ret |= test_cbc_mct_decrypt_128(dev);
    ret |= test_cbc_mct_decrypt_192(dev);
    ret |= test_cbc_mct_decrypt_256(dev);

    printk("CBC MCT: %s\n", ret ? "FAILED" : "PASSED");
    return ret;
}