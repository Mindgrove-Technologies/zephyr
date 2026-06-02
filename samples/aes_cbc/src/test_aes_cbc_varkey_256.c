#include "test_aes_helpers.h"

#if defined(CONFIG_AES_CBC_VARKEY_256_01)
#include "aes256_cbc_varkey_01.h"
#endif
#if defined(CONFIG_AES_CBC_VARKEY_256_02)
#include "aes256_cbc_varkey_02.h"
#endif
#if defined(CONFIG_AES_CBC_VARKEY_256_03)
#include "aes256_cbc_varkey_03.h"
#endif
#if defined(CONFIG_AES_CBC_VARKEY_256_04)
#include "aes256_cbc_varkey_04.h"
#endif
#if defined(CONFIG_AES_CBC_VARKEY_256_05)
#include "aes256_cbc_varkey_05.h"
#endif
#if defined(CONFIG_AES_CBC_VARKEY_256_06)
#include "aes256_cbc_varkey_06.h"
#endif
#if defined(CONFIG_AES_CBC_VARKEY_256_07)
#include "aes256_cbc_varkey_07.h"
#endif
#if defined(CONFIG_AES_CBC_VARKEY_256_08)
#include "aes256_cbc_varkey_08.h"
#endif
#if defined(CONFIG_AES_CBC_VARKEY_256_09)
#include "aes256_cbc_varkey_09.h"
#endif
#if defined(CONFIG_AES_CBC_VARKEY_256_10)
#include "aes256_cbc_varkey_10.h"
#endif
#if defined(CONFIG_AES_CBC_VARKEY_256_11)
#include "aes256_cbc_varkey_11.h"
#endif

/* Same RUN_CBC_BATCH macro — paste from varkey_128.c */
#define RUN_CBC_BATCH(type, arr, cnt, label)                            \
    do {                                                                \
        uint32_t passed = 0, failed = 0;                               \
        for (uint32_t i = 0; i < (cnt); i++) {                        \
            const type *tv = &(arr)[i];                                \
            uint8_t ct[256] = {0};                                     \
            uint8_t pt[256] = {0};                                     \
            uint8_t iv[16];                                            \
            struct cipher_ctx ctx = {0};                               \
            ctx.key.bit_stream = (uint8_t *)tv->key;                  \
            ctx.keylen = tv->key_len;                                  \
            ctx.flags = CAP_RAW_KEY | CAP_SYNC_OPS |                  \
                        CAP_SEPARATE_IO_BUFS;                          \
            if (cipher_begin_session(dev, &ctx,                        \
                    CRYPTO_CIPHER_ALGO_AES,                            \
                    CRYPTO_CIPHER_MODE_CBC,                            \
                    CRYPTO_CIPHER_OP_ENCRYPT)) {                       \
                printk("[" label " %03d] FAIL (enc session)\n", i);   \
                failed++; continue;                                    \
            }                                                          \
            memcpy(iv, tv->iv, tv->iv_len);                           \
            bool enc_ok = true;                                        \
            for (size_t off = 0; off < tv->pt_len; off += 16) {      \
                struct cipher_pkt pkt = {                              \
                    .in_buf      = (uint8_t *)tv->plaintext + off,    \
                    .in_len      = 16,                                 \
                    .out_buf     = ct + off,                           \
                    .out_buf_max = 16,                                 \
                };                                                     \
                if (cipher_cbc_op(&ctx, &pkt, iv)) {                  \
                    printk("[" label " %03d] FAIL (enc op)\n", i);    \
                    enc_ok = false; break;                             \
                }                                                      \
            }                                                          \
            cipher_free_session(dev, &ctx);                            \
            if (!enc_ok) { failed++; continue; }                       \
            if (memcmp(ct, tv->ciphertext, tv->ct_len) != 0) {        \
                printk("[" label " %03d] FAIL (enc mismatch)\n", i);  \
                failed++; continue;                                    \
            }                                                          \
            struct cipher_ctx ctx_dec = {0};                           \
            ctx_dec.key.bit_stream = (uint8_t *)tv->key;              \
            ctx_dec.keylen = tv->key_len;                              \
            ctx_dec.flags = CAP_RAW_KEY | CAP_SYNC_OPS |              \
                            CAP_SEPARATE_IO_BUFS;                      \
            if (cipher_begin_session(dev, &ctx_dec,                    \
                    CRYPTO_CIPHER_ALGO_AES,                            \
                    CRYPTO_CIPHER_MODE_CBC,                            \
                    CRYPTO_CIPHER_OP_DECRYPT)) {                       \
                printk("[" label " %03d] FAIL (dec session)\n", i);   \
                failed++; continue;                                    \
            }                                                          \
            memcpy(iv, tv->iv, tv->iv_len);                           \
            bool dec_ok = true;                                        \
            for (size_t off = 0; off < tv->ct_len; off += 16) {      \
                struct cipher_pkt pkt = {                              \
                    .in_buf      = ct + off,                           \
                    .in_len      = 16,                                 \
                    .out_buf     = pt + off,                           \
                    .out_buf_max = 16,                                 \
                };                                                     \
                if (cipher_cbc_op(&ctx_dec, &pkt, iv)) {              \
                    printk("[" label " %03d] FAIL (dec op)\n", i);    \
                    dec_ok = false; break;                             \
                }                                                      \
            }                                                          \
            cipher_free_session(dev, &ctx_dec);                        \
            if (!dec_ok) { failed++; continue; }                       \
            if (memcmp(pt, tv->plaintext, tv->pt_len) != 0) {         \
                printk("[" label " %03d] FAIL (dec mismatch)\n", i);  \
                failed++;                                              \
            } else {                                                   \
                printk("[" label " %03d] PASS\n", i);                 \
                passed++;                                              \
            }                                                          \
        }                                                              \
        printk(label ": %d Passed, %d Failed\n", passed, failed);     \
        if (failed) return -EIO;                                       \
    } while (0)

int test_aes_cbc_varkey_256(void)
{
    const struct device *dev = aes_get_dev();
    if (!dev) return -ENODEV;

#if defined(CONFIG_AES_CBC_VARKEY_256_01)
    RUN_CBC_BATCH(CBC_VarKey_256_KAT_Test_Vectors_01,
                  cbc_varkey_256_kat_vectors_01,
                  cbc_varkey_256_kat_vectors_01_count,
                  "CBC VARKEY 256-01");
#endif
#if defined(CONFIG_AES_CBC_VARKEY_256_02)
    RUN_CBC_BATCH(CBC_VarKey_256_KAT_Test_Vectors_02,
                  cbc_varkey_256_kat_vectors_02,
                  cbc_varkey_256_kat_vectors_02_count,
                  "CBC VARKEY 256-02");
#endif
#if defined(CONFIG_AES_CBC_VARKEY_256_03)
    RUN_CBC_BATCH(CBC_VarKey_256_KAT_Test_Vectors_03,
                  cbc_varkey_256_kat_vectors_03,
                  cbc_varkey_256_kat_vectors_03_count,
                  "CBC VARKEY 256-03");
#endif
#if defined(CONFIG_AES_CBC_VARKEY_256_04)
    RUN_CBC_BATCH(CBC_VarKey_256_KAT_Test_Vectors_04,
                  cbc_varkey_256_kat_vectors_04,
                  cbc_varkey_256_kat_vectors_04_count,
                  "CBC VARKEY 256-04");
#endif
#if defined(CONFIG_AES_CBC_VARKEY_256_05)
    RUN_CBC_BATCH(CBC_VarKey_256_KAT_Test_Vectors_05,
                  cbc_varkey_256_kat_vectors_05,
                  cbc_varkey_256_kat_vectors_05_count,
                  "CBC VARKEY 256-05");
#endif
#if defined(CONFIG_AES_CBC_VARKEY_256_06)
    RUN_CBC_BATCH(CBC_VarKey_256_KAT_Test_Vectors_06,
                  cbc_varkey_256_kat_vectors_06,
                  cbc_varkey_256_kat_vectors_06_count,
                  "CBC VARKEY 256-06");
#endif
#if defined(CONFIG_AES_CBC_VARKEY_256_07)
    RUN_CBC_BATCH(CBC_VarKey_256_KAT_Test_Vectors_07,
                  cbc_varkey_256_kat_vectors_07,
                  cbc_varkey_256_kat_vectors_07_count,
                  "CBC VARKEY 256-07");
#endif
#if defined(CONFIG_AES_CBC_VARKEY_256_08)
    RUN_CBC_BATCH(CBC_VarKey_256_KAT_Test_Vectors_08,
                  cbc_varkey_256_kat_vectors_08,
                  cbc_varkey_256_kat_vectors_08_count,
                  "CBC VARKEY 256-08");
#endif
#if defined(CONFIG_AES_CBC_VARKEY_256_09)
    RUN_CBC_BATCH(CBC_VarKey_256_KAT_Test_Vectors_09,
                  cbc_varkey_256_kat_vectors_09,
                  cbc_varkey_256_kat_vectors_09_count,
                  "CBC VARKEY 256-09");
#endif
#if defined(CONFIG_AES_CBC_VARKEY_256_10)
    RUN_CBC_BATCH(CBC_VarKey_256_KAT_Test_Vectors_10,
                  cbc_varkey_256_kat_vectors_10,
                  cbc_varkey_256_kat_vectors_10_count,
                  "CBC VARKEY 256-10");
#endif
#if defined(CONFIG_AES_CBC_VARKEY_256_11)
    RUN_CBC_BATCH(CBC_VarKey_256_KAT_Test_Vectors_11,
                  cbc_varkey_256_kat_vectors_11,
                  cbc_varkey_256_kat_vectors_11_count,
                  "CBC VARKEY 256-11");
#endif

    return 0;
}