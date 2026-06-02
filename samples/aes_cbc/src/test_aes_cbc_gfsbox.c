#include "test_aes_helpers.h"
#include "aes128_cbc_gfsbox.h"
#include "aes192_cbc_gfsbox.h"
#include "aes256_cbc_gfsbox.h"

#define DEFINE_CBC_RUNNER(fname, type, arr, cnt, label)                 \
static int fname(const struct device *dev)                              \
{                                                                       \
    uint32_t passed = 0, failed = 0;                                    \
    for (uint32_t i = 0; i < (cnt); i++) {                             \
        const type *tv = &(arr)[i];                                     \
        uint8_t ct[256] = {0};                                          \
        uint8_t pt[256] = {0};                                          \
        uint8_t iv[16];                                                 \
        struct cipher_ctx ctx = {0};                                    \
        ctx.key.bit_stream = (uint8_t *)tv->key;                       \
        ctx.keylen = tv->key_len;                                       \
        ctx.flags = CAP_RAW_KEY | CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS;\
        if (cipher_begin_session(dev, &ctx,                             \
                CRYPTO_CIPHER_ALGO_AES,                                 \
                CRYPTO_CIPHER_MODE_CBC,                                 \
                CRYPTO_CIPHER_OP_ENCRYPT)) {                            \
            printk("[" label " %03d] FAIL (enc session)\n", i);        \
            failed++; continue;                                         \
        }                                                               \
        memcpy(iv, tv->iv, tv->iv_len);                                 \
        for (size_t off = 0; off < tv->pt_len; off += 16) {            \
            struct cipher_pkt pkt = {                                   \
                .in_buf = (uint8_t *)tv->plaintext + off,               \
                .in_len = 16,                                           \
                .out_buf = ct + off,                                    \
                .out_buf_max = 16,                                      \
            };                                                          \
            if (cipher_cbc_op(&ctx, &pkt, iv)) {                       \
                printk("[" label " %03d] FAIL (enc op)\n", i);         \
                failed++;                                               \
                cipher_free_session(dev, &ctx);                         \
                goto next_##fname##_##i;                               \
            }                                                           \
        }                                                               \
        cipher_free_session(dev, &ctx);                                 \
        if (memcmp(ct, tv->ciphertext, tv->ct_len) != 0) {             \
            printk("[" label " %03d] FAIL (enc mismatch)\n", i);       \
            failed++; continue;                                         \
        }                                                               \
        struct cipher_ctx ctx_dec = {0};                                \
        ctx_dec.key.bit_stream = (uint8_t *)tv->key;                   \
        ctx_dec.keylen = tv->key_len;                                   \
        ctx_dec.flags = CAP_RAW_KEY | CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS;\
        if (cipher_begin_session(dev, &ctx_dec,                         \
                CRYPTO_CIPHER_ALGO_AES,                                 \
                CRYPTO_CIPHER_MODE_CBC,                                 \
                CRYPTO_CIPHER_OP_DECRYPT)) {                            \
            printk("[" label " %03d] FAIL (dec session)\n", i);        \
            failed++; continue;                                         \
        }                                                               \
        memcpy(iv, tv->iv, tv->iv_len);                                 \
        for (size_t off = 0; off < tv->ct_len; off += 16) {            \
            struct cipher_pkt pkt = {                                   \
                .in_buf = ct + off,                                     \
                .in_len = 16,                                           \
                .out_buf = pt + off,                                    \
                .out_buf_max = 16,                                      \
            };                                                          \
            if (cipher_cbc_op(&ctx_dec, &pkt, iv)) {                   \
                printk("[" label " %03d] FAIL (dec op)\n", i);         \
                failed++;                                               \
                cipher_free_session(dev, &ctx_dec);                     \
                goto next_##fname##_##i;                               \
            }                                                           \
        }                                                               \
        cipher_free_session(dev, &ctx_dec);                             \
        if (memcmp(pt, tv->plaintext, tv->pt_len) != 0) {              \
            printk("[" label " %03d] FAIL (dec mismatch)\n", i);       \
            failed++;                                                   \
        } else {                                                        \
            printk("[" label " %03d] PASS\n", i);                      \
            passed++;                                                   \
        }                                                               \
        next_##fname##_##i:;                                            \
    }                                                                   \
    printk(label ": %d Passed, %d Failed\n", passed, failed);          \
    return failed ? -EIO : 0;                                           \
}

DEFINE_CBC_RUNNER(run_gfsbox_128,
                  CBC_GFSbox_128_KAT_Test_Vectors,
                  cbc_gfsbox_128_kat_vectors,
                  cbc_gfsbox_128_kat_vectors_count,
                  "CBC GFSbox 128")

DEFINE_CBC_RUNNER(run_gfsbox_192,
                  CBC_GFSbox_192_KAT_Test_Vectors,
                  cbc_gfsbox_192_kat_vectors,
                  cbc_gfsbox_192_kat_vectors_count,
                  "CBC GFSbox 192")

DEFINE_CBC_RUNNER(run_gfsbox_256,
                  CBC_GFSbox_256_KAT_Test_Vectors,
                  cbc_gfsbox_256_kat_vectors,
                  cbc_gfsbox_256_kat_vectors_count,
                  "CBC GFSbox 256")

int test_aes_cbc_gfsbox(void)
{
    const struct device *dev = aes_get_dev();
    if (!dev) return -ENODEV;
    int ret = 0;
    ret |= run_gfsbox_128(dev);
    ret |= run_gfsbox_192(dev);
    ret |= run_gfsbox_256(dev);
    return ret;
}