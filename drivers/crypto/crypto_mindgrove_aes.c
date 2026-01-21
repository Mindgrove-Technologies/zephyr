#define DT_DRV_COMPAT mindgrove_aes

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/crypto/crypto.h>
#include <errno.h>
#include <string.h>
#include "crypto_mindgrove_aes.h"

/* ---- Constants ---- */
#define AES_BLOCK_BITS   128
#define AES_BLOCK_BYTES  16
#define BYTE_LENGTH      8

#define AES_ECB  0
#define AES_CBC  1
#define AES_CTR  4

#define AES_ENC  0
#define AES_DEC  1



static AES_Type *aes_reg;

/* ---- Session ---- */
struct mindgrove_session {
    uint8_t *key;
    uint16_t key_bits;
    uint8_t encrypt;
    uint32_t iterated_bits;
};

/* ---- Low-level helpers ---- */

static void input_text_to_aes(uint8_t *input_text)
{
    for (int i = 0; i < 2; i++) {
        uint64_t v = 0;

        for (int j = 0; j < 8; j++) {
            v = (v << 8) | *(input_text++);
        }

        printk("AES_INPUT[%d] write = 0x%016llx\n",
               i, (unsigned long long)v);

        aes_reg->AES_INPUT = v;

        /* Readback probe */
        uint64_t rb = aes_reg->AES_INPUT;
        printk("AES_INPUT[%d] read  = 0x%016llx\n",
               i, (unsigned long long)rb);
    }
}


static void input_key_to_aes(uint8_t *key, int hex_key_len)
{
    int key_len_mode = (int)(hex_key_len >> 1);

    printk("KEY zero fill count = %d\n", (2 - key_len_mode));

    for (int i = 0; i < (2 - key_len_mode); i++) {
        aes_reg->AES_KEY = 0;
        printk("AES_KEY write = 0x0000000000000000\n");
    }

    for (int i = 0; i < (4 - (2 - key_len_mode)); i++) {
        uint64_t v = 0;

        for (int j = 0; j < 8; j++) {
            v = (v << 8) | *(key++);
        }

        printk("AES_KEY[%d] write = 0x%016llx\n",
               i, (unsigned long long)v);

        aes_reg->AES_KEY = v;

        uint64_t rb = aes_reg->AES_KEY;
        printk("AES_KEY[%d] read  = 0x%016llx\n",
               i, (unsigned long long)rb);
    }
}

static void input_iv_to_aes(uint8_t *iv)
{
    for (int i = 0; i < 2; i++) {
        uint64_t v = 0;

        for (int j = 0; j < 8; j++) {
            v = (v << 8) | *(iv++);
        }

        printk("AES_IV[%d] write = 0x%016llx\n",
               i, (unsigned long long)v);

        aes_reg->AES_IV = v;

        uint64_t rb = aes_reg->AES_IV;
        printk("AES_IV[%d] read  = 0x%016llx\n",
               i, (unsigned long long)rb);
    }
}


static void get_output(uint8_t *out)
{
    uint64_t a = aes_reg->AES_OUTPUT;
    uint64_t b = aes_reg->AES_OUTPUT;

    for (int i = 7; i >= 0; i--) {
        *(out++) = a >> (8 * i);
    }
    for (int i = 7; i >= 0; i--) {
        *(out++) = b >> (8 * i);
    }
}

static int do_checks_params(int key_len_bits, int mode)
{
    if ((key_len_bits != 128) &&
        (key_len_bits != 192) &&
        (key_len_bits != 256)) {
        return -EINVAL;
    }

    if ((mode < 0) || (mode > 4)) {
        return -EINVAL;
    }

    return 0;
}

/* ---- AES core ---- */

uint32_t AES_Run(uint8_t *out,
                 uint8_t *in,
                 uint8_t *key,
                 uint8_t *iv,
                 uint32_t input_len_bits,
                 uint32_t key_len_bits,
                 int mode,
                 int encrypt,
                 uint32_t iterated_bits)
{

    printk("\n=== AES_Run ENTER ===\n");
    printk("input_len_bits = %u\n", input_len_bits);
    printk("key_len_bits   = %u\n", key_len_bits);
    printk("mode           = %d\n", mode);
    printk("encrypt        = %d\n", encrypt);
    printk("iterated_bits  = %u\n", iterated_bits);

    /* Dump plaintext */
    printk("PLAINTEXT:\n");
    for (int i = 0; i < 16; i++) {
        printk("%02x ", in[i]);
    }
    printk("\n");

    /* Dump key */
    printk("KEY:\n");
    for (int i = 0; i < key_len_bits / 8; i++) {
        printk("%02x ", key[i]);
    }
    printk("\n");

    /* Dump IV if present */
    if (iv) {
        printk("IV:\n");
        for (int i = 0; i < 16; i++) {
            printk("%02x ", iv[i]);
        }
        printk("\n");
    }

    int rc = do_checks_params(key_len_bits, mode);
    if (rc) {
        return rc;
    }

    uint8_t hex_key_len;
    if (key_len_bits == 128) {
        hex_key_len = 0x00;
    } else if (key_len_bits == 192) {
        hex_key_len = 0x02;
    } else {
        hex_key_len = 0x04;
    }

    if (iterated_bits == 0) {
        uint8_t hex_mode = (uint8_t)mode << 3;
        uint8_t cfg = encrypt | hex_key_len | hex_mode | 0x40;
        aes_reg->AES_CTRL = 0;
        aes_reg->AES_CTRL = cfg;
    }

    int blocks = input_len_bits / AES_BLOCK_BITS;

    for (int i = 0; i < blocks; i++) {
        if ((i == 0) && (iterated_bits == 0)) {
            input_key_to_aes(key, hex_key_len);
            if (mode != AES_ECB) {
                input_iv_to_aes(iv);
            }
            input_text_to_aes(in);
        } else {
            input_text_to_aes(in + (i * AES_BLOCK_BYTES));
        }

        // while (!(aes_reg->AES_STATUS & 0x2)) {
        //     ;
        // }

        get_output(out + (i * AES_BLOCK_BYTES));

        memcpy(out + (i * AES_BLOCK_BYTES),
       in + (i * AES_BLOCK_BYTES),
       AES_BLOCK_BYTES);
    }   

    return 0;
}

/* ---- Cipher handlers ---- */

static int ecb_crypt(struct cipher_ctx *ctx, struct cipher_pkt *pkt)
{
    struct mindgrove_session *sess = ctx->drv_sessn_state;
    printk("ECB REACHED \n");
    int rc = AES_Run(pkt->out_buf,
                     pkt->in_buf,
                     sess->key,
                     NULL,
                     pkt->in_len * 8,
                     sess->key_bits,
                     AES_ECB,
                     sess->encrypt,
                     sess->iterated_bits);

    if (!rc) {
        sess->iterated_bits += pkt->in_len * 8;
    }

    pkt->out_len = pkt->in_len;
    return rc;
}

static int cbc_crypt(struct cipher_ctx *ctx,
                     struct cipher_pkt *pkt,
                     uint8_t *iv)
{
    struct mindgrove_session *sess = ctx->drv_sessn_state;

    int rc = AES_Run(pkt->out_buf,
                     pkt->in_buf,
                     sess->key,
                     iv,
                     pkt->in_len * 8,
                     sess->key_bits,
                     AES_CBC,
                     sess->encrypt,
                     sess->iterated_bits);

    if (!rc) {
        sess->iterated_bits += pkt->in_len * 8;
    }

    pkt->out_len = pkt->in_len;
    return rc;
}

static int ctr_crypt(struct cipher_ctx *ctx,
                     struct cipher_pkt *pkt,
                     uint8_t *iv)
{
    struct mindgrove_session *sess = ctx->drv_sessn_state;

    int rc = AES_Run(pkt->out_buf,
                     pkt->in_buf,
                     sess->key,
                     iv,
                     pkt->in_len * 8,
                     sess->key_bits,
                     AES_CTR,
                     sess->encrypt,
                     sess->iterated_bits);

    if (!rc) {
        sess->iterated_bits += pkt->in_len * 8;
    }

    pkt->out_len = pkt->in_len;
    return rc;
}

/* ---- Session management ---- */

static int begin_session(const struct device *dev,
                         struct cipher_ctx *ctx,
                         enum cipher_algo algo,
                         enum cipher_mode mode,
                         enum cipher_op op)
{
    if (algo != CRYPTO_CIPHER_ALGO_AES) {
        return -ENOTSUP;
    }

    struct mindgrove_session *sess = k_malloc(sizeof(*sess));
    if (!sess) {
        return -ENOMEM;
    }

    sess->key = ctx->key.bit_stream;
    sess->key_bits = ctx->keylen * 8;
    sess->iterated_bits = 0;
    sess->encrypt = (op == CRYPTO_CIPHER_OP_ENCRYPT) ? AES_ENC : AES_DEC;

    ctx->drv_sessn_state = sess;

    switch (mode) {
    case CRYPTO_CIPHER_MODE_ECB:
        ctx->ops.block_crypt_hndlr = ecb_crypt;
        break;
    case CRYPTO_CIPHER_MODE_CBC:
        ctx->ops.cbc_crypt_hndlr = cbc_crypt;
        break;
    case CRYPTO_CIPHER_MODE_CTR:
        ctx->ops.ctr_crypt_hndlr = ctr_crypt;
        break;
    default:
        k_free(sess);
        return -ENOTSUP;
    }

    return 0;
}

static int free_session(const struct device *dev,
                        struct cipher_ctx *ctx)
{
    struct mindgrove_session *sess = ctx->drv_sessn_state;

    if (sess) {
        memset(sess, 0, sizeof(*sess));
        k_free(sess);
        ctx->drv_sessn_state = NULL;
    }

    return 0;
}

static int query_caps(const struct device *dev)
{
    return CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS | CAP_RAW_KEY;
}

/* ---- Init ---- */

static int aes_init(const struct device *dev)
{
    printk("MindGrove AES init called\n");
    printk("DT_INST_REG_ADDR(0) = %lx\n", (long unsigned)DT_INST_REG_ADDR(0));

    const struct mindgrove_aes_config *cfg = dev->config;
    aes_reg = cfg->aes_reg;

    printk("AES reg base = %p\n", aes_reg);

    if (!aes_reg) {
        printk("AES device not ready!\n");
        return -ENODEV;
    }

    return 0;
}


/* ---- API ---- */

static const struct crypto_driver_api api = {
    .cipher_begin_session = begin_session,
    .cipher_free_session  = free_session,
    .query_hw_caps        = query_caps,
};



#define MINDGROVE_AES_INIT(n) \
    static const struct mindgrove_aes_config aes_cfg_##n = { \
        .aes_reg = (AES_Type *)DT_INST_REG_ADDR(n), \
    }; \
    DEVICE_DT_INST_DEFINE(n, \
                          aes_init, \
                          NULL, \
                          NULL, \
                          &aes_cfg_##n, \
                          PRE_KERNEL_1, \
                          CONFIG_CRYPTO_INIT_PRIORITY, \
                          &api);

DT_INST_FOREACH_STATUS_OKAY(MINDGROVE_AES_INIT)
