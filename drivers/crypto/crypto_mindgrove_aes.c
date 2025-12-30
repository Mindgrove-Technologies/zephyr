/*
 * Mindgrove AES Zephyr Crypto Driver
 *
 * - Single-shot AES only
 * - Polling-based (no interrupts)
 * - Thread-safe via mutex
 * - No async / PSA / secure key storage
 */

 #include <zephyr/kernel.h>
 #include <zephyr/device.h>
 #include <zephyr/drivers/crypto.h>
 #include <string.h>
 #include <errno.h>
 
 #include "aes.h"
 #include "crypto_mindgrove_aes.h"
 
 /* ========================= */
 /* Constants / enums         */
 /* ========================= */
 
 #define AES_BLOCK_BITS        128
 #define AES_BLOCK_BYTES       16
 
 #define AES_ECB               0
 #define AES_CBC               1
 #define AES_CFB               2
 #define AES_OFB               3
 #define AES_CTR               4
 
 #define AES_ENCRYPT           0
 #define AES_DECRYPT           1
 
 #define SUCCESS               0
 
 /* ========================= */
 /* Driver runtime state      */
 /* ========================= */
 
 struct mindgrove_aes_drvdata {
     struct k_mutex lock;
 };
 
 static struct mindgrove_aes_drvdata aes_drv;
 
 /* ========================= */
 /* Hardware base pointer     */
 /* ========================= */
 
 static AES_Type *aes_instance;
 
 /* ========================= */
 /* Low-level HW helpers      */
 /* ========================= */
 
 static void input_text_to_aes(uint8_t *input)
 {
     for (int i = 0; i < 2; i++) {
         uint64_t val = 0;
         for (int j = 0; j < 8; j++) {
             val = (val << 8) | *input++;
         }
         aes_instance->AES_INPUT = val;
     }
 }
 
 static void input_key_to_aes(uint8_t *key, uint8_t hex_key_len)
 {
     uint32_t key_len_mode = hex_key_len >> 1U;
     int blank = 0;
 
     for (; blank < (2 - (int)key_len_mode); blank++) {
         aes_instance->AES_KEY = 0;
     }
 
     for (int i = 0; i < (4 - blank); i++) {
         uint64_t val = 0;
         for (int j = 0; j < 8; j++) {
             val = (val << 8) | *key++;
         }
         aes_instance->AES_KEY = val;
     }
 }
 
 static void input_iv_to_aes(uint8_t *iv)
 {
     for (int i = 0; i < 2; i++) {
         uint64_t val = 0;
         for (int j = 0; j < 8; j++) {
             val = (val << 8) | *iv++;
         }
         aes_instance->AES_IV = val;
     }
 }
 
 static void get_output(uint8_t *out)
 {
     uint64_t a = aes_instance->AES_OUTPUT;
     uint64_t b = aes_instance->AES_OUTPUT;
 
     for (int i = 7; i >= 0; i--) {
         *out++ = a >> (8 * i);
     }
     for (int i = 7; i >= 0; i--) {
         *out++ = b >> (8 * i);
     }
 }
 
 /* ========================= */
 /* AES single-shot runner    */
 /* ========================= */
 
 static uint16_t AES_Run(uint8_t *out,
             uint8_t *in,
             uint8_t *key,
             uint8_t *iv,
             uint32_t in_len_bits,
             uint32_t key_len_bits,
             int mode,
             int enc)
 {
     int blocks = in_len_bits / AES_BLOCK_BITS;
     uint8_t hex_key_len;
 
     if (key_len_bits == 128)      hex_key_len = 0x00;
     else if (key_len_bits == 192) hex_key_len = 0x02;
     else if (key_len_bits == 256) hex_key_len = 0x04;
     else return -EINVAL;
 
     uint8_t ctrl =
         (enc & 0x1) |
         hex_key_len |
         ((uint8_t)mode << 3) |
         0x40;
 
     aes_instance->AES_CTRL = 0;
     aes_instance->AES_CTRL = ctrl;
 
     for (int i = 0; i < blocks; i++) {
 
         if (i == 0) {
             input_key_to_aes(key, hex_key_len);
             if (mode != AES_ECB) {
                 input_iv_to_aes(iv);
             }
         }
 
         input_text_to_aes(&in[i * AES_BLOCK_BYTES]);
 
         while (!(aes_instance->AES_STATUS & 0x2)) {
             ;
         }
 
         get_output(&out[i * AES_BLOCK_BYTES]);
     }
 
     return SUCCESS;
 }
 
 /* ========================= */
 /* Zephyr session object     */
 /* ========================= */
 
 struct mindgrove_aes_session {
     uint8_t *key;           /* Owned by Zephyr */
     uint16_t keylen_bits;
     uint8_t encrypt;
 };
 
 /* ========================= */
 /* Cipher handlers           */
 /* ========================= */
 
 static int mindgrove_aes_ecb(struct cipher_ctx *ctx,
                 struct cipher_pkt *pkt)
 {
     struct mindgrove_aes_session *s = ctx->drv_sessn_state;
 
     k_mutex_lock(&aes_drv.lock, K_FOREVER);
 
     uint16_t ret = AES_Run(pkt->out_buf,
                    pkt->in_buf,
                    s->key,
                    NULL,
                    pkt->in_len * 8,
                    s->keylen_bits,
                    AES_ECB,
                    s->encrypt);
 
     k_mutex_unlock(&aes_drv.lock);
 
     if (ret != SUCCESS) {
         return -EIO;
     }
 
     pkt->out_len = pkt->in_len;
     return 0;
 }
 
 static int mindgrove_aes_cbc(struct cipher_ctx *ctx,
                 struct cipher_pkt *pkt,
                 uint8_t *iv)
 {
     struct mindgrove_aes_session *s = ctx->drv_sessn_state;
 
     if (!iv) {
         return -EINVAL;
     }
 
     k_mutex_lock(&aes_drv.lock, K_FOREVER);
 
     uint16_t ret = AES_Run(pkt->out_buf,
                    pkt->in_buf,
                    s->key,
                    iv,
                    pkt->in_len * 8,
                    s->keylen_bits,
                    AES_CBC,
                    s->encrypt);
 
     k_mutex_unlock(&aes_drv.lock);
 
     if (ret != SUCCESS) {
         return -EIO;
     }
 
     pkt->out_len = pkt->in_len;
     return 0;
 }
 
 static int mindgrove_aes_cfb(struct cipher_ctx *ctx,
                 struct cipher_pkt *pkt,
                 uint8_t *iv)
 {
     struct mindgrove_aes_session *s = ctx->drv_sessn_state;
 
     if (!iv) {
         return -EINVAL;
     }
 
     k_mutex_lock(&aes_drv.lock, K_FOREVER);
 
     uint16_t ret = AES_Run(pkt->out_buf,
                    pkt->in_buf,
                    s->key,
                    iv,
                    pkt->in_len * 8,
                    s->keylen_bits,
                    AES_CFB,
                    s->encrypt);
 
     k_mutex_unlock(&aes_drv.lock);
 
     if (ret != SUCCESS) {
         return -EIO;
     }
 
     pkt->out_len = pkt->in_len;
     return 0;
 }
 
 static int mindgrove_aes_ofb(struct cipher_ctx *ctx,
                 struct cipher_pkt *pkt,
                 uint8_t *iv)
 {
     struct mindgrove_aes_session *s = ctx->drv_sessn_state;
 
     if (!iv) {
         return -EINVAL;
     }
 
     k_mutex_lock(&aes_drv.lock, K_FOREVER);
 
     uint16_t ret = AES_Run(pkt->out_buf,
                    pkt->in_buf,
                    s->key,
                    iv,
                    pkt->in_len * 8,
                    s->keylen_bits,
                    AES_OFB,
                    s->encrypt);
 
     k_mutex_unlock(&aes_drv.lock);
 
     if (ret != SUCCESS) {
         return -EIO;
     }
 
     pkt->out_len = pkt->in_len;
     return 0;
 }
 
 static int mindgrove_aes_ctr(struct cipher_ctx *ctx,
                 struct cipher_pkt *pkt,
                 uint8_t *iv)
 {
     struct mindgrove_aes_session *s = ctx->drv_sessn_state;
 
     if (!iv) {
         return -EINVAL;
     }
 
     k_mutex_lock(&aes_drv.lock, K_FOREVER);
 
     uint16_t ret = AES_Run(pkt->out_buf,
                    pkt->in_buf,
                    s->key,
                    iv,
                    pkt->in_len * 8,
                    s->keylen_bits,
                    AES_CTR,
                    s->encrypt);
 
     k_mutex_unlock(&aes_drv.lock);
 
     if (ret != SUCCESS) {
         return -EIO;
     }
 
     pkt->out_len = pkt->in_len;
     return 0;
 }
 
 /* ========================= */
 /* Session management        */
 /* ========================= */
 
 static int mindgrove_aes_begin_session(const struct device *dev,
                        struct cipher_ctx *ctx,
                        enum cipher_algo algo,
                        enum cipher_mode mode,
                        enum cipher_op op)
 {
     if (algo != CRYPTO_CIPHER_ALGO_AES) {
         return -ENOTSUP;
     }
 
     struct mindgrove_aes_session *s = k_malloc(sizeof(*s));
     if (!s) {
         return -ENOMEM;
     }
 
     s->key = ctx->key.bit_stream;
     s->keylen_bits = ctx->keylen * 8;
     s->encrypt = (op == CRYPTO_CIPHER_OP_ENCRYPT);
 
     ctx->drv_sessn_state = s;
 
     switch (mode) {
     case CRYPTO_CIPHER_MODE_ECB:
         ctx->ops.block_crypt_hndlr = mindgrove_aes_ecb;
         break;
     case CRYPTO_CIPHER_MODE_CBC:
         ctx->ops.cbc_crypt_hndlr = mindgrove_aes_cbc;
         break;
     case CRYPTO_CIPHER_MODE_CFB:
         ctx->ops.cbc_crypt_hndlr = mindgrove_aes_cfb;
         break;
     case CRYPTO_CIPHER_MODE_OFB:
         ctx->ops.cbc_crypt_hndlr = mindgrove_aes_ofb;
         break;
     case CRYPTO_CIPHER_MODE_CTR:
         ctx->ops.ctr_crypt_hndlr = mindgrove_aes_ctr;
         break;
     default:
         k_free(s);
         return -ENOTSUP;
     }
 
     return 0;
 }
 
 static int mindgrove_aes_free_session(const struct device *dev,
                      struct cipher_ctx *ctx)
 {
     k_free(ctx->drv_sessn_state);
     ctx->drv_sessn_state = NULL;
     return 0;
 }
 
 /* ========================= */
 /* Driver init               */
 /* ========================= */
 
 static int mindgrove_aes_init(const struct device *dev)
 {
     const struct mindgrove_aes_config *cfg = dev->config;
 
     aes_instance = cfg->base;
     if (!aes_instance) {
         return -ENODEV;
     }
 
     k_mutex_init(&aes_drv.lock);
 
     aes_instance->AES_CTRL = 0;
     return 0;
 }
 
 /* ========================= */
 /* API + instantiation       */
 /* ========================= */
 
 static const struct crypto_driver_api mindgrove_crypto_api = {
     .begin_session = mindgrove_aes_begin_session,
     .free_session  = mindgrove_aes_free_session,
 };
 
 #define DT_DRV_COMPAT mindgrove_aes
 
 #define MINDGROVE_AES_INIT(n)                                \
     static const struct mindgrove_aes_config                 \
     mindgrove_aes_cfg_##n = {                                 \
         .base = (AES_Type *)DT_INST_REG_ADDR(n),              \
     };                                                        \
     DEVICE_DT_INST_DEFINE(n,                                  \
                   mindgrove_aes_init,                          \
                   NULL,                                        \
                   NULL,                                        \
                   &mindgrove_aes_cfg_##n,                      \
                   POST_KERNEL,                                 \
                   CONFIG_CRYPTO_INIT_PRIORITY,                 \
                   &mindgrove_crypto_api);
 
 DT_INST_FOREACH_STATUS_OKAY(MINDGROVE_AES_INIT)
 