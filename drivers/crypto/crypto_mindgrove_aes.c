#define DT_DRV_COMPAT mindgrove_aes

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/crypto/crypto.h>
#include <zephyr/crypto/cipher.h>
#include <errno.h>
#include <string.h>
#include "crypto_mindgrove_aes.h"

/* ---- Constants ---- */
#define AES_BLOCK_BITS  128
#define AES_BLOCK_BYTES 16
#define BYTE_LENGTH     8

#define AES_ECB 0
#define AES_CBC 1
#define AES_CFB 2
#define AES_OFB 3
#define AES_CTR 4

#define AES_ENC 0
#define AES_DEC 1

static volatile AES_Type *aes_reg;


/* ---- Session ---- */
struct mindgrove_session {
	uint8_t *key;
	uint8_t iv[16]; /* base IV */
	uint16_t key_bits;
	uint32_t iterated_bits;
	uint8_t encrypt;
};

/* ---- Low-level helpers ---- */

static void input_text_to_aes(uint8_t *input_text)
{

	for (int i = 0; i < 2; i++) {
		uint64_t v = 0;

		for (int j = 0; j < 8; j++) {
			v = (v << 8) | *(input_text++);
		}


		aes_reg->AES_INPUT = v;
	}
}

static void input_key_to_aes(unsigned char *key, int hex_key_len)
{

	uint32_t key_len_mode = (unsigned int)hex_key_len >> 1U;
	int blank = 0;

	for (; blank < (2 - (int)key_len_mode); blank++) {
		aes_reg->AES_KEY = 0;
	}

	for (int i = 0; i < (4 - blank); i++) {
		uint64_t v = 0;

		for (int j = 0; j < 8; j++) {
			v = (v << 8) | *key++;
		}


		aes_reg->AES_KEY = v;
	}
}

static void input_iv_to_aes(uint8_t *iv)
{
	for (int i = 0; i < 2; i++) {
		uint64_t v = 0;

		for (int j = 0; j < 8; j++) {
			v = (v << 8) | *(iv++);
		}


		aes_reg->AES_IV = v;
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
	if ((key_len_bits != 128) && (key_len_bits != 192) && (key_len_bits != 256)) {
		return -EINVAL;
	}

	if ((mode < 0) || (mode > 4)) {
		return -EINVAL;
	}

	return 0;
}

/* ---- AES core ---- */

uint32_t AES_Run(uint8_t *out, uint8_t *in, uint8_t *key, uint8_t *iv, uint32_t input_len_bits,
		 uint32_t key_len_bits, int mode, int encrypt, uint32_t iterated_bits)
{
	int offset = 0;
	size_t output_offset = 0;

	int number_of_blocks;

	if (!out || !in || !key) {
		return -EINVAL;
	}

	if ((mode != AES_ECB) && !iv) {
		return -EINVAL;
	}

	int rc = do_checks_params(key_len_bits, mode);
	if (rc) {
		return rc;
	}
	unsigned char *output_start = out;
	unsigned char *current_output = out;

	if ((input_len_bits % AES_BLOCK_BITS) != 0) {
		return -EINVAL;
	}

	unsigned char hex_key_len;
	if (key_len_bits == 128) {
		hex_key_len = 0x00;
	} else if (key_len_bits == 192) {
		hex_key_len = 0x02;
	} else {
		hex_key_len = 0x04;
	}

	/* ---- Configure AES only once per session ---- */
	if (iterated_bits == 0U) {
		uint8_t hex_mode = (uint8_t)mode << 3;
		uint8_t hex_config = (uint8_t)encrypt | hex_key_len | hex_mode | 0x40; /* START */

		aes_reg->AES_CTRL &= 0;
		aes_reg->AES_CTRL |= hex_config;
	}
	int blocks = input_len_bits / AES_BLOCK_BITS;

	for (int block_index = 0; block_index < blocks; block_index++) {
		uint8_t *current_out = &output_start[output_offset];

		if (block_index == 0 && iterated_bits == 0U) {
			input_key_to_aes(key, hex_key_len);

			if (mode == AES_ECB) {
				input_iv_to_aes(in); // input
			} else {
				input_iv_to_aes(iv); // base IV for first block
			}
			input_text_to_aes(in);

		} else {
			offset = ((block_index * 128) / BYTE_LENGTH);

			input_text_to_aes(&in[offset]);
		}
		while (!(aes_reg->AES_STATUS & 0x2U)) {
			; // spin
		}

		get_output(current_out);
		output_offset += AES_BLOCK_BYTES;
	}

	return 0;
}

/* ---- Cipher handlers ---- */

static int ecb_crypt(struct cipher_ctx *ctx, struct cipher_pkt *pkt)
{
	struct mindgrove_session *sess = ctx->drv_sessn_state;


	if (pkt->in_len != AES_BLOCK_BYTES) {
		return -EINVAL;
	}
	

	int rc = AES_Run(pkt->out_buf, pkt->in_buf, sess->key, NULL, /* zero IV */
			 pkt->in_len * 8, sess->key_bits, AES_ECB, sess->encrypt,
			 sess->iterated_bits);

	if (!rc) {
		sess->iterated_bits += pkt->in_len * 8;
	}

	pkt->out_len = pkt->in_len;
	return rc;
}

static int cbc_crypt(struct cipher_ctx *ctx, struct cipher_pkt *pkt, uint8_t *iv)
{
	struct mindgrove_session *s = ctx->drv_sessn_state;

	uint8_t iv_local[16];

	if (s->iterated_bits == 0) {
		memcpy(s->iv, iv, 16); // Save base IV for first run
	}

	// memcpy(iv_local, s->iv, 16);  // Start with previous IV

	int rc = AES_Run(pkt->out_buf, pkt->in_buf, s->key, s->iv, pkt->in_len * 8, s->key_bits,
			 AES_CBC, s->encrypt, s->iterated_bits);

	if (rc) {
		return rc;
	}

	// Update iterated bits
	s->iterated_bits += pkt->in_len * 8;
	//printk("bits inside handler %u\n", s->iterated_bits);
	

	pkt->out_len = pkt->in_len;
	return 0;
}

static int ctr_crypt(struct cipher_ctx *ctx, struct cipher_pkt *pkt, uint8_t *iv)
{
	struct mindgrove_session *s = ctx->drv_sessn_state;

	/* Save base IV for first run */
	if (s->iterated_bits == 0) {
		memcpy(s->iv, iv, AES_BLOCK_BYTES);
	}

	/* Call AES_Run with hardware handling the counter */
	int rc = AES_Run(pkt->out_buf, pkt->in_buf, s->key,
			 s->iv,           // IV hardware will increment
			 pkt->in_len * 8, // input length in bits
			 s->key_bits, AES_CTR, s->encrypt,
			 s->iterated_bits); // pass iterated_bits for AES session tracking

	if (rc) {
		return rc;
	}

	/* Update iterated_bits in bytes */
	s->iterated_bits += pkt->in_len * 8;

	pkt->out_len = pkt->in_len;
	return 0;
}

/* ---- Session management ---- */

static int begin_session(const struct device *dev, struct cipher_ctx *ctx, enum cipher_algo algo,
			 enum cipher_mode mode, enum cipher_op op)
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
	memset(sess->iv, 0, AES_BLOCK_BYTES);

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

static int free_session(const struct device *dev, struct cipher_ctx *ctx)
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
	

	const struct mindgrove_aes_config *cfg = dev->config;
	aes_reg = cfg->aes_reg;


	if (!aes_reg) {
		return -ENODEV;
	}

	return 0;
}

/* ---- API ---- */

static const struct crypto_driver_api api = {
	.cipher_begin_session = begin_session,
	.cipher_free_session = free_session,
	.query_hw_caps = query_caps,
};

#define MINDGROVE_AES_INIT(n)                                                                      \
	static const struct mindgrove_aes_config aes_cfg_##n = {                                   \
		.aes_reg = (AES_Type *)DT_INST_REG_ADDR(n),                                        \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, aes_init, NULL, NULL, &aes_cfg_##n, PRE_KERNEL_1,                 \
			      CONFIG_CRYPTO_INIT_PRIORITY, &api);

DT_INST_FOREACH_STATUS_OKAY(MINDGROVE_AES_INIT)
