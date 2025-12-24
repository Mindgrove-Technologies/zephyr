/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/crypto/crypto.h>
#include <string.h>
#include <errno.h>

#include "crypto_mindgrove_sha_priv.h"

/* ============================= */
/* Constants                     */
/* ============================= */

#define SHA_BITS_PER_BYTE        8
#define SHA256_BLOCK_BITS        512
#define SHA256_HASH_LEN          32
#define SHA256_LENGTH_BITS       64

/* ============================= */
/* Helper functions (unchanged)  */
/* ============================= */

static void input_text_to_sha(volatile struct mindgrove_sha256_regs *regs,
			      unsigned char **final_sha_text_dict,
			      int block_message_length_bits,
			      int sha_append_length_bits,
			      int mode)
{
	if (mode == 0) {
		unsigned char *p = final_sha_text_dict[0];

		for (int i = 0; i < 8; i++) {
			uint64_t v = 0;
			for (int j = 0; j < 8; j++) {
				v = (v << 8) | *p++;
			}
			regs->SHA_INPUT = v;
		}
	} else {
		unsigned char *p = final_sha_text_dict[0];
		int tracker = 0;

		for (int i = 0; i < 8; i++) {
			uint64_t v = 0;
			for (int j = 0; j < 8; j++) {
				v = (v << 8) | *p++;
				tracker++;

				if (tracker ==
				    block_message_length_bits / SHA_BITS_PER_BYTE) {
					p = final_sha_text_dict[1];
				}
			}
			regs->SHA_INPUT = v;
		}
	}
}

static short int get_sha_append_length_bits(int final_block_message_length_bits)
{
	int pad_bits = SHA256_BLOCK_BITS -
		       (final_block_message_length_bits + SHA256_LENGTH_BITS);

	if (pad_bits < 8) {
		return (2 * SHA256_BLOCK_BITS) - final_block_message_length_bits;
	}

	return pad_bits + SHA256_LENGTH_BITS;
}

static void get_sha_append_bits_helper(unsigned char *buf,
				       int append_bits,
				       long int total_len_bits)
{
	int pad_bytes =
		((append_bits - SHA256_LENGTH_BITS) / SHA_BITS_PER_BYTE) - 1;

	buf[0] = 0x80;
	for (int i = 1; i <= pad_bytes; i++) {
		buf[i] = 0x00;
	}

	int idx = (append_bits / SHA_BITS_PER_BYTE) - 1;
	for (int i = 0; i < SHA256_LENGTH_BITS / 8; i++) {
		buf[idx - i] = (uint8_t)(total_len_bits >> (8 * i));
	}
}

static unsigned char *get_sha_append_bits(long int total_len_bits,
					  short int append_bits)
{
	unsigned char *buf =
		k_calloc(append_bits / SHA_BITS_PER_BYTE, 1);

	get_sha_append_bits_helper(buf, append_bits, total_len_bits);
	return buf;
}

static void sha256_read_output(volatile struct mindgrove_sha256_regs *regs,
			       uint8_t *out)
{
	uint64_t tmp[4];

	for (int i = 0; i < 4; i++) {
		tmp[i] = regs->SHA_OUTPUT;
	}

	for (int i = 0; i < 4; i++) {
		for (int j = 7; j >= 0; j--) {
			*out++ = tmp[i] >> (8 * j);
		}
	}
}

/* ============================= */
/* Zephyr crypto callbacks       */
/* ============================= */

static int mindgrove_sha_begin_session(const struct device *dev,
				       struct hash_ctx *ctx,
				       enum hash_algo algo)
{
	struct mindgrove_sha_dev_data *data = dev->data;

	if (algo != CRYPTO_HASH_ALGO_SHA256) {
		return -ENOTSUP;
	}

	if (data->in_use) {
		return -EBUSY;
	}

	data->in_use = true;
	data->iterated_length_bits = 0;

	ctx->drv_sessn_state = data;
	ctx->hash_hndlr = mindgrove_sha_hash;

	return 0;
}

static int mindgrove_sha_free_session(const struct device *dev,
				      struct hash_ctx *ctx)
{
	struct mindgrove_sha_dev_data *data = ctx->drv_sessn_state;

	data->in_use = false;
	data->iterated_length_bits = 0;
	ctx->drv_sessn_state = NULL;

	return 0;
}

/* ============================= */
/* UPDATE (streaming blocks)     */
/* ============================= */

static int mindgrove_sha_update(const struct device *dev,
				struct hash_ctx *ctx,
				struct hash_pkt *pkt)
{
	struct mindgrove_sha_dev_data *data = ctx->drv_sessn_state;
	volatile struct mindgrove_sha256_regs *regs = data->regs;

	long int bits = pkt->in_len * SHA_BITS_PER_BYTE;

	if (bits != SHA256_BLOCK_BITS) {
		return -EINVAL;
	}

	if (data->iterated_length_bits == 0) {
		while (regs->SHA_STATUS & 1) {
			;
		}
	}

	unsigned char *blk[1] = { pkt->in_buf };
	input_text_to_sha(regs, blk, 1, 1, 0);

	while (!(regs->SHA_STATUS & 2)) {
		;
	}

	regs->SHA_CTRL = 1;
	data->iterated_length_bits += SHA256_BLOCK_BITS;

	return 0;
}

/* ============================= */
/* FINAL                         */
/* ============================= */

static int mindgrove_sha_final(const struct device *dev,
			       struct hash_ctx *ctx,
			       struct hash_pkt *pkt)
{
	struct mindgrove_sha_dev_data *data = ctx->drv_sessn_state;
	volatile struct mindgrove_sha256_regs *regs = data->regs;

	long int msg_bits = pkt->in_len * SHA_BITS_PER_BYTE;
	long int total_bits = data->iterated_length_bits + msg_bits;

	short int append_bits = get_sha_append_length_bits(msg_bits);
	unsigned char *append = get_sha_append_bits(total_bits, append_bits);

	if (append_bits <= SHA256_BLOCK_BITS) {
		unsigned char *blk[2] = { pkt->in_buf, append };
		input_text_to_sha(regs, blk, msg_bits, append_bits, 1);
		while (!(regs->SHA_STATUS & 2)) {
			;
		}
	} else {
		unsigned char *blk[2] = { pkt->in_buf, append };
		input_text_to_sha(regs, blk, msg_bits,
				  append_bits - SHA256_BLOCK_BITS, 1);

		while (!(regs->SHA_STATUS & 2)) {
			;
		}

		regs->SHA_CTRL = 1;

		blk[0] = append +
			 (append_bits - SHA256_BLOCK_BITS) /
				 SHA_BITS_PER_BYTE;

		input_text_to_sha(regs, blk, 1, 1, 0);
		while (!(regs->SHA_STATUS & 2)) {
			;
		}
	}

	sha256_read_output(regs, pkt->out_buf);

	regs->SHA_CTRL = 0;
	data->iterated_length_bits = 0;

	k_free(append);
	return 0;
}

/* ============================= */
/* Dispatcher                    */
/* ============================= */

static int mindgrove_sha_hash(const struct device *dev,
			      struct hash_ctx *ctx,
			      struct hash_pkt *pkt)
{
	if (pkt->out_buf == NULL) {
		return mindgrove_sha_update(dev, ctx, pkt);
	}

	return mindgrove_sha_final(dev, ctx, pkt);
}

/* ============================= */
/* Driver API                    */
/* ============================= */

static const struct crypto_driver_api mindgrove_crypto_api = {
	.hash_begin_session = mindgrove_sha_begin_session,
	.hash_free_session  = mindgrove_sha_free_session,
	.hash_async_callback_set = NULL,
    .query_hw_caps = NULL,
};

/* ============================= */
/* Device instantiation          */
/* ============================= */

// static struct mindgrove_sha_dev_data sha_data = {
// 	.regs = (struct mindgrove_sha256_regs *)DT_INST_REG_ADDR(0),
// 	.in_use = false,
// };

// DEVICE_DT_INST_DEFINE(0,
// 		      NULL,
// 		      NULL,
// 		      &sha_data,
// 		      NULL,
// 		      POST_KERNEL,
// 		      CONFIG_CRYPTO_INIT_PRIORITY,
// 		      &mindgrove_crypto_api);
