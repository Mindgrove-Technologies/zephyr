/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/crypto/crypto.h>
#include <errno.h>
#include "crypto_mindgrove_sha.h"

#define DT_DRV_COMPAT mindgrove_sha256

const int sha_max_inputlen_bits = 64;
const int sha_block_length_bits = 512;

/* ============================= */
/* SHA Helpers (integrated)      */
/* ============================= */

static void input_text_to_sha(unsigned char **final_sha_text_dict, int block_message_length_bits,
			      int mode)
{

	if (mode == 0) {
		unsigned char *final_sha_text = final_sha_text_dict[0];

		for (uint64_t i = 0; i < 8; i++) {
			uint64_t temp_64_value = 0;
			for (int j = 0; j < 8; j++) {
				temp_64_value = (temp_64_value << 8) | *final_sha_text;
				final_sha_text++;
			}
			sha_reg->SHA_INPUT = temp_64_value;
		}
	} else {
		unsigned char *final_sha_text = final_sha_text_dict[0];
		int text_end_tracker = 0;

		for (uint64_t i = 0; i < 8; i++) {
			uint64_t temp_64_value = 0;
			for (int j = 0; j < 8; j++) {
				if (text_end_tracker == (block_message_length_bits / byte_length)) {
					final_sha_text = final_sha_text_dict[1];
				}
				temp_64_value = (temp_64_value << 8) | *final_sha_text;
				final_sha_text++;
				text_end_tracker += 1;
			}
			sha_reg->SHA_INPUT = temp_64_value;
		}
	}
}

static int get_sha_append_length_bits(int final_block_message_length_bits)
{
	int sha_padding_length_bits =
		sha_block_length_bits - (final_block_message_length_bits + sha_max_inputlen_bits);

	int sha_append_length_bits;
	if (sha_padding_length_bits < 8) {
		sha_append_length_bits =
			((2 * sha_block_length_bits) - final_block_message_length_bits);
	} else {
		sha_append_length_bits = sha_padding_length_bits + sha_max_inputlen_bits;
	}

	return sha_append_length_bits;
}

static void get_sha_append_bits(unsigned char *sha_append_bits, int input_len_bits,
				int sha_append_length_bits)
{

	size_t sha_max_inputlen_bits_index;
	int sha_padding_index;
	sha_padding_index =
		(int)((sha_append_length_bits - sha_max_inputlen_bits) / byte_length) - 1;

	for (int pad_i = 0; pad_i <= sha_padding_index; pad_i++) {
		if (pad_i == 0) {
			sha_append_bits[pad_i] = 0x80;
			continue;
		}
		sha_append_bits[pad_i] = 0x00;
	}

	sha_max_inputlen_bits_index = (((size_t)sha_append_length_bits / (size_t)byte_length) - 1U);

	for (size_t temp_i = 0U; temp_i < ((size_t)sha_max_inputlen_bits / (size_t)byte_length);
	     temp_i++) {
		if (temp_i == 0U) {
			sha_append_bits[sha_max_inputlen_bits_index - temp_i] = input_len_bits;
		} else {
			sha_append_bits[sha_max_inputlen_bits_index - temp_i] =
				((size_t)input_len_bits >> (temp_i * 8U));
		}
	}
}

uint16_t sha256_read_output(unsigned char *sha_output, size_t *output_length)
{
	uint64_t output_reg[4];
	*output_length = 0;

	if (sha_output == NULL) {
		return EFAULT;
	}

	for (int i = 0; i <= 3; i++) {
		output_reg[i] = sha_reg->SHA_OUTPUT;
	}

	for (int i = 0; i <= 3; i++) {
		for (int j = 7; j >= 0; j--) {
			*sha_output = output_reg[i] >> (8 * j);
			sha_output++;
		}
		*output_length += 8;
	}

	sha_reg->SHA_CTRL = 0U;

	return 0;
}

uint16_t sha256_zeroize(void)
{
	sha_reg->SHA_CTRL = 0U;
	return 0;
}

/** @fn void SHA256_Single_Run(unsigned char *sha_output, unsigned char *input_text, long int
 * input_len_bits)
 * @brief The main function which runs the SHA algorithm on H/W
 * @details This function does a couple of things.
 * 1. It sets up the input and other configs as required and ensures SHA is ready.
 * 2. It then calculates some lengths that are required and the append bits for the last block.
 * 3. It then iterates across blocks and calls the required functions which will run SHA. While
 * doing this, it ensures that the SHA output of previous block is passed on as the pre-hash
 * for the next block.
 * 4. It then gets the output and resets previously set configurations.
 * 5. The final step is returning a pointer to an array of 256 bits (the hash).
 * @param unsigned char *sha_output : pointer to where the SHA256 output has to be stored
 * @param unsigned char *input_text : pointer to the message that has to be encoded
 * @param long int input_len_bits : length of input message in bits
 * @return Returns nothing. Changes sha_output in place.
 */
uint16_t SHA256_Single_Run(unsigned char *sha_output, const unsigned char *input_text,
			   int input_len_bits)
{
	int total_blocks;
	int final_block_message_length_bits;
	int sha_append_length_bits;
	int last_block_double_run = 0;
	int offset = 0;
	uint32_t status = SUCCESS;
	size_t hash_length = 0;
	unsigned char *final_sha_text[1];
	unsigned char *sha_text_final[2];
	unsigned char sha_append_bits[256];
	unsigned char *substring_input_text;

	if ((sha_output == NULL) || (input_text == NULL)) {
		return EFAULT;
	}

	// Wait for sha to be ready
	while ((sha_reg->SHA_STATUS & 1U) != 0U) {
		// Empty loop for MISRA compliance
	}

	// Gets the required lengths
	// long int input_len_bits = StrLen(input_text) * 8;
	total_blocks = (int)(input_len_bits / sha_block_length_bits) + 1;
	final_block_message_length_bits = input_len_bits % sha_block_length_bits;
	sha_append_length_bits = get_sha_append_length_bits(final_block_message_length_bits);

	// Gets the final sha append bits which will be used when the
	// last block is being run
	get_sha_append_bits(sha_append_bits, input_len_bits, sha_append_length_bits);

	printk("\n");

	// Runs the sha for each block of text
	for (int block_index = 0; block_index < total_blocks; block_index += 1) {
		offset = block_index * (sha_block_length_bits / byte_length);
		substring_input_text = &input_text[offset];
		// For last block
		if (block_index == (total_blocks - 1)) {
			// For regular cases of last block
			if (sha_append_length_bits <= sha_block_length_bits) {
				sha_text_final[0] = substring_input_text;
				sha_text_final[1] = sha_append_bits;
				input_text_to_sha(sha_text_final, final_block_message_length_bits,
						  1);
			} else if (sha_append_length_bits < (2 * sha_block_length_bits)) {
				// For case when block text is > 440 bits
				sha_text_final[0] = substring_input_text;
				sha_text_final[1] = sha_append_bits;
				input_text_to_sha(sha_text_final, final_block_message_length_bits,
						  1);
				last_block_double_run = 1;
			} else {
				// No other case should occur
				// log_emit(ERROR, sha_error_message_input_length);
				return EINVAL;
			}
		} else {
			final_sha_text[0] = substring_input_text;
			input_text_to_sha(final_sha_text, 1, 0);
		}
		// Waits for sha output to get ready
		while (!(sha_reg->SHA_STATUS & 2U)) {
			// Empty loop for MISRA Compliance
		}

		// For case of last block when block text is > 440 bits -
		// Run sha a second time.
		if (last_block_double_run == 1) {
			offset = (sha_append_length_bits - sha_block_length_bits) / byte_length;
			final_sha_text[0] = &sha_append_bits[offset];
			input_text_to_sha(final_sha_text, 1, 0);
			while (!(sha_reg->SHA_STATUS & 2U)) {
				// Empty loop for MISRA Compliance
			}
		}
	}
	// Gets the output
	status = sha256_read_output(sha_output, &hash_length);

	return status;
}

uint16_t SHA256_Multi_Run(const unsigned char *input_text, int input_len_bits, int total_length,
			  int iterated_length_bits)
{
	int sha_append_length_bits;
	unsigned char sha_append_bits[128];
	unsigned char *sha_text_final[2];

	if (input_text == NULL) {
		return EFAULT;
	}

	if (iterated_length_bits == 0) {
		while ((sha_reg->SHA_STATUS & 1U) != 0U) {
		}
	}

	/* Check if this is the finalization call (where padding is needed) */
	if ((total_length > 0 || (total_length == 0 && input_len_bits == 0)) &&
	    (total_length - iterated_length_bits) <= sha_block_length_bits) {

		sha_append_length_bits = get_sha_append_length_bits(input_len_bits);
		get_sha_append_bits(sha_append_bits, total_length, sha_append_length_bits);

		if (sha_append_length_bits <= (sha_block_length_bits - input_len_bits)) {
			sha_text_final[0] = (unsigned char *)input_text;
			sha_text_final[1] = sha_append_bits;
			input_text_to_sha(sha_text_final, input_len_bits, 1);

			while (!(sha_reg->SHA_STATUS & 2U)) {
			}
		} else {
			sha_text_final[0] = (unsigned char *)input_text;
			sha_text_final[1] = sha_append_bits;
			input_text_to_sha(sha_text_final, input_len_bits, 1);

			while (!(sha_reg->SHA_STATUS & 2U)) {
			}
			sha_reg->SHA_CTRL = 1;

			int bytes_already_in_first_block =
				(sha_block_length_bits - input_len_bits) / 8;
			unsigned char *remaining_padding =
				&sha_append_bits[bytes_already_in_first_block];

			sha_text_final[0] = remaining_padding;
			input_text_to_sha(sha_text_final, 1, 0);

			while (!(sha_reg->SHA_STATUS & 2U)) {
			}
		}
	} else if (input_len_bits > 0) {
		sha_text_final[0] = (unsigned char *)input_text;
		input_text_to_sha(sha_text_final, 1, 0);

		while (!(sha_reg->SHA_STATUS & 2U)) {
		}

		if (iterated_length_bits == 0) {
			sha_reg->SHA_CTRL = 1;
		}
	}

	return SUCCESS;
}

/* ============================= */
/* Device data structure         */
/* ============================= */

struct mindgrove_sha_dev_data {
	bool in_use;
	long int iterated_length_bits;
	volatile SHA256_Type *regs;
};

/* ============================= */
/* Zephyr hash handler           */
/* ============================= */

static int mindgrove_sha_hash(struct hash_ctx *ctx, struct hash_pkt *pkt, bool finish)
{
    struct mindgrove_sha_dev_data *data = ctx->drv_sessn_state;
    const uint8_t *src = pkt->in_buf;
    uint32_t remaining_bytes = pkt->in_len;
    int ret;

    /* * 1. Process all full 64-byte (512-bit) blocks in this packet.
     * We loop here because the hardware only processes one block at a time 
     * when called with the intermediate (non-final) logic.
     */
    while (remaining_bytes > 64) {
        ret = SHA256_Multi_Run(src, 64 * 8, 0, data->iterated_length_bits);
        if (ret != SUCCESS) {
            return -EIO;
        }

        data->iterated_length_bits += (64 * 8);
        src += 64;
        remaining_bytes -= 64;
    }

    /* * 2. Handle the final chunk of this packet.
     * If 'finish' is true, SHA256_Multi_Run will apply NIST padding 
     * based on the 'hw_total_length'.
     */
    int hw_total_length = finish ? (data->iterated_length_bits + (remaining_bytes * 8)) : 0;
    
    // We pass only the 'remaining_bytes' (0 to 64) to the multi-run function.
    ret = SHA256_Multi_Run(src, remaining_bytes * 8, hw_total_length, 
                           data->iterated_length_bits);
    if (ret != SUCCESS) {
        return -EIO;
    }

    data->iterated_length_bits += (remaining_bytes * 8);

    /* 3. If this is the finalization call, extract the digest from hardware */
    if (finish) {
        size_t hash_len = 0;
        sha256_read_output(pkt->out_buf, &hash_len);
        
        // Reset state for the next potential session
        data->iterated_length_bits = 0;
    }

    return 0;
}

/* ============================= */
/* Zephyr session callbacks      */
/* ============================= */

static int mindgrove_sha_begin_session(const struct device *dev, struct hash_ctx *ctx,
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
	ctx->hash_hndlr = mindgrove_sha_hash; /* Set the handler for streaming */
	ctx->flags = CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS;
	return 0;
}

static int mindgrove_sha_free_session(const struct device *dev, struct hash_ctx *ctx)
{
	struct mindgrove_sha_dev_data *data = ctx->drv_sessn_state;

	data->in_use = false;
	data->iterated_length_bits = 0;
	ctx->drv_sessn_state = NULL;

	return 0;
}

static int query_caps(const struct device *dev)
{
	return CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS | CAP_RAW_KEY;
}

/* ============================= */
/* Zephyr driver API             */
/* ============================= */

static const struct crypto_driver_api mindgrove_crypto_api = {
	.hash_begin_session = mindgrove_sha_begin_session,
	.hash_free_session = mindgrove_sha_free_session,
	.hash_async_callback_set = NULL,
	.query_hw_caps = query_caps,
};

/* ============================= */
/* Device instantiation          */
/* ============================= */

static struct mindgrove_sha_dev_data sha_data = {
	.in_use = false,
	.iterated_length_bits = 0,
};

static int sha_init(const struct device *dev)
{
	const struct mindgrove_sha_config *cfg = dev->config;
	sha_reg = cfg->regs;
	if (!sha_reg) {
		printk("SHA device not ready!\n");
		return -ENODEV;
	}
	return 0;
}

#define MINDGROVE_SHA_INIT(n)                                                                      \
	static const struct mindgrove_sha_config sha_cfg_##n = {                                   \
		.regs = (volatile SHA256_Type *)DT_INST_REG_ADDR(n),                               \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, sha_init, NULL, &sha_data, &sha_cfg_##n, PRE_KERNEL_1,            \
			      CONFIG_CRYPTO_INIT_PRIORITY, &mindgrove_crypto_api);

DT_INST_FOREACH_STATUS_OKAY(MINDGROVE_SHA_INIT)
