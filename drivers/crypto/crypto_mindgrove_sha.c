/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/crypto/crypto.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>

/* ============================= */
/* SHA Hardware Registers        */
/* ============================= */

#include "crypto_mindgrove_sha.h"

/* ============================= */
/* SHA Helpers (integrated)      */
/* ============================= */

#define byte_length 8
const int sha_max_inputlen_bits = 64;
const int sha_block_length_bits = 512;
static volatile SHA256_Type *sha_regs;


/** void input_text_to_sha(unsigned char **final_sha_text_dict
                        , int block_message_length_bits, int sha_append_length_bits, int mode)
 * @brief Sends a text block to the SHA hardware for hashing.
 * @details An input text to SHA is divided into multiple blocks, and each block of
 * text is hashed independently. There are two cases of this.
 * a. When the incoming block is defined by a pointer that has $sha_block_length_bits contiguous locations
 * b. When the incoming block is defined by a pointer that points to two different
 * contiguous locations, the sum of which is $sha_block_length_bits.
 * The specific case is defined by the parameter 'mode'.
 * If mode=0, it is case 'a'. And when the mode=1, it is case 'b'.
 *
 * The $sha_block_length_bits bits of block values are copied to continguous memory locations from the address pointed by
 * sha_regs->SHA_INPUT, via the little endian format - both across registers and between
 * each register. Once the last 64 bits of the sha_regs->SHA_INPUT receive values,
 * the SHA hardware will automatically run.
 * @param unsigned char **final_sha_text_dict : the dict which contains the values to be copied - either in case 'a' and case 'b'
 * @param int block_message_length_bits : length of message in bits
 * @param int sha_append_length_bits : length of append bits
 * @param int mode : when mode=0 -> case 'a', when mode=1 -> case 'b'
 * @return Returns nothing. By the end of the function, you will have transferred
 * the input text block to the SHA input hardware registers.
 */
static void input_text_to_sha(unsigned char **final_sha_text_dict,
                              int block_message_length_bits,
                              int mode) {
    if (mode == 0) {
        unsigned char *final_sha_text = final_sha_text_dict[0];
        for (__uint64_t i = 0; i < 8; i++) {
            __uint64_t temp_64_value = 0;
            for (int j = 0; j < 8; j++) {
                temp_64_value = (temp_64_value << 8) | *final_sha_text;
                final_sha_text++;
            }
            sha_regs->SHA_INPUT = temp_64_value;
        }
    } else {
        unsigned char *final_sha_text = final_sha_text_dict[0];
        int text_end_tracker = 0;
        for (__uint64_t i = 0; i < 8; i++) {
            __uint64_t temp_64_value = 0;
            for (int j = 0; j < 8; j++) {
                // If the addition of append bits is over, reset the pointer to
                // text input. Note that if 64 bytes (8x8) are filled,
                // this is void and the loop will break
                if (text_end_tracker ==
                    (block_message_length_bits / byte_length)) {
                    final_sha_text = final_sha_text_dict[1];
                }
                temp_64_value = (temp_64_value << 8) | *final_sha_text;
                final_sha_text++;
                text_end_tracker += 1;
            }
            sha_regs->SHA_INPUT = temp_64_value;
        }
    }
}

/** @fn short int get_sha_append_length_bits(int final_block_message_length_bits)
 * @brief This decides the number of bits that have to be appended to input text block.
 * @details The number of bits is defaulted to equal the difference between standard block length and
 * the final text block length. But if this length implies that the length of padding available
 * for padding is less than 8 bits (min required), we will need more padding bits which will
 * take another block run to execute. This condition is present in the if part of the statement.
 * @param int final_block_message_length_bits : the length of message in the last block
 * @return Returns a short int which has the length of append bits required
 * (max value = 2*$sha_block_length_bits-448).
 */
static int get_sha_append_length_bits(int final_block_message_length_bits) {
    int sha_padding_length_bits = sha_block_length_bits -
                    (final_block_message_length_bits + sha_max_inputlen_bits);

    int sha_append_length_bits;
    if (sha_padding_length_bits < 8) {
        sha_append_length_bits = ((2 * sha_block_length_bits) -
                                  final_block_message_length_bits);
    } else {
        sha_append_length_bits = sha_padding_length_bits +
                                 sha_max_inputlen_bits;
    }
    return sha_append_length_bits;
}

/** @fn unsigned char * get_sha_append_bits(long int input_len_bits, short int sha_append_length_bits)
 * @brief This creates a continuous memory set to store the append bits and calls the
 * get_sha_append_bits function to fill the memory locations with appropriate values.
 * @details The appending of bits is done only to the last block in SHA.
 * The function first creates memory locations required during runtime and if this works, runs
 * the get_sha_append_bits to fill them with the correct append bits.
 * @param long int input_len_bits : length of input message in bits
 * @param short int sha_append_length_bits : number of bits that have to be appended
 * @return Returns a pointer to the continuous memory locations where the append bits are stored.
 * (max value = 2*$sha_block_length_bits-448).
 */
static void get_sha_append_bits(unsigned char *sha_append_bits,
                                int input_len_bits,
                                int sha_append_length_bits) {
    size_t sha_max_inputlen_bits_index;
    int sha_padding_index;
    sha_padding_index = (int)((sha_append_length_bits -
                               sha_max_inputlen_bits) /
                              byte_length) - 1;

    for (int pad_i = 0; pad_i <= sha_padding_index; pad_i++) {
        if (pad_i == 0) {
            sha_append_bits[pad_i] = 0x80;
            continue;
        }
        sha_append_bits[pad_i] = 0x00;
    }

    sha_max_inputlen_bits_index =
        (((size_t)sha_append_length_bits / (size_t)byte_length) - 1U);
    for (size_t temp_i = 0U;
         temp_i < ((size_t)sha_max_inputlen_bits / (size_t)byte_length);
         temp_i++) {
        if (temp_i == 0U) {
            sha_append_bits[sha_max_inputlen_bits_index - temp_i] =
                input_len_bits;
        } else {
            sha_append_bits[sha_max_inputlen_bits_index - temp_i] =
                ((size_t)input_len_bits >> (temp_i * 8U));
        }
    }
}

/** @fn void *sha256_read_output(unsigned char * sha_output)
 * @brief Used to read and return the output of the SHA hashing.
 * @details The function returns a pointer to access the ($sha_block_length_bits/2) bits
 * SHA outputs from the MSB.
 * @param unsigned char *sha_output : pointer to where the SHA256 output has to be stored
 * @return Returns nothing. Changes sha_output in place.
 */
uint16_t sha256_read_output(unsigned char *sha_output, size_t *output_length) {
    __uint64_t output_reg[4];
    *output_length = 0;

    if (sha_output == NULL) {
        return EFAULT;
    }

    for (int i = 0; i <= 3; i++) {
        output_reg[i] = sha_regs->SHA_OUTPUT;
    }

    for (int i = 0; i <= 3; i++) {
        for (int j = 7; j >= 0; j--) {
            *sha_output = output_reg[i] >> (8 * j);
            sha_output++;
        }
        *output_length = *output_length + 8;
    }

    sha_regs->SHA_CTRL = 0U;

    return SUCCESS;
}

uint16_t sha256_zeroize(void) {
    sha_regs->SHA_CTRL = 0U;
    return SUCCESS;
}

/** @fn void SHA256_Single_Run(unsigned char *sha_output, unsigned char *input_text, long int input_len_bits)
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
uint16_t SHA256_Single_Run(unsigned char *sha_output,
                           const unsigned char *input_text,
                           int input_len_bits) {
    int total_blocks;
    int final_block_message_length_bits;
    int sha_append_length_bits;
    int last_block_double_run = 0;
    int offset = 0;
    __uint32_t status = SUCCESS;
    size_t hash_length = 0;
    unsigned char *final_sha_text[1];
    unsigned char *sha_text_final[2];
    unsigned char sha_append_bits[256];
    unsigned char *substring_input_text;

    if ((sha_output == NULL) || (input_text == NULL)) {
        return EFAULT;
    }

    // Wait for sha to be ready
    while ((sha_regs->SHA_STATUS & 1U) != 0U) {
        // Empty loop for MISRA compliance
    }

    // Gets the required lengths
    // long int input_len_bits = StrLen(input_text) * 8;
    total_blocks = (int)(input_len_bits / sha_block_length_bits) + 1;
    final_block_message_length_bits = input_len_bits % sha_block_length_bits;
    sha_append_length_bits =
        get_sha_append_length_bits(final_block_message_length_bits);

    // Gets the final sha append bits which will be used when the
    // last block is being run
    get_sha_append_bits(sha_append_bits, input_len_bits,
                        sha_append_length_bits);

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
                input_text_to_sha(sha_text_final,
                                  final_block_message_length_bits, 1);
            } else if (sha_append_length_bits < (2 * sha_block_length_bits)) {
                // For case when block text is > 440 bits
                sha_text_final[0] = substring_input_text;
                sha_text_final[1] = sha_append_bits;
                input_text_to_sha(sha_text_final,
                                  final_block_message_length_bits, 1);
                last_block_double_run = 1;
            } else {
                // No other case should occur
                log_emit(ERROR, sha_error_message_input_length);
                return EINVAL;
            }
        } else {
            final_sha_text[0] = substring_input_text;
            input_text_to_sha(final_sha_text, 1, 0);
        }
        // Waits for sha output to get ready
        while (!(sha_regs->SHA_STATUS & 2U)) {
            // Empty loop for MISRA Compliance
        }

        // For case of last block when block text is > 440 bits -
        // Run sha a second time.
        if (last_block_double_run == 1) {
            offset =
                (sha_append_length_bits - sha_block_length_bits) / byte_length;
            final_sha_text[0] = &sha_append_bits[offset];
            input_text_to_sha(final_sha_text, 1, 0);
            while (!(sha_regs->SHA_STATUS & 2U)) {
                // Empty loop for MISRA Compliance
            }
        }
    }
    // Gets the output
    status = sha256_read_output(sha_output, &hash_length);

    return status;
}

/** @fn long int SHA256_Multi_Run(unsigned char *sha_output
                                        , unsigned char *input_text, long int input_len_bits
                                        , long int total_length
                                        , long int iterated_length_bits)
 * @brief The main function which runs the SHA algorithm on H/W by accessing RAM multiple times.
 * @details This function provides the same functionality as singleshotram but with the
 * following differences :
 * 1. It needs to be called multiple times by the user - one each after inputing text to
 * a specific location.
 * 2. The sha_output is a pointer where the final output will be stored - shared by the user.
 * 3. The function returns an iterated_length_bits, which adds the current input_len_bits and
 * the previous iterated_length_bits shared by the user. This will help us keep track of whether
 * we are at the first block of run, or in the middle of the run, or in the last block of run
 * (by comparing iterated_length_bits with the total_length_of_bits) at which point
 * we will need to return the output.
 * @param unsigned char *sha_output : pointer to where the SHA256 output has to be stored
 * @param unsigned char *input_text : pointer to the message that has to be encoded
 * @param long int input_len_bits : length of input message in bits
 * @param long int total_length : length of complete input message in bits
 * @param long int iterated_length_bits : number of bits in message which have already been processed by the SHA h/w
 * This parameter value should be the output from the previous function call. If calling this function for the first time, the value should be 0.
 * @return Returns the number of bits which have been processed by SHA hardware.
 */
uint16_t SHA256_Multi_Run(const unsigned char *input_text,
                          int input_len_bits,
                          int total_length,
                          int iterated_length_bits) {
    int sha_append_length_bits;
    int offset;
    unsigned char sha_append_bits[256];
    unsigned char *final_sha_text[1];
    unsigned char *sha_text_final[2];

    if (input_text == NULL) {
        return EFAULT;
    }

    if (iterated_length_bits == 0) {
        // Wait for sha to be ready
        while ((sha_regs->SHA_STATUS & 1U) != 0U) {
            // Empty loop for MISRA compliance
        }
    }

    if ((input_len_bits == sha_block_length_bits)) {
        final_sha_text[0] = input_text;
        input_text_to_sha(final_sha_text, 1, 0);
        while (!(sha_regs->SHA_STATUS & 2U)) {
            // Empty loop for MISRA compliance
        }
        if (iterated_length_bits == 0) {
            sha_regs->SHA_CTRL = 1;
        }
    }

    if ((total_length - iterated_length_bits) <= sha_block_length_bits) {
        sha_append_length_bits = get_sha_append_length_bits(input_len_bits);
        get_sha_append_bits(sha_append_bits, total_length,
                            sha_append_length_bits);

        if (sha_append_length_bits <= sha_block_length_bits) {
            sha_text_final[0] = input_text;
            sha_text_final[1] = sha_append_bits;
            input_text_to_sha(sha_text_final, input_len_bits, 1);
        } else if (sha_append_length_bits < (2 * sha_block_length_bits)) {
            // For case when block text is > 440 bits
            sha_text_final[0] = input_text;
            sha_text_final[1] = sha_append_bits;
            input_text_to_sha(sha_text_final, input_len_bits, 1);

            while (!(sha_regs->SHA_STATUS & 2U)) {
                // Empty Loop for MISRA Compliance
            }
            sha_regs->SHA_CTRL = 1;

            offset =
                (sha_append_length_bits - sha_block_length_bits) / byte_length;
            sha_text_final[0] = &sha_append_bits[offset];
            input_text_to_sha(sha_text_final, 1, 0);
        } else {
            log_emit(ERROR, sha_error_message_input_length_multishot);
            return EINVAL;
        }
    }

    return SUCCESS;
}


/* ============================= */
/* Device data structure         */
/* ============================= */

struct mindgrove_sha_dev_data {
    bool in_use;
    long int iterated_length_bits; /* Tracks multi-run progress */
};

/* ============================= */
/* Zephyr hash handler           */
/* ============================= */

static int mindgrove_sha_hash(const struct device *dev,
                              struct hash_ctx *ctx,
                              struct hash_pkt *pkt)
{
    struct mindgrove_sha_dev_data *data = ctx->drv_sessn_state;
    long int bits = pkt->in_len * 8;
    long int total_bits = data->iterated_length_bits + bits;
    int ret;

    if (pkt->out_buf == NULL) {
        /* Update: call SHA256_Multi_Run */
        ret = SHA256_Multi_Run(pkt->in_buf, bits, total_bits, data->iterated_length_bits);
        if (ret != SUCCESS) {
            return -EIO;
        }
        data->iterated_length_bits += bits;
        return 0;
    } else {
        /* Final block: run SHA256_Multi_Run for remaining bits and read output */
        ret = SHA256_Multi_Run(pkt->in_buf, bits, total_bits, data->iterated_length_bits);
        if (ret != SUCCESS) {
            return -EIO;
        }
        data->iterated_length_bits += bits;

        size_t hash_len = 0;
        sha256_read_output(pkt->out_buf, &hash_len);

        data->iterated_length_bits = 0;
        return 0;
    }
}

/* ============================= */
/* Zephyr session callbacks      */
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
    ctx->hash_hndlr = mindgrove_sha_hash;  /* Set the handler for streaming */

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
/* Zephyr driver API             */
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

static struct mindgrove_sha_dev_data sha_data = {
    .in_use = false,
    .iterated_length_bits = 0,
};

static int sha_init(const struct device *dev)
{
    sha_regs = (volatile SHA256_Type *)DT_INST_REG_ADDR(0);

    if (!sha_regs) {
        return -ENODEV;
    }

    sha_regs->SHA_CTRL = 0;
    return 0;
}



DEVICE_DT_INST_DEFINE(0,
                      sha_init,       /* init function */
                      NULL,
                      &sha_data,
                      NULL,
                      POST_KERNEL,
                      CONFIG_CRYPTO_INIT_PRIORITY,
                      &mindgrove_crypto_api);
