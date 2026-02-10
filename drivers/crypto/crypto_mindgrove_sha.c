/*
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT mindgrove_sha256

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

static volatile SHA256_Type *sha_reg;

struct mindgrove_sha_config {
    volatile SHA256_Type *regs;
};

static void input_text_to_sha(unsigned char **final_sha_text_dict,
                              int block_message_length_bits,
                              int mode) {

    //printk("\n--- input_text_to_sha ENTER ---\n");
    //printk("mode = %d, block_message_length_bits = %d\n", mode, block_message_length_bits);
    //static int call_id;
    //printk("SHA_INPUT call #%d\n", ++call_id);

    if (mode == 0) {
        unsigned char *final_sha_text = final_sha_text_dict[0];
        // printk("SHA BLOCK TO WRITE (mode 0):\n");
        // for (int i = 0; i < block_message_length_bits / 8; i++) {
        //     printk("%02x ", final_sha_text[i]);
        //     if ((i + 1) % 16 == 0) printk("\n");
        // }
        // printk("\n");

        for (uint64_t i = 0; i < 8; i++) {
            uint64_t temp_64_value = 0;
            for (int j = 0; j < 8; j++) {
                temp_64_value = (temp_64_value << 8) | *final_sha_text;
                final_sha_text++;
            }
            // printk("SHA_INPUT <= 0x%016llx\n", temp_64_value);
            sha_reg->SHA_INPUT = temp_64_value;
        }
    } else {
        unsigned char *final_sha_text = final_sha_text_dict[0];
        int text_end_tracker = 0;
        // printk("SHA BLOCK TO WRITE (mode 1):\n");
        // for (int i = 0; i < block_message_length_bits / 8; i++) {
        //     printk("%02x ", final_sha_text[i]);
        //     if ((i + 1) % 16 == 0) printk("\n");
        // }
        // printk("\n");

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
            // printk("SHA_INPUT <= 0x%016llx\n", temp_64_value);
            sha_reg->SHA_INPUT = temp_64_value;
        }
    }

    // printk("--- input_text_to_sha EXIT ---\n");
}

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

    //printk("get_sha_append_length_bits: final_block_message_length_bits=%d, sha_append_length_bits=%d\n",
    //       final_block_message_length_bits, sha_append_length_bits);

    return sha_append_length_bits;
}

static void get_sha_append_bits(unsigned char *sha_append_bits,
                                int input_len_bits,
                                int sha_append_length_bits) {

    //printk("get_sha_append_bits: input_len_bits=%d, sha_append_length_bits=%d\n",
     //      input_len_bits, sha_append_length_bits);

    size_t sha_max_inputlen_bits_index;
    int sha_padding_index;
    sha_padding_index = (int)((sha_append_length_bits -
                               sha_max_inputlen_bits) / byte_length) - 1;

    for (int pad_i = 0; pad_i <= sha_padding_index; pad_i++) {
        if (pad_i == 0) {
            sha_append_bits[pad_i] = 0x80;
            continue;
        }
        sha_append_bits[pad_i] = 0x00;
    }

    sha_max_inputlen_bits_index =
        (((size_t)sha_append_length_bits / (size_t)byte_length) - 1U);

    for (size_t temp_i = 0U; temp_i < ((size_t)sha_max_inputlen_bits / (size_t)byte_length); temp_i++) {
        if (temp_i == 0U) {
            sha_append_bits[sha_max_inputlen_bits_index - temp_i] = input_len_bits;
        } else {
            sha_append_bits[sha_max_inputlen_bits_index - temp_i] = ((size_t)input_len_bits >> (temp_i * 8U));
        }
    }

    //printk("SHA APPEND BITS:\n");
    // for (int i = 0; i < (sha_append_length_bits / 8); i++) {
    //     printk("%02x ", sha_append_bits[i]);
    //     if ((i + 1) % 16 == 0) printk("\n");
    // }
    //printk("\n");
}

uint16_t sha256_read_output(unsigned char *sha_output, size_t *output_length) {
    uint64_t output_reg[4];
    *output_length = 0;

    //printk("\n--- sha256_read_output ENTER ---\n");

    if (sha_output == NULL) {
        printk("sha256_read_output: NULL output pointer\n");
        return EFAULT;
    }

    for (int i = 0; i <= 3; i++) {
        output_reg[i] = sha_reg->SHA_OUTPUT;
        //printk("SHA_OUTPUT[%d] = 0x%016llx\n", i, output_reg[i]);
    }

    for (int i = 0; i <= 3; i++) {
        for (int j = 7; j >= 0; j--) {
            *sha_output = output_reg[i] >> (8 * j);
            sha_output++;
        }
        *output_length += 8;
    }

    sha_reg->SHA_CTRL = 0U;
    //printk("--- sha256_read_output EXIT ---\n");

    return 0;
}

uint16_t sha256_zeroize(void) {
    //printk("sha256_zeroize called\n");
    sha_reg->SHA_CTRL = 0U;
    return 0;
}

uint16_t SHA256_Multi_Run(const unsigned char *input_text,
                          int input_len_bits,
                          int total_length,
                          int iterated_length_bits) {
    int sha_append_length_bits;
    int offset;
    unsigned char sha_append_bits[256];
    unsigned char *final_sha_text[1];
    unsigned char *sha_text_final[2];

    //printk("\n[SHA256_Multi_Run] ENTER\n");
    //printk("params: in=%dbits, total=%d, iterated=%d\n",
    //       input_len_bits, total_length, iterated_length_bits);

    if (input_text == NULL) {
        //printk("ERROR: input_text is NULL\n");
        return EFAULT;
    }

    if (iterated_length_bits == 0) {
        while ((sha_reg->SHA_STATUS & 1U) != 0U) {}
    }

    /* KEY FIX: Only generate padding if total_length > 0 */
    //if (total_length > 0 && (total_length - iterated_length_bits) <= sha_block_length_bits) {
    if ((total_length > 0 || (total_length == 0 && input_len_bits == 0)) && (total_length - iterated_length_bits) <= sha_block_length_bits) {
        //printk("FINAL BLOCK - generating padding\n");
        
        sha_append_length_bits = get_sha_append_length_bits(input_len_bits);
        get_sha_append_bits(sha_append_bits, total_length,
                            sha_append_length_bits);

        if ((sha_append_length_bits < sha_block_length_bits) || 
            (input_len_bits == 0)) {
            //printk("Single block with padding\n");
            sha_text_final[0] = input_text;
            sha_text_final[1] = sha_append_bits;
            input_text_to_sha(sha_text_final, input_len_bits, 1);
            while (!(sha_reg->SHA_STATUS & 2U)) { 
                // Empty Loop for MISRA Compliance
            }
        } else if (sha_append_length_bits < (2 * sha_block_length_bits)) {
            //printk("Two blocks with padding\n");
            sha_text_final[0] = input_text;
            sha_text_final[1] = sha_append_bits;
            input_text_to_sha(sha_text_final, input_len_bits, 1);

            while (!(sha_reg->SHA_STATUS & 2U)) {}
            sha_reg->SHA_CTRL = 1;

            offset = (sha_append_length_bits - sha_block_length_bits) / byte_length;
            sha_text_final[0] = &sha_append_bits[offset];
            input_text_to_sha(sha_text_final, 1, 0);
        } else {
            return EINVAL;
        }
    } else if (input_len_bits > 0) {
        /* INTERMEDIATE BLOCK - process data only */
        //printk("INTERMEDIATE BLOCK - no padding\n");
        final_sha_text[0] = input_text;
        input_text_to_sha(final_sha_text, 1, 0);
        
        while (!(sha_reg->SHA_STATUS & 2U)) {}
        
        if (iterated_length_bits == 0) {
            sha_reg->SHA_CTRL = 1;
        }
    }

    //printk("[SHA256_Multi_Run] EXIT\n");
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


static int mindgrove_sha_hash(struct hash_ctx *ctx,
                              struct hash_pkt *pkt,
                              bool finish)
{
    struct mindgrove_sha_dev_data *data = ctx->drv_sessn_state;
    long int bits = pkt->in_len * 8;
    long int total_bits = data->iterated_length_bits + bits;
    int ret;
    
     //printk("[SHA] finish=%d, bits=%ld, iterated=%ld\n", 
     //       finish, bits, data->iterated_length_bits);
    
    // For intermediate calls: total_length = 0 (unknown)
    // For final calls: total_length = actual total bits
    int hw_total_length = finish ? total_bits : 0;
    
    // Use dummy buffer if NULL input (for zero-length finalization)
    const uint8_t *input_ptr = pkt->in_buf;
    uint8_t dummy = 0;
    if (input_ptr == NULL && bits == 0) {
        input_ptr = &dummy;
    }
    
    // ALWAYS call SHA256_Multi_Run if we have data OR if it's finalization
    // The hardware needs to know about finish to generate padding
    if (bits > 0 || finish) {
        ret = SHA256_Multi_Run(input_ptr,
                               bits,
                               hw_total_length,
                               data->iterated_length_bits);
        
        if (ret != SUCCESS) {
            printk("[SHA] SHA256_Multi_Run failed: %d\n", ret);
            return -EIO;
        }
        
        // Update iterated length only if we processed data
        if (bits > 0) {
            data->iterated_length_bits += bits;
        }
    }
    
    // If this is the final call, read output
    if (finish) {
         //printk("[SHA] Final call, reading output\n");
        size_t hash_len = 0;
        sha256_read_output(pkt->out_buf, &hash_len);
        data->iterated_length_bits = 0;
    }
    
    return 0;
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
    ctx->flags = CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS;
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

static int query_caps(const struct device *dev)
{
    return CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS | CAP_RAW_KEY;
}

/* ============================= */
/* Zephyr driver API             */
/* ============================= */

static const struct crypto_driver_api mindgrove_crypto_api = {
    .hash_begin_session = mindgrove_sha_begin_session,
    .hash_free_session  = mindgrove_sha_free_session,
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

    // printk("MindGrove SHA init called\n");
    // printk("DT_INST_REG_ADDR = %lx\n",
    //        (unsigned long)DT_INST_REG_ADDR(0));

    sha_reg = cfg->regs;

    printk("SHA reg base = %p\n", sha_reg);

    if (!sha_reg) {
        printk("SHA device not ready!\n");
        return -ENODEV;
    }

    // sha_reg->SHA_CTRL = 0;
    return 0;
}




#define MINDGROVE_SHA_INIT(n)                                      \
    static const struct mindgrove_sha_config sha_cfg_##n = {      \
        .regs = (volatile SHA256_Type *)DT_INST_REG_ADDR(n),      \
    };                                                            \
    DEVICE_DT_INST_DEFINE(n,                                      \
                          sha_init,                               \
                          NULL,                                   \
                          &sha_data,                              \
                          &sha_cfg_##n,                           \
                          PRE_KERNEL_1,                           \
                          CONFIG_CRYPTO_INIT_PRIORITY,            \
                          &mindgrove_crypto_api);

DT_INST_FOREACH_STATUS_OKAY(MINDGROVE_SHA_INIT)
