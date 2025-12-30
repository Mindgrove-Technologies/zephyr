/*
 * crypto_mindgrove_sha.c
 * Complete Zephyr driver using existing SHA functions
 */

 #include <zephyr/kernel.h>
 #include <zephyr/device.h>
 #include <zephyr/crypto/crypto.h>
 #include <string.h>
 #include <errno.h>
 
 #include "crypto_mindgrove_sha.h"
 
 #define DT_DRV_COMPAT mindgrove_sha256
 
 /* Constants from your existing code */
 #define SHA_BITS_PER_BYTE      8
 #define SHA256_BLOCK_BITS      512
 #define SHA256_BLOCK_BYTES     (SHA256_BLOCK_BITS / SHA_BITS_PER_BYTE)
 #define SHA256_HASH_LEN        32
 #define SHA_MAX_INPUTLEN_BITS  64
 #define BYTE_LENGTH            8
 
 /* Global hardware instance */
 static SHA256_Type *sha_instance = NULL;
 
 /* ==================== */
 /* Your existing functions - adapted for Zephyr */
 /* ==================== */
 
 static void input_text_to_sha(unsigned char **final_sha_text_dict,
							  int block_message_length_bits,
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
			 
			 sha_instance->SHA_INPUT = temp_64_value;
		 }
	 } else {
		 unsigned char *final_sha_text = final_sha_text_dict[0];
		 int text_end_tracker = 0;
		 
		 for (uint64_t i = 0; i < 8; i++) {
			 uint64_t temp_64_value = 0;
			 
			 for (int j = 0; j < 8; j++) {
				 if (text_end_tracker == (block_message_length_bits / BYTE_LENGTH)) {
					 final_sha_text = final_sha_text_dict[1];
				 }
				 
				 temp_64_value = (temp_64_value << 8) | *final_sha_text;
				 final_sha_text++;
				 text_end_tracker += 1;
			 }
			 
			 sha_instance->SHA_INPUT = temp_64_value;
		 }
	 }
 }
 
 static int get_sha_append_length_bits(int final_block_message_length_bits)
 {
	 int sha_padding_length_bits = SHA256_BLOCK_BITS - 
								  (final_block_message_length_bits + SHA_MAX_INPUTLEN_BITS);
	 int sha_append_length_bits;
	 
	 if (sha_padding_length_bits < 8) {
		 sha_append_length_bits = ((2 * SHA256_BLOCK_BITS) - final_block_message_length_bits);
	 } else {
		 sha_append_length_bits = sha_padding_length_bits + SHA_MAX_INPUTLEN_BITS;
	 }
	 
	 return sha_append_length_bits;
 }
 
 static void get_sha_append_bits(unsigned char *sha_append_bits,
								int input_len_bits,
								int sha_append_length_bits)
 {
	 size_t sha_max_inputlen_bits_index;
	 int sha_padding_index;
	 
	 sha_padding_index = (int)((sha_append_length_bits - SHA_MAX_INPUTLEN_BITS) / BYTE_LENGTH) - 1;
	 
	 for (int pad_i = 0; pad_i <= sha_padding_index; pad_i++) {
		 if (pad_i == 0) {
			 sha_append_bits[pad_i] = 0x80;
			 continue;
		 }
		 sha_append_bits[pad_i] = 0x00;
	 }
	 
	 sha_max_inputlen_bits_index = (((size_t)sha_append_length_bits / (size_t)BYTE_LENGTH) - 1U);
	 
	 for (size_t temp_i = 0U; temp_i < ((size_t)SHA_MAX_INPUTLEN_BITS / (size_t)BYTE_LENGTH); temp_i++) {
		 if (temp_i == 0U) {
			 sha_append_bits[sha_max_inputlen_bits_index - temp_i] = input_len_bits;
		 } else {
			 sha_append_bits[sha_max_inputlen_bits_index - temp_i] = ((size_t)input_len_bits >> (temp_i * 8U));
		 }
	 }
 }
 
 static uint16_t sha256_read_output_internal(unsigned char *sha_output,
											size_t *output_length)
 {
	 uint64_t output_reg[4];
	 
	 for (int i = 0; i <= 3; i++) {
		 output_reg[i] = sha_instance->SHA_OUTPUT;
	 }
	 
	 for (int i = 0; i <= 3; i++) {
		 for (int j = 7; j >= 0; j--) {
			 *sha_output = output_reg[i] >> (8 * j);
			 sha_output++;
		 }
	 }
	 
	 sha_instance->SHA_CTRL = 0U;
	 *output_length = SHA256_HASH_LEN;
	 
	 return 0; /* SUCCESS */
 }
 
 /* Wrapper for SHA256_Multi_Run that matches Zephyr conventions */
 static int sha256_multi_run_wrapper(unsigned char *input_text,
									int input_len_bits,
									int total_length,
									int iterated_length_bits)
 {
	 int sha_append_length_bits;
	 int offset;
	 unsigned char sha_append_bits[256];
	 unsigned char *final_sha_text[1];
	 unsigned char *sha_text_final[2];
	 
	 if (iterated_length_bits == 0) {
		 /* Wait for SHA to be ready */
		 while ((sha_instance->SHA_STATUS & SHA_STATUS_READY) != 0U) {
			 /* Wait */
		 }
	 }
	 
	 if ((input_len_bits == SHA256_BLOCK_BITS)) {
		 final_sha_text[0] = input_text;
		 input_text_to_sha(final_sha_text, 1, 0);
		 
		 while (!(sha_instance->SHA_STATUS & SHA_STATUS_OUT_READY)) {
			 /* Wait */
		 }
		 
		 if (iterated_length_bits == 0) {
			 sha_instance->SHA_CTRL_b.CONT_PREHASH = 1;
		 }
	 }
	 
	 if ((total_length - iterated_length_bits) <= SHA256_BLOCK_BITS) {
		 sha_append_length_bits = get_sha_append_length_bits(input_len_bits);
		 get_sha_append_bits(sha_append_bits, total_length, sha_append_length_bits);
		 
		 if (sha_append_length_bits <= SHA256_BLOCK_BITS) {
			 sha_text_final[0] = input_text;
			 sha_text_final[1] = sha_append_bits;
			 input_text_to_sha(sha_text_final, input_len_bits, 1);
		 } else if (sha_append_length_bits < (2 * SHA256_BLOCK_BITS)) {
			 /* For case when block text is > 440 bits */
			 sha_text_final[0] = input_text;
			 sha_text_final[1] = sha_append_bits;
			 input_text_to_sha(sha_text_final, input_len_bits, 1);
			 
			 while (!(sha_instance->SHA_STATUS & SHA_STATUS_OUT_READY)) {
				 /* Wait */
			 }
			 
			 sha_instance->SHA_CTRL_b.CONT_PREHASH = 1;
			 offset = (sha_append_length_bits - SHA256_BLOCK_BITS) / BYTE_LENGTH;
			 sha_text_final[0] = &sha_append_bits[offset];
			 input_text_to_sha(sha_text_final, 1, 0);
		 } else {
			 /* Error case */
			 return -EINVAL;
		 }
	 }
	 
	 return 0; /* SUCCESS */
 }
 
 uint16_t SHA256_Single_Run(unsigned char *sha_output,
						   unsigned char *input_text,
						   int input_len_bits)
 {
	 int total_blocks;
	 int final_block_message_length_bits;
	 int sha_append_length_bits;
	 int last_block_double_run = 0;
	 int offset = 0;
	 uint16_t status = 0;
	 unsigned char *final_sha_text[1];
	 unsigned char *sha_text_final[2];
	 unsigned char sha_append_bits[256];
	 unsigned char *substring_input_text;
	 
	 /* Wait for SHA to be ready */
	 while ((sha_instance->SHA_STATUS & SHA_STATUS_READY) != 0U) {
		 /* Empty loop for MISRA compliance */
	 }
	 
	 /* Gets the required lengths */
	 total_blocks = (input_len_bits / SHA256_BLOCK_BITS) + 1;
	 final_block_message_length_bits = input_len_bits % SHA256_BLOCK_BITS;
	 sha_append_length_bits = get_sha_append_length_bits(final_block_message_length_bits);
	 
	 /* Gets the final SHA append bits for padding */
	 get_sha_append_bits(sha_append_bits, input_len_bits, sha_append_length_bits);
	 
	 /* Run SHA for each block of text */
	 for (int block_index = 0; block_index < total_blocks; block_index += 1) {
		 offset = block_index * (SHA256_BLOCK_BITS / BYTE_LENGTH);
		 substring_input_text = &input_text[offset];
		 
		 /* For last block */
		 if (block_index == (total_blocks - 1)) {
			 /* For regular cases of last block */
			 if (sha_append_length_bits <= SHA256_BLOCK_BITS) {
				 sha_text_final[0] = substring_input_text;
				 sha_text_final[1] = sha_append_bits;
				 input_text_to_sha(sha_text_final, final_block_message_length_bits, 1);
			 } else if (sha_append_length_bits < (2 * SHA256_BLOCK_BITS)) {
				 /* For case when block text is > 440 bits */
				 sha_text_final[0] = substring_input_text;
				 sha_text_final[1] = sha_append_bits;
				 input_text_to_sha(sha_text_final, final_block_message_length_bits, 1);
				 last_block_double_run = 1;
			 } else {
				 /* No other case should occur */
				 return -EINVAL;
			 }
		 } else {
			 final_sha_text[0] = substring_input_text;
			 input_text_to_sha(final_sha_text, 1, 0);
		 }
		 
		 /* Wait for SHA output to get ready */
		 while (!(sha_instance->SHA_STATUS & SHA_STATUS_OUT_READY)) {
			 /* Empty loop for MISRA Compliance */
		 }
		 
		 /* For case of last block when block text is > 440 bits */
		 if (last_block_double_run == 1) {
			 offset = (sha_append_length_bits - SHA256_BLOCK_BITS) / BYTE_LENGTH;
			 final_sha_text[0] = &sha_append_bits[offset];
			 input_text_to_sha(final_sha_text, 1, 0);
			 
			 while (!(sha_instance->SHA_STATUS & SHA_STATUS_OUT_READY)) {
				 /* Empty loop for MISRA Compliance */
			 }
		 }
	 }
	 
	 /* Get the output */
	 size_t output_len;
	 status = sha256_read_output_internal(sha_output, &output_len);
	 
	 return status;
 }
 
 /* ==================== */
 /* Zephyr Driver Structures */
 /* ==================== */
 
 struct mindgrove_sha_config {
	 SHA256_Type *base;
 };
 
 /* Single global state */
 static struct {
	 struct k_mutex lock;
	 bool in_use;
	 uint32_t processed_bits; /* Track bits processed */
 } sha_state;
 
 /* ==================== */
 /* Zephyr Compute Function */
 /* ==================== */
 
 static int mindgrove_sha_compute(struct hash_ctx *ctx,
								 struct hash_pkt *pkt,
								 bool finish)
 {
	 /* We only support single-shot hashing */
	 if (!finish) {
		 return -ENOTSUP;
	 }
	 
	 if (k_mutex_lock(&sha_state.lock, K_MSEC(100)) != 0) {
		 return -EBUSY;
	 }
	 
	 int ret = 0;
	 
	 /* Convert input length from bytes to bits */
	 int input_len_bits = pkt->in_len * SHA_BITS_PER_BYTE;
	 
	 uint16_t hw_ret = SHA256_Single_Run(
		 pkt->out_buf,
		 (unsigned char *)pkt->in_buf,
		 input_len_bits
	 );
	 
	 if (hw_ret != 0) {
		 ret = -EIO;
	 }
	 
	 k_mutex_unlock(&sha_state.lock);
	 
	 return ret;
 }
 
 /* ==================== */
 /* Session Management */
 /* ==================== */
 
 static int mindgrove_sha_begin_session(const struct device *dev,
									   struct hash_ctx *ctx,
									   enum hash_algo algo)
 {
	 if (algo != CRYPTO_HASH_ALGO_SHA256) {
		 return -ENOTSUP;
	 }
	 
	 if (k_mutex_lock(&sha_state.lock, K_MSEC(100)) != 0) {
		 return -EBUSY;
	 }
	 
	 if (sha_state.in_use) {
		 k_mutex_unlock(&sha_state.lock);
		 return -EBUSY;
	 }
	 
	 sha_state.in_use = true;
	 sha_state.processed_bits = 0;
	 k_mutex_unlock(&sha_state.lock);
	 
	 ctx->drv_sessn_state = NULL;
	 ctx->hash_hndlr = mindgrove_sha_compute;
	 
	 return 0;
 }
 
 static int mindgrove_sha_free_session(const struct device *dev,
									  struct hash_ctx *ctx)
 {
	 if (k_mutex_lock(&sha_state.lock, K_MSEC(100)) != 0) {
		 return -EBUSY;
	 }
	 
	 sha_state.in_use = false;
	 sha_state.processed_bits = 0;
	 k_mutex_unlock(&sha_state.lock);
	 
	 return 0;
 }
 
 /* ==================== */
 /* Capabilities */
 /* ==================== */
 
 static int mindgrove_sha_query_hw_caps(const struct device *dev)
 {
	 /* Only synchronous, single-shot operations */
	 return (CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS);
 }
 
 /* ==================== */
 /* Driver Initialization */
 /* ==================== */
 
 static int mindgrove_sha_init(const struct device *dev)
 {
	 const struct mindgrove_sha_config *cfg = dev->config;
	 
	 /* Set global instance from DTS */
	 sha_instance = cfg->base;
	 
	 if (sha_instance == NULL) {
		 return -ENODEV;
	 }
	 
	 /* Initialize state */
	 k_mutex_init(&sha_state.lock);
	 sha_state.in_use = false;
	 sha_state.processed_bits = 0;
	 
	 /* Reset hardware */
	 sha_instance->SHA_CTRL = 0U;
	 
	 return 0;
 }
 
 /* ==================== */
 /* Driver API */
 /* ==================== */
 
 static const struct crypto_driver_api mindgrove_crypto_api = {
	 .hash_begin_session = mindgrove_sha_begin_session,
	 .hash_free_session = mindgrove_sha_free_session,
	 .hash_async_callback_set = NULL,
	 .query_hw_caps = mindgrove_sha_query_hw_caps,
 };
 
 /* ==================== */
 /* Device Instantiation */
 /* ==================== */
 
 #define MINDSGROVE_SHA_INIT(n)                               \
	 static const struct mindgrove_sha_config                 \
		 mindgrove_sha_config_##n = {                         \
			 .base = (SHA256_Type *)DT_INST_REG_ADDR(n),      \
	 };                                                       \
															 \
	 DEVICE_DT_INST_DEFINE(n,                                 \
						   mindgrove_sha_init,                \
						   NULL,                              \
						   NULL,                              \
						   &mindgrove_sha_config_##n,         \
						   POST_KERNEL,                       \
						   CONFIG_CRYPTO_INIT_PRIORITY,       \
						   &mindgrove_crypto_api);
 
 DT_INST_FOREACH_STATUS_OKAY(MINDSGROVE_SHA_INIT)