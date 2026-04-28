
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/crypto/crypto.h>
#include "zephyr/crypto/rsa.h" //Custom header for Zephyr RSA API definitions
#include "crypto_mindgrove_rsa.h"
#include "crypto_mindgrove_sha.h"
#include "rsa_padding.h"
#include "bignum.h"

#define DT_DRV_COMPAT mindgrove_rsa2048

struct mg_rsa_config {
	volatile RSA_Type *regs;
};

struct mg_rsa_data {
	bool in_use;
};


#define R2_STR                                                                                 \
	"10000000000000000000000000000000000000000000000"                                          \
	"00000000000000000000000000000000000000000000000"                                          \
	"00000000000000000000000000000000000000000000000"                                          \
	"00000000000000000000000000000000000000000000000"                                          \
	"00000000000000000000000000000000000000000000000"                                          \
	"00000000000000000000000000000000000000000000000"                                          \
	"00000000000000000000000000000000000000000000000"                                          \
	"00000000000000000000000000000000000000000000000"                                          \
	"00000000000000000000000000000000000000000000000"                                          \
	"00000000000000000000000000000000000000000000000"                                          \
	"00000000000000000000000000000000000000000000000"                                          \
	"00000000000000000000000000000000000000000000000"                                          \
	"00000000000000000000000000000000000000000000000"                                          \
	"00000000000000000000000000000000000000000000000"                                          \
	"00000000000000000000000000000000000000000000000"                                          \
	"00000000000000000000000000000000000000000000000"                                          \
	"00000000000000000000000000000000000000000000000"                                          \
	"00000000000000000000000000000000000000000000000"                                          \
	"00000000000000000000000000000000000000000000000"                                          \
	"00000000000000000000000000000000000000000000000"                                          \
	"00000000000000000000000000000000000000000000000"                                          \
	"00000000000000000000000000000000000000"

// Defining important variables
const int rsa_block_length_bits = 2048;
const int rsa_k = rsa_block_length_bits / (int)byte_length;
const int rsa_h_len = 256 / (int)byte_length;
const int rsa_sha_block_length_bits = 512;

RSA_Type *rsa_instance = (RSA_Type *)RSA_BASE;

static uint8_t hex_char_to_nibble(char c)
{
	if ('0' <= c && c <= '9') {
		return c - '0';
	}
	if ('a' <= c && c <= 'f') {
		return c - 'a' + 10;
	}
	if ('A' <= c && c <= 'F') {
		return c - 'A' + 10;
	}
	return 0xFF; // Invalid hex character
}

// Function to convert a hexadecimal string to a byte array
void hex_string_to_byte_array(const char *hex_str, uint8_t *byte_array, size_t *size)
{
	// Calculate the number of bytes in the hexadecimal string (half of its length)
	if (*size % 2 != 0 || size <= 0) {
		return;
	}

	*size = *size / 2;

	// Allocate memory for the byte array
	// *byte_array = (uint8_t *)malloc(*size * sizeof(uint8_t));
	if (byte_array == NULL) {
		
		return;
	}

	// Convert the hex string to bytes
	for (size_t i = 0; i < *size; i++) {
		// Convert each pair of characters to a byte
		uint8_t high = hex_char_to_nibble(hex_str[2 * i]);
		uint8_t low = hex_char_to_nibble(hex_str[2 * i + 1]);
		unsigned int byte_value;
		if (high == 0xFF || low == 0xFF) {
			
			return;
		}
		byte_array[i] = (high << 4) | low;
	}
	
}


/** @fn void do_checks_rsa(unsigned char *mod_text
		    , long int input_len_bits, long int exp_len_bits, long int mod_len_bits
		    , struct_rsa_padding rsa_padding)
 * @brief  A checking function for rsa - exits the code with error if there is one.
 * @details Four checks are made :
 * 1. Input, exp and mod lengths must be less than 2048 bits
 * 2. Mod length should be greater than 8 bits.
 * 3. Value of mod should be odd.
 * 4. Does input length check wrt to padding chosen
 * @param unsigned char *mod_text : pointer to the mod text
 * @param long int input_len_bits : length of input message in bits
 * @param long int exp_len_bits : length of exp in bits
 * @param long int mod_len_bits : length of mod in bits
 * @param struct_rsa_padding rsa_padding : The struct contains three variables :
 * int padding_mode : the padding which should be done : Choose one of 0 (RSA_NULL_PAD),
 1(RSAES_PKCS1_v1_5_PAD), 2 (RSAES_OAEP_PAD)
 * unsigned char * label : a random label string
 * uint64_t label_length : length of label string
 * @return Returns nothing. By the end of the function, you will have either passed the checks
 * or an error would have been thrown causing the program to exit.
 */
static int do_checks_rsa(unsigned char *mod_text, int input_len_bits, int exp_len_bits,
			  int mod_len_bits, struct_rsa_padding rsa_padding)
{
	// Length of input, exp, mod should be less than 2048 bits
	if ((input_len_bits > 2048) || (exp_len_bits > 2048) || (mod_len_bits > 2048)) {
		return -EINVAL;
	}

	// Length of mod should be atleast one octet long.
	if (mod_len_bits < 8) {
		return -EINVAL;
	}

	// Mod (N) should be odd
	if ((mod_text[(mod_len_bits / byte_length) - 1] % 2U) == 0U) {
		return -EINVAL;
	}

	// Checking if input length is apt for the rsa padding chosen
	if ((rsa_padding.padding_mode == RSAES_PKCS1_v1_5_PAD) &&
	    ((input_len_bits / byte_length) > (rsa_k - 11))) {
		return -EINVAL;
	} else if ((rsa_padding.padding_mode == RSAES_OAEP_PAD) &&
		   ((input_len_bits / byte_length) > (rsa_k - ((2 * rsa_h_len) - 2)))) {
		return -EINVAL;
	} else {
		// Nothing to do
	}
}

/** @fn void load_to_rsa(uint64_t *load_input, uint64_t *rsa_reg_to_load)
 * @brief Helper function to load the values of a register to an RSA hardware register.
 * @param uint64_t *load_input : pointer to array which will be loaded
 * @param uint64_t *rsa_reg_to_load : pointer to array which has to be loaded
 * @return Returns nothing.
 */
static void load_to_rsa(uint64_t *load_input, uint64_t *rsa_reg_to_load)
{
	for (int i = 0; i < 32; i++) {
		*(rsa_reg_to_load) = *(load_input + i);
	}
}
/** @fn void get_rsa_output(volatile uint64_t *rsa_output_reg_64, uint64_t *rsa_output)
 * @brief Used to read and return the output of the rsa encryption.
 * @details The function returns a pointer to access the 128 bits rsa output from the MSB.
 * @param volatile uint64_t *rsa_output_reg_64 : h/w addresss where the output will be generated
 * @param unsigned char *rsa_output : pointer to where the rsa output has to be stored
 * @return Returns nothing. Changes rsa_output in place.
 */
static void get_rsa_output(volatile uint64_t *rsa_output_reg_64, uint64_t *rsa_output)
{
	// rsa_output += 32;
	for (int output_registers_index = 0; output_registers_index < 32;
	     output_registers_index++) {
		uint64_t a = *(rsa_output_reg_64);
		// reverse_64bit(a);
		*rsa_output = a;
		rsa_output++;
	}
}

static void swap_endian_in_place(uint64_t *num)
{
	*num = ((*num >> 56) & 0x00000000000000FFULL) | ((*num >> 40) & 0x000000000000FF00ULL) |
	       ((*num >> 24) & 0x0000000000FF0000ULL) | ((*num >> 8) & 0x00000000FF000000ULL) |
	       ((*num << 8) & 0x000000FF00000000ULL) | ((*num << 24) & 0x0000FF0000000000ULL) |
	       ((*num << 40) & 0x00FF000000000000ULL) | ((*num << 56) & 0xFF00000000000000ULL);
}

static inline void zero_ext(uint64_t number, unsigned base)
{
	int i = 0;
	int j = 0;
	int sn = sizeof(number) * 2U;
	unsigned int intermediate = 0;
	unsigned int digits[sn];
	char val_string[100];
	for (j = 0; j < sn; j++) {
		if (number == 0U) {
			digits[j] = 0;
		} else {
			digits[j] = number % base;
			number /= base;
		}
	}
	for (i = sn - 1; i >= 0; i--) {
		if (digits[i] >= 10U) {
			intermediate = (unsigned int)('a' - 10);
		} else {
			intermediate = (unsigned int)'0';
		}
		// putchar(digits[i] + intermediate);
	}
}

/** @fn void RSA_Run(uint8_t *output,uint8_t *input,uint8_t *exp,uint8_t *mod,uint8_t* r2modn)
 * @brief The main function which runs the rsa algorithm on H/W
 * @details This function does a couple of things.
 * 1. It first creates pointers to rsa registers.
 * 2. It then passed on the user inputs to the RSA registers and waits for an output to be created.
 * 3. Once the output is created, the output is decoded as per required and the final output
 * 2. It then passed on the user inputs to the RSA registers and waits for an output to be created.
 * 3. Once the output is created, the output is decoded as per required and the final output
 * is the one that will be accessed by the users.
 * @param output : pointer to where the RSA output has to be stored
 * @param input : pointer to the message that has to be encoded
 * @param exp : pointer to the key text
 * @param mod : pointer to the mod text
 * @param r2modn : pointer to r2modn text
 * @return Returns nothing. Changes output in place.
 * @param output : pointer to where the RSA output has to be stored
 * @param input : pointer to the message that has to be encoded
 * @param exp : pointer to the key text
 * @param mod : pointer to the mod text
 * @param r2modn : pointer to r2modn text
 * @return Returns nothing. Changes output in place.
 */
uint32_t RSA_Run(uint8_t *output, uint8_t *input, uint8_t *exp, uint8_t *mod)
{
	size_t r2modn_size;
	bn_int modulus, r2modn_result;
	uint8_t load_input[256];
	char r2modn[256] __attribute__((aligned(16))) = {0};

	volatile uint8_t *status_reg = &rsa_instance->RSA_STATUS;

	volatile uint64_t *load_input_arr = (uint64_t *)load_input;
	uint64_t *output_arr = (uint64_t *)load_input;

	BN_INIT(&modulus);
	BN_INIT(&r2modn_result);

	// Convert the bytes value to bignum value
	BigNum_Read_Unsigned_Bin(&modulus, mod, 256);

	//  Calculating r2 mod n
	BigNum_Calculate_R2_Mod_N(&modulus, &r2modn_result);

	// Convert the bignum value to byte value
	BigNum_Unsigned_Bin_Size(&r2modn_result, &r2modn_size);
	

	BigNum_Write_Unsigned_Bin(&r2modn_result, (uint8_t *)r2modn + (256 - r2modn_size), 256);

	uint8_t *src[4] = {input, exp, mod, (uint8_t *)r2modn};
	uint64_t *dest[4] = {&rsa_instance->RSA_INPUT, &rsa_instance->RSA_EXP,
			     &rsa_instance->RSA_MOD, &rsa_instance->RSA_RSqrMODN};

	for (uint8_t j = 0; j < 4; j++) {
		
		for (uint16_t i = 0; i < 256; i++) {
			load_input[i] = src[j][i];
		}

		for (uint8_t i = 0; i < 32; i++) {
			swap_endian_in_place(&load_input_arr[i]);
			*(dest[j]) = load_input_arr[i];
		}
	}

	//  Wait for hardware to signal completion
	while (!(rsa_instance->RSA_STATUS & 1))
		;

	for (uint8_t i = 0; i < 32; i++) {
		output_arr[i] = (uint64_t)(rsa_instance->RSA_OUTPUT);
		// zero_ext is kept as per original logic, but its output is commented in the source
		zero_ext(output_arr[i], 16);
		swap_endian_in_place(&output_arr[i]);
	}

	// Copy the processed data from output_arr (which points to load_input) to final output
	memcpy(output, load_input, 256);

	return 0;
}


/* ============================= */
/* The "Invoke" API Implementation */
/* ============================= */

static int mg_rsa_invoke(const struct device *dev, enum rsa_mg_op op, enum rsa_mg_padding pad,
			 struct rsa_mg_pkt *pkt)
{
	uint8_t scratch_pad[256] __aligned(8);
	uint16_t rsa_ret = 0;
	int ret = 0;

	switch (op) {
	case RSA_MG_OP_SIGN:
		if (pad == RSA_MG_PAD_PKCS_V15) {
			ret = RSASSA_PKCS1_v1_5_Sign(scratch_pad, 256, pkt->in, pkt->in_len);
		} else {
			ret = RSASSA_PSS_Sign(scratch_pad, 256, pkt->in, pkt->in_len, pkt->salt,
					      pkt->salt_len, 1);
		}
		if (ret != 0) {
			goto exit;
		}

		rsa_ret = RSA_Run(pkt->out, scratch_pad, pkt->exp, pkt->mod);
		break;

	case RSA_MG_OP_VERIFY:
		rsa_ret = RSA_Run(scratch_pad, pkt->in, pkt->exp, pkt->mod);
		if (rsa_ret != 0) {
			ret = -EIO;
			goto exit;
		}

		if (pad == RSA_MG_PAD_PKCS_V15) {
			ret = RSASSA_PKCS1_v1_5_Verify(scratch_pad, 256, pkt->out, pkt->out_len);
		} else {
			ret = RSASSA_PSS_Verify(scratch_pad, 256, pkt->out, pkt->out_len,
						pkt->salt_len);
		}
		break;

	case RSA_MG_OP_ENCRYPT:
		if (pad == RSA_MG_PAD_OAEP) {
			ret = RSAES_OAEP_Encrypt(scratch_pad, 256, pkt->in, pkt->in_len, pkt->label,
						 pkt->label_len);
		} else {
			ret = RSAES_PKCS1_v1_5_Encrypt(scratch_pad, 256, pkt->in, pkt->in_len);
		}
		if (ret != 0) {
			goto exit;
		}

		rsa_ret = RSA_Run(pkt->out, scratch_pad, pkt->exp, pkt->mod);
		break;

	case RSA_MG_OP_DECRYPT:
		rsa_ret = RSA_Run(scratch_pad, pkt->in, pkt->exp, pkt->mod);
		if (rsa_ret != 0) {
			ret = -EIO;
			goto exit;
		}

		if (pad == RSA_MG_PAD_OAEP) {
			ret = RSAES_OAEP_Decrypt(scratch_pad, 256, pkt->out, &pkt->out_len,
						 pkt->label, pkt->label_len);
		} else {
			ret = RSAES_PKCS1_v1_5_Decrypt(pkt->out, &pkt->out_len, scratch_pad, 256);
		}
		break;

	default:
		ret = -ENOTSUP;
		break;
	}

exit:
	if (ret != 0 || rsa_ret != 0) {
		printk("RSA ERROR: ret=%d, hardware_ret=%u\n", ret, rsa_ret);
	} else {
		printk("RSA SUCCESS\n");
	}
	return ret;
}

/* ============================= */
/* Zephyr Registration          */
/* ============================= */

static const struct rsa_mg_driver_api mg_rsa_api = {
	.invoke = mg_rsa_invoke,
};

static int mg_rsa_init(const struct device *dev)
{

	return 0;
}

#define MG_RSA_INIT(n)                                                                             \
	static const struct mg_rsa_config mg_rsa_cfg_##n = {                                       \
		.regs = (volatile RSA_Type *)DT_INST_REG_ADDR(n),                                  \
	};                                                                                         \
	static struct mg_rsa_data mg_rsa_data_##n;                                                 \
	DEVICE_DT_INST_DEFINE(n, mg_rsa_init, NULL, &mg_rsa_data_##n, &mg_rsa_cfg_##n,             \
			      POST_KERNEL, CONFIG_CRYPTO_INIT_PRIORITY, &mg_rsa_api);

DT_INST_FOREACH_STATUS_OKAY(MG_RSA_INIT)
