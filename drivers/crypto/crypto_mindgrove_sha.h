#ifndef ZEPHYR_DRIVERS_CRYPTO_CRYPTO_MINDGROVE_SHA_H
#define ZEPHYR_DRIVERS_CRYPTO_CRYPTO_MINDGROVE_SHA_H

#include <stdint.h>
#include <stdbool.h>

/* SHA256 Constants */
#define MINDGROVE_SHA256_BLOCK_SIZE     64 /* 512 bits in bytes */
#define MINDGROVE_SHA256_DIGEST_SIZE    32
#define MINDGROVE_SHA256_BITS_PER_BYTE  8
#define MINDGROVE_SHA256_BLOCK_BITS     (MINDGROVE_SHA256_BLOCK_SIZE * MINDGROVE_SHA256_BITS_PER_BYTE)
#define MINDGROVE_SHA256_MAX_INPUT_BITS 64 /* Based on register constraints */
#define SUCCESS                         0

/* Hardware register definitions */
typedef struct {
	/* Input/Output registers */
	volatile uint64_t SHA_INPUT;       /* Input message block */
	volatile uint64_t RESERVED[15];    /* Reserved space */
	volatile uint64_t SHA_OUTPUT;      /* Output hash */
	volatile uint64_t RESERVED_OUT[7]; /* More reserved space */

	/* Control register */
	union {
		volatile uint8_t SHA_CTRL;
		struct {
			volatile uint8_t CONT_PREHASH: 1; /* Continue with previous hash */
			volatile uint8_t RESERVED_CTRL: 7;
		} SHA_CTRL_b;
	};

	/* Status register */
	union {
		volatile uint8_t SHA_STATUS;
		struct {
			volatile uint8_t SHA_STATUS_READY: 1;     /* Ready for new input */
			volatile uint8_t SHA_STATUS_OUT_READY: 1; /* Output digest ready */
			volatile uint8_t RESERVED_STATUS: 6;
		} SHA_STATUS_b;
	};

	volatile uint16_t RESERVED_END;
} SHA256_Type;

/* Control register bits */
#define SHA_CTRL_CONT_PREHASH (1 << 0) /* Continue with previous hash */

/* Status register bits */
#define SHA_STATUS_READY     (1 << 0) /* Ready for new input */
#define SHA_STATUS_OUT_READY (1 << 1) /* Output digest ready */

#define byte_length 8

extern const int sha_block_length_bits;
extern const int sha_max_inputlen_bits;
#define SHA256_HASH_LEN 32

static volatile SHA256_Type *sha_reg;

struct mindgrove_sha_config {
	volatile SHA256_Type *regs;
};

uint16_t SHA256_Single_Run(unsigned char *sha_output, const unsigned char *input_text,
			   int input_len_bits);

#endif /* ZEPHYR_DRIVERS_CRYPTO_CRYPTO_MINDGROVE_SHA_H */
