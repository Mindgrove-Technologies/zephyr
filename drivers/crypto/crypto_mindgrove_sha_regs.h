#ifndef ZEPHYR_DRIVERS_CRYPTO_CRYPTO_MINDGROVE_SHA_REGS_H_
#define ZEPHYR_DRIVERS_CRYPTO_CRYPTO_MINDGROVE_SHA_REGS_H_

#include <stdint.h>

struct mindgrove_sha256_regs {
	/* Input message block (64 bits at a time) */
	volatile uint64_t SHA_INPUT;        /* 0x000 */

	/* Reserved / padding registers */
	volatile uint64_t RESERVED1;
	volatile uint64_t RESERVED2;
	volatile uint64_t RESERVED3;
	volatile uint64_t RESERVED4;
	volatile uint64_t RESERVED5;
	volatile uint64_t RESERVED6;
	volatile uint64_t RESERVED7;
	volatile uint64_t RESERVED8;
	volatile uint64_t RESERVED9;
	volatile uint64_t RESERVED10;
	volatile uint64_t RESERVED11;
	volatile uint64_t RESERVED12;
	volatile uint64_t RESERVED13;
	volatile uint64_t RESERVED14;
	volatile uint64_t RESERVED15;

	/* Output hash (64 bits at a time) */
	volatile uint64_t SHA_OUTPUT;       /* output register */

	volatile uint64_t RESERVED16;
	volatile uint64_t RESERVED17;
	volatile uint64_t RESERVED18;
	volatile uint64_t RESERVED19;
	volatile uint64_t RESERVED20;
	volatile uint64_t RESERVED21;
	volatile uint64_t RESERVED22;

	/* Control register */
	union {
		volatile uint8_t SHA_CTRL;
		struct {
			volatile uint8_t CONT_PREHASH : 1;
			volatile uint8_t RESERVED    : 7;
		} SHA_CTRL_b;
	};

	/* Status register */
	union {
		volatile uint8_t SHA_STATUS;
		struct {
			volatile uint8_t SHA_STATUS_READY     : 1;
			volatile uint8_t SHA_STATUS_OUT_READY : 1;
			volatile uint8_t RESERVED             : 6;
		} SHA_STATUS_b;
	};

	volatile uint16_t RESERVED23;
} SHA256_Type;

#endif /* ZEPHYR_DRIVERS_CRYPTO_CRYPTO_MINDGROVE_SHA_REGS_H_ */
