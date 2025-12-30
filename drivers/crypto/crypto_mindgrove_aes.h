#ifndef ZEPHYR_DRIVERS_CRYPTO_CRYPTO_MINDGROVE_AES_H
#define ZEPHYR_DRIVERS_CRYPTO_CRYPTO_MINDGROVE_AES_H

#include <stdint.h>

/* AES register layout */
typedef struct {
	volatile uint64_t AES_INPUT;
	volatile uint64_t RESERVED0[3];

	volatile uint64_t AES_KEY;
	volatile uint64_t RESERVED1[3];

	volatile uint64_t AES_OUTPUT;
	volatile uint64_t RESERVED2;

	volatile uint64_t AES_IV;
	volatile uint64_t RESERVED3;

	union {
		volatile uint8_t AES_CTRL;
		struct {
			volatile uint8_t ENCDEC : 1;
			volatile uint8_t KEYLEN : 2;
			volatile uint8_t MODE   : 3;
			volatile uint8_t END    : 1;
			volatile uint8_t        : 1;
		} AES_CTRL_b;
	};

	union {
		volatile uint8_t AES_STATUS;
		struct {
			volatile uint8_t CAN_TAKE_INPUT : 1;
			volatile uint8_t OUT_READY     : 1;
			volatile uint8_t               : 6;
		} AES_STATUS_b;
	};

	volatile uint16_t RESERVED_END;
} AES_Type;

/* Zephyr device config */
struct mindgrove_aes_config {
	AES_Type *base;
};

#endif
