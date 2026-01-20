#ifndef ZEPHYR_DRIVERS_CRYPTO_CRYPTO_MINDGROVE_AES_H
#define ZEPHYR_DRIVERS_CRYPTO_CRYPTO_MINDGROVE_AES_H

#include <stdint.h>


typedef struct {
    volatile uint64_t AES_INPUT;
    volatile uint64_t RESERVED1;
    volatile uint64_t RESERVED2;
    volatile uint64_t RESERVED3;

    volatile uint64_t AES_KEY;
    volatile uint64_t RESERVED4;
    volatile uint64_t RESERVED5;
    volatile uint64_t RESERVED6;

    volatile uint64_t AES_OUTPUT;
    volatile uint64_t RESERVED7;

    volatile uint64_t AES_IV;
    volatile uint64_t RESERVED8;

    union {
        volatile uint8_t AES_CTRL;
        struct {
            volatile uint8_t AES_CTRL_ENCDEC : 1;
            volatile uint8_t AES_CTRL_KEYLEN : 2;
            volatile uint8_t AES_CTRL_MODE   : 3;
            volatile uint8_t AES_CTRL_END    : 1;
            volatile uint8_t                : 1;
        } AES_CTRL_b;
    };

    union {
        volatile uint8_t AES_STATUS;
        struct {
            volatile uint8_t AES_STATUS_CAN_TAKE_INPUT : 1;
            volatile uint8_t AES_STATUS_OUTP_READY     : 1;
            volatile uint8_t                           : 6;
        } AES_STATUS_b;
    };

    volatile uint16_t RESERVED9;
} AES_Type;

/* Zephyr device config */
struct mindgrove_aes_config {
	AES_Type *aes_reg;
};

#endif