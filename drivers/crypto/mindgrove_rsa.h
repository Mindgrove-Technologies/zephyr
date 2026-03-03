#ifndef MINDGROVE_RSA_HW_H
#define MINDGROVE_RSA_HW_H

#include <stdint.h>
// RSA Padding type
#define RSA_NULL_PAD         0
#define RSAES_PKCS1_v1_5_PAD 1
#define RSAES_OAEP_PAD       2
/* The raw hardware math function */
uint32_t RSA_Run(uint8_t *output, uint8_t *input, uint8_t *exp, uint8_t *mod);

#endif
