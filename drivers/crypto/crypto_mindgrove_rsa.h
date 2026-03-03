
#ifndef BSP_INCLUDE_RSA_H_
#define BSP_INCLUDE_RSA_H_

#ifdef __cplusplus
extern "C" {
#endif

#define byte_length 8
#define RSA_BASE    0x05000000UL

#include <stdint.h>
#include <stdbool.h>
#include "mindgrove_rsa.h"
#include "rsa_padding.h"
typedef struct { /*!< RSA Structure                                                             */
	// volatile  uint32_t  RESERVED[48];
	volatile uint64_t RSA_INPUT; /*!< RSA input register */

	volatile uint64_t RESERVED1;
	volatile uint64_t RESERVED2;
	volatile uint64_t RESERVED3;

	volatile uint64_t RSA_EXP; /*!< RSA exponent register */

	volatile uint64_t RESERVED4;
	volatile uint64_t RESERVED5;
	volatile uint64_t RESERVED6;

	volatile uint64_t RSA_MOD; /*!< RSA modulus register */

	volatile uint64_t RESERVED7;
	volatile uint64_t RESERVED8;
	volatile uint64_t RESERVED9;

	volatile uint64_t RSA_RSqrMODN; /*!< RSA modulus register */

	volatile uint64_t RESERVED10;
	volatile uint64_t RESERVED11;
	volatile uint64_t RESERVED12;

	volatile uint64_t RSA_OUTPUT; /*!< RSA modulus register */

	volatile uint64_t RESERVED13;
	volatile uint64_t RESERVED14;
	volatile uint64_t RESERVED15;
	volatile uint64_t RESERVED16;
	volatile uint64_t RESERVED17;
	volatile uint64_t RESERVED18;
	volatile uint64_t RESERVED19;

	union {
		volatile uint8_t RSA_STATUS; /*!< RSA status register */

		struct {
			volatile uint8_t RSA_OUTP_READY: 1;   /*!< RSA can give output   */
			volatile uint8_t RSA_STATUS_READY: 1; /*!< RSA is ready to take input */
			uint8_t: 6;
		} RSA_STATUS_b;
	};

	volatile uint8_t RESERVED20;
	volatile uint16_t RESERVED21;
} RSA_Type; /*!< Size = 196 (0xc4)                                                         */

// Padding mode structure
typedef struct {
	int padding_mode;
	unsigned char *label;
	uint64_t label_length;
} struct_rsa_padding;

#ifdef __cplusplus
}
#endif

#endif // BSP_INCLUDE_RSA_H_
