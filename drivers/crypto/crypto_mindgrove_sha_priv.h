#include "crypto_mindgrove_sha_regs.h"

struct mindgrove_sha_dev_data {
	volatile struct mindgrove_sha256_regs *regs;
	bool in_use;
	long int iterated_length_bits;
};
