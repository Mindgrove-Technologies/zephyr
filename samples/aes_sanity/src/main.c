#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/crypto/crypto.h>
#include <string.h>

static const uint8_t key[16] = {
	0x2b,0x7e,0x15,0x16,0x28,0xae,0xd2,0xa6,
	0xab,0xf7,0x15,0x88,0x09,0xcf,0x4f,0x3c
};

static const uint8_t plaintext[16] = {
	0x6b,0xc1,0xbe,0xe2,0x2e,0x40,0x9f,0x96,
	0xe9,0x3d,0x7e,0x11,0x73,0x93,0x17,0x2a
};

static const uint8_t expected[16] = {
	0x69,0xc4,0xe0,0xd8,0x6a,0x7b,0x04,0x30,
	0xd8,0xcd,0xb7,0x80,0x70,0xb4,0xc5,0x5a
};

void main(void)
{
	printk("AES sanity test start\n");

	const struct device *dev = DEVICE_DT_GET_ONE(mindgrove_aes);
	if (!device_is_ready(dev)) {
		printk("AES device not ready\n");
		return;
	}

	printk("AES device ready\n");

	uint8_t out[16];

	struct cipher_ctx ctx = {
		.keylen = sizeof(key),
		.key.bit_stream = (uint8_t *)key,
		.flags = CAP_RAW_KEY | CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS,
	};

	struct cipher_pkt pkt = {
		.in_buf = (uint8_t *)plaintext,
		.in_len = 16,
		.out_buf = out,
		.out_buf_max = 16,
	};

	if (cipher_begin_session(dev, &ctx,
				 CRYPTO_CIPHER_ALGO_AES,
				 CRYPTO_CIPHER_MODE_ECB,
				 CRYPTO_CIPHER_OP_ENCRYPT)) {
		printk("begin_session failed\n");
		return;
	}

	printk("Session started\n");
	

	if (cipher_block_op(&ctx, &pkt)) {
		printk("cipher op failed\n");
		return;
	}

	cipher_free_session(dev, &ctx);

	printk("Cipher done\n");


    printk("\n=== Output Byte Order Analysis ===\n");
    
    // Your ENC=0 output
    uint8_t enc0_output[16] = {
        0x3a, 0xd7, 0x7b, 0xb4, 0x0d, 0x7a, 0x36, 0x60,
        0xa8, 0x9e, 0xca, 0xf3, 0x24, 0x66, 0xef, 0x97
    };
    
    // Your ENC=1 output  
    uint8_t enc1_output[16] = {
        0x50, 0x05, 0x94, 0xe2, 0x0d, 0x6d, 0x6c, 0x8c,
        0x71, 0x6b, 0x66, 0x7c, 0x38, 0xf0, 0x85, 0xf1
    };
    
    // Expected output
    uint8_t expected[16] = {
        0x69, 0xc4, 0xe0, 0xd8, 0x6a, 0x7b, 0x04, 0x30,
        0xd8, 0xcd, 0xb7, 0x80, 0x70, 0xb4, 0xc5, 0x5a
    };
    
    printk("\n1. Try reversing byte order:\n");
    
    uint8_t enc0_reversed[16], enc1_reversed[16];
    for (int i = 0; i < 16; i++) {
        enc0_reversed[i] = enc0_output[15-i];
        enc1_reversed[i] = enc1_output[15-i];
    }
    
    printk("   ENC=0 reversed: ");
    for (int i = 0; i < 16; i++) printk("%02x ", enc0_reversed[i]);
    
    printk("\n   ENC=1 reversed: ");
    for (int i = 0; i < 16; i++) printk("%02x ", enc1_reversed[i]);
    
    printk("\n\n2. Try 64-bit word swapping:\n");
    
    uint8_t enc0_word_swapped[16], enc1_word_swapped[16];
    uint64_t *enc0_words = (uint64_t*)enc0_output;
    uint64_t *enc1_words = (uint64_t*)enc1_output;
    uint64_t *enc0_out = (uint64_t*)enc0_word_swapped;
    uint64_t *enc1_out = (uint64_t*)enc1_word_swapped;
    
    enc0_out[0] = enc0_words[1];
    enc0_out[1] = enc0_words[0];
    enc1_out[0] = enc1_words[1];
    enc1_out[1] = enc1_words[0];
    
    printk("   ENC=0 word swapped: ");
    for (int i = 0; i < 16; i++) printk("%02x ", enc0_word_swapped[i]);
    
    printk("\n   ENC=1 word swapped: ");
    for (int i = 0; i < 16; i++) printk("%02x ", enc1_word_swapped[i]);
    
    printk("\n\n3. Check if output matches AES decryption:\n");
    
    // Maybe we're getting DECRYPTION output instead of encryption?
    // Expected decryption of the ciphertext would give us the plaintext back
    // But we're encrypting plaintext, so this doesn't make sense...


	if (memcmp(out, expected, 16) == 0) {
		printk("AES ECB PASS\n");
	} else {
		printk("AES ECB FAIL\n");
	}
}