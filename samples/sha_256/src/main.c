#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/crypto/crypto.h>
#include <zephyr/crypto/hash.h>
#include <zephyr/sys/printk.h>
#include <string.h>
#include "rsp_files.h"
#include <string.h>
#include <zephyr/kernel.h>

#define SHA256_HASH_LEN 32
#define SHA256_DIGEST_SIZE 32

/* HELPER FUNCTIONS */
static bool check_hash(const uint8_t *got, const uint8_t *expected, size_t len)
{
    return memcmp(got, expected, len) == 0;
}


/* Helper function to print hex */
static void print_hex(const char *label, const uint8_t *data, size_t len)
{
    printk("%s: ", label);
    for (size_t i = 0; i < len; i++) {
        printk("%02x", data[i]);
        if ((i + 1) % 32 == 0 && i + 1 < len) printk("\n          ");
    }
    printk("\n");
}

/* Helper function to print digest */
static void print_digest(const uint8_t *digest)
{
    for (int i = 0; i < SHA256_DIGEST_SIZE; i++) {
        printk("%02x", digest[i]);
    }
}

/* TEST FUNCTIONS */

/* Single-block KAT test for Zephyr */
static int test_sha256_single_block_kat(void)
{
    extern const sha256_kat_t kat_vectors[];
    extern const uint16_t kat_vectors_count;

    const struct device *dev = DEVICE_DT_GET_ONE(mindgrove_sha256);
    if (!device_is_ready(dev)) {
        printk("SHA256 device not ready\n");
        return -ENODEV;
    }

    for (uint16_t i = 0; i < kat_vectors_count; i++) {
        const sha256_kat_t *tv = &kat_vectors[i];
        uint8_t output[SHA256_HASH_LEN] = {0};

        struct hash_ctx ctx;
        struct hash_pkt pkt;

        /* Begin session */
        if (hash_begin_session(dev, &ctx, CRYPTO_HASH_ALGO_SHA256)) {
            printk("hash_begin_session failed\n");
            return -EIO;
        }

        uint32_t total_len_bits = tv->input_text_len;
        uint32_t total_len_bytes = total_len_bits / 8;

        if (total_len_bits == 512) {
            /* * SPECIAL CASE: 512 bits (exactly one block).
             * We call compute directly. This forces the driver to handle 
             * the 64 bytes of data and the mandatory second padding block 
             * in one single logical operation.
             */
            pkt.in_buf = (uint8_t *)tv->input_text;
            pkt.in_len = 64;
            pkt.out_buf = output;

            if (hash_compute(&ctx, &pkt)) {
                printk("hash_compute (512-bit special) failed\n");
                hash_free_session(dev, &ctx);
                return -EIO;
            }
        } else {
            /* * GENERAL CASE: < 512 or > 512 bits.
             * Standard streaming approach.
             */
            uint32_t sent_bytes = 0;
            while ((total_len_bytes - sent_bytes) > 64) {
                pkt.in_buf = (uint8_t *)&tv->input_text[sent_bytes];
                pkt.in_len = 64;
                pkt.out_buf = NULL;
                if (hash_update(&ctx, &pkt)) {
                    hash_free_session(dev, &ctx);
                    return -EIO;
                }
                sent_bytes += 64;
            }

            pkt.in_buf = (uint8_t *)&tv->input_text[sent_bytes];
            pkt.in_len = total_len_bytes - sent_bytes;
            pkt.out_buf = output;
            if (hash_compute(&ctx, &pkt)) {
                hash_free_session(dev, &ctx);
                return -EIO;
            }
        }

        hash_free_session(dev, &ctx);

        /* Validate output */
        bool pass = true;
        for (uint8_t j = 0; j < SHA256_HASH_LEN; j++) {
            if (output[j] != tv->output_hash[j]) {
                printk("KAT #%d FAILED. Expected: %02x Got: %02x\n",
                       i + 1, tv->output_hash[j], output[j]);
                pass = false;
                break;
            }
        }

        if (pass) {
            printk("KAT #%d PASS\n", i + 1);
        } else {
            return -EFAULT;
        }
    }

    return 0;
}

/* Multi-block KAT test for Zephyr */

static int test_sha256_multi_block_kat_fixed(void)
{
    extern const sha256_multi_kat_t multi_kat_vectors[];
    extern const uint16_t multi_kat_vectors_count;

    const struct device *dev = DEVICE_DT_GET_ONE(mindgrove_sha256);
    if (!device_is_ready(dev)) {
        printk("SHA256 device not ready\n");
        return -ENODEV;
    }

    int total_pass = 0;

    for (int v_idx = 0; v_idx < 64; v_idx++) {
        const sha256_multi_kat_t *tv = &multi_kat_vectors[v_idx];
        uint8_t output[SHA256_HASH_LEN] = {0};
        struct hash_ctx ctx;
        struct hash_pkt pkt;

        printk("\n=== Testing Vector %d (%u bits) ===\n", v_idx, tv->input_text_len);

        /* Begin session */
        if (hash_begin_session(dev, &ctx, CRYPTO_HASH_ALGO_SHA256)) {
            printk("Session begin failed for vector %d\n", v_idx);
            return -EIO;
        }

        uint32_t total_bits = tv->input_text_len;
        uint32_t sent_bits = 0;
        
        /* Send complete 512-bit blocks via update */
        while ((total_bits - sent_bits) > 512) {
            uint32_t bytes_to_send = 512 / 8;
            uint32_t byte_offset = sent_bits / 8;
            
            pkt.in_buf = (uint8_t *)&tv->input_text[byte_offset];
            pkt.in_len = bytes_to_send;
            pkt.out_buf = NULL;
            
            if (hash_update(&ctx, &pkt)) {
                printk("hash_update failed at offset %u\n", byte_offset);
                hash_free_session(dev, &ctx);
                return -EIO;
            }
            sent_bits += 512;
        }

        /* Send FINAL block via compute */
        uint32_t remaining_bits = total_bits - sent_bits;
        uint32_t remaining_bytes = (remaining_bits + 7) / 8;
        uint32_t byte_offset = sent_bits / 8;
        
        pkt.in_buf = (remaining_bytes > 0) ? (uint8_t *)&tv->input_text[byte_offset] : NULL;
        pkt.in_len = remaining_bytes;
        pkt.out_buf = output;
        
        if (hash_compute(&ctx, &pkt)) {
            printk("hash_compute failed for final block\n");
            hash_free_session(dev, &ctx);
            return -EIO;
        }

        hash_free_session(dev, &ctx);

        /* Verify Result */
        bool match = true;
        for (int i = 0; i < 32; i++) {
            if (output[i] != tv->output_hash[i]) {
                match = false;
                break;
            }
        }

        if (match) {
            printk("Vector %d: PASS\n", v_idx);
            total_pass++;
        } else {
            printk("Vector %d: FAIL\n", v_idx);
            printk("Expected: ");
            for(int i=0; i<32; i++) printk("%02x", tv->output_hash[i]);
            printk("\nActual:   ");
            for(int i=0; i<32; i++) printk("%02x", output[i]);
            printk("\n");
            return -EFAULT; // Stop on first failure for debugging
        }
    }

    printk("\nAll %d vectors passed!\n", total_pass);
    return 0;
}

/* Monte Carlo test using proper streaming API */
int test_sha_monte_carlo(void)
{
    const struct device *dev = DEVICE_DT_GET_ONE(mindgrove_sha256);
    
    if (!device_is_ready(dev)) {
        printk("SHA device not ready\n");
        return -ENODEV;
    }

    uint8_t Seed[SHA256_DIGEST_SIZE] = {
        0x6d, 0x1e, 0x72, 0xad, 0x03, 0xdd, 0xeb, 0x5d,
        0xe8, 0x91, 0xe5, 0x72, 0xe2, 0x39, 0x6f, 0x8d,
        0xa0, 0x15, 0xd8, 0x99, 0xef, 0x0e, 0x79, 0x50,
        0x31, 0x52, 0xd6, 0x01, 0x0a, 0x3f, 0xe6, 0x91};

    uint8_t MD0[SHA256_DIGEST_SIZE];
    uint8_t MD1[SHA256_DIGEST_SIZE];
    uint8_t MD2[SHA256_DIGEST_SIZE];
    uint8_t MDnew[SHA256_DIGEST_SIZE];
    
    /* Buffer for Mi = MD[i-3] || MD[i-2] || MD[i-1] */
    uint8_t Mi[3 * SHA256_DIGEST_SIZE]; // 96 bytes
    
    struct hash_ctx ctx;
    struct hash_pkt pkt;
    int ret;
    
    printk("Starting SHA256 Monte Carlo test...\n");

    /*------------------------------------------------------------
     * Perform 100 outer iterations
     *------------------------------------------------------------*/
    for (int j = 0; j < 100; j++) {
        /* Initialize MD0, MD1, MD2 = Seed */
        memcpy(MD0, Seed, SHA256_DIGEST_SIZE);
        memcpy(MD1, Seed, SHA256_DIGEST_SIZE);
        memcpy(MD2, Seed, SHA256_DIGEST_SIZE);

        /*--------------------------------------------------------
         * Perform 1000 rounds
         *--------------------------------------------------------*/
        for (int i = 3; i < 1003; i++) {
            /* Mi = MD[i-3] || MD[i-2] || MD[i-1] */
            memcpy(Mi, MD0, SHA256_DIGEST_SIZE);
            memcpy(Mi + SHA256_DIGEST_SIZE, MD1, SHA256_DIGEST_SIZE);
            memcpy(Mi + 2 * SHA256_DIGEST_SIZE, MD2, SHA256_DIGEST_SIZE);

            /* ============================================
             * KEY FIX: Use hash_update() + hash_final()
             * instead of single hash_hndlr() call
             * ============================================ */
            
            /* Begin a NEW session for each independent SHA256 operation */
            ret = hash_begin_session(dev, &ctx, CRYPTO_HASH_ALGO_SHA256);
            if (ret != 0) {
                printk("Failed to begin session at i=%d: %d\n", i, ret);
                return ret;
            }

            /* STEP 1: hash_update() for first 512 bits (64 bytes) */
            memset(&pkt, 0, sizeof(pkt));
            pkt.in_buf = Mi;
            pkt.in_len = 64;  // 512 bits = 64 bytes
            ret = hash_update(&ctx, &pkt);
            if (ret != 0) {
                printk("hash_update failed at j=%d, i=%d: %d\n", j, i, ret);
                hash_free_session(dev, &ctx);
                return ret;
            }

            /* STEP 2: hash_final() for remaining 256 bits (32 bytes) */
            memset(&pkt, 0, sizeof(pkt));
            pkt.in_buf = Mi + 64;  // Remaining 32 bytes
            pkt.in_len = 32;       // 256 bits = 32 bytes
            pkt.out_buf = MDnew;
            ret = hash_compute(&ctx, &pkt);
            if (ret != 0) {
                printk("hash_final failed at j=%d, i=%d: %d\n", j, i, ret);
                hash_free_session(dev, &ctx);
                return ret;
            }

            /* Free session - each SHA256(Mi) is independent */
            hash_free_session(dev, &ctx);

            /* Shift the MD buffers */
            memcpy(MD0, MD1, SHA256_DIGEST_SIZE);
            memcpy(MD1, MD2, SHA256_DIGEST_SIZE);
            memcpy(MD2, MDnew, SHA256_DIGEST_SIZE);
        }

        /* End of 1000 rounds: Seed = MD1002 (MD2) */
        memcpy(Seed, MD2, SHA256_DIGEST_SIZE);

        /* Print checkpoint */
        printk("COUNT = %d\n", j);
        printk("MD = ");
        print_digest(MD2);
        printk("\n");

    }
    // NIST Expected Result for Outer Loop 0
    uint8_t expected_0[32] = {
        0x6a, 0x91, 0x2b, 0xa4, 0x18, 0x83, 0x91, 0xa7,
        0x8e, 0x6f, 0x13, 0xd8, 0x8e, 0xd2, 0xd1, 0x4e,
        0x13, 0xaf, 0xce, 0x9d, 0xb6, 0xf7, 0xdc, 0xbf,
        0x4a, 0x48, 0xc2, 0x4f, 0x3d, 0xb0, 0x27, 0x78
    };
    if (memcmp(MD2, expected_0, 32) == 0) {
        printk(" -> [PASS] MATCHES NIST GROUND TRUTH\n");
        return 0;
    } else {
        printk(" -> [FAIL] DOES NOT MATCH NIST\n");
        return EFAULT;
    }
}

/* You can call this from your main function */
int main(void)
{
    printk("\n===== Mindgrove SHA256 Tests =====\n");

     if(test_sha256_single_block_kat()){
        printk("SHA256 SHORT MESSAGE KAT FAILED\n");
    }
    else{
        printk("SHA256 SHORT MESSAGE KAT PASSED\n");
    }

    if(test_sha256_multi_block_kat_fixed()){
        printk("SHA256 MULTI MESSAGE KAT FAILED\n");
    }
    else{
        printk("SHA256 MULTI MESSAGE KAT PASSED\n");
    }

    if(test_sha_monte_carlo()){
        printk("SHA256 MONTE CARLO TEST FAILED\n");
    }
    else{
        printk("SHA256 MONTE CARLO TEST  PASSED\n");
    }

    printk("\n===== SHA256 Tests Complete =====\n");
    
}