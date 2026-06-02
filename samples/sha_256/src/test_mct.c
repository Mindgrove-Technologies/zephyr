#include <zephyr/kernel.h>
#include <zephyr/crypto/crypto.h>
#include <zephyr/crypto/hash.h>
#include <zephyr/sys/printk.h>
#include <string.h>
#include "test_helpers.h"

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