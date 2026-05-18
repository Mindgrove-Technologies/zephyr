#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <autoconf.h>

#if defined(CONFIG_SHA256_TEST_SINGLE_KAT)
int test_sha256_single_block_kat(void);
#endif

#if defined(CONFIG_SHA256_TEST_MMT01)
int test_sha256_mmt_01(void);
#endif
#if defined(CONFIG_SHA256_TEST_MMT02)
int test_sha256_mmt_02(void);
#endif
#if defined(CONFIG_SHA256_TEST_MMT03)
int test_sha256_mmt_03(void);
#endif
#if defined(CONFIG_SHA256_TEST_MMT04)
int test_sha256_mmt_04(void);
#endif
#if defined(CONFIG_SHA256_TEST_MMT05)
int test_sha256_mmt_05(void);
#endif
#if defined(CONFIG_SHA256_TEST_MMT06)
int test_sha256_mmt_06(void);
#endif

#if defined(CONFIG_SHA256_TEST_MCT)
int test_sha_monte_carlo(void);
#endif

int main(void)
{
    printk("\n===== Mindgrove SHA256 Tests =====\n");

#if defined(CONFIG_SHA256_TEST_SINGLE_KAT)
    printk(test_sha256_single_block_kat() ? "SINGLE KAT: FAILED\n" : "SINGLE KAT: PASSED\n");
#endif

#if defined(CONFIG_SHA256_TEST_MMT01)
    printk(test_sha256_mmt_01() ? "MMT01: FAILED\n" : "MMT01: PASSED\n");
#endif
#if defined(CONFIG_SHA256_TEST_MMT02)
    printk(test_sha256_mmt_02() ? "MMT02: FAILED\n" : "MMT02: PASSED\n");
#endif
#if defined(CONFIG_SHA256_TEST_MMT03)
    printk(test_sha256_mmt_03() ? "MMT03: FAILED\n" : "MMT03: PASSED\n");
#endif
#if defined(CONFIG_SHA256_TEST_MMT04)
    printk(test_sha256_mmt_04() ? "MMT04: FAILED\n" : "MMT04: PASSED\n");
#endif
#if defined(CONFIG_SHA256_TEST_MMT05)
    printk(test_sha256_mmt_05() ? "MMT05: FAILED\n" : "MMT05: PASSED\n");
#endif
#if defined(CONFIG_SHA256_TEST_MMT06)
    printk(test_sha256_mmt_06() ? "MMT06: FAILED\n" : "MMT06: PASSED\n");
#endif

#if defined(CONFIG_SHA256_TEST_MCT)
    printk(test_sha_monte_carlo() ? "MCT: FAILED\n" : "MCT: PASSED\n");
#endif

    printk("\n===== SHA256 Tests Complete =====\n");
    return 0;
}