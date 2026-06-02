#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

/* --- 1. Function Declarations --- */
#if defined(CONFIG_AES_CTR_GFSBOX)
int test_aes_ctr_gfsbox(void);
#endif

/* Modified to track specific bit lengths for KEYSBOX */
#if defined(CONFIG_AES_CTR_KEYSBOX_128) || \
    defined(CONFIG_AES_CTR_KEYSBOX_192) || \
    defined(CONFIG_AES_CTR_KEYSBOX_256)
int test_aes_ctr_keysbox(void);
#endif

#if defined(CONFIG_AES_CTR_MMT)
int test_aes_ctr_mmt(void);
#endif

#if defined(CONFIG_AES_CTR_VARKEY_128_01) || defined(CONFIG_AES_CTR_VARKEY_128_02) || \
    defined(CONFIG_AES_CTR_VARKEY_128_03) || defined(CONFIG_AES_CTR_VARKEY_128_04) || \
    defined(CONFIG_AES_CTR_VARKEY_128_05) || defined(CONFIG_AES_CTR_VARKEY_128_06)
int test_aes_ctr_varkey_128(void);
#endif

#if defined(CONFIG_AES_CTR_VARKEY_192_01) || defined(CONFIG_AES_CTR_VARKEY_192_02) || \
    defined(CONFIG_AES_CTR_VARKEY_192_03) || defined(CONFIG_AES_CTR_VARKEY_192_04) || \
    defined(CONFIG_AES_CTR_VARKEY_192_05) || defined(CONFIG_AES_CTR_VARKEY_192_06) || \
    defined(CONFIG_AES_CTR_VARKEY_192_07) || defined(CONFIG_AES_CTR_VARKEY_192_08)
int test_aes_ctr_varkey_192(void);
#endif

#if defined(CONFIG_AES_CTR_VARKEY_256_01) || defined(CONFIG_AES_CTR_VARKEY_256_02) || \
    defined(CONFIG_AES_CTR_VARKEY_256_03) || defined(CONFIG_AES_CTR_VARKEY_256_04) || \
    defined(CONFIG_AES_CTR_VARKEY_256_05) || defined(CONFIG_AES_CTR_VARKEY_256_06) || \
    defined(CONFIG_AES_CTR_VARKEY_256_07) || defined(CONFIG_AES_CTR_VARKEY_256_08) || \
    defined(CONFIG_AES_CTR_VARKEY_256_09) || defined(CONFIG_AES_CTR_VARKEY_256_10) || \
    defined(CONFIG_AES_CTR_VARKEY_256_11)
int test_aes_ctr_varkey_256(void);
#endif

#if defined(CONFIG_AES_CTR_VARTXT_128_01) || defined(CONFIG_AES_CTR_VARTXT_128_02) || \
    defined(CONFIG_AES_CTR_VARTXT_128_03) || defined(CONFIG_AES_CTR_VARTXT_128_04) || \
    defined(CONFIG_AES_CTR_VARTXT_128_05) || defined(CONFIG_AES_CTR_VARTXT_128_06)
int test_aes_ctr_vartxt_128(void);
#endif

#if defined(CONFIG_AES_CTR_VARTXT_192_01) || defined(CONFIG_AES_CTR_VARTXT_192_02) || \
    defined(CONFIG_AES_CTR_VARTXT_192_03) || defined(CONFIG_AES_CTR_VARTXT_192_04) || \
    defined(CONFIG_AES_CTR_VARTXT_192_05) || defined(CONFIG_AES_CTR_VARTXT_192_06)
int test_aes_ctr_vartxt_192(void);
#endif

#if defined(CONFIG_AES_CTR_VARTXT_256_01) || defined(CONFIG_AES_CTR_VARTXT_256_02) || \
    defined(CONFIG_AES_CTR_VARTXT_256_03) || defined(CONFIG_AES_CTR_VARTXT_256_04) || \
    defined(CONFIG_AES_CTR_VARTXT_256_05) || defined(CONFIG_AES_CTR_VARTXT_256_06)
int test_aes_ctr_vartxt_256(void);
#endif

/* --- 2. Test Runner Macro --- */
#define RUN(fn, label) \
    do { int _r = fn(); \
         printk(label ": %s\n", _r ? "FAILED" : "PASSED"); } while(0)

int main(void)
{
    printk("\n===== AES CTR Tests =====\n");

#if defined(CONFIG_AES_CTR_GFSBOX)
    RUN(test_aes_ctr_gfsbox,  "CTR GFSBOX");
#endif

    /* KEYSBOX Execution Blocks (Updated for dynamic labels based on configuration) */
#if defined(CONFIG_AES_CTR_KEYSBOX_128)
    RUN(test_aes_ctr_keysbox, "CTR KEYSBOX 128");
#endif
#if defined(CONFIG_AES_CTR_KEYSBOX_192)
    RUN(test_aes_ctr_keysbox, "CTR KEYSBOX 192");
#endif
#if defined(CONFIG_AES_CTR_KEYSBOX_256)
    RUN(test_aes_ctr_keysbox, "CTR KEYSBOX 256");
#endif

#if defined(CONFIG_AES_CTR_MMT)
    RUN(test_aes_ctr_mmt,     "CTR MMT");
#endif

    /* VARKEY 128 */
#if defined(CONFIG_AES_CTR_VARKEY_128_01) || defined(CONFIG_AES_CTR_VARKEY_128_02) || \
    defined(CONFIG_AES_CTR_VARKEY_128_03) || defined(CONFIG_AES_CTR_VARKEY_128_04) || \
    defined(CONFIG_AES_CTR_VARKEY_128_05) || defined(CONFIG_AES_CTR_VARKEY_128_06)
    RUN(test_aes_ctr_varkey_128, "CTR VARKEY 128");
#endif

    /* VARKEY 192 */
#if defined(CONFIG_AES_CTR_VARKEY_192_01) || defined(CONFIG_AES_CTR_VARKEY_192_02) || \
    defined(CONFIG_AES_CTR_VARKEY_192_03) || defined(CONFIG_AES_CTR_VARKEY_192_04) || \
    defined(CONFIG_AES_CTR_VARKEY_192_05) || defined(CONFIG_AES_CTR_VARKEY_192_06) || \
    defined(CONFIG_AES_CTR_VARKEY_192_07) || defined(CONFIG_AES_CTR_VARKEY_192_08)
    RUN(test_aes_ctr_varkey_192, "CTR VARKEY 192");
#endif

    /* VARKEY 256 */
#if defined(CONFIG_AES_CTR_VARKEY_256_01) || defined(CONFIG_AES_CTR_VARKEY_256_02) || \
    defined(CONFIG_AES_CTR_VARKEY_256_03) || defined(CONFIG_AES_CTR_VARKEY_256_04) || \
    defined(CONFIG_AES_CTR_VARKEY_256_05) || defined(CONFIG_AES_CTR_VARKEY_256_06) || \
    defined(CONFIG_AES_CTR_VARKEY_256_07) || defined(CONFIG_AES_CTR_VARKEY_256_08) || \
    defined(CONFIG_AES_CTR_VARKEY_256_09) || defined(CONFIG_AES_CTR_VARKEY_256_10) || \
    defined(CONFIG_AES_CTR_VARKEY_256_11)
    RUN(test_aes_ctr_varkey_256, "CTR VARKEY 256");
#endif

    /* VARTXT 128 */
#if defined(CONFIG_AES_CTR_VARTXT_128_01) || defined(CONFIG_AES_CTR_VARTXT_128_02) || \
    defined(CONFIG_AES_CTR_VARTXT_128_03) || defined(CONFIG_AES_CTR_VARTXT_128_04) || \
    defined(CONFIG_AES_CTR_VARTXT_128_05) || defined(CONFIG_AES_CTR_VARTXT_128_06)
    RUN(test_aes_ctr_vartxt_128, "CTR VARTXT 128");
#endif

    /* VARTXT 192 */
#if defined(CONFIG_AES_CTR_VARTXT_192_01) || defined(CONFIG_AES_CTR_VARTXT_192_02) || \
    defined(CONFIG_AES_CTR_VARTXT_192_03) || defined(CONFIG_AES_CTR_VARTXT_192_04) || \
    defined(CONFIG_AES_CTR_VARTXT_192_05) || defined(CONFIG_AES_CTR_VARTXT_192_06)
    RUN(test_aes_ctr_vartxt_192, "CTR VARTXT 192");
#endif

    /* VARTXT 256 */
#if defined(CONFIG_AES_CTR_VARTXT_256_01) || defined(CONFIG_AES_CTR_VARTXT_256_02) || \
    defined(CONFIG_AES_CTR_VARTXT_256_03) || defined(CONFIG_AES_CTR_VARTXT_256_04) || \
    defined(CONFIG_AES_CTR_VARTXT_256_05) || defined(CONFIG_AES_CTR_VARTXT_256_06)
    RUN(test_aes_ctr_vartxt_256, "CTR VARTXT 256");
#endif

    printk("\n===== AES CTR Tests Complete =====\n");
    return 0;
}