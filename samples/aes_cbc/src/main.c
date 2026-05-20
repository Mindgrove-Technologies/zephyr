#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

/* * NOTE ON MEMORY CONSTRAINTS: 
 * Due to strict hardware memory limits, only ONE CONFIG_AES_CBC_* test 
 * variant should be enabled in prj.conf / Kconfig at any given time.
 */

/* --- 1. Function Declarations --- */
#if defined(CONFIG_AES_CBC_GFSBOX)
int test_aes_cbc_gfsbox(void);
#endif

#if defined(CONFIG_AES_CBC_KEYSBOX_128) || \
    defined(CONFIG_AES_CBC_KEYSBOX_192) || \
    defined(CONFIG_AES_CBC_KEYSBOX_256)
int test_aes_cbc_keysbox(void);
#endif
#if defined(CONFIG_AES_CBC_MMT)
int test_aes_cbc_mmt(void);
#endif

#if defined(CONFIG_AES_CBC_VARKEY_128_01) || defined(CONFIG_AES_CBC_VARKEY_128_02) || \
    defined(CONFIG_AES_CBC_VARKEY_128_03) || defined(CONFIG_AES_CBC_VARKEY_128_04) || \
    defined(CONFIG_AES_CBC_VARKEY_128_05) || defined(CONFIG_AES_CBC_VARKEY_128_06)
int test_aes_cbc_varkey_128(void);
#endif

#if defined(CONFIG_AES_CBC_VARKEY_192_01) || defined(CONFIG_AES_CBC_VARKEY_192_02) || \
    defined(CONFIG_AES_CBC_VARKEY_192_03) || defined(CONFIG_AES_CBC_VARKEY_192_04) || \
    defined(CONFIG_AES_CBC_VARKEY_192_05) || defined(CONFIG_AES_CBC_VARKEY_192_06) || \
    defined(CONFIG_AES_CBC_VARKEY_192_07) || defined(CONFIG_AES_CBC_VARKEY_192_08)
int test_aes_cbc_varkey_192(void);
#endif

#if defined(CONFIG_AES_CBC_VARKEY_256_01) || defined(CONFIG_AES_CBC_VARKEY_256_02) || \
    defined(CONFIG_AES_CBC_VARKEY_256_03) || defined(CONFIG_AES_CBC_VARKEY_256_04) || \
    defined(CONFIG_AES_CBC_VARKEY_256_05) || defined(CONFIG_AES_CBC_VARKEY_256_06) || \
    defined(CONFIG_AES_CBC_VARKEY_256_07) || defined(CONFIG_AES_CBC_VARKEY_256_08) || \
    defined(CONFIG_AES_CBC_VARKEY_256_09) || defined(CONFIG_AES_CBC_VARKEY_256_10) || \
    defined(CONFIG_AES_CBC_VARKEY_256_11)
int test_aes_cbc_varkey_256(void);
#endif

#if defined(CONFIG_AES_CBC_VARTXT_128_01) || defined(CONFIG_AES_CBC_VARTXT_128_02) || \
    defined(CONFIG_AES_CBC_VARTXT_128_03) || defined(CONFIG_AES_CBC_VARTXT_128_04) || \
    defined(CONFIG_AES_CBC_VARTXT_128_05) || defined(CONFIG_AES_CBC_VARTXT_128_06)
int test_aes_cbc_vartxt_128(void);
#endif

#if defined(CONFIG_AES_CBC_VARTXT_192_01) || defined(CONFIG_AES_CBC_VARTXT_192_02) || \
    defined(CONFIG_AES_CBC_VARTXT_192_03) || defined(CONFIG_AES_CBC_VARTXT_192_04) || \
    defined(CONFIG_AES_CBC_VARTXT_192_05) || defined(CONFIG_AES_CBC_VARTXT_192_06)
int test_aes_cbc_vartxt_192(void);
#endif

#if defined(CONFIG_AES_CBC_VARTXT_256_01) || defined(CONFIG_AES_CBC_VARTXT_256_02) || \
    defined(CONFIG_AES_CBC_VARTXT_256_03) || defined(CONFIG_AES_CBC_VARTXT_256_04) || \
    defined(CONFIG_AES_CBC_VARTXT_256_05) || defined(CONFIG_AES_CBC_VARTXT_256_06)
int test_aes_cbc_vartxt_256(void);
#endif

/* --- 2. Test Runner Infrastructure --- */
#define RUN(fn, label) \
    do { int _r = fn(); \
         printk(label ": %s\n", _r ? "FAILED" : "PASSED"); } while(0)

int main(void)
{
    printk("\n===== AES CBC Single-Test Execution =====\n");

#if defined(CONFIG_AES_CBC_GFSBOX)
    RUN(test_aes_cbc_gfsbox,  "CBC GFSBOX");
#endif
#if defined(CONFIG_AES_CBC_KEYSBOX_128) || defined(CONFIG_AES_CBC_KEYSBOX_192) || defined(CONFIG_AES_CBC_KEYSBOX_256)
    RUN(test_aes_cbc_keysbox, "CBC KEYSBOX");
#endif
#if defined(CONFIG_AES_CBC_MMT)
    RUN(test_aes_cbc_mmt,     "CBC MMT");
#endif

    /* VARKEY 128 */
#if defined(CONFIG_AES_CBC_VARKEY_128_01)
    RUN(test_aes_cbc_varkey_128, "CBC VARKEY 128-01");
#endif
#if defined(CONFIG_AES_CBC_VARKEY_128_02)
    RUN(test_aes_cbc_varkey_128, "CBC VARKEY 128-02");
#endif
#if defined(CONFIG_AES_CBC_VARKEY_128_03)
    RUN(test_aes_cbc_varkey_128, "CBC VARKEY 128-03");
#endif
#if defined(CONFIG_AES_CBC_VARKEY_128_04)
    RUN(test_aes_cbc_varkey_128, "CBC VARKEY 128-04");
#endif
#if defined(CONFIG_AES_CBC_VARKEY_128_05)
    RUN(test_aes_cbc_varkey_128, "CBC VARKEY 128-05");
#endif
#if defined(CONFIG_AES_CBC_VARKEY_128_06)
    RUN(test_aes_cbc_varkey_128, "CBC VARKEY 128-06");
#endif

    /* VARKEY 192 */
#if defined(CONFIG_AES_CBC_VARKEY_192_01)
    RUN(test_aes_cbc_varkey_192, "CBC VARKEY 192-01");
#endif
#if defined(CONFIG_AES_CBC_VARKEY_192_02)
    RUN(test_aes_cbc_varkey_192, "CBC VARKEY 192-02");
#endif
#if defined(CONFIG_AES_CBC_VARKEY_192_03)
    RUN(test_aes_cbc_varkey_192, "CBC VARKEY 192-03");
#endif
#if defined(CONFIG_AES_CBC_VARKEY_192_04)
    RUN(test_aes_cbc_varkey_192, "CBC VARKEY 192-04");
#endif
#if defined(CONFIG_AES_CBC_VARKEY_192_05)
    RUN(test_aes_cbc_varkey_192, "CBC VARKEY 192-05");
#endif
#if defined(CONFIG_AES_CBC_VARKEY_192_06)
    RUN(test_aes_cbc_varkey_192, "CBC VARKEY 192-06");
#endif
#if defined(CONFIG_AES_CBC_VARKEY_192_07)
    RUN(test_aes_cbc_varkey_192, "CBC VARKEY 192-07");
#endif
#if defined(CONFIG_AES_CBC_VARKEY_192_08)
    RUN(test_aes_cbc_varkey_192, "CBC VARKEY 192-08");
#endif

    /* VARKEY 256 */
#if defined(CONFIG_AES_CBC_VARKEY_256_01)
    RUN(test_aes_cbc_varkey_256, "CBC VARKEY 256-01");
#endif
#if defined(CONFIG_AES_CBC_VARKEY_256_02)
    RUN(test_aes_cbc_varkey_256, "CBC VARKEY 256-02");
#endif
#if defined(CONFIG_AES_CBC_VARKEY_256_03)
    RUN(test_aes_cbc_varkey_256, "CBC VARKEY 256-03");
#endif
#if defined(CONFIG_AES_CBC_VARKEY_256_04)
    RUN(test_aes_cbc_varkey_256, "CBC VARKEY 256-04");
#endif
#if defined(CONFIG_AES_CBC_VARKEY_256_05)
    RUN(test_aes_cbc_varkey_256, "CBC VARKEY 256-05");
#endif
#if defined(CONFIG_AES_CBC_VARKEY_256_06)
    RUN(test_aes_cbc_varkey_256, "CBC VARKEY 256-06");
#endif
#if defined(CONFIG_AES_CBC_VARKEY_256_07)
    RUN(test_aes_cbc_varkey_256, "CBC VARKEY 256-07");
#endif
#if defined(CONFIG_AES_CBC_VARKEY_256_08)
    RUN(test_aes_cbc_varkey_256, "CBC VARKEY 256-08");
#endif
#if defined(CONFIG_AES_CBC_VARKEY_256_09)
    RUN(test_aes_cbc_varkey_256, "CBC VARKEY 256-09");
#endif
#if defined(CONFIG_AES_CBC_VARKEY_256_10)
    RUN(test_aes_cbc_varkey_256, "CBC VARKEY 256-10");
#endif
#if defined(CONFIG_AES_CBC_VARKEY_256_11)
    RUN(test_aes_cbc_varkey_256, "CBC VARKEY 256-11");
#endif

    /* VARTXT 128 */
#if defined(CONFIG_AES_CBC_VARTXT_128_01)
    RUN(test_aes_cbc_vartxt_128, "CBC VARTXT 128-01");
#endif
#if defined(CONFIG_AES_CBC_VARTXT_128_02)
    RUN(test_aes_cbc_vartxt_128, "CBC VARTXT 128-02");
#endif
#if defined(CONFIG_AES_CBC_VARTXT_128_03)
    RUN(test_aes_cbc_vartxt_128, "CBC VARTXT 128-03");
#endif
#if defined(CONFIG_AES_CBC_VARTXT_128_04)
    RUN(test_aes_cbc_vartxt_128, "CBC VARTXT 128-04");
#endif
#if defined(CONFIG_AES_CBC_VARTXT_128_05)
    RUN(test_aes_cbc_vartxt_128, "CBC VARTXT 128-05");
#endif
#if defined(CONFIG_AES_CBC_VARTXT_128_06)
    RUN(test_aes_cbc_vartxt_128, "CBC VARTXT 128-06");
#endif

    /* VARTXT 192 */
#if defined(CONFIG_AES_CBC_VARTXT_192_01)
    RUN(test_aes_cbc_vartxt_192, "CBC VARTXT 192-01");
#endif
#if defined(CONFIG_AES_CBC_VARTXT_192_02)
    RUN(test_aes_cbc_vartxt_192, "CBC VARTXT 192-02");
#endif
#if defined(CONFIG_AES_CBC_VARTXT_192_03)
    RUN(test_aes_cbc_vartxt_192, "CBC VARTXT 192-03");
#endif
#if defined(CONFIG_AES_CBC_VARTXT_192_04)
    RUN(test_aes_cbc_vartxt_192, "CBC VARTXT 192-04");
#endif
#if defined(CONFIG_AES_CBC_VARTXT_192_05)
    RUN(test_aes_cbc_vartxt_192, "CBC VARTXT 192-05");
#endif
#if defined(CONFIG_AES_CBC_VARTXT_192_06)
    RUN(test_aes_cbc_vartxt_192, "CBC VARTXT 192-06");
#endif

    /* VARTXT 256 */
#if defined(CONFIG_AES_CBC_VARTXT_256_01)
    RUN(test_aes_cbc_vartxt_256, "CBC VARTXT 256-01");
#endif
#if defined(CONFIG_AES_CBC_VARTXT_256_02)
    RUN(test_aes_cbc_vartxt_256, "CBC VARTXT 256-02");
#endif
#if defined(CONFIG_AES_CBC_VARTXT_256_03)
    RUN(test_aes_cbc_vartxt_256, "CBC VARTXT 256-03");
#endif
#if defined(CONFIG_AES_CBC_VARTXT_256_04)
    RUN(test_aes_cbc_vartxt_256, "CBC VARTXT 256-04");
#endif
#if defined(CONFIG_AES_CBC_VARTXT_256_05)
    RUN(test_aes_cbc_vartxt_256, "CBC VARTXT 256-05");
#endif
#if defined(CONFIG_AES_CBC_VARTXT_256_06)
    RUN(test_aes_cbc_vartxt_256, "CBC VARTXT 256-06");
#endif

    printk("\n===== Execution Complete =====\n");
    return 0;
}