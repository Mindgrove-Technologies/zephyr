#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#if defined(CONFIG_RSA_TEST_OAEP_DECRYPT_01)
int test_rsa_oaep_decrypt_01(void);
#endif
#if defined(CONFIG_RSA_TEST_OAEP_DECRYPT_02)
int test_rsa_oaep_decrypt_02(void);
#endif
#if defined(CONFIG_RSA_TEST_OAEP_ENCRYPT)
int test_rsa_oaep_encrypt(void);
#endif
#if defined(CONFIG_RSA_TEST_PKCS_ENC_DEC)
int test_rsa_pkcs_enc_dec(void);
#endif
#if defined(CONFIG_RSA_TEST_PKCS_SIGN_GEN)
int test_rsa_pkcs_sign_gen(void);
#endif
#if defined(CONFIG_RSA_TEST_PKCS_SIGN_VERIF)
int test_rsa_pkcs_sign_verif(void);
#endif
#if defined(CONFIG_RSA_TEST_PSS_SIGN_GEN)
int test_rsa_pss_sign_gen(void);
#endif
#if defined(CONFIG_RSA_TEST_PSS_SIGN_VERIF)
int test_rsa_pss_sign_verif(void);
#endif

#define RUN(fn, label)                                  \
    do {                                                \
        int _r = fn();                                  \
        printk(label ": %s\n", _r ? "FAILED" : "PASSED"); \
    } while (0)

int main(void)
{
    printk("\n===== Mindgrove RSA2048 Tests =====\n");

#if defined(CONFIG_RSA_TEST_OAEP_DECRYPT_01)
    RUN(test_rsa_oaep_decrypt_01, "OAEP DECRYPT 01");
#endif
#if defined(CONFIG_RSA_TEST_OAEP_DECRYPT_02)
    RUN(test_rsa_oaep_decrypt_02, "OAEP DECRYPT 02");
#endif
#if defined(CONFIG_RSA_TEST_OAEP_ENCRYPT)
    RUN(test_rsa_oaep_encrypt,    "OAEP ENCRYPT");
#endif
#if defined(CONFIG_RSA_TEST_PKCS_ENC_DEC)
    RUN(test_rsa_pkcs_enc_dec,    "PKCS ENC/DEC");
#endif
#if defined(CONFIG_RSA_TEST_PKCS_SIGN_GEN)
    RUN(test_rsa_pkcs_sign_gen,   "PKCS SIGN GEN");
#endif
#if defined(CONFIG_RSA_TEST_PKCS_SIGN_VERIF)
    RUN(test_rsa_pkcs_sign_verif, "PKCS SIGN VERIF");
#endif
#if defined(CONFIG_RSA_TEST_PSS_SIGN_GEN)
    RUN(test_rsa_pss_sign_gen,    "PSS SIGN GEN");
#endif
#if defined(CONFIG_RSA_TEST_PSS_SIGN_VERIF)
    RUN(test_rsa_pss_sign_verif,  "PSS SIGN VERIF");
#endif

    printk("\n===== RSA2048 Tests Complete =====\n");
    return 0;
}