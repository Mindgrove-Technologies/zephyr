#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/crypto/crypto.h>
#include <zephyr/crypto/rsa.h>
#include <zephyr/sys/printk.h>
#include <string.h>
#include "rsp_files.h"

#define RSA_BYTES 256

static const struct device *get_rsa_dev(void)
{
    const struct device *dev = DEVICE_DT_GET(DT_INST(0, mindgrove_rsa2048));
    if (!device_is_ready(dev)) {
        printk("RSA device not ready\n");
        return NULL;
    }
    return dev;
}

/* ============================================================
 * PKCS1 v1.5 Encrypt / Decrypt KAT
 * ============================================================ */
int PKCS_Encrypt_Decrypt_KATS(void)
{
    const struct device *dev = get_rsa_dev();
    if (!dev) return -ENODEV;

    const struct rsa_mg_driver_api *api = dev->api;
    uint32_t passed = 0, failed = 0;

    for (uint16_t i = 0; i < sign_kat_vectors_count; i++) {

        const rsa_sign_test_vectors *tv = &sign_kat_vectors[i];
        uint8_t ciphertext[RSA_BYTES] __aligned(8) = {0};
        uint8_t decrypted[RSA_BYTES] __aligned(8) = {0};
        struct rsa_mg_pkt pkt = {0};

        /* Encrypt */
        pkt.in = tv->input;
        pkt.in_len = tv->input_len_bits / 8;
        pkt.out = ciphertext;
        pkt.out_len = RSA_BYTES;
        pkt.exp = tv->public_exponent;
        pkt.mod = tv->modulus;

        int ret = api->invoke(dev, RSA_MG_OP_ENCRYPT,
                              RSA_MG_PAD_PKCS_V15, &pkt);
        if (ret) { failed++; continue; }

        /* Decrypt */
        pkt.in = ciphertext;
        pkt.in_len = RSA_BYTES;
        pkt.out = decrypted;
        pkt.out_len = RSA_BYTES;
        pkt.exp = tv->private_exponent;

        ret = api->invoke(dev, RSA_MG_OP_DECRYPT,
                          RSA_MG_PAD_PKCS_V15, &pkt);

        bool ok = (!ret &&
                  pkt.out_len == tv->input_len_bits / 8 &&
                  memcmp(decrypted, tv->input,
                         pkt.out_len) == 0);

        printk("[PKCS ENC %03d] %s\n", i, ok ? "PASS" : "FAIL");
        ok ? passed++ : failed++;
    }

    printk("\nPKCS ENC/DEC: %d Passed, %d Failed\n\n", passed, failed);
    return failed ? -EIO : 0;
}

/* ============================================================
 * OAEP Encrypt / Decrypt KAT
 * ============================================================ */
int OAEP_Encrypt_Decrypt_KATS(void)
{
    const struct device *dev = get_rsa_dev();
    if (!dev) return -ENODEV;

    const struct rsa_mg_driver_api *api = dev->api;
    uint32_t passed = 0, failed = 0;

    for (uint16_t i = 0; i < encrypt_decrypt_oaep_vectors_count; i++) {

        const rsa_encrypt_decrypt_oaep_test_vectors *tv =
            &encrypt_decrypt_oaep_vectors[i];

        uint8_t decrypted[RSA_BYTES] __aligned(8) = {0};
        struct rsa_mg_pkt pkt = {0};

        pkt.in = (uint8_t *)tv->cipher_text;
        pkt.in_len = RSA_BYTES;
        pkt.out = decrypted;
        pkt.out_len = RSA_BYTES;
        pkt.exp = (uint8_t *)tv->private_exponent;
        pkt.mod = (uint8_t *)tv->modulus;
        pkt.label = tv->label_len_bits ?
                    (uint8_t *)tv->label : NULL;
        pkt.label_len = tv->label_len_bits / 8;

        int ret = api->invoke(dev, RSA_MG_OP_DECRYPT,
                              RSA_MG_PAD_OAEP, &pkt);

        bool ok = false;

        if (tv->expected_result == 'P') {
            ok = (!ret &&
                  pkt.out_len == tv->input_len_bits / 8 &&
                  memcmp(tv->input, decrypted,
                         pkt.out_len) == 0);
        } else {
            ok = (ret != 0);
        }

        printk("[OAEP %03d] %s\n", i, ok ? "PASS" : "FAIL");
        ok ? passed++ : failed++;
    }

    printk("\nOAEP: %d Passed, %d Failed\n\n", passed, failed);
    return failed ? -EIO : 0;
}

/* ============================================================
 * PKCS1 v1.5 Sign Generate KAT
 * ============================================================ */
int PKCS_Sign_Generate_KATS(void)
{
    const struct device *dev = get_rsa_dev();
    if (!dev) return -ENODEV;

    const struct rsa_mg_driver_api *api = dev->api;
    uint32_t passed = 0, failed = 0;

    for (uint16_t i = 0; i < sign_kat_vectors_count; i++) {

        const rsa_sign_test_vectors *tv = &sign_kat_vectors[i];
        uint8_t sig[RSA_BYTES] __aligned(8) = {0};
        struct rsa_mg_pkt pkt = {0};

        pkt.in = tv->input;
        pkt.in_len = tv->input_len_bits / 8;
        pkt.out = sig;
        pkt.out_len = RSA_BYTES;
        pkt.exp = tv->private_exponent;
        pkt.mod = tv->modulus;

        int ret = api->invoke(dev, RSA_MG_OP_SIGN,
                              RSA_MG_PAD_PKCS_V15, &pkt);

        bool ok = (!ret &&
                   memcmp(sig, tv->signature, RSA_BYTES) == 0);

        printk("[PKCS SIGN %03d] %s\n", i, ok ? "PASS" : "FAIL");
        ok ? passed++ : failed++;
    }

    printk("\nPKCS SIGN GEN: %d Passed, %d Failed\n\n",
           passed, failed);
    return failed ? -EIO : 0;
}

/* ============================================================
 * PKCS1 v1.5 Verify KAT
 * ============================================================ */
int PKCS_Sign_Verify_KATS(void)
{
    const struct device *dev = get_rsa_dev();
    if (!dev) return -ENODEV;

    const struct rsa_mg_driver_api *api = dev->api;
    uint32_t passed = 0, failed = 0;

    for (uint16_t i = 0; i < verif_kat_vectors_count; i++) {

        const rsa_verif_test_vectors *tv = &verif_kat_vectors[i];
        struct rsa_mg_pkt pkt = {0};

        pkt.in = (uint8_t *)tv->signature;
        pkt.in_len = RSA_BYTES;
        pkt.out = (uint8_t *)tv->input;
        pkt.out_len = tv->input_len_bits / 8;
        pkt.exp = (uint8_t *)tv->public_exponent;
        pkt.mod = (uint8_t *)tv->modulus;

        int ret = api->invoke(dev, RSA_MG_OP_VERIFY,
                              RSA_MG_PAD_PKCS_V15, &pkt);

        bool ok = (tv->expected_result == 'P') ?
                  (ret == 0) : (ret != 0);

        printk("[PKCS VERIFY %03d] %s\n",
               i, ok ? "PASS" : "FAIL");
        ok ? passed++ : failed++;
    }

    printk("\nPKCS VERIFY: %d Passed, %d Failed\n\n",
           passed, failed);
    return failed ? -EIO : 0;
}

/* ============================================================
 * PSS Sign Generate KAT
 * ============================================================ */
int PSS_Sign_Generate_KATS(void)
{
    const struct device *dev = get_rsa_dev();
    if (!dev) return -ENODEV;

    const struct rsa_mg_driver_api *api = dev->api;
    uint32_t passed = 0, failed = 0;

    for (uint16_t i = 0; i < sign_kat_pss_vectors_count; i++) {

        const rsa_sign_pss_pad_test_vectors *tv =
            &sign_kat_pss_vectors[i];

        uint8_t sig[RSA_BYTES] __aligned(8) = {0};
        struct rsa_mg_pkt pkt = {0};

        pkt.in = tv->input;
        pkt.in_len = tv->input_len_bits / 8;
        pkt.out = sig;
        pkt.out_len = RSA_BYTES;
        pkt.exp = tv->private_exponent;
        pkt.mod = tv->modulus;
        pkt.salt = tv->salt;
        pkt.salt_len = tv->salt_len_bits / 8;

        int ret = api->invoke(dev, RSA_MG_OP_SIGN,
                              RSA_MG_PAD_PSS, &pkt);

        bool ok = (!ret &&
                   memcmp(sig, tv->signature, RSA_BYTES) == 0);

        printk("[PSS SIGN %03d] %s\n",
               i, ok ? "PASS" : "FAIL");
        ok ? passed++ : failed++;
    }

    printk("\nPSS SIGN GEN: %d Passed, %d Failed\n\n",
           passed, failed);
    return failed ? -EIO : 0;
}

/* ============================================================
 * PSS Verify KAT
 * ============================================================ */
int PSS_Sign_Verify_KATS(void)
{
    const struct device *dev = get_rsa_dev();
    if (!dev) return -ENODEV;

    const struct rsa_mg_driver_api *api = dev->api;
    uint32_t passed = 0, failed = 0;

    for (uint16_t i = 0; i < verif_kat_pss_vectors_count; i++) {

        const rsa_verif_pss_pad_test_vectors *tv =
            &verif_kat_pss_vectors[i];

        struct rsa_mg_pkt pkt = {0};

        pkt.in = (uint8_t *)tv->signature;
        pkt.in_len = RSA_BYTES;
        pkt.out = (uint8_t *)tv->input;
        pkt.out_len = tv->input_len_bits / 8;
        pkt.exp = (uint8_t *)tv->public_exponent;
        pkt.mod = (uint8_t *)tv->modulus;
        pkt.salt_len = tv->salt_len_bits / 8;

        int ret = api->invoke(dev, RSA_MG_OP_VERIFY,
                              RSA_MG_PAD_PSS, &pkt);

        bool ok = (tv->expected_result == 'P') ?
                  (ret == 0) : (ret != 0);

        printk("[PSS VERIFY %03d] %s\n",
               i, ok ? "PASS" : "FAIL");
        ok ? passed++ : failed++;
    }

    printk("\nPSS VERIFY: %d Passed, %d Failed\n\n",
           passed, failed);
    return failed ? -EIO : 0;
}

/* ============================================================
 * MAIN
 * ============================================================ */
int main(void)
{
    printk("\n===== Mindgrove RSA2048 Tests =====\n");

    if (!PKCS_Encrypt_Decrypt_KATS())
        printk("PKCS ENC/DEC PASSED\n");
    else
        printk("PKCS ENC/DEC FAILED\n");

    if (!OAEP_Encrypt_Decrypt_KATS())
        printk("OAEP PASSED\n");
    else
        printk("OAEP FAILED\n");

    if (!PKCS_Sign_Generate_KATS())
        printk("PKCS SIGN GEN PASSED\n");
    else
        printk("PKCS SIGN GEN FAILED\n");

    if (!PKCS_Sign_Verify_KATS())
        printk("PKCS VERIFY PASSED\n");
    else
        printk("PKCS VERIFY FAILED\n");

    if (!PSS_Sign_Generate_KATS())
        printk("PSS SIGN GEN PASSED\n");
    else
        printk("PSS SIGN GEN FAILED\n");

    if (!PSS_Sign_Verify_KATS())
        printk("PSS VERIFY PASSED\n");
    else
        printk("PSS VERIFY FAILED\n");

    printk("\n===== RSA TESTS COMPLETE =====\n");
    return 0;
}