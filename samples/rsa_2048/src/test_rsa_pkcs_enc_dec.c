#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/crypto/rsa.h>
#include <zephyr/sys/printk.h>
#include <string.h>
#include "pkcs_encrypt_decrypt_kat_testvectors.h"

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

int test_rsa_pkcs_enc_dec(void)
{
    const struct device *dev = get_rsa_dev();
    if (!dev) return -ENODEV;
    const struct rsa_mg_driver_api *api = dev->api;

    uint32_t passed = 0, failed = 0;
    uint16_t count = pkcs_enc_dec_kat_vectors_count;

    for (uint16_t i = 0; i < count; i++) {
        const PKCS_Encrypt_Decrypt_Test_Vectors *tv =
            &pkcs_encrypt_decrypt_kat_vectors[i];
        uint8_t ciphertext[RSA_BYTES] __aligned(8) = {0};
        uint8_t decrypted[RSA_BYTES]  __aligned(8) = {0};
        struct rsa_mg_pkt pkt = {0};

        /* Encrypt */
        pkt.in     = (uint8_t *)tv->input;
        pkt.in_len = tv->input_len_bits / 8;
        pkt.out    = ciphertext;
        pkt.out_len = RSA_BYTES;
        pkt.exp    = (uint8_t *)tv->public_exponent;
        pkt.mod    = (uint8_t *)tv->modulus;

        int ret = api->invoke(dev, RSA_MG_OP_ENCRYPT,
                              RSA_MG_PAD_PKCS_V15, &pkt);
        if (ret) {
            printk("[PKCS ENC/DEC %03d] FAIL (encrypt error %d)\n", i, ret);
            failed++;
            continue;
        }

        /* Decrypt */
        memset(&pkt, 0, sizeof(pkt));
        pkt.in      = ciphertext;
        pkt.in_len  = RSA_BYTES;
        pkt.out     = decrypted;
        pkt.out_len = RSA_BYTES;
        pkt.exp     = (uint8_t *)tv->private_exponent;
        pkt.mod     = (uint8_t *)tv->modulus;

        ret = api->invoke(dev, RSA_MG_OP_DECRYPT,
                          RSA_MG_PAD_PKCS_V15, &pkt);

        bool ok = (!ret &&
                   pkt.out_len == (tv->input_len_bits / 8) &&
                   memcmp(decrypted, tv->input, pkt.out_len) == 0);

        printk("[PKCS ENC/DEC %03d] %s\n", i, ok ? "PASS" : "FAIL");
        ok ? passed++ : failed++;
    }

    printk("PKCS ENC/DEC: %d Passed, %d Failed\n\n", passed, failed);
    return failed ? -EIO : 0;
}