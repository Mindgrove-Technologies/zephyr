#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/crypto/rsa.h>
#include <zephyr/sys/printk.h>
#include <string.h>
#include "oaep_decrypt_kat_testvectors_01.h"

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

int test_rsa_oaep_decrypt_01(void)
{
    const struct device *dev = get_rsa_dev();
    if (!dev) return -ENODEV;
    const struct rsa_mg_driver_api *api = dev->api;

    uint32_t passed = 0, failed = 0;
    uint16_t count = oaep_dec_kat_vectors_count_01;

    for (uint16_t i = 0; i < count; i++) {
        const OAEP_Decrypt_Test_Vectors_01 *tv = &oaep_decrypt_kat_vectors_01[i];
        uint8_t decrypted[RSA_BYTES] __aligned(8) = {0};
        struct rsa_mg_pkt pkt = {0};

        /* Check ciphertext < modulus */
        bool valid = false;
        for (size_t k = 0; k < RSA_BYTES; k++) {
            if (tv->cipher_text[k] < tv->modulus[k]) {
                valid = true; break;
            } else if (tv->cipher_text[k] > tv->modulus[k]) {
                break;
            }
        }
        if (!valid) {
            if (tv->expected_result == 'F') {
                printk("[OAEP DEC 01 %03d] PASS (c>=n, expected fail)\n", i);
                passed++;
            } else {
                printk("[OAEP DEC 01 %03d] FAIL (c>=n but expected pass)\n", i);
                failed++;
            }
            continue;
        }

        pkt.in        = (uint8_t *)tv->cipher_text;
        pkt.in_len    = RSA_BYTES;
        pkt.out       = decrypted;
        pkt.out_len   = RSA_BYTES;
        pkt.exp       = (uint8_t *)tv->private_exponent;
        pkt.mod       = (uint8_t *)tv->modulus;
        pkt.label     = (tv->label_len_bits > 0) ?
                         (uint8_t *)tv->label : NULL;
        pkt.label_len = tv->label_len_bits / 8;

        int ret = api->invoke(dev, RSA_MG_OP_DECRYPT, RSA_MG_PAD_OAEP, &pkt);

        bool ok;
        if (tv->expected_result == 'P') {
            ok = (!ret &&
                  pkt.out_len == (tv->input_len_bits / 8) &&
                  memcmp(tv->input, decrypted, pkt.out_len) == 0);
            printk("[OAEP DEC 01 %03d] %s%s\n", i,
                   ok ? "PASS" : "FAIL",
                   ok ? "" : " (expected pass)");
        } else {
            ok = (ret != 0) ||
                 (pkt.out_len != (tv->input_len_bits / 8)) ||
                 (memcmp(tv->input, decrypted, pkt.out_len) != 0);
            printk("[OAEP DEC 01 %03d] %s%s\n", i,
                   ok ? "PASS" : "FAIL",
                   ok ? " (expected fail)" : " (should have failed)");
        }
        ok ? passed++ : failed++;
    }

    printk("OAEP DECRYPT 01: %d Passed, %d Failed\n", passed, failed);
    return failed ? -EIO : 0;
}