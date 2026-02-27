#define DT_DRV_COMPAT mindgrove_rsa2048

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/crypto/crypto.h>
#include "rsa_padding.h"
#include "crypto_mindgrove_rsa.h"
#include "zephyr/crypto/rsa.h"
#include "mindgrove_rsa_hw.h"
#include "crypto_mindgrove_sha.h"

// Note: LOG_MODULE_REGISTER removed if you are only using //printk
// but kept as a placeholder if other parts of your app need it.

struct mg_rsa_config {
    volatile RSA_Type *regs;
};

/* Data struct is now empty/minimal as lock is removed */
struct mg_rsa_data {
    bool in_use; 
};

static void dump_buffer(const char *label, const uint8_t *buf, size_t len)
{
    //printk("--- %s (%zu bytes) ---\n", label, len);
    for (size_t i = 0; i < len; i++) {
        //printk("%02x ", buf[i]);
        if ((i + 1) % 16 == 0) {
            //printk("\n");
        }
    }
    //printk("\n------------------\n");
}

/* ============================= */
/* The "Invoke" API Implementation */
/* ============================= */

static int mg_rsa_invoke(const struct device *dev, enum rsa_mg_op op, 
                         enum rsa_mg_padding pad, struct rsa_mg_pkt *pkt) {
    // struct mg_rsa_data *data = dev->data; // Unused now
    uint8_t scratch_pad[256] __aligned(8);
    uint16_t rsa_ret = 0;
    int ret = 0;

    //printk("RSA Invoke Started (No Lock): Op=%d, Pad=%d\n", op, pad);

    /* SEMAPHORE REMOVED: Hardware is now unguarded */

    switch (op) {
        case RSA_MG_OP_SIGN:
            if (pad == RSA_MG_PAD_PKCS_V15) {
                ret = RSASSA_PKCS1_v1_5_Sign(scratch_pad, 256, pkt->in, pkt->in_len);
            } else {
                ret = RSASSA_PSS_Sign(scratch_pad, 256, pkt->in, pkt->in_len, pkt->salt, pkt->salt_len, 1);
            }
            if (ret != 0) goto exit;

            //dump_buffer("Padded Message (EM)", scratch_pad, 256);
            rsa_ret = RSA_Run(pkt->out, scratch_pad, pkt->exp, pkt->mod);
            break;

        case RSA_MG_OP_VERIFY:
            //dump_buffer("Signature Input", pkt->in, 256);
            rsa_ret = RSA_Run(scratch_pad, pkt->in, pkt->exp, pkt->mod);
            if (rsa_ret != 0) {
                ret = -EIO;
                goto exit;
            }

            if (pad == RSA_MG_PAD_PKCS_V15) {
                ret = RSASSA_PKCS1_v1_5_Verify(scratch_pad, 256, pkt->out, pkt->out_len);
            } else {
                ret = RSASSA_PSS_Verify(scratch_pad, 256, pkt->out, pkt->out_len, pkt->salt_len);
            }
            break;

        case RSA_MG_OP_ENCRYPT:
            if (pad == RSA_MG_PAD_OAEP) {
                ret = RSAES_OAEP_Encrypt(scratch_pad, 256, pkt->in, pkt->in_len, 
                                         pkt->label, pkt->label_len);
            } else {
                ret = RSAES_PKCS1_v1_5_Encrypt(scratch_pad, 256, pkt->in, pkt->in_len);
            }
            if (ret != 0) goto exit;

            //dump_buffer("Padded Plaintext", scratch_pad, 256);
            rsa_ret = RSA_Run(pkt->out, scratch_pad, pkt->exp, pkt->mod);
            break;

        case RSA_MG_OP_DECRYPT:
            rsa_ret = RSA_Run(scratch_pad, pkt->in, pkt->exp, pkt->mod);
            if (rsa_ret != 0) {
                ret = -EIO;
                goto exit;
            }
            
            //dump_buffer("Raw Hardware Decrypt Result", scratch_pad, 256);

            if (pad == RSA_MG_PAD_OAEP) {
                ret = RSAES_OAEP_Decrypt(scratch_pad, 256, pkt->out, &pkt->out_len, 
                                         pkt->label, pkt->label_len);
            } else {
                ret = RSAES_PKCS1_v1_5_Decrypt(pkt->out, &pkt->out_len, scratch_pad, 256);
            }
            break;

        default:
            ret = -ENOTSUP;
            break;
    }

exit:
    if (ret != 0 || rsa_ret != 0) {
        //printk("RSA ERROR: ret=%d, hardware_ret=%u\n", ret, rsa_ret);
    } else {
        //printk("RSA SUCCESS\n");
        //dump_buffer("Final Output", pkt->out, (op == RSA_MG_OP_SIGN || op == RSA_MG_OP_ENCRYPT) ? 256 : pkt->out_len);
    }

    /* SEMAPHORE GIVE REMOVED */
    return ret;
}

/* ============================= */
/* Zephyr Registration          */
/* ============================= */

static const struct rsa_mg_driver_api mg_rsa_api = {
    .invoke = mg_rsa_invoke,
};

static int mg_rsa_init(const struct device *dev) {
    //printk("MindGrove RSA init called (No Semaphores)\n");
    //printk("DT_INST_REG_ADDR = %lx\n", (unsigned long)DT_INST_REG_ADDR(0));
    return 0;
}

#define MG_RSA_INIT(n) \
    static const struct mg_rsa_config mg_rsa_cfg_##n = { \
        .regs = (volatile RSA_Type *)DT_INST_REG_ADDR(n), \
    }; \
    static struct mg_rsa_data mg_rsa_data_##n; \
    DEVICE_DT_INST_DEFINE(n, mg_rsa_init, NULL, \
                          &mg_rsa_data_##n, &mg_rsa_cfg_##n, \
                          POST_KERNEL, CONFIG_CRYPTO_INIT_PRIORITY, \
                          &mg_rsa_api);

DT_INST_FOREACH_STATUS_OKAY(MG_RSA_INIT)