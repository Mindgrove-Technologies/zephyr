#include <zephyr/device.h>

/* include/zephyr/drivers/crypto/rsa_mg.h */

enum rsa_mg_padding {
    RSA_MG_PAD_PKCS_V15,
    RSA_MG_PAD_PSS,
    RSA_MG_PAD_OAEP
};

enum rsa_mg_op {
    RSA_MG_OP_ENCRYPT,
    RSA_MG_OP_DECRYPT,
    RSA_MG_OP_SIGN,
    RSA_MG_OP_VERIFY
};

struct rsa_mg_pkt {
    uint8_t *in;        /* Input data (Message or Ciphertext) */
    size_t  in_len;
    uint8_t *out;       /* Result buffer */
    size_t  out_len;
    uint8_t *exp;       /* Exponent (Public or Private) */
    uint8_t *mod;       /* Modulus N */
    
    /* Padding Specifics */
    uint8_t *label;     /* For OAEP */
    size_t  label_len;
    uint8_t *salt;      /* For PSS */
    size_t  salt_len;
};

/* The API Structure */
struct rsa_mg_driver_api {
    int (*invoke)(const struct device *dev, enum rsa_mg_op op, 
                  enum rsa_mg_padding pad, struct rsa_mg_pkt *pkt);
};