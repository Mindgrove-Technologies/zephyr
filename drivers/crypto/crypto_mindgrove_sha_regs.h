#ifndef ZEPHYR_DRIVERS_CRYPTO_CRYPTO_MINDGROVE_SHA_H
#define ZEPHYR_DRIVERS_CRYPTO_CRYPTO_MINDGROVE_SHA_H

#include <zephyr/drivers/crypto.h>

/* SHA256 Constants */
#define MINDGROVE_SHA256_BLOCK_SIZE     64    /* 512 bits in bytes */
#define MINDGROVE_SHA256_DIGEST_SIZE    32
#define MINDGROVE_SHA256_BITS_PER_BYTE  8
#define MINDGROVE_SHA256_BLOCK_BITS     (MINDGROVE_SHA256_BLOCK_SIZE * MINDGROVE_SHA256_BITS_PER_BYTE)
#define MINDGROVE_SHA256_MAX_INPUT_BITS 64    /* Based on register constraints */

/* Hardware register definitions */
typedef struct {
    /* Input/Output registers */
    volatile uint64_t SHA_INPUT;          /* Input message block */
    volatile uint64_t RESERVED[15];       /* Reserved space */
    volatile uint64_t SHA_OUTPUT;         /* Output hash */
    volatile uint64_t RESERVED_OUT[7];    /* More reserved space */
    
    /* Control register */
    union {
        volatile uint8_t SHA_CTRL;
        struct {
            volatile uint8_t CONT_PREHASH : 1;  /* Continue with previous hash */
            volatile uint8_t RESERVED_CTRL : 7;
        } SHA_CTRL_b;
    };
    
    /* Status register */
    union {
        volatile uint8_t SHA_STATUS;
        struct {
            volatile uint8_t SHA_STATUS_READY : 1;      /* Ready for new input */
            volatile uint8_t SHA_STATUS_OUT_READY : 1;  /* Output digest ready */
            volatile uint8_t RESERVED_STATUS : 6;
        } SHA_STATUS_b;
    };
    
    volatile uint16_t RESERVED_END;
} SHA256_Type;

/* Control register bits */
#define SHA_CTRL_CONT_PREHASH  (1 << 0)  /* Continue with previous hash */

/* Status register bits */
#define SHA_STATUS_READY       (1 << 0)  /* Ready for new input */
#define SHA_STATUS_OUT_READY   (1 << 1)  /* Output digest ready */

/* Session context */
struct mindgrove_sha256_session {
    /* Session state machine */
    enum {
        SESSION_IDLE = 0,      /* No active session */
        SESSION_ACTIVE,        /* hash_begin() called */
        SESSION_PROCESSING,    /* Data being processed */
        SESSION_FINALIZED      /* hash_compute(finish=true) completed */
    } state;
    
    /* Message tracking */
    uint64_t total_len_bits;   /* Total message length in bits (software tracked) */
    uint64_t processed_bits;   /* Bits processed by hardware */
    
    /* Block buffering */
    uint8_t block_buffer[MINDGROVE_SHA256_BLOCK_SIZE];
    uint32_t block_offset;
    
    /* Operation mode */
    bool is_single_shot;       /* TRUE: Single-shot mode, FALSE: Streaming mode */
};

/* Device instance data */
struct mindgrove_sha256_data {
    struct k_mutex device_lock;        /* Hardware access mutex */
    SHA256_Type *regs;                 /* Hardware registers */
    struct mindgrove_sha256_session *active_session;
    bool hardware_in_use;
};

/* Device configuration */
struct mindgrove_sha256_config {
    SHA256_Type *base;
    uint32_t irq_num;
    void (*irq_config_func)(const struct device *dev);
};

#endif /* ZEPHYR_DRIVERS_CRYPTO_CRYPTO_MINDGROVE_SHA_H */