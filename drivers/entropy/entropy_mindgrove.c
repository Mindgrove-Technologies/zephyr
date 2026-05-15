#include <zephyr/kernel.h>             /* for k_busy_wait */
#include <zephyr/device.h>             /* for DEVICE_DT_INST_DEFINE */
#include <zephyr/drivers/entropy.h>    /* for entropy_driver_api */
#include <zephyr/sys/printk.h>         /* for debugging printk */
#include <zephyr/logging/log.h>        /* for LOG_ERR/LOG_INF */

#define DT_DRV_COMPAT mindgrove_trng

/* Register Bit Definitions */
#define TRNG_CTRL_CMD_GET_RANDOM    0x1U
#define TRNG_STAT_BUSY_BIT          (1UL << (31))

/* Hardware Structure Mapping */
typedef struct {
    volatile uint32_t TRNG_CTRL;
    volatile uint32_t TRNG_STAT;
    volatile uint32_t TRNG_IE;
    volatile uint32_t TRNG_ISTAT;
    volatile uint32_t TRNG_RAND[4]; /* TRNG_RAND_0 to TRNG_RAND_3 */
} trng_regs_t;

struct trng_config {
    uintptr_t base;
    uint8_t instance_id;
};

#define IS_ALIGNED(addr, size) \
    ((((uintptr_t)(const uint8_t *)(addr)) & ((size) - 1U)) == 0U)

/**
 * @brief Core hardware logic. 
 * This is mapped directly to .get_entropy in the API struct.
 */
static int trng_generate(const struct device *dev, uint8_t *out, uint16_t len)
{
    //printk("TRNG: Generating %u bytes of entropy...\n", len);
    const struct trng_config *cfg = dev->config;
    trng_regs_t *regs = (trng_regs_t *)(cfg->base);
    uint16_t remaining = len;


    while (remaining > 0U) {
        regs->TRNG_CTRL = TRNG_CTRL_CMD_GET_RANDOM;

        while (regs->TRNG_STAT & TRNG_STAT_BUSY_BIT) {
            k_busy_wait(1);
        }

        for (int i = 0; i < 4 && remaining > 0U; i++) {
            uint32_t reg_val = regs->TRNG_RAND[i];
            uint8_t chunk = (remaining < 4U) ? (uint8_t)remaining : 4U;

            if (chunk == 4U && IS_ALIGNED(out, 4)) {
                *(uint32_t *)out = reg_val;
            } else {
                for (uint8_t b = 0; b < chunk; b++) {
                    out[b] = (uint8_t)(reg_val >> (b * 8));
                }
            }
            out += chunk;
            remaining -= chunk;
        }
    }

    return 0;
}

/**
 * @brief Minimal wrapper for ISR context to handle the extra 'flags' argument.
 */
static int trng_get_entropy_isr(const struct device *dev, uint8_t *buf, 
                                 uint16_t len, uint32_t flags)
{
    if (!(flags & ENTROPY_BUSYWAIT)) {
        return -ENOTSUP;
    }
    return trng_generate(dev, buf, len);
}

/* Direct Mapping */
static const struct entropy_driver_api trng_api = {
    .get_entropy = trng_generate,      /* Direct map */
    .get_entropy_isr = trng_get_entropy_isr
};

#define TRNG_INIT(inst) \
	static const struct trng_config trng_cfg_##inst = { \
		.base = DT_INST_REG_ADDR(inst), \
		.instance_id = DT_INST_PROP(inst, instance_id), \
	}; \
	DEVICE_DT_INST_DEFINE(inst, \
			    NULL, \
			    NULL, \
			    NULL, \
			    &trng_cfg_##inst, \
			    PRE_KERNEL_1, \
			    CONFIG_ENTROPY_INIT_PRIORITY, \
			    &trng_api);

DT_INST_FOREACH_STATUS_OKAY(TRNG_INIT)

