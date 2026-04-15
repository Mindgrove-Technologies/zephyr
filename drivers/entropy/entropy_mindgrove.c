#include <zephyr/drivers/entropy.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h> // <--- Add this

#define DT_DRV_COMPAT mindgrove_trng

/* Register Bit Definitions */
#define VCTRL_CMD_GET_RANDOM    0x1U
#define VSTAT_BUSY_BIT          BIT(31)

/* Hardware Structure Mapping */
typedef struct {
    volatile uint32_t VCTRL;
    volatile uint32_t VSTAT;
    volatile uint32_t VIE;
    volatile uint32_t VISTAT;
    volatile uint32_t VRAND[4]; /* VRAND_0 to VRAND_3 */
} vtrng_regs_t;

struct vtrng_config {
    uintptr_t base;
    uint8_t instance_id;
};


/**
 * @brief Core hardware logic. 
 * This is mapped directly to .get_entropy in the API struct.
 */
static int vtrng_generate(const struct device *dev, uint8_t *out, uint16_t len)
{
    printk("TRNG: Generating %u bytes of entropy...\n", len);
    const struct vtrng_config *cfg = dev->config;
    vtrng_regs_t *regs = (vtrng_regs_t *)(cfg->base + (cfg->instance_id * sizeof(vtrng_regs_t)));
    uint16_t remaining = len;


    while (remaining > 0U) {
        regs->VCTRL = VCTRL_CMD_GET_RANDOM;

        while (regs->VSTAT & VSTAT_BUSY_BIT) {
            k_busy_wait(1);
        }

        for (int i = 0; i < 4 && remaining > 0U; i++) {
            uint32_t reg_val = regs->VRAND[i];
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
static int vtrng_get_entropy_isr(const struct device *dev, uint8_t *buf, 
                                 uint16_t len, uint32_t flags)
{
    if (!(flags & ENTROPY_BUSYWAIT)) {
        return -ENOTSUP;
    }
    return vtrng_generate(dev, buf, len);
}

/* Direct Mapping */
static const struct entropy_driver_api vtrng_api = {
    .get_entropy = vtrng_generate,      /* Direct map */
    .get_entropy_isr = vtrng_get_entropy_isr
};

#define VTRNG_INIT(inst) \
	static const struct vtrng_config vtrng_cfg_##inst = { \
		.base = DT_INST_REG_ADDR(inst), \
		.instance_id = DT_INST_PROP(inst, instance_id), \
	}; \
	DEVICE_DT_INST_DEFINE(inst, \
			    NULL, \
			    NULL, \
			    NULL, \
			    &vtrng_cfg_##inst, \
			    PRE_KERNEL_1, \
			    CONFIG_ENTROPY_INIT_PRIORITY, \
			    &vtrng_api);

DT_INST_FOREACH_STATUS_OKAY(VTRNG_INIT)

