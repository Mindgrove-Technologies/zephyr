


#define DT_DRV_COMPAT mindgrove_wdt

#include <zephyr/kernel.h>
#include <soc.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>

LOG_MODULE_REGISTER(wdt_mindgrove, CONFIG_WDT_LOG_LEVEL);

/* Bit Definitions based on new WDT_Type struct */
#define WDT_CTRL_EN_POS    0
#define WDT_CTRL_MODE_POS  1
#define WDT_CTRL_SOFT_POS  2

#define WD_ENABLE          (1U << WDT_CTRL_EN_POS)
#define WD_DISABLE         (0U)
#define MODE_INTERRUPT     (0U << WDT_CTRL_MODE_POS)
#define MODE_RESET         (1U << WDT_CTRL_MODE_POS)
#define MODE_SOFT_RESET    (1U << WDT_CTRL_SOFT_POS)
#define SOFT_RESET 7
#define WDT_NODE DT_NODELABEL(watchdog0)
#define WDT_BASE_ADDR DT_REG_ADDR(WDT_NODE)
#define WDT_CLK_FREQ  DT_PROP(WDT_NODE, clock_frequency)

/* New Register Structure Mapping */
typedef struct {
    volatile uint64_t WDT_CYCLES;
    union {
        volatile uint16_t WDT_CTRL;
        struct {
            volatile uint16_t WDT_CTRL_EN    : 1;
            volatile uint16_t WDT_CTRL_MODE  : 1;
            volatile uint16_t WDT_CTRL_SOFT  : 1;
            volatile uint16_t                : 13;
        } WDT_CTRL_b;
    };
    const uint16_t RESERVED1;
    const uint32_t RESERVED2;
    volatile uint16_t WDT_RESET_CYCLES;
    const uint16_t RESERVED3;
    const uint32_t RESERVED4;
    volatile uint32_t WDT_ACTIVE;
} wdt_mindgrove_reg_t;

struct wdt_mindgrove_device_config {
    uintptr_t regs;
    uint32_t sys_clk_freq;
};

struct wdt_mindgrove_dev_data {
    wdt_callback_t cb;
    uint64_t wcycles;
    uint16_t rcycles;
    uint16_t ctrl_flags;
    bool timeout_valid;
};

#define DEV_REG(dev) \
    ((volatile wdt_mindgrove_reg_t *)((const struct wdt_mindgrove_device_config *const)(dev)->config)->regs)

static int wdt_mindgrove_disable(const struct device *dev)
{
    volatile wdt_mindgrove_reg_t *wdt = DEV_REG(dev);

    wdt->WDT_CTRL = WD_DISABLE;
    wdt->WDT_CYCLES = 0;
    wdt->WDT_ACTIVE = 0;

    LOG_DBG("Watchdog disabled");
    return 0;
}

static int wdt_mindgrove_setup(const struct device *dev, uint8_t options)
{
    volatile wdt_mindgrove_reg_t *wdt = DEV_REG(dev);
    struct wdt_mindgrove_dev_data *data = dev->data;

    if (!data->timeout_valid) {
        LOG_ERR("No valid timeout installed before setup");
        return -EINVAL;
    }

    /* Always disable first */
    wdt->WDT_CTRL = WD_DISABLE;

    if (data->ctrl_flags & MODE_SOFT_RESET) {
        /*
         * Soft reset path — matches baremetal exactly:
         * Just set CTRL with SOFT_RESET | INTR | ENABLE.
         * Do NOT write WDT_CYCLES or WDT_ACTIVE — hardware
         * triggers immediately from ctrl register alone.
         */
        wdt->WDT_CTRL = MODE_SOFT_RESET | MODE_INTERRUPT | WD_ENABLE;
        LOG_DBG("Watchdog soft reset armed");
    } else {
        /*
         * Hard reset path — matches baremetal exactly:
         * Load cycles, activate, then set ctrl.
         */
        wdt->WDT_CYCLES       = data->wcycles;
        wdt->WDT_RESET_CYCLES = data->rcycles;
        wdt->WDT_ACTIVE       = 1;
        wdt->WDT_CTRL         = data->ctrl_flags | WD_ENABLE;
        LOG_DBG("Watchdog hard reset armed, cycles=%" PRIu64, data->wcycles);
    }

    return 0;
}

/* Add at top of driver, alongside other defines */
#define WDT_FLAG_MINDGROVE_SOFT_RESET  BIT(3)  /* vendor-specific, bit 3 is free */

static int wdt_mindgrove_install_timeout(const struct device *dev,
                                          const struct wdt_timeout_cfg *cfg)
{
    struct wdt_mindgrove_dev_data *data = dev->data;
    const struct wdt_mindgrove_device_config *config = dev->config;

    if (data->timeout_valid) {
        return -ENOMEM;
    }

    if (cfg->window.min != 0) {
        return -EINVAL;
    }

    if (cfg->flags & WDT_FLAG_MINDGROVE_SOFT_RESET) {
        /*
         * Soft reset: no timeout window needed — fires immediately
         * on setup. window.max is irrelevant but we don't reject it.
         */
        data->ctrl_flags    = MODE_SOFT_RESET;  /* setup() adds INTR+ENABLE */
        data->wcycles       = 0;
        data->rcycles       = 0;
        data->cb            = NULL;
        data->timeout_valid = true;
        return 0;
    }

    /* Hard reset and interrupt mode both need a valid window */
    if (cfg->window.max == 0) {
        return -EINVAL;
    }

    uint64_t timeout_cycles = ((uint64_t)cfg->window.max * config->sys_clk_freq) / 1000U;

    if (cfg->flags & WDT_FLAG_RESET_SOC) {
        data->ctrl_flags = MODE_RESET;
        data->cb         = NULL;
    } else {
        data->ctrl_flags = MODE_INTERRUPT;
        data->cb         = cfg->callback;
    }

    uint32_t reset_cycles = (uint32_t)(timeout_cycles / 4);
    if (reset_cycles > 0xFFFF) {
        LOG_WRN("reset_cycles clamped to 0xFFFF");
        reset_cycles = 0xFFFF;
    }

    data->wcycles       = timeout_cycles;
    data->rcycles       = (uint16_t)reset_cycles;
    data->timeout_valid = true;

    return 0;
}

static int wdt_mindgrove_feed(const struct device *dev, int channel_id)
{
    volatile wdt_mindgrove_reg_t *wdt = DEV_REG(dev);
    ARG_UNUSED(channel_id);

    /* According to updated reference driver layout: 
     * Kick/Reload is performed by setting WDT_ACTIVE to 1 */
    wdt->WDT_ACTIVE = 1;

    return 0;
}

static const struct wdt_driver_api wdt_mindgrove_api = {
    .setup = wdt_mindgrove_setup,
    .disable = wdt_mindgrove_disable,
    .install_timeout = wdt_mindgrove_install_timeout,
    .feed = wdt_mindgrove_feed,
};

static int wdt_mindgrove_init(const struct device *dev)
{
    /* Turn off watchdog during system startup until explicitly initialized */
    return wdt_mindgrove_disable(dev);
}

static struct wdt_mindgrove_dev_data wdt_mindgrove_data_0;

static const struct wdt_mindgrove_device_config wdt_mindgrove_cfg_0 = {
    .regs = WDT_BASE_ADDR,
    .sys_clk_freq = WDT_CLK_FREQ 
};

DEVICE_DT_INST_DEFINE(0, wdt_mindgrove_init, NULL,
                      &wdt_mindgrove_data_0, &wdt_mindgrove_cfg_0, 
                      PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, 
                      &wdt_mindgrove_api);