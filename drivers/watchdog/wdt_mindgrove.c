#define DT_DRV_COMPAT mindgrove_wdt

#include <zephyr/kernel.h>
#include <soc.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>

LOG_MODULE_REGISTER(wdt_mindgrove, CONFIG_WDT_LOG_LEVEL);

#define WDT_CTRL_EN_POS   0
#define WDT_CTRL_MODE_POS 1
#define WDT_CTRL_SOFT_POS 2

#define WD_ENABLE       (1U << WDT_CTRL_EN_POS)
#define WD_DISABLE      (0U)
#define MODE_INTERRUPT  (0U << WDT_CTRL_MODE_POS)
#define MODE_RESET      (1U << WDT_CTRL_MODE_POS)
#define MODE_SOFT_RESET (1U << WDT_CTRL_SOFT_POS)

/* Vendor-specific flag for soft reset — bit 3, free from Zephyr's bits 0-1 */
#define WDT_FLAG_MINDGROVE_SOFT_RESET BIT(3)

#define WDT_NODE      DT_NODELABEL(watchdog0)
#define WDT_BASE_ADDR DT_REG_ADDR(WDT_NODE)
#define WDT_CLK_FREQ  DT_PROP(WDT_NODE, clock_frequency)

typedef struct {
    volatile uint64_t WDT_CYCLES;
    union {
        volatile uint16_t WDT_CTRL;
        struct {
            volatile uint16_t WDT_CTRL_EN   : 1;
            volatile uint16_t WDT_CTRL_MODE : 1;
            volatile uint16_t WDT_CTRL_SOFT : 1;
            volatile uint16_t               : 13;
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
    uint32_t  sys_clk_freq;
};

struct wdt_mindgrove_dev_data {
    wdt_callback_t cb;
    uint64_t wcycles;
    uint16_t rcycles;
    uint16_t ctrl_flags;
    bool timeout_valid; /* install_timeout has been called */
    bool is_running;    /* wdt_setup has been called */
};

#define DEV_REG(dev) \
    ((volatile wdt_mindgrove_reg_t *) \
     ((const struct wdt_mindgrove_device_config *const)(dev)->config)->regs)

/* ------------------------------------------------------------------ */

static int wdt_mindgrove_install_timeout(const struct device *dev,
                                         const struct wdt_timeout_cfg *cfg)
{
    struct wdt_mindgrove_dev_data *data = dev->data;
    const struct wdt_mindgrove_device_config *config = dev->config;

    /*
     * Check state first, then validate flags, then validate window.
     * Order matters — tests expect specific error codes at each stage.
     */

    /* 1. If already running, no more installs allowed */
    if (data->is_running) {
        return -EBUSY;
    }

    /* 2. Only one channel — slot already taken */
    if (data->timeout_valid) {
        return -EINVAL;
    }

    /* 3. Validate window */
    if (cfg->window.min != 0 || cfg->window.max == 0) {
        return -EINVAL;
    }
    /* 4. Reject unsupported reset flags */
    if (cfg->flags & WDT_FLAG_RESET_CPU_CORE) {
        return -ENOTSUP;
    }
    if (cfg->flags == WDT_FLAG_RESET_NONE) {
        return -ENOTSUP;
    }


    /* 5. Determine mode */
    if (cfg->flags & WDT_FLAG_MINDGROVE_SOFT_RESET) {
        data->ctrl_flags = MODE_SOFT_RESET;
        data->cb         = NULL;
    } else if (cfg->flags & WDT_FLAG_RESET_SOC) {
        data->ctrl_flags = MODE_RESET;
        data->cb         = NULL;
    } else {
        return -ENOTSUP;
    }

    /* 6. Calculate cycles */
    uint64_t timeout_cycles =
        ((uint64_t)cfg->window.max * config->sys_clk_freq) / 1000U;

    uint32_t reset_cycles = (uint32_t)(timeout_cycles / 4);
    if (reset_cycles > 0xFFFF) {
        LOG_WRN("reset_cycles clamped to 0xFFFF");
        reset_cycles = 0xFFFF;
    }

    data->wcycles       = timeout_cycles;
    data->rcycles       = (uint16_t)reset_cycles;
    data->timeout_valid = true;

    return 0; /* channel 0 */
}

/* ------------------------------------------------------------------ */

static int wdt_mindgrove_setup(const struct device *dev, uint8_t options)
{
    volatile wdt_mindgrove_reg_t *wdt = DEV_REG(dev);
    struct wdt_mindgrove_dev_data *data = dev->data;

    if (!data->timeout_valid) {
        LOG_ERR("No valid timeout installed before setup");
        return -EINVAL;
    }

    if (data->is_running) {
        return -EBUSY;
    }

    /* Mindgrove WDT has no pause support */
    if (options & WDT_OPT_PAUSE_IN_SLEEP) {
        return -ENOTSUP;
    }
    if (options & WDT_OPT_PAUSE_HALTED_BY_DBG) {
        return -ENOTSUP;
    }

    wdt->WDT_CTRL = WD_DISABLE;

    if (data->ctrl_flags & MODE_SOFT_RESET) {
        /* Soft reset: CTRL write alone triggers it — no cycles needed */
        wdt->WDT_CTRL = MODE_SOFT_RESET | MODE_INTERRUPT | WD_ENABLE;
        LOG_DBG("Watchdog soft reset armed");
    } else {
        /* Hard reset: load cycles, activate, then arm */
        wdt->WDT_CYCLES       = data->wcycles;
        wdt->WDT_RESET_CYCLES = data->rcycles;
        wdt->WDT_ACTIVE       = 1;
        wdt->WDT_CTRL         = data->ctrl_flags | WD_ENABLE;
        LOG_DBG("Watchdog hard reset armed, cycles=%" PRIu64, data->wcycles);
    }

    data->is_running = true;
    return 0;
}

/* ------------------------------------------------------------------ */

static int wdt_mindgrove_disable(const struct device *dev)
{
    volatile wdt_mindgrove_reg_t *wdt = DEV_REG(dev);
    struct wdt_mindgrove_dev_data *data = dev->data;

    if (!data->is_running && !data->timeout_valid) {
        /*
         * Nothing was ever started or installed.
         * Return -EFAULT as the API requires for "never started".
         */
        return -EFAULT;
    }

    /* Always silence hardware and reset all state */
    wdt->WDT_CTRL   = WD_DISABLE;
    wdt->WDT_CYCLES = 0;
    wdt->WDT_ACTIVE = 0;

    data->timeout_valid = false;
    data->is_running    = false;
    data->cb            = NULL;
    data->ctrl_flags    = 0;

    LOG_DBG("Watchdog disabled");
    return 0;
}
/* ------------------------------------------------------------------ */

static int wdt_mindgrove_feed(const struct device *dev, int channel_id)
{
    volatile wdt_mindgrove_reg_t *wdt = DEV_REG(dev);
    struct wdt_mindgrove_dev_data *data = dev->data;

    if (!data->is_running) {
        return -EINVAL;
    }

    if (channel_id != 0) {
        return -EINVAL;
    }

    wdt->WDT_ACTIVE = 0;
    wdt->WDT_ACTIVE = 1;
    return 0;
}

/* ------------------------------------------------------------------ */

static const struct wdt_driver_api wdt_mindgrove_api = {
    .setup           = wdt_mindgrove_setup,
    .disable         = wdt_mindgrove_disable,
    .install_timeout = wdt_mindgrove_install_timeout,
    .feed            = wdt_mindgrove_feed,
};

static int wdt_mindgrove_init(const struct device *dev)
{
    volatile wdt_mindgrove_reg_t *wdt = DEV_REG(dev);

    /* Silence hardware directly at boot — bypass is_running guard */
    wdt->WDT_CTRL   = WD_DISABLE;
    wdt->WDT_CYCLES = 0;
    wdt->WDT_ACTIVE = 0;

    return 0;
}

static struct wdt_mindgrove_dev_data wdt_mindgrove_data_0; /* zero-initialized */

static const struct wdt_mindgrove_device_config wdt_mindgrove_cfg_0 = {
    .regs         = WDT_BASE_ADDR,
    .sys_clk_freq = WDT_CLK_FREQ,
};

DEVICE_DT_INST_DEFINE(0, wdt_mindgrove_init, NULL,
                      &wdt_mindgrove_data_0, &wdt_mindgrove_cfg_0,
                      PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
                      &wdt_mindgrove_api);