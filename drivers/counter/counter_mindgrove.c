/*
 * Mindgrove GPTimer — Zephyr counter driver
 * Rewritten against baremetal reference (gptimer_driver.c v1.1)
 *
 * Copyright (c) Mindgrove Technologies Pvt. Ltd 2025.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Key hardware facts from baremetal driver:
 *   - CTRL is 32-bit (union with bitfield, full uint32_t access)
 *   - Mode 1 = UP   → overflow  interrupt (bit 9)  on reaching PERIOD
 *   - Mode 2 = DOWN → underflow interrupt (bit 10) on reaching 0
 *   - Prescaler: CLOCK_CTRL |= (GPT_CLK_PRESCALER(x) & GPT_PRSC_EN)
 *   - Interrupt enable written in same CTRL word as GPT_EN | GPT_MODE
 *   - GPT_COUNT_RESET (bit 5) resets counter AND clears interrupt status
 *   - No separate W1C register — reset is the clear mechanism
 */

#define DT_DRV_COMPAT mindgrove_gptimer

#include <zephyr/device.h>
#include <zephyr/types.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <zephyr/drivers/interrupt_controller/riscv_plic.h>

/* ------------------------------------------------------------------ */
/* Hardware register map — CTRL is 32-bit                             */
/* ------------------------------------------------------------------ */

typedef struct {
    volatile uint16_t CTRL;        /* 0x00 — 16-bit */
    volatile uint16_t RESERVED0;   /* 0x02 — pad     */
    volatile uint32_t CLOCK_CTRL;  /* 0x04           */
    volatile uint32_t COUNT;       /* 0x08           */
    volatile uint32_t RPTD_COUNT;  /* 0x0C           */
    volatile uint32_t DUTY_CYCLE;  /* 0x10           */
    volatile uint32_t PERIOD;      /* 0x14           */
    volatile uint32_t CAPTURE_INP; /* 0x18           */
} GPTIMER_Type;
/* ------------------------------------------------------------------ */
/* CTRL register bit macros — copied verbatim from baremetal driver   */
/* ------------------------------------------------------------------ */

#define GPT_EN                  ((uint32_t)1U << 0U)
#define GPT_MODE(x)             ((uint32_t)(x) << 2U)
#define GPT_OUTPUT_EN           ((uint32_t)1U << 4U)
#define GPT_COUNT_RESET         ((uint32_t)1U << 5U)
#define GPT_CONTIN_CNT_EN       ((uint32_t)1U << 6U)
#define GPT_PWM_FALL_INTR_EN    ((uint32_t)1U << 7U)
#define GPT_PWM_RISE_INTR_EN    ((uint32_t)1U << 8U)
#define GPT_CNTR_OFLOW_INTR_EN  ((uint32_t)1U << 9U)   /* UP mode   */
#define GPT_CNTR_UFLOW_INTR_EN  ((uint32_t)1U << 10U)  /* DOWN mode */
#define GPT_CAPTURE_IP(x)       ((uint32_t)(x) << 15U)

/* Interrupt status bits (read-only, cleared by GPT_COUNT_RESET) */
#define GPT_PWM_FALL_INTR       ((uint32_t)1U << 11U)
#define GPT_PWM_RISE_INTR       ((uint32_t)1U << 12U)
#define GPT_OFLOW_INTR          ((uint32_t)1U << 13U)  /* UP overflow  */
#define GPT_UFLOW_INTR          ((uint32_t)1U << 14U)  /* DOWN underflow */
#define GPT_INTR_STATUS_MASK    (GPT_PWM_FALL_INTR | GPT_PWM_RISE_INTR | \
                                  GPT_OFLOW_INTR   | GPT_UFLOW_INTR)

/* Timer mode values — confirmed from INTR_EN macro in baremetal */
#define GPT_MODE_UP             1U   /* overflow  fires at PERIOD */
#define GPT_MODE_DOWN           2U   /* underflow fires at 0      */

/* ------------------------------------------------------------------ */
/* CLOCK_CTRL macros — copied verbatim from baremetal driver          */
/* ------------------------------------------------------------------ */

#define GPT_CLK_PRESCALER(x)    ((uint32_t)(x) << 1U)
#define GPT_CHECK_UPDATE_EN     0x20000U
#define GPT_CLK_EN              0xFFFDFFFFU   /* clears UPDATE_EN     */
#define GPT_PRSC_EN             0xFFFFFFFEU   /* masks CLK_SRC bit 0  */

/* ------------------------------------------------------------------ */
/* Driver structs                                                      */
/* ------------------------------------------------------------------ */

typedef void (*gptimer_irq_config_func_t)(const struct device *dev);

struct gptimer_counter_config {
    struct counter_config_info info; /* MUST be first — Zephyr API req  */
    uint32_t base_addr;
    uint32_t freq_hz;
    uint32_t prescaler;
    uint32_t mode;                   /* GPT_MODE_UP or GPT_MODE_DOWN    */
    gptimer_irq_config_func_t irq_config_func;
};

struct gptimer_counter_data {
    counter_alarm_callback_t  alarm_cb;
    void                     *alarm_user_data;
    counter_top_callback_t    top_cb;
    void                     *top_user_data;
    uint32_t                  guard_period;
};

/* debug — remove when stable */
volatile uint32_t gpt_isr_count;

/* ------------------------------------------------------------------ */
/* Register accessor                                                   */
/* ------------------------------------------------------------------ */

static inline GPTIMER_Type *get_regs(const struct device *dev)
{
    return (GPTIMER_Type *)(uintptr_t)
           ((const struct gptimer_counter_config *)dev->config)->base_addr;
}

/* ------------------------------------------------------------------ */
/* Internal: build the correct interrupt-enable bit for the mode      */
/* Matches baremetal INTR_EN() macro logic                            */
/* ------------------------------------------------------------------ */

static inline uint32_t intr_en_bit(uint32_t mode)
{
    if (mode == GPT_MODE_UP) {
        return GPT_CNTR_OFLOW_INTR_EN;
    } else if (mode == GPT_MODE_DOWN) {
        return GPT_CNTR_UFLOW_INTR_EN;
    }
    return 0U;
}

static inline uint32_t intr_status_bit(uint32_t mode)
{
    if (mode == GPT_MODE_UP) {
        return GPT_OFLOW_INTR;
    } else if (mode == GPT_MODE_DOWN) {
        return GPT_UFLOW_INTR;
    }
    return 0U;
}

/* ------------------------------------------------------------------ */
/* Counter API                                                         */
/* ------------------------------------------------------------------ */

static int gptimer_start(const struct device *dev)
{
    /* Matches baremetal GPT_Reset: set COUNT_RESET then keep EN */
    get_regs(dev)->CTRL |= GPT_COUNT_RESET | GPT_EN;
    return 0;
}

static int gptimer_stop(const struct device *dev)
{
    get_regs(dev)->CTRL &= ~GPT_EN;
    return 0;
}

static int gptimer_get_value(const struct device *dev, uint32_t *ticks)
{
    if (!ticks) {
        return -EINVAL;
    }
    *ticks = get_regs(dev)->COUNT;
    return 0;
}

static int gptimer_set_top_value(const struct device *dev,
                                  const struct counter_top_cfg *top_cfg)
{
    GPTIMER_Type *regs = get_regs(dev);
    struct gptimer_counter_data *data = dev->data;

    if (!top_cfg) {
        return -EINVAL;
    }

    /* Deactivate UPDATE_EN before writing PERIOD — matches baremetal init */
    if (regs->CLOCK_CTRL & GPT_CHECK_UPDATE_EN) {
        regs->CLOCK_CTRL &= GPT_CLK_EN;
    }

    regs->PERIOD = top_cfg->ticks;

    if (!(top_cfg->flags & COUNTER_TOP_CFG_DONT_RESET)) {
        regs->CTRL |= GPT_COUNT_RESET;
    }

    data->top_cb        = top_cfg->callback;
    data->top_user_data = top_cfg->user_data;

    return 0;
}

static uint32_t gptimer_get_top_value(const struct device *dev)
{
    return get_regs(dev)->PERIOD;
}

static uint32_t gptimer_get_freq(const struct device *dev)
{
    return ((const struct gptimer_counter_config *)dev->config)->freq_hz;
}

static int gptimer_set_alarm(const struct device *dev, uint8_t chan_id,
                              const struct counter_alarm_cfg *alarm_cfg)
{
    if (chan_id != 0) {
        return -ENOTSUP;
    }
    if (!alarm_cfg || !alarm_cfg->callback) {
        return -EINVAL;
    }

    const struct gptimer_counter_config *cfg = dev->config;
    struct gptimer_counter_data *data = dev->data;
    GPTIMER_Type *regs = get_regs(dev);

    data->alarm_cb        = alarm_cfg->callback;
    data->alarm_user_data = alarm_cfg->user_data;

    /* Enable the correct interrupt bit for this mode */
    regs->CTRL |= intr_en_bit(cfg->mode);

    return 0;
}

static int gptimer_cancel_alarm(const struct device *dev, uint8_t chan_id)
{
    if (chan_id != 0) {
        return -ENOTSUP;
    }

    const struct gptimer_counter_config *cfg = dev->config;
    struct gptimer_counter_data *data = dev->data;
    GPTIMER_Type *regs = get_regs(dev);

    data->alarm_cb        = NULL;
    data->alarm_user_data = NULL;

    /* Only disable if top_cb also not using it */
    if (!data->top_cb) {
        regs->CTRL &= ~intr_en_bit(cfg->mode);
    }

    return 0;
}

static uint32_t gptimer_get_guard_period(const struct device *dev,
                                          uint32_t flags)
{
    ARG_UNUSED(flags);
    return ((struct gptimer_counter_data *)dev->data)->guard_period;
}

static int gptimer_set_guard_period(const struct device *dev, uint32_t ticks,
                                     uint32_t flags)
{
    ARG_UNUSED(flags);
    ((struct gptimer_counter_data *)dev->data)->guard_period = ticks;
    return 0;
}

/* ------------------------------------------------------------------ */
/* ISR                                                                 */
/* ------------------------------------------------------------------ */

static void gptimer_isr(const struct device *dev)
{
    const struct gptimer_counter_config *cfg = dev->config;
    GPTIMER_Type *regs = get_regs(dev);
    struct gptimer_counter_data *data = dev->data;

    gpt_isr_count++;

    /* Read CTRL as 32-bit — status bits 11-14 are above uint16 range */
    uint32_t ctrl   = regs->CTRL;
    uint32_t status = ctrl & GPT_INTR_STATUS_MASK;

    if (status == 0U) {
        return; /* spurious */
    }

    uint32_t now = regs->COUNT;

    /*
     * Clear interrupt status by issuing GPT_COUNT_RESET.
     * This is the mechanism used by baremetal GPT_Reset().
     * Do this BEFORE callbacks so re-arm inside callback works cleanly.
     */
    regs->CTRL |= GPT_COUNT_RESET;

    /* Check the correct status bit for this mode */
    if (status & intr_status_bit(cfg->mode)) {

        /* Alarm: fires once, auto-cancels */
        if (data->alarm_cb) {
            counter_alarm_callback_t cb = data->alarm_cb;
            void *ud = data->alarm_user_data; /* save before clearing */

            data->alarm_cb        = NULL;
            data->alarm_user_data = NULL;

            /* Disable interrupt only if top_cb not also using it */
            if (!data->top_cb) {
                regs->CTRL &= ~intr_en_bit(cfg->mode);
            }

            cb(dev, 0, now, ud);
        }

        /* Top callback: stays armed, fires every period */
        if (data->top_cb) {
            data->top_cb(dev, data->top_user_data);
        }
    }
}

/* ------------------------------------------------------------------ */
/* Driver API table                                                    */
/* ------------------------------------------------------------------ */

static const struct counter_driver_api gptimer_api = {
    .start            = gptimer_start,
    .stop             = gptimer_stop,
    .get_value        = gptimer_get_value,
    .set_top_value    = gptimer_set_top_value,
    .get_top_value    = gptimer_get_top_value,
    .get_freq         = gptimer_get_freq,
    .set_alarm        = gptimer_set_alarm,
    .cancel_alarm     = gptimer_cancel_alarm,
    .get_guard_period = gptimer_get_guard_period,
    .set_guard_period = gptimer_set_guard_period,
};

/* ------------------------------------------------------------------ */
/* Init — mirrors baremetal GPT_Init flow exactly                     */
/* ------------------------------------------------------------------ */

static int gptimer_init(const struct device *dev)
{
    const struct gptimer_counter_config *cfg = dev->config;
    GPTIMER_Type *regs = get_regs(dev);

    /* Step 1: Clear UPDATE_EN before touching PERIOD/prescaler */
    if (regs->CLOCK_CTRL & GPT_CHECK_UPDATE_EN) {
        regs->CLOCK_CTRL &= GPT_CLK_EN;
    }

    /* Step 2: Set PERIOD to max for free-running */
    regs->PERIOD = 0xFFFFFFFFU;

    /* Step 3: Write prescaler — matches baremetal exactly:
     *   CLOCK_CTRL |= (GPT_CLK_PRESCALER(x) & GPT_PRSC_EN)
     *   GPT_PRSC_EN = 0xFFFFFFFE preserves CLK_SRC bit 0
     */
    if (cfg->prescaler == 0U) {
        return -EINVAL;
    }
    regs->CLOCK_CTRL |= (GPT_CLK_PRESCALER(cfg->prescaler) & GPT_PRSC_EN);

    /* Step 4: Write CTRL — matches baremetal:
     *   CTRL = GPT_EN | GPT_MODE(x) | control | GPT_COUNT_RESET
     *   Interrupts NOT enabled here — enabled only when alarm/top set
     */
    regs->CTRL = GPT_EN                  |
                 GPT_MODE(cfg->mode)     |
                 GPT_CONTIN_CNT_EN       |
                 GPT_COUNT_RESET;

    /* Step 5: Connect PLIC IRQ */
    cfg->irq_config_func(dev);

    return 0;
}

/* ------------------------------------------------------------------ */
/* Per-instance macro                                                  */
/* ------------------------------------------------------------------ */

#define GPTIMER_DEVICE(inst)                                                    \
                                                                                \
    static void gptimer_irq_config_##inst(const struct device *dev)             \
    {                                                                           \
        ARG_UNUSED(dev);                                                        \
        IRQ_CONNECT(DT_INST_IRQN(inst),                                         \
                    DT_INST_IRQ(inst, priority),                                \
                    gptimer_isr,                                                \
                    DEVICE_DT_INST_GET(inst),                                   \
                    0);                                                         \
        riscv_plic_set_priority(DT_INST_IRQN(inst),                             \
                                DT_INST_IRQ(inst, priority));                   \
        irq_enable(DT_INST_IRQN(inst));                                         \
        printk("[GPT%d] flat_irq=%d plic_en=%d\n", inst,                        \
               DT_INST_IRQN(inst),                                              \
               riscv_plic_irq_is_enabled(DT_INST_IRQN(inst)));                  \
    }                                                                           \
                                                                                \
    static const struct gptimer_counter_config gpt_config_##inst = {            \
        .info = {                                                               \
            .max_top_value = 0xFFFFFFFFU,                                       \
            .freq          = DT_INST_PROP(inst, clock_frequency),               \
            .flags         = COUNTER_CONFIG_INFO_COUNT_UP,                      \
            .channels      = 1,                                                 \
        },                                                                      \
        .base_addr       = DT_INST_REG_ADDR(inst),                              \
        .freq_hz         = DT_INST_PROP(inst, clock_frequency),                 \
        .prescaler       = DT_INST_PROP(inst, prescaler),                       \
        .mode            = DT_INST_PROP(inst, timer_mode),                      \
        .irq_config_func = gptimer_irq_config_##inst,                           \
    };                                                                          \
                                                                                \
    static struct gptimer_counter_data gpt_data_##inst = { 0 };                 \
                                                                                \
    DEVICE_DT_INST_DEFINE(inst,                                                 \
                          gptimer_init,                                         \
                          NULL,                                                 \
                          &gpt_data_##inst,                                     \
                          &gpt_config_##inst,                                   \
                          POST_KERNEL,                                          \
                          CONFIG_COUNTER_INIT_PRIORITY,                         \
                          &gptimer_api);

DT_INST_FOREACH_STATUS_OKAY(GPTIMER_DEVICE)