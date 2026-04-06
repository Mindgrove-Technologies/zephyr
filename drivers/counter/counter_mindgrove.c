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
#include <stdint.h>
/**
  * @brief General purpose timer (GPTIMER0)
  */

typedef struct {                                /*!< GPTIMER0 Structure                                                        */
  
  union {
    volatile uint16_t CTRL;                        /*!< Control register                                                          */
    
    struct {
      volatile uint16_t CTRL_EN    : 1;            /*!< Timer enable flag                                                         */
            uint16_t            : 1;
      volatile uint16_t CTRL_MODE  : 2;            /*!< Timer mode select: 0=PWM, 1=Down, 2=Up, 3=UpDown                          */
      volatile uint16_t CTRL_OUTPUT_EN : 1;        /*!< Timer output enable bit                                                   */
      volatile uint16_t CTRL_COUNT_RESET : 1;      /*!< Timer counter reset bit                                                   */
      volatile uint16_t CTRL_CNT_COUNT_EN : 1;     /*!< Timer continuous count enable                                             */
      volatile uint16_t CTRL_PWM_FALL_INTR_EN : 1; /*!< PWM fall interupt enable                                                  */
      volatile uint16_t CTRL_PWM_RISE_INTR_EN : 1; /*!< PWM rise interupt enable                                                  */
      volatile uint16_t CTRL_OFLOW_INTR_EN : 1;    /*!< Counter overflow interrupt enable                                         */
      volatile uint16_t CTRL_UFLOW_INTR_EN : 1;    /*!< Counter underflow interrupt enable                                        */
      volatile  uint16_t CTRL_PWM_FALL_INTR : 1;    /*!< PWM fall interupt bit                                                     */
      volatile  uint16_t CTRL_PWM_RISE_INTR : 1;    /*!< PWM rise interupt bit                                                     */
      volatile  uint16_t CTRL_OFLOW_INTR : 1;       /*!< Counter overflow interrupt bit                                            */
      volatile  uint16_t CTRL_UFLOW_INTR : 1;       /*!< Counter underflow interrupt bit                                           */
      volatile uint16_t CTRL_CAPTURE_INP_EN : 1;   /*!< Counter capture input enable                                              */
    } CTRL_b;
  } ;
  volatile  uint16_t  RESERVED;
  
  union {
    volatile uint32_t CLOCK_CTRL;                  /*!< Clock control register                                                    */
    
    struct {
      volatile uint32_t CLK_SRC    : 1;            /*!< GPTIMER clock select bit                                                  */
      volatile uint32_t CLK_PRESCALAR : 16;        /*!< GPTIMER prescalar value                                                   */
      volatile uint32_t UPDATE_EN  : 1;            /*!< GPTIMER update enable                                                     */
            uint32_t            : 14;
    } CLOCK_CTRL_b;
  } ;
  volatile  uint32_t  COUNT;                        /*!< Counter register                                                          */
  volatile  uint32_t  RPTD_COUNT;                   /*!< Repeated count register                                                   */
  volatile uint32_t  DUTY_CYCLE;                   /*!< PWM duty cycle register                                                   */
  volatile uint32_t  PERIOD;                       /*!< PWM period register                                                       */
  volatile uint32_t  CAPTURE_INP;                  /*!< Timer capture input register                                              */
} GPTIMER_Type;    


/*
 * PLIC offset: number of CPU-level IRQ slots before PLIC sources start.
 * Always use: zirq = DT_INST_IRQ_BY_IDX(n, 0, irq) + PLIC_OFFSET
 */
#define PLIC_OFFSET CONFIG_2ND_LVL_ISR_TBL_OFFSET

/*--------------------------------------
  CTRL register bit macros
--------------------------------------*/
#define GPT_EN                   (1U << 0)  
#define GPT_MODE(x)              ((x) << 2) 
#define GPT_OUTPUT_EN            (1U << 4)  
#define GPT_COUNT_RESET          (1U << 5) 
#define GPT_CONTIN_CNT_EN        (1U << 6)  
#define GPT_PWM_FALL_INTR_EN     (1U << 7)  
#define GPT_PWM_RISE_INTR_EN     (1U << 8)  
#define GPT_CNTR_OFLOW_INTR_EN   (1U << 9)  
#define GPT_CNTR_UFLOW_INTR_EN   (1U << 10) 
#define GPT_CAPTURE_INP_EN       (1U << 15) 

/*--------------------------------------
  CLOCK register macros
--------------------------------------*/
// Bit specifications of GPTimer CLOCK CTRL register
#define GPT_CLK_PRESCALER(x) ((uint32_t)(x) << 1U)
#define GPT_CHECK_UPDATE_EN 0x20000U
#define GPT_CLK_EN 0xFFFDFFFFU
#define GPT_PRSC_EN 0xFFFFFFFEU

/* Minimal inline helpers */
static inline void gptimer_reset_and_enable(GPTIMER_Type *regs) {
    regs->CTRL |= GPT_COUNT_RESET | GPT_EN;
}

static inline void gptimer_set_prescaler(GPTIMER_Type *regs, uint32_t presc)
{
    /* Clear prescaler bits (bits 1–16) */
    regs->CLOCK_CTRL &= ~(0xFFFF << 1);

    /* Set prescaler */
    regs->CLOCK_CTRL |= ((presc & 0xFFFF) << 1);
}

/* Calculate Zephyr IRQ */
static inline uint32_t get_zirq(uint32_t plic_src)
{
    return plic_src + PLIC_OFFSET;
}

/*--------------------------------------
  Runtime data
--------------------------------------*/
struct gptimer_counter_data {
    counter_alarm_callback_t alarm_cb; 
    void *alarm_user_data;             
    uint32_t guard_period;             
};

/*--------------------------------------
  Device configuration
--------------------------------------*/
struct gptimer_counter_config {
    uint32_t base_addr; /* Store as uint32_t to pull directly from DT */
    uint32_t freq_hz;   
    uint32_t max_top;   
    uint8_t flags;      
    uint32_t plic_src;
    uint32_t prescaler;
    uint32_t priority;
    void (*irq_config_func)(const struct device *dev);
};

/*--------------------------------------
  Counter API functions
--------------------------------------*/

static int gptimer_start(const struct device *dev)
{
    const struct gptimer_counter_config *cfg = dev->config;
    GPTIMER_Type *regs = (GPTIMER_Type *)cfg->base_addr;
    gptimer_reset_and_enable(regs);
    return 0;
}

static int gptimer_stop(const struct device *dev)
{
    const struct gptimer_counter_config *cfg = dev->config;
    GPTIMER_Type *regs = (GPTIMER_Type *)cfg->base_addr;
    regs->CTRL &= ~GPT_EN;
    return 0;
}

static int gptimer_get_value(const struct device *dev, uint32_t *ticks)
{
    const struct gptimer_counter_config *cfg = dev->config;
    GPTIMER_Type *regs = (GPTIMER_Type *)cfg->base_addr;
    if (!ticks) return -EINVAL;
    *ticks = regs->COUNT;
    return 0;
}

static int gptimer_set_top_value(const struct device *dev,
                                 const struct counter_top_cfg *top_cfg)
{
    const struct gptimer_counter_config *cfg = dev->config;
    GPTIMER_Type *regs = (GPTIMER_Type *)cfg->base_addr;

    if (top_cfg->ticks > 0xFFFF) {
        return -ENOTSUP; 
    }
    /* Latch the shadow register into the active logic */
    if ((regs->CLOCK_CTRL & GPT_CHECK_UPDATE_EN) != 0U) {
        regs->CLOCK_CTRL &= GPT_CLK_EN;
    }

    /* Write to the shadow register */
    regs->PERIOD = top_cfg->ticks;

    if (!(top_cfg->flags & COUNTER_TOP_CFG_DONT_RESET)) {
        regs->CTRL |= GPT_COUNT_RESET;
    }

    return 0;
}

static uint32_t gptimer_get_top_value(const struct device *dev)
{
    const struct gptimer_counter_config *cfg = dev->config;
    GPTIMER_Type *regs = (GPTIMER_Type *)cfg->base_addr;
    return regs->PERIOD;
}

static uint32_t gptimer_get_freq(const struct device *dev)
{
    const struct gptimer_counter_config *cfg = dev->config;
    return cfg->freq_hz;
}

static int gptimer_set_alarm(const struct device *dev, uint8_t chan_id,
                             const struct counter_alarm_cfg *alarm_cfg)
{
    if (chan_id != 0) return -ENOTSUP; 
    struct gptimer_counter_data *data = dev->data;
    const struct gptimer_counter_config *cfg = dev->config;
    GPTIMER_Type *regs = (GPTIMER_Type *)cfg->base_addr;

    if (!alarm_cfg || !alarm_cfg->callback) return -EINVAL;

    data->alarm_cb = alarm_cfg->callback;
    data->alarm_user_data = alarm_cfg->user_data;

    /* Enable hardware interrupt flag for overflow */
    regs->CTRL |= GPT_CNTR_OFLOW_INTR_EN;

    return 0;
}

static int gptimer_cancel_alarm(const struct device *dev, uint8_t chan_id)
{
    if (chan_id != 0) return -ENOTSUP;
    struct gptimer_counter_data *data = dev->data;
    const struct gptimer_counter_config *cfg = dev->config;
    GPTIMER_Type *regs = (GPTIMER_Type *)cfg->base_addr;

    data->alarm_cb = NULL;
    data->alarm_user_data = NULL;
    
    /* Disable hardware interrupt flag */
    regs->CTRL &= ~GPT_CNTR_OFLOW_INTR_EN;

    return 0;
}

/*--------------------------------------
  ISR for GPTIMER instance
--------------------------------------*/
static void gptimer_isr(const void *arg)
{
    const struct device *dev = arg;
    const struct gptimer_counter_config *cfg = dev->config;
    struct gptimer_counter_data *data = dev->data;
    GPTIMER_Type *regs = (GPTIMER_Type *)cfg->base_addr;

    uint32_t ticks = regs->COUNT;

    /* Clear interrupt by resetting counter/flag */
    regs->CTRL |= GPT_COUNT_RESET;

    /* Invoke user callback if registered */
    if (data->alarm_cb) {
        data->alarm_cb(dev, 0, ticks, data->alarm_user_data);
    }
}

static const struct counter_driver_api gptimer_api = {
    .start       = gptimer_start,
    .stop        = gptimer_stop,
    .get_value   = gptimer_get_value,
    .set_top_value = gptimer_set_top_value,
    .get_top_value = gptimer_get_top_value,
    .get_freq    = gptimer_get_freq,
    .set_alarm   = gptimer_set_alarm,
    .cancel_alarm= gptimer_cancel_alarm,
};

/*--------------------------------------
  INIT
--------------------------------------*/
/* 
 * === Zephyr Counter Adaptation ===
 *
 * GPTimer hardware is inherently period-based and resets on reaching PERIOD.
 * However, Zephyr expects a monotonic free-running counter.
 * To emulate this behavior:
 * - Set PERIOD to maximum (0xFFFFFFFF) to minimize wraparound frequency
 */

static int gptimer_init(const struct device *dev)
{
    const struct gptimer_counter_config *cfg = dev->config;
    GPTIMER_Type *regs = (GPTIMER_Type *)cfg->base_addr;

    /* Clear update_enable if it's currently set */
    if ((regs->CLOCK_CTRL & GPT_CHECK_UPDATE_EN) != 0U) {
        regs->CLOCK_CTRL &= GPT_CLK_EN;
    }

    /* 
    * Set PERIOD to maximum to emulate free-running counter.
    * This avoids frequent resets and allows long continuous counting.
    */
    regs->PERIOD = cfg->max_top;

    /* 3. Write Prescaler to CLK_CTRL Register 
     */
    if (cfg->prescaler != 0U) {
        regs->CLOCK_CTRL |= (GPT_CLK_PRESCALER(cfg->prescaler) & GPT_PRSC_EN);
    } else {
        return -EINVAL;
    }


    /* 
    * Configure control register:
    * - Enable timer
    * - Set UP counting mode (monotonic increment)
    * - Enable continuous counting
    * - Reset counter to start from known state
    */
    regs->CTRL = (uint32_t)(GPT_EN | 
                           GPT_MODE(1) |          /* Default mode as UP COUNTER Mode*/
                           GPT_CONTIN_CNT_EN | 
                           GPT_COUNT_RESET);

    cfg->irq_config_func(dev);

    return 0;
}

/*--------------------------------------
  Device Instantiation Macro
--------------------------------------*/
#define GPTIMER_CUSTOM_DEVICE(inst)                                             \
    static void gptimer_irq_config_##inst(const struct device *dev)             \
    {                                                                           \
        uint32_t plic_src = DT_INST_IRQ_BY_IDX(inst, 0, irq);                   \
        uint32_t zirq = plic_src + PLIC_OFFSET;                                 \
        uint32_t prio = DT_INST_IRQ_BY_IDX(inst, 0, priority);                  \
                                                                                \
        if (zirq >= CONFIG_NUM_IRQS) {                                          \
            printk("  ERROR: zirq out of range!\n");                            \
        }                                                                       \
                                                                                \
        riscv_plic_set_priority(zirq, prio);                                    \
        irq_connect_dynamic(zirq, prio, gptimer_isr,                            \
                            DEVICE_DT_INST_GET(inst), 0);                       \
        irq_enable(zirq);                                                       \
        printk("  GPTIMER%d irq connected & enabled\n", inst);                  \
    }                                                                           \
    static const struct gptimer_counter_config gpt_config_##inst = {            \
        .base_addr       = DT_INST_REG_ADDR(inst),                              \
        .freq_hz         = 30810U, /* Can also be mapped to DT if needed */   \
        .max_top         = 0xFFFFFFFF,                                              \
        .flags           = COUNTER_CONFIG_INFO_COUNT_UP,                        \
        .plic_src        = DT_INST_IRQ_BY_IDX(inst, 0, irq),                    \
        .priority        = DT_INST_IRQ_BY_IDX(inst, 0, priority),               \
        .prescaler       = 1,                                                  \
        .irq_config_func = gptimer_irq_config_##inst,                           \
    };                                                                          \
    static struct gptimer_counter_data gpt_data_##inst = {0};                   \
    DEVICE_DT_INST_DEFINE(inst, gptimer_init, NULL,                             \
                          &gpt_data_##inst, &gpt_config_##inst,                 \
                          POST_KERNEL, CONFIG_COUNTER_INIT_PRIORITY,            \
                          &gptimer_api);

DT_INST_FOREACH_STATUS_OKAY(GPTIMER_CUSTOM_DEVICE)