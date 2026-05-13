/*
 * Mindgrove SoC PLIC driver
 * Zephyr 3.2 + 4.4 compatible
 */

#define DT_DRV_COMPAT mindgrove_plic

#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/devicetree/interrupt_controller.h>
#include <zephyr/irq.h>
#include <zephyr/sw_isr_table.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/drivers/interrupt_controller/riscv_plic.h>
#include <zephyr/logging/log.h>
#include <zephyr/version.h>
#include "sw_isr_common.h"

LOG_MODULE_REGISTER(plic_mindgrove, LOG_LEVEL_INF);

/* ------------------------------------------------------------------ */
/* Version compatibility guards                                        */
/* ------------------------------------------------------------------ */

/*
 * isr_table in plic_config gained const in 4.0
 */
#if KERNEL_VERSION_NUMBER >= 0x040000
#define ISR_TABLE_CONST const
#else
#define ISR_TABLE_CONST
#endif

/*
 * DT_INST_INTC_GET_AGGREGATOR_LEVEL was added after 3.2.
 * On 3.2 the PLIC is always level 2.
 */
#ifndef DT_INST_INTC_GET_AGGREGATOR_LEVEL
#define DT_INST_INTC_GET_AGGREGATOR_LEVEL(n) 2
#endif

/*
 * K_SPINLOCK() statement-expression form added after 3.2.
 * Provide a fallback using the explicit lock/unlock form.
 */
#ifndef K_SPINLOCK
#define K_SPINLOCK(lk)                                   \
    for (k_spinlock_key_t _key = k_spin_lock(lk),       \
         _once = (k_spinlock_key_t){0};                  \
         !_once.__val;                                   \
         k_spin_unlock((lk), _key), _once.__val = 1)
#endif

/*
 * riscv_plic_irq_complete was added in 4.4.
 * We define it unconditionally — on 3.2 it simply won't be
 * declared in riscv_plic.h so we add a forward declaration.
 */
#if KERNEL_VERSION_NUMBER < 0x040000
void riscv_plic_irq_complete(uint32_t irq);
#endif

/* ------------------------------------------------------------------ */
/* Register offsets                                                    */
/* ------------------------------------------------------------------ */

#define CONTEXT_BASE         0x200000U
#define CONTEXT_SIZE         0x1000U
#define CONTEXT_THRESHOLD    0x00U
#define CONTEXT_CLAIM        0x04U
#define CONTEXT_ENABLE_BASE  0x2000U
#define CONTEXT_ENABLE_SIZE  0x80U

#define PLIC_REG_SIZE        32U
#define PLIC_REG_MASK        (PLIC_REG_SIZE - 1U)

/* ------------------------------------------------------------------ */
/* Test visibility                                                     */
/* ------------------------------------------------------------------ */

#ifdef CONFIG_TEST_INTC_PLIC
#define INTC_PLIC_STATIC
#define INTC_PLIC_STATIC_INLINE
#else
#define INTC_PLIC_STATIC        static
#define INTC_PLIC_STATIC_INLINE static inline
#endif

/* ------------------------------------------------------------------ */
/* Structs                                                             */
/* ------------------------------------------------------------------ */

typedef void (*plic_irq_config_func_t)(void);

struct plic_config {
    mem_addr_t  prio;
    mem_addr_t  irq_en;
    mem_addr_t  reg;
    uint32_t    max_prio;
    uint32_t    riscv_ndev;
    uint32_t    nr_irqs;
    uint32_t    irq;
    plic_irq_config_func_t irq_config_func;
    ISR_TABLE_CONST struct _isr_table_entry *isr_table;
    const uint32_t *const hart_context;
};

struct plic_data {
    struct k_spinlock lock;
};

static uint32_t             save_irq[CONFIG_MP_MAX_NUM_CPUS];
static const struct device *save_dev[CONFIG_MP_MAX_NUM_CPUS];

/* ------------------------------------------------------------------ */
/* Functions exposed for intc_plic ztest                               */
/* ------------------------------------------------------------------ */

uint32_t local_irq_to_reg_index(uint32_t local_irq)
{
	return local_irq / PLIC_REG_SIZE;
}

uint32_t local_irq_to_reg_offset(uint32_t local_irq)
{
	return local_irq_to_reg_index(local_irq) * sizeof(uint32_t);
}

/* ------------------------------------------------------------------ */
/* Address helpers                                                     */
/* ------------------------------------------------------------------ */

static ALWAYS_INLINE uint32_t get_hart_context(const struct device *dev,
                                                uint32_t hartid)
{
    const struct plic_config *cfg = dev->config;

    return cfg->hart_context[hartid];
}

static inline mem_addr_t get_context_en_addr(const struct device *dev,
                                              uint32_t cpu_num)
{
    const struct plic_config *cfg = dev->config;
    uint32_t hartid;

#if CONFIG_MP_MAX_NUM_CPUS > 1
    hartid = _kernel.cpus[cpu_num].arch.hartid;
#else
    ARG_UNUSED(cpu_num);
    hartid = arch_proc_id();
#endif

    return cfg->irq_en +
           get_hart_context(dev, hartid) * CONTEXT_ENABLE_SIZE;
}

static inline mem_addr_t get_claim_complete_addr(const struct device *dev)
{
    const struct plic_config *cfg = dev->config;

    return cfg->reg +
           get_hart_context(dev, arch_proc_id()) * CONTEXT_SIZE +
           CONTEXT_CLAIM;
}

static inline mem_addr_t get_threshold_addr(const struct device *dev,
                                             uint32_t cpu_num)
{
    const struct plic_config *cfg = dev->config;
    uint32_t hartid;

#if CONFIG_MP_MAX_NUM_CPUS > 1
    hartid = _kernel.cpus[cpu_num].arch.hartid;
#else
    ARG_UNUSED(cpu_num);
    hartid = arch_proc_id();
#endif

    return cfg->reg +
           get_hart_context(dev, hartid) * CONTEXT_SIZE +
           CONTEXT_THRESHOLD;
}

static inline uint32_t get_plic_enabled_size(const struct device *dev)
{
    const struct plic_config *cfg = dev->config;

    return local_irq_to_reg_index(cfg->nr_irqs) + 1U;
}

/* ------------------------------------------------------------------ */
/* Internal helpers                                                    */
/* ------------------------------------------------------------------ */

static const struct device *get_plic_dev_from_irq(uint32_t irq)
{
#ifdef CONFIG_DYNAMIC_INTERRUPTS
    return z_get_sw_isr_device_from_irq(irq);
#else
    return DEVICE_DT_INST_GET(0);
#endif
}

static void plic_irq_enable_set_state(uint32_t irq, bool enable)
{
    const struct device *dev = get_plic_dev_from_irq(irq);
    const uint32_t local_irq = irq_from_level_2(irq);

    for (uint32_t cpu = 0; cpu < arch_num_cpus(); cpu++) {
        mem_addr_t en_addr = get_context_en_addr(dev, cpu) +
                             local_irq_to_reg_offset(local_irq);
        uint32_t val = sys_read32(en_addr);

        WRITE_BIT(val, local_irq & PLIC_REG_MASK, enable);
        sys_write32(val, en_addr);
    }
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

void riscv_plic_irq_enable(uint32_t irq)
{
    const struct device *dev = get_plic_dev_from_irq(irq);
    struct plic_data *data = dev->data;
    k_spinlock_key_t key = k_spin_lock(&data->lock);

    plic_irq_enable_set_state(irq, true);
    k_spin_unlock(&data->lock, key);
}

void riscv_plic_irq_disable(uint32_t irq)
{
    const struct device *dev = get_plic_dev_from_irq(irq);
    struct plic_data *data = dev->data;
    k_spinlock_key_t key = k_spin_lock(&data->lock);

    plic_irq_enable_set_state(irq, false);
    k_spin_unlock(&data->lock, key);
}

int riscv_plic_irq_is_enabled(uint32_t irq)
{
    const struct device *dev = get_plic_dev_from_irq(irq);
    struct plic_data *data = dev->data;
    const uint32_t local_irq = irq_from_level_2(irq);
    int ret = 0;

    /* Use explicit lock/unlock — compatible with both 3.2 and 4.4 */
    k_spinlock_key_t key = k_spin_lock(&data->lock);

    mem_addr_t en_addr = get_context_en_addr(dev, 0) +
                         local_irq_to_reg_offset(local_irq);

    ret = !!(sys_read32(en_addr) & BIT(local_irq & PLIC_REG_MASK));

    k_spin_unlock(&data->lock, key);

    return ret;
}

void riscv_plic_set_priority(uint32_t irq, uint32_t priority)
{
    const struct device *dev = get_plic_dev_from_irq(irq);
    const struct plic_config *cfg = dev->config;
    const uint32_t local_irq = irq_from_level_2(irq);

    if (priority > cfg->max_prio) {
        priority = cfg->max_prio;
    }

    sys_write32(priority, cfg->prio + local_irq * sizeof(uint32_t));
}

unsigned int riscv_plic_get_irq(void)
{
    return save_irq[arch_curr_cpu()->id];
}

const struct device *riscv_plic_get_dev(void)
{
    return save_dev[arch_curr_cpu()->id];
}

void riscv_plic_irq_complete(uint32_t irq)
{
    const struct device *dev = get_plic_dev_from_irq(irq);
    const uint32_t local_irq = irq_from_level_2(irq);

    sys_write32(local_irq, get_claim_complete_addr(dev));
}

/* ------------------------------------------------------------------ */
/* ISR                                                                 */
/* ------------------------------------------------------------------ */

static void plic_irq_handler(const struct device *dev)
{
    const struct plic_config *cfg = dev->config;
    mem_addr_t claim_addr = get_claim_complete_addr(dev);
    uint32_t cpu_id = arch_curr_cpu()->id;
    const uint32_t local_irq = sys_read32(claim_addr);

    if ((CONFIG_MP_MAX_NUM_CPUS > 1) && (local_irq == 0U)) {
        return;
    }

    save_irq[cpu_id] = local_irq;
    save_dev[cpu_id] = dev;

    if ((local_irq == 0U) || (local_irq >= cfg->nr_irqs)) {
        z_irq_spurious(NULL);
        return;
    }

    const struct _isr_table_entry *e = &cfg->isr_table[local_irq];

    e->isr(e->arg);

    sys_write32(local_irq, claim_addr);
}

/* ------------------------------------------------------------------ */
/* Init                                                                */
/* ------------------------------------------------------------------ */

static int plic_init(const struct device *dev)
{
    const struct plic_config *cfg = dev->config;

    //printk("[PLIC] base=0x%lx\n", (unsigned long)cfg->prio);
    //printk("[PLIC] irq_en=0x%lx\n", (unsigned long)cfg->irq_en);
    //printk("[PLIC] reg=0x%lx\n", (unsigned long)cfg->reg);
    //printk("[PLIC] nr_irqs=%u max_prio=%u\n", cfg->nr_irqs, cfg->max_prio);
    //printk("[PLIC] hart_context[0]=%u\n", cfg->hart_context[0]);

    mem_addr_t en_addr = get_context_en_addr(dev, 0);
    mem_addr_t thres_addr = get_threshold_addr(dev, 0);
    //printk("[PLIC] en_addr=0x%lx thres_addr=0x%lx\n",
           //(unsigned long)en_addr, (unsigned long)thres_addr);

    /* Now do the actual init */
    for (uint32_t cpu = 0; cpu < arch_num_cpus(); cpu++) {
        en_addr    = get_context_en_addr(dev, cpu);
        thres_addr = get_threshold_addr(dev, cpu);

        //printk("[PLIC] clearing %u enable words at 0x%lx\n",
               //get_plic_enabled_size(dev), (unsigned long)en_addr);

        for (uint32_t i = 0; i < get_plic_enabled_size(dev); i++) {
            sys_write32(0U, en_addr + i * sizeof(uint32_t));
        }
        sys_write32(0U, thres_addr);
    }

    //printk("[PLIC] clearing %u priorities at 0x%lx\n",
           //cfg->nr_irqs, (unsigned long)cfg->prio);

    for (uint32_t i = 0U; i < cfg->nr_irqs; i++) {
        sys_write32(0U, cfg->prio + i * sizeof(uint32_t));
    }

    cfg->irq_config_func();
    //printk("[PLIC] init done\n");
    return 0;
}

/* ------------------------------------------------------------------ */
/* Hart context array                                                  */
/* ------------------------------------------------------------------ */

#define HART_CONTEXTS(i, n) \
    IF_ENABLED(IS_EQ(DT_INST_IRQN_BY_IDX(n, i), DT_INST_IRQN(n)), (i,))

#define PLIC_HART_CONTEXT_DECLARE(n)                                     \
    INTC_PLIC_STATIC const uint32_t                                      \
    plic_hart_contexts_##n[DT_CHILD_NUM(DT_PATH(cpus))] = {              \
        LISTIFY(DT_INST_NUM_IRQS(n), HART_CONTEXTS, (), n)               \
    }

/* ------------------------------------------------------------------ */
/* Per-instance macros                                                 */
/* ------------------------------------------------------------------ */

#define PLIC_MIN_IRQ_NUM(n) \
    MIN(DT_INST_PROP(n, riscv_ndev), CONFIG_MAX_IRQ_PER_AGGREGATOR)

#define PLIC_IRQ_FUNC_DECLARE(n) \
    static void plic_irq_config_func_##n(void)

#define PLIC_IRQ_FUNC_DEFINE(n)                                          \
    static void plic_irq_config_func_##n(void)                           \
    {                                                                    \
        IRQ_CONNECT(DT_INST_IRQN(n), 0,                                  \
                    plic_irq_handler, DEVICE_DT_INST_GET(n), 0);         \
        irq_enable(DT_INST_IRQN(n));                                     \
    }

#define PLIC_CONFIG_INIT(n)                                              \
    PLIC_IRQ_FUNC_DECLARE(n);                                            \
    PLIC_HART_CONTEXT_DECLARE(n);                                        \
    static const struct plic_config plic_config_##n = {                  \
        .prio            = DT_INST_REG_ADDR(n),                          \
        .irq_en          = DT_INST_REG_ADDR(n) + CONTEXT_ENABLE_BASE,   \
        .reg             = DT_INST_REG_ADDR(n) + CONTEXT_BASE,          \
        .max_prio        = DT_INST_PROP(n, riscv_max_priority),          \
        .riscv_ndev      = DT_INST_PROP(n, riscv_ndev),                 \
        .nr_irqs         = PLIC_MIN_IRQ_NUM(n),                          \
        .irq             = DT_INST_IRQN(n),                              \
        .irq_config_func = plic_irq_config_func_##n,                     \
        .isr_table       = &_sw_isr_table[INTC_INST_ISR_TBL_OFFSET(n)], \
        .hart_context    = plic_hart_contexts_##n,                       \
    };                                                                   \
    PLIC_IRQ_FUNC_DEFINE(n)

#define PLIC_DATA_INIT(n) \
    static struct plic_data plic_data_##n = { 0 };

#define PLIC_DEVICE_INIT(n)                                              \
    IRQ_PARENT_ENTRY_DEFINE(                                             \
        plic##n,                                                         \
        DEVICE_DT_INST_GET(n),                                           \
        DT_INST_IRQN(n),                                                 \
        INTC_INST_ISR_TBL_OFFSET(n),                                     \
        DT_INST_INTC_GET_AGGREGATOR_LEVEL(n));                           \
    PLIC_CONFIG_INIT(n)                                                  \
    PLIC_DATA_INIT(n)                                                    \
    DEVICE_DT_INST_DEFINE(n,                                             \
                          plic_init,                                     \
                          NULL,                                          \
                          &plic_data_##n,                                \
                          &plic_config_##n,                              \
                          PRE_KERNEL_1,                                  \
                          CONFIG_INTC_INIT_PRIORITY,                     \
                          NULL);

DT_INST_FOREACH_STATUS_OKAY(PLIC_DEVICE_INIT)