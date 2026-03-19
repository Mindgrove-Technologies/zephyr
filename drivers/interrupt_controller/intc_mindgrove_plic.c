/*
 * Mindgrove PLIC Driver
 *
 * Zephyr IRQ = PLIC source + CONFIG_2ND_LVL_ISR_TBL_OFFSET (= 41).
 * DT_IRQN() is unreliable on this SoC — it encodes priority bits into
 * the IRQ number. Always use DT_IRQ_BY_IDX(node, idx, irq) + offset.
 */

#define DT_DRV_COMPAT mindgrove_plic

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/irq.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/interrupt_controller/riscv_plic.h>
#include <zephyr/sw_isr_table.h>

LOG_MODULE_REGISTER(intc_plic, LOG_LEVEL_INF);

#ifndef PLIC_NODE
#define PLIC_NODE DT_NODELABEL(plic0)
#endif

#define PLIC_BASE        DT_REG_ADDR(PLIC_NODE)
#define PLIC_SIZE        DT_REG_SIZE(PLIC_NODE)
#define PLIC_CPU_IRQ     DT_IRQN(PLIC_NODE)
#define PLIC_MAX_SRC     DT_PROP(PLIC_NODE, riscv_ndev)
#define PLIC_MIN_SRC     1U

#define PLIC_PRIO_BASE   0x0000U
#define PLIC_PEND_BASE   0x1000U
#define PLIC_EN_BASE     0x2000U
#define PLIC_THRESHOLD   0x200000U
#define PLIC_CLAIM       0x200004U
#define PLIC_PRIO_STRIDE 4U
#define PLIC_THRESH_VAL  0U

/*
 * g_plic_offset: number of CPU-level IRQ slots before PLIC sources start.
 * = CONFIG_2ND_LVL_ISR_TBL_OFFSET = 41 on this SoC.
 *
 * All peripheral drivers must compute their Zephyr IRQ as:
 *   zirq = DT_INST_IRQ_BY_IDX(n, 0, irq) + g_plic_offset
 *
 * Never use DT_INST_IRQN() — it encodes priority bits on this SoC.
 */
static uint32_t g_plic_offset = CONFIG_2ND_LVL_ISR_TBL_OFFSET;

static inline uint32_t plic_rd(uint32_t off)
{
    return sys_read32((mem_addr_t)(PLIC_BASE + off));
}

static inline void plic_wr(uint32_t off, uint32_t val)
{
    sys_write32(val, (mem_addr_t)(PLIC_BASE + off));
}

/* Convert Zephyr IRQ to PLIC source. Returns 0 on error. */
static uint32_t to_src(uint32_t zirq)
{
    if (zirq < g_plic_offset) return 0U;
    uint32_t src = zirq - g_plic_offset;
    printk("to_src: zirq=%u offset=%u src=%u\n", zirq, g_plic_offset, src);
    if (src < PLIC_MIN_SRC || src > PLIC_MAX_SRC) {
        LOG_ERR("src %u out of range (zirq=%u offset=%u)",
                src, zirq, g_plic_offset);
        return 0U;
    }
    return src;
}

static void en_reg(uint32_t src, uint32_t *off, uint32_t *bit)
{
    *off = PLIC_EN_BASE + (src / 32U) * 4U;
    *bit = src % 32U;
}

/* --- Public API --- */

uint32_t riscv_plic_get_irq(void)
{
    return plic_rd(PLIC_CLAIM);
}

void riscv_plic_complete(uint32_t src)
{
    if (src == 0U) return;
    plic_wr(PLIC_CLAIM, src);
    __asm__ volatile("fence iorw, iorw" ::: "memory");
}

void riscv_plic_set_priority(uint32_t zirq, uint32_t prio)
{
    uint32_t src = (zirq >= g_plic_offset) ? to_src(zirq) : zirq;
    if (src < PLIC_MIN_SRC || src > PLIC_MAX_SRC) return;
    plic_wr(PLIC_PRIO_BASE + src * PLIC_PRIO_STRIDE, prio);
    __asm__ volatile("fence iorw, iorw" ::: "memory");
}

void riscv_plic_irq_enable(uint32_t zirq)
{
    uint32_t src = to_src(zirq);
    if (src == 0U) return;
    uint32_t off, bit;
    en_reg(src, &off, &bit);
    plic_wr(off, plic_rd(off) | BIT(bit));
    __asm__ volatile("fence iorw, iorw" ::: "memory");
}

void riscv_plic_irq_disable(uint32_t zirq)
{
    uint32_t src = to_src(zirq);
    if (src == 0U) return;
    uint32_t off, bit;
    en_reg(src, &off, &bit);
    plic_wr(off, plic_rd(off) & ~BIT(bit));
    __asm__ volatile("fence iorw, iorw" ::: "memory");
}

uint32_t riscv_plic_get_pending(uint32_t src)
{
    if (src < PLIC_MIN_SRC || src > PLIC_MAX_SRC) return 0U;
    return (plic_rd(PLIC_PEND_BASE + (src / 32U) * 4U) >> (src % 32U)) & 1U;
}

const struct device *riscv_plic_get_dev(void)
{
    return DEVICE_DT_INST_GET(0);
}

/* Top-level machine external interrupt handler */
static void plic_ext_handler(const void *arg)
{
    ARG_UNUSED(arg);

    uint32_t src = riscv_plic_get_irq();
    if (src == 0U || src > PLIC_MAX_SRC) {
        riscv_plic_complete(src);
        return;
    }

    uint32_t idx = src + g_plic_offset;
    struct _isr_table_entry *e = &_sw_isr_table[idx];
    if (e->isr != NULL) {
        e->isr(e->arg);
    } else {
        LOG_WRN("no ISR for src=%u idx=%u", src, idx);
    }

    riscv_plic_complete(src);
}

struct plic_cfg { uintptr_t base; unsigned int irq; };
struct plic_dat { };

static const struct plic_cfg plic_cfg0 = {
    .base = PLIC_BASE,
    .irq  = PLIC_CPU_IRQ,
};
static struct plic_dat plic_dat0;

static int plic_init(const struct device *dev)
{
    ARG_UNUSED(dev);

    /* g_plic_offset is already set to CONFIG_2ND_LVL_ISR_TBL_OFFSET */
    printk("[PLIC] base=0x%lx cpu_irq=%u max_src=%u offset=%u\n",
           (unsigned long)PLIC_BASE, PLIC_CPU_IRQ, PLIC_MAX_SRC, g_plic_offset);

    /* Clear all enable registers */
    for (unsigned int i = 0; i < ((PLIC_MAX_SRC + 31U) / 32U); i++) {
        plic_wr(PLIC_EN_BASE + i * 4U, 0U);
    }

    /* Clear all source priorities */
    for (unsigned int i = PLIC_MIN_SRC; i <= PLIC_MAX_SRC; i++) {
        plic_wr(PLIC_PRIO_BASE + i * PLIC_PRIO_STRIDE, 0U);
    }

    /* Set threshold — only sources with priority > threshold fire */
    plic_wr(PLIC_THRESHOLD, PLIC_THRESH_VAL);

    /* Connect and enable CPU-level machine external interrupt */
    IRQ_CONNECT(PLIC_CPU_IRQ, 0, plic_ext_handler, NULL, 0);
    irq_enable(PLIC_CPU_IRQ);

    return 0;
}

DEVICE_DT_INST_DEFINE(0, plic_init, NULL,
                      &plic_dat0, &plic_cfg0,
                      PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
                      NULL);