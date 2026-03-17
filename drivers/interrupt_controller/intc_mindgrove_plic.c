/* 
 * @copyright Copyright (c) Mindgrove Technologies Pvt. Ltd 2025. All rights reserved.
 * 
 * SPDX-License-Identifier: Apache-2.0
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

LOG_MODULE_REGISTER(intc_plic, LOG_LEVEL_DBG);

#ifndef PLIC_NODE
#define PLIC_NODE DT_NODELABEL(plic0)
#endif

#define PLIC_BASE_ADDR     DT_REG_ADDR(PLIC_NODE)
#define PLIC_REG_SIZE      DT_REG_SIZE(PLIC_NODE)
#define PLIC_CPU_IRQ_NUM   DT_IRQN(PLIC_NODE)
#define PLIC_MAX_SOURCES   DT_PROP(PLIC_NODE, riscv_ndev)
#define PLIC_MIN_IRQ       1U

#define PLIC_PRIORITY_OFFSET   0x0000U  
#define PLIC_PENDING_OFFSET    0x1000U  
#define PLIC_ENABLE_OFFSET     0x2000U  
#define PLIC_THRESHOLD_OFFSET  0x200000U 
#define PLIC_CLAIM_OFFSET      0x200004U 
#define PLIC_PRIORITY_STRIDE   4U
#define PLIC_ENABLE_UNIT_BITS  32U 
#define PLIC_ENABLE_UNIT_SIZE  4U 

static inline uint32_t plic_read32(uintptr_t a) { return sys_read32((mem_addr_t)a); }
static inline void plic_write32(uintptr_t a, uint32_t v) { sys_write32(v, (mem_addr_t)a); }

static inline bool plic_addr_valid(uintptr_t addr)
{
    if (PLIC_BASE_ADDR == 0) return false;
    if (addr < PLIC_BASE_ADDR) return false;
    uintptr_t off = addr - PLIC_BASE_ADDR;
    return (off < (uintptr_t)PLIC_REG_SIZE);
}

static inline uintptr_t plic_ctx_threshold_addr(uintptr_t base, unsigned ctx)
{
    return base + PLIC_THRESHOLD_OFFSET + (ctx * 0x1000U);
}

uint32_t riscv_plic_get_irq(void)
{
    uintptr_t claim = PLIC_BASE_ADDR + PLIC_CLAIM_OFFSET;
    if (!plic_addr_valid(claim)) return 0U;
    return plic_read32(claim);
}

void riscv_plic_complete(uint32_t irq)
{
    uintptr_t claim = PLIC_BASE_ADDR + PLIC_CLAIM_OFFSET;
    if (plic_addr_valid(claim)) {
        plic_write32(claim, irq);
        __asm__ volatile ("fence iorw, iorw" ::: "memory");
    }
}

void riscv_plic_set_priority(uint32_t irq, uint32_t priority)
{
    if (irq < PLIC_MIN_IRQ || irq > PLIC_MAX_SOURCES) return;

    uintptr_t prio_addr = PLIC_BASE_ADDR + PLIC_PRIORITY_OFFSET + (irq * PLIC_PRIORITY_STRIDE);
    if (plic_addr_valid(prio_addr)) {
        plic_write32(prio_addr, priority);
        __asm__ volatile ("fence iorw, iorw" ::: "memory");
    }
}

void riscv_plic_irq_enable(uint32_t irq)
{
    if (irq < PLIC_MIN_IRQ || irq > PLIC_MAX_SOURCES) return;

    uintptr_t word_addr = PLIC_BASE_ADDR + PLIC_ENABLE_OFFSET + ((irq / PLIC_ENABLE_UNIT_BITS) * PLIC_ENABLE_UNIT_SIZE);
    uint32_t bit_mask = (1U << (irq % PLIC_ENABLE_UNIT_BITS));

    if (plic_addr_valid(word_addr)) {
        uint32_t val = plic_read32(word_addr);
        val |= bit_mask; 
        plic_write32(word_addr, val);
        __asm__ volatile ("fence iorw, iorw" ::: "memory");
    }
}

void riscv_plic_irq_disable(uint32_t irq)
{
    if (irq < PLIC_MIN_IRQ || irq > PLIC_MAX_SOURCES) return;

    uintptr_t word_addr = PLIC_BASE_ADDR + PLIC_ENABLE_OFFSET + ((irq / PLIC_ENABLE_UNIT_BITS) * PLIC_ENABLE_UNIT_SIZE);
    uint32_t bit_mask = (1U << (irq % PLIC_ENABLE_UNIT_BITS));

    if (plic_addr_valid(word_addr)) {
        uint32_t val = plic_read32(word_addr);
        val &= ~bit_mask; 
        plic_write32(word_addr, val);
        __asm__ volatile ("fence iorw, iorw" ::: "memory");
    }
}

const struct device *riscv_plic_get_dev(void)
{
    return DEVICE_DT_INST_GET(0);
}

static void plic_ext_handler(const void *arg)
{
    ARG_UNUSED(arg);
    while (1) {
        uint32_t src = riscv_plic_get_irq();
        if (src == 0U) break;
        
        if (src < PLIC_MIN_IRQ || src > PLIC_MAX_SOURCES) {
            riscv_plic_complete(src);
            continue;
        }

        if (_sw_isr_table[src].isr) {
            _sw_isr_table[src].isr(_sw_isr_table[src].arg);
        }
        riscv_plic_complete(src);
    }
    LOG_INF("PLIC CPU IRQ enabled via workqueue.");
}

/* Config and Init */
struct plic_config { uintptr_t base; unsigned int ext_irq; unsigned int ctx_index; };
struct plic_data { };

static const struct plic_config plic_config_0 = {
    .base = PLIC_BASE_ADDR,
    .ext_irq = PLIC_CPU_IRQ_NUM,
    .ctx_index = 0,
};

static int plic_init(const struct device *dev)
{
    const struct plic_config *cfg = dev->config;
    uintptr_t base = cfg->base;
    unsigned int ctx = cfg->ctx_index;

    if (base == 0) return -ENODEV;

    /* 1. Clear all Enables First */
    unsigned int num_enable_words = (PLIC_MAX_SOURCES / PLIC_ENABLE_UNIT_BITS) + 1U;
    for (unsigned i = 0; i < num_enable_words; ++i) {
        uintptr_t addr = base + PLIC_ENABLE_OFFSET + (i * PLIC_ENABLE_UNIT_SIZE); 
        if (plic_addr_valid(addr)) {
             plic_write32(addr, 0U);
        }
    }
    
    /* 2. Clear all Priorities */
    for (unsigned i = PLIC_MIN_IRQ; i <= PLIC_MAX_SOURCES; ++i) {
        riscv_plic_set_priority(i, 0U);
    }

    /* 3. Connect the CPU aggregated IRQ */
    IRQ_CONNECT(PLIC_CPU_IRQ_NUM,
                0,
                plic_ext_handler,
                NULL,
                0);

    /* 4. Lower Threshold IMMEDIATELY to allow interrupts */
    plic_write32(plic_ctx_threshold_addr(base, ctx), 0U);
    
    /* 5. Enable the CPU's external interrupt line (IRQ 11) */
    irq_enable(PLIC_CPU_IRQ_NUM);

    LOG_INF("PLIC init OK. Threshold=0. CPU IRQ %d Enabled.", PLIC_CPU_IRQ_NUM);
    return 0;
}

DEVICE_DT_INST_DEFINE(0, plic_init, NULL, NULL, &plic_config_0,
                      PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, NULL);

