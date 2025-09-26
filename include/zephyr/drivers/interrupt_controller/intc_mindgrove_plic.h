/*
 * @copyright Copyright (c) Mindgrove Technologies Pvt. Ltd 2025. All rights reserved.
 * 
 * Global interrupt data maintenance structure
*/

#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <soc.h>

#include <zephyr/sw_isr_table.h>
#include <zephyr/drivers/interrupt_controller/riscv_plic.h>
#include <zephyr/irq.h>

enum{
   PLIC_PRIORITY_1,
   PLIC_PRIORITY_2,
   PLIC_PRIORITY_3,
   PLIC_PRIORITY_4,
   PLIC_PRIORITY_5,
   PLIC_PRIORITY_6,
   PLIC_PRIORITY_7 
};

void plic_mindgrove_irq_handler(const void *arg);
int8_t plic_mindgrove_irq_claim(uint32_t interrupt_id);
int8_t plic_mindgrove_irq_enable(uint32_t interrupt_id);
int8_t plic_mindgrove_irq_disable(uint32_t interrupt_id);
int8_t plic_mindgrove_irq_threshold(uint32_t priority_val);
int8_t plic_mindgrove_irq_priority(uint32_t interrupt_id, uint32_t priority_val);
int8_t plic_mindgrove_irq_pending(uint32_t interrupt_id);
int plic_mindgrove_init(const struct device *dev);

// static inline void isr_default(uint32_t interrupt_id);
// unsigned int riscv_plic_get_irq(void);
// void interrupt_disable(uint32_t id);
// void interrupt_priority(uint32_t id);
// // int plic_mindgrove_init(const struct device *dev);
// uint32_t plic_get_irq_id(uint32_t int_num);