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


void plic_irq_handler( const void *arg);
static inline void isr_default(uint32_t interrupt_id);
int riscv_plic_get_irq(void);
void plic_irq_enable(uint32_t interrupt_id);
void interrupt_disable(uint32_t id);
void interrupt_priority(uint32_t id);
void plic_mindgrove_irq_disable(uint32_t interrupt_id);
void plic_mindgrove_set_irq_threshold(uint32_t priority_value);
void plic_mindgrove_set_priority(uint32_t priority_value, uint32_t int_id);
int plic_mindgrove_init(const struct device *dev);
uint32_t plic_get_irq_id(uint32_t int_num);