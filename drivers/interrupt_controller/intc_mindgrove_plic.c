// #include "pwm_driver.h"
// #include "plic_driver.h"
// #include "platform.h"
// #include "log.h"
// #include "stddef.h"
// #include "gpio.h"
// #include "utils.h"

/*
   Global interrupt data maintenance structure
*/

// Zephyr-plic_shakti

#define DT_DRV_COMPAT mindgrove_plic
#define PLIC_BASE_ADDRESS DT_INST_REG_ADDR(0)

//--------------------------------

#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <soc.h>
#include <zephyr/sw_isr_table.h>
#include <zephyr/sw_isr_table.h>
#include <zephyr/drivers/interrupt_controller/riscv_plic.h>
#include <zephyr/irq.h>
#include <zephyr/arch/riscv/arch_inlines.h>
#include <zephyr/drivers/interrupt_controller/intc_mindgrove_plic.h>
#include <zephyr/sw_isr_table.h>

//Defines

#define PLIC_PRIORITY_OFFSET            0x0000UL
#define PLIC_PENDING_OFFSET             0x1000UL
#define PLIC_ENABLE_OFFSET              0x2000UL

//#if defined(SOS) 
#define PLIC_THRESHOLD_OFFSET           0x200000UL
#define PLIC_CLAIM_OFFSET               0x200004UL

#define PLIC_REG_OFFSET                 ((uint32_t)PLIC_BASE_ADDRESS + PLIC_THRESHOLD_OFFSET)

#define PLIC_MAX_INTERRUPT_SRC          58 // set this value to CONFIG_NUM_IRQS
#define PLIC_EN_SIZE                    ((uint32_t)(PLIC_MAX_INTERRUPT_SRC/32) * (sizeof(uint32_t))) 
#define PLIC_PRIORITY_SHIFT_PER_INT     2 // to calculate offset for priority reg
// #define CONFIG_NUM_IRQS					7

#define PLIC_IRQ_PRIO CONFIG_PLIC_MEIP_PRIORITY

volatile static uint8_t nested_interrupt_mode = 0;
typedef struct mindgrove_plic_regs_t
{
    uint32_t priority_thershold;
    uint32_t claim_register;
    uint32_t interrupt_complete;

}plic_regs_t;

static int track_irq_num;
volatile int key=0;

static uint32_t save_irq[CONFIG_MP_MAX_NUM_CPUS];
static const struct device *save_dev[CONFIG_MP_MAX_NUM_CPUS];

extern struct _isr_table_entry _sw_isr_table[];

int8_t plic_mindgrove_irq_claim(uint32_t interrupt_id)
{
	if (interrupt_id >= PLIC_MAX_INTERRUPT_SRC)
	{
		return -1;
	}
	volatile uint32_t *claim_addr = (uint32_t *)(DT_REG_ADDR(DT_NODELABEL(plic)) + PLIC_CLAIM_OFFSET);
	*claim_addr = interrupt_id;
	return 0;
}

void plic_mindgrove_irq_handler(const void *arg)
{
	volatile uint32_t *claim_addr = (uint32_t *)(DT_REG_ADDR(DT_NODELABEL(plic)) + PLIC_CLAIM_OFFSET);
	volatile uint32_t plic_interrupt_src;

	while (1)
	{
		plic_interrupt_src = *claim_addr;
		if(plic_interrupt_src == 0)
		{
			break;
		}
		if (plic_interrupt_src < CONFIG_NUM_IRQS)
		{
			const struct _isr_table_entry *isr_entry = &_sw_isr_table[plic_interrupt_src];
			if (isr_entry->isr)
			{
				isr_entry->isr(isr_entry->arg);
			}
			
		}
		
	}

	*claim_addr = plic_interrupt_src;
}

int8_t plic_mindgrove_irq_enable(uint32_t interrupt_id)
{
	if (interrupt_id >= PLIC_MAX_INTERRUPT_SRC)
	{
		return -1;
	}
	volatile uint32_t *irq_enable_addr = (uint32_t*)(DT_REG_ADDR(DT_NODELABEL(plic)) + PLIC_ENABLE_OFFSET + ((interrupt_id / 32) * 4));
	*irq_enable_addr |= (0x1U << (interrupt_id % 32));
	return 0;
}

int8_t plic_mindgrove_irq_disable(uint32_t interrupt_id)
{
	if (interrupt_id >= PLIC_MAX_INTERRUPT_SRC)
	{
		return -1;
	}
	volatile uint32_t *irq_disable_addr = (uint32_t*)(DT_REG_ADDR(DT_NODELABEL(plic)) + PLIC_ENABLE_OFFSET + ((interrupt_id / 32) * 4));
	*irq_disable_addr &= (~(0x1U << (interrupt_id % 32)));
	return 0;
}

int8_t plic_mindgrove_irq_threshold(uint32_t priority_val)
{
	if (priority_val > PLIC_PRIORITY_7)
	{
		return -1;
	}
	volatile uint32_t *irq_threshold_priority = (uint32_t*)(DT_REG_ADDR(DT_NODELABEL(plic)) + PLIC_THRESHOLD_OFFSET);
	*irq_threshold_priority = priority_val;
	return 0;
}

int8_t plic_mindgrove_irq_priority(uint32_t interrupt_id, uint32_t priority_val)
{
	if (interrupt_id >= PLIC_MAX_INTERRUPT_SRC)
	{
		return -1;
	}
	volatile uint32_t *irq_priority_addr = (uint32_t*)(DT_REG_ADDR(DT_NODELABEL(plic)) + PLIC_PRIORITY_OFFSET + (interrupt_id << PLIC_PRIORITY_SHIFT_PER_INT));
	*irq_priority_addr = priority_val;
	return 0;
}

int8_t plic_mindgrove_irq_pending(uint32_t interrupt_id)
{
	if (interrupt_id >= PLIC_MAX_INTERRUPT_SRC)
	{
		return -1;
	}
	volatile uint32_t *pending_reg = (volatile uint32_t*)(DT_REG_ADDR(DT_NODELABEL(plic)) + PLIC_PENDING_OFFSET + (interrupt_id / 32) * 4);
	volatile uint32_t pending_val = *pending_reg;
	return (pending_val & (1U << (interrupt_id % 32))) ? 1 : 0;
}

int plic_mindgrove_init(const struct device *dev)
{
	// ARG_UNUSED(dev);
	
	*((volatile uint32_t*)(DT_REG_ADDR(DT_NODELABEL(plic)) + PLIC_ENABLE_OFFSET + 0x0)) = 0x0;	// clears the lower 32 interrupt enable bits.
	*((volatile uint32_t*)(DT_REG_ADDR(DT_NODELABEL(plic)) + PLIC_ENABLE_OFFSET + 0x4)) = 0x0;	// clears the upper 32 interrupt enable bits.
	
	plic_mindgrove_irq_threshold(PLIC_PRIORITY_1);	// Set global threshold to PRIORITY_1 (allow all >1 priority)

	// // Connect the top-level PLIC IRQ handler
    // IRQ_CONNECT(RISCV_MACHINE_EXT_IRQ, 0,
    //             plic_mindgrove_irq_handler, NULL, 0);

    // irq_enable(RISCV_MACHINE_EXT_IRQ);
	#ifdef CONFIG_RISCV_DIRECT_IRQ_CONNECT
	IRQ_DIRECT_CONNECT(DT_IRQN(DT_NODELABEL(plic)),
					PLIC_IRQ_PRIO, /* Use the defined constant priority */
					plic_mindgrove_irq_handler, 0);
	#else
	IRQ_CONNECT(DT_IRQN(DT_NODELABEL(plic)),
				PLIC_IRQ_PRIO, /* Use the defined constant priority */
				plic_mindgrove_irq_handler, 0, 0);
	#endif

	// // Enable Global and local (PLIC) interrupts.
    // __asm__ volatile("li      t0, 8\t\n"
    //          "csrrs   zero, mstatus, t0\t\n"
	// 		 "li      t0, 0x800\t\n"
    //          "csrrs   zero, mie, t0\t\n"
    //         );
	return 0;
}

void riscv_plic_irq_enable(uint32_t irq)
{
    plic_mindgrove_irq_enable(irq); 
}

void riscv_plic_set_priority(uint32_t irq, uint32_t priority)
{
    plic_mindgrove_irq_priority(irq, priority);
}

uint32_t riscv_plic_get_irq(void)
{
    return (uint32_t)plic_mindgrove_irq_claim(0);
}

const struct device *riscv_plic_get_dev(void)
{
    return DEVICE_DT_INST_GET(0); 
}

#define PLIC_INIT(n) \
    DEVICE_DT_INST_DEFINE(n, \
        plic_mindgrove_init, \
        NULL, \
        NULL, \
        NULL, \
        PRE_KERNEL_1, \
        CONFIG_INTC_INIT_PRIORITY, \
        NULL);

DT_INST_FOREACH_STATUS_OKAY(PLIC_INIT)