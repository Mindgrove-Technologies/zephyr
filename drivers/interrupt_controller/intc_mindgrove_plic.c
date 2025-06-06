/*
   Global interrupt data maintenance structure
*/

#define DT_DRV_COMPAT mindgrove_plic
#define PLIC_BASE_ADDRESS DT_INST_PROP(0, base)

//--------------------------------

#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <soc.h>

#include <zephyr/sw_isr_table.h>
#include <zephyr/drivers/interrupt_controller/riscv_plic.h>
#include <zephyr/irq.h>

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

typedef struct plic_mindgrove_regs_t
{
    uint32_t priority_thershold;
    uint32_t claim_register;
    uint32_t interrupt_complete;

}plic_regs_t;

static int track_irq_num;
volatile int key=0;

/** 
 * @brief handle machine mode plic interrupts
 * @details find the int id that caused of interrupt, 
 *	    process it and complete the interrupt.
 * @param uintptr_t int_id
 * @param uintptr_t epc
 */

/* Fucntion that calls PLIC_IRQ_HANDLER */
void arch_system_halt(unsigned int reason)
{
	struct _isr_table_entry *plic_int_t;
	plic_int_t = (struct _isr_table_entry *)(&_sw_isr_table[11]);
    plic_int_t->isr(plic_int_t->arg);
}


void plic_irq_handler( const void *arg)
{
	uint32_t  interrupt_id;

    volatile plic_regs_t *regs = (volatile plic_regs_t *) PLIC_REG_OFFSET;

    struct _isr_table_entry *int_handle_t;

    interrupt_id = regs->claim_register;

    track_irq_num = interrupt_id;

    if (track_irq_num == 0U || track_irq_num >= CONFIG_NUM_IRQS)
	    z_irq_spurious(NULL);

	/*call relevant interrupt service routine*/
    int_handle_t = (struct _isr_table_entry *)&_sw_isr_table[interrupt_id+31];
    int_handle_t->isr(int_handle_t->arg);

    regs->claim_register = track_irq_num;
}

/** @fn uint32_t isr_default(uint32_t interrupt_id) 
 * @brief default interrupt service routine
 * @details Default isr. Use it when you dont know what to do with interrupts
 * @param uint32_t interrupt_id
 * @return uint32_t
 */
static inline void isr_default(uint32_t interrupt_id)
{
    printk("Entered isr_default\n");
}

int riscv_plic_get_irq(void)
{
	return track_irq_num;
}

/** @fn void interrupt_enable(uint32_t interrupt_id)
 * @brief enable the interrupt
 * @details A single bit that enables an interrupt. The bit position corresponds to the interrupt id
 * @param uint32_t interrupt_id
 */
void plic_irq_enable(uint32_t interrupt_id)
{
	uint32_t *interrupt_enable_addr;
	uint32_t current_value = 0x00, new_value;

	interrupt_enable_addr = (uint32_t *) (PLIC_BASE_ADDRESS +
			PLIC_ENABLE_OFFSET +
			(((interrupt_id / 32)-1)*sizeof(uint32_t)));

	current_value = *interrupt_enable_addr;

	/*set the bit corresponding to the interrupt src*/
	new_value = current_value | (0x1 << ((interrupt_id % 32)+1)); //Changed thissssss

	key = irq_lock();
	*((uint32_t*)interrupt_enable_addr) = new_value;
	irq_unlock(key);
}

void interrupt_disable(uint32_t id)
{
  	uint32_t *interrupt_disable_addr = 0U;

	interrupt_disable_addr = (uint32_t *) (PLIC_BASE_ADDRESS +
					      PLIC_ENABLE_OFFSET +
					      (id / 32)*sizeof(uint32_t));

	key = irq_lock();
	*interrupt_disable_addr &= (~(0x1 << (id % 32)));
	irq_unlock(key);
	
}

void interrupt_priority(uint32_t id)
{
	uint32_t *interrupt_priority_address = 0U;

	interrupt_priority_address = (uint32_t *) (PLIC_BASE_ADDRESS +
					   PLIC_PRIORITY_OFFSET +
					   (id <<
					    PLIC_PRIORITY_SHIFT_PER_INT));

	*interrupt_priority_address = 0x02;
}


/** @fn void interrupt_disable(uint32_t interrupt_id) 
 * @brief disable an interrupt
 * @details A single bit that enables an interrupt.
 *          The bit position corresponds to the interrupt id
 * @param uint32_t interrupt_id
 */
void plic_mindgrove_irq_disable(uint32_t interrupt_id)
{
	uint32_t *interrupt_disable_addr = 0;
	uint32_t current_value = 0x00, new_value;

	interrupt_disable_addr = (uint32_t *) (PLIC_BASE_ADDRESS +
					      PLIC_ENABLE_OFFSET +
					      (interrupt_id / 32)*sizeof(uint32_t));

	/*unset the bit corresponding to the interrupt src*/
	new_value = current_value & (~(0x1 << ((interrupt_id % 32) + 1)));

	key = irq_lock();
	*interrupt_disable_addr = new_value;
	irq_unlock(key);
}

/** @fn void set_interrupt_threshold(uint32_t priority_value)
 * @brief set priority threshold for all interrupts
 * @details set a threshold on interrrupt priority. Any interruptthat has lesser priority than the threshold is ignored.
 * @param uint32_t priority_value
 */
void plic_mindgrove_set_irq_threshold(uint32_t priority_value)
{
	uint32_t *interrupt_threshold_priority = NULL;

	interrupt_threshold_priority = (uint32_t *) (PLIC_BASE_ADDRESS +
						     PLIC_THRESHOLD_OFFSET);

	*interrupt_threshold_priority = priority_value;
}

/** @fn void set_interrupt_priority(uint32_t priority_value, uint32_t int_id)
 * @brief set priority for an interrupt source
 * @details set priority for each interrupt. This is a 4 byte field.
 * @param uint32_t priority_value
 * @param uint32_t int_id
 */
void plic_mindgrove_set_priority(uint32_t priority_value, uint32_t int_id)
{
	uint32_t * interrupt_priority_address;

	/*
	   base address + priority offset + 4*interruptId
	 */

	interrupt_priority_address = (uint32_t *) (PLIC_BASE_ADDRESS +
						   PLIC_PRIORITY_OFFSET +
						   (int_id <<
						    PLIC_PRIORITY_SHIFT_PER_INT));

	*interrupt_priority_address = priority_value;
}

/** @fn void plic_init
 * @brief intitializes the plic module
 * @details Intitializes the plic registers to default values.
 *          Sets up the plic meta data table. Assigns the plic
 *          handler to mcause_interrupt_table.,By default interrupts are disabled.
 */
void plic_mindgrove_init(const struct device *dev)
{
    volatile uint32_t *interrupt_disable_addr = (uint32_t *) (PLIC_BASE_ADDRESS + PLIC_ENABLE_OFFSET);
    volatile uint32_t *interrupt_threshold_priority = (uint32_t *) (PLIC_BASE_ADDRESS + PLIC_THRESHOLD_OFFSET);    

    volatile plic_regs_t *regs = (volatile plic_regs_t *)PLIC_REG_OFFSET;

	for (int id = 1; id < PLIC_MAX_INTERRUPT_SRC; id++)
	{
		interrupt_disable(id);
		interrupt_priority(id);
	}

    regs->priority_thershold = 0U;

    	/* Setup IRQ handler for PLIC driver */
	IRQ_CONNECT(RISCV_MACHINE_EXT_IRQ,
		    0,
		    plic_irq_handler,
		    NULL,
		    0);

	/* Enable IRQ for PLIC driver */
	irq_enable(RISCV_MACHINE_EXT_IRQ);

	// Enable Global (PLIC) interrupts.
    __asm__ volatile("li      t0, 8\t\n" 
				"csrrs   zero, mstatus, t0\t\n"
				);

    // Enable Local (PLIC) interrupts.
    __asm__ volatile("li      t0, 0x800\t\n"
                 "csrrs   zero, mie, t0\t\n"
                );
}

uint32_t plic_get_irq_id(uint32_t int_num)
{
	return (int_num + 32);
}

DEVICE_DT_INST_DEFINE(0, plic_mindgrove_init, NULL, NULL, NULL,
		      PRE_KERNEL_1, CONFIG_INTC_INIT_PRIORITY, NULL);