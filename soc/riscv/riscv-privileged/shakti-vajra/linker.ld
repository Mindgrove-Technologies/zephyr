/* For Shakti Vajra SOC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Linker script for Shakti Vajra processor
 */

#include <zephyr/arch/riscv/common/linker.ld>

SECTION_PROLOGUE(.exception.vector_table,,)
{
    /* Aligning to 256 bytes is a common requirement for RISC-V mtvec */
    . = ALIGN(256); 
    
    __irq_vector_table = .;
    
    /* CRITICAL: This connects the generated table (containing the jump opcodes) 
     * to the address the CPU fetches from.
     */
    KEEP(*(.irq_vector_table)) 
    
    __irq_vector_table_end = .;
} > ROM /* Or > FLASH, depending on where your code lives */
