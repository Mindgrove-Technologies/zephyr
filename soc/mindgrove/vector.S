/* In your reset.S or vector.S */

/* Setup Machine Trap Vector (mtvec) */
.align 8 
_init_mtvec:
    /* Load address defined in linker script: __irq_vector_table */
    la t0, __irq_vector_table   
    
    /* Set Bit 0 (Vectored Mode) to 1. 
     * If the mode is 0 (Direct/Single handler), the address must be 4-byte aligned.
     * If the mode is 1 (Vectored), the address must be 256-byte aligned.
     * Your linker script aligned it, so we just set the mode bit.
     */
    ori t0, t0, 1               
    
    csrw mtvec, t0              /* Write the final address (base + mode) to CSR */