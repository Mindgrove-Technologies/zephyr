/*
 * Copyright (c) 2017 Jean-Paul Etienne <fractalclone@gmail.com>
 * Contributors: 2018 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/toolchain.h>

/* exports */
GTEXT(__start)

/* imports */
GTEXT(__initialize)
#if defined(CONFIG_GEN_SW_ISR_TABLE)
GTEXT(_isr_wrapper)
#endif

SECTION_FUNC(vectors, __start)
#if defined(CONFIG_RISCV_GP)
	/* Initialize global pointer */
	.option push
	.option norelax
	la gp, __global_pointer$
	.option pop
#endif

	.option norvc;

#if defined(CONFIG_SOC_RESET_HOOK)
	call soc_reset_hook
#endif

/* ------------------------------------------------------------------ */
/* --- START OF TEMPORARY FORCED MTVEC CONFIGURATION BLOCK (HACK) --- */
/* ------------------------------------------------------------------ */

#if 0 
/* The entire original conditional block is commented out using '#if 0' */

#if defined(CONFIG_RISCV_VECTORED_MODE)
#if defined(CONFIG_RISCV_HAS_CLIC)

    /* Original CLIC vectored mode code block... */
#if defined(CONFIG_GEN_SW_ISR_TABLE)
    la t0, _isr_wrapper
#else
    add t0, zero, zero
#endif
    addi t0, t0, 0x03 /* Enable CLIC vectored mode by setting LSB */
    csrw mtvec, t0

    la t0, _irq_vector_table
    csrw 0x307, t0 /* mtvt */

#else /* !CONFIG_RISCV_HAS_CLIC */

    /* Original CLINT vectored mode code block... */
    la t0, _irq_vector_table /* Load address of interrupt vector table */
    addi t0, t0, 0x01 /* Enable vectored mode by setting LSB */
    csrw mtvec, t0

#endif /* CONFIG_RISCV_HAS_CLIC */

#else /* !CONFIG_RISCV_VECTORED_MODE */

#if defined(CONFIG_RISCV_HAS_CLIC) && !defined(CONFIG_LEGACY_CLIC)

    la t0, _isr_wrapper
    addi t0, t0, 0x03 /* Set mode bits to 3, signifying CLIC. Everything else is reserved. */
    csrw mtvec, t0

#else /* !CONFIG_RISCV_HAS_CLIC || CONFIG_LEGACY_CLIC */

    /* Original CLINT direct mode code block... */
    la t0, _isr_wrapper
    csrw mtvec, t0

#endif /* CONFIG_RISCV_HAS_CLIC&& !CONFIG_LEGACY_CLIC */

#endif /* CONFIG_RISCV_VECTORED_MODE */
#endif 
/* --- END OF ORIGINAL CONDITIONAL BLOCK --- */


    /* * FORCED IMPLEMENTATION: Standard PLIC/CLINT Vectored Mode.
     * This forces mtvec to point to the jump table (_irq_vector_table) 
     * and sets the LSB to 1 (Vectored Mode).
     * If the interrupt works now, the problem was Kconfig mismatch.
     */
    la t0, _irq_vector_table /* Load address of interrupt vector table from linker script */
    ori t0, t0, 0x01         /* Set Bit 0 (Mode = Vectored) */
    csrw mtvec, t0           /* Write the final address (base + mode) to CSR */


/* ------------------------------------------------------------------ */
/* --- END OF TEMPORARY FORCED MTVEC CONFIGURATION BLOCK --- */
/* ------------------------------------------------------------------ */


     /* Jump to __reset */
     tail __reset