/*
 * Copyright (c) 2025 Mindgrove Technologies
 */

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "coremark_zephyr.h"

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

#define COOP_PRIO -1

static K_SEM_DEFINE(start_coremark, 0, 1);
static atomic_t coremark_in_progress;

static void main_thread_priority_cooperative_set(void)
{
	BUILD_ASSERT(CONFIG_MAIN_THREAD_PRIORITY >= 0);
	k_thread_priority_set(k_current_get(), COOP_PRIO);
}

int main(void)
{
	/* Drivers need to be run from a non-blocking thread.
	 * We need preemptive priority during init.
	 * Later we prefer cooperative priority to ensure no interference with the benchmark.
	 */
	main_thread_priority_cooperative_set();

	printk("CoreMark sample for %s\n\r", CONFIG_BOARD_TARGET);

    (void)atomic_set(&coremark_in_progress, true);
	k_sem_give(&start_coremark);
    
	while (true) {
		k_sem_take(&start_coremark, K_FOREVER);

		printk("CoreMark started!\n\r");
		printk("Threads: %d\n\r", CONFIG_COREMARK_THREADS_NUMBER);  
		printk("Iterations: %d\n\r", CONFIG_COREMARK_ITERATIONS);

		coremark_run();
        
        printk("CoreMark finished!\n\r");

		(void)atomic_set(&coremark_in_progress, false);
	};

	return 0;
}