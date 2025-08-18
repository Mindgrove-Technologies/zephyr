/*
 * Copyright (c) 2025 Mindgrove Technologies
 */

#include "coremark.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#define THREAD_STACK_SIZE (8192)

/* Allocate thread stacks using Zephyr macro */
K_THREAD_STACK_ARRAY_DEFINE(thread_stacks, CONFIG_COREMARK_THREADS_NUMBER, THREAD_STACK_SIZE);

/* Thread descriptors */
static struct k_thread thread_descriptors[CONFIG_COREMARK_THREADS_NUMBER];
static int thread_cnt;

/*
 * Predefined seed values are required by CoreMark for given run types.
 */
#if CONFIG_COREMARK_RUN_TYPE_VALIDATION
volatile ee_s32 seed1_volatile = 0x3415;
volatile ee_s32 seed2_volatile = 0x3415;
volatile ee_s32 seed3_volatile = 0x66;
#elif CONFIG_COREMARK_RUN_TYPE_PERFORMANCE
volatile ee_s32 seed1_volatile = 0x0;
volatile ee_s32 seed2_volatile = 0x0;
volatile ee_s32 seed3_volatile = 0x66;
#elif CONFIG_COREMARK_RUN_TYPE_PROFILE
volatile ee_s32 seed1_volatile = 0x8;
volatile ee_s32 seed2_volatile = 0x8;
volatile ee_s32 seed3_volatile = 0x8;
#endif

volatile ee_s32 seed4_volatile = CONFIG_COREMARK_ITERATIONS;
volatile ee_s32 seed5_volatile = 0;

ee_u32 default_num_contexts = CONFIG_COREMARK_THREADS_NUMBER;

BUILD_ASSERT((CONFIG_COREMARK_THREADS_NUMBER >= 1), "Number of threads has to be positive");

// Defined for RISCV - Your original timing implementation
#define NSECS_PER_SEC 100000000
#define EE_TIMER_TICKER_RATE 100
#define CORETIMETYPE clock_t
#define read_csr(reg)                                                   \
({ unsigned long __tmp; \
    __asm__ volatile ("csrr %0, " #reg : "=r"(__tmp)); __tmp; })

#define GETMYTIME(_t) (*_t=read_csr(mcycle))
#define MYTIMEDIFF(fin,ini) ((fin)-(ini))
#define TIMER_RES_DIVIDER 1
#define EE_TICKS_PER_SEC (NSECS_PER_SEC / TIMER_RES_DIVIDER)

/** Define Host specific (POSIX), or target specific global time variables. */
static CORE_TICKS start_time_val;
static CORE_TICKS stop_time_val;

/* Your original timing functions - unchanged */
void start_time(void) {
    GETMYTIME(&start_time_val);
}

void stop_time(void) {
    GETMYTIME(&stop_time_val);
}

CORE_TICKS get_time(void) {
    CORE_TICKS elapsed = (CORE_TICKS)(MYTIMEDIFF(stop_time_val, start_time_val));
    return elapsed;
}

secs_ret time_in_secs(CORE_TICKS ticks) {
    secs_ret retval = ((secs_ret)ticks) / (secs_ret)EE_TICKS_PER_SEC;
    return retval;
}

void portable_init(core_portable *p, int *argc, char *argv[]) {
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    if (sizeof(ee_ptr_int) != sizeof(void *)) {
        ee_printf("ERROR! Please define ee_ptr_int to a type that holds a pointer!\n");
        k_panic();
    }

    if (sizeof(ee_u32) != 4) {
        ee_printf("ERROR! Please define ee_u32 to a 32b unsigned type!\n");
        k_panic();
    }

    p->portable_id = 1;
}

void portable_fini(core_portable *p) {
    p->portable_id = 0;
}

static void coremark_thread(void *id, void *pres, void *p3) {
    ARG_UNUSED(id);
    iterate(pres);
}

/* Nordic's core_start_parallel implementation */
ee_u8 core_start_parallel(core_results *res) {
    __ASSERT(thread_cnt < CONFIG_COREMARK_THREADS_NUMBER, "Reached max number of threads");

    (void)k_thread_create(&thread_descriptors[thread_cnt],
                          thread_stacks[thread_cnt],
                          THREAD_STACK_SIZE,
                          coremark_thread,
                          (void *)thread_cnt,
                          res,
                          NULL,
                          CONFIG_COREMARK_THREADS_PRIORITY, 0, K_NO_WAIT);

    thread_cnt++;

    return 0;
}

/* Nordic's core_stop_parallel implementation */
ee_u8 core_stop_parallel(core_results *res) {
    int ret;

    __ASSERT(thread_cnt > 0, "Can't have negative number of active threads");
    thread_cnt--;

    if (thread_cnt == 0) {
        for (size_t i = 0; i < CONFIG_COREMARK_THREADS_NUMBER; i++) {
            ret = k_thread_join(&thread_descriptors[i], K_MSEC(CONFIG_COREMARK_THREADS_TIMEOUT_MS));
            if (ret == -EAGAIN) {
                ee_printf("Error: Thread join failed due to timeout. Consider increasing CONFIG_COREMARK_THREADS_TIMEOUT_MS\n");
                k_panic();
            } else if (ret) {
                ee_printf("Error: Thread %d join failed error: %d", i, ret);
                k_panic();
            }
        }
    }

    return 0;
}
