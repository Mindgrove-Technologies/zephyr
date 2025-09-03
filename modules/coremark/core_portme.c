/*
 * Copyright (c) 2025 Mindgrove Technologies
 */

#include "coremark.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#define THREAD_STACK_SIZE   (8192)  // Increased for CoreMark requirements

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

#ifndef CONFIG_COREMARK_THREADS_PRIORITY
#define CONFIG_COREMARK_THREADS_PRIORITY 5
#endif

#ifndef CONFIG_COREMARK_THREADS_TIMEOUT_MS
#define CONFIG_COREMARK_THREADS_TIMEOUT_MS 30000  // 30 second timeout
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
    CORE_TICKS elapsed;
    if (stop_time_val >= start_time_val) {
        elapsed = stop_time_val - start_time_val;
    } else {
        // Handle counter overflow
        elapsed = (0xFFFFFFFFFFFFFFFF - start_time_val) + stop_time_val + 1;
    }
    return elapsed;
}

secs_ret time_in_secs(CORE_TICKS ticks) {
    return ((secs_ret)ticks) / (secs_ret)EE_TICKS_PER_SEC;
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

#if (MULTITHREAD > 1)
#if defined(CONFIG_COREMARK_PTHREADS) // POSIX Threads

// POSIX spinlock for thread synchronization
static pthread_spinlock_t thread_spinlock;
static bool spinlock_initialized = false;

static void init_pthread_spinlock(void) {
    if (!spinlock_initialized) {
        if (pthread_spin_init(&thread_spinlock, PTHREAD_PROCESS_PRIVATE) != 0) {
            ee_printf("ERROR: Failed to initialize pthread spinlock\n");
            k_panic();
        }
        spinlock_initialized = true;
    }
}

ee_u8 core_start_parallel(core_results *res)
{
    init_pthread_spinlock();
    
    pthread_spin_lock(&thread_spinlock);
    int ret = pthread_create(&(res->port.thread), NULL, (void*(*)(void*))iterate, (void *)res);
    pthread_spin_unlock(&thread_spinlock);
    
    return (ee_u8)ret;
}

ee_u8 core_stop_parallel(core_results *res)
{
    void *retval;
    
    pthread_spin_lock(&thread_spinlock);
    int ret = pthread_join(res->port.thread, &retval);
    pthread_spin_unlock(&thread_spinlock);
    
    return (ee_u8)ret;
}

#elif defined(CONFIG_COREMARK_ZTHREADS) // Zephyr KThreads

#define _COREMARK_THREAD_STACK_ARRAY_ITEM(n, _) CONCAT(CONCAT(coremark_thread_, n), _stack)

#define _COREMARK_THREAD_STACK_DEFINE(n, _)							\
	K_THREAD_STACK_DEFINE(_COREMARK_THREAD_STACK_ARRAY_ITEM(n, _), THREAD_STACK_SIZE)

/**
 *  @brief Statically define thread stack structure array.
 *
 *  Helper macro to statically define thread stack structure array.
 *
 *  @param _name	 Name of stack structure array.
 *  @param _instance_num Number of elements in instance array.
 */
#define COREMARK_THREAD_STACK_INSTANCE_DEFINE(_name, _instance_num)		 \
	LISTIFY(_instance_num, _COREMARK_THREAD_STACK_DEFINE, (;));		 \
	static k_thread_stack_t *_name[] = {					 \
		LISTIFY(_instance_num, _COREMARK_THREAD_STACK_ARRAY_ITEM, (,))	 \
	}

COREMARK_THREAD_STACK_INSTANCE_DEFINE(thread_stacks, CONFIG_COREMARK_THREADS_NUMBER);

static struct k_thread thread_descriptors[CONFIG_COREMARK_THREADS_NUMBER];
static volatile int thread_cnt = 0;

// Zephyr spinlock for thread synchronization
static struct k_spinlock thread_spinlock;

static void coremark_thread(void *id, void *pres, void *p3) {
    ARG_UNUSED(id);
    ARG_UNUSED(p3);
    
    core_results *res = (core_results *)pres;
    if (res != NULL) {
        iterate(res);
    }
}

ee_u8 core_start_parallel(core_results *res)
{
    k_spinlock_key_t key = k_spin_lock(&thread_spinlock);
    
    if (thread_cnt >= CONFIG_COREMARK_THREADS_NUMBER) {
        ee_printf("ERROR: Reached max number of threads (%d)\n", CONFIG_COREMARK_THREADS_NUMBER);
        k_spin_unlock(&thread_spinlock, key);
        return 1;
    }

    k_tid_t tid = k_thread_create(&thread_descriptors[thread_cnt],
                                  thread_stacks[thread_cnt],
                                  THREAD_STACK_SIZE,
                                  coremark_thread,
                                  (void *)(intptr_t)thread_cnt,
                                  res,
                                  NULL,
                                  CONFIG_COREMARK_THREADS_PRIORITY, 
                                  0, 
                                  K_NO_WAIT);

    if (tid == NULL) {
        ee_printf("ERROR: Failed to create thread %d\n", thread_cnt);
        k_spin_unlock(&thread_spinlock, key);
        return 1;
    }

    // Store the thread ID in the results structure for this specific thread
    res->port.thread_id = tid;
    thread_cnt++;
    
    k_spin_unlock(&thread_spinlock, key);
    return 0;
}

ee_u8 core_stop_parallel(core_results *res)
{
    int ret = 0;
    k_spinlock_key_t key = k_spin_lock(&thread_spinlock);

    if (thread_cnt <= 0) {
        ee_printf("ERROR: Can't have negative or zero number of active threads\n");
        k_spin_unlock(&thread_spinlock, key);
        return 1;
    }

    thread_cnt--;
    k_tid_t thread_to_join = res->port.thread_id;
    
    k_spin_unlock(&thread_spinlock, key);

    // Join the specific thread for this results structure (outside spinlock to avoid deadlock)
    if (thread_to_join != NULL) {
        ret = k_thread_join(thread_to_join, K_MSEC(CONFIG_COREMARK_THREADS_TIMEOUT_MS));
        if (ret == -EAGAIN) {
            ee_printf("ERROR: Thread join timed out after %d ms. "
                     "Consider increasing CONFIG_COREMARK_THREADS_TIMEOUT_MS\n",
                     CONFIG_COREMARK_THREADS_TIMEOUT_MS);
            return 1;
        } else if (ret != 0) {
            ee_printf("ERROR: Thread join failed with error: %d\n", ret);
            return 1;
        }
    }
    
    return 0;
}

#else /* Single-threaded implementation */

ee_u8 core_start_parallel(core_results *res)
{
    /* Single-threaded mode - just call iterate directly */
    iterate(res);
    return 0;
}

ee_u8 core_stop_parallel(core_results *res)
{
    ARG_UNUSED(res);
    /* Nothing to do in single-threaded mode */
    return 0;
}

#endif /* Threading implementation selection */
#endif // End Multithread Configurations