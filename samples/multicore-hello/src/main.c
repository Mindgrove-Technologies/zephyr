#include <zephyr/arch/cpu.h>

// #define STACK_SIZE 2048
// #define NUM_CORES 4

// /* Thread prototypes */
// void thread0_entry(void *, void *, void *);
// void thread1_entry(void *, void *, void *);
// void thread2_entry(void *, void *, void *);
// void thread3_entry(void *, void *, void *);

// #define STACK_SIZE 4096

// K_THREAD_DEFINE(thread0, STACK_SIZE, thread0_entry, 5, NULL, NULL, 0, 0, 0);
// K_THREAD_DEFINE(thread1, STACK_SIZE, thread1_entry, 5, NULL, NULL, 0, 0, 0);
// K_THREAD_DEFINE(thread2, STACK_SIZE, thread2_entry, 5, NULL, NULL, 0, 0, 0);
// K_THREAD_DEFINE(thread3, STACK_SIZE, thread3_entry, 5, NULL, NULL, 0, 0, 0);

// void thread0_entry(void *p1, void *p2, void *p3) 
// {
//     printk("Core %d: Hello from thread %p\n", 
//            arch_curr_cpu()->id, k_current_get());
// }

// void thread1_entry(void *p1, void *p2, void *p3) 
// {
//     printk("Core %d: Hello from thread %p\n", 
//            arch_curr_cpu()->id, k_current_get());
// }

// void thread2_entry(void *p1, void *p2, void *p3) 
// {
//     printk("Core %d: Hello from thread %p\n", 
//            arch_curr_cpu()->id, k_current_get());
// }

// void thread3_entry(void *p1, void *p2, void *p3) 
// {
//     printk("Core %d: Hello from thread %p\n", 
//            arch_curr_cpu()->id, k_current_get());
// }

int main(void)
{
    printk("Main running on CPU %d\n", arch_curr_cpu()->id);
    return 0;
}