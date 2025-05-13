#include <soc.h>

bool arch_cpu_active(int cpu_num)
{
    return (cpu_num < CONFIG_MP_MAX_NUM_CPUS);
}

struct _cpu *arch_curr_cpu(void)
{
    return &_kernel.cpus[arch_proc_id()];
}