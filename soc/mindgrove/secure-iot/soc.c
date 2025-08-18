// soc.c

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <soc.h>

static int soc_init(void)
{
    // This is where you configure the SoC's clock tree,
    // power settings, and other very early init steps.
    // Example:
    // 1. Set flash wait states.
    // 2. Configure and enable the main PLL (Phase-Locked Loop).
    // 3. Switch the system clock to the PLL.
    // 4. Enable clocks for essential peripherals like GPIO/UART.

    // This code is highly specific to your chip and will be
    // based on your vendor's reference manual and HAL.

    return 0;
}

// SYS_INIT registers soc_init() to be called at the PRE_KERNEL_1 priority.
// This ensures it runs before any device drivers are initialized.
SYS_INIT(soc_init, PRE_KERNEL_1, 0);