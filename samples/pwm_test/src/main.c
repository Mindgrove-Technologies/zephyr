#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pwm.h>
#include <stdio.h>

#define PWM_NODE DT_NODELABEL(pwm0)

#define PWM_PERIOD_CYCLES 1000
#define NUM_CHANNELS 8 
#define CHANNEL_NO 0

static inline void delay_loop(volatile int count)
{
    while (count--)
    {
        __asm__ volatile("nop");
    }
}

int main(void)
{
    const struct device *pwm = DEVICE_DT_GET(PWM_NODE);

    if (!device_is_ready(pwm))
    {
        printf("PWM device not ready\n");
        return 0;
    }

    printf("Starting Multi-Channel PWM Test\n");

    while (1)
    {

        /* ---------------- PHASE 1 ---------------- */
        /* Increasing duty cycle pattern */
        for (int ch = CHANNEL_NO; ch < NUM_CHANNELS; ch++)
        {

            uint32_t pulse =
                (PWM_PERIOD_CYCLES * (ch + 1)) / (NUM_CHANNELS + 1);

            pwm_set_cycles(pwm,
                           ch,
                           PWM_PERIOD_CYCLES,
                           pulse,
                           0);
        }

        printf("P1 -> Increasing duty cycle\n");
        delay_loop(50000);

        /* ---------------- PHASE 2 ---------------- */
        /* All channels at 50% */
        for (int ch = CHANNEL_NO; ch < NUM_CHANNELS; ch++)
        {

            pwm_set_cycles(pwm,
                           ch,
                           PWM_PERIOD_CYCLES,
                           PWM_PERIOD_CYCLES / 2,
                           0);
        }

        printf("P2 -> All channels 50%%\n");
        delay_loop(50000);

        /* ---------------- PHASE 3 ---------------- */
        /* Reverse decreasing duty cycle */
        for (int ch = CHANNEL_NO; ch < NUM_CHANNELS; ch++)
        {

            uint32_t pulse =
                (PWM_PERIOD_CYCLES * (NUM_CHANNELS - ch)) /
                (NUM_CHANNELS + 1);

            pwm_set_cycles(pwm,
                           ch,
                           PWM_PERIOD_CYCLES,
                           pulse,
                           0);
        }

        printf("P3 -> Reverse duty cycle\n");
        delay_loop(50000);

        /* ---------------- PHASE 4 ---------------- */
        /* Stop all PWM channels */
        for (int ch = CHANNEL_NO; ch < NUM_CHANNELS; ch++)
        {

            pwm_set_cycles(pwm,
                           ch,
                           PWM_PERIOD_CYCLES,
                           0,
                           0);
        }

        printf("P4 -> All channels stopped\n");
        delay_loop(50000);
    }
}