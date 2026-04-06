#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/sys/printk.h>

const struct device *timer0 = DEVICE_DT_GET(DT_NODELABEL(timer0));

static volatile bool alarm_fired = false;

/* Alarm callback */
static void alarm_callback(const struct device *dev, uint8_t chan_id,
                          uint32_t ticks, void *user_data)
{
    alarm_fired = true;
    printk("  ✓ Alarm fired at %u ticks\n", ticks);
}

int main(void)
{
    uint32_t v1, v2, freq;
    int ret;
    
    printk("\n=== GPTIMER ESSENTIAL TESTS ===\n\n");
    
    /* Device ready check */
    if (!device_is_ready(timer0)) {
        printk("ERROR: Timer not ready\n");
        return -1;
    }
    printk("✓ Timer device ready\n");
    
    /* Get frequency */
    freq = counter_get_frequency(timer0);
    printk("✓ Frequency: %u Hz\n", freq);
    
    /* TEST 1: Set reasonable top value for testing */
    printk("\n--- TEST 1: Set Top Value ---\n");
    struct counter_top_cfg top_cfg = {
        .ticks = 60000,  /* ~0.1s at 1MHz */
        .callback = NULL,
        .flags = 0
    };
    ret = counter_set_top_value(timer0, &top_cfg);
    printk("Set top value: %s\n", ret == 0 ? "✓ OK" : "✗ FAIL");
    
    /* TEST 2: Basic counting */
    printk("\n--- TEST 2: Basic Counting ---\n");
    counter_start(timer0);
    counter_get_value(timer0, &v1);
    k_sleep(K_MSEC(100));
    counter_get_value(timer0, &v2);
    counter_stop(timer0);
    
    printk("v1=%u, v2=%u, delta=%d\n", v1, v2, (int32_t)(v2-v1));
    if (v2 > v1) {
        printk("✓ Counter incrementing correctly\n");
    } else {
        printk("✗ Counter NOT incrementing!\n");
    }
    
    /* TEST 3: Stop behavior */
    printk("\n--- TEST 3: Stop Freezes Counter ---\n");
    counter_start(timer0);
    k_sleep(K_MSEC(50));
    counter_stop(timer0);
    counter_get_value(timer0, &v1);
    k_sleep(K_MSEC(50));
    counter_get_value(timer0, &v2);
    
    printk("Stopped: v1=%u, v2=%u\n", v1, v2);
    if (v1 == v2) {
        printk("✓ Counter properly frozen\n");
    } else {
        printk("✗ Counter still running after stop!\n");
    }
    
    /* TEST 4: Frequency measurement */
    printk("\n--- TEST 4: Frequency Measurement (1 second) ---\n");
    struct counter_top_cfg large_top = {
        .ticks = 0xFFFFFFFF,
        .callback = NULL,
        .flags = 0
    };
    counter_set_top_value(timer0, &large_top);
    
    counter_start(timer0);
    counter_get_value(timer0, &v1);
    k_sleep(K_SECONDS(1));
    counter_get_value(timer0, &v2);
    counter_stop(timer0);
    
    uint32_t measured = v2 - v1;
    int32_t error = (int32_t)(measured - freq);
    float error_pct = (float)error * 100.0f / (float)freq;
    
    printk("Expected: %u Hz\n", freq);
    printk("Measured: %u Hz\n", measured);
    printk("Error: %d (%.1f%%)\n", error, error_pct);
    
    if (error < (int32_t)(freq / 10)) {
        printk("✓ Frequency within 10%% tolerance\n");
    } else {
        printk("✗ Frequency out of tolerance!\n");
    }
    
    /* TEST 5: Wraparound */
    printk("\n--- TEST 5: Wraparound Test ---\n");
    struct counter_top_cfg small_top = {
        .ticks = 5000,
        .callback = NULL,
        .flags = 0
    };
    counter_set_top_value(timer0, &small_top);
    counter_start(timer0);
    
    counter_get_value(timer0, &v1);
    bool wrapped = false;
    
    for (int i = 0; i < 35 && !wrapped; i++) {
        k_sleep(K_MSEC(10));
        counter_get_value(timer0, &v2);
        if (v2 < v1) {
            wrapped = true;
            printk("Wraparound: %u -> %u\n", v1, v2);
        }
        v1 = v2;
    }
    counter_stop(timer0);
    
    if (wrapped) {
        printk("✓ Counter wraps correctly\n");
    } else {
        printk("✗ Counter did not wrap!\n");
    }
    
    // /* TEST 6: Alarm/Interrupt */
    // printk("\n--- TEST 6: Alarm (Interrupt) Test ---\n");
    // alarm_fired = false;
    
    // struct counter_top_cfg alarm_top = {
    //     .ticks = 20000,
    //     .callback = NULL,
    //     .flags = 0
    // };
    // counter_set_top_value(timer0, &alarm_top);
    
    // struct counter_alarm_cfg alarm_cfg = {
    //     .callback = alarm_callback,
    //     .ticks = 0,
    //     .user_data = NULL,
    //     .flags = 0
    // };
    
    // ret = counter_set_alarm(timer0, 0, &alarm_cfg);
    // printk("Set alarm: %s\n", ret == 0 ? "✓ OK" : "✗ FAIL");
    
    // counter_start(timer0);
    // printk("Waiting for alarm (max 1s)...\n");
    
    // for (int i = 0; i < 20 && !alarm_fired; i++) {
    //     k_sleep(K_MSEC(50));
    // }
    // counter_stop(timer0);
    
    // if (alarm_fired) {
    //     printk("✓ Alarm callback executed\n");
    // } else {
    //     printk("✗ Alarm did NOT fire!\n");
    // }
    
    // /* TEST 7: Cancel alarm */
    // printk("\n--- TEST 7: Cancel Alarm ---\n");
    // alarm_fired = false;
    
    // counter_set_alarm(timer0, 0, &alarm_cfg);
    // ret = counter_cancel_alarm(timer0, 0);
    // printk("Cancel alarm: %s\n", ret == 0 ? "✓ OK" : "✗ FAIL");
    
    // counter_start(timer0);
    // k_sleep(K_MSEC(200));
    // counter_stop(timer0);
    
    // if (!alarm_fired) {
    //     printk("✓ Alarm properly cancelled\n");
    // } else {
    //     printk("✗ Alarm fired after cancel!\n");
    // }
    
    printk("\n=== TESTS COMPLETE ===\n\n");
    
    while (1) {
        k_sleep(K_FOREVER);
    }
    
    return 0;
}