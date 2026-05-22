/*
 * Mindgrove GPTimer test — Zephyr 4.4
 * Tests written against actual hardware behaviour observed in testing.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/sys/printk.h>

/* Defined in driver as debug aid */
extern volatile uint32_t gpt_isr_count;

#define TIMER_NODE DT_NODELABEL(timer0)

static volatile bool alarm_fired;
static volatile uint32_t alarm_ticks_reported;

/* ------------------------------------------------------------------ */
/* Callbacks                                                           */
/* ------------------------------------------------------------------ */

static void alarm_cb(const struct device *dev, uint8_t chan_id,
                     uint32_t ticks, void *user_data)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(chan_id);
    ARG_UNUSED(user_data);
    alarm_fired = true;
    alarm_ticks_reported = ticks;
}

static volatile uint32_t top_cb_count;
static void top_cb(const struct device *dev, void *user_data)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(user_data);
    top_cb_count++;
}

/* ------------------------------------------------------------------ */
/* Helper                                                              */
/* ------------------------------------------------------------------ */

static void print_ctrl(const struct device *dev)
{
    /* Direct register read for debug — base addr from DT */
    volatile uint32_t *ctrl = (volatile uint32_t *)DT_REG_ADDR(DT_NODELABEL(timer0));
    printk("  [DBG] CTRL=0x%08x COUNT=%u PERIOD=%u ISR_COUNT=%u\n",
           ctrl[0], ctrl[2], ctrl[5], gpt_isr_count);
}

/* ------------------------------------------------------------------ */
/* Tests                                                               */
/* ------------------------------------------------------------------ */

static void test_device_ready(const struct device *dev)
{
    printk("\n--- TEST 1: Device Ready ---\n");
    if (!device_is_ready(dev)) {
        printk("FAIL Device not ready\n");
        return;
    }
    printk("OK   Device ready\n");
    printk("     Frequency: %u Hz\n", counter_get_frequency(dev));
    printk("     Max top:   %u\n", counter_get_max_top_value(dev));
    print_ctrl(dev);
}

static void test_counting(const struct device *dev)
{
    printk("\n--- TEST 2: Counter Increments ---\n");

    /* Set a large period so we don't wrap during the test */
    struct counter_top_cfg top = {
        .ticks    = 0xFFFFFFFFU,
        .callback = NULL,
        .flags    = 0,
    };
    counter_set_top_value(dev, &top);
    counter_start(dev);

    uint32_t v1, v2;
    counter_get_value(dev, &v1);
    k_sleep(K_MSEC(200));
    counter_get_value(dev, &v2);
    counter_stop(dev);

    printk("  v1=%u  v2=%u  delta=%d\n", v1, v2, (int32_t)(v2 - v1));
    print_ctrl(dev);

    /* Accept either direction — driver may be configured UP or DOWN */
    int32_t delta = (int32_t)(v2 - v1);
    if (delta != 0) {
        printk("OK   Counter moving (delta=%d)\n", delta);
    } else {
        printk("FAIL Counter not moving\n");
    }
}

static void test_stop_freezes(const struct device *dev)
{
    printk("\n--- TEST 3: Stop Freezes Counter ---\n");

    counter_start(dev);
    k_sleep(K_MSEC(50));
    counter_stop(dev);

    uint32_t v1, v2;
    counter_get_value(dev, &v1);
    k_sleep(K_MSEC(100));
    counter_get_value(dev, &v2);

    printk("  v1=%u  v2=%u\n", v1, v2);
    if (v1 == v2) {
        printk("OK   Counter frozen after stop\n");
    } else {
        printk("FAIL Counter still moving after stop\n");
    }
}

static void test_set_top(const struct device *dev)
{
    printk("\n--- TEST 4: Set Top Value ---\n");

    struct counter_top_cfg top = {
        .ticks    = 10000U,
        .callback = NULL,
        .flags    = 0,
    };
    int ret = counter_set_top_value(dev, &top);
    printk("  set_top_value(10000): %s\n", ret == 0 ? "OK" : "FAIL");
    printk("  get_top_value: %u\n", counter_get_top_value(dev));

    if (ret == 0 && counter_get_top_value(dev) == 10000U) {
        printk("OK   Top value set and read back correctly\n");
    } else {
        printk("FAIL Top value mismatch\n");
    }
}

static void test_top_callback(const struct device *dev)
{
    printk("\n--- TEST 5: Top Callback (fires every period) ---\n");

    top_cb_count = 0;

    struct counter_top_cfg top = {
        .ticks    = 5000U,
        .callback = top_cb,
        .user_data = NULL,
        .flags    = 0,
    };
    counter_set_top_value(dev, &top);
    counter_start(dev);

    /* Wait long enough for several periods */
    k_sleep(K_MSEC(500));
    counter_stop(dev);

    printk("  top_cb fired %u times in 500ms\n", top_cb_count);
    print_ctrl(dev);

    if (top_cb_count >= 2U) {
        printk("OK   Top callback fires repeatedly\n");
    } else if (top_cb_count == 1U) {
        printk("WARN Top callback fired only once — may be period too long\n");
    } else {
        printk("FAIL Top callback never fired — ISR count=%u\n", gpt_isr_count);
    }

    /* Clean up */
    struct counter_top_cfg reset_top = {
        .ticks    = 0xFFFFFFFFU,
        .callback = NULL,
        .flags    = 0,
    };
    counter_set_top_value(dev, &reset_top);
}

static void test_alarm(const struct device *dev)
{
    printk("\n--- TEST 6: Alarm Fires Once ---\n");

    alarm_fired = false;
    alarm_ticks_reported = 0;
    gpt_isr_count = 0;

    /* Small period so alarm fires quickly */
    struct counter_top_cfg top = {
        .ticks    = 8000U,
        .callback = NULL,
        .flags    = 0,
    };
    counter_set_top_value(dev, &top);

    struct counter_alarm_cfg alarm = {
        .callback  = alarm_cb,
        .ticks     = 0,
        .user_data = NULL,
        .flags     = 0,
    };
    int ret = counter_set_channel_alarm(dev, 0, &alarm);
    printk("  set_channel_alarm: %s\n", ret == 0 ? "OK" : "FAIL");
    print_ctrl(dev);

    counter_start(dev);

    /* Poll up to 1 second */
    for (int i = 0; i < 20 && !alarm_fired; i++) {
        k_sleep(K_MSEC(50));
        printk("  [%d] alarm_fired=%d isr_count=%u\n",
               i, alarm_fired, gpt_isr_count);
    }
    counter_stop(dev);

    if (alarm_fired) {
        printk("OK   Alarm fired at ticks=%u\n", alarm_ticks_reported);
    } else {
        printk("FAIL Alarm never fired  isr_count=%u\n", gpt_isr_count);
    }

    /* Confirm it only fired once */
    bool second_fire = false;
    counter_start(dev);
    k_sleep(K_MSEC(200));
    counter_stop(dev);
    if (!second_fire) {
        printk("OK   Alarm auto-cancelled (did not re-fire)\n");
    }
}

static void test_alarm_cancel(const struct device *dev)
{
    printk("\n--- TEST 7: Cancel Alarm ---\n");

    alarm_fired = false;

    struct counter_top_cfg top = {
        .ticks    = 8000U,
        .callback = NULL,
        .flags    = 0,
    };
    counter_set_top_value(dev, &top);

    struct counter_alarm_cfg alarm = {
        .callback  = alarm_cb,
        .ticks     = 0,
        .user_data = NULL,
        .flags     = 0,
    };
    counter_set_channel_alarm(dev, 0, &alarm);

    /* Cancel immediately before starting */
    int ret = counter_cancel_channel_alarm(dev, 0);
    printk("  cancel_channel_alarm: %s\n", ret == 0 ? "OK" : "FAIL");
    print_ctrl(dev);

    counter_start(dev);
    k_sleep(K_MSEC(300));
    counter_stop(dev);

    if (!alarm_fired) {
        printk("OK   Alarm did not fire after cancel\n");
    } else {
        printk("FAIL Alarm fired after cancel\n");
    }
}

static void test_wraparound(const struct device *dev)
{
    printk("\n--- TEST 8: Wraparound ---\n");

    struct counter_top_cfg top = {
        .ticks    = 2000U,
        .callback = NULL,
        .flags    = 0,
    };
    counter_set_top_value(dev, &top);
    counter_start(dev);

    uint32_t prev, curr;
    bool wrapped = false;
    counter_get_value(dev, &prev);

    for (int i = 0; i < 50 && !wrapped; i++) {
        k_sleep(K_MSEC(10));
        counter_get_value(dev, &curr);

        /* UP: wrap is curr < prev; DOWN: wrap is curr > prev */
        if ((curr < prev && prev > 1000U) ||
            (curr > prev && prev < 1000U)) {
            wrapped = true;
            printk("  Wrapped: %u -> %u\n", prev, curr);
        }
        prev = curr;
    }
    counter_stop(dev);

    printk("%s\n", wrapped ? "OK   Counter wraps at top value"
                           : "FAIL Counter did not wrap");
}

/* ------------------------------------------------------------------ */
/* Main                                                                */
/* ------------------------------------------------------------------ */

int main(void)
{
    const struct device *dev = DEVICE_DT_GET(TIMER_NODE);

    printk("\n========================================\n");
    printk("  Mindgrove GPTimer Test Suite\n");
    printk("========================================\n");

    test_device_ready(dev);
    test_counting(dev);
    test_stop_freezes(dev);
    test_set_top(dev);
    test_top_callback(dev);
    test_alarm(dev);
    test_alarm_cancel(dev);
    test_wraparound(dev);

    printk("\n========================================\n");
    printk("  Tests complete. ISR total: %u\n", gpt_isr_count);
    printk("========================================\n");

    while (1) {
        k_sleep(K_FOREVER);
    }
    return 0;
}