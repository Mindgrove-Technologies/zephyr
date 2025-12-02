#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/irq.h>
#include <zephyr/sys/printk.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/interrupt_controller/intc_mindgrove_plic.h>

#define GPIO0_NODE         DT_NODELABEL(gpio0) 

// --- Configuration based on vendor IRQ mapping ---
#define GPIO_PIN_NUM       1        // Using pin 1 for the interrupt test
#define IRQ_PIN_OFFSET     32       // PLIC IRQ ID offset for GPIO
#define MY_DEV_IRQ         (uint32_t)(GPIO_PIN_NUM + IRQ_PIN_OFFSET) // PLIC IRQ ID = 33
#define MY_DEV_PRIO        1        // Priority for the PLIC source
#define MY_IRQ_FLAGS       0

static const struct device *gpio_dev;

// 1. Declare the ISR using the direct method macro
ISR_DIRECT_DECLARE(my_gpio_isr)
{
    printk(">>> ISR TRIGGERED: GPIO pin %d (IRQ ID: %d) <<<\n", GPIO_PIN_NUM, MY_DEV_IRQ);
    
    // In this context, the Zephyr kernel handles the PLIC EOI.
    // We only print and return.
    
    // The macro returns 0 by default, which is required for PLIC.
    return 0;
}

void main(void)
{
    printk("GPIO + ISR test starting...\n");

    gpio_dev = DEVICE_DT_GET(GPIO0_NODE);

    if (!device_is_ready(gpio_dev)) {
        printk("GPIO device not ready\n");
        return;
    }

    int ret;

    // 1. Connect the direct ISR to the PLIC source ID 
    // NOTE: This must be called before irq_enable, and before using the pin.
    IRQ_DIRECT_CONNECT(MY_DEV_IRQ, MY_DEV_PRIO, my_gpio_isr, MY_IRQ_FLAGS);
    
    // 2. Enable the IRQ line in the PLIC
    irq_enable(MY_DEV_IRQ);
    printk("PLIC IRQ %d enabled and connected directly.\n", MY_DEV_IRQ);


    // 3. Configure the GPIO pin interrupt settings (Falling Edge)
    ret = gpio_pin_interrupt_configure(gpio_dev,
                                 GPIO_PIN_NUM,
                                 GPIO_INT_EDGE_FALLING); 
    
    if (ret != 0) {
        printk("Error configuring GPIO interrupt: %d\n", ret);
        return;
    }
    
    // 4. Configure GPIO pin 1 as INPUT with PULL-UP resistor
    ret = gpio_pin_configure(gpio_dev, 
                                 GPIO_PIN_NUM, 
                                 GPIO_INPUT | GPIO_PULL_UP);

    if (ret != 0) {
        printk("Error configuring GPIO pin: %d\n", ret);
        return;
    }
    
    printk("System Ready. Waiting for interrupt...\n");

    int current_pin_val;
    
    while (1) {
        current_pin_val = gpio_pin_get_raw(gpio_dev, GPIO_PIN_NUM);
        
        if (current_pin_val >= 0) {
            printk("[Polling] Pin %d Status: %d\n", GPIO_PIN_NUM, current_pin_val);
        } else {
            printk("[Polling] Failed to read GPIO pin\n");
        }

        k_sleep(K_SECONDS(1)); 
    }
}

