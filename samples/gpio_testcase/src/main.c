#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <stdio.h>

/* Use DT_NODELABEL to get the node ID for the GPIO controller defined as 'gpio0' in DTS */
#define GPIO_NODE DT_NODELABEL(gpio0)

// Check if the node label is available and defined
#if !DT_NODE_HAS_STATUS(GPIO_NODE, okay)
#error "Devicetree node label 'gpio0' not found or disabled in DTS."
#endif

void main(void)
{
    // Get the device structure for the GPIO controller
    const struct device *dev = DEVICE_DT_GET(GPIO_NODE); 
    int ret;
    
    if (!device_is_ready(dev)) {
        printf("Error: GPIO device %s is not ready.\n", dev->name);
        return;
    }

    // Configure Pin 0, 1, and 2 as Outputs, initialized Low
    
    // Configure Pin 0
    ret = gpio_pin_configure(dev, 0, GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW);
    if (ret < 0) {
        printf("Error configuring Pin 0: %d\n", ret);
        return;
    }

    // Configure Pin 1
    ret = gpio_pin_configure(dev, 1, GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW);
    if (ret < 0) {
        printf("Error configuring Pin 1: %d\n", ret);
        return;
    }

    // Configure Pin 2
    ret = gpio_pin_configure(dev, 2, GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW);
    if (ret < 0) {
        printf("Error configuring Pin 2: %d\n", ret);
        return;
    }

    printf("GPIO Output Toggle Test (Pins 0, 1, 2) Starting...\n");

    while (1) {
        // Toggle the state of all three pins simultaneously
        printf("Toggle...\n");
        ret = gpio_port_toggle_bits(dev, BIT(0) | BIT(1) | BIT(2));
        if (ret < 0) {
            printf("Error toggling pins: %d\n", ret);
        }
        
        k_msleep(500);
    }
}

