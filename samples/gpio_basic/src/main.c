#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

#include <stdio.h>
#include <string.h>

#include <zephyr/drivers/gpio.h>



void main(){
    
    const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(gpio0)); //DEVICE_DT_GET_ONE(shakti_gpio0) This can also be used to define the device.
    
    int ret;
    ret = gpio_pin_configure(dev, 0, 1);
    ret = gpio_pin_configure(dev, 1, 1);
    ret = gpio_pin_configure(dev, 2, 1);

    while (1){
        printf("set\n");
        ret = gpio_port_toggle_bits(dev, 0);
        ret = gpio_port_toggle_bits(dev, 1);
        ret = gpio_port_toggle_bits(dev, 2);
        // ret = gpio_port_set_bits_raw(dev, 0);
        // ret = gpio_port_set_bits_raw(dev, 1);
        // ret = gpio_port_set_bits_raw(dev, 2);
        // ret = gpio_port_clear_bits_raw(dev, 0);
        // ret = gpio_port_clear_bits_raw(dev, 1);
        // ret = gpio_port_clear_bits_raw(dev, 2);
    }
}