#include <stdio.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>

#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/display.h>
#include <zephyr/display/display_ili9xxx.h>
#include <zephyr/display/display_ili9341.h>

#include <zephyr/dt-bindings/display/panel.h>


// int main()
// {
//     const struct device *const dev = DEVICE_DT_GET(DT_NODELABEL(spi1));
//     // ILI9341_REGS_INIT(spi1);
//     struct spi_config *config;
//     const struct display_buffer_descriptor *desc;
//     // const void *buf;
//     uint16_t buf[] = {0xFFFF};
//     struct display_driver_api display;
//     printf("hello world\n");
//     // ili9341_regs_init(dev);
//     // printf("initialization\n");
//     display.write(dev, 1, 1, desc->, 0xFFFF);
//     printf("write\n");
//     // if (ret != 0) {
//     //     printf("Failed to write to display. Error code: %d\n", ret);
//     //     return -1;
//     // } else {
//     //     printf("Successfully wrote to display.\n");
//     // }
// }

// #define DISPLAY_DEV_NAME  "ilitek,ili9341"  

// // Assuming you want to write red color (replace with your desired color)
// #define COLOR_TO_WRITE  0xFF0000

// int main()
// {
// 	const struct device *const display_dev = DEVICE_DT_GET(DT_NODELABEL(ili9341));
// 	if (!display_dev) {
// 		printk("Error: Could not find display device\n");
// 		return 1;
// 	}
// 	printk("display device\n");

// 	// int ret1 = ili9341_regs_init(display_dev);
// 	// Assuming a width and height of 10 pixels each for this example (adjust as needed)
// 	uint16_t width = 1;
// 	uint16_t height = 1;

// 	// Assuming a single pixel buffer descriptor for simplicity
// 	struct display_buffer_descriptor desc = {
// 		.width = width,
// 		.height = height,
// 		.pitch = width * sizeof(uint16_t), // Assuming 16-bit color depth
// 		.buf_size = width * height * sizeof(uint16_t),
// 	};
// 	printk("struct\n");

// 	// Allocate memory for the pixel buffer (replace with your allocation method)
// 	uint16_t pixel_buffer[width * height]; // Pre-allocate buffer in program memory
// 	if (!pixel_buffer) {
// 		printk("Error: Failed to allocate memory for pixel buffer\n");
// 		return 1;
// 	}

// 	// Fill the pixel buffer with the desired color
// 	for (int i = 0; i < width * height; i++) {
// 		pixel_buffer[i] = COLOR_TO_WRITE;
// 		printk("i = %d\n", i);
// 	}

// 	// Set the display area to write to (replace with desired coordinates)
// 	uint16_t x_pos = 0;
// 	uint16_t y_pos = 0;
// 	// Get the display API
// 	const struct display_driver_api *api;
// 	printk("struct\n");
// 	// Call the write function using the display API
// 	int ret = api->write(display_dev, x_pos, y_pos, &desc, pixel_buffer);
// 	if (ret < 0) {
// 		printk("Error: Failed to write to display (%d)\n", ret);
// 		free(pixel_buffer);
// 		return 1;
// 	}

// 	printk("Successfully wrote color to display\n");

// 	// Free the allocated memory
// 	free(pixel_buffer);

// 	//   return 0;
// }

#define BKGCOLOR 0x88

int spi_ctrl_transmit(const struct device *dev, uint8_t cmd,
                    const void *tx_data, uint16_t tx_len){
	struct spi_buf tx_buf;
	struct spi_buf_set tx_bufs; 
	const struct ili9xxx_config *config;

	tx_buf.buf = &cmd;
	tx_buf.len = 1U;
	tx_bufs.buffers = &tx_buf;
	tx_bufs.count = 1U;

	gpio_pin_set_dt(&config->cmd_data, 1);
	int err = spi_write_dt(&config->spi, &tx_bufs);
	if(err < 0){
		LOG_ERR("spi_ctrl_transmit: Error on %s", config->spi.bus->name);		return err;
		return err;
	}

	if(tx_data != NULL){
		tx_buf.buf = (void *)tx_data;
		tx_buf.len = tx_len;
		gpio_pin_set_dt(&config->cmd_data, 0);
		err = spi_write_dt(&config->spi, &tx_bufs);
		if (err < 0) {
			LOG_ERR("spi_ctrl_transmit: Error on %s", config->spi.bus->name);
			return err;
		}		
	}
}

int screen_write(const struct device *dev, uint8_t x_start, uint8_t y_start,
                 const struct display_buffer_descriptor *desc, const void *data) {
    struct spi_buf tx_buf;
    struct spi_buf_set tx_bufs;
    int err;
	const struct ili9xxx_config *config;

    // 1. Set CASET and PASET commands to define the active area
    err = spi_ctrl_transmit(dev, ILI9XXX_CASET, &x_start, 1U);
    if (err < 0) {
        return err;
    }
    err = spi_ctrl_transmit(dev, ILI9XXX_PASET, &y_start, 1U);
    if (err < 0) {
        return err;
    }

    // 2. Set RAMWR command to initiate writing to RAM
    err = spi_ctrl_transmit(dev, ILI9XXX_RAMWR, NULL, 0U);
    if (err < 0) {
        return err;
    }

    // 3. Set SPI transmit buffers and write data
    tx_buf.buf = data;
    tx_buf.len = desc->width * desc->height * sizeof(uint8_t);
    tx_bufs.buffers = &tx_buf;
    tx_bufs.count = 1U;
    err = spi_write_dt(&config->spi, &tx_bufs);
    if (err < 0) {
        LOG_ERR("screen_write: Error on %s", config->spi.bus->name);
        return err;
    }

    return 0;
}

void set_background_color(const struct device *dev,
                         const struct display_capabilities *cap) {
    // Fill the buffer with the background color
    memset(buf, BKGCOLOR, sizeof(buf));

    // Write the background color to each row of the display
    for (int idx = 0; idx < cap->y_resolution; idx += h_step) {
        int ret = screen_write(screen, 0, idx, &buf_desc, buf);
        if (ret < 0) {
            return;
        }
    }

    // Free the allocated buffer memory
    k_free(buf);
}

