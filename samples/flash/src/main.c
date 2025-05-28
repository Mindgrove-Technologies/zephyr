#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/drivers/flash.h>

int main()
{
	const struct device *const flash_dev = DEVICE_DT_GET(DT_NODELABEL(qspi0));
	off_t offset = 0x200;
	size_t size = 16;
	uint8_t write_data[16];
	uint8_t read_data[16];
	int ret;

	for (int i = 0; i < size; i++) {
        write_data[i] = i;
    }

	if (!device_is_ready(flash_dev)) {
		printk("Flash device not ready!\n");
		return 1;
    }

// Erase the flash region
    ret = flash_erase(flash_dev, offset, size);
    if (ret != 0) {
        printk("Flash erase failed! (err %d)\n", ret);
        return 1;
    }

    // Write data to flash
    ret = flash_write(flash_dev, offset, write_data, size);
    if (ret != 0) {
        printk("Flash write failed! (err %d)\n", ret);
        return 1;
    }

    // Read data back from flash
    ret = flash_read(flash_dev, offset, read_data, size);
    if (ret != 0) {
        printk("Flash read failed! (err %d)\n", ret);
        return 1;
    }

    // Verify data
    for (int i = 0; i < size; i++) {
        if (read_data[i] != write_data[i]) {
            printk("Data mismatch at offset %d: wrote 0x%02x, read 0x%02x\n", i, write_data[i], read_data[i]);
            return 1;
        }
    }

    printk("Flash erase, write, and read successful!\n");
}