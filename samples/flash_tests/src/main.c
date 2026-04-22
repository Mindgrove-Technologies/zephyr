#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <string.h>

#define FLASH_DEVICE_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(mindgrove_qspi_flash)
#define TEST_ADDR  0x1000
#define TEST_SIZE  128
#define FLASH_TOTAL_SIZE (1 * 1024 * 1024)

/* ================= HELPERS ================= */

static void print_buf(const char *label, const uint8_t *buf, size_t len)
{
    printk("%s", label);
    for (size_t i = 0; i < len; i++) {
        printk("0x%02X ", buf[i]);
    }
    printk("\n");
}

/* ================= INDIRECT READ TEST ================= */

int test_indirect_read(const struct device *flash_dev)
{
    uint8_t read_data[8];
    int ret;

    printk("\n--- Indirect Read Test ---\n");
    printk("Reading 8 bytes from address 0x000000\n");

    ret = flash_read(flash_dev, 0x0, read_data, sizeof(read_data));
    if (ret != 0) {
        printk("FAIL: flash_read returned %d\n", ret);
        return -1;
    }

    print_buf("Read data: ", read_data, 8);
    printk("PASS: Indirect read test\n");
    return 0;
}

/* ================= INDIRECT WRITE TEST ================= */

int test_indirect_write(const struct device *flash_dev)
{
    uint8_t write_data[16] = {
        [0 ... 7]  = 0x11,
        [8 ... 15] = 0x22
    };
    uint8_t read_data[16];
    off_t addr = 0x600;
    off_t erase_addr = addr & ~(off_t)(4096 - 1);  /* 0x000 */
    int ret;

    printk("\n--- Indirect Write Test ---\n");

    /* Erase */
    printk("Erasing 4KB sector at 0x%06X\n", (uint32_t)erase_addr);
    ret = flash_erase(flash_dev, erase_addr, 4096);
    if (ret != 0) {
        printk("FAIL: flash_erase returned %d\n", ret);
        return -1;
    }
    printk("Erase OK\n");

    /* Write */
    print_buf("Writing data: ", write_data, sizeof(write_data));
    printk("Write address: 0x%06X, length: %zu\n", (uint32_t)addr, sizeof(write_data));
    ret = flash_write(flash_dev, addr, write_data, sizeof(write_data));
    if (ret != 0) {
        printk("FAIL: flash_write returned %d\n", ret);
        return -1;
    }
    printk("Write OK\n");

    /* Read back */
    printk("Reading back %zu bytes from 0x%06X\n", sizeof(read_data), (uint32_t)addr);
    ret = flash_read(flash_dev, addr, read_data, sizeof(read_data));
    if (ret != 0) {
        printk("FAIL: flash_read returned %d\n", ret);
        return -1;
    }
    print_buf("Read back:    ", read_data, sizeof(read_data));

    /* Compare */
    printk("Comparing written vs read:\n");
    int mismatch = 0;
    for (int i = 0; i < 16; i++) {
        printk("  [%2d] wrote=0x%02X  read=0x%02X  %s\n",
               i, write_data[i], read_data[i],
               (read_data[i] == write_data[i]) ? "OK" : "MISMATCH");
        if (read_data[i] != write_data[i]) {
            mismatch++;
        }
    }

    if (mismatch > 0) {
        printk("FAIL: %d byte(s) mismatched\n", mismatch);
        return -1;
    }

    printk("PASS: Indirect write test\n");
    return 0;
}

/* ================= CHIP ERASE TEST ================= */

int test_chip_erase(const struct device *flash_dev)
{
    uint8_t write_data[16] = {
        [0 ... 7]  = 0x11U,
        [8 ... 15] = 0x22U
    };
    uint8_t read_data[16];
    off_t addr = 0x600;
    int ret;

    printk("\n--- Chip Erase Test ---\n");

    /* Write */
    print_buf("Writing data: ", write_data, sizeof(write_data));
    printk("Write address: 0x%06X\n", (uint32_t)addr);
    ret = flash_write(flash_dev, addr, write_data, sizeof(write_data));
    if (ret != 0) {
        printk("FAIL: flash_write returned %d\n", ret);
        return -1;
    }
    printk("Write OK\n");

    /* Verify write */
    printk("Verifying write at 0x%06X\n", (uint32_t)addr);
    ret = flash_read(flash_dev, addr, read_data, sizeof(read_data));
    if (ret != 0) {
        printk("FAIL: flash_read returned %d\n", ret);
        return -1;
    }
    print_buf("Read back:    ", read_data, sizeof(read_data));

    int pre_mismatch = 0;
    for (int i = 0; i < 16; i++) {
        printk("  [%2d] wrote=0x%02X  read=0x%02X  %s\n",
               i, write_data[i], read_data[i],
               (read_data[i] == write_data[i]) ? "OK" : "MISMATCH");
        if (read_data[i] != write_data[i]) {
            pre_mismatch++;
        }
    }
    if (pre_mismatch > 0) {
        printk("FAIL: pre-erase verify failed (%d mismatch)\n", pre_mismatch);
        return -1;
    }
    printk("Pre-erase verify OK\n");

    /* Chip erase */
    printk("Issuing chip erase (offset=0x000000, size=0x%06X)\n", FLASH_TOTAL_SIZE);
    ret = flash_erase(flash_dev, 0, FLASH_TOTAL_SIZE);
    if (ret != 0) {
        printk("FAIL: flash_erase (chip) returned %d\n", ret);
        return -1;
    }
    printk("Chip erase OK\n");

    /* Verify erased */
    printk("Verifying erase at 0x%06X (expect all 0xFF)\n", (uint32_t)addr);
    ret = flash_read(flash_dev, addr, read_data, sizeof(read_data));
    if (ret != 0) {
        printk("FAIL: flash_read returned %d\n", ret);
        return -1;
    }
    print_buf("Post-erase:   ", read_data, sizeof(read_data));

    int post_mismatch = 0;
    for (int i = 0; i < 16; i++) {
        printk("  [%2d] expected=0xFF  read=0x%02X  %s\n",
               i, read_data[i],
               (read_data[i] == 0xFFU) ? "OK" : "NOT ERASED");
        if (read_data[i] != 0xFFU) {
            post_mismatch++;
        }
    }
    if (post_mismatch > 0) {
        printk("FAIL: %d byte(s) not erased\n", post_mismatch);
        return -1;
    }

    printk("PASS: Chip erase test\n");
    return 0;
}

/* ================= MAIN ================= */

void main(void)
{
    const struct device *flash_dev = DEVICE_DT_GET(FLASH_DEVICE_NODE);

    if (!device_is_ready(flash_dev)) {
        printk("Flash device %s not ready\n", flash_dev->name);
        return;
    }
    printk("Flash device ready: %s\n", flash_dev->name);

    if (test_indirect_read(flash_dev) != 0) {
        printk("Indirect read test FAILED\n");
        return;
    }

    if (test_indirect_write(flash_dev) != 0) {
        printk("Indirect write test FAILED\n");
        return;
    }

    if (test_chip_erase(flash_dev) != 0) {
        printk("Chip erase test FAILED\n");
        return;
    }

    printk("\nAll tests PASSED\n");
}