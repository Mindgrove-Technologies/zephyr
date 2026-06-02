#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <stdint.h>
#include <stdio.h>

/* Get PSRAM base address from DTS — the memory-mapped window address */
#define PSRAM_DEVICE_NODE   DT_COMPAT_GET_ANY_STATUS_OKAY(mindgrove_qspi_psram)
#define PSRAM_BASE_ADDR     (0xB0000000U)
#define PSRAM_TEST_WORDS    16U

/* ================= HELPERS ================= */

static int test_word_rw(volatile uint32_t *psram)
{
    printk("\n--- Word Read/Write Test ---\n");
    printk("Writing %u words to PSRAM @ 0x%08X\n", PSRAM_TEST_WORDS,
           PSRAM_BASE_ADDR);

    for (uint32_t i = 0; i < PSRAM_TEST_WORDS; i++) {
        psram[i] = 0xA5A50000U | i;
    }

    printk("Reading back...\n");
    int fail = 0;
    for (uint32_t i = 0; i < PSRAM_TEST_WORDS; i++) {
        uint32_t expected = 0xA5A50000U | i;
        uint32_t got      = psram[i];
        printk("  [%2u] expected=0x%08X  got=0x%08X  %s\n",
               i, expected, got,
               (got == expected) ? "OK" : "MISMATCH");
        if (got != expected) {
            fail++;
        }
    }

    if (fail) {
        printk("FAIL: %d word(s) mismatched\n", fail);
        return -1;
    }
    printk("PASS: Word read/write test\n");
    return 0;
}

static int test_byte_rw(volatile uint8_t *psram_bytes)
{
    printk("\n--- Byte Read/Write Test ---\n");

    uint8_t pattern[16];
    for (int i = 0; i < 16; i++) {
        pattern[i] = (uint8_t)(0xC0 + i);
        psram_bytes[0x100 + i] = pattern[i];   /* offset to avoid overlap */
    }

    int fail = 0;
    for (int i = 0; i < 16; i++) {
        uint8_t got = psram_bytes[0x100 + i];
        printk("  [%2d] wrote=0x%02X  read=0x%02X  %s\n",
               i, pattern[i], got,
               (got == pattern[i]) ? "OK" : "MISMATCH");
        if (got != pattern[i]) fail++;
    }

    if (fail) {
        printk("FAIL: %d byte(s) mismatched\n", fail);
        return -1;
    }
    printk("PASS: Byte read/write test\n");
    return 0;
}

static int test_memset_pattern(volatile uint32_t *psram)
{
    printk("\n--- Memset Pattern Test ---\n");

    /* Fill with 0xDEADBEEF */
    for (uint32_t i = 0; i < PSRAM_TEST_WORDS; i++) {
        psram[0x200 + i] = 0xDEADBEEFU;
    }

    int fail = 0;
    for (uint32_t i = 0; i < PSRAM_TEST_WORDS; i++) {
        uint32_t got = psram[0x200 + i];
        if (got != 0xDEADBEEFU) {
            printk("  [%2u] expected=0xDEADBEEF  got=0x%08X  MISMATCH\n",
                   i, got);
            fail++;
        }
    }

    if (fail) {
        printk("FAIL: %d word(s) mismatched\n", fail);
        return -1;
    }
    printk("PASS: Memset pattern test\n");
    return 0;
}

static int test_zero_fill(volatile uint32_t *psram)
{
    printk("\n--- Zero Fill Test ---\n");

    for (uint32_t i = 0; i < PSRAM_TEST_WORDS; i++) {
        psram[i] = 0xFFFFFFFFU;
    }
    for (uint32_t i = 0; i < PSRAM_TEST_WORDS; i++) {
        psram[i] = 0x00000000U;
    }

    int fail = 0;
    for (uint32_t i = 0; i < PSRAM_TEST_WORDS; i++) {
        uint32_t got = psram[i];
        if (got != 0x00000000U) {
            printk("  [%2u] expected=0x00000000  got=0x%08X  MISMATCH\n",
                   i, got);
            fail++;
        }
    }

    if (fail) {
        printk("FAIL: %d word(s) not zeroed\n", fail);
        return -1;
    }
    printk("PASS: Zero fill test\n");
    return 0;
}

/* ================= MAIN ================= */

int main(void)
{
    const struct device *psram_dev =
        DEVICE_DT_GET(PSRAM_DEVICE_NODE);

    if (!device_is_ready(psram_dev)) {
        printk("PSRAM device not ready\n");
        return -1;
    }
    printk("PSRAM device ready: %s\n", psram_dev->name);
    printk("Memory-mapped base: 0x%08X\n\n", PSRAM_BASE_ADDR);

    volatile uint32_t *psram       = (volatile uint32_t *)PSRAM_BASE_ADDR;
    volatile uint8_t  *psram_bytes = (volatile uint8_t  *)PSRAM_BASE_ADDR;

    if (test_word_rw(psram) != 0) {
        printk("Word R/W test FAILED\n");
        return -1;
    }

    if (test_byte_rw(psram_bytes) != 0) {
        printk("Byte R/W test FAILED\n");
        return -1;
    }

    if (test_memset_pattern(psram) != 0) {
        printk("Memset pattern test FAILED\n");
        return -1;
    }

    if (test_zero_fill(psram) != 0) {
        printk("Zero fill test FAILED\n");
        return -1;
    }

    printk("\nAll PSRAM tests PASSED\n");
    return 0;
}