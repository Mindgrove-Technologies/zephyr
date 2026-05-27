// #include <zephyr/kernel.h>
// #include <zephyr/device.h>
// #include <zephyr/devicetree.h>
// #include <zephyr/drivers/spi.h>
// #include <zephyr/sys/printk.h>
// #include <zephyr/logging/log.h>
// #include <string.h>

// LOG_MODULE_REGISTER(spi_loopback, LOG_LEVEL_INF);

// #define SPI_NODE DT_NODELABEL(spi2)

// int main(void)
// {
//     const struct device *spi_dev = DEVICE_DT_GET(SPI_NODE);

//     /* ✅ Your improvement: device readiness check */
//     if (!device_is_ready(spi_dev)) {
//         printk("SPI device not ready\n");
//         return 0;
//     }

//     printk("=== SPI LOOPBACK TEST START ===\n");

//     /* ✅ Recommended stability delay */
//     k_busy_wait(100);

//     /* ✅ Safe + Correct SPI configuration */
//     struct spi_config spi_cfg = {
//         .frequency = 100000,  // safer initial speed (100 kHz)
//         .operation = SPI_OP_MODE_MASTER |   // 🔥 critical fix
//                      SPI_TRANSFER_MSB |
//                      SPI_WORD_SET(8),
//         .slave = 0,
//     };

//     /* Test buffers */
//     uint8_t tx_buf[4];
//     uint8_t rx_buf[4];

//     int len = sizeof(tx_buf) / sizeof(tx_buf[0]);  // ✅ generic-safe

//     /* Fill TX buffer */
//     for (int i = 0; i < len; i++) {
//         tx_buf[i] = i;
//         printk("TX[%d] = %d\n", i, tx_buf[i]);
//     }

//     struct spi_buf tx = {
//         .buf = tx_buf,
//         .len = len,
//     };

//     struct spi_buf rx = {
//         .buf = rx_buf,
//         .len = len,
//     };

//     struct spi_buf_set tx_set = {
//         .buffers = &tx,
//         .count = 1,
//     };

//     struct spi_buf_set rx_set = {
//         .buffers = &rx,
//         .count = 1,
//     };

//     /* 🔁 Continuous monitoring (your improvement) */
//     while (1) {

//         memset(rx_buf, 0, sizeof(rx_buf));  // ✅ clean RX before transfer

//         int ret = spi_transceive(spi_dev, &spi_cfg, &tx_set, &rx_set);

//         if (ret == 0) {
//             printk("SUCCESS\n");

//             printk("TX: ");
//             for (int i = 0; i < len; i++) {
//                 printk("%02x ", tx_buf[i]);
//             }

//             printk(" | RX: ");
//             for (int i = 0; i < len; i++) {
//                 printk("%02x ", rx_buf[i]);
//             }

//             printk("\n");
//         } else {
//             printk("SPI ERROR: %d\n", ret);
//         }

//         k_sleep(K_SECONDS(1));
//     }
// }

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/drivers/spi.h>


int main()
{
    const struct device *const dev = DEVICE_DT_GET(DT_NODELABEL(spi3));
        if (!device_is_ready(dev)) {
            printf("SPI device not ready\n");
            return 0;
        }
    struct spi_config config;
    config.operation = SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8);
    k_busy_wait(100);
    printf("Started\n");
    // uint8_t tx_buff[3] = {0x12, 0x34, 0x56};
    // uint16_t tx_buff[3] = {0x1234, 0xabcd, 0x5678};
    //uint32_t tx_buff[3] = {0x12345678, 0xabcdefaa, 0xafcefbca};

    // uint8_t rx_buff[3];
    // uint16_t rx_buff[3];
    //uint32_t rx_buff[3];
    uint8_t tx_buff[4] = {0x15, 0x38, 0x50, 0x89};
    uint8_t rx_buff[4];
	int len = sizeof(tx_buff) / sizeof(tx_buff[0]);

    // for(int i = 1;i<5;i++){
    //     tx_buff[i] = i;
    //     printf("\nTransmitted Data [%d] = %d", i, tx_buff[i]);
    // }
    printf("\nlen :%d", len);
    k_busy_wait(100);
    struct spi_buf tx_buf = { .buf = tx_buff, .len = len };
	struct spi_buf_set tx_bufs = { .buffers = &tx_buf, .count = 1};

    struct spi_buf rx_buf = { .buf = rx_buff, .len = len};
    struct spi_buf_set rx_bufs = { .buffers = &rx_buf, .count = 1};
    k_busy_wait(1000);
    int ret = spi_transceive(dev, &config, &tx_bufs, &rx_bufs);
    k_busy_wait(1000);
    for (int i = 0; i < 4; i++)
    {
        printf("\nReceived Data [%d] = %x", i, rx_buff[i]);
    }
    printf("completed transmission\n");

}