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
    uint8_t tx_buff[4] = {0x12, 0x34, 0x56, 0x78};
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
        printf("\nReceived Data [%d] = %d", i, rx_buff[i]);
    }
    printf("completed transmission\n");

}
