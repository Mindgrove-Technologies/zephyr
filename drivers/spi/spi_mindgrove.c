/**
 * Project                           : Secure IoT SoC
 * Name of the file                  : spi_mindgrove.c
 * Brief Description of file         : This is a zephyr rtos SPI Driver file for Mindgrove Silicon's SPI Peripheral.
 * Name of Author                    : Kishore J
 * Email ID                          : kishore@mindgrovetech.in
 * 
 * @file spi_mindgrove.c
 * @author Kishore J (kishore@mindgrovetech.in)
 * @brief This is a zephyr rtos SPI Driver file for Mindgrove Silicon's SPI Peripheral.
 * @version 0.1
 * @date 2024-04-17
 * 
 * @copyright Copyright (c) Mindgrove Technologies Pvt. Ltd 2025. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT mindgrove_spi

#define LOG_LEVEL CONFIG_SPI_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(spi_mindgrove);

#include <stdio.h>
#include <stdbool.h>
#include <soc.h>
#include <zephyr/sys/util.h>
#include<zephyr/kernel.h>
#include "spi_mindgrove.h"

/*HELPER FUNCTIONS*/
int spi_number;
spi_struct *spi_instance[SPI_MAX_COUNT];

#define CLEAR_MASK                         0x0000

typedef struct 
{
    uint8_t spi_number      :2;
    uint8_t pol             :1;
    uint8_t pha             :1;
    uint8_t prescale;
    uint8_t setup_time;
    uint8_t hold_time;
    uint8_t spi_mode        :1;
    uint8_t lsb_first       :1;
    uint8_t comm_mode       :2;
    uint8_t spi_size;
    uint8_t bits;
    uint8_t configure       :1;
}SPI_Config_t;

static int spi_mindgrove_transceive(const struct device *dev,
			  const struct spi_config *config,
			  const struct spi_buf_set *tx_bufs,
			  const struct spi_buf_set *rx_bufs)
{
  uint32_t len;
  volatile uint32_t temp = 0;
  SPI_Config_t spi_config;
  while (1){
    temp =  spi_instance[spi_number]->comm_status;
    temp = temp & SPI_BUSY;
    if (temp != SPI_BUSY){
        temp =0;
        break;
    }
  }

  if ((config->operation & 0x1) == 0)
  {
    spi_config.spi_mode = MASTER;
  }
  else
  {
    printk("Slave is not supported\n");
    return 1;
  }
  
  // spi.h have 3 modes, but secure-iot spi only supports 2 modes pol and pha. 
  if((config->operation & POL_AND_PHA) == POL_AND_PHA){
    spi_config.pol = 1;
    spi_config.pha = 1;
  } else if((config -> operation & INV_POLANDPHA) == INV_POLANDPHA){
    spi_config.pol = 0;
    spi_config.pha = 0;
  }
  else{
    printk("Invalid pol and pha combination \n");
    return 0;
  }

  // If lsb is set, then 4th bit in the operation is set to 1 or if msb is set the 4th bit in the operation is set to 0.
  if((config -> operation & SPI_TRANSFER_LSB) == SPI_TRANSFER_MSB){
    spi_config.lsb_first = MSB_FIRST;
  }
  else {
    spi_config.lsb_first = LSB_FIRST; 
  }
  
  // spi data size fields in operation is from 5 to 10, if the word size is set to 8 then 8th bit is set to 1.
  if( config -> operation & SPI_WORD_SET(8))
  {
    spi_config.spi_size = DATA_SIZE_8;
  }
  // spi data size fields in operation is from 5 to 10, if the word size is set to 16 then 9th bit is set to 1.
  else if(config -> operation & SPI_WORD_SET(16))
  { 
    spi_config.spi_size = DATA_SIZE_16;
  }
  // spi data size fields in operation is from 5 to 10, if the word size is set to 32 then 10th bit is set to 1.
  else if(config -> operation & SPI_WORD_SET(32))
  {
    spi_config.spi_size = DATA_SIZE_32;
  }
  else {
    printk("Invalid data size \n");
    return 0;
  }

  
  if((config -> operation & HALFDUPLEX) ==  HALFDUPLEX){
    spi_config.comm_mode = HALF_DUPLEX;
  }
  else if((config -> operation & FULLDUPLEX) == FULLDUPLEX){
    spi_config.comm_mode = FULL_DUPLEX;
  }
  else if((config -> operation & SIMPLEX_TX) == SIMPLEX_TX)
  {
    spi_config.comm_mode = SIMPLEX_TX;
  }
  else if((config -> operation & SIMPLEX_RX) == SIMPLEX_RX)
  {
    spi_config.comm_mode = SIMPLEX_RX;
  }
  else{
    printk("Mode is not supported \n");
    return 0;
  } 
  spi_config.prescale = 10;
  spi_config.setup_time = 0x0;
  spi_config.hold_time = 0x0;

  uint8_t rx_buff[16];
  struct spi_buf rx_loc_bufs = { .buf = rx_buff, .len = tx_bufs->buffers->len};
  struct spi_buf_set rx_loc_buffs = { .buffers = &rx_loc_bufs, .count = 1};
  if(rx_bufs == NULL){
    rx_bufs = &rx_loc_buffs;
  }

  if ((spi_config.pol == 0 && spi_config.pha == 1)||(spi_config.pol == 1 && spi_config.pha == 0)) return 1;

  k_mutex_lock(&(((struct spi_mindgrove_cfg*)(dev->config))->mutex),K_FOREVER);
  spi_instance[spi_number]->clk_control = CLEAR_MASK;
  spi_instance[spi_number]->clk_control = SPI_CLK_POLARITY(spi_config.pol) | SPI_CLK_PHASE(spi_config.pha) | SPI_PRESCALE(spi_config.prescale) | SPI_SS2TX_DELAY(spi_config.setup_time) | SPI_TX2SS_DELAY(spi_config.hold_time);

  spi_instance[spi_number]->comm_control = CLEAR_MASK;
  if (spi_config.spi_mode == MASTER)
  {
    spi_instance[spi_number]->comm_control = SPI_MASTER(spi_config.spi_mode) | SPI_LSB_FIRST(spi_config.lsb_first) | SPI_COMM_MODE(spi_config.comm_mode) | SPI_TOTAL_BITS_TX(spi_config.spi_size) | SPI_TOTAL_BITS_RX(spi_config.spi_size) | SPI_OUT_EN_SCLK(1) | SPI_OUT_EN_NCS(1) | SPI_OUT_EN_MOSI(1) | SPI_OUT_EN_MISO(0);
  }
  else
  {
    spi_instance[spi_number]->comm_control = SPI_MASTER(spi_config.spi_mode) | SPI_LSB_FIRST(spi_config.lsb_first) | SPI_COMM_MODE(spi_config.comm_mode) | SPI_TOTAL_BITS_TX(spi_config.spi_size) | SPI_TOTAL_BITS_RX(spi_config.spi_size) | SPI_OUT_EN_SCLK(0) | SPI_OUT_EN_NCS(0) | SPI_OUT_EN_MOSI(0) | SPI_OUT_EN_MISO(1);
  }
  
  if (tx_bufs == NULL && rx_bufs == NULL) return 1;
  spi_context_buffers_setup(&SPI_DATA(dev)->ctx, tx_bufs, rx_bufs, 1);
  len = tx_bufs ? tx_bufs->buffers->len : rx_bufs->buffers->len;

  if (spi_config.comm_mode == SIMPLEX_TX)
  {
    for (int i = 0; i < len; i++)
    {
      if ((spi_config.spi_size == DATA_SIZE_8) && ((spi_instance[spi_number]->fifo_status & SPI_TX_FULL) != SPI_TX_FULL))
      {
        spi_instance[spi_number]->data_tx.data_8 = ((uint8_t*)(tx_bufs->buffers->buf))[i];
        printk("tx_data= %d\n", spi_instance[spi_number]->data_tx.data_8);
      }
      else if ((spi_config.spi_size == DATA_SIZE_16) && (((spi_instance[spi_number]->fifo_status & SPI_TX_30 == SPI_TX_30) && ((spi_instance[spi_number]->comm_status & SPI_TX_FIFO(7)) == SPI_TX_FIFO(7))) || ((spi_instance[spi_number]->comm_status & SPI_TX_FIFO(7)) < SPI_TX_FIFO(7))))
      {
        spi_instance[spi_number]->data_tx.data_16 = ((uint16_t*)(tx_bufs->buffers->buf))[i];
        printk("tx_data= %d\n", spi_instance[spi_number]->data_tx.data_16);
      }
      else if ((spi_config.spi_size == DATA_SIZE_32) && (((spi_instance[spi_number]->fifo_status & SPI_TX_28 == SPI_TX_28) && ((spi_instance[spi_number]->comm_status & SPI_TX_FIFO(7)) == SPI_TX_FIFO(6))) || ((spi_instance[spi_number]->comm_status & SPI_TX_FIFO(7)) < SPI_TX_FIFO(6))))
      {
        spi_instance[spi_number]->data_tx.data_32 = ((uint32_t*)(tx_bufs->buffers->buf))[i];
        printk("tx_data= %d\n", spi_instance[spi_number]->data_tx.data_32);
      }
      while (1)
      {
        temp = spi_instance[spi_number]->comm_status & SPI_TX_EN;
        if (temp != SPI_TX_EN)
        {
          temp = 0;
          break;
        }        
      }
      if ((spi_instance[spi_number]->fifo_status & SPI_TX_EMPTY) != SPI_TX_EMPTY)
      {
        while (1)
        {
          temp = spi_instance[spi_number]->comm_status & SPI_BUSY;
          if (temp != SPI_BUSY)
          {
            temp = 0;
            break;
          }          
        }
        spi_instance[spi_number]->comm_control |= SPI_ENABLE(ENABLE);
      }      
    }    
  }

  else if (spi_config.comm_mode == SIMPLEX_RX)
  {
    for (int i = 0; i < len; i++)
    {
      if ((spi_instance[spi_number]->comm_control & SPI_COMM_MODE(3)) == SPI_COMM_MODE(1))
      {
        while (1)
        {
          temp = spi_instance[spi_number]->comm_status & SPI_BUSY;
          if (temp != SPI_BUSY)
          {
            temp = 0;
            break;
          }          
        }
        spi_instance[spi_number]->comm_control |= SPI_ENABLE(ENABLE);
      }
      if (spi_config.spi_size == DATA_SIZE_8)
      {
        while (1)
        {
          temp = spi_instance[spi_number]->fifo_status & SPI_RX_EMPTY;
          if (temp != SPI_RX_EMPTY)
          {
            temp = 0;
            break;
          }
        }
        ((uint8_t*)(rx_bufs->buffers->buf))[i] = spi_instance[spi_number]->data_rx.data_8;
        printk("rx_data= %d\n", ((uint8_t*)(rx_bufs->buffers->buf))[i]);
      }
      else if (spi_config.spi_size == DATA_SIZE_16)
      {
        while (1)
        {
          temp = spi_instance[spi_number]->comm_status & SPI_RX_FIFO(7);
          if (temp >= SPI_RX_FIFO(1))
          {
            temp = 0;
            break;
          }          
        }
        ((uint16_t*)(rx_bufs->buffers->buf))[i] = spi_instance[spi_number]->data_rx.data_16;
        printk("rx_data= %d\n", ((uint16_t*)(rx_bufs->buffers->buf))[i]);
      }
      else if (spi_config.spi_size == DATA_SIZE_32)
      {
        while (1)
        {
          temp = spi_instance[spi_number]->comm_status & SPI_RX_FIFO(7);
          if (temp >= SPI_RX_FIFO(2))
          {
            temp = 0;
            break;
          }          
        }
        ((uint32_t*)(rx_bufs->buffers->buf))[i] = spi_instance[spi_number]->data_rx.data_32;
        printk("rx_data= %d\n", ((uint32_t*)(rx_bufs->buffers->buf))[i]);
      }
    } 
  }
  
  else if (spi_config.comm_mode == FULL_DUPLEX || spi_config.comm_mode == HALF_DUPLEX)
  {
    printk("mode = %d\n", spi_config.comm_mode);
    for (int i = 0; i < len; i++)
    {
      if ((spi_config.spi_size == DATA_SIZE_8) && ((spi_instance[spi_number]->fifo_status & SPI_TX_FULL) != SPI_TX_FULL))
      {
        spi_instance[spi_number]->data_tx.data_8 = ((uint8_t*)(tx_bufs->buffers->buf))[i];
      }
      else if ((spi_config.spi_size == DATA_SIZE_16) && (((spi_instance[spi_number]->fifo_status & SPI_TX_30 == SPI_TX_30) && ((spi_instance[spi_number]->comm_status & SPI_TX_FIFO(7)) == SPI_TX_FIFO(7))) || ((spi_instance[spi_number]->comm_status & SPI_TX_FIFO(7)) < SPI_TX_FIFO(7))))
      {
        printk("tx_data= %d\n", spi_instance[spi_number]->data_tx.data_16);
        spi_instance[spi_number]->data_tx.data_16 = ((uint16_t*)(tx_bufs->buffers->buf))[i];
      }
      else if ((spi_config.spi_size == DATA_SIZE_32) && (((spi_instance[spi_number]->fifo_status & SPI_TX_28 == SPI_TX_28) && ((spi_instance[spi_number]->comm_status & SPI_TX_FIFO(7)) == SPI_TX_FIFO(6))) || ((spi_instance[spi_number]->comm_status & SPI_TX_FIFO(7)) < SPI_TX_FIFO(6))))
      {
        printk("tx_data= %d\n", spi_instance[spi_number]->data_tx.data_32);
        spi_instance[spi_number]->data_tx.data_16 = ((uint16_t*)(tx_bufs->buffers->buf))[i];
      }
      while (1)
      {
        temp = spi_instance[spi_number]->comm_status & SPI_TX_EN;
        if (temp != SPI_TX_EN)
        {
          temp = 0;
          break;
        }        
      }
      if(((spi_instance[spi_number]->fifo_status & SPI_TX_EMPTY) != SPI_TX_EMPTY) || ((spi_instance[spi_number]->comm_control & SPI_COMM_MODE(3)) == SPI_COMM_MODE(1)))
      {
        while (1)
        {
          temp = spi_instance[spi_number]->comm_status & SPI_BUSY;
          if (temp != SPI_BUSY)
          {
            temp = 0;
            break;
          }          
        }
        spi_instance[spi_number]->comm_control |= SPI_ENABLE(ENABLE);
      }
      if (spi_config.spi_size == DATA_SIZE_8)
      {
        while (1)
        {
          temp = spi_instance[spi_number]->fifo_status & SPI_RX_EMPTY;
          if (temp != SPI_RX_EMPTY)
          {
            temp = 0;
            break;
          }
        }        
        ((uint8_t*)(rx_bufs->buffers->buf))[i] = spi_instance[spi_number]->data_rx.data_8;
      }
      else if (spi_config.spi_size == DATA_SIZE_16)
      {
        while (1)
        {
          temp = spi_instance[spi_number]-> comm_status & SPI_RX_FIFO(7);
          if (temp >= SPI_RX_FIFO(1))
          {
            temp = 0;
            break;
          }          
        }
        printk("rx_data= %d\n", ((uint8_t*)(rx_bufs->buffers->buf))[i]);
        ((uint16_t*)(rx_bufs->buffers->buf))[i] = spi_instance[spi_number]->data_rx.data_16;
      }
      else if (spi_config.spi_size == DATA_SIZE_32)
      {
        while (1)
        {
          temp = spi_instance[spi_number]-> comm_status & SPI_RX_FIFO(7);
          if (temp >= SPI_RX_FIFO(2))
          {
            temp = 0;
            break;
          }          
        }
        printk("rx_data= %d\n", ((uint8_t*)(rx_bufs->buffers->buf))[i]);
        ((uint32_t*)(rx_bufs->buffers->buf))[i] = spi_instance[spi_number]->data_rx.data_32;
      }
    }    
  }
  k_mutex_unlock(&(((struct spi_mindgrove_cfg*)(dev->config))->mutex));
  return 0;  
}

int spi_mindgrove_init(const struct device *dev)
{ 
  spi_struct *spi_base;
  char *spi_inst; 
  spi_inst = dev->name;
  struct spi_mindgrove_cfg *confg = (struct spi_mindgrove_cfg *)dev->config;
  printk("SPI: %s\n", spi_inst);
  spi_number = spi_inst[6] - '0';
  gpio_pin_configure_dt(&(((struct spi_mindgrove_cfg*)(dev->config))->ncs),1);
  printk("SPI NUMBER: %d\n", spi_number);
  k_mutex_init(&(confg->mutex));
  if (spi_number < SPI_MAX_COUNT & spi_number >= 0){
    spi_instance[spi_number] = (spi_struct*) ( (SPI0_BASE_ADDRESS + ( spi_number * SPI_BASE_OFFSET) ) );
    spi_base = spi_instance[spi_number];
    #ifdef SPI_DEBUG
    printk("\nSPI%d Initialized..", spi_number);
    #endif
    k_busy_wait(10);
    return 0;
  }
  else{
    printk("\nInvalid SPI instance %d. This SoC supports only SPI-0 to SPI-3", spi_number);
    return -1;
  }
  return 0;
}

static int spi_mindgrove_release(const struct device *dev,
		       const struct spi_config *config)
{
	// spi_context_unlock_unconditionally(&SPI_DATA(dev)->ctx);
	return 0;
}

static struct spi_driver_api spi_mindgrove_api = {
	.transceive = spi_mindgrove_transceive,
	.release = spi_mindgrove_release,
};

#define SPI_INIT(n)	\
  static struct spi_mindgrove_data spi_mindgrove_data_##n = { \
    SPI_CONTEXT_INIT_LOCK(spi_mindgrove_data_##n, ctx), \
    SPI_CONTEXT_INIT_SYNC(spi_mindgrove_data_##n, ctx), \
    SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(n), ctx)	\
  }; \
  static struct spi_mindgrove_cfg spi_mindgrove_cfg_##n = { \
    .ncs = GPIO_DT_SPEC_INST_GET(n, cs_gpios),\
    .base = DT_INST_REG_ADDR(n) , \ 
    .f_sys = DT_INST_PROP(n, clock_frequency), \
  }; \
  DEVICE_DT_INST_DEFINE(n, \
        spi_mindgrove_init, \
        NULL, \
        &spi_mindgrove_data_##n, \
        &spi_mindgrove_cfg_##n, \
        POST_KERNEL, \
        CONFIG_SPI_INIT_PRIORITY, \
        &spi_mindgrove_api);

DT_INST_FOREACH_STATUS_OKAY(SPI_INIT)