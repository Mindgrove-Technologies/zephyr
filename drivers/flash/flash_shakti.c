#define DT_DRV_COMPAT shakti_qspi_flash

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/init.h>
#include<string.h>
#include <zephyr/logging/log.h>

#include "flash_shakti.h"
#include "spi_nor.h"
#include "jesd216.h"
#include "flash_priv.h"

// #define QSPI_DEBUG

//Bit vectors for CCR values
/**
 * The qspi_Data type is a union that can hold either a 32-bit, 16-bit, or 8-bit data value.
 * 
 */

typedef void (*flash_shakti_qspi_config_func_t)(void);

struct flash_shakti_qspi_config {
  flash_shakti_qspi_config_func_t cfg_func;
  struct flash_parameters parameters;
};

struct flash_shakti_qspi_data {
  struct k_sem sem;
	struct k_sem device_sync_sem;
};

uint8_t readFlagStatusRegister(const struct device *dev );

typedef union{
    uint32_t data_32;
    uint16_t data_16;
    uint8_t data_8;
} qspi_Data;


typedef struct
{
  uint32_t cr     ;
  uint32_t dcr    ;
  uint32_t sr     ;
  uint32_t fcr    ;
  uint32_t dlr    ;
  uint32_t ccr    ;
  uint32_t ar     ;
  uint32_t abr    ;
  qspi_Data dr    ;
  uint32_t psmkr  ;
  uint32_t psmar  ;
  uint32_t pir    ;
  uint32_t lptr   ;
}qspi_struct;

qspi_struct *qspi_instance[QSPI_MAX_COUNT];

/*----------------------------------------------qspi driver-----------------------------------------------------*/

int qspi_num;

int qspi_init(const struct device *dev)
{
  
  int qspi_base;
  char *qspi_inst;
  qspi_inst = dev->name;
  printk("QSPI: %s\n", qspi_inst);
  qspi_num = qspi_inst[7] - '0';
  printk("QSPI NUMBER: %d\n", qspi_num);

  if (qspi_num < QSPI_MAX_COUNT & qspi_num >= 0){
    printk("qspi initializtaion\n");
    qspi_instance[qspi_num] = (qspi_struct*) ( (QSPI0_BASE_ADDRESS + ( qspi_num * QSPI_OFFSET) ) );
    printk("qspi initializtaion\n");

    qspi_base = qspi_instance[qspi_num];
    // printf("\nQSPI%d Initialized..", qspi_num);
    return qspi_base;
  }
  else{
    printf("\nInvalid QSPI instance %d. This SoC supports only QSPI-0 to QSPI-1", qspi_num);
    return -1;
  }
}


void qspi_config_cr(const struct device *dev, int prescale, int pmm, int apms, int toie, int smie, int ftie, int tcie, int teie, int fifo_th, int timeout_en){

   // qspi_instance[qspi_num]->cr = (CR_PRESCALER(prescale) | CR_PMM(pmm) | CR_APMS(apms) | CR_TOIE(toie) |\
      CR_SMIE(smie) | CR_FTIE(ftie) | CR_TCIE(tcie) | CR_TEIE(teie) | CR_FTHRES(fifo_th) | CR_TCEN(timeout_en));
   qspi_instance[qspi_num]->cr = (CR_PRESCALER(prescale) | CR_PMM(pmm) | CR_APMS(apms) | CR_TOIE(toie) |\
      CR_SMIE(smie) | CR_FTIE(ftie) | CR_TCIE(tcie) | CR_TEIE(teie) | CR_FTHRES(fifo_th) | CR_TCEN(timeout_en) | CR_EN(1));
}


void qspi_enable(const struct device *dev){
  uint32_t temp = qspi_instance[qspi_num]->cr;
  temp = temp | CR_EN(1);

  qspi_instance[qspi_num]->cr = temp;
}


void qspi_disable(const struct device *dev){
  printk("Disabled qspi \n");
  uint32_t temp = qspi_instance[qspi_num]->cr;
  printk("Disabled qspi \n");
  uint32_t disable = ~CR_EN(1);
  temp = temp & disable;

  qspi_instance[qspi_num]->cr = temp;
  printk("Disabled qspi successfully \n");

}



void qspi_abort(const struct device *dev){
    uint32_t temp = qspi_instance[qspi_num]->cr;
  temp = temp | CR_ABORT(1);

  qspi_instance[qspi_num]->cr = temp;
}


void qspi_config_dcr(const struct device *dev, int mode_byte, int fmem_size, int clk_mode){
  //qspi_wait_till_not_busy(qspi_num);
  printk("qspi config dcr\n");
  qspi_instance[qspi_num]->dcr = (DCR_MODE_BYTE(mode_byte) | DCR_FSIZE(fmem_size) | DCR_CKMODE(clk_mode));
  printk("qspi config dcr\n");

}


uint32_t qspi_read_status(const struct device *dev){
  printk(" Qspi Read status \n");
  uint32_t status;
  status = qspi_instance[qspi_num]->sr;

  return status;
}


int qspi_check_fifo_full(const struct device *dev){
  uint32_t temp;
  temp = qspi_read_status(qspi_num);
  temp = temp & SR_FLEVEL(FIFO_FULL);
  if (temp == SR_FLEVEL(FIFO_FULL)){
    //#ifdef QSPI_DEBUG
    printf("\nFIFO is full.");
    //#endif
    return 1;
  }
  else{
    #ifdef QSPI_DEBUG
    printf("\nFIFO is not full");
    #endif
    return 0;
  }
}


int qspi_check_fifo_empty(const struct device *dev){
  uint32_t temp;
  temp = qspi_read_status(qspi_num);
  temp = temp & SR_FLEVEL(0x1F);
  if (temp == SR_FLEVEL(0x00)){//TODO: should be 0x00
    #ifdef QSPI_DEBUG
    printf("\nFIFO is empty");
    #endif
    return 1;
  }
  else{
    #ifdef QSPI_DEBUG
    printf("\nFIFO is not empty");
    #endif
    return 0;
  }
}


void qspi_wait_till_tx_complete(const struct device *dev){
  int s=0;
  while (1){
    s = qspi_check_fifo_empty(qspi_num);
    if (s == 1){
      break;
    }
    else{
      continue;
    }
  }
}


void qspi_wait_till_not_busy(const struct device *dev){
  uint32_t temp;

  while(1){
    temp = qspi_read_status(qspi_num);
    temp =  temp & SR_BUSY;
    
    #ifdef QSPI_DEBUG
    printf("%llx",temp);
    printf("\nchecking QSPI%d busy", qspi_num);
    #endif

    if (temp == SR_BUSY){
      printf("\nQSPI%d is busy", qspi_num);
    }
    else {
      break;
    }
  }

  #ifdef QSPI_DEBUG
  printf("\nQSPI%d is not busy", qspi_num);
  #endif

}


void qspi_clear_flags(const struct device *dev, int clr_tof, int clr_smf, int clr_tcf, int clr_tef){
 uint32_t temp = 0;qspi_instance[qspi_num]->fcr;
 if (clr_tof==1){
   temp = temp | FCR_CTOF;
 }
 if (clr_smf==1){
   temp = temp | FCR_CSMF;
 }
 if (clr_tcf==1){
   temp = temp | FCR_CTCF;
 }
 if (clr_tef==1){
   temp = temp | FCR_CTEF;
 }

 qspi_instance[qspi_num]->fcr = temp;

}


int qspi_data_len(const struct device *dev, uint32_t data_len, int wr_rd){

  uint32_t rd_data_len = 0;
  if (wr_rd == WRITE){
    qspi_wait_till_not_busy(qspi_num);
    qspi_instance[qspi_num]->dlr = data_len;
  }
  else if (wr_rd == READ){
    rd_data_len = qspi_instance[qspi_num]->dlr;
    return rd_data_len;
  }
  return 0;
} 


void qspi_config_ccr(const struct device *dev, uint8_t instr, int imode, int admode, int adsize, int abmode, int absize,\
    int dummy_cycles, int dummy_conf, int dmode, int fmode, int sioo, int dummy_bit, int ddr_mode,int mm_mode){

  qspi_wait_till_not_busy(qspi_num);
  qspi_instance[qspi_num]->ccr = (CCR_INSTRUCTION(instr) | CCR_IMODE(imode) | CCR_ADMODE(admode) | CCR_ADSIZE(adsize) |\
      CCR_ABMODE(abmode) | CCR_ABSIZE(absize) | CCR_DCYC(dummy_cycles) | CCR_DUMMY_CONFIRMATION(dummy_conf) |\
      CCR_DMODE(dmode) | CCR_FMODE(fmode) | CCR_SIOO(sioo) | CCR_DUMMY_BIT(dummy_bit) | CCR_DDRM(ddr_mode) | CCR_MM_MODE(mm_mode));

}


int qspi_config_ar(const struct device *dev, uint32_t addr, int fmode){
  if (fmode == 3){
    printf("\nCan't write to AR reg in mem map mode");
    return -1;
  }
  else {
    qspi_wait_till_not_busy(qspi_num);
    qspi_instance[qspi_num]->ar = addr;
    return 0;
  }
}


void qspi_config_abr(const struct device *dev, uint32_t alt_byte){
  qspi_wait_till_not_busy(qspi_num);
  qspi_instance[qspi_num]->abr = alt_byte;
}



void qspi8_wr_dr(const struct device *dev, uint8_t* tx_data, int data_size){
   uint32_t data_len = data_size;
   int no_of_itr = data_size;
   qspi_data_len(qspi_num, data_len, WRITE); 
   for (int i=0; i < no_of_itr; i++){
     /* while(((((qspi_instance[qspi_num]->sr)>>8)& 0x1F)>16)&(i!=0)); */
     /* if(i==16) qspi_instance[qspi_num]->dr.data_8=0x00; */
     /* else */
        qspi_check_fifo_full(qspi_num);
        qspi_instance[qspi_num]->dr.data_8 = *(tx_data++);
    }
}


void qspi16_wr_dr(const struct device *dev, uint16_t tx_data[FIFO_DEPTH_16], int data_size){
  uint32_t data_len = data_size*2;
  int no_of_itr = data_size/2;

  qspi_data_len(qspi_num, data_len, WRITE);
  for (int i=0; i < no_of_itr; i++){
    qspi_check_fifo_full(qspi_num);
    qspi_instance[qspi_num]->dr.data_16 = tx_data[i] ;
  }

}


void qspi32_wr_dr(const struct device *dev, uint32_t tx_data[FIFO_DEPTH_32], int data_size){
  uint32_t data_len = data_size*4;
  int no_of_itr = data_size/4;

  qspi_data_len(qspi_num, data_len, WRITE);
  for (int i=0; i < no_of_itr; i++){
    qspi_check_fifo_full(qspi_num);
    qspi_instance[qspi_num]->dr.data_32 = tx_data[i] ;
  }

}


void qspi_wait_till_rx_fifo_fills(const struct device *dev, uint32_t data_len){
  uint32_t temp;
  while(1){
    temp = qspi_read_status(qspi_num);
    temp = temp & SR_FLEVEL(data_len);
    #ifdef QSPI_DEBUG
    printf("\nQSPI Read Status & SR_FLEVEL %llx", temp);
    #endif
    if (temp == SR_FLEVEL(data_len)){
       break;
    }

  }
}


uint8_t qspi8_rd_dr(const struct device *dev){
  uint8_t rx_data_8=0;
  int temp=0;
  temp = qspi_check_fifo_empty(qspi_num);
  if(temp==0){
    rx_data_8 = qspi_instance[qspi_num]->dr.data_8;
    return rx_data_8;
  }
  else{
    printf("\n Can't read data from empty FIFO..");
    return -1;
  }
}


uint16_t qspi16_rd_dr(const struct device *dev){
  uint16_t rx_data_16=0;
  int temp=0;
  temp = qspi_check_fifo_empty(qspi_num);

  if(temp==0){
    rx_data_16 = qspi_instance[qspi_num]->dr.data_16;
    return rx_data_16;
  }
  else{
    printf("\nCan't read data from empty FIFO..");
    return -1;
  }
}


uint32_t qspi32_rd_dr(const struct device *dev){
  uint8_t rx_data_32=0;
  int temp=0;
  temp = qspi_check_fifo_empty(qspi_num);

  if(temp==0){
    rx_data_32 = qspi_instance[qspi_num]->dr.data_32;
    return rx_data_32;
  }
  else{
    printf("\nCan't read data from empty FIFO..");
    return -1;
  }
}


void qspi_config_psmkr(const struct device *dev, uint32_t status_mask){
  qspi_wait_till_not_busy(qspi_num);
  qspi_instance[qspi_num]->psmkr = status_mask;
}


void qspi_config_psmar(const struct device *dev, uint32_t status_match){
  qspi_wait_till_not_busy(qspi_num);
  qspi_instance[qspi_num]->psmar = status_match;
}


void qspi_config_pir(const struct device *dev, uint32_t poll_interval){
  qspi_wait_till_not_busy(qspi_num);
  qspi_instance[qspi_num]->pir = poll_interval;
}


void qspi_config_lptr(const struct device *dev, uint32_t time_out){
  qspi_wait_till_not_busy(qspi_num);
  qspi_instance[qspi_num]->lptr = time_out;
}


uint8_t qspi_check_transaction_complete (const struct device *dev){
  uint8_t temp = 0;
  temp = qspi_instance[qspi_num]->sr;
  temp = temp & SR_TCF;
  if(temp == SR_TCF){
    printk(" qpsi transaction complete\n");  
    return 1;
  }
  else {
    printk(" qpsi transaction not complete\n");  
    return 0;
  }
}


void qspi_wait_till_transaction_complete (const struct device *dev){
  uint8_t temp = 0;
  while(qspi_check_transaction_complete(dev) == 0);
}



/* -------------------------------------------------------flash driver---------------------------------------------------------*/ 
// #include<qspi_w25q32bv_flash.h>

#define PRESCALE_QSPI 32
//read functions
//FLASH CONSTANTS
#define SR_PROG_STATUS (1<<7)

void fastReadQuad(const struct device *dev ,uint8_t* data,uint32_t address,uint8_t data_length){
    qspi_init(dev);
	qspi_config_dcr(dev,0,0b11111,1);
    qspi_config_cr(dev, PRESCALE_QSPI,0,0,0,0, \
        0,0,0,data_length,0);
    qspi_clear_flags(dev,1, 1, 1, 1);
    qspi_data_len(dev,(uint32_t)data_length,WRITE);
    qspi_config_ccr(dev, SPI_NOR_CMD_QREAD, CCR_IMODE_SINGLE_LINE, CCR_ADMODE_SINGLE_LINE, CCR_ADSIZE_24_BIT,CCR_ABMODE_NIL,0,\
        7,0, CCR_DMODE_FOUR_LINE, CCR_FMODE_INDIRECT_READ, 0, 0, 0, CCR_MM_MODE_XIP);
    qspi_config_ar(dev,address, CCR_FMODE_INDIRECT_READ);
	// if(data_length>10)
	// qspi_wait_till_rx_fifo_fills(dev,10);
	// else
    // waitfor(15);
	qspi_wait_till_rx_fifo_fills(dev,data_length);
	for(uint8_t i=0;i < data_length;i++)
	    *(data ++)=qspi8_rd_dr(dev);
    // for(uint8_t i=0;i < data_length;i++){
    //     data[i]=qspi8_rd_dr(dev);
    //     printf("\t %x", data[i]);
    // }
    // waitfor(1);
	qspi_disable(dev);
}

void fastReadQuad32(const struct device *dev ,uint32_t* data,uint32_t address,uint8_t data_length){
    qspi_init(dev);
	qspi_config_dcr(dev,0,0b11111,1);
    qspi_config_cr(dev, PRESCALE_QSPI,0,0,0,0, \
        0,0,0,data_length,0);
    qspi_clear_flags(dev,1, 1, 1, 1);
    qspi_data_len(dev,(uint32_t)data_length,WRITE);
    qspi_config_ccr(dev, SPI_NOR_CMD_QREAD, CCR_IMODE_SINGLE_LINE, CCR_ADMODE_SINGLE_LINE, CCR_ADSIZE_24_BIT,CCR_ABMODE_NIL,0,\
        7,0, CCR_DMODE_FOUR_LINE, CCR_FMODE_INDIRECT_READ, 0, 0, 0, CCR_MM_MODE_XIP);
    qspi_config_ar(dev,address, CCR_FMODE_INDIRECT_READ);
	// if(data_length>10)
	// qspi_wait_till_rx_fifo_fills(dev,10);
	// else
    // waitfor(15);
	qspi_wait_till_rx_fifo_fills(dev,data_length);
	for(uint8_t i=0;i < data_length;i++)
	    *(data ++)=qspi32_rd_dr(dev);
    // for(uint8_t i=0;i < data_length;i++){
    //     data[i]=qspi8_rd_dr(dev);
    //     printf("\t %x", data[i]);
    // }
    // waitfor(1);
	qspi_disable(dev);
}

void fastReadQuadIO(const struct device *dev ,uint8_t *data,uint32_t address,uint8_t data_length){
    qspi_init(dev);
	qspi_config_dcr(dev,0,0b11111,1);
	qspi_config_cr(dev, PRESCALE_QSPI,0,0,0,0, \
        0,0,0,16,0);
    qspi_clear_flags(dev,1, 1, 1, 1);
	qspi_data_len(dev,(uint32_t)data_length,WRITE);
    qspi_config_ccr(dev, SPI_NOR_CMD_4READ, CCR_IMODE_TWO_LINE, CCR_ADMODE_SINGLE_LINE, CCR_ADSIZE_24_BIT,CCR_ABMODE_FOUR_LINE,CCR_ABSIZE_8_BIT,\
        4,0, CCR_DMODE_FOUR_LINE, CCR_FMODE_INDIRECT_READ, 0, 0, 0, CCR_MM_MODE_XIP);
    qspi_config_abr(dev, 0x20);
	qspi_config_ar(dev,address, CCR_FMODE_INDIRECT_READ);
	// if(data_length>16)
	// qspi_wait_till_rx_fifo_fills(dev,10);
	// else
	qspi_wait_till_rx_fifo_fills(dev,data_length);
	for(uint8_t i=0;i < data_length;i++)
	    *(data ++)=qspi8_rd_dr(dev);
    // for(uint8_t i=0;i < data_length;i++){
    //     data[i]=qspi8_rd_dr(dev);
    //     printf("\t %x", data[i]);
    // }
	qspi_disable(dev);
}

void wait_till_program_or_erase_complete(const struct device *dev )
{
    uint8_t temp = 0;
    while(1)
    {
        // printf("\nInside loop B4");
        temp =  readFlagStatusRegister(dev);
        // printf("\nInside loop AF : %02x",temp);
        temp = temp & SR_PROG_STATUS;
        if(temp == SR_PROG_STATUS)
            break;
    }
}

void inputpageQuad(const struct device *dev ,uint8_t* data,uint32_t address,uint8_t data_length){
    printf("\n Inside input page quad.");
    qspi_init(dev);
	qspi_config_dcr(dev,0,0b11111,1);
    qspi_config_cr(dev, PRESCALE_QSPI,0,0,0,0, \
        0,0,0,data_length,0);
    qspi_clear_flags(dev,1, 1, 1, 1);
    qspi_data_len(dev,(uint32_t)(data_length - 1),WRITE);
    qspi_config_ccr(dev,SPI_NOR_CMD_PP_1_1_4, CCR_IMODE_SINGLE_LINE, CCR_ADMODE_SINGLE_LINE, CCR_ADSIZE_24_BIT,CCR_ABMODE_NIL,0,\
        0,0, CCR_DMODE_FOUR_LINE, CCR_FMODE_INDIRECT_WRITE, 0, 0, 0, CCR_MM_MODE_XIP);
    qspi_config_ar(dev,address, CCR_FMODE_INDIRECT_WRITE);
    qspi8_wr_dr(dev,data,data_length);
    qspi_wait_till_tx_complete(dev);
    // wait_till_program_or_erase_complete(dev);
    qspi_disable(dev);
    printf("\n Completed input page quad!");
}

void sector4KErase(const struct device *dev ,uint32_t address){
    printf("\n Sector 4K erase!");
    qspi_init(dev);
    qspi_config_dcr(dev,0,0b11111,1);
    qspi_config_cr(dev, PRESCALE_QSPI,0,0,0,0, \
        0,0,0,16,0);
    qspi_clear_flags(dev,1, 1, 1, 1);
    qspi_config_ccr(dev, SPI_NOR_CMD_SE, CCR_IMODE_SINGLE_LINE, CCR_ADMODE_SINGLE_LINE,CCR_ADSIZE_24_BIT, CCR_ABMODE_NIL, 0,\
        1, 0, CCR_DMODE_NO_DATA, CCR_FMODE_INDIRECT_WRITE, 0, 0, 0, CCR_MM_MODE_XIP);
    qspi_config_ar(dev,address,CCR_FMODE_INDIRECT_WRITE);
    qspi_wait_till_transaction_complete(dev);
    // wait_till_program_or_erase_complete(dev);
    qspi_disable(dev);
    printf("\n Completed!");
}

void sector32KErase(const struct device *dev ,uint32_t address){
    qspi_init(dev);
	qspi_config_dcr(dev,0,0b11111,1);
    qspi_config_cr(dev, PRESCALE_QSPI,0,0,0,0, \
        0,0,0,16,0);
    qspi_clear_flags(dev,1, 1, 1, 1);
    // qspi_data_len(dev,32000,WRITE);
    qspi_config_ccr(dev, SPI_NOR_CMD_BE_32K, CCR_IMODE_SINGLE_LINE, CCR_ADMODE_SINGLE_LINE,CCR_ADSIZE_24_BIT, CCR_ABMODE_NIL, 0,\
        1, 0, CCR_DMODE_NO_DATA, CCR_FMODE_INDIRECT_WRITE, 0, 0, 0, CCR_MM_MODE_XIP);
    qspi_config_ar(dev,address,CCR_FMODE_INDIRECT_WRITE);
    qspi_wait_till_transaction_complete(dev);
    // wait_till_program_or_erase_complete(dev);
    qspi_disable(dev);
}

void chipErase(const struct device *dev ){
    qspi_init(dev);
	qspi_config_dcr(dev,0,0b11111,1);
    qspi_config_cr(dev, PRESCALE_QSPI,0,0,0,0, \
        0,0,0,16,0);
    qspi_clear_flags(dev,1, 1, 1, 1);
    qspi_config_ccr(dev, SPI_NOR_CMD_BULKE, CCR_IMODE_SINGLE_LINE, CCR_ADMODE_NIL,CCR_ADSIZE_8_BIT, CCR_ABMODE_NIL, 0,\
        1, 0, CCR_DMODE_NO_DATA, CCR_FMODE_INDIRECT_WRITE, 0, 0, 0, CCR_MM_MODE_XIP);
    /* qspi_wait_till_tx_complete(dev); */
    qspi_wait_till_transaction_complete(dev);
    qspi_disable(dev);
}

void writeEnable(const struct device *dev ){
    printf("\n Inside Write Enable!");
    qspi_init(dev);
	qspi_config_dcr(dev,0,0b11111,1);
    qspi_config_cr(dev, PRESCALE_QSPI,0,0,0,0, \
        0,0,0,16,0);
    qspi_clear_flags(dev,1, 1, 1, 1);
    qspi_config_ccr(dev, SPI_NOR_CMD_WREN, CCR_IMODE_SINGLE_LINE, CCR_ADMODE_NIL,CCR_ADSIZE_8_BIT, CCR_ABMODE_NIL, 0,\
        1, 0, CCR_DMODE_NO_DATA, CCR_FMODE_INDIRECT_WRITE, 0, 0, 0, CCR_MM_MODE_XIP);
    /* qspi_wait_till_tx_complete(dev); */
    /* qspi_disable(dev); */
    qspi_wait_till_transaction_complete(dev);
    printf("\n Write Enable completed!");
}

void writeDisable(const struct device *dev ){
    printf("\n Inside Write Disable!");
    qspi_init(dev);
	qspi_config_dcr(dev,0,0b11111,1);
    qspi_config_cr(dev, PRESCALE_QSPI,0,0,0,0, \
        0,0,0,16,0);
    qspi_clear_flags(dev,1, 1, 1, 1);
    qspi_config_ccr(dev, SPI_NOR_CMD_WRDI, CCR_IMODE_SINGLE_LINE, CCR_ADMODE_NIL,CCR_ADSIZE_8_BIT, CCR_ABMODE_NIL, 0,\
        1, 0, CCR_DMODE_NO_DATA, CCR_FMODE_INDIRECT_WRITE, 0, 0, 0, CCR_MM_MODE_XIP);
    qspi_wait_till_transaction_complete(dev);
    qspi_disable(dev);
    printf("\n Write Disable completed!");
}

void suspend(const struct device *dev ){
    qspi_init(dev);
	qspi_config_dcr(dev,0,0b11111,1);
    qspi_config_cr(dev, PRESCALE_QSPI,0,0,0,0, \
        0,0,0,16,0);
    qspi_clear_flags(dev,1, 1, 1, 1);
    qspi_config_ccr(dev, 0x75,CCR_IMODE_SINGLE_LINE, CCR_ADMODE_NIL,CCR_ADSIZE_8_BIT, CCR_ABMODE_NIL, 0,\
        0, 0, CCR_DMODE_NO_DATA, CCR_FMODE_INDIRECT_WRITE, 0, 0, 0, CCR_MM_MODE_XIP);
    qspi_wait_till_tx_complete(dev);
    qspi_disable(dev);
}

void continuosmode_reset(const struct device *dev ){
    qspi_init(dev);
	qspi_config_dcr(dev,0,0b11111,1);
    qspi_config_cr(dev, PRESCALE_QSPI,0,0,0,0, \
        0,0,0,16,0);
    qspi_clear_flags(dev,1, 1, 1, 1);
    qspi_config_ccr(dev, 0xFF,CCR_IMODE_SINGLE_LINE, CCR_ADMODE_NIL,CCR_ADSIZE_8_BIT, CCR_ABMODE_NIL, 0,\
        0, 0, CCR_DMODE_NO_DATA, CCR_FMODE_INDIRECT_WRITE, 0, 0, 0, CCR_MM_MODE_XIP);
    qspi_wait_till_tx_complete(dev);
    qspi_disable(dev);
}


void resume(const struct device *dev ){
    qspi_init(dev);
	qspi_config_dcr(dev,0,0b11111,1);
    qspi_config_cr(dev, PRESCALE_QSPI,0,0,0,0, \
        0,0,0,16,0);
    qspi_clear_flags(dev,1, 1, 1, 1);
    qspi_config_ccr(dev, 0x30,CCR_IMODE_SINGLE_LINE, CCR_ADMODE_NIL,CCR_ADSIZE_8_BIT, CCR_ABMODE_NIL, 0,\
        0, 0, CCR_DMODE_NO_DATA, CCR_FMODE_INDIRECT_WRITE, 0, 0, 0, CCR_MM_MODE_XIP);
    qspi_wait_till_tx_complete(dev);
    qspi_disable(dev);
}
void power_down(const struct device *dev ){
    qspi_init(dev);
	qspi_config_dcr(dev,0,0b11111,1);
    qspi_config_cr(dev, PRESCALE_QSPI,0,0,0,0, \
        0,0,0,16,0);
    qspi_clear_flags(dev,1, 1, 1, 1);
    qspi_config_ccr(dev, SPI_NOR_CMD_DPD, CCR_IMODE_SINGLE_LINE, CCR_ADMODE_NIL,CCR_ADSIZE_8_BIT, CCR_ABMODE_NIL, 0,\
        0, 0, CCR_DMODE_NO_DATA, CCR_FMODE_INDIRECT_WRITE, 0, 0, 0, CCR_MM_MODE_XIP);
    qspi_wait_till_tx_complete(dev);
    qspi_disable(dev);
}

void release_power_down(const struct device *dev ){
    qspi_init(dev);
	qspi_config_dcr(dev,0,0b11111,1);
    qspi_config_cr(dev, PRESCALE_QSPI,0,0,0,0, \
        0,0,0,16,0);
    qspi_clear_flags(dev,1, 1, 1, 1);
    qspi_config_ccr(dev, SPI_NOR_CMD_RDPD, CCR_IMODE_SINGLE_LINE, CCR_ADMODE_NIL,CCR_ADSIZE_8_BIT, CCR_ABMODE_NIL, 0,\
        0, 0, CCR_DMODE_NO_DATA, CCR_FMODE_INDIRECT_WRITE, 0, 0, 0, CCR_MM_MODE_XIP);
    qspi_wait_till_tx_complete(dev);
    qspi_disable(dev);
}
uint8_t readStatusRegister(const struct device *dev ){
    printk("Read status Register\n");
    // qspi_disable(dev);
    qspi_init(dev);
	qspi_config_dcr(dev,0,27,1);
    qspi_config_cr(dev, PRESCALE_QSPI,0,0,0,0, \
        0,0,0,1,0);
	qspi_clear_flags(dev,1, 1, 1, 1);
    qspi_data_len(dev,1,WRITE);
	qspi_config_ccr(dev, SPI_NOR_CMD_RDSR, CCR_IMODE_SINGLE_LINE, CCR_ADMODE_NIL, CCR_ADSIZE_24_BIT,CCR_ABMODE_NIL,0,\
        0,0, CCR_DMODE_SINGLE_LINE, CCR_FMODE_INDIRECT_READ, 0, 0, 0, CCR_MM_MODE_XIP);
	qspi_wait_till_rx_fifo_fills(dev,1);
	uint8_t data=qspi8_rd_dr(dev);
	qspi_disable(dev);
	return data;
}
uint8_t readFlagStatusRegister(const struct device *dev ){
    qspi_init(dev);
	qspi_config_dcr(dev,0,0b11111,1);
    qspi_config_cr(dev, PRESCALE_QSPI,0,0,0,0, \
        0,0,0,1,0);
	qspi_clear_flags(dev,1, 1, 1, 1);
    qspi_data_len(dev,1,WRITE);
	qspi_config_ccr(dev,0x70,CCR_IMODE_SINGLE_LINE, CCR_ADMODE_NIL, CCR_ADSIZE_24_BIT,CCR_ABMODE_NIL,0,\
        0,0, CCR_DMODE_SINGLE_LINE, CCR_FMODE_INDIRECT_READ, 0, 0, 0, CCR_MM_MODE_XIP);
	// qspi_config_ar(dev,address, CCR_FMODE_INDIRECT_READ);
	qspi_wait_till_rx_fifo_fills(dev,1);
	uint8_t data=qspi8_rd_dr(dev);
	qspi_disable(dev);
	return data;
}

void writeEnableStatusRegister(const struct device *dev ){
    qspi_init(dev);
	qspi_config_dcr(dev,0,0b11111,1);
    qspi_config_cr(dev, PRESCALE_QSPI,0,0,0,0, \
        0,0,0,16,0);
    qspi_clear_flags(dev,1, 1, 1, 1);
    qspi_data_len(dev,1,WRITE);
    qspi_config_ccr(dev, 0x50,CCR_IMODE_SINGLE_LINE, CCR_ADMODE_NIL,CCR_ADSIZE_8_BIT, CCR_ABMODE_NIL, 0,\
        1, 0, CCR_DMODE_NO_DATA, CCR_FMODE_INDIRECT_WRITE, 0, 0, 0, CCR_MM_MODE_XIP);
    qspi_wait_till_tx_complete(dev);
    qspi_disable(dev);
}

void writeStatusRegister(const struct device *dev ,uint8_t* statusData){
    qspi_init(dev);
	qspi_config_dcr(dev,0,0b11111,1);
    qspi_config_cr(dev, PRESCALE_QSPI,0,0,0,0, \
        0,0,0,1,0);
	qspi_clear_flags(dev,1, 1, 1, 1);
    qspi_data_len(dev,1,WRITE);
	qspi_config_ccr(dev,0x01,CCR_IMODE_SINGLE_LINE, CCR_ADMODE_NIL, CCR_ADSIZE_8_BIT,CCR_ABMODE_NIL,0,\
        1,0, CCR_DMODE_SINGLE_LINE, CCR_FMODE_INDIRECT_WRITE, 0, 0, 0, CCR_MM_MODE_XIP);
    
    // Define the address of the status register you want to write to.
    // This address depends on your specific flash memory device.
    uint32_t statusRegAddress = 0x01;

    // Send the write command and address to the flash memory.
    qspi_config_ar(dev, statusRegAddress, CCR_FMODE_INDIRECT_WRITE);

    // Write the status data to the flash memory.
    qspi8_wr_dr(dev, statusData, 1);

	// qspi_config_ar(dev,address, CCR_FMODE_INDIRECT_READ);
	qspi_wait_till_tx_complete(dev);
    qspi_disable(dev);
    printf("\n Completed Writing Status Register!\n");
}

uint8_t readGlobalFreezeBit(const struct device *dev ){
    qspi_init(dev);
	qspi_config_dcr(dev,0,0b11111,1);
    qspi_config_cr(dev, PRESCALE_QSPI,0,0,0,0, \
        0,0,0,1,0);
	qspi_clear_flags(dev,1, 1, 1, 1);
    qspi_data_len(dev,1,WRITE);
	qspi_config_ccr(dev,0xA7,CCR_IMODE_SINGLE_LINE, CCR_ADMODE_NIL, CCR_ADSIZE_24_BIT,CCR_ABMODE_NIL,0,\
        1,0, CCR_DMODE_SINGLE_LINE, CCR_FMODE_INDIRECT_READ, 0, 0, 0, CCR_MM_MODE_XIP);
	// qspi_config_ar(dev,address, CCR_FMODE_INDIRECT_READ);
	qspi_wait_till_rx_fifo_fills(dev,1);
	uint8_t data=qspi8_rd_dr(dev);
	qspi_disable(dev);
	return data;
}

void writeGlobalFreezeBit(const struct device *dev ){
    qspi_init(dev);
	qspi_config_dcr(dev,0,0b11111,1);
    qspi_config_cr(dev, PRESCALE_QSPI,0,0,0,0, \
        0,0,0,1,0);
	qspi_clear_flags(dev,1, 1, 1, 1);
    qspi_data_len(dev,1,WRITE);
	qspi_config_ccr(dev,0xA6,CCR_IMODE_SINGLE_LINE, CCR_ADMODE_NIL, CCR_ADSIZE_24_BIT,CCR_ABMODE_NIL,0,\
        1,0, CCR_DMODE_SINGLE_LINE, CCR_FMODE_INDIRECT_WRITE, 0, 0, 0, CCR_MM_MODE_XIP);
    qspi_wait_till_tx_complete(dev);
	qspi_disable(dev);
}

uint8_t readFlashSFDP(const struct device *dev ,uint32_t address){
    qspi_init(dev);
	qspi_config_dcr(dev,0,0b11111,1);
    qspi_config_cr(dev, PRESCALE_QSPI,0,0,0,0, \
        0,0,0,1,0);
	qspi_clear_flags(dev,1, 1, 1, 1);
    // qspi_data_len(dev,1,WRITE);
	qspi_config_ccr(dev,0x5A,CCR_IMODE_SINGLE_LINE, CCR_ADMODE_SINGLE_LINE, CCR_ADSIZE_24_BIT,CCR_ABMODE_NIL,0,\
        7,0, CCR_DMODE_SINGLE_LINE, CCR_FMODE_INDIRECT_READ, 0, 0, 0, CCR_MM_MODE_XIP);
	qspi_config_ar(dev,address, CCR_FMODE_INDIRECT_READ);
	qspi_wait_till_rx_fifo_fills(dev,1);
	uint8_t data=qspi8_rd_dr(dev);
	qspi_disable(dev);
	return data;
}

uint8_t readNVCR(const struct device *dev , uint8_t* data){
    qspi_init(dev);
    qspi_config_dcr(dev,0,0b11111,1);
    qspi_config_cr(dev, PRESCALE_QSPI,0,0,0,0, \
        0,0,0,1,0);
	qspi_clear_flags(dev,1, 1, 1, 1);
    // qspi_data_len(dev,1,WRITE);
	qspi_config_ccr(dev,0xB5,CCR_IMODE_SINGLE_LINE, CCR_ADMODE_NIL, CCR_ADSIZE_24_BIT,CCR_ABMODE_NIL,0,\
        0,0, CCR_DMODE_SINGLE_LINE, CCR_FMODE_INDIRECT_READ, 0, 0, 0, CCR_MM_MODE_XIP);
	// qspi_config_ar(dev,address, CCR_FMODE_INDIRECT_READ);
	// qspi_wait_till_rx_fifo_fills(dev,1);
	for(uint8_t i=0;i < 2;i++)
	    *(data ++)=qspi8_rd_dr(dev);
	qspi_disable(dev);
	return data;
}

void writeNVCR(const struct device *dev , uint8_t* data){
    qspi_init(dev);
	qspi_config_dcr(dev,0,0b11111,1);
    qspi_config_cr(dev, PRESCALE_QSPI,0,0,0,0, \
        0,0,0,1,0);
	qspi_clear_flags(dev,1, 1, 1, 1);
    // qspi_data_len(dev,1,WRITE);
	qspi_config_ccr(dev,0xB1,CCR_IMODE_SINGLE_LINE, CCR_ADMODE_NIL, CCR_ADSIZE_24_BIT,CCR_ABMODE_NIL,0,\
        0,0, CCR_DMODE_SINGLE_LINE, CCR_FMODE_INDIRECT_WRITE, 0, 0, 0, CCR_MM_MODE_XIP);
    qspi8_wr_dr(dev, data, 16);
    qspi_wait_till_tx_complete(dev);
	qspi_disable(dev);
}

// Number of bytes in flash memory = 2 ^ [flash_size + 1]
void qspi_xip_init(const struct device *dev , int flash_size){
    /* set non volatile memory of Flash to XIP mode */
    qspi_init(dev);
    int mode_byte = 0; //set 16 to exit XIP
    qspi_config_dcr(dev, mode_byte, flash_size, 1);
    qspi_config_cr(dev, PRESCALE_QSPI,0,0,0,0,0,0,0,15,0);
    qspi_clear_flags(dev,1, 1, 1, 1);
    // qspi_data_len(dev,1,WRITE);
	qspi_config_ccr(dev,0x03,CCR_IMODE_SINGLE_LINE, CCR_ADMODE_SINGLE_LINE, CCR_ADSIZE_24_BIT,CCR_ABMODE_NIL,0,\
        0,0, CCR_DMODE_SINGLE_LINE, CCR_FMODE_MMM, 0, 0, 0, CCR_MM_MODE_XIP);
    // qspi_wait_till_tx_complete(dev);
}

void qspi_ram_init(const struct device *dev , int flash_size){
    /* set non volatile memory of Flash to XIP mode */
    qspi_init(dev);
    int mode_byte = 0; //set 16 to exit XIP
    qspi_config_dcr(dev, mode_byte, flash_size, 1);
    qspi_config_cr(dev, PRESCALE_QSPI,0,0,0,0,0,0,0,3,0);
    qspi_clear_flags(dev,1, 1, 1, 1);
    qspi_data_len(dev,4,WRITE);
	qspi_config_ccr(dev,0x03,CCR_IMODE_SINGLE_LINE, CCR_ADMODE_SINGLE_LINE, CCR_ADSIZE_24_BIT,CCR_ABMODE_NIL,0,\
        0,0, CCR_DMODE_SINGLE_LINE, CCR_FMODE_MMM, 0, 0, 0, CCR_MM_MODE_RAM);
    // qspi_wait_till_tx_complete(dev);
}


/*------------------------------------------------------API Functions------------------------------------------------------------*/

int flash_shakti_erase(const struct device *dev, off_t addr, size_t size)
{
    printk("flash shakti erase\n");
    uint8_t sr,fsr;
    // uint8_t sfdp[304];

    // for(int i = 0; i<304; i++)
    // {   
    //     sfdp[i] = readFlashSFDP(qspi_num, i);
    // }
 
    // printf("\n BA \t Value\n");
    // for(int i = 0; i<304; i++){
    //     printf("%03x \t %04x \n", i, sfdp[i]);
    // }

    sr = readStatusRegister(dev);
    printf("\nStatus Register MN: %x \n",sr);
    fsr = readFlagStatusRegister(dev);
    printf("Flag Status Register MN: %x \n",fsr);
    
    writeEnable(dev);
    sr = readStatusRegister(dev);
    printf("\nStatus Register : %x \n",sr);
    fsr = readFlagStatusRegister(dev);
    printf("Flag Status Register : %x \n",fsr);
    if(size == FLASH_4K_ERASE){
      printk("4k erase\n");
      sector4KErase(dev,addr);
    }
    if(size == FLASH_32K_ERASE){
      printk("32k erase\n");
      sector32KErase(dev,addr); 
    }
    if(size ==  FLASH_CHIP_ERASE){
      printk("chip erase\n");
      chipErase(dev);
    }
    sr = readStatusRegister(dev);
    printf("Status Register : %x \n",sr);
    // fsr = readFlagStatusRegister(qspi_num);
    // printf("Flag Status Register : %x \n",fsr);

    uint8_t temp = 0;
    while(1)
    {
        // printf("\nInside loop B4");
        temp =  readFlagStatusRegister(dev);
        // printf("\nInside loop AF : %02x",temp);
        temp = temp & (1<<7);
        if(temp == (1<<7))
            break;
    }
    writeDisable(dev);
    return 0;
}


int flash_shakti_write(const struct device *dev, off_t addr, 
                        const void *src, size_t size)
{
    // uint8_t sha_text[] = {0x67, 0xd9, 0x0d, 0x71, 0xfe, 0x2b, 0x25, 0x66, 0xb7, 0xa2, 0xd3, 0x0e, 0xf6, 0x6c, 0xad, 0x7f, 0x7c, 0x9b, 0x79, 0xcd, 0xdf, 0xe7, 0x8d, 0x60, 0xbd, 0x72, 0x8e, 0x98, 0x49, 0x3e, 0xa0, 0x10};
    // uint8_t data[]={0x52, 0x4a, 0x6b, 0x45, 0x8a, 0x42, 0xaa, 0x12, 0x6f, 0x6b, 0x45, 0x8a, 0x42, 0xaa, 0x12, 0x8b};
	
	// uint8_t data[]={0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
	  //  uint8_t data[]={0x18, 0x6f, 0x96, 0xac, 0x04, 0xbb, 0xef, 0x54, 0x4b, 0xad, 0xcc, 0x1e, 0xe8, 0x00, 0xec, 0x62};
	
	// uint8_t rsa_mod1[] =  {0xa7, 0x09, 0xe2, 0xf8, 0x4a, 0xc0, 0xe2, 0x1e, 0xb0, 0xca, 0xa0, 0x18, 0xcf, 0x7f, 0x69, 0x7f, 0x77, 0x4e, 0x96, 0xf8, 0x11, 0x5f, 0xc2, 0x35, 0x9e, 0x9c, 0xf6, 0x0b, 0x1d, 0xd8, 0xd4, 0x04};
	// uint8_t rsa_mod15[] = {0x8d, 0x97, 0x4c, 0xdf, 0x84, 0x22, 0xbe, 0xf6, 0xbe, 0x3c, 0x16, 0x2b, 0x04, 0xb9, 0x16, 0xf7, 0xea, 0x21, 0x33, 0xf0, 0xe3, 0xe4, 0xe0, 0xee, 0xe1, 0x64, 0x85, 0x9b, 0xd9, 0xc1, 0xe0, 0xef};
	// uint8_t rsa_mod2[] =  {0x03, 0x57, 0xc1, 0x42, 0xf4, 0xf6, 0x33, 0xb4, 0xad, 0xd4, 0xaa, 0xb8, 0x6c, 0x8f, 0x88, 0x95, 0xcd, 0x33, 0xfb, 0xf4, 0xe0, 0x24, 0xd9, 0xa3, 0xad, 0x6b, 0xe6, 0x26, 0x75, 0x70, 0xb4, 0xa7};
	// uint8_t rsa_mod25[] = {0x2d, 0x2c, 0x34, 0x35, 0x4e, 0x01, 0x39, 0xe7, 0x4a, 0xda, 0x66, 0x5a, 0x16, 0xa2, 0x61, 0x14, 0x90, 0xde, 0xbb, 0x8e, 0x13, 0x1a, 0x6c, 0xff, 0xc7, 0xef, 0x25, 0xe7, 0x42, 0x40, 0x80, 0x3d};
	// uint8_t rsa_mod3[] =  {0xd7, 0x1a, 0x4f, 0xcd, 0x95, 0x3c, 0x98, 0x81, 0x11, 0xb0, 0xaa, 0x9b, 0xbc, 0x4c, 0x57, 0x02, 0x4f, 0xc5, 0xe8, 0xc4, 0x46, 0x2a, 0xd9, 0x04, 0x9c, 0x7f, 0x1a, 0xbe, 0xd8, 0x59, 0xc6, 0x34};
	// uint8_t rsa_mod35[] = {0x55, 0xfa, 0x6d, 0x58, 0xb5, 0xcc, 0x34, 0xa3, 0xd3, 0x20, 0x6f, 0xf7, 0x4b, 0x9e, 0x96, 0xc3, 0x36, 0xdb, 0xac, 0xf0, 0xcd, 0xd1, 0x8e, 0xd0, 0xc6, 0x67, 0x96, 0xce, 0x00, 0xab, 0x07, 0xf3};
	// uint8_t rsa_mod4[] =  {0x6b, 0x24, 0xcb, 0xe3, 0x34, 0x25, 0x23, 0xfd, 0x82, 0x15, 0xa8, 0xe7, 0x7f, 0x89, 0xe8, 0x6a, 0x08, 0xdb, 0x91, 0x1f, 0x23, 0x74, 0x59, 0x38, 0x8d, 0xee, 0x64, 0x2d, 0xae, 0x7c, 0xb2, 0x64};
	// uint8_t rsa_mod45[] = {0x4a, 0x03, 0xe7, 0x1e, 0xd5, 0xc6, 0xfa, 0x50, 0x77, 0xcf, 0x40, 0x90, 0xfa, 0xfa, 0x55, 0x60, 0x48, 0xb5, 0x36, 0xb8, 0x79, 0xa8, 0x8f, 0x62, 0x86, 0x98, 0xf0, 0xc7, 0xb4, 0x20, 0xc4, 0xb7};

	// uint8_t *test_msg = &data;
	// uint8_t *test_msg = &sha_text;
	// uint8_t *test_msg = &rsa_mod1;

	// uint32_t address=0x00000400;
	// uint8_t data_length=16;
  uint8_t *data_ptr = (uint8_t *)src;

	qspi_init(dev);

	chipErase(dev);

	writeEnable(dev);

	inputpageQuad(dev, src, addr, size);

	printf("\n Data in addr 0x%x is: %x %x %x %x", addr, data_ptr[0], data_ptr[1], data_ptr[2], data_ptr[3]); //*src, *(src + 1), *(src + 2));

	printf("\n Write Done Successfully!");
	
	writeDisable(dev);
  return 0;
}


int flash_shakti_read(const struct device *dev, off_t addr, 
                        void *dest, size_t size)
{
    // uint32_t address=0x00000400;
    // uint8_t data_length=16;
    // uint8_t data_read[data_length];

    qspi_init(dev);

    // writeEnable(qspi_num);

    // chipErase(qspi_num);

    // writeDisable(qspi_num);

    uint8_t NVCR[2];
    readNVCR(dev, NVCR);
    printf("\n NVCR Value is : ");
    for(int i=0; i<2; i++)
        printf("%02x", NVCR[i]);
    
    // uint8_t nvcr[] = {0xf1, 0xff};
    // writeNVCR(qspi_num, nvcr);

    // readNVCR(qspi_num, NVCR);
    // printf("\n NVCR Value is : ");
    // for(int i=0; i<2; i++)
    //     printf("%02x", NVCR[i]);
    uint8_t *data_buffer = (uint8_t *)dest;

    fastReadQuad(dev, dest, addr, size);
    printf("\n Data Read from addr 0x%x  is: %x %x %x %x ", addr, data_buffer[0], data_buffer[1], data_buffer[2], data_buffer[3]);
    // for (int i=0; i < size; i++){
	  //   printf("%02x ", data_read[i]);
    // }

    // addr=0x00000010;
    // fastReadQuad(qspi_num, data_read, addr, data_length);
    // printf("\n Data Read from addr 0x%x is: 0x ", addr);
    // for (int i=0; i < data_length; i++){
	//     printf("%02x ", data_read[i]);
    // }

    // addr=0x00000050;
    // fastReadQuad(qspi_num, data_read, addr, data_length);
    // printf("\n Data Read from addr 0x%x is: 0x ", addr);
    // for (int i=0; i < data_length; i++){
	//     printf("%02x ", data_read[i]);
    // }

    // addr=0x000000a0;
    // fastReadQuad(qspi_num, data_read, addr, data_length);
    // printf("\n Data Read from addr 0x%x is: 0x ", addr);
    // for (int i=0; i < data_length; i++){
	//     printf("%02x ", data_read[i]);
    // }

    // addr=0x000000b0;
    // fastReadQuad(qspi_num, data_read, addr, data_length);
    // printf("\n Data Read from addr 0x%x is: 0x ", addr);
    // for (int i=0; i < data_length; i++){
	//     printf("%02x ", data_read[i]);
    // }
    printf("\n Read Done Successfully!");
    return 0;
}


static const struct flash_driver_api flash_shakti_api = {
    .erase = flash_shakti_erase,
    .write = flash_shakti_write,
    .read  = flash_shakti_read,
};

#define FLASH_SHAKTI_QSPI_INIT(n) \
  static struct flash_shakti_qspi_data flash_shakti_qspi_data_##n; \
  static const struct flash_shakti_qspi_config \
    flash_shakti_qspi_config_##n = { \
      .parameters = { \
        .write_block_size = 1, \
        .erase_value = 0xff \
      } \
    }; \
  DEVICE_DT_INST_DEFINE(n,					\
			&qspi_init,				\
			NULL,						\
			&flash_shakti_qspi_data_##n,			\
			&flash_shakti_qspi_config_##n,			\
			POST_KERNEL,					\
			CONFIG_FLASH_INIT_PRIORITY,		\
			&flash_shakti_api);	
    
DT_INST_FOREACH_STATUS_OKAY(FLASH_SHAKTI_QSPI_INIT)