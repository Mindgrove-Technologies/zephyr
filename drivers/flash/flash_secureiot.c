#include <zephyr/drivers/flash/flash_secureiot.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/device.h>
#include<zephyr/kernel.h>

#define DT_DRV_COMPAT secureiot_flash_controller

qspi_msg flash_msg={.PRESCALER=6,.CLK_MODE=0,.FMEM_SIZE = 27,.FTIE = 0,.TCEN=0,.TEIE=0,.TOIE=0,.SMIE = 0,.APMS= 0,.PMM=0};

volatile uint32_t qspinum;

uint32_t QSPI_Transaction(uint32_t instance_number,qspi_msg *msg){
    if(instance_number>1)
      return 1;
    if(msg->length>16)
      return 1;
    QUADSPI_Reg(instance_number)->CR=(CR_PRESCALER(msg->PRESCALER) |CR_PMM(msg->PMM) | CR_APMS(msg->APMS) | CR_TOIE(msg->TOIE) |\
      CR_SMIE(msg->SMIE) | CR_FTIE(msg->FTIE) | CR_TCIE(msg->TCIE) | CR_TEIE(msg->TEIE) | CR_TCEN(msg->TOIE) | CR_EN(1));
    QUADSPI_Reg(instance_number)->DCR = (DCR_FSIZE(msg->FMEM_SIZE) | DCR_CKMODE(msg->CLK_MODE));    
    uint32_t temp;
    do{
        temp = QUADSPI_Reg(instance_number)->SR;
        temp &= SR_BUSY;
    }while(temp==SR_BUSY);//check for busy status
    QUADSPI_Reg(instance_number)->FCR =(FCR_CTOF|FCR_CSMF|FCR_CTCF|FCR_CTEF);//clear flags
    QUADSPI_Reg(instance_number)->DLR = msg->length;
    temp = (CCR_INSTRUCTION(msg->instruction) | CCR_IMODE(msg->instruction_mode) | CCR_ADMODE(msg->address_mode) | CCR_ADSIZE(msg->address_size) |\
    CCR_ABMODE(msg->alternate_byte_mode) | CCR_ABSIZE(msg->sioo) | CCR_DCYC(msg->dummy_cycles) | CCR_DUMMY_CONFIRMATION(msg->dummy_mode) |\
    CCR_DMODE(msg->data_mode) | CCR_FMODE(msg->functional_mode) | CCR_SIOO(msg->sioo) | CCR_DUMMY_BIT(msg->dummy_bit) | CCR_MM_MODE(msg->mm_mode));
    QUADSPI_Reg(instance_number)->CCR = temp;
    QUADSPI_Reg(instance_number)->AR = msg->address;
    QUADSPI_Reg(instance_number)->ABR = msg->alternate_byte;

    uint8_t i = 0;
    uint32_t status_reg;
    if(msg->functional_mode == CCR_FMODE_INDIRECT_WRITE && msg->length!= 0)
    {
      QUADSPI_Reg(instance_number)->CR&= ~(CR_FTHRES(15));
      QUADSPI_Reg(instance_number)->CR|= CR_FTHRES(0);
      while(1){
        status_reg = QUADSPI_Reg(instance_number)->SR;
        status_reg &= SR_FTF;
        if(status_reg){
          QUADSPI_Reg(instance_number)->DR.data_8 = msg->data_buffer[i];
          i++;
            if(i == msg->length)
              break;
        }
      }
      do{
          temp = QUADSPI_Reg(instance_number)->SR;
          temp&=SR_FLEVEL(FIFO_EMPTY);
        }while(temp != 0);
        
    }
    else if(msg->functional_mode == CCR_FMODE_INDIRECT_READ)
    {
      QUADSPI_Reg(instance_number)->CR&= ~(CR_FTHRES(15));
      QUADSPI_Reg(instance_number)->CR|= CR_FTHRES(0);
      while(1){
        status_reg = QUADSPI_Reg(instance_number)->SR;
        status_reg &= SR_FTF;
        if(status_reg){
          msg->data_buffer[i] = QUADSPI_Reg(instance_number)->DR.data_8;
          i++;
            if(i == msg->length)
              break;
        }
      }
    }
    if(msg->functional_mode == CCR_FMODE_MMM && msg->mm_mode == CCR_MM_MODE_RAM){
      QUADSPI_Reg(instance_number)->CR&=~(CR_FTHRES(15));
      QUADSPI_Reg(instance_number)->CR|=(CR_FTHRES(msg->fthresh));
      return 0;
    }
    if(msg->functional_mode == CCR_FMODE_MMM && msg->mm_mode == CCR_MM_MODE_XIP){
      QUADSPI_Reg(instance_number)->CR&=~(CR_FTHRES(15));
      return 0;
    }

    if(msg->data_mode == CCR_DMODE_NO_DATA)
    {
      do{
      temp = QUADSPI_Reg(instance_number)->SR;
      temp &= SR_TCF;
    }while(temp == 0);
    }
    k_busy_wait(100);   
    QUADSPI_Reg(instance_number)->CR&= ~CR_EN(1);
    return 0;
}

uint8_t readStatusRegister1(const struct device *dev){// status reg 1
    uint8_t data;
    flash_msg.address_mode = CCR_ADMODE_NIL;
    flash_msg.address_size = CCR_ADSIZE_24_BIT;
    flash_msg.instruction = 0x05;
    flash_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    flash_msg.data_mode = CCR_DMODE_SINGLE_LINE;
    flash_msg.data_buffer = &data;
    flash_msg.functional_mode = CCR_FMODE_INDIRECT_READ;
    flash_msg.dummy_mode = 0;
    flash_msg.dummy_cycles = 0;
    flash_msg.dummy_bit = 0;
    flash_msg.mm_mode = CCR_MM_MODE_XIP;
    flash_msg.alternate_byte_mode = CCR_ABMODE_NIL;
    flash_msg.length = 1;
    QSPI_Transaction(qspinum,&flash_msg);
	return data;
}

uint32_t fastReadQuad(const struct device *dev,uint8_t* data,uint32_t address,uint8_t data_length){
    flash_msg.address = address;
    flash_msg.address_mode = CCR_ADMODE_SINGLE_LINE;
    flash_msg.address_size = CCR_ADSIZE_24_BIT;
    flash_msg.instruction = 0x6B;
    flash_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    flash_msg.data_mode = CCR_DMODE_FOUR_LINE;
    flash_msg.data_buffer = data;
    flash_msg.functional_mode = CCR_FMODE_INDIRECT_READ;
    flash_msg.dummy_mode = 0;
    flash_msg.dummy_cycles = 7;
    flash_msg.dummy_bit = 0;
    flash_msg.mm_mode = CCR_MM_MODE_XIP;
    flash_msg.alternate_byte_mode = CCR_ABMODE_NIL;
    flash_msg.length = data_length;
    return QSPI_Transaction(qspinum,&flash_msg);
}

uint32_t fastReadQuadIO(const struct device *dev,uint8_t *data,uint32_t address,uint8_t data_length){
    flash_msg.address = address;
    flash_msg.address_mode = CCR_ADMODE_SINGLE_LINE;
    flash_msg.address_size = CCR_ADSIZE_24_BIT;
    flash_msg.instruction = 0xEB;
    flash_msg.instruction_mode = CCR_IMODE_TWO_LINE;
    flash_msg.data_mode = CCR_DMODE_FOUR_LINE;
    flash_msg.data_buffer = data;
    flash_msg.functional_mode = CCR_FMODE_INDIRECT_READ;
    flash_msg.dummy_mode = 0;
    flash_msg.dummy_cycles = 4;
    flash_msg.dummy_bit = 0;
    flash_msg.mm_mode = CCR_MM_MODE_XIP;
    flash_msg.alternate_byte_mode = CCR_ABMODE_FOUR_LINE;
    flash_msg.alternate_byte = 0x20;
    flash_msg.length = data_length;
    return QSPI_Transaction(qspinum,&flash_msg);   
}

uint32_t fastReadSingle(const struct device *dev,uint8_t *data,uint32_t address,uint8_t data_length){
    flash_msg.address = address;
    flash_msg.address_mode = CCR_ADMODE_SINGLE_LINE;
    flash_msg.address_size = CCR_ADSIZE_24_BIT;
    flash_msg.instruction = 0x03;
    flash_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    flash_msg.data_mode = CCR_DMODE_SINGLE_LINE;
    flash_msg.data_buffer = data;
    flash_msg.functional_mode = CCR_FMODE_INDIRECT_READ;
    flash_msg.dummy_mode = 0;
    flash_msg.dummy_cycles = 0;
    flash_msg.dummy_bit = 0;
    flash_msg.mm_mode = CCR_MM_MODE_XIP;
    flash_msg.alternate_byte_mode = CCR_ABMODE_NIL;
    flash_msg.length = data_length;
    return QSPI_Transaction(qspinum,&flash_msg);
}

uint32_t inputpageQuad(const struct device *dev,uint8_t* data,uint32_t address,uint8_t data_length){
    flash_msg.address = address;
    flash_msg.address_mode = CCR_ADMODE_SINGLE_LINE;
    flash_msg.address_size = CCR_ADSIZE_24_BIT;
    flash_msg.instruction = 0x32;
    flash_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    flash_msg.data_mode = CCR_DMODE_FOUR_LINE;
    flash_msg.data_buffer = data;
    flash_msg.functional_mode = CCR_FMODE_INDIRECT_WRITE;
    flash_msg.dummy_mode = 0;
    flash_msg.dummy_cycles = 0;
    flash_msg.dummy_bit = 0;
    flash_msg.mm_mode = CCR_MM_MODE_XIP;
    flash_msg.alternate_byte_mode = CCR_ABMODE_NIL;
    flash_msg.length = data_length;
    return QSPI_Transaction(qspinum,&flash_msg);    
}

uint32_t inputpageSingle(const struct device *dev,uint8_t* data,uint32_t address,uint8_t data_length){
    flash_msg.address = address;
    flash_msg.address_mode = CCR_ADMODE_SINGLE_LINE;
    flash_msg.address_size = CCR_ADSIZE_24_BIT;
    flash_msg.instruction = 0x02;
    flash_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    flash_msg.data_mode = CCR_DMODE_SINGLE_LINE;
    flash_msg.data_buffer = data;
    flash_msg.functional_mode = CCR_FMODE_INDIRECT_WRITE;
    flash_msg.dummy_mode = 0;
    flash_msg.dummy_cycles = 0;
    flash_msg.dummy_bit = 0;
    flash_msg.mm_mode = CCR_MM_MODE_XIP;
    flash_msg.alternate_byte_mode = CCR_ABMODE_NIL;
    flash_msg.length = data_length;
    return QSPI_Transaction(qspinum,&flash_msg);   
}

uint32_t sector4KErase(const struct device *dev,uint32_t address){
    flash_msg.address = address;
    flash_msg.address_mode = CCR_ADMODE_SINGLE_LINE;
    flash_msg.address_size = CCR_ADSIZE_24_BIT;
    flash_msg.instruction = 0x20;
    flash_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    flash_msg.data_mode = CCR_DMODE_NO_DATA;
    flash_msg.functional_mode = CCR_FMODE_INDIRECT_WRITE;
    flash_msg.dummy_mode = 0;
    flash_msg.dummy_cycles = 1;
    flash_msg.dummy_bit = 0;
    flash_msg.mm_mode = CCR_MM_MODE_XIP;
    flash_msg.alternate_byte_mode = CCR_ABMODE_NIL;
    QSPI_Transaction(qspinum,&flash_msg);
    uint8_t temp;
    while(1){
        temp = readStatusRegister1(dev);
        temp = temp & 0x01;
        if(temp != 0x01)
        break;
    }
    return 0;    
}

uint32_t sector32KErase(const struct device *dev,uint32_t address){
    flash_msg.address = address;
    flash_msg.address_mode = CCR_ADMODE_SINGLE_LINE;
    flash_msg.address_size = CCR_ADSIZE_24_BIT;
    flash_msg.instruction = 0x52;
    flash_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    flash_msg.data_mode = CCR_DMODE_NO_DATA;
    flash_msg.functional_mode = CCR_FMODE_INDIRECT_WRITE;
    flash_msg.dummy_mode = 0;
    flash_msg.dummy_cycles = 1;
    flash_msg.dummy_bit = 0;
    flash_msg.mm_mode = CCR_MM_MODE_XIP;
    flash_msg.alternate_byte_mode = CCR_ABMODE_NIL;
    QSPI_Transaction(qspinum,&flash_msg);    
    uint8_t temp;
    while(1){
        temp = readStatusRegister1(dev);
        temp = temp & 0x01;
        if(temp != 0x01)
        break;
    }
    return 0;
}

uint32_t chipErase(const struct device *dev){
    //currently not working so used 32Kchip erase as patch
    flash_msg.address_mode = CCR_ADMODE_NIL;
    flash_msg.address_size = CCR_ADSIZE_8_BIT;
    flash_msg.instruction = 0xC7;
    flash_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    flash_msg.data_mode = CCR_DMODE_NO_DATA;
    flash_msg.functional_mode = CCR_FMODE_INDIRECT_WRITE;
    flash_msg.dummy_mode = 0;
    flash_msg.dummy_cycles = 1;
    flash_msg.dummy_bit = 0;
    flash_msg.mm_mode = CCR_MM_MODE_XIP;
    flash_msg.alternate_byte_mode = CCR_ABMODE_NIL;
    return QSPI_Transaction(qspinum,&flash_msg);
}

uint32_t writeEnable(const struct device *dev){
    flash_msg.address_mode = CCR_ADMODE_NIL;
    flash_msg.address_size = CCR_ADSIZE_8_BIT;
    flash_msg.instruction = 0x06;
    flash_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    flash_msg.data_mode = CCR_DMODE_NO_DATA;
    flash_msg.functional_mode = CCR_FMODE_INDIRECT_WRITE;
    flash_msg.dummy_mode = 0;
    flash_msg.dummy_cycles = 1;
    flash_msg.dummy_bit = 0;
    flash_msg.mm_mode = CCR_MM_MODE_XIP;
    flash_msg.alternate_byte_mode = CCR_ABMODE_NIL;
    QSPI_Transaction(qspinum,&flash_msg);
    uint8_t temp;
    while(1){
        temp = readStatusRegister1(dev);
        temp = temp & 0x02;
        if(temp == 0x02)
        break;
    }
    return 0;
}

uint32_t writeDisable(const struct device *dev){
    flash_msg.address_mode = CCR_ADMODE_NIL;
    flash_msg.address_size = CCR_ADSIZE_8_BIT;
    flash_msg.instruction = 0x04;
    flash_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    flash_msg.data_mode = CCR_DMODE_NO_DATA;
    flash_msg.functional_mode = CCR_FMODE_INDIRECT_WRITE;
    flash_msg.dummy_mode = 0;
    flash_msg.dummy_cycles = 1;
    flash_msg.dummy_bit = 0;
    flash_msg.mm_mode = CCR_MM_MODE_XIP;
    flash_msg.alternate_byte_mode = CCR_ABMODE_NIL;  
    return QSPI_Transaction(qspinum,&flash_msg);   
}

uint32_t suspend(const struct device *dev){
    flash_msg.address_mode = CCR_ADMODE_NIL;
    flash_msg.address_size = CCR_ADSIZE_8_BIT;
    flash_msg.instruction = 0x75;
    flash_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    flash_msg.data_mode = CCR_DMODE_NO_DATA;
    flash_msg.functional_mode = CCR_FMODE_INDIRECT_WRITE;
    flash_msg.dummy_mode = 0;
    flash_msg.dummy_cycles = 0;
    flash_msg.dummy_bit = 0;
    flash_msg.mm_mode = CCR_MM_MODE_XIP;
    flash_msg.alternate_byte_mode = CCR_ABMODE_NIL;
    return QSPI_Transaction(qspinum,&flash_msg);
}

uint32_t resume(const struct device *dev){
    flash_msg.address_mode = CCR_ADMODE_NIL;
    flash_msg.address_size = CCR_ADSIZE_8_BIT;
    flash_msg.instruction = 0x7A;
    flash_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    flash_msg.data_mode = CCR_DMODE_NO_DATA;
    flash_msg.functional_mode = CCR_FMODE_INDIRECT_WRITE;
    flash_msg.dummy_mode = 0;
    flash_msg.dummy_cycles = 0;
    flash_msg.dummy_bit = 0;
    flash_msg.mm_mode = CCR_MM_MODE_XIP;
    flash_msg.alternate_byte_mode = CCR_ABMODE_NIL;
    return QSPI_Transaction(qspinum,&flash_msg);
}

uint32_t power_down(const struct device *dev){
    flash_msg.address_mode = CCR_ADMODE_NIL;
    flash_msg.address_size = CCR_ADSIZE_8_BIT;
    flash_msg.instruction = 0xB9;
    flash_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    flash_msg.data_mode = CCR_DMODE_NO_DATA;
    flash_msg.functional_mode = CCR_FMODE_INDIRECT_WRITE;
    flash_msg.dummy_mode = 0;
    flash_msg.dummy_cycles = 0;
    flash_msg.dummy_bit = 0;
    flash_msg.mm_mode = CCR_MM_MODE_XIP;
    flash_msg.alternate_byte_mode = CCR_ABMODE_NIL;
    return QSPI_Transaction(qspinum,&flash_msg);
}

uint32_t release_power_down(const struct device *dev){
    flash_msg.address_mode = CCR_ADMODE_NIL;
    flash_msg.address_size = CCR_ADSIZE_8_BIT;
    flash_msg.instruction = 0xAB;
    flash_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    flash_msg.data_mode = CCR_DMODE_NO_DATA;
    flash_msg.functional_mode = CCR_FMODE_INDIRECT_WRITE;
    flash_msg.dummy_mode = 0;
    flash_msg.dummy_cycles = 0;
    flash_msg.dummy_bit = 0;
    flash_msg.mm_mode = CCR_MM_MODE_XIP;
    flash_msg.alternate_byte_mode = CCR_ABMODE_NIL;
    return QSPI_Transaction(qspinum,&flash_msg);
}



uint8_t readStatusRegister2(const struct device *dev){// status reg 1
    uint8_t data;
    flash_msg.address_mode = CCR_ADMODE_NIL;
    flash_msg.address_size = CCR_ADSIZE_24_BIT;
    flash_msg.instruction = 0x35;
    flash_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    flash_msg.data_mode = CCR_DMODE_SINGLE_LINE;
    flash_msg.data_buffer = &data;
    flash_msg.functional_mode = CCR_FMODE_INDIRECT_READ;
    flash_msg.dummy_mode = 0;
    flash_msg.dummy_cycles = 0;
    flash_msg.dummy_bit = 0;
    flash_msg.mm_mode = CCR_MM_MODE_XIP;
    flash_msg.alternate_byte_mode = CCR_ABMODE_NIL;
    flash_msg.length = 1;
    QSPI_Transaction(qspinum,&flash_msg);
	return data;
}

uint8_t readStatusRegister3(const struct device *dev){// status reg 1
    uint8_t data;
    flash_msg.address_mode = CCR_ADMODE_NIL;
    flash_msg.address_size = CCR_ADSIZE_24_BIT;
    flash_msg.instruction = 0x15;
    flash_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    flash_msg.data_mode = CCR_DMODE_SINGLE_LINE;
    flash_msg.data_buffer = &data;
    flash_msg.functional_mode = CCR_FMODE_INDIRECT_READ;
    flash_msg.dummy_mode = 0;
    flash_msg.dummy_cycles = 0;
    flash_msg.dummy_bit = 0;
    flash_msg.mm_mode = CCR_MM_MODE_XIP;
    flash_msg.alternate_byte_mode = CCR_ABMODE_NIL;
    flash_msg.length = 1;
    QSPI_Transaction(qspinum,&flash_msg);
	return data;
}

uint8_t readFlagStatusRegister(const struct device *dev){
    uint8_t data;
    flash_msg.address_mode = CCR_ADMODE_NIL;
    flash_msg.address_size = CCR_ADSIZE_24_BIT;
    flash_msg.instruction = 0x70;
    flash_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    flash_msg.data_mode = CCR_DMODE_SINGLE_LINE;
    flash_msg.data_buffer = &data;
    flash_msg.functional_mode = CCR_FMODE_INDIRECT_READ;
    flash_msg.dummy_mode = 0;
    flash_msg.dummy_cycles = 0;
    flash_msg.dummy_bit = 0;
    flash_msg.mm_mode = CCR_MM_MODE_XIP;
    flash_msg.alternate_byte_mode = CCR_ABMODE_NIL;
    flash_msg.length = 1;
    QSPI_Transaction(qspinum,&flash_msg);
	return data;
}   

uint32_t writeEnableStatusRegister(const struct device *dev){
    flash_msg.address_mode = CCR_ADMODE_NIL;
    flash_msg.instruction = 0x50;
    flash_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    flash_msg.data_mode = CCR_DMODE_NO_DATA;
    flash_msg.functional_mode = CCR_FMODE_INDIRECT_WRITE;
    flash_msg.dummy_mode = 0;
    flash_msg.dummy_cycles = 1;
    flash_msg.dummy_bit = 0;
    flash_msg.mm_mode = CCR_MM_MODE_XIP;
    flash_msg.alternate_byte_mode = CCR_ABMODE_NIL;
    flash_msg.length = 0;
    return QSPI_Transaction(qspinum,&flash_msg);
}

uint32_t writeStatusRegister1(const struct device *dev,uint8_t* statusData){
    flash_msg.address_mode = CCR_ADMODE_NIL;
    flash_msg.instruction = 0x01;
    flash_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    flash_msg.data_mode = CCR_DMODE_SINGLE_LINE;
    flash_msg.functional_mode = CCR_FMODE_INDIRECT_WRITE;
    flash_msg.dummy_mode = 0;
    flash_msg.dummy_cycles = 1;
    flash_msg.dummy_bit = 0;
    flash_msg.mm_mode = CCR_MM_MODE_XIP;
    flash_msg.alternate_byte_mode = CCR_ABMODE_NIL;
    flash_msg.length = 1;
    flash_msg.data_buffer = statusData;
    return QSPI_Transaction(qspinum,&flash_msg);
}

uint32_t writeStatusRegister2(const struct device *dev,uint8_t* statusData){
    flash_msg.address_mode = CCR_ADMODE_NIL;
    flash_msg.instruction = 0x31;
    flash_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    flash_msg.data_mode = CCR_DMODE_SINGLE_LINE;
    flash_msg.functional_mode = CCR_FMODE_INDIRECT_WRITE;
    flash_msg.dummy_mode = 0;
    flash_msg.dummy_cycles = 1;
    flash_msg.dummy_bit = 0;
    flash_msg.mm_mode = CCR_MM_MODE_XIP;
    flash_msg.alternate_byte_mode = CCR_ABMODE_NIL;
    flash_msg.length = 1;
    flash_msg.data_buffer = statusData;
    return QSPI_Transaction(qspinum,&flash_msg);
}

uint32_t writeStatusRegister3(const struct device *dev,uint8_t* statusData){
    flash_msg.address_mode = CCR_ADMODE_NIL;
    flash_msg.instruction = 0x11;
    flash_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    flash_msg.data_mode = CCR_DMODE_SINGLE_LINE;
    flash_msg.functional_mode = CCR_FMODE_INDIRECT_WRITE;
    flash_msg.dummy_mode = 0;
    flash_msg.dummy_cycles = 1;
    flash_msg.dummy_bit = 0;
    flash_msg.mm_mode = CCR_MM_MODE_XIP;
    flash_msg.alternate_byte_mode = CCR_ABMODE_NIL;
    flash_msg.length = 1;
    flash_msg.data_buffer = statusData;
    return QSPI_Transaction(qspinum,&flash_msg);
}

uint8_t readGlobalFreezeBit(const struct device *dev){
    uint8_t data;
    flash_msg.address_mode = CCR_ADMODE_NIL;
    flash_msg.address_size = CCR_ADSIZE_24_BIT;
    flash_msg.instruction = 0xA7;
    flash_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    flash_msg.data_mode = CCR_DMODE_SINGLE_LINE;
    flash_msg.functional_mode = CCR_FMODE_INDIRECT_READ;
    flash_msg.dummy_mode = 0;
    flash_msg.dummy_cycles = 1;
    flash_msg.dummy_bit = 0;
    flash_msg.mm_mode = CCR_MM_MODE_XIP;
    flash_msg.alternate_byte_mode = CCR_ABMODE_NIL;
    flash_msg.length = 1;
    flash_msg.data_buffer = &data;
    QSPI_Transaction(qspinum,&flash_msg);
	return data;
}

uint32_t writeGlobalFreezeBit(const struct device *dev,uint8_t *data){
    flash_msg.address_mode = CCR_ADMODE_NIL;
    flash_msg.address_size = CCR_ADSIZE_24_BIT;
    flash_msg.instruction = 0xA6;
    flash_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    flash_msg.data_mode = CCR_DMODE_SINGLE_LINE;
    flash_msg.functional_mode = CCR_FMODE_INDIRECT_WRITE;
    flash_msg.dummy_mode = 0;
    flash_msg.dummy_cycles = 1;
    flash_msg.dummy_bit = 0;
    flash_msg.mm_mode = CCR_MM_MODE_XIP;
    flash_msg.alternate_byte_mode = CCR_ABMODE_NIL;
    flash_msg.data_buffer = data;
    flash_msg.length = 1;
    return QSPI_Transaction(qspinum,&flash_msg);
}

uint8_t readFlashSFDP(const struct device *dev,uint32_t address){
    uint8_t data;
    flash_msg.address_mode = CCR_ADMODE_SINGLE_LINE;
    flash_msg.address_size = CCR_ADSIZE_24_BIT;
    flash_msg.address = address;
    flash_msg.instruction = 0x5A;
    flash_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    flash_msg.data_mode = CCR_DMODE_SINGLE_LINE;
    flash_msg.functional_mode = CCR_FMODE_INDIRECT_READ;
    flash_msg.dummy_mode = 0;
    flash_msg.dummy_cycles = 7;
    flash_msg.dummy_bit = 0;
    flash_msg.mm_mode = CCR_MM_MODE_XIP;
    flash_msg.alternate_byte_mode = CCR_ABMODE_NIL;
    flash_msg.length = 1;
    flash_msg.data_buffer = &data;
    QSPI_Transaction(qspinum,&flash_msg);
	return data;
}

uint32_t readNVCR(const struct device *dev, uint8_t* data){
    flash_msg.address_mode = CCR_ADMODE_NIL;
    flash_msg.address_size = CCR_ADSIZE_24_BIT;
    flash_msg.instruction = 0xB5;
    flash_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    flash_msg.data_mode = CCR_DMODE_SINGLE_LINE;
    flash_msg.functional_mode = CCR_FMODE_INDIRECT_READ;
    flash_msg.dummy_mode = 0;
    flash_msg.dummy_cycles = 0;
    flash_msg.dummy_bit = 0;
    flash_msg.mm_mode = CCR_MM_MODE_XIP;
    flash_msg.alternate_byte_mode = CCR_ABMODE_NIL;
    flash_msg.length = 2;
    flash_msg.data_buffer = data;
    return QSPI_Transaction(qspinum,&flash_msg);
}

uint32_t readJedecID(const struct device *dev, uint8_t *id){
    flash_msg.address_mode = CCR_ADMODE_NIL;
    flash_msg.address_size = CCR_ADSIZE_24_BIT;
    flash_msg.instruction = 0x9F;
    flash_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    flash_msg.data_mode = CCR_DMODE_SINGLE_LINE;
    flash_msg.functional_mode = CCR_FMODE_INDIRECT_READ;
    flash_msg.dummy_mode = 0;
    flash_msg.dummy_cycles = 0;
    flash_msg.dummy_bit = 0;
    flash_msg.mm_mode = CCR_MM_MODE_XIP;
    flash_msg.alternate_byte_mode = CCR_ABMODE_NIL;
    flash_msg.length = 3;
    flash_msg.data_buffer = id;
    return QSPI_Transaction(qspinum,&flash_msg);
}

uint32_t writeNVCR(const struct device *dev, uint8_t* data){
    flash_msg.address_mode = CCR_ADMODE_NIL;
    flash_msg.address_size = CCR_ADSIZE_24_BIT;
    flash_msg.instruction = 0xB1;
    flash_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    flash_msg.data_mode = CCR_DMODE_SINGLE_LINE;
    flash_msg.functional_mode = CCR_FMODE_INDIRECT_WRITE;
    flash_msg.dummy_mode = 0;
    flash_msg.dummy_cycles = 0;
    flash_msg.dummy_bit = 0;
    flash_msg.mm_mode = CCR_MM_MODE_XIP;
    flash_msg.alternate_byte_mode = CCR_ABMODE_NIL;
    flash_msg.length = 16;
    flash_msg.data_buffer = data;
    return QSPI_Transaction(qspinum,&flash_msg);
}

int qspi_secureiot_init(const struct device *dev)
{
    char *qspi_instance;
    qspi_instance = dev->name;
    qspinum =  qspi_instance[7] - '2';
    if (qspinum > QSPI_MAX_COUNT & qspinum < 0) return 1;
    else return 0;

}

int flash_secureiot_read(const struct device *dev, off_t offset, void *data, size_t len)
{
    fastReadQuad(dev, (uint8_t)data, (uint32_t)offset, len);
    for (int i = 0; i < len; i++)
    {
        printk("DATA[%d] = %x\r \n",i, (uint8_t*) &data[i]);
    }
    return 0;
}

int flash_secureiot_write(const struct device *dev, off_t offset, const void *data, size_t len)
{
    writeEnable(dev);
    inputpageQuad(dev, (uint8_t)data, (uint32_t) offset, len);
    writeDisable(dev);

    return 0;
}

int flash_secureiot_erase(const struct device *dev, off_t offset,
			       size_t size)
{
    uint8_t sr, fsr;
    writeEnable(dev);
    chipErase(dev);
    writeDisable(dev);
    uint8_t temp = 0;
    while (1)
    {
        sr = readStatusRegister1(dev);
        if (sr == 0)    break;
    }
    printk("chip Erase completed\n");

    return 0;
}

static const struct flash_driver_api flash_secureiot_api = {
    .erase = flash_secureiot_erase,
    .write = flash_secureiot_write,
    .read = flash_secureiot_read,
};

#define QSPI_INIT(n) \
DEVICE_DT_INST_DEFINE(n, \
        qspi_secureiot_init, \
        NULL, \
        NULL, \
        NULL, \
        POST_KERNEL, \
        CONFIG_FLASH_INIT_PRIORITY, \
        &flash_secureiot_api);
        
DT_INST_FOREACH_STATUS_OKAY(QSPI_INIT)
