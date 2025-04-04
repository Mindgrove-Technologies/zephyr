/**
 * Project                           : Secure IoT SoC
 * Name of the file                  : flash_shakti.c
 * Brief Description of file         : This is a zephyr rtos FLASH Driver file for Mindgrove Silicon's QSPI Peripheral.
 * Name of Author                    : Kishore J
 * Email ID                          : kishore@mindgrovetech.in
 * 
 * @file flash_shakti.c
 * @author Kishore J (kishore@mindgrovetech.in)
 * @brief This is a zephyr rtos FLASH Driver file for Mindgrove Silicon's QSPI Peripheral.
 * @version 0.1
 * @date 2024-05-14
 * 
 * @copyright Copyright (c) Mindgrove Technologies Pvt. Ltd 2023. All rights reserved.
 * 
 * @copyright Copyright (c) 2017 Google LLC.
 * @copyright Copyright (c) 2018 qianfan Zhao.
 * @copyright Copyright (c) 2023 Gerson Fernando Budke.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

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

/**
 * The struct `flash_shakti_qspi_config` contains a function pointer and a struct for flash parameters.
 * @property {flash_shakti_qspi_config_func_t} cfg_func - The `cfg_func` property in the
 * `flash_shakti_qspi_config` struct is of type `flash_shakti_qspi_config_func_t`. This property 
 * represents a function pointer or a callback function that is used to configure the QSPI flash
 * device.
 * @property parameters - The `parameters` property in the `flash_shakti_qspi_config` struct 
 * contains specific configuration parameters related to the flash memory device being used. These
 * parameters could include details such as the memory size, page size, block size, and other
 * characteristics that are necessary for properly configuring and interfacing
 */
 
struct flash_shakti_qspi_config {
  flash_shakti_qspi_config_func_t cfg_func;
  struct flash_parameters parameters;
};


/**
 * The struct flash_shakti_qspi_data contains two instances of the k_sem structure for synchronization
 * purposes.
 * @property sem - The `sem` property is a semaphore variable defined within the
 * `flash_shakti_qspi_data` struct. Semaphores are synchronization primitives used in concurrent
 * programming to control access to shared resources. In this case, the `sem` semaphore is used
 * to coordinate access to resources or synchronize operations.
 * @property device_sync_sem - The `device_sync_sem` is a semaphore used for synchronizing access to
 * the device in the context of the `flash_shakti_qspi_data` structure. Semaphores are synchronization
 * primitives that are commonly used in concurrent programming to control access to shared resources.
 */

struct flash_shakti_qspi_data {
  struct k_sem sem;
	struct k_sem device_sync_sem;
};

uint8_t readFlagStatusRegister(const struct device *dev );


/**
 * The `qspi_Data` type is a union that can hold either a 32-bit, 16-bit, or 8-bit unsigned integer.
*/

typedef union{
    uint32_t data_32;
    uint16_t data_16;
    uint8_t data_8;
} qspi_Data;


/**
 * The qspi_struct type defines a structure with multiple uint32_t members for controlling a Quad
 * Serial Peripheral Interface (QSPI) module.
 * @property {uint32_t} cr - The `cr` property in the `qspi_struct` struct is used to store
 * control register values for a Quad Serial Peripheral Interface (QSPI) controller. This register
 * typically contains configuration settings and control bits that govern the behavior of the QSPI
 * controller.
 * @property {uint32_t} dcr - The `dcr` property in the `qspi_struct` structure stands for
 * Data Configuration Register. It is a 32-bit unsigned integer that hold configuration settings
 * related to data transfer operations in a Quad Serial Peripheral Interface (QSPI) controller. 
 * @property {uint32_t} sr - The `sr` property in the `qspi_struct` structure stands for
 * Status Register. It is used to store status information related to the Quad Serial Peripheral
 * Interface (QSPI) module. This register contain flags that indicate the current status of
 * the QSPI module.
 * @property {uint32_t} fcr - The `fcr` property in the `qspi_struct` structure stands for
 * "Flash Control Register". It is a 32-bit unsigned integer variable used to store control information
 * related to flash memory operations in a Quad Serial Peripheral Interface (QSPI) module. 
 * @property {uint32_t} dlr - The `dlr` property in the `qspi_struct` structure stands for
 * "Data Length Register." It is a 32-bit unsigned integer that is used to store information
 * related to the length of data being transferred or processed in a Quad Serial Peripheral Interface
 * (QSPI) communication.
 * @property {uint32_t} ccr - The `ccr` property in the `qspi_struct` structure stands for
 * "Clock Control Register." It is a 32-bit unsigned integer that stores configuration settings
 * related to the clock signal used in the Quad Serial Peripheral Interface (QSPI) communication.
 * @property {uint32_t} ar - The `ar` property in the `qspi_struct` structure is a 32-bit
 * unsigned integer representing a specific register related to Quad Serial Peripheral Interface (QSPI)
 * communication. This register stores some configuration or control information for the QSPI
 * peripheral.
 * @property {uint32_t} abr - The `abr` property in the `qspi_struct` structure is of type `uint32_t`,
 * which typically represents an unsigned 32-bit integer. It  stores data related to the QSPI (Quad Serial 
 * Peripheral Interface) module.
 * @property {qspi_Data} dr - The `dr` property in the `qspi_struct` structure is of type `qspi_Data`. Which is 
 * a data register.
 * @property {uint32_t} psmkr - The `psmkr` property in the `qspi_struct` structure is used to
 * store a value related to the Parallel Synchronous Mode Key Register in a Quad Serial Peripheral
 * Interface (QSPI) controller. This register is used in QSPI controllers to configure
 * settings related to parallel data transfer
 * @property {uint32_t} psmar - The `psmar` property in the `qspi_struct` structure is used to
 * store the Physical Sector Memory Address Register value in a QSPI (Quad Serial Peripheral Interface)
 * controller. This register holds the memory address for programming or erasing operations
 * in the QSPI flash memory.
 * @property {uint32_t} pir - The `pir` property in the `qspi_struct` structure stands for
 * "Polling Interval Register". It is a 32-bit unsigned integer variable that is used to store the
 * polling interval value for a Quad Serial Peripheral Interface (QSPI) controller.
 * @property {uint32_t} lptr - The `lptr` property in the `qspi_struct` structure stands for
 * "Last Pointer" or "Loopback Pointer". It is a variable of type `uint32_t` which is used to store
 * the last pointer value or loopback pointer value in the context of the Quad Serial
 */

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

/**
 * @fn int qspi_init(const struct device *dev)
 * @details The function initializes a QSPI instance based on the device name and returns the base
 * address of the QSPI instance.
 * @param dev in the `qspi_init` function is a pointer to a structure of type `device`. 
 * This structure likely contains information about the device being initialized, such as its
 * name and possibly other configuration parameters.
 * @return the base address of the QSPI instance if the initialization is successful. 
 * If the QSPI instance number is invalid, it will return -1.
 */

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


/**
 * @fn void qspi_config_cr(const struct device *dev, int prescale, int pmm, int apms, int toie, int smie, int ftie, int tcie, int teie, int fifo_th, int timeout_en)
 * @details The function `qspi_config_cr` configures various settings for a QSPI device based on the provided parameters.
 * @param dev is a pointer to a structure of type `device`. This parameter is used to specify the device on 
 * which the QSPI configuration is being applied.
 * @param prescale is used to set the prescaler value for the QSPI peripheral. It determines the division factor for the QSPI clock frequency.
 * @param pmm stands for "Polling Match Mode". It is used to configure whether the QSPI peripheral operates in polling match mode or not. 
 * When `pmm` is set to 1, the QSPI peripheral will operate in polling.
 * @param apms "Automatic Polling Mode Selection". It is used to configure whether the QSPI controller should operate in automatic polling
 * mode or not. When `apms` is set to 1, the controller will operate in automatic
 * @param toie stands for "Timeout Interrupt Enable". It is used to enable or disable the timeout interrupt feature in the QSPI controller. 
 * @param smie stands for "Serial Memory Interface Enable". It is used to enable or disable the Serial Memory Interface feature in the QSPI 
 * controller. When `smie` is set to 1, it enables the Serial Memory Interface feature.
 * @param ftie stands for "FIFO Threshold Interrupt Enable". It is used to enable or disable the interrupt when the FIFO threshold is reached
 * in the QSPI controller.
 * @param tcie stands for "Transfer Complete Interrupt Enable". It is used to enable or disable the interrupt that is triggered when a transfer
 * operation is completed in the Quad Serial Peripheral Interface (QSPI) module.
 * @param teie stands for "Transfer Error Interrupt Enable". It is used to enable or disable the interrupt for transfer errors in the Quad
 * Serial Peripheral Interface (QSPI) controller. When `teie` is set to 1, the transfer error interrupt is enabled.
 * @param fifo_th represents the FIFO threshold level for the Quad Serial Peripheral Interface (QSPI) controller. It determines the number
 * of entries in the FIFO that will trigger an interrupt or other actions related to the FIFO.
 * @param timeout_en is used to enable or disable the timeout feature in the QSPI controller. If `timeout_en` is set to 1, it means the
 * timeout feature is enabled, and if it is set to 0, the timeout feature is disabled.
 */

void qspi_config_cr(const struct device *dev, int prescale, int pmm, int apms, int toie, int smie, int ftie, int tcie, int teie, int fifo_th, int timeout_en)
{
   // qspi_instance[qspi_num]->cr = (CR_PRESCALER(prescale) | CR_PMM(pmm) | CR_APMS(apms) | CR_TOIE(toie) |\
      CR_SMIE(smie) | CR_FTIE(ftie) | CR_TCIE(tcie) | CR_TEIE(teie) | CR_FTHRES(fifo_th) | CR_TCEN(timeout_en));
   qspi_instance[qspi_num]->cr = (CR_PRESCALER(prescale) | CR_PMM(pmm) | CR_APMS(apms) | CR_TOIE(toie) |\
      CR_SMIE(smie) | CR_FTIE(ftie) | CR_TCIE(tcie) | CR_TEIE(teie) | CR_FTHRES(fifo_th) | CR_TCEN(timeout_en) | CR_EN(1));
}


/**
 * @fn void qspi_enable(const struct device *dev)
 * @details The function `qspi_enable` enables the QSPI peripheral by setting the EN bit in the control register.
 * @param dev in the `qspi_enable` function is a pointer to a structure of type `device`. 
 * This structure likely contains information about the device being initialized, such as its
 * name and possibly other configuration parameters.
 */

void qspi_enable(const struct device *dev)
{
  uint32_t temp = qspi_instance[qspi_num]->cr;
  temp = temp | CR_EN(1);

  qspi_instance[qspi_num]->cr = temp;
}


/**
 * @fn void qspi_disable(const struct device *dev)
 * @details The function `qspi_disable` disables the QSPI device and prints a message indicating successful
 * completion.
 * @param dev in the `qspi_init` function is a pointer to a structure of type `device`. 
 * This structure likely contains information about the device being initialized, such as its
 * name and possibly other configuration parameters.
 */

void qspi_disable(const struct device *dev)
{
  printk("Disabled qspi \n");
  uint32_t temp = qspi_instance[qspi_num]->cr;
  printk("Disabled qspi \n");
  uint32_t disable = ~CR_EN(1);
  temp = temp & disable;

  qspi_instance[qspi_num]->cr = temp;
  printk("Disabled qspi successfully \n");
}


/**
 * @fn void qspi_abort(const struct device *dev)
 * @details The function `qspi_abort` sets the ABORT bit in the Control Register of a QSPI device to abort
 * ongoing operations.
* @param dev in the `qspi_init` function is a pointer to a structure of type `device`. 
 * This structure likely contains information about the device being initialized, such as its
 * name and possibly other configuration parameters.
 */

void qspi_abort(const struct device *dev)
{
  uint32_t temp = qspi_instance[qspi_num]->cr;
  temp = temp | CR_ABORT(1);
  qspi_instance[qspi_num]->cr = temp;
}


/**
 * @fn void qspi_config_dcr(const struct device *dev, int mode_byte, int fmem_size, int clk_mode)
 * @details The function `qspi_config_dcr` configures the DCR register of a QSPI device with the specified mode
 * byte, flash memory size, and clock mode.
 * @param dev The `dev` parameter is a pointer to a structure of type `device`. It is used to reference
 * the device for which the QSPI configuration is being done.
 * @param mode_byte is used to specify the mode byte configuration for the QSPI device. It is likely a bit field 
 * or value that determines the specific mode settings for the QSPI device.
 * @param fmem_size represents the Flash memory size configuration for the Quad Serial Peripheral Interface (QSPI) controller. 
 * It specifies the size of the Flash memory connected to the QSPI controller.
 * @param clk_mode refers to the clock mode configuration for the Quad Serial Peripheral Interface (QSPI) controller. 
 * This parameter is used to specify how the clock signal should be generated and transmitted during QSPI
 * communication.
 */

void qspi_config_dcr(const struct device *dev, int mode_byte, int fmem_size, int clk_mode)
{
  //qspi_wait_till_not_busy(qspi_num);
  printk("qspi config dcr\n");
  qspi_instance[qspi_num]->dcr = (DCR_MODE_BYTE(mode_byte) | DCR_FSIZE(fmem_size) | DCR_CKMODE(clk_mode));
  printk("qspi config dcr\n");

}


/**
 * @fn uint32_t qspi_read_status(const struct device *dev)
 * @details The function qspi_read_status reads the status register of a QSPI device and returns the status
 * value.
 * @param dev The `dev` parameter in the `qspi_read_status` function is a pointer to a structure of
 * type `device`. It is used to access device-specific information or configurations related to the
 * QSPI (Quad Serial Peripheral Interface) device.
 * @return the value of the status register `sr` from the QSPI instance specified by `qspi_num`.
 */

uint32_t qspi_read_status(const struct device *dev)
{
  printk(" Qspi Read status \n");
  uint32_t status;
  status = qspi_instance[qspi_num]->sr;
  return status;
}


/**
 * @fn int qspi_check_fifo_full(const struct device *dev)
 * @details The function `qspi_check_fifo_full` checks if the FIFO of a QSPI device is full and returns a
 * corresponding status. 
 * @param dev The `dev` parameter is a pointer to a structure of type `device`. This structure contains 
 * information or configurations related to the QSPI device.
 * @return 1 if the FIFO is full or 0 if the FIFO is not full.
 */

int qspi_check_fifo_full(const struct device *dev)
{
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


/**
 * @fn int qspi_check_fifo_empty(const struct device *dev)
 * @details The function `qspi_check_fifo_empty` checks if the FIFO is empty and returns a corresponding status.
 * @param dev The `dev` parameter in the `qspi_check_fifo_empty` function is a pointer to a structure
 * of type `device`. This structure likely contains information or configurations related to the QSPI
 * (Quad Serial Peripheral Interface) device that the function is interacting with.
 * @return 1 if the FIFO is empty, 0 if the FIFO is not empty.
 */

int qspi_check_fifo_empty(const struct device *dev)
{
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


/**
 * @fn void qspi_wait_till_tx_complete(const struct device *dev)
 * @details The function `qspi_wait_till_tx_complete` waits until the QSPI transmit FIFO is empty.
 * @param dev is a pointer to a structure of type `device`. This structure likely contains information 
 * about the device being initialized, such as its name and possibly other configuration parameters.
 */

void qspi_wait_till_tx_complete(const struct device *dev)
{
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


/**
 * @fn void qspi_wait_till_not_busy(const struct device *dev)
 * @details The function `qspi_wait_till_not_busy` continuously checks the status of a QSPI device until it is
 * no longer busy.
 * @param dev is a pointer to a structure of type `device`. This structure likely contains information about the 
 * device being initialized, such as its name and possibly other configuration parameters.
 */

void qspi_wait_till_not_busy(const struct device *dev)
{
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


/**
 * @fn void qspi_clear_flags(const struct device *dev, int clr_tof, int clr_smf, int clr_tcf, int clr_tef)
 * @details The function `qspi_clear_flags` clears specific flags in a QSPI device based on the input
 * parameters.
 * @param dev is a pointer to a structure of type `device`. This structure likely contains information about the 
 * device being initialized, such as its name and possibly other configuration parameters.
 * @param clr_tof is used to determine whether to clear the Timeout Flag (TOF) in the QSPI Flag Clear Register (FCR). 
 * If `clr_tof` is set to 1, the Timeout Flag will be cleared by setting the corresponding bit in the FCR register
 * @param clr_smf is used to determine whether to clear the SMF (Status Match Flag) in the QSPI controller. If `clr_smf`
 *  is set to 1, the function will set the corresponding flag in the `temp'.
 * @param clr_tcf is used to specify whether the Transfer Complete Flag (TCF) should be cleared. If `clr_tcf` is set to 1, 
 * the function will clear the TCF flag by setting the corresponding bit.
 * @param clr_tef is used to specify whether the Transmission Error Flag (TEF) should be cleared. If `clr_tef` is set to 1, 
 * the function will set the corresponding bit in the `temp` variable to clear.
 */

void qspi_clear_flags(const struct device *dev, int clr_tof, int clr_smf, int clr_tcf, int clr_tef)
{
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


/**
 * @fn int qspi_data_len(const struct device *dev, uint32_t data_len, int wr_rd)
 * @details The function `qspi_data_len` sets or retrieves the data length register value based on the write or
 * read operation specified.
 * @param dev The `dev` parameter is a pointer to a structure of type `device`. It is used to reference
 * the device for which the QSPI data length is being manipulated in the `qspi_data_len` function.
 * @param data_len represents the length of the data to be written or read in the QSPI (Quad Serial Peripheral Interface) communication. 
 * It is the number of data bytes that will be transferred during the operation.
 * @param wr_rd is used to specify whether the operation is write or read operation. It is an integer value 
 * that can be either `WRITE` or `READ`.
 * @return If the `wr_rd` parameter is set to `READ`, the function will return the value of
 * `rd_data_len`, which is the data length stored in the QSPI instance. If `wr_rd` is not set to
 * `READ`, the function will return 0.
 */

int qspi_data_len(const struct device *dev, uint32_t data_len, int wr_rd)
{
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


/**
 * @fn void qspi_config_ccr(const struct device *dev, uint8_t instr, int imode, int admode, int adsize, int abmode, int absize,\
    int dummy_cycles, int dummy_conf, int dmode, int fmode, int sioo, int dummy_bit, int ddr_mode,int mm_mode)
 * @details The function `qspi_config_ccr` configures the various parameters of the control register for Quad SPI communication.
 * @param dev is a pointer to a structure of type `device`. It is used to access device-specific information or configurations related to the
 * QSPI (Quad Serial Peripheral Interface) device.
 * @param instr represents the instruction code that will be sent to the Quad SPI (QSPI) peripheral. This code will be 
 * used to instruct the QSPI device on what operation to perform, such as read, write, erase, etc.
 * @param imode represents the instruction mode for QuadSPI communication. It specifies how the instruction is sent to 
 * the device. 
 * @param admode represents the address mode for the Quad SPI peripheral. It specifies how the address is accessed during Quad SPI operations.
 * The possible values for `admode` could include different modes such as continuous, wrapped, or other
 * addressing modes
 * @param adsize represents the address size for the Quad SPI peripheral configuration. It specifies the size of the address that 
 * will be used during Quad SPI operations. The value of `adsize` will determine the number of address bits that can
 * be sent.
 * @param abmode represents the address bytes mode for the Quad SPI peripheral. It specifies how the address bytes are accessed during Quad SPI
 * transactions. The possible values for `abmode` typically include options like continuous, individual, or other specific modes
 * @param absize represents the address bus size. It specifies the size of the address bus used for the Quad SPI peripheral. The value of
 * `absize` determines the number of address bits that can be used for addressing memory locations.
 * @param dummy_cycles represents the number of dummy cycles to insert between the address and data phases during a QSPI transaction.
 * These dummy cycles are often used to allow for proper timing and synchronization in QSPI communication.
 * @param dummy_conf is used to specify the number of dummy cycles to be inserted between the address and data phases during QSPI
 * communication. This parameter allows you to configure the QSPI controller to insert a specific
 * number of dummy cycles for timing purposes.
 * @param dmode represents the data transfer mode for the Quad-SPI peripheral. It specifies how data is transferred between the Quad-SPI
 * peripheral and the external device. The possible values for `dmode` depend on the specific Quad-SPI.
 * @param fmode represents the functional mode of the Quad-SPI interface. It specifies the functional mode of the external memory connected to the
 * Quad-SPI interface. The value of `fmode` will determine how the Quad-SPI controller interacts with the external memory.
 * @param sioo function stands for "Send Instruction Only Once." It is used to specify whether the instruction should be sent only once or for every
 * address byte during a transaction on the Quad SPI interface.
 * @param dummy_bit represents the number of dummy cycles to insert between address and data phases during a read operation in Quad SPI (QSPI)
 * communication. Dummy cycles are used to allow the device to prepare the data to be read after receiving the
 * @param ddr_mode is used to specify the Double Data Rate (DDR) mode for the Quad SPI (QSPI) peripheral. DDR mode allows data to be
 * transferred on both the rising and falling edges of the clock signal.
 * @param mm_mode is used to configure the Multi-line mode for the Quad-SPI peripheral. It specifies whether the Quad-SPI peripheral operates
 * in single-line or multi-line mode. In multi-line mode, data is transferred using multiple I/O.
 */

void qspi_config_ccr(const struct device *dev, uint8_t instr, int imode, int admode, int adsize, int abmode, int absize,\
    int dummy_cycles, int dummy_conf, int dmode, int fmode, int sioo, int dummy_bit, int ddr_mode,int mm_mode)
{
  qspi_wait_till_not_busy(qspi_num);
  qspi_instance[qspi_num]->ccr = (CCR_INSTRUCTION(instr) | CCR_IMODE(imode) | CCR_ADMODE(admode) | CCR_ADSIZE(adsize) |\
      CCR_ABMODE(abmode) | CCR_ABSIZE(absize) | CCR_DCYC(dummy_cycles) | CCR_DUMMY_CONFIRMATION(dummy_conf) |\
      CCR_DMODE(dmode) | CCR_FMODE(fmode) | CCR_SIOO(sioo) | CCR_DUMMY_BIT(dummy_bit) | CCR_DDRM(ddr_mode) | CCR_MM_MODE(mm_mode));
}


/**
 * @fn int qspi_config_ar(const struct device *dev, uint32_t addr, int fmode)
 * @details The function `qspi_config_ar` configures the address register of a QSPI device, with a check to
 * prevent writing in memory map mode.
 * @param dev is a pointer to a structure of type `device`. It is used to access device-specific information or 
 * configurations related to the QSPI (Quad Serial Peripheral Interface) device.
 * @param addr represents the address value that will be written to the AR (Address Register) of the QSPI (Quad Serial Peripheral Interface) device.
 * This address is used for various operations such as read, write, or execute commands on the QSPI.
 * @param fmode represents the functional mode of the QSPI (Quad Serial Peripheral Interface) device. If fmode is equal to 3, it indicates that the QSPI 
 * is in memory-mapped mode where writing to the address register is not allowed.
 * @return 0 if the `fmode` is not equal to 3 and the address is successfully written to the AR register. If `fmode` is equal to 3, it will print a message and
 * return -1.
 */

int qspi_config_ar(const struct device *dev, uint32_t addr, int fmode)
{
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


/**
 * @fn qspi_config_abr(const struct device *dev, uint32_t alt_byte)
 * @details The function `qspi_config_abr` configures the alternate byte register of a QSPI device with a
 * specified value.
 * @param dev is a pointer to a structure of type `device`. It is used to access device-specific information or configurations related to the
 * QSPI (Quad Serial Peripheral Interface) device.
 * @param alt_byte is a 32-bit unsigned integer that represents the alternate byte value to be written to the 
 * ABR (Alternate Byte Register) of the QSPI (Quad Serial Peripheral Interface) device.
 */

void qspi_config_abr(const struct device *dev, uint32_t alt_byte)
{
  qspi_wait_till_not_busy(qspi_num);
  qspi_instance[qspi_num]->abr = alt_byte;
}


/**
 * @fn qspi8_wr_dr(const struct device *dev, uint8_t* tx_data, int data_size)
 * @details The function qspi8_wr_dr writes a specified amount of data to a QSPI device using 8-bit data width.
 * @param dev is a pointer to a structure of type `device`. It is used to access device-specific information or configurations related to the
 * QSPI (Quad Serial Peripheral Interface) device.
 * @param tx_data is a pointer to an array of uint8_t type, which contains the data to be written to the 
 * QSPI (Quad Serial Peripheral Interface) device.
 * @param data_size represents the size of the data array `tx_data` that you want to write to the QSPI (Quad Serial Peripheral Interface) device.
 * It indicates the number of bytes of data that you want to transfer.
 */

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


/**
 * @fn void qspi16_wr_dr(const struct device *dev, uint16_t tx_data[FIFO_DEPTH_16], int data_size)
 * @details The function `qspi16_wr_dr` writes 16-bit data to a QSPI device in chunks of a specified size.
 * @param dev is a pointer to a structure of type `device`. It is used to access device-specific information
 * or configurations related to the QSPI (Quad Serial Peripheral Interface) device.
 * @param tx_data is an array of 16-bit unsigned integers that contains the data to be written to the QSPI (Quad 
 * Serial Peripheral Interface) device. The function `qspi16_wr_dr` writes this data to the QSPI device in chunks of 16 bits at a time.
 * @param data_size is the total number of 16-bit data elements to be written to the QSPI device.
 */

void qspi16_wr_dr(const struct device *dev, uint16_t tx_data[FIFO_DEPTH_16], int data_size)
{
  uint32_t data_len = data_size*2;
  int no_of_itr = data_size/2;

  qspi_data_len(qspi_num, data_len, WRITE);
  for (int i=0; i < no_of_itr; i++){
    qspi_check_fifo_full(qspi_num);
    qspi_instance[qspi_num]->dr.data_16 = tx_data[i] ;
  }
}


/**
 * @fn void qspi32_wr_dr(const struct device *dev, uint32_t tx_data[FIFO_DEPTH_32], int data_size)
 * @details The function `qspi32_wr_dr` writes 32-bit data to a QSPI device in chunks of 4 bytes.
 * @param dev is a pointer to a structure of type `device`. It is used to access device-specific information 
 * or configurations related to the QSPI (Quad Serial Peripheral Interface) device.
 * @param tx_data is an array of 32-bit unsigned integers with a size of `FIFO_DEPTH_32`. This array is used to 
 * store the data that will be written to the QSPI (Quad Serial Peripheral Interface) device.
 * @param data_size represents the number of 32-bit words to be written to the QSPI peripheral. It is used to 
 * calculate the total data length in bytes and the number of iterations needed to write the data in chunks of 32 bits
 */

void qspi32_wr_dr(const struct device *dev, uint32_t tx_data[FIFO_DEPTH_32], int data_size)
{
  uint32_t data_len = data_size*4;
  int no_of_itr = data_size/4;

  qspi_data_len(qspi_num, data_len, WRITE);
  for (int i=0; i < no_of_itr; i++){
    qspi_check_fifo_full(qspi_num);
    qspi_instance[qspi_num]->dr.data_32 = tx_data[i] ;
  }
}


/**
 * @fn void qspi_wait_till_rx_fifo_fills(const struct device *dev, uint32_t data_len)
 * @details The function `qspi_wait_till_rx_fifo_fills` waits until the receive FIFO in a QSPI device fills up
 * to a specified data length.
 * @param dev is a pointer to a structure of type `device`. It is used to access device-specific information 
 * or configurations related to the QSPI (Quad Serial Peripheral Interface) device.
 * @param data_len represents the length of data that needs to be received and stored in the receive FIFO of the QSPI (Quad Serial
 * Peripheral Interface) device. This function continuously checks the status of the QSPI device.
 */

void qspi_wait_till_rx_fifo_fills(const struct device *dev, uint32_t data_len)
{
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


/**
 * @fn uint8_t qspi8_rd_dr(const struct device *dev)
 * @details The function qspi8_rd_dr reads an 8-bit data from a QSPI device's data register if the FIFO is not
 * empty.
 * @param dev is a pointer to a structure of type `device`. It is used to access device-specific information 
 * or configurations related to the QSPI (Quad Serial Peripheral Interface) device.
 * @return the received 8-bit data (`rx_data_8`) if the FIFO is not empty, or -1 if the FIFO is empty.
 */

uint8_t qspi8_rd_dr(const struct device *dev)
{
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


/**
 * @fn uint16_t qspi16_rd_dr(const struct device *dev)
 * @details The function `qspi16_rd_dr` reads a 16-bit data from a QSPI device's data register if the FIFO is
 * not empty.
 * @param dev is a pointer to a structure of type `device`. It is used to access device-specific information 
 * or configurations related to the QSPI (Quad Serial Peripheral Interface) device.
 * @return the 16-bit data read from the QSPI data register if the FIFO is not empty, or -1 if the FIFO is empty.
 */

uint16_t qspi16_rd_dr(const struct device *dev)
{
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


/**
 * @fn uint32_t qspi32_rd_dr(const struct device *dev)
 * @details The function `qspi32_rd_dr` reads a 32-bit data from a QSPI device's data register if the FIFO is
 * not empty.
 * @param dev is a pointer to a structure of type `device`. It is used to access device-specific information 
 * or configurations related to the QSPI (Quad Serial Peripheral Interface) device.
 * @return the 32-bit data read from the QSPI device's data register if the FIFO is not empty, or -1 if the FIFO is empty.
 */

uint32_t qspi32_rd_dr(const struct device *dev)
{
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


/**
 * @fn void qspi_config_psmkr(const struct device *dev, uint32_t status_mask)
 * @details The function `qspi_config_psmkr` configures the status mask register of a QSPI device after ensuring
 * it is not busy.
 * @param dev is a pointer to a structure of type `device`. It is used to access device-specific information 
 * or configurations related to the QSPI (Quad Serial Peripheral Interface) device.
 * @param status_mask is a 32-bit unsigned integer that is used to configure the PSMKR (Polling Status Mask Register) 
 * in the QSPI (Quad Serial Peripheral Interface) controller. This register is typically used to mask specific status bits 
 * for polling operations in QSPI communication.
 */

void qspi_config_psmkr(const struct device *dev, uint32_t status_mask)
{
  qspi_wait_till_not_busy(qspi_num);
  qspi_instance[qspi_num]->psmkr = status_mask;
}


/**
 * @fn void qspi_config_psmar(const struct device *dev, uint32_t status_match)
 * @details The function `qspi_config_psmar` configures the status match register of a QSPI device with a
 * specified value.
 * @param dev is a pointer to a structure of type `device`. It is used to access device-specific information 
 * or configurations related to the QSPI (Quad Serial Peripheral Interface) device.
 * @param status_match is a 32-bit unsigned integer that represents the value to be written to the PSMAR (Polling Status 
 * Match Register) register in the QSPI (Quad Serial Peripheral Interface) controller configuration. This value is 
 * used for setting up status match conditions for the QSPI.
 */

void qspi_config_psmar(const struct device *dev, uint32_t status_match)
{
  qspi_wait_till_not_busy(qspi_num);
  qspi_instance[qspi_num]->psmar = status_match;
}


/**
 * @fn void qspi_config_pir(const struct device *dev, uint32_t poll_interval)
 * @details The function `qspi_config_pir` configures the poll interval register of a QSPI device.
 * @param dev is a pointer to a structure of type `device`. It is used to access device-specific information 
 * or configurations related to the QSPI (Quad Serial Peripheral Interface) device.
 * @param poll_interval represents the time interval in which the Quad Serial Peripheral Interface (QSPI) 
 * controller will poll for status during operations. It is a uint32_t type variable that specifies the duration 
 * in clock cycles for the QSPI controller.
 */
 
void qspi_config_pir(const struct device *dev, uint32_t poll_interval)
{
  qspi_wait_till_not_busy(qspi_num);
  qspi_instance[qspi_num]->pir = poll_interval;
}


/**
 * @fn void qspi_config_lptr(const struct device *dev, uint32_t time_out)
 * @details The function `qspi_config_lptr` sets the timeout value for the QSPI peripheral.
 * @param dev is a pointer to a structure of type `device`. It is used to access device-specific information 
 * or configurations related to the QSPI (Quad Serial Peripheral Interface) device.
 * @param time_out is a `uint32_t` type variable that represents the timeout value to be set for the LPTR 
 * (Last Pointer) register in the QSPI (Quad Serial Peripheral Interface) controller.
 */
 
void qspi_config_lptr(const struct device *dev, uint32_t time_out)
{
  qspi_wait_till_not_busy(qspi_num);
  qspi_instance[qspi_num]->lptr = time_out;
}


/**
 * @fn uint8_t qspi_check_transaction_complete (const struct device *dev)
 * @details The function `qspi_check_transaction_complete` checks if a QSPI transaction is complete and returns
 * a status flag.
 * @param dev is a pointer to a structure of type `device`. It is used to access device-specific information 
 * or configurations related to the QSPI (Quad Serial Peripheral Interface) device.
 * @return a `uint8_t` value, which is either 1 if the QSPI transaction is complete or 0 if the transaction is not complete.
 */

uint8_t qspi_check_transaction_complete (const struct device *dev)
{
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


/**
 * @fn void qspi_wait_till_transaction_complete (const struct device *dev)
 * @details The function `qspi_wait_till_transaction_complete` waits until a QSPI transaction is complete.
 * @param dev is a pointer to a structure of type `device`. It is used to access device-specific information 
 * or configurations related to the QSPI (Quad Serial Peripheral Interface) device.
 */

void qspi_wait_till_transaction_complete (const struct device *dev)
{
  uint8_t temp = 0;
  while(qspi_check_transaction_complete(dev) == 0);
}



/* -------------------------------------------------------flash driver---------------------------------------------------------*/ 

#define PRESCALE_QSPI 32
//read functions
//FLASH CONSTANTS
#define SR_PROG_STATUS (1<<7)

/**
 * @fn void fastReadQuad(const struct device *dev ,uint8_t* data,uint32_t address,uint8_t data_length)
 * @details The function `fastReadQuad` reads data from a specified address using Quad SPI communication.
 * @param dev is a pointer to a structure of type `device`. It is used to access device-specific information 
 * or configurations related to the QSPI (Quad Serial Peripheral Interface) device.
 * @param data is a pointer to a uint8_t type, which is used to store the data read from the QSPI flash memory. 
 * The function reads data from the specified address in the QSPI flash memory and stores it in the `data`.
 * @param address represents the memory address from which data will be read using Quad SPI (QSPI) communication. 
 * This address specifies the location in the memory device from which the data transfer will start.
 * @param data_length represents the length of the data (in bytes) that you want to read from the specified address using Quad SPI
 * communication. It indicates how many bytes of data you expect to read from the device starting at the given address.
 */

void fastReadQuad(const struct device *dev ,uint8_t* data,uint32_t address,uint8_t data_length)
{
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


/**
 * @fn void fastReadQuad32(const struct device *dev ,uint32_t* data,uint32_t address,uint8_t data_length)
 * @details The function `fastReadQuad32` reads 32-bit data from a specified address using Quad SPI
 * communication.
 * @param dev is a pointer to a structure of type `device`. It is used to access device-specific information 
 * or configurations related to the QSPI (Quad Serial Peripheral Interface) device.
 * @param data is a pointer to a `uint32_t` type, which means it is a pointer to a 32-bit unsigned integer where 
 * the read data will be stored.
 * @param address represents the memory address from which you want to read data. It is used to specify the starting 
 * address for the read operation in the memory device connected to the QSPI interface.
 * @param data_length represents the length of the data that you want to read from the QSPI flash memory device. 
 * It specifies the number of 32-bit data elements that you want to read and store in the `data` array.
 */

void fastReadQuad32(const struct device *dev ,uint32_t* data,uint32_t address,uint8_t data_length)
{
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


/**
 * @fn void fastReadQuadIO(const struct device *dev ,uint8_t *data,uint32_t address,uint8_t data_length)
 * @details The function `fastReadQuadIO` reads data from a specified address using Quad SPI communication.
 * @param dev is a pointer to a structure of type `device`. It is used to access device-specific information 
 * or configurations related to the QSPI (Quad Serial Peripheral Interface) device.
 * @param data is a pointer to a uint8_t type, which is used to store the data read from the QSPI flash memory.
 * @param address represents the memory address from which data will be read using Quad SPI (QSPI) communication. 
 * This address specifies the location in the memory device from which the data transfer will start.
 * @param data_length represents the length of the data that you want to read from the specified address using 
 * Quad SPI communication. It indicates the number of bytes you want to read from the memory starting at the given address.
 */

void fastReadQuadIO(const struct device *dev ,uint8_t *data,uint32_t address,uint8_t data_length)
{
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


/**
 * 
 * @fn void wait_till_program_or_erase_complete(const struct device *dev)
 * @details The function `wait_till_program_or_erase_complete` waits until a program or erase operation is
 * complete by continuously checking a flag status register.
 * @param dev is a pointer to a structure of type `device`. It is used to access device-specific information 
 * or configurations related to the QSPI (Quad Serial Peripheral Interface) device.
 */

void wait_till_program_or_erase_complete(const struct device *dev)
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


/**
 * @fn void inputpageQuad(const struct device *dev ,uint8_t* data,uint32_t address,uint8_t data_length)
 * @details The function `inputpageQuad` initializes and configures a Quad SPI interface to write data to a
 * specific memory address.
 * @param dev is a pointer to a structure of type `device`. It is used to access device-specific information 
 * or configurations related to the QSPI (Quad Serial Peripheral Interface) device.
 * @param data is a pointer to an array of `uint8_t` type, which represents the data that needs to be written 
 * to the QSPI device.
 * @param address represents the memory address where the data will be written to in the Quad SPI flash memory device. 
 * It is a 32-bit unsigned integer (`uint32_t`) that specifies the starting address for the data transfer operation.
 * @param data_length represents the length of the data array that you want to write to the QSPI device. It indicates
 * the number of bytes of data that you want to transfer or write to the specified address in the QSPI device.
 */

void inputpageQuad(const struct device *dev ,uint8_t* data,uint32_t address,uint8_t data_length)
{
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


/**
 * @fn void sector4KErase(const struct device *dev ,uint32_t address)
 * @details The function `sector4KErase` erases a 4K sector in a QSPI flash memory device.
 * @param dev is a pointer to a structure of type `device`. It is used to access device-specific information 
 * or configurations related to the QSPI (Quad Serial Peripheral Interface) device.
 * @param address represents the memory address of the sector that you want to erase. This address specifies 
 * the starting address of the 4K sector that you want to erase in the memory device.
 */

void sector4KErase(const struct device *dev ,uint32_t address)
{
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


/**
 * @fn void sector32KErase(const struct device *dev ,uint32_t address)
 * @details The function `sector32KErase` erases a 32K sector in a QSPI flash memory device at a specified
 * address.
 * @param dev is a pointer to a structure of type `device`. It is used to access device-specific information 
 * or configurations related to the QSPI (Quad Serial Peripheral Interface) device.
 * @param address is the memory address of the sector that you want to erase. This address specifies the 
 * starting address of the 32KB sector in the memory device that you want to erase.
 */

void sector32KErase(const struct device *dev ,uint32_t address)
{
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


/**
 * @fn void chipErase(const struct device *dev)
 * @details The function `chipErase` performs a chip erase operation on a QSPI flash memory device.
 * @param dev is a pointer to a structure of type `device`. It is used to access device-specific information 
 * or configurations related to the QSPI (Quad Serial Peripheral Interface) device.
 */

void chipErase(const struct device *dev)
{
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


/**
 * @fn void writeEnable(const struct device *dev)
 * @details The function `writeEnable` initializes and configures a QSPI device for writing data to a SPI NOR
 * flash memory.
 * @param dev is a pointer to a structure of type `device`. It is used to access device-specific information 
 * or configurations related to the QSPI (Quad Serial Peripheral Interface) device.
 */

void writeEnable(const struct device *dev)
{
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


/**
 * @fn  void writeDisable(const struct device *dev)
 * @details The function `writeDisable` disables writing operations on a QSPI device after initializing and
 * configuring it.
 * @param dev is a pointer to a structure of type `device`. It is used to access device-specific information 
 * or configurations related to the QSPI (Quad Serial Peripheral Interface) device.
 */

void writeDisable(const struct device *dev)
{
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


/**
 * @fn void suspend(const struct device *dev)
 * @details The function "suspend" initializes and configures a QSPI device before disabling it.
 * @param dev is a pointer to a structure of type `device`. It is used to access device-specific information 
 * or configurations related to the QSPI (Quad Serial Peripheral Interface) device.
 */

void suspend(const struct device *dev)
{
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


/**
 * @fn void continuosmode_reset(const struct device *dev)
 * @details The function `continuosmode_reset` initializes and configures a QSPI device for continuous mode
 * operation.
 * @param dev is a pointer to a structure of type `device`. It is used to access device-specific information 
 * or configurations related to the QSPI (Quad Serial Peripheral Interface) device.
 */

void continuosmode_reset(const struct device *dev)
{
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


/**
 * @fn void resume(const struct device *dev
 * @details The function `resume` initializes and configures a QSPI device before disabling it.
 * @param dev is a pointer to a structure of type `device`. It is used to access device-specific information 
 * or configurations related to the QSPI (Quad Serial Peripheral Interface) device.
 */
void resume(const struct device *dev)
{
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


/**
 * @fn void power_down(const struct device *dev) 
 * @details The function `power_down` initializes and configures a QSPI device for power-down mode operation.
 * @param dev is a pointer to a structure of type `device`. It is used to access device-specific information 
 * or configurations related to the QSPI (Quad Serial Peripheral Interface) device.
 */

void power_down(const struct device *dev)
{
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


/**
 * @fn void release_power_down(const struct device *dev)
 * @details The function `release_power_down` initializes and configures a QSPI device before disabling it.
 * @param dev is a pointer to a structure of type `device`. It is used to access device-specific information 
 * or configurations related to the QSPI (Quad Serial Peripheral Interface) device.
 */

void release_power_down(const struct device *dev)
{
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


/**
 * @fn uint8_t readStatusRegister(const struct device *dev)
 * @details The function `readStatusRegister` reads the status register of a device using QSPI communication.
 * @param dev is a pointer to a structure of type `device`. It is used to access device-specific information 
 * or configurations related to the QSPI (Quad Serial Peripheral Interface) device.
 * @return a `uint8_t` value, which is the data read from the status register of the device.
 */

uint8_t readStatusRegister(const struct device *dev)
{
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


/**
 * @fn uint8_t readFlagStatusRegister(const struct device *dev)
 * @details The function `readFlagStatusRegister` reads the flag status register using QSPI communication
 * protocol.
 * @param dev is a pointer to a structure of type `device`. It is used to access device-specific information 
 * or configurations related to the QSPI (Quad Serial Peripheral Interface) device.
 * @return a 8-bit unsigned integer (uint8_t) value, which represents the data read from the Flag Status Register of the device.
 */

uint8_t readFlagStatusRegister(const struct device *dev)
{
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


/**
 * @fn void writeEnableStatusRegister(const struct device *dev)
 * @details The function `writeEnableStatusRegister` initializes and configures a QSPI device to enable writing
 * to a status register.
 * @param dev is a pointer to a structure of type `device`. It is used to access device-specific information 
 * or configurations related to the QSPI (Quad Serial Peripheral Interface) device.
 */

void writeEnableStatusRegister(const struct device *dev)
{
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


/**
 * @fn void writeStatusRegister(const struct device *dev ,uint8_t* statusData)
 * @details The function `writeStatusRegister` initializes and configures a QSPI device to write data to a
 * specific status register address in a flash memory device.
 * @param dev is a pointer to a structure of type `device`. It is used to access device-specific information 
 * or configurations related to the QSPI (Quad Serial Peripheral Interface) device.
 * @param statusData is a pointer to a uint8_t type, which is essentially an 8-bit unsigned integer. This parameter
 * is used to pass the data that you want to write to the status register of the flash memory device.
 */

void writeStatusRegister(const struct device *dev ,uint8_t* statusData)
{
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


/**
 * @fn uint8_t readGlobalFreezeBit(const struct device *dev)
 * @details The function reads the global freeze bit from a QSPI device after initializing and configuring it.
 * @param dev is a pointer to a structure of type `device`. It is used to access device-specific information 
 * or configurations related to the QSPI (Quad Serial Peripheral Interface) device.
 * @return a `uint8_t` value, which is the data read from a specific memory address using QSPI communication.
 */

uint8_t readGlobalFreezeBit(const struct device *dev)
{
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


/**
 * @fn void writeGlobalFreezeBit(const struct device *dev)
 * @details The function `writeGlobalFreezeBit` initializes a QSPI device, configures various settings, writes
 * data using indirect write mode, and then disables the QSPI device.
 * @param dev is a pointer to a structure of type `device`. It is used to access device-specific information 
 * or configurations related to the QSPI (Quad Serial Peripheral Interface) device.
 */

void writeGlobalFreezeBit(const struct device *dev)
{
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


/**
 * @fn uint8_t readFlashSFDP(const struct device *dev ,uint32_t address)
 * @details The function `readFlashSFDP` reads data from a specific address in flash memory using QSPI
 * communication.
 * @param dev is a pointer to a structure of type `device`. It is used to access device-specific information 
 * or configurations related to the QSPI (Quad Serial Peripheral Interface) device.
 * @param address is the memory address from which you want to read data. It is a 32-bit unsigned integer (`uint32_t`) 
 * that specifies the location in the flash memory where the data is to be read from.
 * @return a `uint8_t` data type, which is the data read.
 */

uint8_t readFlashSFDP(const struct device *dev ,uint32_t address)
{
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


/**
 * @fn uint8_t readNVCR(const struct device *dev , uint8_t* data)
 * @details The function `readNVCR` reads data from a non-volatile configuration register using QSPI
 * communication.
 * @param dev is a pointer to a structure of type `device`. It is used to access device-specific information 
 * or configurations related to the QSPI (Quad Serial Peripheral Interface) device.
 * @param data is a pointer to a `uint8_t` type, which is used to store the data read from the QSPI device. The 
 * function reads 2 bytes of data from the QSPI device and stores them in the memory locations pointed.
 * @return a `uint8_t` type, it is actually returning a pointer to a `uint8_t` data array. 
 */

uint8_t readNVCR(const struct device *dev , uint8_t* data)
{
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


/**
 * @fn void writeNVCR(const struct device *dev , uint8_t* data)
 * @details The function `writeNVCR` initializes a QSPI device, configures various settings, writes data to the
 * device using single line mode, and then disables the QSPI device.
 * @param dev is a pointer to a structure of type `device`. It is used to access device-specific information 
 * or configurations related to the QSPI (Quad Serial Peripheral Interface) device.
 * @param data is a pointer to an array of `uint8_t` type, which represents the data that you want to write to the 
 * non-volatile configuration register (NVCR) using the QSPI (Quad Serial Peripheral Interface) communication protocol.
 */

void writeNVCR(const struct device *dev , uint8_t* data)
{
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


/**
 * @fn void qspi_xip_init(const struct device *dev , int flash_size)
 * @details The function `qspi_xip_init` initializes a QSPI device for executing code directly from flash memory
 * in XIP mode.
 * @param dev is a pointer to a structure of type `device`. It is used to access device-specific information 
 * or configurations related to the QSPI (Quad Serial Peripheral Interface) device.
 * @param flash_size represents the size of the flash memory in terms of its addressable space. The formula used in 
 * the comment suggests that the number of bytes in flash memory is calculated as 2 raised to the power of (`flash_size + 1).
 */

// Number of bytes in flash memory = 2 ^ [flash_size + 1]
void qspi_xip_init(const struct device *dev , int flash_size)
{
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


/**
 * @fn void qspi_ram_init(const struct device *dev , int flash_size)
 * @details The function `qspi_ram_init` initializes a QSPI flash memory device for XIP mode operation.
 * @param dev is a pointer to a structure of type `device`. It is used to access device-specific information 
 * or configurations related to the QSPI (Quad Serial Peripheral Interface) device.
 * @param flash_size represents the size of the flash memory in bytes. It is used to configure the Direct Memory 
 * Access (DMA) controller for the Quad Serial Peripheral Interface (QSPI) peripheral.
 */

void qspi_ram_init(const struct device *dev , int flash_size)
{
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

/**
 * @fn int flash_shakti_erase(const struct device *dev, off_t addr, size_t size)
 * @details The function `flash_shakti_erase` performs erasing operations on a flash memory device and provides
 * status information during the process.
 * @param dev is a pointer to a structure of type `device`. It is used to access device-specific information 
 * or configurations related to the QSPI (Quad Serial Peripheral Interface) device.
 * @param addr represents the starting address in the flash memory from where the erase operation should begin.  
 * This address specifies the location within the flash memory where the erase operation will be performed.
 * @param size represents the size of the flash memory region to be erased.
 * @return a integer value of 0.
 */

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


/**
 * @fn int flash_shakti_write(const struct device *dev, off_t addr, 
                        const void *src, size_t size)
 * @details The function `flash_shakti_write` writes data to a specified address in flash memory using quad SPI
 * communication.
 * @param dev is a pointer to a structure of type `device`. It is used to access device-specific information 
 * or configurations related to the QSPI (Quad Serial Peripheral Interface) device.
 * @param addr represents the starting address in the flash memory where the data specified by the `src` parameter 
 * will be written. It is of type `off_t`, which is typically used to represent file offsets or memory addresses.
 * @param src is a pointer to the data that needs to be written to the flash memory. It points to the memory location 
 * where the data is stored.
 * @param size represents the number of bytes to be written to the flash memory starting from the specified address `addr`. 
 * It indicates the size of the data pointed to by the `src` pointer that needs to be written to the flash memory.
 * @return a integer value of 0.
 */

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


/**
 * @fn int flash_shakti_read(const struct device *dev, off_t addr, 
                        void *dest, size_t size)
 * @details The function `flash_shakti_read` reads data from a specific address using Quad SPI communication.
 * @param dev is a pointer to a structure of type `device`. It is used to access device-specific information 
 * or configurations related to the QSPI (Quad Serial Peripheral Interface) device.
 * @param addr represents the address from which data will be read. It is of type `off_t`, which is typically 
 * used to represent file offsets or memory addresses. In this function, it is used as the starting address for reading data.
 * @param dest is a pointer to the memory location where the data read from the flash memory will be stored. It is of type `void *`, which
 * means it can point to data of any type.
 * @param size represents the number of bytes to read from the flash memory starting at the specified address (`addr`). 
 * It indicates the size of the data that needs to be read and stored in the `dest` buffer.
 * @return a integer value of 0.
 */

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


/* The below code is defining a static constant structure `flash_driver_api` with three function
pointers `erase`, `write`, and `read` pointing to functions `flash_shakti_erase`,
`flash_shakti_write`, and `flash_shakti_read` respectively. This structure is used to define an API
for flash driver operations in a C program. */

static const struct flash_driver_api flash_shakti_api = {
    .erase = flash_shakti_erase,
    .write = flash_shakti_write,
    .read  = flash_shakti_read,
};


/* The below code is a macro definition in C that is used to initialize a flash memory device on a
Shakti processor using Quad Serial Peripheral Interface (QSPI) communication. The macro
`FLASH_SHAKTI_QSPI_INIT(n)` is used to define the necessary data structures and configurations for
the flash memory device. It initializes a static data structure `flash_shakti_qspi_data_##n` and a
configuration structure `flash_shakti_qspi_config_##n` for the specified instance `n`. */

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