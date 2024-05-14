#ifndef FLASH_DRIVER_H
#define FLASH_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
// #include "platform.h"
// #include "log.h"

#define QSPI_NUM 0

/*!Quad Serial Peripheral Interface Offsets */

#define QSPI_MAX_COUNT 2

#define QSPI0_BASE_ADDRESS  0x00040000 /*! Standard Quad Serial Peripheral Interface Base address*/
#define QSPI0_END_ADDRESS   0x000400FF
#define QSPI1_BASE_ADDRESS  0x00040100
#define QSPI1_END_ADDRESS    0x000401FF /*! Standard Quad Serial Peripheral Interface End address*/

#define QSPI_OFFSET 0x100

/*--------------Flash erase----------------*/

// Define the sizes for your data structures
#define SHA_SIZE 32
#define RSA_SIZE 256
#define MODN_SIZE 256
#define N_SIZE 256
#define EXP_SIZE 3

// Define memory addresses
#define RSA_SIGNATURE_START 0x00000000 + 0x00006700
#define SHA256_HASH_START   0x00000120 + 0x00006700
#define R2MODN_START        0x00000160 + 0x00006700
#define N_START             0x00000280 + 0x00006700
#define EXP_START           0x00000400 + 0x00006700


#define ERASE_ADDR 0x00006000
#define SectorErase 0x00000000

/*-----------------------------------------*/

#define PRESCALE_QSPI 3

#define STARTMM 0x90000000                                                      
#define ENDMM   0x9FFFFFFF

#define WRITE 0
#define READ 1

#define FIFO_FULL 16
#define FIFO_EMPTY 0

#define FIFO_DEPTH_8  16
#define FIFO_DEPTH_16 FIFO_DEPTH_8/2
#define FIFO_DEPTH_32 FIFO_DEPTH_8/4

#define QSPI0 0
#define QSPI1 1

#define CCR_IMODE_NIL             0x0
#define CCR_IMODE_SINGLE_LINE     0x1
#define CCR_IMODE_TWO_LINE        0x2
#define CCR_IMODE_FOUR_LINE       0x3
#define CCR_ADMODE_NIL            0x0
#define CCR_ADMODE_SINGLE_LINE    0x1
#define CCR_ADMODE_TWO_LINE       0x2
#define CCR_ADMODE_FOUR_LINE      0x3
#define CCR_ADSIZE_8_BIT          0x0
#define CCR_ADSIZE_16_BIT         0x1
#define CCR_ADSIZE_24_BIT         0x2
#define CCR_ADSIZE_32_BIT         0x3
#define CCR_ABMODE_NIL            0x0
#define CCR_ABMODE_SINGLE_LINE    0x1
#define CCR_ABMODE_TW0_LINE       0x2
#define CCR_ABMODE_FOUR_LINE      0x3
#define CCR_ABSIZE_8_BIT          0x0
#define CCR_ABSIZE_16_BIT         0x1
#define CCR_ABSIZE_24_BIT         0x2
#define CCR_ABSIZE_32_BIT         0x3
#define CCR_DMODE_NO_DATA         0x0
#define CCR_DMODE_SINGLE_LINE     0x1
#define CCR_DMODE_TWO_LINE        0x2
#define CCR_DMODE_FOUR_LINE       0x3
#define CCR_FMODE_INDIRECT_WRITE  0x0
#define CCR_FMODE_INDIRECT_READ   0x1
#define CCR_FMODE_APM             0x2
#define CCR_FMODE_MMM             0x3
#define CCR_MM_MODE_XIP           0x0
#define CCR_MM_MODE_RAM           0x1

//! Enable QSPI_DEBUG for debugging purposes.
// #define QSPI_DEBUG 1

//Defines for configuring the registers at ease
//Bit vectors for all the parameters in the CR
#define CR_PRESCALER(x)   (x<<24)//8bit
#define CR_PMM(x)         (x<<23)
#define CR_APMS(x)        (x<<22)
#define CR_TOIE(x)        (x<<20)//1bit
#define CR_SMIE(x)        (x<<19)
#define CR_FTIE(x)        (x<<18)
#define CR_TCIE(x)        (x<<17)
#define CR_TEIE(x)        (x<<16)
#define CR_FTHRES(x)      (x<<8 )//4bit
#define CR_FSEL(x)        (x<<7 )//Not used
#define CR_DFM(x)         (x<<6 )//Not used
#define CR_SSHIF(x)       (x<<4 )//Not used 1bit
#define CR_TCEN(x)        (x<<3 )
#define CR_DMAEN(x)       (x<<2 )//Not used
#define CR_ABORT(x)       (x<<1 )
#define CR_EN(x)          (x<<0 )

//Bit vectors for DCR 
#define DCR_MODE_BYTE(x)   (x<<21)
#define DCR_FSIZE(x)       (x<<16)//5bit 
#define DCR_CSHT(x)        (x<<8 )//3bit Not used
#define DCR_CKMODE(x)        (x)//1bit 

//Bit vectors for status register
#define SR_FLEVEL(x)      (x<<8)//5bit
#define SR_BUSY           (1<<5)//1bit
#define SR_TOF            (1<<4)
#define SR_SMF            (1<<3)
#define SR_FTF            (1<<2)
#define SR_TCF            (1<<1)
#define SR_TEF            (1<<0)

//Bit vectors for flag clear register 
#define FCR_CTOF (1<<4)
#define FCR_CSMF (1<<3)
#define FCR_CTCF (1<<1)//1bit
#define FCR_CTEF (1<<0)

//Bit vectors for CCR
#define CCR_DDRM(x)                (x<<31) 
/* #define CCR_DHHC(x)                (x<<30)//Not used */
#define CCR_MM_MODE(x)             (x<<30) //memory Map mode XIP=0;RAM=1;
#define CCR_DUMMY_BIT(x)           (x<<29) // Needed by Micron Flash Memories
#define CCR_SIOO(x)                (x<<28)
#define CCR_FMODE(x)               (x<<26)
#define CCR_DMODE(x)               (x<<24)
#define CCR_DUMMY_CONFIRMATION(x)  (x<<23) // Needed by Micron Flash Memories
#define CCR_DCYC(x)                (x<<18)
#define CCR_ABSIZE(x)              (x<<16)
#define CCR_ABMODE(x)              (x<<14)
#define CCR_ADSIZE(x)              (x<<12)
#define CCR_ADMODE(x)              (x<<10)
#define CCR_IMODE(x)               (x<<8 )
#define CCR_INSTRUCTION(x)         (x<<0 )

#define FLASH_4K_ERASE             4096
#define FLASH_32K_ERASE            32768
#define FLASH_CHIP_ERASE           134217728            

#ifdef __cplusplus
}
#endif

#endif // FLASH_DRIVER_H