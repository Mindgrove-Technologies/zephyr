
#include <stdint.h>
#define MAX_QSPI_FREQ 75000000UL

// QSPI BARE METAL MACROS (from QSPI driver header, adapted for flash use)
#define WRITE 0
#define READ 1

typedef union{
  uint64_t data_64;
  uint32_t data_32;
  uint16_t data_16;
  uint8_t data_8;
} QSPI_Data;
/* ============================================================
 * General Constants
 * ============================================================*/
#define QSPI_WRITE     (0U)
#define QSPI_READ      (1U)

#define QSPI_FIFO_FULL   (16U)
#define QSPI_FIFO_EMPTY  (0U)

/* ============================================================
 * CCR Mode Macros
 * ============================================================*/

/* Instruction mode */
#define CCR_IMODE_NIL          (0x0U)
#define CCR_IMODE_SINGLE_LINE  (0x1U)
#define CCR_IMODE_TWO_LINE     (0x2U)
#define CCR_IMODE_FOUR_LINE    (0x3U)

/* Address mode */
#define CCR_ADMODE_NIL          (0x0U)
#define CCR_ADMODE_SINGLE_LINE  (0x1U)
#define CCR_ADMODE_TWO_LINE     (0x2U)
#define CCR_ADMODE_FOUR_LINE    (0x3U)

/* Address size */
#define CCR_ADSIZE_8_BIT   (0x0U)
#define CCR_ADSIZE_16_BIT  (0x1U)
#define CCR_ADSIZE_24_BIT  (0x2U)
#define CCR_ADSIZE_32_BIT  (0x3U)

/* Alternate byte mode */
#define CCR_ABMODE_NIL          (0x0U)
#define CCR_ABMODE_SINGLE_LINE  (0x1U)
#define CCR_ABMODE_TWO_LINE     (0x2U)
#define CCR_ABMODE_FOUR_LINE    (0x3U)

/* Alternate byte size */
#define CCR_ABSIZE_8_BIT   (0x0U)
#define CCR_ABSIZE_16_BIT  (0x1U)
#define CCR_ABSIZE_24_BIT  (0x2U)
#define CCR_ABSIZE_32_BIT  (0x3U)

/* Data mode */
#define CCR_DMODE_NO_DATA      (0x0U)
#define CCR_DMODE_SINGLE_LINE  (0x1U)
#define CCR_DMODE_TWO_LINE     (0x2U)
#define CCR_DMODE_FOUR_LINE    (0x3U)

/* Functional mode */
#define CCR_FMODE_INDIRECT_WRITE  (0x0U)
#define CCR_FMODE_INDIRECT_READ   (0x1U)
#define CCR_FMODE_APM             (0x2U)
#define CCR_FMODE_MMM             (0x3U)

/* Memory mapped mode */
#define CCR_MM_MODE_XIP  (0x0U)
#define CCR_MM_MODE_RAM  (0x1U)

/* Poll match mode */
#define PMM_AND  (0U)
#define PMM_OR   (1U)

/* ============================================================
 * Register Bitfield Macros
 * ============================================================*/

#define CR_FTHRES_MASK   (0xFU << 7U)

#define CR_PRESCALER(x)  ((uint32_t)(x) << 18U)
#define CR_PMM(x)        ((uint32_t)(x) << 17U)
#define CR_APMS(x)       ((uint32_t)(x) << 16U)
#define CR_TOIE(x)       ((uint32_t)(x) << 15U)
#define CR_SMIE(x)       ((uint32_t)(x) << 14U)
#define CR_FTIE(x)       ((uint32_t)(x) << 13U)
#define CR_TCIE(x)       ((uint32_t)(x) << 12U)
#define CR_TEIE(x)       ((uint32_t)(x) << 11U)
#define CR_FTHRES(x)     ((uint32_t)(x) << 7U)
#define CR_TCEN(x)       ((uint32_t)(x) << 3U)
#define CR_ABORT(x)      ((uint32_t)(x) << 1U)
#define CR_EN(x)         ((uint32_t)(x) << 0U)

/* DCR */
#define DCR_MODE_BYTE(x) ((uint32_t)(x) << 21U)
#define DCR_FSIZE(x)     ((uint32_t)(x) << 16U)
#define DCR_CSHT(x)      ((uint32_t)(x) << 8U)
#define DCR_CKMODE(x)    ((uint32_t)(x))

/* Status register */
#define SR_FLEVEL   (1U << 8)
#define SR_BUSY     (1U << 5)
#define SR_TOF      (1U << 4)
#define SR_SMF      (1U << 3)
#define SR_FTF      (1U << 2)
#define SR_TCF      (1U << 1)
#define SR_TEF      (1U << 0)

/* Flag clear */
#define FCR_CTOF  (1U << 4)
#define FCR_CSMF  (1U << 3)
#define FCR_CTCF  (1U << 1)
#define FCR_CTEF  (1U << 0)

/* CCR */
#define CCR_DDRM(x)                ((uint32_t)(x) << 31U)
#define CCR_MM_MODE(x)             ((uint32_t)(x) << 30U)
#define CCR_DUMMY_BIT(x)           ((uint32_t)(x) << 29U)
#define CCR_SIOO(x)                ((uint32_t)(x) << 28U)
#define CCR_FMODE(x)               ((uint32_t)(x) << 26U)
#define CCR_DMODE(x)               ((uint32_t)(x) << 24U)
#define CCR_DUMMY_CONFIRMATION(x)  ((uint32_t)(x) << 23U)
#define CCR_DCYC(x)                ((uint32_t)(x) << 18U)
#define CCR_ABSIZE(x)              ((uint32_t)(x) << 16U)
#define CCR_ABMODE(x)              ((uint32_t)(x) << 14U)
#define CCR_ADSIZE(x)              ((uint32_t)(x) << 12U)
#define CCR_ADMODE(x)              ((uint32_t)(x) << 10U)
#define CCR_IMODE(x)               ((uint32_t)(x) << 8U)
#define CCR_INSTRUCTION(x)         ((uint32_t)(x))

/* RMC */
#define RMC_WDCYC(x)   ((uint32_t)(x) << 21U)
#define RMC_RDCYC(x)   ((uint32_t)(x) << 16U)
#define RMC_WINSTR(x)  ((uint32_t)(x) << 8U)
#define RMC_RINSTR(x)  ((uint32_t)(x))

#define MAX_QSPI_FREQ 75000000UL
#define CLOCK_FREQUENCY_FPGA        30000000UL

//ll
typedef struct {                                /*!< QUADSPI0 Structure                                                        */
  
  union {
    volatile uint32_t CR;                          /*!< Control Register                                                          */
    
    struct {
      volatile uint32_t EN         : 1;            /*!< QSPI Communication Enable                                                 */
      volatile uint32_t ABORT      : 1;            /*!< QSPI Communication Abort request                                          */
      volatile uint32_t DMAEN      : 1;            /*!< DMA Transfer enable                                                       */
      volatile uint32_t TCEN       : 1;            /*!< Timeout counter enable                                                    */
            uint32_t            : 3;
      volatile uint32_t FTHRES     : 4;            /*!< IFO threshold level                                                       */
      volatile uint32_t TEIE       : 1;            /*!< Transfer error interrupt enable                                           */
      volatile uint32_t TCIE       : 1;            /*!< Transfer complete interrupt enable                                        */
      volatile uint32_t FTIE       : 1;            /*!< FIFO threshold interrupt enable                                           */
      volatile uint32_t SMIE       : 1;            /*!< Status match interrupt enable                                             */
      volatile uint32_t TOIE       : 1;            /*!< TimeOut interrupt enable                                                  */
      volatile uint32_t APMS       : 1;            /*!< Automatic poll mode stop                                                  */
      volatile uint32_t PMM        : 1;            /*!< Polling match mode                                                        */
      volatile uint32_t PRESCALER  : 8;            /*!< Clock prescaler                                                           */
            uint32_t            : 6;
    } CR_b;
  } ;
  
  union {
    volatile uint32_t DCR;                         /*!< Device Configuration Register                                             */
    
    struct {
      volatile uint32_t CKMODE     : 1;            /*!< Mode 0 / mode 3                                                           */
            uint32_t            : 7;
      volatile uint32_t CSHT       : 3;            /*!< Chip select high time                                                     */
            uint32_t            : 5;
      volatile uint32_t FSIZE      : 5;            /*!< FLASH memory size                                                         */
      volatile uint32_t MODE_Byte  : 8;            /*!< Dummy Cycle Mode Byte for Micron Flash                                    */
            uint32_t            : 3;
    } DCR_b;
  } ;
  
  union {
    volatile  uint32_t SR;                          /*!< Status Register                                                           */
    
    struct {
      volatile  uint32_t TEF        : 1;            /*!< Transfer error flag                                                       */
      volatile  uint32_t TCF        : 1;            /*!< Transfer complete flag                                                    */
      volatile  uint32_t FTF        : 1;            /*!< FIFO threshold flag                                                       */
      volatile  uint32_t SMF        : 1;            /*!< Status match flag                                                         */
      volatile  uint32_t TOF        : 1;            /*!< Timeout flag                                                              */
      volatile  uint32_t BUSY       : 1;            /*!< Busy                                                                      */
            uint32_t            : 2;
      volatile  uint32_t FLEVEL     : 7;            /*!< FIFO level                                                                */
            uint32_t            : 17;
    } SR_b;
  } ;
  
  union {
    volatile uint32_t FCR;                         /*!< Flag Clear Register                                                       */
    
    struct {
      volatile uint32_t CTEF       : 1;            /*!< Clear transfer error flag                                                 */
      volatile uint32_t CTCF       : 1;            /*!< Clear transfer complete flag                                              */
            uint32_t            : 1;
      volatile uint32_t CSMF       : 1;            /*!< Clear status match flag                                                   */
      volatile uint32_t CTOF       : 1;            /*!< Clear timeout flag                                                        */
            uint32_t            : 27;
    } FCR_b;
  } ;
  
  union {
    volatile uint32_t DLR;                         /*!< data length register                                                      */
    
    struct {
      volatile uint32_t DL         : 32;           /*!< Data length                                                               */
    } DLR_b;
  } ;
  
  union {
    volatile uint32_t CCR;                         /*!< communication configuration register                                      */
    
    struct {
      volatile uint32_t INSTRUCTION : 8;           /*!< Instruction                                                               */
      volatile uint32_t IMODE      : 2;            /*!< Instruction mode                                                          */
      volatile uint32_t ADMODE     : 2;            /*!< Address mode                                                              */
      volatile uint32_t ADSIZE     : 2;            /*!< Address size                                                              */
      volatile uint32_t ABMODE     : 2;            /*!< Alternate bytes mode                                                      */
      volatile uint32_t ABSIZE     : 2;            /*!< Alternate bytes size                                                      */
      volatile uint32_t DCYC       : 5;            /*!< Number of dummy cycles                                                    */
            uint32_t            : 1;
      volatile uint32_t DMODE      : 2;            /*!< Data mode                                                                 */
      volatile uint32_t FMODE      : 2;            /*!< Functional mode                                                           */
      volatile uint32_t SIOO       : 1;            /*!< Send instruction only once mode                                           */
      volatile uint32_t DUMMY_BIT  : 1;            /*!< Set value 1 to send Dummy Cycles. Default 0                               */
      volatile uint32_t MM_MODE    : 1;            /*!< Memory Mapped Mode. Default value 0. For XIP Mode, set 0, for
                                                     RAM Mode, set 1.                                                          */
            uint32_t            : 1;
    } CCR_b;
  } ;
  
  union {
    volatile uint32_t AR;                          /*!< Address Register                                                          */
    
    struct {
      volatile uint32_t ADDRESS    : 32;           /*!< Address                                                                   */
    } AR_b;
  } ;
  
  union {
    volatile uint32_t ABR;                         /*!< Alternate Byte Register                                                   */
    
    struct {
      volatile uint32_t ALTERNATE  : 32;           /*!< Alternate Byte                                                            */
    } ABR_b;
  } ;
  // volatile  uint32_t  RESERVED[2];

  volatile QSPI_Data DR;                            /*!< Data Register                                                             */
  
  union {
    volatile uint32_t PSMKR;                       /*!< Polling Status Mask Register                                              */
    
    struct {
      volatile uint32_t MASK       : 32;           /*!< Status mask                                                               */
    } PSMKR_b;
  } ;
  
  union {
    volatile uint32_t PSMAR;                       /*!< Polling Status Match Register                                             */
    
    struct {
      volatile uint32_t Match      : 32;           /*!< Status Match                                                              */
    } PSMAR_b;
  } ;
  volatile  uint32_t  RESERVED1;
  
  union {
    volatile uint32_t LPTR;                        /*!< Low Power Timeout Register                                                */
    
    struct {
      volatile uint32_t Match      : 32;           /*!< Status Match                                                              */
    } LPTR_b;
  } ;
  
  union {
    volatile uint32_t RMC;                         /*!< RAM Mode Configuration Register                                           */
    
    struct {
      volatile uint32_t WDCYC      : 5;            /*!< RAM Write Mode Dummy Cycle Count                                          */
      volatile uint32_t RDCYC      : 5;            /*!< RAM Read Mode Dummy Cycle Count                                           */
      volatile uint32_t WINST      : 8;            /*!< RAM Write Mode Instruction                                                */
      volatile uint32_t RINST      : 8;            /*!< RAM Read Mode Instruction                                                 */
            uint32_t            : 6;
    } RMC_b;
  } ;
  
} QUADSPI_Type;                                /*!< Size = 60 (0x3c)                                                          */

typedef struct {
    volatile QUADSPI_Type *qspi_inst;

    uint8_t functional_mode;
    uint8_t instruction;
    uint8_t instruction_mode;

    uint8_t address_mode;
    uint8_t address_size;
    uint32_t address;

    uint8_t alternate_byte_mode;
    uint32_t alternate_byte;
    uint8_t alternate_byte_size;

    uint8_t dummy_mode  :1;
    uint8_t dummy_bit   :1;
    uint8_t dummy_cycles:5;
    uint8_t sioo        :1;
    uint8_t mm_mode     :1;

    uint8_t data_mode;
    uint32_t length;
    uint8_t *data_buffer;

    uint8_t FMEM_SIZE;
    uint8_t CLK_MODE:1;
    uint8_t fthresh;
    uint8_t csht;

    uint32_t TCEN:1;
    uint32_t TEIE:1;
    uint32_t TCIE:1;
    uint32_t FTIE:1;
    uint32_t SMIE:1;
    uint32_t TOIE:1;
    uint32_t APMS:1;
    uint32_t PMM:1;
    uint32_t PRESCALER:14;

    uint32_t status_mask;
    uint32_t status_match;

    uint32_t rd_instr:8;
    uint32_t wr_instr:8;
    uint32_t rd_dcyc:5;
    uint32_t wr_dcyc:5;
} qspi_msg;



/* ================================
 * FLASH COMMANDS (JEDEC standard)
 * ================================ */

/* Read */
#define FLASH_CMD_READ                0x03   /* 1-1-1 */
#define FLASH_CMD_FAST_READ           0x0B   /* 1-1-1 fast */
#define FLASH_CMD_FAST_READ_QUAD      0x6B   /* 1-1-4 */
#define FLASH_CMD_FAST_READ_QUAD_IO   0xEB   /* 1-4-4 */

/* Program */
#define FLASH_CMD_PAGE_PROGRAM        0x02   /* 1-1-1 */
#define FLASH_CMD_QUAD_PAGE_PROGRAM   0x32   /* 1-1-4 */

/* Erase */
#define FLASH_CMD_ERASE_4K            0x20
#define FLASH_CMD_ERASE_32K           0x52
#define FLASH_CMD_ERASE_CHIP          0xC7

/* Control */
#define FLASH_CMD_WRITE_ENABLE        0x06
#define FLASH_CMD_WRITE_DISABLE       0x04

/* Status */
#define FLASH_CMD_READ_SR1            0x05
#define FLASH_CMD_READ_SR2            0x35
#define FLASH_CMD_READ_SR3            0x15

/* IDs */
#define FLASH_CMD_READ_JEDEC_ID       0x9F
#define FLASH_CMD_READ_SFDP           0x5A