/* drivers/memc/mmemc_qspi_ap_psram_mindgrove.c */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

#define DT_DRV_COMPAT mindgrove_qspi_psram

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
#define CLOCK_FREQUENCY_ASIC        700000000UL

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


/* PSRAM msg instance */
static qspi_msg psram_msg = {
    .PRESCALER = 20,
    .CLK_MODE  = 0,
    .FMEM_SIZE = 27,
    .csht      = 7,
};

struct psram_config {
    const QUADSPI_Type *qspi;
    uint32_t size;
    uint8_t  prescaler;
};

struct psram_data {
    bool initialized;
};

#define POW2_MINUS1(n)   ((1U << (n)) - 1U)

/* QSPI_Transaction*/

uint16_t QSPI_Transaction(qspi_msg *msg) {
    uint32_t remaining;
    volatile QUADSPI_Type *qspi_regs;
    void *word_ptr;
    uint64_t *word64;
    uint32_t *word32;
    uint16_t *word16;
    uint8_t *word8;

    if (msg == NULL) {
        return -EFAULT;
    }

    if (msg->qspi_inst == NULL) {
        return -EFAULT;
    }

    if ((msg->data_mode != CCR_DMODE_NO_DATA) &&
        (msg->functional_mode != CCR_FMODE_MMM) &&
        (msg->data_buffer == NULL)) {
        return -EFAULT;
    }

    /**
     * Clock validation: QSPI clock = 30MHz / (PRESCALER + 1)
     * Must not exceed MAX_QSPI_FREQ.
     */
    if (((uint64_t)CLOCK_FREQUENCY_ASIC /
            ((uint64_t)msg->PRESCALER + 1ULL)) > MAX_QSPI_FREQ) {
        return EPERM;
    }

    qspi_regs = (volatile QUADSPI_Type *)msg->qspi_inst;

    /* Wait for QSPI controller to become idle before reconfiguration */
    while ((qspi_regs->SR & SR_BUSY) != 0U) { }

    /**
     * Configure QSPI Control Register (CR):
     * PRESCALER, PMM, APMS, interrupt enables, TCEN, EN
     */
    qspi_regs->CR =
        CR_PRESCALER(msg->PRESCALER) | CR_PMM(msg->PMM) | CR_APMS(msg->APMS) |
        CR_TOIE(msg->TOIE) | CR_SMIE(msg->SMIE) | CR_FTIE(msg->FTIE) |
        CR_TCIE(msg->TCIE) | CR_TEIE(msg->TEIE) | CR_TCEN(msg->TCEN) |
        CR_EN(1U);

    /**
     * Configure Device Configuration Register (DCR):
     * FSIZE, CKMODE, CSHT
     */
    qspi_regs->DCR =
        DCR_FSIZE(msg->FMEM_SIZE) | DCR_CKMODE(msg->CLK_MODE) |
        DCR_CSHT(msg->csht);

    /* Clear all pending flags before initiating new transaction */
    qspi_regs->FCR = (FCR_CTOF | FCR_CSMF | FCR_CTCF | FCR_CTEF);

    /* Configure data length (register uses 0-based count) */
    if (msg->length > 0U) {
        qspi_regs->DLR = msg->length - 1U;
    }

    /**
     * Configure Communication Configuration Register (CCR):
     * instruction, address, alternate byte, dummy cycles, data, functional mode
     */
    qspi_regs->CCR =
        CCR_INSTRUCTION(msg->instruction) | CCR_IMODE(msg->instruction_mode) |
        CCR_ADMODE(msg->address_mode) | CCR_ADSIZE(msg->address_size) |
        CCR_ABMODE(msg->alternate_byte_mode) |
        CCR_ABSIZE(msg->alternate_byte_size) | CCR_DCYC(msg->dummy_cycles) |
        CCR_DUMMY_CONFIRMATION(msg->dummy_mode) | CCR_DMODE(msg->data_mode) |
        CCR_FMODE(msg->functional_mode) | CCR_SIOO(msg->sioo) |
        CCR_DUMMY_BIT(msg->dummy_bit) | CCR_MM_MODE(msg->mm_mode);

    if ((msg->functional_mode == CCR_FMODE_MMM) &&
            (msg->mm_mode == CCR_MM_MODE_RAM)) {
        /* Memory-Mapped RAM mode */
        qspi_regs->RMC = RMC_WDCYC(msg->wr_dcyc) | RMC_RDCYC(msg->rd_dcyc) |
                    RMC_WINSTR(msg->wr_instr) | RMC_RINSTR(msg->rd_instr);

    } else if ((msg->functional_mode == CCR_FMODE_INDIRECT_READ) ||
               (msg->functional_mode == CCR_FMODE_INDIRECT_WRITE)) {
        /* Indirect Read/Write mode */

        if (msg->alternate_byte_mode != CCR_ABMODE_NIL) {
            qspi_regs->ABR = msg->alternate_byte;
        }

        if (msg->address_mode != CCR_ADMODE_NIL) {
            qspi_regs->AR = msg->address;
        }

        remaining = msg->length;
        word_ptr = msg->data_buffer;

        /* 64-bit burst transfers */
        qspi_regs->CR &= ~CR_FTHRES_MASK;
        qspi_regs->CR |= CR_FTHRES(7U);

        while (IS_ALIGNED(word_ptr, 8U) && (remaining >= 8U)) {
            word64 = (uint64_t *)word_ptr;
            while ((qspi_regs->SR & SR_FTF) == 0U) { }
            if (msg->functional_mode == CCR_FMODE_INDIRECT_WRITE) {
                qspi_regs->DR.data_64 = *word64;
            } else {
                *word64 = qspi_regs->DR.data_64;
            }
            word_ptr = (void *)((uint8_t *)word_ptr + 8U);
            remaining -= 8U;
        }

        /* 32-bit burst transfers */
        qspi_regs->CR &= ~CR_FTHRES_MASK;
        qspi_regs->CR |= CR_FTHRES(3U);

        while (IS_ALIGNED(word_ptr, 4U) && (remaining >= 4U)) {
            word32 = (uint32_t *)word_ptr;
            while ((qspi_regs->SR & SR_FTF) == 0U) { }
            if (msg->functional_mode == CCR_FMODE_INDIRECT_WRITE) {
                qspi_regs->DR.data_32 = *word32;
            } else {
                *word32 = qspi_regs->DR.data_32;
            }
            word_ptr = (void *)((uint8_t *)word_ptr + 4U);
            remaining -= 4U;
        }

        /* 16-bit burst transfers */
        qspi_regs->CR &= ~CR_FTHRES_MASK;
        qspi_regs->CR |= CR_FTHRES(1U);

        while (IS_ALIGNED(word_ptr, 2U) && (remaining >= 2U)) {
            word16 = (uint16_t *)word_ptr;
            while ((qspi_regs->SR & SR_FTF) == 0U) { }
            if (msg->functional_mode == CCR_FMODE_INDIRECT_WRITE) {
                qspi_regs->DR.data_16 = *word16;
            } else {
                *word16 = qspi_regs->DR.data_16;
            }
            word_ptr = (void *)((uint8_t *)word_ptr + 2U);
            remaining -= 2U;
        }

        /* 8-bit transfers for remaining bytes */
        qspi_regs->CR &= ~CR_FTHRES_MASK;
        qspi_regs->CR |= CR_FTHRES(0U);

        while (remaining > 0U) {
            word8 = (uint8_t *)word_ptr;
            while ((qspi_regs->SR & SR_FTF) == 0U) { }
            if (msg->functional_mode == CCR_FMODE_INDIRECT_WRITE) {
                qspi_regs->DR.data_8 = *word8;
            } else {
                *word8 = qspi_regs->DR.data_8;
            }
            word_ptr = (void *)((uint8_t *)word_ptr + 1U);
            remaining -= 1U;
        }

        /* Wait for transfer completion and controller idle */
        while ((qspi_regs->SR & SR_TCF) == 0U) { }
        while ((qspi_regs->SR & SR_BUSY) != 0U) { }

        /* Disable QSPI controller to conserve power */
        qspi_regs->CR &= ~CR_EN(1U);

    } else if ((msg->functional_mode == CCR_FMODE_MMM) &&
               (msg->mm_mode == CCR_MM_MODE_XIP)) {
        /* XIP memory-mapped mode — no data transfer */

    } else if (msg->functional_mode == CCR_FMODE_APM) {
        /* Automatic polling mode */
        qspi_regs->PSMKR = msg->status_mask;
        qspi_regs->PSMAR = msg->status_match;

    } else {
        return -EINVAL;
    }

    return 0;
}
static int psram_init(const struct device *dev)
{
    const struct psram_config *cfg = dev->config;
    struct psram_data *data = dev->data;
    uint16_t ret;

    /* Reset Enable */
    psram_msg.address          = 0x0U;
    psram_msg.qspi_inst        = (volatile QUADSPI_Type *)cfg->qspi;
    psram_msg.address_mode     = CCR_ADMODE_NIL;
    psram_msg.address_size     = CCR_ADSIZE_24_BIT;
    psram_msg.instruction      = 0x66;
    psram_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    psram_msg.dummy_mode       = 0;
    psram_msg.data_mode        = CCR_DMODE_NO_DATA;
    psram_msg.functional_mode  = CCR_FMODE_INDIRECT_WRITE;
    psram_msg.dummy_cycles     = 1;
    psram_msg.dummy_bit        = 0;
    psram_msg.mm_mode          = CCR_MM_MODE_XIP;
    psram_msg.alternate_byte_mode = CCR_ABMODE_NIL;
    psram_msg.fthresh          = POW2_MINUS1(0);
    psram_msg.length           = 0;

    ret = QSPI_Transaction(&psram_msg);
    if (ret != 0) return -EIO;

    /* Reset Command */
    psram_msg.instruction = 0x99;
    ret = QSPI_Transaction(&psram_msg);
    if (ret != 0) return -EIO;

    /* Enter Quad Mode */
    psram_msg.instruction = 0x35;
    ret = QSPI_Transaction(&psram_msg);
    if (ret != 0) return -EIO;

    /* Enter RAM Mode - 4 line */
    psram_msg.mm_mode          = CCR_MM_MODE_RAM;
    psram_msg.functional_mode  = CCR_FMODE_MMM;
    psram_msg.instruction_mode = CCR_IMODE_FOUR_LINE;
    psram_msg.address_mode     = CCR_ADMODE_FOUR_LINE;
    psram_msg.data_mode        = CCR_DMODE_FOUR_LINE;
    psram_msg.wr_instr         = 0x38;
    psram_msg.wr_dcyc          = 0;
    psram_msg.rd_instr         = 0xEB;
    psram_msg.rd_dcyc          = 6;

    ret = QSPI_Transaction(&psram_msg);
    if (ret != 0) return -EIO;

    data->initialized = true;
    return 0;
}

#define PSRAM_DEFINE(inst)                                              \
    static struct psram_data psram_data_##inst;                        \
                                                                        \
    static const struct psram_config psram_cfg_##inst = {              \
        .qspi      = (const QUADSPI_Type *)DT_INST_REG_ADDR(inst),    \
        .size      = DT_INST_PROP(inst, size),                        \
    };                                                                 \
                                                                        \
    DEVICE_DT_INST_DEFINE(inst,                                        \
                          psram_init,                                  \
                          NULL,                                        \
                          &psram_data_##inst,                          \
                          &psram_cfg_##inst,                           \
                          POST_KERNEL,                                 \
                          CONFIG_MEMC_MINDGROVE_PSRAM_INIT_PRIORITY,   \
                          NULL);   /* no standard API for PSRAM */

DT_INST_FOREACH_STATUS_OKAY(PSRAM_DEFINE)