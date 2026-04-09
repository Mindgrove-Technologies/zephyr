#include <stdint.h>

/* =========================================================================================================================== */
/* ================                                            DMA                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief Direct Memory Access controller. (DMA)
  */

typedef struct {                                /*!< DMA Structure                                                             */
  
  union {
    volatile uint32_t CONFIG_REG;                    /*!< Channel configuration regisster of channel                              */
    
    struct {
      volatile uint32_t CHANNEL_EN         : 1;            /*!< To enable the channel                                                     */
      volatile uint32_t TRANSFER_COMPLETE_INT_EN    : 1;            /*!< Transfer complete interrupt enable                                        */
      volatile uint32_t HALF_TRANSFER_INT_EN       : 1;            /*!< Half transfer interrupt enable                                            */
      volatile uint32_t TRANSFER_ERROR_INT_EN       : 1;            /*!< Transfer error interrupt enable                                           */
      volatile uint32_t TRANSFER_DIRECTION        : 1;            /*!< Data transfer direction                                                   */
            uint32_t            : 1;
      volatile uint32_t PERIPH_ADDR_INCREMENT       : 2;            /*!< Peripheral increment mode                                                  */
      volatile uint32_t MEM_ADDR_INCREMENT       : 2;            /*!< Memory increment mode                                                     */
      volatile uint32_t PERIPH_TRANSFER_SIZE      : 2;            /*!< Data size of each DMA transfer to the peripheral                          */
      volatile uint32_t MEM_TRANSFER_SIZE      : 2;            /*!< Data size of each DMA transfer to the memory                              */
      volatile uint32_t PRIORITY_LEVEL         : 2;            /*!< Priority level                                                            */
      volatile uint32_t MEMORY_TO_MEMORY    : 1;            /*!< Memory to memory mode                                                     */
            uint32_t            : 1;
      volatile uint32_t PERIPH_TO_PERIPH        : 1;            /*!< Periheral to peripheral mode                                              */
            uint32_t            : 13;
    } DMA_CCR_b;
  } ;
  volatile  uint32_t  RESERVED;
  
  union {
    volatile uint32_t TRANSFER_LENGTH_REG;         /* < Register specifying the number of bytes to be transferred by the DMA channel */
    
  } ;
  volatile  uint32_t  RESERVED2;
  
  union {
    volatile uint32_t PERIPH_ADDR_REG;                   /*!< Channel peripheral address register                                      */
    
  } ;
  volatile  uint32_t  RESERVED3;
  
  union {
    volatile uint32_t MEM_ADDR_REG;                   /*!< Channel memory address register                                          */
  
  } ;
  volatile  uint32_t  RESERVED4;
  
  union {
    volatile uint16_t REQUEST_SELECT_REG;          /*!< Channel selection register                                               */
    
    struct {
      volatile uint16_t SRC_REQUEST_ID : 6;           /*!< Selects the source peripheral request line for the DMA channel.        */
      volatile uint16_t DEST_REQUEST_ID : 6;           /*!< Selects the destination peripheral request line for the DMA channel.      */
            uint16_t            : 4;
    } REQUEST_SELECT_REG_b;
  } ;
  volatile  uint16_t  RESERVED5;
  volatile  uint32_t  RESERVED6;
  
}  DMA_CHANNEL_Type;

typedef struct { 

  DMA_CHANNEL_Type CHANNEL[8];

  union {
      volatile uint32_t INTERRUPT_STATUS_REG; /*!< Interrupt status register */

      struct {
          volatile uint32_t GLOBAL_INT_FLAG_CH0        : 1; /*!< global interrupt flag for channel0 */
          volatile uint32_t TRANSFER_COMPLETE_FLAG_CH0 : 1; /*!< Transfer completion flag for channel0 */
          volatile uint32_t HALF_TRANSFER_FLAG_CH0     : 1; /*!< Half transfer flag for channel0 */
          volatile uint32_t TRANSFER_ERROR_FLAG_CH0    : 1; /*!< Transfer error flag for channel0 */

          volatile uint32_t GLOBAL_INT_FLAG_CH1        : 1; /*!< global interrupt flag for channel1 */
          volatile uint32_t TRANSFER_COMPLETE_FLAG_CH1 : 1; /*!< Transfer completion flag for channel1 */
          volatile uint32_t HALF_TRANSFER_FLAG_CH1     : 1; /*!< Half transfer flag for channel1 */
          volatile uint32_t TRANSFER_ERROR_FLAG_CH1    : 1; /*!< Transfer error flag for channel1 */

          volatile uint32_t GLOBAL_INT_FLAG_CH2        : 1; /*!< global interrupt flag for channel2 */
          volatile uint32_t TRANSFER_COMPLETE_FLAG_CH2 : 1; /*!< Transfer completion flag for channel2 */
          volatile uint32_t HALF_TRANSFER_FLAG_CH2     : 1; /*!< Half transfer flag for channel2 */
          volatile uint32_t TRANSFER_ERROR_FLAG_CH2    : 1; /*!< Transfer error flag for channel2 */

          volatile uint32_t GLOBAL_INT_FLAG_CH3        : 1; /*!< global interrupt flag for channel3 */
          volatile uint32_t TRANSFER_COMPLETE_FLAG_CH3 : 1; /*!< Transfer completion flag for channel3 */
          volatile uint32_t HALF_TRANSFER_FLAG_CH3     : 1; /*!< Half transfer flag for channel3 */
          volatile uint32_t TRANSFER_ERROR_FLAG_CH3    : 1; /*!< Transfer error flag for channel3 */

          volatile uint32_t GLOBAL_INT_FLAG_CH4        : 1; /*!< global interrupt flag for channel4 */
          volatile uint32_t TRANSFER_COMPLETE_FLAG_CH4 : 1; /*!< Transfer completion flag for channel4 */
          volatile uint32_t HALF_TRANSFER_FLAG_CH4     : 1; /*!< Half transfer flag for channel4 */
          volatile uint32_t TRANSFER_ERROR_FLAG_CH4    : 1; /*!< Transfer error flag for channel4 */

          volatile uint32_t GLOBAL_INT_FLAG_CH5        : 1; /*!< global interrupt flag for channel5 */
          volatile uint32_t TRANSFER_COMPLETE_FLAG_CH5 : 1; /*!< Transfer completion flag for channel5 */
          volatile uint32_t HALF_TRANSFER_FLAG_CH5     : 1; /*!< Half transfer flag for channel5 */
          volatile uint32_t TRANSFER_ERROR_FLAG_CH5    : 1; /*!< Transfer error flag for channel5 */

          volatile uint32_t GLOBAL_INT_FLAG_CH6        : 1; /*!< global interrupt flag for channel6 */
          volatile uint32_t TRANSFER_COMPLETE_FLAG_CH6 : 1; /*!< Transfer completion flag for channel6 */
          volatile uint32_t HALF_TRANSFER_FLAG_CH6     : 1; /*!< Half transfer flag for channel6 */
          volatile uint32_t TRANSFER_ERROR_FLAG_CH6    : 1; /*!< Transfer error flag for channel6 */

          volatile uint32_t GLOBAL_INT_FLAG_CH7        : 1; /*!< global interrupt flag for channel7 */
          volatile uint32_t TRANSFER_COMPLETE_FLAG_CH7 : 1; /*!< Transfer completion flag for channel7 */
          volatile uint32_t HALF_TRANSFER_FLAG_CH7     : 1; /*!< Half transfer flag for channel7 */
          volatile uint32_t TRANSFER_ERROR_FLAG_CH7    : 1; /*!< Transfer error flag for channel7 */
      } ISR_b;
  };
  volatile  uint32_t  RESERVED56;
  
union {
    volatile uint32_t INT_FLAG_CLEAR_REG; /*!< Interrupt flag clear register */

    struct {
        volatile uint32_t CLEAR_GLOBAL_FLAG_CH0            : 1; /*!< global interrupt flag clear for channel0 */
        volatile uint32_t CLEAR_TRANSFER_COMPLETE_FLAG_CH0 : 1; /*!< Transfer completion flag clear channel0 */
        volatile uint32_t CLEAR_HALF_TRANSFER_FLAG_CH0     : 1; /*!< Half transfer flag clear channel0 */
        volatile uint32_t CLEAR_TRANSFER_ERROR_FLAG_CH0    : 1; /*!< Transfer error clear channel0 */

        volatile uint32_t CLEAR_GLOBAL_FLAG_CH1            : 1; /*!< global interrupt flag clear for channel1 */
        volatile uint32_t CLEAR_TRANSFER_COMPLETE_FLAG_CH1 : 1; /*!< Transfer completion flag clear channel1 */
        volatile uint32_t CLEAR_HALF_TRANSFER_FLAG_CH1     : 1; /*!< Half transfer flag clear channel1 */
        volatile uint32_t CLEAR_TRANSFER_ERROR_FLAG_CH1    : 1; /*!< Transfer error clear channel1 */

        volatile uint32_t CLEAR_GLOBAL_FLAG_CH2            : 1; /*!< global interrupt flag clear for channel2 */
        volatile uint32_t CLEAR_TRANSFER_COMPLETE_FLAG_CH2 : 1; /*!< Transfer completion flag clear channel2 */
        volatile uint32_t CLEAR_HALF_TRANSFER_FLAG_CH2     : 1; /*!< Half transfer flag clear channel2 */
        volatile uint32_t CLEAR_TRANSFER_ERROR_FLAG_CH2    : 1; /*!< Transfer error clear channel2 */

        volatile uint32_t CLEAR_GLOBAL_FLAG_CH3            : 1; /*!< global interrupt flag clear for channel3 */
        volatile uint32_t CLEAR_TRANSFER_COMPLETE_FLAG_CH3 : 1; /*!< Transfer completion flag clear channel3 */
        volatile uint32_t CLEAR_HALF_TRANSFER_FLAG_CH3     : 1; /*!< Half transfer flag clear channel3 */
        volatile uint32_t CLEAR_TRANSFER_ERROR_FLAG_CH3    : 1; /*!< Transfer error clear channel3 */

        volatile uint32_t CLEAR_GLOBAL_FLAG_CH4            : 1; /*!< global interrupt flag clear for channel4 */
        volatile uint32_t CLEAR_TRANSFER_COMPLETE_FLAG_CH4 : 1; /*!< Transfer completion flag clear channel4 */
        volatile uint32_t CLEAR_HALF_TRANSFER_FLAG_CH4     : 1; /*!< Half transfer flag clear channel4 */
        volatile uint32_t CLEAR_TRANSFER_ERROR_FLAG_CH4    : 1; /*!< Transfer error clear channel4 */

        volatile uint32_t CLEAR_GLOBAL_FLAG_CH5            : 1; /*!< global interrupt flag clear for channel5 */
        volatile uint32_t CLEAR_TRANSFER_COMPLETE_FLAG_CH5 : 1; /*!< Transfer completion flag clear channel5 */
        volatile uint32_t CLEAR_HALF_TRANSFER_FLAG_CH5     : 1; /*!< Half transfer flag clear channel5 */
        volatile uint32_t CLEAR_TRANSFER_ERROR_FLAG_CH5    : 1; /*!< Transfer error clear channel5 */

        volatile uint32_t CLEAR_GLOBAL_FLAG_CH6            : 1; /*!< global interrupt flag clear for channel6 */
        volatile uint32_t CLEAR_TRANSFER_COMPLETE_FLAG_CH6 : 1; /*!< Transfer completion flag clear channel6 */
        volatile uint32_t CLEAR_HALF_TRANSFER_FLAG_CH6     : 1; /*!< Half transfer flag clear channel6 */
        volatile uint32_t CLEAR_TRANSFER_ERROR_FLAG_CH6    : 1; /*!< Transfer error clear channel6 */

        volatile uint32_t CLEAR_GLOBAL_FLAG_CH7            : 1; /*!< global interrupt flag clear for channel7 */
        volatile uint32_t CLEAR_TRANSFER_COMPLETE_FLAG_CH7 : 1; /*!< Transfer completion flag clear channel7 */
        volatile uint32_t CLEAR_HALF_TRANSFER_FLAG_CH7     : 1; /*!< Half transfer flag clear channel7 */
        volatile uint32_t CLEAR_TRANSFER_ERROR_FLAG_CH7    : 1; /*!< Transfer error clear channel7 */
    } IFCR_b;
};} DMA_Type;                                     /*!< Size = 332 (0x14c)                                                        */





#define DMA_BASE                    0x07000000UL

/* ========== DMA Channel Configuration Bit Definitions ================= */
#define DMA_CFG_PERIPH_TO_PERIPH          ((uint32_t)1U << 18)
#define DMA_CFG_MEM_TO_MEM              ((uint32_t)1U << 16)
#define DMA_CFG_PRIORITY_LEVEL(x)        ((uint32_t)(x) << 14)
#define DMA_CFG_MEM_DATA_SIZE(x)     ((uint32_t)(x) << 12)
#define DMA_CFG_PERIPH_DATA_SIZE(x)     ((uint32_t)(x) << 10)
#define DMA_CFG_MEM_ADDR_INC(x)      ((uint32_t)(x) << 8)
#define DMA_CFG_PERIPH_ADDR_INC(x)      ((uint32_t)(x) << 6)
#define DMA_CFG_TRANSFER_DIR            ((uint32_t)1U << 4)
#define DMA_CFG_ERR_INT_ENABLE          ((uint32_t)1U << 3)
#define DMA_CFG_HALF_INT_ENABLE         ((uint32_t)1U << 2)
#define DMA_CFG_TC_INT_ENABLE           ((uint32_t)1U << 1)
#define DMA_CFG_CHANNEL_ENABLE          ((uint32_t)1U << 0)
/* ================= Config Clear Masks ================= */

#define DMA_CFG_TRANSFER_DIR_MASK            ((uint32_t)1U << 4)
#define DMA_CFG_PERIPH_ADDR_INC_MASK         ((uint32_t)0x3U << 6)
#define DMA_CFG_MEM_ADDR_INC_MASK            ((uint32_t)0x3U << 8)
#define DMA_CFG_PERIPH_DATA_SIZE_MASK        ((uint32_t)0x3U << 10)
#define DMA_CFG_MEM_DATA_SIZE_MASK           ((uint32_t)0x3U << 12)
#define DMA_CFG_PRIORITY_LEVEL_MASK          ((uint32_t)0x3U << 14)
#define DMA_CFG_MEM_TO_MEM_MASK              ((uint32_t)1U << 16)
#define DMA_CFG_PERIPH_TO_PERIPH_MASK        ((uint32_t)1U << 18)

/* ================= DMA IFCR Bit Definitions ================= */

#define DMA_CLEAR_XFER_ERROR_FLAG       ((uint32_t)1U << 3)
#define DMA_CLEAR_HALF_XFER_FLAG        ((uint32_t)1U << 2)
#define DMA_CLEAR_XFER_CMPLT_FLAG       ((uint32_t)1U << 1)
#define DMA_CLEAR_GLOBAL_FLAG           ((uint32_t)1U << 0)


/* ================= DMA Internal Mode Flags ================= */

#define DMA_MODE_FAST_SOURCE            ((uint8_t)1U << 2)
#define DMA_MODE_FAST_DESTINATION       ((uint8_t)1U << 3)
/* ================= DMA Configuration Constants ================= */

#define DMA_FAST_PERIPH_BURST           (0U)
#define DMA_INC_ENABLE                  (1U)
#define DMA_SLOW_PERIPH_BURST           (3U)

#define DMA_MAX_CHANNELS                (8U)
#define DMA_INTERRUPT_MASK              (0x0FU)
#define DMA_CHANNEL_FIELD_WIDTH         (4U)

/* ================= Memory Regions ================= */

#define DMA_RAM_START_ADDR        (0x80000000U)
#define DMA_RAM_END_ADDR          (0x80020000U)

#define DMA_FLASH_START_ADDR      (0x90000000U)
#define DMA_FLASH_END_ADDR        (0xD0000000U)


/*
 * Enumeration of DMA request sources and destinations.
 * Each value corresponds to a hardware request line mapping
 * used in DMA_CSELR register configuration.
 */
typedef enum {
    SHA_OUTP_READY = 0,          /* SHA output ready request */
    SHA_CAN_TAKE_INPUT,          /* SHA input request */
    RSA_OUTP_READY,              /* RSA output ready request */
    RSA_CAN_TAKE_INPUT,          /* RSA input request */
    AES_OUTP_READY,              /* AES output ready request */
    AES_CAN_TAKE_INPUT,          /* AES input request */

    UART0_OUTP_READY,            /* UART0 RX ready */
    UART0_CAN_TAKE_INPUT,        /* UART0 TX ready */
    UART1_OUTP_READY,            /* UART1 RX ready */
    UART1_CAN_TAKE_INPUT,        /* UART1 TX ready */
    UART2_OUTP_READY,            /* UART2 RX ready */
    UART2_CAN_TAKE_INPUT,        /* UART2 TX ready */
    UART3_OUTP_READY,            /* UART3 RX ready */
    UART3_CAN_TAKE_INPUT,        /* UART3 TX ready */
    UART4_OUTP_READY,            /* UART4 RX ready */
    UART4_CAN_TAKE_INPUT,        /* UART4 TX ready */

    SPI0_OUTP_READY,             /* SPI0 RX ready */
    SPI0_CAN_TAKE_INPUT,         /* SPI0 TX ready */
    SPI1_OUTP_READY,             /* SPI1 RX ready */
    SPI1_CAN_TAKE_INPUT,         /* SPI1 TX ready */
    SPI2_OUTP_READY,             /* SPI2 RX ready */
    SPI2_CAN_TAKE_INPUT,         /* SPI2 TX ready */
    SPI3_OUTP_READY,             /* SPI3 RX ready */
    SPI3_CAN_TAKE_INPUT,         /* SPI3 TX ready */

    QSPI1_READY,                 /* QSPI1 data ready */
    QSPI0_READY,                 /* QSPI0 data ready */

    PRO_IO_FUSION_OUTP_READY,   /* Pro_IO Fusion (12-bit) output ready */
    PRO_IO_OCTA_OUTP_READY,     /* Pro_IO Octa (8-bit) output ready */
    PRO_IO_TETRA_OUTP_READY,    /* Pro_IO Tetra (4-bit) output ready */
    PRO_IO_DUO_OUTP_READY,      /* Pro_IO Duo (2-bit) output ready */

    PRO_IO_FUSION_CAN_TAKE_INP, /* Pro_IO Fusion (12-bit) input request */
    PRO_IO_OCTA_CAN_TAKE_INP,   /* Pro_IO Octa (8-bit) input request */
    PRO_IO_TETRA_CAN_TAKE_INP,  /* Pro_IO Tetra (4-bit) input request */
    PRO_IO_DUO_CAN_TAKE_INP,    /* Pro_IO Duo (2-bit) input request */

    ITRACE_OUTP_READY,           /* Instruction trace output ready */
    ADC_OUTP_READY               /* ADC conversion complete */
};

/* ================= CRYPTO PERIPHERALS ================= */

#define AES_INP_REG_ADDR        (0x04000000U)  /* AES input register */
#define SHA_INP_REG_ADDR        (0x03000000U)  /* SHA input register */
#define RSA_INP_REG_ADDR        (0x05000000U)  /* RSA input register */

#define AES_OUT_REG_ADDR        (0x40000040U)  /* AES output register */
#define SHA_OUT_REG_ADDR        (0x03000080U)  /* SHA output register */
#define RSA_OUT_REG_ADDR        (0x05000080U)  /* RSA output register */

/* ================= QSPI ================= */

#define QSPI0_DATA_REG_ADDR     (0x00060220U)  /* QSPI0 data register */
#define QSPI1_DATA_REG_ADDR     (0x00060320U)  /* QSPI1 data register */


/* ================= UART TX/RX REGISTERS ================= */

#define UART0_TX_REG_ADDR       (0x00011304U)  /* UART0 TX register */
#define UART1_TX_REG_ADDR       (0x00011404U)  /* UART1 TX register */
#define UART2_TX_REG_ADDR       (0x00011504U)  /* UART2 TX register */
#define UART3_TX_REG_ADDR       (0x00011604U)  /* UART3 TX register */
#define UART4_TX_REG_ADDR       (0x00011704U)  /* UART4 TX register */

#define UART0_RX_REG_ADDR       (0x00011308U)  /* UART0 RX register */
#define UART1_RX_REG_ADDR       (0x00011408U)  /* UART1 RX register */
#define UART2_RX_REG_ADDR       (0x00011508U)  /* UART2 RX register */
#define UART3_RX_REG_ADDR       (0x00011608U)  /* UART3 RX register */
#define UART4_RX_REG_ADDR       (0x00011708U)  /* UART4 RX register */

/* ================= SPI TX/RX REGISTERS ================= */

#define SPI0_TX_REG_ADDR        (0x00020008U)  /* SPI0 TX register */
#define SPI1_TX_REG_ADDR        (0x00020108U)  /* SPI1 TX register */
#define SPI2_TX_REG_ADDR        (0x00020208U)  /* SPI2 TX register */
#define SPI3_TX_REG_ADDR        (0x00020308U)  /* SPI3 TX register */

#define SPI0_RX_REG_ADDR        (0x0002000CU)  /* SPI0 RX register */
#define SPI1_RX_REG_ADDR        (0x0002010CU)  /* SPI1 RX register */
#define SPI2_RX_REG_ADDR        (0x0002020CU)  /* SPI2 RX register */
#define SPI3_RX_REG_ADDR        (0x0002030CU)  /* SPI3 RX register */


/* ================= OTHER PERIPHERALS ================= */

/* Instruction trace data register */
#define ITRACE_DATA_REG_ADDR    (0x00060140U)

/* ADC data register */
#define ADC_DATA_REG_ADDR       (0x00032004U)


/* ================= PRO IO ============================= */

/* Pro IO Duo data register */
#define PRO_IO_DUO_DATA_REG_ADDR     (0x00040270U)

/* Pro IO Tetra data register */
#define PRO_IO_TETRA_DATA_REG_ADDR   (0x00040278U)

/* Pro IO Octa data register */
#define PRO_IO_OCTA_DATA_REG_ADDR    (0x00040280U)

/* Pro IO Fusion data register */
#define PRO_IO_FUSION_DATA_REG_ADDR  (0x00040288U)

/* ================= Memory Regions ================= */

#define DMA_RAM_START_ADDR        (0x80000000U)
#define DMA_RAM_END_ADDR          (0x80020000U)

#define DMA_FLASH_START_ADDR      (0x90000000U)
#define DMA_FLASH_END_ADDR        (0xD0000000U)
