#include <stdint.h>

#define DMA_BASE                    0x07000000UL

#define DMA_CFG_PERIPH_TO_PERIPH          ((uint32_t)1U << 18)
#define DMA_CFG_MEM_TO_MEM              ((uint32_t)1U << 16)
#define DMA_CCR_PL(x)        ((uint32_t)(x) << 14)
#define DMA_CCR_MSIZE(x)     ((uint32_t)(x) << 12)
#define DMA_CCR_PSIZE(x)     ((uint32_t)(x) << 10)
#define DMA_CCR_MINC(x)      ((uint32_t)(x) << 8)
#define DMA_CCR_PINC(x)      ((uint32_t)(x) << 6)
#define DMA_CFG_TRANSFER_DIR          (1U << 4)
#define DMA_CCR_TEIE         (1U << 3)
#define DMA_CCR_HTIE         (1U << 2)
#define DMA_CCR_TCIE         (1U << 1)
#define DMA_IFCR_TEIF        (1U << 3)
#define DMA_IFCR_HTIF        (1U << 2)
#define DMA_IFCR_TCIF        (1U << 1)
#define DMA_IFCR_GIF         (1U << 0)
#define DMA_CCR_EN           (1U << 0)

#define DMA_MODE_FAST_SOURCE            ((uint8_t)1U << 2)
#define DMA_MODE_FAST_DESTINATION       ((uint8_t)1U << 3)

#define DMA_FAST_PERIPH_BURST   (0U)
#define DMA_INC_ENABLE          (1U)
#define DMA_SLOW_PERIPH_BURST   (3U)
#define DMA_MAX_CHANNELS        (8U)
#define DMA_INTERRUPT_MASK      (0x0FU)

typedef union{
  uint32_t data_32;
  uint16_t data_16;
  uint8_t data_8;
} Data;

typedef struct {                                /*!< DMA Structure                                                             */
  
  union {
    volatile uint32_t DMA_CCR;                    /*!< Channel configuration regisster of channel                              */
    
    struct {
      volatile uint32_t EN         : 1;            /*!< To enable the channel                                                     */
      volatile uint32_t TCIE       : 1;            /*!< Transfer complete interrupt enable                                        */
      volatile uint32_t HTIE       : 1;            /*!< Half transfer interrupt enable                                            */
      volatile uint32_t TEIE       : 1;            /*!< Transfer error interrupt enable                                           */
      volatile uint32_t DIR        : 1;            /*!< Data transfer direction                                                   */
            uint32_t            : 1;
      volatile uint32_t PINC       : 2;            /*!< Periheral increment mode                                                  */
      volatile uint32_t MINC       : 2;            /*!< Memory increment mode                                                     */
      volatile uint32_t PSIZE      : 2;            /*!< Data size of each DMA transfer to the peripheral                          */
      volatile uint32_t MSIZE      : 2;            /*!< Data size of each DMA transfer to the memory                              */
      volatile uint32_t PL         : 2;            /*!< Priority level                                                            */
      volatile uint32_t MEM2MEM    : 1;            /*!< Memory to memory mode                                                     */
            uint32_t            : 1;
      volatile uint32_t P2P        : 1;            /*!< Periheral to peripheral mode                                              */
            uint32_t            : 13;
    } DMA_CCR_b;
  } ;
  volatile  uint32_t  reserved;
  
  union {
    volatile uint16_t DMA_CNDTR;                  /*!< Channel number of data to transfer                                       */
    
    struct {
      volatile uint16_t NDT        : 16;           /*!< This register has the value of total number of data to transfer
                                                     channel                                                                 */
    } DMA_CNDTR_b;
  } ;
  volatile  uint16_t  reserved1;
  volatile  uint32_t  reserved2;
  
  union {
    volatile uint32_t DMA_CPAR;                   /*!< Channel peripheral address register                                      */
    
    struct {
      volatile uint32_t PA         : 32;           /*!< It consists the base address of peripheral data register depends
                                                     on the direction bit it can be source or destination address              */
    } DMA_CPAR_b;
  } ;
  volatile  uint32_t  reserved3;
  
  union {
    volatile uint32_t DMA_CMAR;                   /*!< Channel memory address register                                          */
    
    struct {
      volatile uint32_t PA         : 32;           /*!< It consists the base address of peripheral data register depends
                                                     on the direction bit it can be source or destination address              */
    } DMA_CMAR_b;
  } ;
  volatile  uint32_t  reserved4;
  
  union {
    volatile uint16_t DMA_CSELR;                  /*!< Channel selection register                                               */
    
    struct {
      volatile uint16_t CS         : 10;           /*!< This register is used to the DMA channel                                 */
            uint16_t            : 6;
    } DMA_CSELR_b;
  } ;
  volatile  uint16_t  reserved5;
  volatile  uint32_t  reserved6;
  
}  DMA_CHANNEL_Type;


typedef struct { 

  DMA_CHANNEL_Type CH[8];

  union {
    volatile uint32_t DMA_ISR;                     /*!< Interrupt status register                                                 */
    
    struct {
      volatile uint32_t GIF0       : 1;            /*!< global interrupt flag for channel0                                        */
      volatile uint32_t TCIF0      : 1;            /*!< Transfer completion flag for channel0                                     */
      volatile uint32_t HTIF0      : 1;            /*!< Half transfer flag for channel0                                           */
      volatile uint32_t TEIF0      : 1;            /*!< Transfer error flag channel0                                              */
      volatile uint32_t GIF1       : 1;            /*!< global interrupt flag for channel1                                        */
      volatile uint32_t TCIF1      : 1;            /*!< Transfer completion flag for channel1                                     */
      volatile uint32_t HTIF1      : 1;            /*!< Half transfer flag for channel1                                           */
      volatile uint32_t TEIF1      : 1;            /*!< Transfer error flag channel1                                              */
      volatile uint32_t GIF2       : 1;            /*!< global interrupt flag for channel2                                        */
      volatile uint32_t TCIF2      : 1;            /*!< Transfer completion flag for channel2                                     */
      volatile uint32_t HTIF2      : 1;            /*!< Half transfer flag for channel2                                           */
      volatile uint32_t TEIF2      : 1;            /*!< Transfer error flag channel2                                              */
      volatile uint32_t GIF3       : 1;            /*!< global interrupt flag for channel3                                        */
      volatile uint32_t TCIF3      : 1;            /*!< Transfer completion flag for channel3                                     */
      volatile uint32_t HTIF3      : 1;            /*!< Half transfer flag for channel3                                           */
      volatile uint32_t TEIF3      : 1;            /*!< Transfer error flag channel3                                              */
      volatile uint32_t GIF4       : 1;            /*!< global interrupt flag for channel4                                        */
      volatile uint32_t TCIF4      : 1;            /*!< Transfer completion flag for channel4                                     */
      volatile uint32_t HTIF4      : 1;            /*!< Half transfer flag for channel4                                           */
      volatile uint32_t TEIF4      : 1;            /*!< Transfer error flag channel4                                              */
      volatile uint32_t GIF5       : 1;            /*!< global interrupt flag for channel5                                        */
      volatile uint32_t TCIF5      : 1;            /*!< Transfer completion flag for channel5                                     */
      volatile uint32_t HTIF5      : 1;            /*!< Half transfer flag for channel5                                           */
      volatile uint32_t TEIF5      : 1;            /*!< Transfer error flag channel5                                              */
      volatile uint32_t GIF6       : 1;            /*!< global interrupt flag for channel6                                        */
      volatile uint32_t TCIF6      : 1;            /*!< Transfer completion flag for channel6                                     */
      volatile uint32_t HTIF6      : 1;            /*!< Half transfer flag for channel6                                           */
      volatile uint32_t TEIF6      : 1;            /*!< Transfer error flag channel6                                              */
      volatile uint32_t GIF7       : 1;            /*!< global interrupt flag for channel7                                        */
      volatile uint32_t TCIF7      : 1;            /*!< Transfer completion flag for channel7                                     */
      volatile uint32_t HTIF7      : 1;            /*!< Half transfer flag for channel7                                           */
      volatile uint32_t TEIF7      : 1;            /*!< Transfer error flag channel7                                              */
    } DMA_ISR_b;
  } ;
  volatile  uint32_t  RESERVED56;
  
  union {
    volatile uint32_t DMA_IFCR;                    /*!< Interrupt flag clear register                                             */
    
    struct {
      volatile uint32_t CGIF0      : 1;            /*!< global interrupt flag clear for channel0                                  */
      volatile uint32_t CTCIF0     : 1;            /*!< Transfer completion flag clear channel0                                   */
      volatile uint32_t CHTIF0     : 1;            /*!< Half transfer flag clear channel0                                         */
      volatile uint32_t CTEIF0     : 1;            /*!< Transfer errorclear channel0                                              */
      volatile uint32_t CGIF1      : 1;            /*!< global interrupt flag clear for channel1                                  */
      volatile uint32_t CTCIF1     : 1;            /*!< Transfer completion flag clear channel1                                   */
      volatile uint32_t CHTIF1     : 1;            /*!< Half transfer flag clear channel1                                         */
      volatile uint32_t CTEIF1     : 1;            /*!< Transfer error clear channel1                                             */
      volatile uint32_t CGIF2      : 1;            /*!< global interrupt flag clear for channel0                                  */
      volatile uint32_t CTCIF2     : 1;            /*!< Transfer completion flag clear channel0                                   */
      volatile uint32_t CHTIF2     : 1;            /*!< Half transfer flag clear channel2                                         */
      volatile uint32_t CTEIF2     : 1;            /*!< Transfer errorclear channel2                                              */
      volatile uint32_t CGIF3      : 1;            /*!< global interrupt flag clear for channel3                                  */
      volatile uint32_t CTCIF3     : 1;            /*!< Transfer completion flag clear channel3                                   */
      volatile uint32_t CHTIF3     : 1;            /*!< Half transfer flag clear channel3                                         */
      volatile uint32_t CTEIF3     : 1;            /*!< Transfer errorclear channel3                                              */
      volatile uint32_t CGIF4      : 1;            /*!< global interrupt flag clear for channel4                                  */
      volatile uint32_t CTCIF4     : 1;            /*!< Transfer completion flag clear channel4                                   */
      volatile uint32_t CHTIF4     : 1;            /*!< Half transfer flag clear channel4                                         */
      volatile uint32_t CTEIF4     : 1;            /*!< Transfer errorclear channel4                                              */
      volatile uint32_t CGIF5      : 1;            /*!< global interrupt flag clear for channel5                                  */
      volatile uint32_t CTCIF5     : 1;            /*!< Transfer completion flag clear channel5                                   */
      volatile uint32_t CHTIF5     : 1;            /*!< Half transfer flag clear channel5                                         */
      volatile uint32_t CTEIF5     : 1;            /*!< Transfer errorclear channel5                                              */
      volatile uint32_t CGIF6      : 1;            /*!< global interrupt flag clear for channel6                                  */
      volatile uint32_t CTCIF06    : 1;            /*!< Transfer completion flag clear channel6                                   */
      volatile uint32_t CHTIF6     : 1;            /*!< Half transfer flag clear channel6                                         */
      volatile uint32_t CTEIF6     : 1;            /*!< Transfer errorclear channel6                                              */
      volatile uint32_t CGIF7      : 1;            /*!< global interrupt flag clear for channel7                                  */
      volatile uint32_t CTCIF7     : 1;            /*!< Transfer completion flag clear channel7                                   */
      volatile uint32_t CHTIF7     : 1;            /*!< Half transfer flag clear channel7                                         */
      volatile uint32_t CTEIF7     : 1;            /*!< Transfer errorclear channel7                                              */
    } DMA_IFCR_b;
  } ;
} DMA_Type;                                     /*!< Size = 332 (0x14c)                                                        */


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
