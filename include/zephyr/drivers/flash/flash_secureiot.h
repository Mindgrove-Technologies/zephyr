#include <stdint.h>

#define WRITE 0
#define READ 1

#define FIFO_FULL 16
#define FIFO_EMPTY 0

#define QUADSPI_BASE               0x00040000UL
#define QSPI_OFFSET                 0x00000100UL
#define QSPI_MAX_COUNT 2


#define QUADSPI_Reg(x)  ((QUADSPI_Type*)(QUADSPI_BASE + ((x) * QSPI_OFFSET)))


/*Macros for Instruction MODE*/
#define CCR_IMODE_NIL             0x0
#define CCR_IMODE_SINGLE_LINE     0x1
#define CCR_IMODE_TWO_LINE        0x2
#define CCR_IMODE_FOUR_LINE       0x3
/*Macros for Address MODE*/
#define CCR_ADMODE_NIL            0x0
#define CCR_ADMODE_SINGLE_LINE    0x1
#define CCR_ADMODE_TWO_LINE       0x2
#define CCR_ADMODE_FOUR_LINE      0x3
/*Macros for Address Size*/
#define CCR_ADSIZE_8_BIT          0x0
#define CCR_ADSIZE_16_BIT         0x1
#define CCR_ADSIZE_24_BIT         0x2
#define CCR_ADSIZE_32_BIT         0x3
/*Macros for Alternate Byte mode*/
#define CCR_ABMODE_NIL            0x0
#define CCR_ABMODE_SINGLE_LINE    0x1
#define CCR_ABMODE_TW0_LINE       0x2
#define CCR_ABMODE_FOUR_LINE      0x3
/*Macros for Alternate Byte size*/
#define CCR_ABSIZE_8_BIT          0x0
#define CCR_ABSIZE_16_BIT         0x1
#define CCR_ABSIZE_24_BIT         0x2
#define CCR_ABSIZE_32_BIT         0x3
/*Macros for Data mode*/
#define CCR_DMODE_NO_DATA         0x0
#define CCR_DMODE_SINGLE_LINE     0x1
#define CCR_DMODE_TWO_LINE        0x2
#define CCR_DMODE_FOUR_LINE       0x3
/*Macros for Functional mode*/
#define CCR_FMODE_INDIRECT_WRITE  0x0
#define CCR_FMODE_INDIRECT_READ   0x1
#define CCR_FMODE_APM             0x2
#define CCR_FMODE_MMM             0x3
/*Macros for Memory map mode*/
#define CCR_MM_MODE_XIP           0x0
#define CCR_MM_MODE_RAM           0x1



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

typedef struct{
    uint8_t functional_mode;              /**< Functional mode                                                           */
    uint8_t instruction;                  /**< Instruction                                                               */
    uint8_t instruction_mode;             /**< Instruction mode                                                          */
    uint8_t address_mode;                 /**< Address mode                                                              */
    uint8_t address_size;                 /**< Address size                                                              */
    uint32_t address;                     /**< Address                                                                   */
    uint8_t alternate_byte_mode;          /**< Alternate byte mode                                                       */
    uint8_t alternate_byte;               /**< Alternate byte                                                            */
    uint8_t dummy_mode:1;                 /**< Dummy mode                                                                */
    uint8_t dummy_bit:1;                  /**< Dummy bit                                                                 */
    uint8_t dummy_cycles:5;               /**< Dummy Cycles                                                              */
    uint8_t sioo:1;                       /**< Send instruction only once                                                */
    uint8_t mm_mode:1;                    /**< Memory map mode enable                                                    */
    uint8_t data_mode;                    /**< Data mode                                                                 */
    uint32_t length;                      /**< Data length                                                               */
    uint8_t *data_buffer;                 /**< Pointer to data buffer                                                    */
    uint8_t FMEM_SIZE;                    /**< Flash memory size                                                         */
    uint8_t CLK_MODE:1;                   /**< Clock mode                                                                */
      uint32_t TCEN       : 1;            /**< Timeout counter enable                                                    */
      uint32_t TEIE       : 1;            /**< Transfer error interrupt enable                                           */
      uint32_t TCIE       : 1;            /**< Transfer complete interrupt enable                                        */
      uint32_t FTIE       : 1;            /**< FIFO threshold interrupt enable                                           */
      uint32_t SMIE       : 1;            /**< Status match interrupt enable                                             */
      uint32_t TOIE       : 1;            /**< TimeOut interrupt enable                                                  */
      uint32_t APMS       : 1;            /**< Automatic poll mode stop                                                  */
      uint32_t PMM        : 1;            /**< Polling match mode                                                        */
      uint32_t PRESCALER  : 8;            /**< Clock prescaler                                                           */
      uint8_t fthresh;
}qspi_msg;


typedef union{
    uint32_t data_32;
    uint16_t data_16;
    uint8_t data_8;
} qspi_Data;

typedef struct {                                /*!< QUADSPI0 Structure                                                        */
  
  union {
    volatile uint32_t CR;                          /*!< Control Register                                                          */
    
    struct {
      volatile uint32_t EN         : 1;            /*!< QSPI Communication Enable                                                 */
      volatile uint32_t ABORT      : 1;            /*!< QSPI Communication Abort request                                          */
            uint32_t            : 1;
      volatile uint32_t TCEN       : 1;            /*!< Timeout counter enable                                                    */
            uint32_t            : 4;
      volatile uint32_t FTHRES     : 5;            /*!< IFO threshold level                                                       */
            uint32_t            : 3;
      volatile uint32_t TEIE       : 1;            /*!< Transfer error interrupt enable                                           */
      volatile uint32_t TCIE       : 1;            /*!< Transfer complete interrupt enable                                        */
      volatile uint32_t FTIE       : 1;            /*!< FIFO threshold interrupt enable                                           */
      volatile uint32_t SMIE       : 1;            /*!< Status match interrupt enable                                             */
      volatile uint32_t TOIE       : 1;            /*!< TimeOut interrupt enable                                                  */
            uint32_t            : 1;
      volatile uint32_t APMS       : 1;            /*!< Automatic poll mode stop                                                  */
      volatile uint32_t PMM        : 1;            /*!< Polling match mode                                                        */
      volatile uint32_t PRESCALER  : 8;            /*!< Clock prescaler                                                           */
    } CR_b;
  } ;
  
  union {
    volatile uint32_t DCR;                         /*!< Device Configuration Register                                             */
    
    struct {
      volatile uint32_t CKMODE     : 1;            /*!< Mode 0 / mode 3                                                           */
            uint32_t            : 15;
      volatile uint32_t FSIZE      : 5;            /*!< FLASH memory size                                                         */
            uint32_t            : 11;
    } DCR_b;
  } ;
  
  union {
    volatile const  uint32_t SR;                          /*!< Status Register                                                           */
    
    struct {
      volatile const  uint32_t TEF        : 1;            /*!< Transfer error flag                                                       */
      volatile const  uint32_t TCF        : 1;            /*!< Transfer complete flag                                                    */
      volatile const  uint32_t FTF        : 1;            /*!< FIFO threshold flag                                                       */
      volatile const  uint32_t SMF        : 1;            /*!< Status match flag                                                         */
      volatile const  uint32_t TOF        : 1;            /*!< Timeout flag                                                              */
      volatile const  uint32_t BUSY       : 1;            /*!< Busy                                                                      */
            uint32_t            : 2;
      volatile const  uint32_t FLEVEL     : 7;            /*!< FIFO level                                                                */
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
  
  union {
    volatile qspi_Data DR;                          /*!< Data Register                                                             */
    
    struct {
      volatile uint32_t DATA       : 32;           /*!< Data                                                                      */
    } DR_b;
  } ;
} QUADSPI_Type;                                /*!< Size = 36 (0x24)                                                          */
