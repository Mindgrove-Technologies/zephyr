#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <errno.h>
#include "flash_mindgrove.h"

/* Logging module registration for Zephyr 3.2 */
#define DT_DRV_COMPAT mindgrove_qspi_flash
#define LOG_LEVEL CONFIG_FLASH_LOG_LEVEL
#define CLOCK_FREQUENCY_FPGA        35000000UL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(qspi_flash);

/* ================= Flash Command Opcodes (JEDEC Standard) ================= */

/** @brief Sector Erase (4KB) command */
#define FLASH_CMD_ERASE_4K   0x20

/** @brief Block Erase (32KB) command */
#define FLASH_CMD_ERASE_32K  0x52

/** @brief Chip Erase command (entire flash memory) */
#define FLASH_CMD_ERASE_CHIP 0xC7

/** @brief Write Enable command for status register */
#define FLASH_CMD_WRITE_ENABLE   0x06

/** @brief Write Disable command for status register */
#define FLASH_CMD_WRITE_DISABLE  0x04

/** @brief Read Status Register 1 command */
#define FLASH_CMD_READ_STATUS1   0x05

/** @brief Quad I/O Fast Read command (4-bit data, 4-bit address) */
#define FLASH_CMD_QUAD_READ_IO   0xEB

/** @brief Quad Page Program command (4-bit data) */
#define FLASH_CMD_QUAD_PAGE_PROGRAM 0x32

/* ================= Flash Status Register Bit Definitions ================= */

/** @brief Write In Progress (WIP) bit - set during write/erase operations */
#define FLASH_SR_WIP_BIT      (0x01U)

/** @brief Write Enable Latch (WEL) bit - set after write enable command */
#define FLASH_SR_WEL_BIT      (0x02U)

/* ================= QSPI Transaction Message Global Instance ================= */

static qspi_msg flash_msg = {
    .PRESCALER = 10,
    .CLK_MODE = 1,
    .FMEM_SIZE = 27,
    .FTIE = 0,
    .TCEN = 0,
    .TEIE = 0,
    .TOIE = 0,
    .SMIE = 0,
    .APMS = 0,
    .PMM = 0,
    .csht = 7
};

/* ============================================================================
 * QSPI Flash Driver Configuration and Data Structures
 * ============================================================================ */

/**
 * @brief QSPI flash device configuration structure
 *
 * Contains hardware-specific configuration data that remains constant
 * throughout the device lifetime. This structure is instantiated via
 * Device Tree macros during compilation.
 */
struct flash_qspi_config {
    const QUADSPI_Type *qspi;  /**< Base address of QSPI hardware registers */
    size_t flash_size;          /**< Total flash capacity in bytes */
};

/**
 * @brief QSPI flash device runtime data structure
 *
 * Currently reserved for future extensions such as power management state,
 * runtime statistics, or operation mutex locks.
 */
struct flash_qspi_data {
    uint8_t reserved;  /**< Placeholder for future runtime state */
};

/* ============================================================================
 * Static Flash Parameters
 * ============================================================================ */

static const struct flash_parameters flash_qspi_parameters = {
    .write_block_size = 256,   /**< Page program granularity in bytes */
    .erase_value = 0xFF,       /**< Erased NOR flash cells read as 0xFF */
};

/* ============================================================================
 * Zephyr Flash API Implementation - Parameter Retrieval
 * ============================================================================ */

static const struct flash_parameters *
flash_qspi_get_parameters(const struct device *dev)
{
    ARG_UNUSED(dev);
    return &flash_qspi_parameters;
}

/* ============================================================================
 * Low-Level QSPI Transaction
 * ============================================================================ */

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
     * Clock validation: QSPI clock = 700MHz / (PRESCALER + 1)
     * Must not exceed MAX_QSPI_FREQ.
     */
    if (((uint64_t)CLOCK_FREQUENCY_FPGA /
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

/* ============================================================================
 * Flash Status Register Operations
 * ============================================================================ */

uint16_t Flash_Read_Status_Register1(const QUADSPI_Type *qspi_inst,
                                     uint8_t *data) {
    flash_msg.qspi_inst = qspi_inst;
    flash_msg.address_mode = CCR_ADMODE_NIL;
    flash_msg.address_size = CCR_ADSIZE_24_BIT;
    flash_msg.instruction = FLASH_CMD_READ_STATUS1;
    flash_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    flash_msg.data_mode = CCR_DMODE_SINGLE_LINE;
    flash_msg.data_buffer = data;
    flash_msg.functional_mode = CCR_FMODE_INDIRECT_READ;
    flash_msg.dummy_mode = 0;
    flash_msg.dummy_cycles = 0;
    flash_msg.dummy_bit = 0;
    flash_msg.mm_mode = CCR_MM_MODE_XIP;
    flash_msg.alternate_byte_mode = CCR_ABMODE_NIL;
    flash_msg.length = 1;

    return QSPI_Transaction(&flash_msg);
}

uint16_t Flash_Write_Enable(const QUADSPI_Type *qspi_inst) {
    uint16_t ret;
    volatile uint8_t status_reg;
    int timeout = 1000;

    flash_msg.qspi_inst = qspi_inst;
    flash_msg.address_mode = CCR_ADMODE_NIL;
    flash_msg.address_size = CCR_ADSIZE_8_BIT;
    flash_msg.instruction = FLASH_CMD_WRITE_ENABLE;
    flash_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    flash_msg.data_mode = CCR_DMODE_NO_DATA;
    flash_msg.functional_mode = CCR_FMODE_INDIRECT_WRITE;
    flash_msg.dummy_mode = 0;
    flash_msg.dummy_cycles = 1;  /**< RTL requirement */
    flash_msg.dummy_bit = 0;
    flash_msg.mm_mode = CCR_MM_MODE_XIP;
    flash_msg.alternate_byte_mode = CCR_ABMODE_NIL;

    ret = QSPI_Transaction(&flash_msg);
    if (ret != 0) {
        return ret;
    }

    /* Poll until WEL bit is set */
    do {
        ret = Flash_Read_Status_Register1(qspi_inst, &status_reg);
        if (ret != 0) {
            return ret;
        }
        if (status_reg & FLASH_SR_WEL_BIT) {
            return 0;
        }
    } while (--timeout > 0);

    return -EIO;
}

uint16_t Flash_Write_Disable(const QUADSPI_Type *qspi_inst) {
    flash_msg.qspi_inst = qspi_inst;
    flash_msg.address_mode = CCR_ADMODE_NIL;
    flash_msg.address_size = CCR_ADSIZE_8_BIT;
    flash_msg.instruction = FLASH_CMD_WRITE_DISABLE;
    flash_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    flash_msg.data_mode = CCR_DMODE_NO_DATA;
    flash_msg.functional_mode = CCR_FMODE_INDIRECT_WRITE;
    flash_msg.dummy_mode = 0;
    flash_msg.dummy_cycles = 1;  /**< RTL requirement */
    flash_msg.dummy_bit = 0;
    flash_msg.mm_mode = CCR_MM_MODE_XIP;
    flash_msg.alternate_byte_mode = CCR_ABMODE_NIL;

    return QSPI_Transaction(&flash_msg);
}

/* ============================================================================
 * Flash Erase Operations
 * ============================================================================ */

static uint16_t Flash_Erase(const QUADSPI_Type *qspi,
                            uint32_t address,
                            uint8_t cmd)
{
    flash_msg.address = address;
    flash_msg.qspi_inst = qspi;

    /* Chip erase has no address phase; sector/block erases do */
    if (cmd == FLASH_CMD_ERASE_4K || cmd == FLASH_CMD_ERASE_32K) {
        flash_msg.address_mode = CCR_ADMODE_SINGLE_LINE;
        flash_msg.address_size = CCR_ADSIZE_24_BIT;
    } else {
        flash_msg.address_mode = CCR_ADMODE_NIL;
        flash_msg.address_size = CCR_ADSIZE_24_BIT;
    }

    flash_msg.instruction = cmd;
    flash_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    flash_msg.data_mode = CCR_DMODE_NO_DATA;
    flash_msg.functional_mode = CCR_FMODE_INDIRECT_WRITE;
    flash_msg.dummy_mode = 0;
    flash_msg.dummy_cycles = 1;  /**< RTL requirement */
    flash_msg.dummy_bit = 0;
    flash_msg.mm_mode = CCR_MM_MODE_XIP;
    flash_msg.alternate_byte_mode = CCR_ABMODE_NIL;

    return QSPI_Transaction(&flash_msg);
}

/* ============================================================================
 * Zephyr Flash API Implementation - Read Operation
 * ============================================================================ */

int flash_qspi_read(const struct device *dev,
                    off_t addr,
                    void *data,
                    size_t len)
{
    const struct flash_qspi_config *cfg = dev->config;
    const QUADSPI_Type *qspi = cfg->qspi;

    if ((addr < 0) || ((addr + len) > cfg->flash_size)) {
        return -EINVAL;
    }

    flash_msg.address = addr;
    flash_msg.qspi_inst = qspi;
    flash_msg.address_mode = CCR_ADMODE_FOUR_LINE;
    flash_msg.address_size = CCR_ADSIZE_24_BIT;
    flash_msg.instruction = FLASH_CMD_QUAD_READ_IO;
    flash_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    flash_msg.data_mode = CCR_DMODE_FOUR_LINE;
    flash_msg.data_buffer = data;
    flash_msg.functional_mode = CCR_FMODE_INDIRECT_READ;
    flash_msg.dummy_mode = 0;
    flash_msg.dummy_cycles = 4;
    flash_msg.dummy_bit = 0;
    flash_msg.mm_mode = CCR_MM_MODE_XIP;
    flash_msg.alternate_byte_mode = CCR_ABMODE_FOUR_LINE;
    flash_msg.alternate_byte = 0x20;  /**< Mode byte = continuous read */
    flash_msg.length = len;

    return QSPI_Transaction(&flash_msg);
}

/* ============================================================================
 * Zephyr Flash API Implementation - Write Operation
 * ============================================================================ */

int flash_qspi_write(const struct device *dev,
                     off_t addr,
                     const void *data,
                     size_t len)
{
    const struct flash_qspi_config *cfg = dev->config;
    const QUADSPI_Type *qspi = cfg->qspi;
    uint16_t ret;
    uint8_t status_reg;

    if ((addr < 0) || ((addr + len) > cfg->flash_size)) {
        return -EINVAL;
    }

    ret = Flash_Write_Enable(qspi);
    if (ret != 0) {
        return ret;
    }

    flash_msg.address = addr;
    flash_msg.qspi_inst = qspi;
    flash_msg.address_mode = CCR_ADMODE_SINGLE_LINE;
    flash_msg.address_size = CCR_ADSIZE_24_BIT;
    flash_msg.instruction = FLASH_CMD_QUAD_PAGE_PROGRAM;
    flash_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    flash_msg.data_mode = CCR_DMODE_FOUR_LINE;
    flash_msg.data_buffer = (void *)data;
    flash_msg.functional_mode = CCR_FMODE_INDIRECT_WRITE;
    flash_msg.dummy_mode = 0;
    flash_msg.dummy_cycles = 0;
    flash_msg.dummy_bit = 0;
    flash_msg.mm_mode = CCR_MM_MODE_XIP;
    flash_msg.alternate_byte_mode = CCR_ABMODE_NIL;
    flash_msg.length = len;

    ret = QSPI_Transaction(&flash_msg);
    if (ret != 0) {
        return ret;
    }

    /* Poll WIP bit until write completes */
    do {
        Flash_Read_Status_Register1(qspi, &status_reg);
    } while (status_reg & FLASH_SR_WIP_BIT);

    Flash_Write_Disable(qspi);

    return 0;
}

/* ============================================================================
 * Zephyr Flash API Implementation - Erase Operation
 * ============================================================================ */

static int flash_qspi_erase(const struct device *dev,
                            off_t offset,
                            size_t size)
{
    const struct flash_qspi_config *cfg = dev->config;
    const QUADSPI_Type *qspi = cfg->qspi;
    uint32_t addr = offset;
    uint16_t ret;
    uint8_t sr;

    if (size == 0) {
        return 0;
    }

    if ((offset < 0) || ((offset + size) > cfg->flash_size)) {
        return -EINVAL;
    }

    /**
     * Optimisation: if the caller requests a full chip erase
     * (offset=0, size=flash_size), use the atomic chip erase command
     * instead of looping over individual 32KB block erases.
     */
    if (offset == 0 && size == cfg->flash_size) {
        ret = Flash_Write_Enable(qspi);
        if (ret != 0) {
            return -EIO;
        }
        Flash_Erase(qspi, 0, FLASH_CMD_ERASE_CHIP);
        Flash_Write_Disable(qspi);
        do {
            Flash_Read_Status_Register1(qspi, &sr);
        } while (sr & FLASH_SR_WIP_BIT);
        return 0;
    }

    /* Iterate over erase region using optimal block sizes */
    while (size > 0) {
        if ((size >= 32768) && (addr % 32768 == 0)) {
            ret = Flash_Write_Enable(qspi);
            if (ret != 0) {
                return -EIO;
            }
            Flash_Erase(qspi, addr, FLASH_CMD_ERASE_32K);
            Flash_Write_Disable(qspi);
            size -= 32768;
            addr += 32768;
        } else if ((size >= 4096) && (addr % 4096 == 0)) {
            ret = Flash_Write_Enable(qspi);
            if (ret != 0) {
                return -EIO;
            }
            Flash_Erase(qspi, addr, FLASH_CMD_ERASE_4K);
            Flash_Write_Disable(qspi);
            size -= 4096;
            addr += 4096;
        } else {
            /* Unaligned erase address - NOR flash requires alignment */
            return -EINVAL;
        }

        /* Poll WIP bit until erase completes */
        do {
            Flash_Read_Status_Register1(qspi, &sr);
        } while (sr & FLASH_SR_WIP_BIT);
    }

    return 0;
}
#if defined(CONFIG_FLASH_JESD216_API)
static int flash_qspi_sfdp_read(const struct device *dev,
                                off_t addr,
                                void *data,
                                size_t len)
{
    const struct flash_qspi_config *cfg = dev->config;
    const QUADSPI_Type *qspi = cfg->qspi;

    flash_msg.qspi_inst = qspi;
    flash_msg.address_mode = CCR_ADMODE_SINGLE_LINE;
    flash_msg.address_size = CCR_ADSIZE_24_BIT;
    flash_msg.address = addr;
    flash_msg.instruction = 0x5A;
    flash_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    flash_msg.data_mode = CCR_DMODE_SINGLE_LINE;
    flash_msg.functional_mode = CCR_FMODE_INDIRECT_READ;
    flash_msg.dummy_cycles = 8;
    flash_msg.length = len;
    flash_msg.data_buffer = data;

    return QSPI_Transaction(&flash_msg);
}
static int flash_qspi_read_jedec_id(const struct device *dev,
                                    uint8_t *id)
{
    const struct flash_qspi_config *cfg = dev->config;
    const QUADSPI_Type *qspi = cfg->qspi;

    flash_msg.qspi_inst = qspi;
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

    return QSPI_Transaction(&flash_msg);
}
#endif
/* ============================================================================
 * Flash Driver API Structure
 * ============================================================================ */

static const struct flash_driver_api flash_qspi_api = {
    .read = flash_qspi_read,
    .write = flash_qspi_write,
    .erase = flash_qspi_erase,
    .get_parameters = flash_qspi_get_parameters,
#if defined(CONFIG_FLASH_JESD216_API)
    .sfdp_read = flash_qspi_sfdp_read,
    .read_jedec_id = flash_qspi_read_jedec_id
#endif
};

/* ============================================================================
 * Device Initialisation
 * ============================================================================ */

static int flash_qspi_init(const struct device *dev)
{
    ARG_UNUSED(dev);
    return 0;
}

/* ============================================================================
 * Device Instantiation Macros
 * ============================================================================ */

#define FLASH_QSPI_DEFINE(inst)                                        \
    static struct flash_qspi_data flash_qspi_data_##inst;              \
                                                                        \
    static const struct flash_qspi_config flash_qspi_cfg_##inst = {    \
        .qspi = (const QUADSPI_Type *)DT_INST_REG_ADDR(inst),          \
        .flash_size = DT_INST_PROP(inst, size),                        \
    };                                                                  \
                                                                        \
    DEVICE_DT_INST_DEFINE(inst,                                         \
                          flash_qspi_init,                              \
                          NULL,                                         \
                          &flash_qspi_data_##inst,                      \
                          &flash_qspi_cfg_##inst,                       \
                          POST_KERNEL,                                  \
                          CONFIG_FLASH_INIT_PRIORITY,                   \
                          &flash_qspi_api);

DT_INST_FOREACH_STATUS_OKAY(FLASH_QSPI_DEFINE)