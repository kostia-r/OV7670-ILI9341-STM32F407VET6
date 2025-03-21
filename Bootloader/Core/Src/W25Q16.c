/*
 * W25Q16.c
 * W25Q16JV Flash driver
 *  Created on: Dec 4, 2024
 *      Author: ashen
 */

/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/

#include "W25Q16.h"
#include "spi.h"
#include "main.h"

/******************************************************************************
 *                            LOCAL DATA TYPES                                *
 ******************************************************************************/

typedef struct
{
  uint32_t  id;
  uint16_t  PageSize;
  uint32_t  PageCount;
  uint32_t  SectorSize;
  uint32_t  SectorCount;
  uint32_t  BlockSize;
  uint32_t  BlockCount;
  uint32_t  NumKB;
  uint8_t   SR1;
  uint8_t   SR2;
  uint8_t   SR3;
  uint8_t   high_cap;
  uint8_t   StatusRegister1;
  uint8_t   StatusRegister2;
  uint8_t   StatusRegister3;
  uint32_t  MetadataSector;   // Reserved sector for metadata
  uint32_t  EraseCounts[32];  // Wear leveling data (for 32 blocks)
}w25_info_t;
w25_info_t  w25_info;

/******************************************************************************
 *                               LOCAL MACRO                                  *
 ******************************************************************************/
/* Instructions */
#define W25_ENABLE_RESET                                                (0x66U)
#define W25_RESET                                                       (0x99U)
#define W25_READ                                                        (0x03U)
#define W25_FAST_READ                                                   (0x0BU)
#define W25_GET_JEDEC_ID                                                (0x9FU)
#define W25_WRITE_DISABLE                                               (0x04U)
#define W25_WRITE_ENABLE                                                (0x06U)
#define W25_SECTOR_ERASE                                                (0x20U)
#define W25_BLOCK_ERASE                                                 (0xD8U)
#define W25_CHIP_ERASE                                                  (0xC7U)
#define W25_PAGE_PROGRAMM                                               (0x02U)
#define W25_READ_STATUS_1                                               (0x05U)
#define W25_READ_STATUS_2                                               (0x35U)
#define W25_READ_STATUS_3                                               (0x15U)
#define W25_WRITE_STATUS_1                                              (0x01U)
#define W25_WRITE_STATUS_2                                              (0x31U)
#define W25_WRITE_STATUS_3                                              (0x11U)

/* Chip Select macros */
#define W25_CS_HIGH()                    SET_BIT(W25_CS_PORT->BSRR, W25_CS_PIN)
#define W25_CS_LOW()   SET_BIT(W25_CS_PORT->BSRR,((uint32_t)W25_CS_PIN << 16U))

/******************************************************************************
 *                        GLOBAL DATA PROTOTYPES                              *
 ******************************************************************************/

extern SPI_HandleTypeDef hspi1;

/******************************************************************************
 *                         LOCAL DATA PROTOTYPES                              *
 ******************************************************************************/

uint8_t buf[10];

/******************************************************************************
 *                       LOCAL FUNCTIONS PROTOTYPES                           *
 ******************************************************************************/

static void w25_Reset(void);
static uint32_t w25_ReadID(void);
static void w25_WriteEnable(void);
static void w25_WriteDisable(void);
static void w25_SetBlockProtect(uint8_t val);
static void w25_WaitWriteEnd(void);
static BL_Status_t Metadata_Read(void);
static BL_Status_t Metadata_Write(void);
static uint32_t FindLeastUsedBlock(void);
static BL_Status_t w25_EraseSector(uint32_t addr);
static BL_Status_t w25_EraseBlock(uint32_t addr);
static void SPI_Send(const uint8_t *pData, uint16_t size);
static void SPI_Recv(uint8_t *pBuffer, uint16_t size);

/******************************************************************************
 *                            GLOBAL FUNCTIONS                                *
 ******************************************************************************/
BL_Status_t W25Q_Init(void)
{
	HAL_Delay(100);
	w25_Reset();
	HAL_Delay(100);

	w25_info.id = w25_ReadID();
	w25_info.BlockCount = 32;
	w25_info.PageSize = 256;
	w25_info.SectorSize = 0x1000;
	w25_info.SectorCount = w25_info.BlockCount * 16;
	w25_info.PageCount = (w25_info.SectorCount * w25_info.SectorSize) / w25_info.PageSize;
	w25_info.BlockSize = w25_info.SectorSize * 16;
	w25_info.NumKB = (w25_info.SectorCount * w25_info.SectorSize) / 1024;
	w25_info.MetadataSector = w25_info.SectorCount - 1;  // Reserve the last sector

	if ((w25_info.id == 0x401AU) || (w25_info.id == 0x4019U))
	{
		w25_info.high_cap = 1U; // use 32-bit address space for 512 and 1024 MBit chips
	}
	else
	{
		w25_info.high_cap = 0U; // otherwize - 24-bit address space
	}

	// Load metadata (e.g., erase counts) from the reserved sector
	if (Metadata_Read() != BL_OK)
	{
		return BL_ERROR;
	}

	return BL_OK;
}


BL_Status_t W25Q_ReadData(uint32_t addr, uint8_t* pBuffer, uint32_t size)
 {
	uint8_t* cmdPtr = buf;
	W25_CS_LOW();
	*cmdPtr++ = W25_READ;
	*cmdPtr++ = (addr >> 16) & 0xFFU;
	*cmdPtr++ = (addr >> 8) & 0xFFU;
	*cmdPtr = addr & 0xFFU;
	SPI_Send(buf, 4U);
	SPI_Recv(pBuffer, size);
	W25_CS_HIGH();
	return BL_OK;
}


BL_Status_t W25Q_Erase(uint32_t addr, uint32_t size)
{
    BL_Status_t status = BL_ERROR;

    do {
        // Check for valid size: must be a multiple of sector size
        if ((size % w25_info.SectorSize) != 0U)
        {
        	status = BL_ERROR;
            //status = BL_INVALID_SIZE;
            break;
        }

        // Erase by sectors if size is less than a block
        if (size < w25_info.BlockSize)
        {
            while (size > 0U)
            {
                status = w25_EraseSector(addr / w25_info.SectorSize);
                if (status != BL_OK)
                {
                	break;
                }

                addr += w25_info.SectorSize;
                size -= w25_info.SectorSize;
            }
        }
        // Erase by blocks if size matches block size
        else if (size == w25_info.BlockSize)
        {
            status = w25_EraseBlock(addr / w25_info.BlockSize);
        }
        // Invalid size for erasure
        else
        {
        	status = BL_ERROR;
            //status = BL_INVALID_SIZE;
        }
    } while (false);

    return status;
}



BL_Status_t W25Q_ReadPage(uint32_t page_numb, uint32_t offset, uint8_t* pBuffer, uint32_t size)
 {
	uint32_t page_addr;
	uint8_t* cmdPtr = buf;

	if (size > w25_info.PageSize)
	{
		size = w25_info.PageSize;
	}

	if ((offset + size) > w25_info.PageSize)
	{
		size = w25_info.PageSize - offset;
	}

	size = w25_info.PageSize - offset;
	page_addr = page_numb * w25_info.PageSize + offset;

	*cmdPtr++ = W25_FAST_READ;
	*cmdPtr++ = (page_addr >> 16U) & 0xFFU;
	*cmdPtr++ = (page_addr >> 8U) & 0xFFU;
	*cmdPtr++ = page_addr & 0xFFU;
	*cmdPtr = 0U;

	W25_CS_LOW();
	SPI_Send(buf, 5);
	SPI_Recv(pBuffer, size);
	W25_CS_HIGH();
	return BL_OK;
}


static BL_Status_t w25_EraseSector(uint32_t addr)
{
	uint8_t* cmdPtr = buf;
	w25_WaitWriteEnd();
	w25_SetBlockProtect(0x00);
	addr = addr * w25_info.SectorSize;
	w25_WriteEnable();
	W25_CS_LOW();
	*cmdPtr++ = W25_SECTOR_ERASE;

	if (w25_info.high_cap)
	{
		*cmdPtr++ = (addr >> 24) & 0xFFU;
		*cmdPtr++ = (addr >> 16) & 0xFFU;
		*cmdPtr++ = (addr >> 8) & 0xFFU;
		*cmdPtr = addr & 0xFFU;
		SPI_Send(buf, 5);
	}
	else
	{
		*cmdPtr++ = (addr >> 16) & 0xFFU;
		*cmdPtr++ = (addr >> 8) & 0xFFU;
		*cmdPtr = addr & 0xFFU;
		SPI_Send(buf, 4);
	}

	W25_CS_HIGH();
	w25_WaitWriteEnd();
	w25_WriteDisable();
	w25_SetBlockProtect(0x0F);
	return BL_OK;
}


static BL_Status_t w25_EraseBlock(uint32_t addr)
{
	uint8_t* cmdPtr = buf;
	w25_WaitWriteEnd();
	addr = addr * w25_info.BlockSize;
	w25_WriteEnable();
	W25_CS_LOW();
	*cmdPtr++ = W25_BLOCK_ERASE;

	if (w25_info.high_cap)
	{
		*cmdPtr++ = (addr >> 24) & 0xFFU;
		*cmdPtr++ = (addr >> 16) & 0xFFU;
		*cmdPtr++ = (addr >> 8) & 0xFFU;
		*cmdPtr++ = addr & 0xFFU;
		SPI_Send(buf, 5);
	}
	else
	{
		*cmdPtr++ = (addr >> 16) & 0xFFU;
		*cmdPtr++ = (addr >> 8) & 0xFFU;
		*cmdPtr++ = addr & 0xFFU;
		SPI_Send(buf, 4);
	}

	W25_CS_HIGH();
	w25_WaitWriteEnd();
	w25_WriteDisable();
	w25_SetBlockProtect(0x0F);
	return BL_OK;
}


BL_Status_t W25Q_EraseChip(void)
{
	uint8_t* cmdPtr = buf;
	w25_WaitWriteEnd();
	w25_WriteEnable();
	W25_CS_LOW();
	*cmdPtr = W25_CHIP_ERASE;
	SPI_Send(buf, 1);
	W25_CS_HIGH();
	w25_WaitWriteEnd();
	w25_WriteDisable();
	w25_SetBlockProtect(0x0F);
	return BL_OK;
}


BL_Status_t W25Q_WriteData(uint32_t addr, const uint8_t* pData, uint32_t size)
{
	uint8_t* cmdPtr = buf;
	w25_WaitWriteEnd();
	w25_SetBlockProtect(0x00);
	w25_WriteEnable();
	W25_CS_LOW();
	*cmdPtr++ = W25_PAGE_PROGRAMM;

	if (w25_info.high_cap)
	{
		*cmdPtr++ = (addr >> 24U) & 0xFFU;
		*cmdPtr++ = (addr >> 16U) & 0xFFU;
		*cmdPtr++ = (addr >> 8U) & 0xFFU;
		*cmdPtr++ = addr & 0xFFU;
		SPI_Send(buf, 5);
	}
	else
	{
		*cmdPtr++ = (addr >> 16U) & 0xFFU;
		*cmdPtr++ = (addr >> 8U) & 0xFFU;
		*cmdPtr++ = addr & 0xFFU;
		SPI_Send(buf, 4);
	}

	SPI_Send(pData, size);
	W25_CS_HIGH();
	w25_WaitWriteEnd();
	w25_WriteDisable();
	w25_SetBlockProtect(0x0F);
	return BL_OK;
}


BL_Status_t W25Q_WritePage(uint32_t page_addr, uint32_t offset, const uint8_t *pData, uint32_t size)
{
	uint8_t* cmdPtr = buf;

	if (size > w25_info.PageSize)
	{
		size = w25_info.PageSize;
	}

	if ((offset + size) > w25_info.PageSize)
	{
		size = w25_info.PageSize - offset;
	}

	page_addr = page_addr * w25_info.PageSize + offset;

	w25_WaitWriteEnd();
	w25_SetBlockProtect(0x00);
	w25_WriteEnable();
	W25_CS_LOW();
	*cmdPtr++ = W25_PAGE_PROGRAMM;

	if (w25_info.high_cap)
	{
		*cmdPtr++ = (page_addr >> 24U) & 0xFFU;
		*cmdPtr++ = (page_addr >> 16U) & 0xFFU;
		*cmdPtr++ = (page_addr >> 8U) & 0xFFU;
		*cmdPtr++ = page_addr & 0xFFU;
		SPI_Send(buf, 5U);
	}
	else
	{
		*cmdPtr++ = (page_addr >> 16U) & 0xFFU;
		*cmdPtr++ = (page_addr >> 8U) & 0xFFU;
		*cmdPtr++ = page_addr & 0xFFU;
		SPI_Send(buf, 4U);
	}

	SPI_Send(pData, size);
	W25_CS_HIGH();
	w25_WaitWriteEnd();
	w25_WriteDisable();
	w25_SetBlockProtect(0x0F);
	return BL_OK;
}

/******************************************************************************
 *                              LOCAL FUNCTIONS                               *
 ******************************************************************************/

static void w25_Reset(void)
{
	uint8_t* cmdPtr = buf;
	W25_CS_LOW();
	*cmdPtr++ = W25_ENABLE_RESET;
	*cmdPtr++ = W25_RESET;
	SPI_Send(buf, 2);
	W25_CS_HIGH();
}


static uint32_t w25_ReadID(void)
{
  uint8_t dt[4];
  buf[0] = W25_GET_JEDEC_ID;
  W25_CS_LOW();
  SPI_Send(buf, 1);
  SPI_Recv(dt,3);
  W25_CS_HIGH();
  return (dt[0] << 16U) | (dt[1] << 8U) | dt[2];
}

static void w25_WriteEnable(void)
{
	W25_CS_LOW();
	buf[0] = W25_WRITE_ENABLE;
	SPI_Send(buf, 1);
	W25_CS_HIGH();
}

static void w25_WriteDisable(void)
{
	W25_CS_LOW();
	buf[0] = W25_WRITE_DISABLE;
	SPI_Send(buf, 1);
	W25_CS_HIGH();
}

static void w25_SetBlockProtect(uint8_t val)
{
    buf[0] = 0x50;
    W25_CS_LOW();
    SPI_Send(buf, 1);
    W25_CS_HIGH();
    buf[0] = W25_WRITE_STATUS_1;
    buf[1] = ((val & 0x0F) << 2);
    W25_CS_LOW();
    SPI_Send(buf, 2);
    W25_CS_HIGH();
}

static void w25_WaitWriteEnd(void)
{
	HAL_Delay(1);
	W25_CS_LOW();
	buf[0] = W25_READ_STATUS_1;
	SPI_Send(buf, 1);

	do
	{
		SPI_Recv(buf, 1);
		w25_info.StatusRegister1 = buf[0];
		HAL_Delay(1);
	}
	while ((w25_info.StatusRegister1 & 0x01U) == 0x01U);

	W25_CS_HIGH();
}

/**
 * @brief Reads metadata (e.g., wear leveling data) from Flash memory.
 * @return BL_OK on success, BL_ERROR otherwise.
 */
static BL_Status_t Metadata_Read(void)
{
    return W25Q_ReadData(w25_info.MetadataSector * w25_info.SectorSize,
                         (uint8_t *)w25_info.EraseCounts,
                         sizeof(w25_info.EraseCounts));
}

/**
 * @brief Writes metadata (e.g., wear leveling data) to Flash memory.
 * @return BL_OK on success, BL_ERROR otherwise.
 */
static BL_Status_t Metadata_Write(void)
{
    return W25Q_WriteData(w25_info.MetadataSector * w25_info.SectorSize,
                          (const uint8_t *)w25_info.EraseCounts,
                          sizeof(w25_info.EraseCounts));
}

/**
 * @brief Finds the least-used block for wear leveling.
 * @return Block index with the lowest erase count.
 */
static uint32_t FindLeastUsedBlock(void)
{
    uint32_t min_count = UINT32_MAX, min_index = 0;

    for (uint32_t i = 0; i < w25_info.BlockCount; ++i)
    {
        if (w25_info.EraseCounts[i] < min_count)
        {
            min_count = w25_info.EraseCounts[i];
            min_index = i;
        }
    }

    return min_index;
}

static void SPI_Send(const uint8_t *pData, uint16_t size)
{
	// TODO: -> CMSIS
	HAL_SPI_Transmit(&hspi1, pData, size, HAL_MAX_DELAY);
}


static void SPI_Recv(uint8_t *pBuffer, uint16_t size)
{
	// TODO: -> CMSIS
	HAL_SPI_Receive(&hspi1, pBuffer, size, HAL_MAX_DELAY);
}
