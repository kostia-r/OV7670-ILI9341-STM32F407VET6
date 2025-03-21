/*
 * NvM.c
 * Non-volatile Memory Manager
 *  Created on: Dec 5, 2024
 *      Author: K.Rudenko
 */

/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/

#include "NvM.h"
#include "crc.h"
#include "main.h"

#include <string.h> // memcpy

/******************************************************************************
 *                            LOCAL DATA TYPES                                *
 ******************************************************************************/
//typedef enum
//{
//	NVM_APPLICATION_BLOCK,
//	NVM_BOOTLOADER_BLOCK,
//	LOGO_BLOCK,
//	MAX_BLOCKS,
//}NvM_BlockID_t;
//
//typedef struct
//{
//	NvM_BlockID_t blockID;
//	uint32_t dataSize;
//	uint32_t instance;
//	uint32_t crc;
//}NvM_DataBlock_t;

/* Logical block metadata structure */
typedef struct {
    uint32_t id;                    // Logical block ID
    uint32_t size;                  // Size of the logical block in bytes
    uint32_t num_physical;          // Number of physical blocks used by this logical block
    uint32_t physical_blocks[MAX_PHYSICAL_BLOCKS]; // List of physical blocks associated with this logical block
    uint32_t crc;                   // CRC32 checksum for data integrity verification
} LogicalBlockMetadata;

/******************************************************************************
 *                               LOCAL MACRO                                  *
 ******************************************************************************/

/******************************************************************************
 *                        GLOBAL DATA PROTOTYPES                              *
 ******************************************************************************/

/* Internal metadata tables */
static LogicalBlockMetadata block_table[MAX_LOGICAL_BLOCKS]; // Table containing metadata for all logical blocks
static uint32_t wear_leveling_map[MAX_PHYSICAL_BLOCKS];      // Wear leveling table mapping logical to physical blocks
static uint32_t wear_counter[MAX_PHYSICAL_BLOCKS];          // Tracks the wear level (number of writes) for each physical block

/******************************************************************************
 *                         LOCAL DATA PROTOTYPES                              *
 ******************************************************************************/

/******************************************************************************
 *                       LOCAL FUNCTIONS PROTOTYPES                           *
 ******************************************************************************/

static uint32_t NvM_CalculateCRC(const uint8_t *data, uint32_t size);
static NvM_ReturnType NvM_LoadMetadata(void);
static NvM_ReturnType NvM_SaveMetadata(void);
static uint32_t NvM_AllocatePhysicalBlock(void);
static NvM_ReturnType NvM_GarbageCollect(void);

/******************************************************************************
 *                            GLOBAL FUNCTIONS                                *
 ******************************************************************************/

/* Initialization function */
NvM_ReturnType NvM_Init(void) {
    NvM_ReturnType status = E_OK;

    // Load metadata from the reserved sector in flash memory
    status = NvM_LoadMetadata();
    if (status != E_OK) {
        // If loading fails, reset the metadata to default state
        printf("Failed to load metadata. Resetting to default state.\n");
        memset(block_table, 0xFF, sizeof(block_table)); // Mark all entries as unused
        memset(wear_leveling_map, 0xFF, sizeof(wear_leveling_map)); // Clear wear leveling map
    }

    // Initialize wear counters and wear leveling map
    for (int i = 0; i < MAX_PHYSICAL_BLOCKS; i++) {
        wear_counter[i] = 0;       // Reset the wear counter for each physical block
        wear_leveling_map[i] = i; // Initially map logical to physical 1:1
    }

    return status;
}

/* Retrieves the size of a logical block */
uint32_t NvM_GetLogicalBlockSize(uint32_t block_id) {
    for (int i = 0; i < MAX_LOGICAL_BLOCKS; i++) {
        if (block_table[i].id == block_id) {
            return block_table[i].size; // Return the size of the block
        }
    }
    return 0; // Return 0 if the block does not exist
}

/* Writes data to a logical block */
NvM_ReturnType NvM_Write(uint32_t block_id, const uint8_t *data, uint32_t size) {
    NvM_ReturnType status = E_ERROR;
    do {
        LogicalBlockMetadata *block = NULL;

        // Search for an existing block or an empty slot
        for (int i = 0; i < MAX_LOGICAL_BLOCKS; i++) {
            if (block_table[i].id == block_id || block_table[i].id == 0xFFFFFFFF) {
                block = &block_table[i];
                break;
            }
        }
        if (!block) {
            status = E_OUT_OF_MEMORY;
            break;
        }

        // Allocate a new block if not found
        if (block->id == 0xFFFFFFFF) {
            block->id = block_id;
        }

        // Perform garbage collection if necessary
        if (NvM_GarbageCollect() != E_OK) {
            status = E_WEAR_LEVELING_ERROR;
            break;
        }

        // Write data to physical blocks, splitting into chunks as necessary
        uint32_t remaining = size;
        uint32_t written = 0;
        uint32_t current_physical_block = 0;

        while (remaining > 0) {
            uint32_t chunk_size = (remaining > FLASH_BLOCK_SIZE) ? FLASH_BLOCK_SIZE : remaining;

            uint32_t physical_address = NvM_AllocatePhysicalBlock();
            if (physical_address == 0xFFFFFFFF) {
                status = E_OUT_OF_MEMORY;
                break;
            }

            status = W25Q_WriteData(physical_address, &data[written], chunk_size);
            if (status != E_OK) break;

            block->physical_blocks[current_physical_block++] = physical_address;
            written += chunk_size;
            remaining -= chunk_size;
        }

        if (status != E_OK) break;

        // Update metadata for the block
        block->size = size;
        block->crc = NvM_CalculateCRC(data, size);

        // Save updated metadata to flash
        status = NvM_SaveMetadata();
    } while (0);

    return status;
}

/* Reads data from a logical block */
NvM_ReturnType NvM_Read(uint32_t block_id, uint8_t *buffer, uint32_t size) {
    NvM_ReturnType status = E_ERROR;
    do {
        LogicalBlockMetadata *block = NULL;

        // Find the metadata for the requested block
        for (int i = 0; i < MAX_LOGICAL_BLOCKS; i++) {
            if (block_table[i].id == block_id) {
                block = &block_table[i];
                break;
            }
        }
        if (!block || size > block->size) {
            status = E_INVALID_BLOCK;
            break;
        }

        // Read data from the associated physical blocks
        uint32_t remaining = size;
        uint32_t read = 0;
        uint32_t current_physical_block = 0;

        while (remaining > 0) {
            uint32_t chunk_size = (remaining > FLASH_BLOCK_SIZE) ? FLASH_BLOCK_SIZE : remaining;
            uint32_t physical_address = wear_leveling_map[block->physical_blocks[current_physical_block]];

            status = W25Q_ReadData(physical_address, &buffer[read], chunk_size);
            if (status != E_OK) break;

            read += chunk_size;
            remaining -= chunk_size;
            current_physical_block++;
        }

        if (status != E_OK) break;

        // Verify data integrity using CRC32
        uint32_t calculated_crc = NvM_CalculateCRC(buffer, size);
        if (calculated_crc != block->crc) {
            status = E_FLASH_ERROR;
            break;
        }

        status = E_OK;
    } while (0);

    return status;
}

/* Begins reading a logical block in chunks.
 * This is useful when the block is too large to fit into memory at once.
 */
NvM_ReturnType NvM_BeginRead(uint32_t block_id, NvM_ReadState *state) {
    NvM_ReturnType status = E_ERROR;

    do {
        LogicalBlockMetadata *block = NULL;

        // Locate the metadata for the block
        for (uint32_t i = 0; i < MAX_LOGICAL_BLOCKS; i++) {
            if (block_table[i].id == block_id) {
                block = &block_table[i];
                break;
            }
        }

        if (!block) {
            status = E_INVALID_BLOCK; // Block not found
            break;
        }

        // Initialize the read state
        state->block_id = block_id;
        state->offset = 0;
        state->total_size = block->size;

        status = E_OK;
    } while (0);

    return status;
}

/* Reads a chunk of data from the current logical block.
 * This is part of the chunked read process started by `NvM_BeginRead`.
 */
NvM_ReturnType NvM_ReadChunk(NvM_ReadState *state, uint8_t *buffer, uint32_t chunk_size) {
    NvM_ReturnType status = E_ERROR;

    do {
        if (state->offset >= state->total_size) {
            status = E_ERROR; // No more data to read
            break;
        }

        uint32_t remaining = state->total_size - state->offset;
        chunk_size = (chunk_size < remaining) ? chunk_size : remaining;

        uint32_t physical_block_index = state->offset / FLASH_BLOCK_SIZE;
        uint32_t physical_offset = state->offset % FLASH_BLOCK_SIZE;

        uint32_t physical_address = block_table[state->block_id].physical_blocks[physical_block_index] + physical_offset;

        // Read the chunk from the flash memory
        status = W25Q_ReadData(physical_address, buffer, chunk_size);
        if (status != E_OK) {
            break;
        }

        state->offset += chunk_size; // Update the read offset
        status = E_OK;
    } while (0);

    return status;
}

/* Ends a chunked read operation.
 * Currently, this is a placeholder for potential cleanup steps.
 */
NvM_ReturnType NvM_EndRead(NvM_ReadState *state)
{
    return E_OK; // No special actions needed for cleanup in this example
}

/* Debugging utility to dump the current metadata and wear state.
 * This function outputs the logical block table and wear counters for analysis.
 */
void NvM_DebugDump(void)
{
    printf("=== Metadata Dump ===\n");
    for (uint32_t i = 0; i < MAX_LOGICAL_BLOCKS; i++)
    {
        if (block_table[i].id != 0xFFFFFFFF)
        {
            printf("Block ID: %lu, Size: %lu bytes, CRC: 0x%08lX\n",
                   block_table[i].id, block_table[i].size, block_table[i].crc);
        }
    }

    printf("=== Wear Counter Dump ===\n");
    for (uint32_t i = 0; i < MAX_PHYSICAL_BLOCKS; i++)
    {
        printf("Block %lu: Wear Count = %lu\n", i, wear_counter[i]);
    }
}


/******************************************************************************
 *                              LOCAL FUNCTIONS                               *
 ******************************************************************************/
/* Saves the metadata table to a reserved sector in flash memory.
 * This ensures that the metadata remains persistent across power cycles.
 */
static NvM_ReturnType NvM_SaveMetadata(void) {
    return W25Q_WriteData(RESERVED_METADATA_BLOCK_ADDR, (uint8_t *)block_table, sizeof(block_table));
}

/* Loads the metadata table from the reserved sector in flash memory.
 * If this fails, the system assumes the metadata is invalid or absent.
 */
static NvM_ReturnType NvM_LoadMetadata(void) {
    return W25Q_ReadData(RESERVED_METADATA_BLOCK_ADDR, (uint8_t *)block_table, sizeof(block_table));
}

/* Allocates a new physical block for writing.
 * This function uses a wear-leveling algorithm to minimize wear on the flash.
 * It selects the physical block with the lowest write count.
 */
static uint32_t NvM_AllocatePhysicalBlock(void) {
    uint32_t min_wear_level = UINT32_MAX; // Start with the highest possible wear count
    uint32_t selected_block = 0xFFFFFFFF; // Invalid block ID

    // Find the block with the lowest wear count
    for (uint32_t i = 0; i < MAX_PHYSICAL_BLOCKS; i++) {
        if (wear_counter[i] < min_wear_level) {
            min_wear_level = wear_counter[i];
            selected_block = i;
        }
    }

    // Update the wear counter for the selected block
    if (selected_block != 0xFFFFFFFF) {
        wear_counter[selected_block]++;
    }

    return selected_block;
}

/* Performs garbage collection to reclaim space from old or unused blocks.
 * This process erases physical blocks that are no longer referenced by the metadata table.
 */
static NvM_ReturnType NvM_GarbageCollect(void)
{
    NvM_ReturnType status = E_OK;

    for (uint32_t i = 0; i < MAX_PHYSICAL_BLOCKS; i++)
    {
    	// Calculate the start address of the current block
    	uint32_t block_address = i * FLASH_BLOCK_SIZE;

        // Check if the block is excessively worn or no longer in use
        if (wear_counter[i] > 100)
        {
        	// Example threshold for garbage collection
            status = W25Q_Erase(block_address, FLASH_BLOCK_SIZE);

            if (status != E_OK)
            {
                printf("Garbage collection failed on block %lu.\n", i);
                return E_FLASH_ERROR;
            }

            // Reset the wear counter for the erased block
            wear_counter[i] = 0;
        }
    }

    return status;
}

/* Computes the CRC32 checksum for a block of data.
 * This function uses the STM32 HAL CRC peripheral to offload computation.
 */
static uint32_t NvM_CalculateCRC(const uint8_t *data, uint32_t size) {
    CRC_HandleTypeDef hcrc;
    HAL_CRC_Init(&hcrc); // Initialize the HAL CRC peripheral
    return HAL_CRC_Calculate(&hcrc, (uint32_t *)data, size / sizeof(uint32_t));
}

