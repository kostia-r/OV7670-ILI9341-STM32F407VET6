/*
 * flash.h
 * Non-volatile Memory Manager
 *  Created on: Dec 5, 2024
 *      Author: K.Rudenko
 */

#ifndef NVM_H_
#define NVM_H_

/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "W25Q16.h"

/******************************************************************************
 *                               GLOBAL MACRO                                 *
 ******************************************************************************/
/* Configuration parameters */
#define MAX_LOGICAL_BLOCKS 16        // Maximum number of logical blocks managed by the NvM component
#define MAX_PHYSICAL_BLOCKS 64       // Maximum number of physical blocks available in flash memory
#define FLASH_BLOCK_SIZE 4096        // Size of a single physical block in bytes
#define FLASH_SIZE (FLASH_BLOCK_SIZE * MAX_PHYSICAL_BLOCKS) // Total flash memory size
#define RESERVED_METADATA_BLOCK_ADDR 0x0000 // Address of the reserved block for metadata storage

/******************************************************************************
 *                           GLOBAL DATA TYPES                                *
 ******************************************************************************/

/* Return type definition for NvM functions */
typedef enum {
    E_OK = 0,           // Operation completed successfully
    E_ERROR = -1,       // General error
    E_INVALID_BLOCK = -2, // Invalid block ID or block does not exist
    E_FLASH_ERROR = -3, // Error occurred during flash operation
    E_OUT_OF_MEMORY = -4, // Not enough space to complete the operation
    E_WEAR_LEVELING_ERROR = -5 // Error related to wear leveling or garbage collection
} NvM_ReturnType;

/* Structure representing the state of a block being read */
typedef struct {
    uint32_t block_id;    // ID of the logical block being read
    uint32_t offset;      // Current offset within the block
    uint32_t total_size;  // Total size of the block
} NvM_ReadState;


/******************************************************************************
 *                        GLOBAL DATA PROTOTYPES                              *
 ******************************************************************************/

/******************************************************************************
 *                      GLOBAL FUNCTIONS PROTOTYPES                           *
 ******************************************************************************/
/* Initializes the NvM component */
extern NvM_ReturnType NvM_Init(void);
/* Writes a logical block to flash memory */
extern NvM_ReturnType NvM_Write(uint32_t block_id, const uint8_t *data, uint32_t size);
/* Reads a logical block into a buffer */
extern NvM_ReturnType NvM_Read(uint32_t block_id, uint8_t *buffer, uint32_t size);
/* Begins reading a logical block piece by piece */
extern NvM_ReturnType NvM_BeginRead(uint32_t block_id, NvM_ReadState *state);
/* Reads a chunk of data from a logical block */
extern NvM_ReturnType NvM_ReadChunk(NvM_ReadState *state, uint8_t *buffer, uint32_t chunk_size);
/* Ends the read operation for a logical block */
extern NvM_ReturnType NvM_EndRead(NvM_ReadState *state);
/* Retrieves the size of a logical block */
extern uint32_t NvM_GetLogicalBlockSize(uint32_t block_id);
/* Debugging utility to print the metadata and state of blocks */
extern void NvM_DebugDump(void);

#endif /* NVM_H_ */
