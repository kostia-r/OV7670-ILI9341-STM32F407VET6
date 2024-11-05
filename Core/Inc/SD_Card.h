/*
 * SD_Card.h
 * SD Card Photo Viewer
 *  Created on: Nov 5, 2024
 *      Author: K.Rudenko
 */

#ifndef SD_CARD_H_
#define SD_CARD_H_

/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/

#include "main.h"

/******************************************************************************
 *                        GLOBAL DATA PROTOTYPES                              *
 ******************************************************************************/

// Date and Time (preconfigured) of creation for photos saved to the SD Card:
extern uint16_t year;
extern uint8_t month;
extern uint8_t day;
extern uint8_t hours;
extern uint8_t minutes;
extern uint8_t seconds;
/******************************************************************************
 *                      GLOBAL FUNCTIONS PROTOTYPES                           *
 ******************************************************************************/
extern void SD_PhotoViewer_Init(void);
extern void SD_PhotoViewer_Prev(void);
extern void SD_PhotoViewer_Next(void);
extern void SD_PhotoViewer_Open(void);
extern void SD_PhotoViewer_Save(void);

#endif /* SD_CARD_H_ */
