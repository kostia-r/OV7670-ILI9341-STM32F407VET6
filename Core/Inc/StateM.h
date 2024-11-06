/*
 * StateM.h
 * State Machine Manager (RTC) for Camera Application
 *  Created on: Oct 8, 2024
 *      Author: K.Rudenko
 */

#ifndef STATEM_H_
#define STATEM_H_

/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/

#include "stdbool.h"

#define STATEM_ERR_CODE                                                (0xFFFFU)

/******************************************************************************
 *                            CONFIGURATION ENUMS                             *
 ******************************************************************************/

typedef enum
{
    STATEM_STATE_IDLE,     // <-- Initial state
    STATEM_STATE_VIDEO,
    STATEM_STATE_VIEWER,
    STATEM_STATE_MAX_STATES,
    STATEM_STATE_NO_STATE = STATEM_ERR_CODE,
}StateM_state_t;

typedef enum
{
    STATEM_SIGNAL_L_SHORT_PRESS,
    STATEM_SIGNAL_L_DOUBLE_PRESS,
    STATEM_SIGNAL_L_LONG_PRESS,     // not used
    STATEM_SIGNAL_R_SHORT_PRESS,
    STATEM_SIGNAL_R_DOUBLE_PRESS,
    STATEM_SIGNAL_R_LONG_PRESS,     // not used
    STATEM_SIGNAL_L_SHORT_PRESS_2,
    STATEM_SIGNAL_R_SHORT_PRESS_2,
    STATEM_SIGNAL_L_SHORT_PRESS_3,
    STATEM_SIGNAL_MAX_SIGNALS,
    STATEM_SIGNAL_NO_SIGNAL = STATEM_ERR_CODE,
}StateM_signal_t;

/******************************************************************************
 *                      GLOBAL FUNCTIONS PROTOTYPES                           *
 ******************************************************************************/

extern void StateM_Init(void);
void StateM_SetSignal(StateM_signal_t signal);
extern void StateM_Dispatch(void);
extern StateM_state_t StateM_GetState(void);

#endif /* STATEM_H_ */
