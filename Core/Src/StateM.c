/*
 * StateM.c
 * State Machine Manager (RTC) for Camera Application
 *  Created on: Oct 8, 2024
 *      Author: K.Rudenko
 */

/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include "StateM.h"
#include "CAMERA_APP.h"
#include "LED.h"
#include "main.h"

/******************************************************************************
 *                           LOCAL DATA TYPES                                 *
 ******************************************************************************/

typedef enum
{
    STATEM_ENTRY,
    STATEM_EXIT,
    STATEM_MAX_ACTIONS,
}StateM_intAction_t;

typedef enum
{
    STATEM_BUSY,
    STATEM_READY,
}StateM_intState_t;

typedef void (*StateM_Action_t)(void);

typedef struct
{
    volatile StateM_signal_t signal;
    StateM_state_t appState;
    StateM_intState_t intState;
    const StateM_state_t* state_table;
    const StateM_Action_t** states_AL_table;
    const StateM_Action_t** transitions_AL_table;
    bool isInitDone;
}StateM_t;

/******************************************************************************
 *                         LOCAL DATA PROTOTYPES                              *
 ******************************************************************************/

/******************************************************************************
 *                             CONFIGURATION                                  *
 ******************************************************************************/

/************************** Action lists for states: **************************/

const static StateM_Action_t IDLE_E_AL[] =
{ /* IDLE state Entry Action List */
    CAM_clrScr,
    CAM_drawIdle,
    NULL,
};

const static StateM_Action_t VIDEO_E_AL[] =
{ /* VIDEO state Entry Action List */
    CAM_clrScr,
    CAM_startVideo,
    NULL,
};

const static StateM_Action_t VIDEO_X_AL[] =
{ /* VIDEO state Exit Action List */
    CAM_stopVideo,
    NULL,
};

const static StateM_Action_t VIEWER_E_AL[] =
{ /* VIEWER state Entry Action List */
    CAM_clrScr,
    CAM_readLastFromSD,
    NULL,
};

/* Entry, Exit Action list table for states */
const static StateM_Action_t* \
StateM_AL_States[STATEM_STATE_MAX_STATES][STATEM_MAX_ACTIONS] =
{   /* STATE,                ENTRY,     |  EXIT  Action Lists */
    [STATEM_STATE_IDLE]   = {IDLE_E_AL ,   NULL               },
    [STATEM_STATE_VIDEO]  = {VIDEO_E_AL,   VIDEO_X_AL         },
    [STATEM_STATE_VIEWER] = {VIEWER_E_AL,  NULL               },
};

/************************ Action lists for transitions: ***********************/

const static StateM_Action_t L_SHORT_PRESS_AL[] =
{ /* LEFT SHORT PRESS_1 Transition Action List: IDLE -> VIDEO */
    LED_onePulse,
    NULL,
};

const static StateM_Action_t L_SHORT_PRESS_2_AL[] =
{ /* LEFT SHORT PRESS_2 Transition Action List: VIEWER_NextPhoto */
    LED_onePulse,
    CAM_readNextFromSD,
    NULL,
};

const static StateM_Action_t R_SHORT_PRESS_2_AL[] =
{ /* RIGHT SHORT PRESS_2 Transition Action List: VIEWER_PrevPhoto */
    LED_onePulse,
    CAM_readPrevFromSD,
    NULL,
};

const static StateM_Action_t L_SHORT_PRESS_3_AL[] =
{ /* LEFT SHORT PRESS_3 Transition Action List: VIDEO_takePhoto */
    CAM_stopVideo,
    LED_startBlinking,
    CAM_writeToSD,
    LED_stopBlinking,
    CAM_clrScr,
    CAM_startVideo,
    NULL,
};

const static StateM_Action_t L_DOUBLE_PRESS_AL[] =
{ /* LEFT DOUBLE PRESS Transition Action List:  VIDEO -> IDLE   */
    LED_onePulse,
    NULL,
};

const static StateM_Action_t R_DOUBLE_PRESS_AL[] =
{ /* RIGHT DOUBLE PRESS Transition Action List: VIDEO -> VIEWER, VIEWER -> VIDEO */
    LED_onePulse,
    NULL,
};

/* Action list table for transitions */
const static StateM_Action_t* \
StateM_AL_Transitions[STATEM_STATE_MAX_STATES][STATEM_SIGNAL_MAX_SIGNALS] =
    /*                      |                                                         SIGNALS AND TRANSITIONS ACTION LISTS                                                                                                                              */
{   /* SOURCE STATES        |   L SHORT_PRESS        | L DOUBLE_PRESS        | L LONG_PRESS          | R SHORT_PRESS          | R DOUBLE_PRESS         | R LONG_PRESS           | L SHORT_PRESS_2       | R SHORT_PRESS_2       | L SHORT_PRESS_3       */
    [STATEM_STATE_IDLE]     = { L_SHORT_PRESS_AL,      NULL,                   NULL,                   NULL,                    NULL,                    NULL,                    NULL,                   NULL,                   NULL                  },
    [STATEM_STATE_VIDEO]    = { NULL,                  L_DOUBLE_PRESS_AL,      NULL,                   NULL,                    R_DOUBLE_PRESS_AL,       NULL,                    NULL,                   NULL,                   L_SHORT_PRESS_3_AL    },
    [STATEM_STATE_VIEWER]   = { NULL,                  NULL,                   NULL,                   NULL,                    R_DOUBLE_PRESS_AL,       NULL,                    L_SHORT_PRESS_2_AL,     R_SHORT_PRESS_2_AL,     NULL                  },
};

/* State Machine Table */
const static StateM_state_t \
StateM_StateTable[STATEM_STATE_MAX_STATES][STATEM_SIGNAL_MAX_SIGNALS] =
    /*                      |                                                         SIGNALS AND TARGET STATES                                                                                                                                         */
{   /* SOURCE STATES        |   L SHORT_PRESS        | L DOUBLE_PRESS        | L LONG_PRESS          | R SHORT_PRESS          | R DOUBLE_PRESS         | R LONG_PRESS           | L SHORT_PRESS_2       | R SHORT_PRESS_2       | L SHORT_PRESS_3       */
    [STATEM_STATE_IDLE]     = {STATEM_STATE_VIDEO,     STATEM_STATE_NO_STATE,  STATEM_STATE_NO_STATE,  STATEM_STATE_NO_STATE,   STATEM_STATE_NO_STATE,   STATEM_STATE_NO_STATE,   STATEM_STATE_NO_STATE,  STATEM_STATE_NO_STATE,  STATEM_STATE_NO_STATE },
    [STATEM_STATE_VIDEO]    = {STATEM_STATE_NO_STATE,  STATEM_STATE_IDLE,      STATEM_STATE_NO_STATE,  STATEM_STATE_NO_STATE,   STATEM_STATE_VIEWER,     STATEM_STATE_NO_STATE,   STATEM_STATE_NO_STATE,  STATEM_STATE_NO_STATE,  STATEM_STATE_VIDEO    },
    [STATEM_STATE_VIEWER]   = {STATEM_STATE_NO_STATE,  STATEM_STATE_NO_STATE,  STATEM_STATE_NO_STATE,  STATEM_STATE_NO_STATE,   STATEM_STATE_VIDEO,      STATEM_STATE_NO_STATE,   STATEM_STATE_VIEWER,    STATEM_STATE_VIEWER,    STATEM_STATE_NO_STATE },
};

/********************************************************************************************************************************/
/*                                                  STATE MACHINE DIAGRAM                                                       */
/*      Initial Transition                                                                                                      */
/*              O                                                                                                               */
/*              |                                                                                                               */
/*    __________V__________                         _______________________                     _________________________       */
/*   |  STATEM_STATE_IDLE  |                       |   STATEM_STATE_VIDEO  |                   |   STATEM_STATE_VIEWER   |      */
/*   |                     |   L_SHORT_PRESS/      |                       |  R_DOUBLE_PRESS/  |                         |      */
/*   |  entry/             |---------------------->|  entry/               |------------------>|  entry/                 |      */
/*   |   CAM_clrScr();     |   LED_onePulse();     |   CAM_clrScr();       |  LED_onePulse();  |   CAM_clrScr();         |      */
/*   |   CAM_drawIdle();   |                       |   CAM_startVideo();   |                   |   CAM_readLastFromSD(); |      */
/*   |  -----------------  |                       |  -------------------  |                   |  ---------------------  |      */
/*   | exit/               |                       | exit/                 |                   | exit/                   |      */
/*   |                     |                       |   CAM_stopVideo();    |                   |                         |      */
/*   |  -----------------  |                       |  -------------------  |                   |  ---------------------  |      */
/*   |                     |                       |                       |                   |                         |      */
/*   |                     |                       | L_SHORT_PRESS_3/      |                   | L_SHORT_PRESS_2/        |      */
/*   |                     |                       |   CAM_stopVideo();    |                   |   LED_onePulse();       |      */
/*   |                     |                       |   LED_startBlinking();|                   |   CAM_readNextFromSD(); |      */
/*   |                     |                       |   CAM_writeToSD();    |                   |------------->           |      */
/*   |                     |                       |   LED_stopBlinking(); |                   |                         |      */
/*   |                     |                       |   CAM_clrScr();       |                   | R_SHORT_PRESS_2/        |      */
/*   |                     |                       |   CAM_startVideo();   |                   |   LED_onePulse();       |      */
/*   |                     |                       |------------>          |                   |   CAM_readPrevFromSD(); |      */
/*   |                     |                       |                       |                   |------------->           |      */
/*   |                     |   L_DOUBLE_PRESS/     |                       |  R_DOUBLE_PRESS/  |                         |      */
/*   |                     |<----------------------|                       |<------------------|                         |      */
/*   |                     |   LED_onePulse();     |                       |  LED_onePulse();  |                         |      */
/*   |_____________________|                       |_______________________|                   |_________________________|      */
/*                                                                                                                              */
/********************************************************************************************************************************/

static StateM_t StateM =
{
    .signal = STATEM_SIGNAL_NO_SIGNAL,
    .state_table = &StateM_StateTable[0][0],
    .states_AL_table = &StateM_AL_States[0][0],
    .transitions_AL_table = &StateM_AL_Transitions[0][0],
};

/******************************************************************************
 *                       LOCAL FUNCTIONS PROTOTYPES                           *
 ******************************************************************************/

static void StateM_LRunActionList(const StateM_Action_t* AL);

/******************************************************************************
 *                              GLOBAL FUNCTIONS                              *
 ******************************************************************************/

void StateM_Init(void)
{
    if (StateM.isInitDone == false)
    {
        // Switch internal StateM state to BUSY
        StateM.intState = STATEM_BUSY;

        // Run Entry Action List of Initial state
        StateM_LRunActionList(StateM_AL_States[StateM.appState][STATEM_ENTRY]);

        // Set internal StateM status to Initialized
        StateM.isInitDone = true;

        // Switch internal StateM state to READY
        StateM.intState = STATEM_READY;
    }
}

void StateM_Dispatch(void)
{
    StateM_state_t sourse, target;

    if (StateM.signal < STATEM_SIGNAL_MAX_SIGNALS && StateM.isInitDone == true)
    {
        // Get the source and target states
        sourse = StateM.appState;
        target = StateM.state_table[sourse * STATEM_SIGNAL_MAX_SIGNALS + StateM.signal];

        if (target != STATEM_STATE_NO_STATE && target != sourse)
        {
            /* Do external  transition */

            // Switch internal StateM state to BUSY
            StateM.intState = STATEM_BUSY;

            // Run Exit Action List
            StateM_LRunActionList(StateM.states_AL_table[sourse * STATEM_MAX_ACTIONS + STATEM_EXIT]);

            // Run Transition Action List
            StateM_LRunActionList(StateM.transitions_AL_table[sourse * STATEM_SIGNAL_MAX_SIGNALS + StateM.signal]);

            // Update current state
            StateM.appState = target;

            // Run Entry Action List
            StateM_LRunActionList(StateM.states_AL_table[target * STATEM_MAX_ACTIONS + STATEM_ENTRY]);

            // Switch internal StateM state to READY
            StateM.intState = STATEM_READY;
        }
        else if (target == sourse)
        {
            /* Do internal transition */

            // Switch internal StateM state to BUSY
            StateM.intState = STATEM_BUSY;

            // Run Transition Action List
            StateM_LRunActionList(StateM.transitions_AL_table[sourse * STATEM_SIGNAL_MAX_SIGNALS + StateM.signal]);

            // Switch internal StateM state to READY
            StateM.intState = STATEM_READY;
        }
    }

    // Reset signal to STATEM_SIGNAL_NO_SIGNAL:
    StateM.signal = STATEM_SIGNAL_NO_SIGNAL;
}

void StateM_SetSignal(StateM_signal_t signal)
{
    __disable_irq();
    StateM.signal = signal;
    __enable_irq();
}

StateM_state_t StateM_GetState(void)
{
    return StateM.appState;
}

/******************************************************************************
 *                              LOCAL FUNCTIONS                               *
 ******************************************************************************/

static void StateM_LRunActionList(const StateM_Action_t* AL)
{
    uint32_t idx = 0U;

    if (AL != NULL)
    {
        while (*AL[idx] != NULL)
        {
            AL[idx++]();
        }
    }
}

