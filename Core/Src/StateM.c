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

const static StateM_Action_t PHOTO_E_AL[] =
{ /* PHOTO state Entry Action List */
    CAM_takePhoto,

    LED_startBlinking,
    CAM_writeToSD,
    LED_stopBlinking,
    NULL,
};

/* Entry, Exit Action list table for states */
const static StateM_Action_t* \
StateM_AL_States[STATEM_STATE_MAX_STATES][STATEM_MAX_ACTIONS] =
{   /* STATE,               ENTRY,      EXIT  Action Lists */
    [STATEM_STATE_IDLE]  = {IDLE_E_AL , NULL      },
    [STATEM_STATE_VIDEO] = {VIDEO_E_AL, VIDEO_X_AL},
    [STATEM_STATE_PHOTO] = {PHOTO_E_AL, NULL      },
};

/************************ Action lists for transitions: ***********************/

const static StateM_Action_t L_SHORT_PRESS_AL[] =
{ /* LEFT SHORT PRESS Transition Action List */
    LED_onePulse,
    NULL,
};

const static StateM_Action_t L_LONG_PRESS_AL[] =
{ /* LEFT LONG PRESS Transition Action List */
    LED_startBlinking,
    CAM_writeToSD,
    LED_stopBlinking,
    NULL,
};

/* Action list table for transitions */
const static StateM_Action_t* \
StateM_AL_Transitions[STATEM_STATE_MAX_STATES][STATEM_SIGNAL_MAX_SIGNALS] =
{   /* Source state      |  L SHORT_PRESS    | L DOUBLE_PRESS   | L LONG_PRESS | R SHORT_PRESS | R DOUBLE_PRESS | R LONG_PRESS */
    [STATEM_STATE_IDLE]  = { L_SHORT_PRESS_AL,   NULL,            NULL,          NULL,           NULL,            NULL         },
    [STATEM_STATE_VIDEO] = { L_SHORT_PRESS_AL,   NULL,            NULL,          NULL,           NULL,            NULL         },
    [STATEM_STATE_PHOTO] = { L_SHORT_PRESS_AL,   L_LONG_PRESS_AL, NULL,          NULL,           NULL,            NULL         },
};

/* State Machine Table */
const static StateM_state_t \
StateM_StateTable[STATEM_STATE_MAX_STATES][STATEM_SIGNAL_MAX_SIGNALS] =
{   /* Source state/trigger |   L SHORT_PRESS      | L DOUBLE_PRESS       | L LONG_PRESS          | R SHORT_PRESS          | R DOUBLE_PRESS         | R LONG_PRESS */
    [STATEM_STATE_IDLE]     = {STATEM_STATE_VIDEO,  STATEM_STATE_NO_STATE,  STATEM_STATE_NO_STATE,  STATEM_STATE_NO_STATE,   STATEM_STATE_NO_STATE,   STATEM_STATE_NO_STATE },  /* Tar-   */
    [STATEM_STATE_VIDEO]    = {STATEM_STATE_PHOTO,  STATEM_STATE_IDLE,      STATEM_STATE_NO_STATE,  STATEM_STATE_NO_STATE,   STATEM_STATE_NO_STATE,   STATEM_STATE_NO_STATE },  /* get    */
    [STATEM_STATE_PHOTO]    = {STATEM_STATE_VIDEO,  STATEM_STATE_NO_STATE,  STATEM_STATE_VIDEO,     STATEM_STATE_NO_STATE,   STATEM_STATE_NO_STATE,   STATEM_STATE_NO_STATE },  /* states */
};

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

