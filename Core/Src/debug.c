/*
 * debug.c
 * Debug module to perform debug output (ITM SWO),
 * stack analysis, and time measurements
 * Created on: Jul 26, 2024
 *     Author: k.rudenko
 */


#include "debug.h"
#include "stm32f4xx_hal.h"
#include "core_cm4.h"

#if (DEBUG_EN == 1)

/* Time measurement */
static volatile uint32_t startTick, endTick;
volatile uint32_t DEBUG_elapsedTimeMicroseconds;
volatile uint32_t DEBUG_fps;


/* Stack consumption measurement */
extern uint32_t _estack; // End of the stack memory
extern uint32_t _Min_Stack_Size; // stack size in bytes
volatile uint32_t DEBUG_usedStackBytes, DEBUG_stackUsagePercentage;

/* Redirecting printf() output to SWO ITM */
int _write(int file, char *ptr, int len)
{
  /* Implement your write code here, this is used by puts and printf for example */
  int i=0;

  for(i=0 ; i<len ; i++)
  {
	  ITM_SendChar((*ptr++));
  }

  return len;
}

void StackPaint(void)
{
    uint32_t *stack = (uint32_t*) (&_estack - ((uint32_t) &_Min_Stack_Size / sizeof(uint32_t))); // Bottom of stack
    uint32_t *end = (uint32_t*) &_estack;  // Top of stack

    while (stack < end)
    {
        *stack++ = 0xAAAAAAAA;  // Fill stack with test pattern (0xAA)
    }
}

void MeasureStackUsage(void)
{
    uint32_t *stack = (uint32_t*) (&_estack
            - ((uint32_t) &_Min_Stack_Size / sizeof(uint32_t)));
    uint32_t *end = (uint32_t*) &_estack;
    uint32_t totalStackSize = (uint32_t) &_Min_Stack_Size; // Total stack size in bytes

    // Scan for the overwritten pattern (0xAAAAAAAA) from the stack start to the end
    while (stack < end && *stack == 0xAAAAAAAA)
    {
        stack++;
    }

    // Calculate used stack in bytes
    DEBUG_usedStackBytes = (uint32_t) end - (uint32_t) stack;

    // Calculate stack usage percentage (integer)
    DEBUG_stackUsagePercentage = (DEBUG_usedStackBytes * 100) / totalStackSize;
}

void DEBUG_MeasStart(void)
{
    startTick = SysTick->VAL;
}

void DEBUG_MeasStop(void)
{
    endTick = SysTick->VAL;
    // since each counter increment represents 125 nanoseconds (1ms/8MHz = 0.125 microseconds),
    // and reload value is reached every 1ms
    DEBUG_elapsedTimeMicroseconds = ((endTick - startTick) * 125UL) / 1000UL;
    startTick = 0U;
    endTick = 0U;

}

void DEBUG_MeasPeriod(void)
{
    static uint32_t call_count = 0U;

    call_count++;

    if (call_count == 1U)
    {
        // First call: only call DEBUG_MeasStart()
        DEBUG_MeasStart();
    }
    else
    {
        DEBUG_MeasStop();
        DEBUG_MeasStart();
    }
}

void DEBUG_MeasFps(void)
{
    static uint32_t call_count = 0U;
    static uint32_t startMS, endMS;

    call_count++;

    if (call_count == 1U)
    {
        startMS = HAL_GetTick();
    }
    else
    {
        endMS = HAL_GetTick();
        DEBUG_fps = 1000UL / (endMS - startMS);
        startMS = HAL_GetTick();
    }
}

#endif /* DEBUG_EN */

