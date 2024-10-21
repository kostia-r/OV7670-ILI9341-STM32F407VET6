/*
 * debug.h
 * Debug module to perform debug output (ITM SWO),
 * stack analysis, and time measurements
 * Created on: Jul 26, 2024
 *     Author: k.rudenko
 */

#ifndef INC_DEBUG_H_
#define INC_DEBUG_H_

#if (DEBUG_EN == 1)
#include <stdio.h>
#include <stdint.h>

/* Redirecting printf() output to SWO ITM */
extern int _write(int file, char *ptr, int len);

#define   DEBUG_LOG_OPT                      1
#if (DEBUG_LOG_OPT == 1)
#define DEBUG_LOG(...)                       do{ printf(__VA_ARGS__);\
		                                     printf("\n"); } while (0)
#else  /* (DEBUG_LOG_OPT == 1) */
#define DEBUG_LOG(...)                       printf(__VA_ARGS__"\n")
#endif /* (DEBUG_LOG_OPT != 1) */

/* Stack consumption measurement */
extern volatile uint32_t DEBUG_usedStackBytes, DEBUG_stackUsagePercentage;

extern void StackPaint(void);
extern void MeasureStackUsage(void);

#define DEBUG_STACK_PAINT()                  StackPaint()
#define DEBUG_STACK_ANALYZE()                MeasureStackUsage()

/* Time measurement */
extern volatile uint32_t DEBUG_elapsedTimeMicroseconds;

extern void DEBUG_MeasStart(void);
extern void DEBUG_MeasStop(void);
extern void DEBUG_MeasPeriod(void);
extern void DEBUG_MeasFps(void);

#define DEBUG_TIMEMEAS_START()               DEBUG_MeasStart()
#define DEBUG_TIMEMEAS_END()                 DEBUG_MeasStop()
#define DEBUG_PERIODMEAS()                   DEBUG_MeasPeriod()
#define DEBUG_MEAS_FPS(_CONDITION_)          do{if(_CONDITION_){DEBUG_MeasFps();}}while(0)

#else  /*!DEBUG_EN*/
#define DEBUG_LOG(...)                       (void)0

#define DEBUG_STACK_PAINT()                  (void)0
#define DEBUG_STACK_ANALYZE()                (void)0

#define DEBUG_TIMEMEAS_START()               (void)0
#define DEBUG_TIMEMEAS_END()                 (void)0
#define DEBUG_PERIODMEAS()                   (void)0
#define DEBUG_MEAS_FPS(_CONDITION_)          (void)0

#endif /*!DEBUG_EN*/
#endif /* INC_DEBUG_H_ */
