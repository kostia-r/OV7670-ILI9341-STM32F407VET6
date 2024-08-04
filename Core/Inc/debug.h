/*
 * debug.h
 *
 *  Created on: Jul 26, 2024
 *      Author: ashen
 */

#ifndef INC_DEBUG_H_
#define INC_DEBUG_H_

#if (DEBUG_LOG_EN == 1)
#include <stdio.h>

/* Redirecting printf() output to SWO ITM */
extern int _write(int file, char *ptr, int len);

#define   DEBUG_LOG_OPT                      1
#if (DEBUG_LOG_OPT == 1)
#define DEBUG_LOG(...)                       do{ printf(__VA_ARGS__);\
		                                     printf("\n"); } while (0)
#else  /* (DEBUG_LOG_OPT == 1) */
#define DEBUG_LOG(...)                       printf(__VA_ARGS__"\n")
#endif /* (DEBUG_LOG_OPT != 1) */
#else  /*!DEBUG_LOG_EN*/
#define DEBUG_LOG(...)                       (void)0
#endif /*!DEBUG_LOG_EN*/

#endif /* INC_DEBUG_H_ */
