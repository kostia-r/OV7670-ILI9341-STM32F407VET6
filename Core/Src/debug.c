/*
 * debug.c
 *
 *  Created on: Jul 26, 2024
 *      Author: ashen
 */

#include "debug.h"
#include "stm32f4xx_hal.h"
#include "core_cm4.h"

#ifdef    DEBUG_LOG_EN
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
#endif /* DEBUG_LOG_EN */

