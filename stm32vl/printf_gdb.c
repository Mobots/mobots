/*
 * printf_gdb.c
 *
 *  Created on: 10.07.2012
 *      Author: simon
 */
#include "core_cm3.h"

int printf_gdb(const unsigned char *format) {
  const unsigned char *fmt = format;
  unsigned int len = 0;

  while(*fmt) {
    fmt++;
    len++;
  }

  int sent = 0;

  while(sent < len)
  {
	ITM_SendChar(format[sent]);
  }
  return len;
}
