#ifndef __PRINTF_H
#define __PRINTF_H

#ifdef SANDBOX

#include <stdio.h>
int print(const char *format);
#else

#undef printf

int print(const char *format);
int printf(const char *format, ...);

void testprintf();

#endif

#endif
