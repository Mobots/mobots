/*
 * FingerTip Sensor
 * Copyright (C) 2010 Alexis Maldonado <maldonad@cs.tum.edu>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __UTIL_H
#define __UTIL_H

#include "misc.h"


//CRC-32-Polygon
#define CRCPOLY_LE 0x4C11DB7


void my_delay(unsigned long delay);
void delay_ms(unsigned long ms);
void delay_us(unsigned long us);

/* CRC32-Sum-function from linux kernel source */
uint32_t crc32(uint32_t crc, uint8_t *p, uint32_t len);

#endif /* __UTIL_H */
