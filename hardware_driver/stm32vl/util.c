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


#include "util.h"
/* simple delay - idle-loop */
void my_delay(unsigned long delay ){
    volatile unsigned long tmp=delay;
    while(tmp){
        tmp--;
    }
}
/* delay for "ms" using idle loop above */
void delay_ms(unsigned long ms) {
    unsigned long ticks=ms*2000; //For the STM32 running at 24MHz
    my_delay(ticks);

}
/* delay for "mikroseconds" us */
void delay_us(unsigned long us) {
    unsigned long ticks=us*3; //For the STM32 running at 24MHz
    my_delay(ticks);
}

/* CRC32-Sum-function from linux kernel source */
uint32_t crc32(uint32_t crc, uint8_t *p, uint32_t len)
{
	int i;
	while (len--) {
		crc ^= *p++;
		for (i = 0; i < 8; i++)
			crc = (crc >> 1) ^ ((crc & 1) ? CRCPOLY_LE : 0);
	}
	return crc;
}

