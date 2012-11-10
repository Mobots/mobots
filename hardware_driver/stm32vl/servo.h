/*
 * servo.h
 *
 *  Created on: 21.06.2012
 *      Author: simon
 */

#ifndef SERVO_H_
#define SERVO_H_

#include "protocol.h"

void servo_init();
void servo_set(const struct ServoSpeed * const servo_speed);

#endif /* SERVO_H_ */
