/*
 * servo.h
 *
 *  Created on: 21.06.2012
 *      Author: simon
 */

#ifndef SERVO_H_
#define SERVO_H_

struct Velocity {
	float x;
	float y;
	float theta;
}__attribute__((packed)) __attribute__((__may_alias__));

struct ServoSpeed {
	float v[3]; // v0, v1, v2
}__attribute__((packed)) __attribute__((__may_alias__));

enum Servos{
	Servo_1 = 0,
	Servo_2,
	Servo_3
};

void servo_init();
void servo_setAngle(enum Servos servo, int value);
void set(const struct ServoSpeed * const servo_speed);

#endif /* SERVO_H_ */
