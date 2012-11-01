/*
 * servo.h
 *
 *  Created on: 21.06.2012
 *      Author: simon
 */

#ifndef SERVO_H_
#define SERVO_H_


struct ServoSpeed{
	short s1;
	short s2;
	short s3;
}__attribute__((packed)) __attribute__((__may_alias__));

enum Servos{

	Servo_1 = 0,
	Servo_2,
	Servo_3
};

void servo_init();
void servo_setAngle(enum Servos servo, int value);

#endif /* SERVO_H_ */
