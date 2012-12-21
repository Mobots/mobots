/*
 * protocol_handler.c
 *
 *  Created on: 24.09.2012
 *      Author: simon
 */
#include "protocol.h"
#include "printf.h"
#include "mousesensor.h"
#include "servo.h"
#include "controller.h"


void protocol_defaultHandler(enum PROTOCOL_IDS id, unsigned char *data, unsigned short size) {
	unsigned short idl = id;
	printf("Warning, packet with id %hu was not handled \n", idl);
}

void setVelocityHandler(enum PROTOCOL_IDS id, unsigned char *data, unsigned short size) {
	if (id != VELOCITY) {
		print("Error, wrong ID\n");
		return;
	}

	if (size != sizeof(struct Velocity)) {
		print("Error, wrong size\n");
		return;
	}

	struct Velocity *velocity = (struct Velocity*) data;


	struct ServoSpeed speed;
	omniwheelTransformationAlt(velocity, &speed);
	servo_set(&speed);

	//neuen Sollwert setzen in controller.c:
	//setSollV(velocity);
}

void setServoHandler(enum PROTOCOL_IDS id, unsigned char *data, 	unsigned short size) {
	if (id != SERVO_SPEED) {
		print("Error, wrong ID\n");
		return;
	}

	if (size != sizeof(struct ServoSpeed)) {
		print("Error, wrong size\n");
		return;
	}

	struct ServoSpeed *servo_speed = (struct ServoSpeed*) data;

	servo_set(servo_speed);
}

void protocol_handler_init() {
	protocol_registerHandler(VELOCITY, setVelocityHandler);
	protocol_registerHandler(SERVO_SPEED, setServoHandler);
}

