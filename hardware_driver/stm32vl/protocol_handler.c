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

void protocol_defaultHandler(enum PROTOCOL_IDS id, unsigned char *data,
		unsigned short size) {
	unsigned short idl = id;
	printf("Warning, packet with id %hu was not handled \n", idl);
}

/*
 * folgende Funktion wird aufgerufen wenn es sich um einen neu Punkt
 * ,welcher angefahren werden soll, handelt
 */
void setPointHandler(enum PROTOCOL_IDS id, unsigned char *data,
		unsigned short size) {

}

void setVelocityHandler(enum PROTOCOL_IDS id, unsigned char *data, unsigned short size) {
	if (id != Servo) {
		print("Error, wrong ID\n");
		return;
	}

	if (size != sizeof(struct Velocity)) {
		print("Error, wrong size\n");
		return;
	}

	struct Velocity *velocity = (struct Velocity*) data;

	struct ServoSpeed servo_speed;
	omniwheelTransformation(velocity, &servo_speed);
	set(&servo_speed);
}

void setServoHandler(enum PROTOCOL_IDS id, unsigned char *data,
		unsigned short size) {

	if (id != Servo) {
		print("Error, wrong ID\n");
		return;
	}

	if (size != sizeof(struct ServoSpeed)) {
		print("Error, wrong size\n");
		return;
	}

	struct ServoSpeed *servo_speed = (struct ServoSpeed*) data;

	set(servo_speed);
}

/*
 * folgende Funktion wird aufgerufen wenn es sich um eine neue Trajektory
 * ,welcher abgefahren werden soll, handelt
 */
void setTrajektoryHandler(enum PROTOCOL_IDS id, unsigned char *data,
		unsigned short size) {

}

// Wird eine Anfrage gestellt wird sie in folgender Funktion abgearbeitet
void setRequestHandler(enum PROTOCOL_IDS id, unsigned char *data,
		unsigned short size) {
	//print("Receive a request\n");

	if (id != REQUEST) {
		print("Error, wrong ID\n");
		return;
	}

	if (size != sizeof(struct Request)) {
		print("Error, wrong size\n");
		return;
	}

	struct Request *req = (struct Request*) data;

	switch (req->req_typ) {
	case MouseData_All:
		protocol_sendData(MouseData_All, (unsigned char*) &mouse_data,
				sizeof(mouse_data));
		break;
	case MouseData_DeltaVal:
		//print("MouseData_DeltaVal");
		protocol_sendData(SensorData_DeltaVal, (unsigned char*) &delta_vals,
				sizeof(struct Mouse_Data_DeltaVal));
		break;
	default:
		break;
	}

}

//Wird eine Warnung gesendet wird sie in folgender Funktion abgearbeitet
void setWarningHandler(enum PROTOCOL_IDS id, unsigned char *data,
		unsigned short size) {

}

void protocol_handler_init() {
	protocol_registerHandler(SET_POINT, setPointHandler);
	protocol_registerHandler(SET_TRAJEKTORY, setTrajektoryHandler);
	protocol_registerHandler(REQUEST, setRequestHandler);
	protocol_registerHandler(WARNING, setWarningHandler);
	protocol_registerHandler(Servo, setServoHandler);
	protocol_registerHandler(VELOCITY, setVelocityHandler);
}

