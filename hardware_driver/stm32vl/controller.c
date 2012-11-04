/*
 * movement.c
 *
 *  Created on: 20.07.2012
 *      Author: simon, flo
 */

#include "fixmath.h"
#include "servo.h"
#include "controller.h"

static const double V_MAX = 1000;
static const double sin_120 = -0.5;
static const double cos_120 = 0.866028;
double eSum1, eSum2, eSum3, tA, kI, kP;

volatile struct ServoSpeed sollV = {0,0,0};



void init(int delay) {

	kI = 0.01;
	kP = 0.01;
	tA = delay / 1000; //convert to seconds //TODO, braucht 2 oder 3 nachkomma stellen
}

//input: soll x,y,theta  Wird verschoben


void control(struct ServoSpeed *vIst) {


//PI Controller
	double e1 = vIst->s1 - sollV.s1;
	int s1 = kP * e1 + kI * tA * eSum1;

	double e2 = vIst->s2 - sollV.s2;
	int s2 = kP * e2 + kI * tA * eSum2;

	double e3 = vIst->s3 - sollV.s3;
	int s3 = kP * e3 + kI * tA * eSum3;

	servo_setAngle(Servo_1, s1);
	servo_setAngle(Servo_2, s2);
	servo_setAngle(Servo_3, s3);
}

void referenceLogic(double Vx, double Vy, double omega) { //Erwartungswerte 0-1000. Omega <=200. Betrag(Vx,Vy) nach Möglichkeit kleiner, gleich 1000

	double v0, v1, v2, a0, a1, a2, a;

	v0 = (Vx - omega);
	v1 = sin_120 * Vx + cos_120 * Vy - omega;
	v2 = sin_120 * Vx - cos_120 * Vy - omega;

	if (v0 > V_MAX) {
		a0 = (V_MAX + omega) / (Vx);
	}

	if (v0 < (-V_MAX)) {
		a0 = (-V_MAX + omega) / (Vx);
	}
	if (v1 > V_MAX) {
		a1 = (V_MAX + 100 * omega) / (-50 * Vx + 86 * Vy);
	}

	if (v1 < (-V_MAX)) {
		a1 = (-V_MAX + omega) / (sin_120 * Vx + cos_120 * Vy);
	}
	if (v2 > V_MAX) {
		a2 = (V_MAX + omega) / (sin_120 * Vx - cos_120 * Vy);
	}

	if (v2 < (-V_MAX)) {
		a2 = (-V_MAX + omega) / (sin_120 * Vx - cos_120 * Vy);
	}

	a = 100; //Minimum Prüfung des Skalierungswert
	a = (a0 < a) ? a0 : a;
	a = (a1 < a) ? a0 : a;
	a = (a2 < a) ? a2 : a;

	Vx = Vx * a / 100;
	Vy = Vy * a / 100;

	sollV.s1 = Vx - omega;
	sollV.s2 = sin_120 * Vx + cos_120 * Vy - omega;
	sollV.s3 = sin_120 * Vx - cos_120 * Vy - omega;

}

void setSollV(struct ServoSpeed *soll) {
	//calculate the single servo speed
	sollV = *soll;
	referenceLogic(sollV.s1, sollV.s2, sollV.s3);
}

void feedback() {

}
