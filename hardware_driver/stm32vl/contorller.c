/*
 * movement.c
 *
 *  Created on: 20.07.2012
 *      Author: simon
 */

#include "fixmath.h"

static const fix16_t V_MAX = 1000;
static const fix16_t sin_120 = 0xffff8000; // -0.5
static const fix16_t cos_120 = 0xddb4; // 0.866028

void referenceLogic(fix16_t Vx, fix16_t Vy, fix16_t omega) { //Erwartungswerte 0-1000. Omega <=200. Betrag(Vx,Vy) nach Möglichkeit kleiner, gleich 1000

	fix16_t i = fix16_from_float(23.1324);

	i = fix16_sin(i); //TODO in fix16 umschreiben

	fix16_t v0, v1, v2, a0, a1, a2, a;

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

	v0 = (Vx - omega);
	v1 = sin_120 * Vx + cos_120 * Vy - omega;
	v2 = sin_120 * Vx - cos_120 * Vy - omega;

	//TODO v0...2 fix16_to_int
}


void feedback(){

}
