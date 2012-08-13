/*
 * movement.c
 *
 *  Created on: 20.07.2012
 *      Author: simon
 */

#include "fixmath.h"




#define V_MAX 100000 // 1000*100


void sollWertLogik(int Vx, int Vy, int omega) { //Erwartungswerte 0-1000. Omega <=200. Betrag(Vx,Vy) nach Möglichkeit kleiner, gleich 1000

	fix16_t i = fix16_from_float(23.1324);

	i = fix16_sin(i); //TODO in fix16 umschreiben


	int v0, v1, v2, a0, a1, a2, a;

	v0 = (Vx - omega) * 100;
	v1 = -50 * Vx + 86 * Vy - omega * 100;
	v2 = -50 * Vx - 86 * Vy - omega * 100;

	if (v0 > V_MAX) {
		a0 = (V_MAX + 100*omega)/(100*Vx);
	}

	if (v0 < (- V_MAX)) {
		a0 = (-V_MAX + 100*omega)/(100*Vx);
	}
	if (v1 > V_MAX) {
		a1 = (V_MAX + 100*omega)/(-50*Vx + 86 *Vy);
	}

	if (v1 < (- V_MAX)) {
		a1 = (-V_MAX + 100*omega)/(-50*Vx + 86 *Vy);
	}
	if (v2 > V_MAX) {
		a2 = (V_MAX + 100*omega)/(-50*Vx - 86 *Vy);
	}

	if (v2 < (- V_MAX)) {
		a2 = (-V_MAX + 100*omega)/(-50*Vx - 86 *Vy);
	}

	a=100;					//Minimum Prüfung des Skalierungswert
	a = (a0<a) ? a0 : a;
	a = (a1<a) ? a0 : a;
	a = (a2<a) ? a2 : a;

	Vx=Vx*a/100;
	Vy=Vy*a/100;

	v0 = (Vx - omega) * 100;
	v1 = -50 * Vx + 86 * Vy - omega * 100;
	v2 = -50 * Vx - 86 * Vy - omega * 100;



}
