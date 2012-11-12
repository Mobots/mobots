/*
 * controller.c
 *
 *  Created on: 20.07.2012
 *      Author: simon, flo
 */

#include "servo.h"
#include "controller.h"
#include <math.h>

#if 0
float eSum1, eSum2, eSum3, tA, kI, kP;

volatile struct ServoSpeed sollV = {0,0,0};



void init(int delay) {

	kI = 0.01;
	kP = 0.01;
	tA = delay / 1000; //convert to seconds //TODO, braucht 2 oder 3 nachkomma stellen
}

//input: soll x,y,theta  Wird verschoben


/*
wird durch den systick zyklisch aufgerufen und bekommt als übergabe wert den ist-wert von den maussensoren
*/
void control(struct ServoSpeed *vIst) {


//PI Controller
	float e1 = vIst->s1 - sollV.s1;
	int s1 = kP * e1 + kI * tA * eSum1;

	float e2 = vIst->s2 - sollV.s2;
	int s2 = kP * e2 + kI * tA * eSum2;

	float e3 = vIst->s3 - sollV.s3;
	int s3 = kP * e3 + kI * tA * eSum3;

	servo_setAngle(Servo_1, s1);
	servo_setAngle(Servo_2, s2);
	servo_setAngle(Servo_3, s3);
}

/*
brechnet aus den übergebenen x-, y-, theta-werten die geschwindigkeit für jeden mobot
*/

#endif

/* Die Funktion rechnet die gewünschte velocity im Koordinatensystem des Mobots auf Gechwindigkeiten für die servos um.
 *
 * Wertebereich für die Eingabe:
 * velocity.x und velocity.y: [-1, 1]
 * velocity.theta = ?
 */
void omniwheelTransformation(struct Velocity * const velocity, struct ServoSpeed * const servo_speed)
{
	static const float R = 1; // in m
	static const float ALPHA = - 14.04 * M_PI / 180;

	/* Die Eingabewerte werden mit diesem Faktor herunterskaliert, um die maximal möglichen "Verluste" der Omniwheelanordnung zu berücksichtigen. */
	//static const float scaler = 0.5 + M_SQRT3/2;
	const float scaler = cos(M_PI/3 - ALPHA) + sin(M_PI/3 - ALPHA);

	velocity->x /= scaler;
	velocity->y /= scaler;

	/*
	servo_speed->v[0] =        velocity->x                           - velocity->theta * R;
	servo_speed->v[1] = -0.5 * velocity->x + M_SQRT3/2 * velocity->y - velocity->theta * R;
	servo_speed->v[2] = -0.5 * velocity->x - M_SQRT3/2 * velocity->y - velocity->theta * R;
	*/

	servo_speed->v[0] =   cos(ALPHA)          * velocity->x  - sin(ALPHA)          * velocity->y  - cos(ALPHA) * velocity->theta * R;
	servo_speed->v[1] = - cos(M_PI/3 + ALPHA) * velocity->x  + sin(M_PI/3 + ALPHA) * velocity->y  - cos(ALPHA) * velocity->theta * R;
	servo_speed->v[2] = - cos(M_PI/3 - ALPHA) * velocity->x  - sin(M_PI/3 - ALPHA) * velocity->y  - cos(ALPHA) * velocity->theta * R;


	return;

#if 0
	float v0, v1, v2, a0, a1, a2, a;

	if (v0 > V_MAX) {
		a0 = (V_MAX + velocity->theta) / (velocity->x);
	}

	if (v0 < (-V_MAX)) {
		a0 = (-V_MAX + velocity->theta) / (velocity->x);
	}
	if (v1 > V_MAX) {
		a1 = (V_MAX + 100 * velocity->theta) / (-50 * velocity->x + 86 * velocity->y);
	}

	if (v1 < (-V_MAX)) {
		a1 = (-V_MAX + velocity->theta) / (sin_120 * velocity->x + cos_120 * velocity->y);
	}
	if (v2 > V_MAX) {
		a2 = (V_MAX + velocity->theta) / (sin_120 * velocity->x - cos_120 * velocity->y);
	}

	if (v2 < (-V_MAX)) {
		a2 = (-V_MAX + velocity->theta) / (sin_120 * velocity->x - cos_120 * velocity->y);
	}

	a = 100; //Minimum Prüfung des Skalierungswert
	a = (a0 < a) ? a0 : a;
	a = (a1 < a) ? a0 : a;
	a = (a2 < a) ? a2 : a;

	Vx = Vx * a / 100;
	velocity->y = velocity->y * a / 100;

	sollV.s1 = Vx - velocity->theta;
	sollV.s2 = sin_120 * Vx + cos_120 * velocity->y - velocity->theta;
	sollV.s3 = sin_120 * Vx - cos_120 * velocity->y - velocity->theta;
#endif
}

#if 0
/*
schiebt den mobot an indem sollwerte als struct übergeben werden. diese können von einer engine oder dem client kommen.
*/
void setSollV(struct ServoSpeed *soll) {
	//calculate the single servo speed
	sollV = *soll;
	referenceLogic(sollV.s1, sollV.s2, sollV.s3);
}
#endif
