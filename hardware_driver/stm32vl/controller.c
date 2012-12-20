/*
 * controller.c
 *
 *  Created on: 20.07.2012
 *      Author: simon, flo
 */

#include "servo.h"
#include "controller.h"
#include "mousesensor.h"
#include <math.h>

float eSum0, eSum1, eSum2, tA, Ti, kP;

float v_soll_rad0,v_soll_rad1,v_soll_rad2;

volatile struct Velocity sollV = {0, 0, 0};




void control_init(float t_i, float k_p, int delay) {

	Ti = t_i;
	kP = k_p;
	tA = delay / 1000; //convert to seconds //TODO, braucht 2 oder 3 nachkomma stellen
}

//input: soll x,y,theta  Wird verschoben



/*
wird durch den systick zyklisch aufgerufen und bekommt als übergabe wert den ist-wert von den maussensoren
*/
void control(struct MouseData *cur_delta_s)
{
	//calculate current servo speed
	struct Velocity cur_vel;
	cur_vel.x=cur_delta_s->x/tA;
	cur_vel.y=cur_delta_s->y/tA;
	cur_vel.theta=cur_delta_s->theta/tA;
	float v_ist_rad0,v_ist_rad1,v_ist_rad2;
	omniwheelTransformation(&cur_vel,&v_ist_rad0,&v_ist_rad1,&v_ist_rad2);

    //PI Controller

	//berechnen der regeldifferenzen
	float e0 = v_soll_rad0-v_ist_rad0;
	float e1 = v_soll_rad1-v_ist_rad1;
	float e2 = v_soll_rad2-v_ist_rad2;

	eSum0+=e0;
	eSum1+=e1;
	eSum2+=e2;

	//Stellwert generieren
	struct ServoSpeed servo_speed;
	servo_speed.v[0] = (kP * e0 + Ti * tA * eSum0)/V_MAX;
	servo_speed.v[1] = (kP * e1 + Ti * tA * eSum1)/V_MAX;
	servo_speed.v[2] = (kP * e2 + Ti * tA * eSum2)/V_MAX;

	servo_set(&servo_speed);

}


/*
brechnet aus den übergebenen x-, y-, theta-werten die geschwindigkeit für jeden mobot
*/
/* Die Funktion rechnet die gewünschte velocity im Koordinatensystem des Mobots auf Gechwindigkeiten für die servos um.
 *
 * Wertebereich für die Eingabe:
 * velocity.x und velocity.y: [-1, 1]
 * velocity.theta = ?
 */


static const float ALPHA_RAD = - 0.245; // in grad: 14.04
#define r_rad  0.1027	//m
#define v_max 0.15		//m/s

//zentraler Bewegungsvektor ==> drei Radbewegungen
void omniwheelTransformation(struct Velocity * const velocity_mobot, float *v0,float *v1,float *v2)
{

	transformAchse(&velocity_mobot->x,&velocity_mobot->y,-ALPHA_RAD);

	*v0 = (float)  (cos(ALPHA_RAD)          * velocity_mobot->x  - sin(ALPHA_RAD)          * velocity_mobot->y  - cos(ALPHA_RAD) * velocity_mobot->theta * r_rad);
	*v1 = (float) (- cos(M_PI/3 + ALPHA_RAD) * velocity_mobot->x  + sin(M_PI/3 + ALPHA_RAD) * velocity_mobot->y  - cos(ALPHA_RAD) * velocity_mobot->theta * r_rad);
	*v2 = (float) (- cos(M_PI/3 - ALPHA_RAD) * velocity_mobot->x  - sin(M_PI/3 - ALPHA_RAD) * velocity_mobot->y  - cos(ALPHA_RAD) * velocity_mobot->theta * r_rad);
	return;
}


void omniwheelTransformationReverse(float *v0,float *v1,float *v2,struct Velocity * const velocity_mobot)
{


	velocity_mobot->x		=(0.67* 	(1.681		*(*v0)	-0.4664*(*v1)	-1.203 *(*v2)));
	velocity_mobot->y		=(0.7*		(-0.419		*(*v0)	+0.9703*(*v1)	-0.5513*(*v2)));
	velocity_mobot->theta	=(0.7204*	(-0.866025	*(*v0)	-0.3663*(*v1)	-0.1983*(*v2))/r_rad);

	transformAchse(&velocity_mobot->x,&velocity_mobot->y,ALPHA_RAD);
	return;
}

void omniwheelTransformationAlt(struct Velocity * const velocity, struct ServoSpeed * const servo_speed)
{
	const int R = 1;

	//transformAchse(&velocity->x,&velocity->y,-ALPHA_RAD);	//hier um negativen winkel drehen
	/* Die Eingabewerte werden mit diesem Faktor herunterskaliert, um die maximal möglichen "Verluste" der Omniwheelanordnung zu berücksichtigen. */
	//static const float scaler = 0.5 + M_SQRT3/2;

	const float scaler = cos(M_PI/3 - ALPHA_RAD) + sin(M_PI/3 - ALPHA_RAD);

	velocity->x /= scaler;
	velocity->y /= scaler;

	/*
	servo_speed->v[0] =        velocity->x                           - velocity->theta * R;
	servo_speed->v[1] = -0.5 * velocity->x + M_SQRT3/2 * velocity->y - velocity->theta * R;
	servo_speed->v[2] = -0.5 * velocity->x - M_SQRT3/2 * velocity->y - velocity->theta * R;
	*/

	servo_speed->v[0] =   cos(ALPHA_RAD)          * velocity->x  - sin(ALPHA_RAD)          * velocity->y  - cos(ALPHA_RAD) * velocity->theta * R;
	servo_speed->v[1] = - cos(M_PI/3 + ALPHA_RAD) * velocity->x  + sin(M_PI/3 + ALPHA_RAD) * velocity->y  - cos(ALPHA_RAD) * velocity->theta * R;
	servo_speed->v[2] = - cos(M_PI/3 - ALPHA_RAD) * velocity->x  - sin(M_PI/3 - ALPHA_RAD) * velocity->y  - cos(ALPHA_RAD) * velocity->theta * R;


	return;
}

/*
schiebt den mobot an indem sollwerte als struct übergeben werden. diese können von einer engine oder dem client kommen.
*/
void setSollV(struct Velocity *soll) {

	//maxima_runterskalieren(soll);
	sollV.x	=soll->x;
	sollV.y	=soll->y;
	sollV.theta	=soll->theta;
}

//additiv unmögliche geschwindigkeiten runterskalieren:
void maxima_runterskalieren(struct Velocity *max)
{
	float v0, v1, v2, a0, a1, a2, a;
	omniwheelTransformation(max,&v0,&v1,&v2);

	if (v0 > V_MAX) 		a0 = (V_MAX + (cos(ALPHA_RAD)* r_rad *max->theta)) / (v0);
	if (v0 < (-V_MAX)) 		a0 = (V_MAX - (cos(ALPHA_RAD)* r_rad *max->theta)) / (-v0);


	if (v1 > V_MAX)		 a1 = (V_MAX + (cos(ALPHA_RAD)* r_rad *max->theta)) / (v1);
	if (v1 < (-V_MAX)) 	 a1 = (V_MAX - (cos(ALPHA_RAD)* r_rad *max->theta)) / (-v1);


	if (v2 > V_MAX) 		a2 = (V_MAX + (cos(ALPHA_RAD)* r_rad *max->theta)) / (v2);
	if (v2 < (-V_MAX)) 		a2 = (V_MAX - (cos(ALPHA_RAD)* r_rad *max->theta)) / (-v2);


	a = 1; //Minimum Prüfung des Skalierungswert
	a = (a0 < a) ? a0 : a;
	a = (a1 < a) ? a1 : a;
	a = (a2 < a) ? a2 : a;

	if (a<1) { 		//scale
		v0 =  a*(		cos(ALPHA_RAD)        * max->x  	- sin(ALPHA_RAD)          * max->y	 	)	 - cos(ALPHA_RAD) * max->theta * r_rad;
		v1 =  a*(	- cos(M_PI/3 + ALPHA_RAD) * max->x 		+ sin(M_PI/3 + ALPHA_RAD) * max->y 		)	 - cos(ALPHA_RAD) * max->theta * r_rad;
		v2 =  a*(	- cos(M_PI/3 - ALPHA_RAD) * max->x  	- sin(M_PI/3 - ALPHA_RAD) * max->y 		)  	 - cos(ALPHA_RAD) * max->theta * r_rad;

		// inverse Omniwheel-transformation
		omniwheelTransformationReverse(&v0,&v1,&v2,max);
	}
}
