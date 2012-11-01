/*
 * engine.c
 *
 *  Created on: 13.08.2012
 *      Author: simon
 */

#include "engine.h"


fix16_t current_phi = 0; //rotation um die z-Achse im Roboter-Koordinatensystem




void coord_Transformation(struct nextPoint worldCoord, struct robotCoord *rc){

	fix16_t temp_x = fix16_from_int(worldCoord.delta_x);
	fix16_t temp_y = fix16_from_int(worldCoord.delta_y);
	fix16_t temp_ang = fix16_from_int(worldCoord.angle);



	rc->delta_x =
	rc->delta_y = - fix16_mul(temp_x, fix16_sin(current_phi)) + fix16_mul(temp_y, fix16_cos(current_phi));
	rc->angle = temp_ang + current_phi;
	rc->time_ms = worldCoord.time_ms;
}


void Coords_to_Velocity(struct robotCoord rc, fix16_t *Vx, fix16_t *Vy, fix16_t *phi){

	*Vx = rc.delta_x / rc.time_ms;
	*Vy = rc.delta_y / rc.time_ms;
	*phi = rc.angle / rc.time_ms;
}
