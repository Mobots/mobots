/*
 * engine.h
 *
 *  Created on: 13.08.2012
 *      Author: simon
 */

#ifndef ENGINE_H_
#define ENGINE_H_


#define MAX_TRAJECTORY_SIZE 16
#define MAX_ACCELERATION 0.2 // m/s²
/*
 * Der nächste Zielpunkt den der Roboter anfahren soll. Angaben sind in Weltkoordinaten. Ist das Anfahren in gegebener Zeit nicht möglich wird V_max genommen
 */
struct nextPoint {
	int delta_x; // in mm
	int delta_y; // in mm
	int angle; // in deg
	int time_ms;
}__attribute__ ((packed)) __attribute__((__may_alias__));

/*
Eine Trajektory setzt sich aus mehreren Punkten zusammen, die der reihe nach angefahren werden.
*/
struct Trajectory {
	unsigned short size;
	struct nextPoint data[MAX_TRAJECTORY_SIZE];
}__attribute__ ((packed)) __attribute__((__may_alias__));



/*
 * Wandelt Weltkoordinaten in Roboterkoordinaten um
 */
void coord_Transformation(struct nextPoint worldCoord, struct robotCoord *rc);

#endif /* ENGINE_H_ */
