/*
 * engine.c
 *
 *  Created on: 13.08.2012
 *      Author: simon
 */

#include "engine.h"

#define r_aussen 0.125
#define r_innen  0.10
#define v_max 0.15
#define sqrt3 1,73205081
#define omega -r_innen/r_aussen*y1/sqrt3-r_innen/r_aussen*2/3*x2-r_innen/r_aussen*x1/3

/*
////gibt die pixel als strecke in meter im mobot_koordinatensystem aus
void transformMouseToCoordinateSystem(struct Mouse_Data_DeltaVal* data,
		struct Mouse_Data_DeltaValOut* dataOut) {

	float x1 = -data->delta_y1;
	float y1 = -data->delta_x1;
	float x2 = -data->delta_y2;
	//transform
	dataOut->delta_x = ((x1 - x2) / 3 - y1 * 1.1547) * 5040 / 0.0254; //TODO correct dpi insert
	dataOut->delta_y = (x1 - x2) / 3 * 5040 / 0.0254;
	dataOut->delta_theta = (y1 * 0.5774 + 0.6667 * x2 + x1 * 0.3333) / r_aussen
			* 5040 / 0.0254; //TODO eventuell nicht bogenmass

}
*/

/*void transformToServoSpeed(struct Mouse_Data_DeltaVal* data,
		struct ServoSpeed* sOut, float totzeit) {

	float x1 = -data->delta_y1;
	float y1 = -data->delta_x1;
	float x2 = -data->delta_y2;



	sOut->s1 = (short) ((x1 / 3 - x2 / 3 - 2 / sqrt3 * y1 + omega) / v_max
			* 1000 / totzeit);
	sOut->s2 = (x1 / sqrt3 - x2 / 3 + y1 / sqrt3 + omega) / v_max * 1000
			/ totzeit;
	sOut->s3 = (-y1 / sqrt3 - 2 / 3 * x1 + 2 / 3 * x2 + omega) / v_max * 1000
			/ totzeit; //TODO, nicht sicher, ob richtig abgeschrieben

}*/
