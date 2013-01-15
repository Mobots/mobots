#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include "servo.h"

#define  V_MAX  1000  // als define lösen!!
#define sin_120  -0.5
#define cos_120  0.866028


void control_init(float t_i, float k_p, int delay);
void control(struct MouseData *cur_delta_s);

void omniwheelTransformation(struct Velocity * const velocity_mobot, float *rad0,float *rad1,float *rad2);
void omniwheelTransformationReverse(float *v0,float *v1,float *v2,struct Velocity * const velocity_mobot);
void omniwheelTransformationAlt(struct Velocity * const velocity, struct ServoSpeed * const servo_speed);

void setSollV(struct Velocity *soll);
void maxima_runterskalieren(struct Velocity *max);

#endif
