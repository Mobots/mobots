#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include "servo.h"

#define  V_MAX  1000  // als define lösen!!
#define sin_120  -0.5
#define cos_120  0.866028


void setSollV(struct ServoSpeed *sollV);
void control_init(int delay);
void control(struct ServoSpeed *vIst);

void omniwheelTransformation(const struct Velocity * const velocity, struct ServoSpeed * const servo_speed);

#endif
