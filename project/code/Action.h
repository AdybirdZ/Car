#ifndef _ACTION_H_
#define _ACTION_H_

#include "Angle_PID.h"
#include "Gray_Line.h"
#include "Position.h"

#define ACTION_TURN_RIGHT_ANGLE         (90.0f)
#define ACTION_TURN_LEFT_ANGLE          (-90.0f)
#define ACTION_TURN_ANGLE_TOLERANCE     (5.0f)
#define ACTION_TURN_STABLE_COUNT        (5)
#define ACTION_TURN_TIMEOUT_MS          (3000)

uint8 Action_Turn (float angle);
uint8 Action_Turn_To (float target);
uint8 Action_Turn_Right ();
uint8 Action_Turn_Left ();
void Turn (float angle);

#endif
