#ifndef _ACTION_H_
#define _ACTION_H_

#include "Angle_PID.h"
#include "Gray_Line.h"
#include "Position.h"

#define ACTION_TURN_RIGHT_ANGLE         (90.0f)
#define ACTION_TURN_LEFT_ANGLE          (-90.0f)
#define ACTION_TURN_ANGLE_TOLERANCE     (2.5f)
#define ACTION_TURN_STABLE_COUNT        (5)
#define ACTION_TURN_TIMEOUT_MS          (3000)
#define ACTION_TURN_CLOCKWISE            (1)
#define ACTION_TURN_COUNTERCLOCKWISE     (-1)
#define ACTION_TURN_DIRECTION_STEP       (90.0f)

extern uint16 straight_count;

uint8 Action_Turn (float angle);
uint8 Action_Turn_To (float target);
uint8 Action_Turn_Direction (float angle, int8 direction);
uint8 Action_Turn_Right ();
uint8 Action_Turn_Left ();
void Action_Drive_Equal_Target (float target, uint32 duration_ms);
void Turn (float angle);
void Straight_Forward (float time);
void Straight_Backward (float time);

#endif
