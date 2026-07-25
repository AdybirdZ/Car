#ifndef _POSITION_H_
#define _POSITION_H_

#include "zf_common_headfile.h"

#define ROLL                        (0)
#define PITCH                       (1)
#define YAW                         (2)

extern float euler_angle[3];
extern bool enable_position;
extern bool position_init_ok;

void Position_Init ();
void Position_Update ();

#endif
