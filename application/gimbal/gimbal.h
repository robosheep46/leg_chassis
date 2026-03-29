#ifndef GIMBAL_H
#define GIMBAL_H
#include "robot_def.h"

void GimbalInit(GimbalQueues_t *gimbal_queue);
void GimbalTask(void *argument);



#endif