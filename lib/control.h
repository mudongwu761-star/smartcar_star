#ifndef CONTROL_H
#define CONTROL_H

#include <unistd.h>
#include <chrono>
#include <iostream>

#include "MotorController.h"
#include "PIDController.h"
#include "global.h"
#include "serial.h"
#include "GPIO.h"
#include "camera.h"
#include "encoder.h"

#define SERVO_MID_DUTY  1500000

extern MotorController* motorController[2]; // 声明全局变量
static PIDController SpeedPID(0, 0, 0, 0, INCREMENTAL);  // 速度环（增量式）
extern int target;
extern double last_duty;
extern int target_stop_speed;

void ControlInit();
void ControlMain();
void ControlExit();
void DetermineBarrierPosition();
void AvoidObstacle();
void motorcontrol();
double Turn_Out_Filter(float turn_out);



#endif