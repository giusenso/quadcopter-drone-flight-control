/**
 * @file pid.h
 * @author Giuseppe Sensolini (https://github.com/giusenso)
 * @brief PID control
 * @version 0.1
 * @date 2020-09-26
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#pragma once
#ifndef PID_H
#define PID_H

#include <stdint.h>
#include "Arduino.h"
#include "MPU9250.h"

// saturation thresholds
#define PID_MIN -999  
#define PID_MAX 999
#define X_SETPOINT 500
#define Y_SETPOINT 500

/**
 * @brief PID controller
 * 
 */
typedef struct PID{
    float setpoint;
    float Kp, Ki, Kd;
    float dt;
    float err[2];
    float output[2];
    float P, I, D;
    float integral;
    float min, max;
}PID;


// Function signatures
void PID_init(PID* pid, float setpoint, float Kp, float Ki, float Kd);
void PID_update(PID* pid, float sample);

#endif