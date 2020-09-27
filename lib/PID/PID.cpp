/**
 * @file pid.cpp
 * @author Giuseppe Sensolini (https://github.com/giusenso)
 * @brief 
 * @version 0.1
 * @date 2020-09-26
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include "PID.h"


/**
 * @brief Initialize PID data structure
 * 
 * @param pid pointer to a pid structure
 * @param setpoint system target setpoint
 * @param Kp proportional gain
 * @param Ki integral gain
 * @param Kd derivative gain
 * @param max maximum pid value
 * @param min minimum pid value
 */
void PID_init(PID* pid, float setpoint, float Kp, float Ki, float Kd){
    pid->setpoint = setpoint;
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->dt = 0;
    pid->err[0] = pid->err[1] = 0;
    pid->output[0] = pid->output[1] = 0;
    pid->min = PID_MIN;
    pid->max = PID_MAX;
}

/**
 * @brief Compute Proportional Integrative Derivative (PID) control law
 * 
 * @param pid pointer to a PID structure
 * @param sample new sensor data sample
 */
void PID_update(PID* pid, float sample){

    // compute error
    pid->err[1] = pid->err[0];
    pid->err[0] = pid->setpoint - sample;

    // compute integral
    pid->integral += pid->err[0]*pid->dt;

    // compute control law
    pid->P = pid->Kp * pid->err[0];
    pid->I = pid->Ki * pid->integral;
    pid->D = pid->Kd * (sample/pid->dt);
    pid->output[1] = pid->output[0];
    pid->output[0] = pid->P + pid->I + pid->D;

    // saturation filter
    if (pid->output[0] > pid->max) pid->output[0] = pid->max;
    else if (pid->output[0] < pid->min) pid->output[0] = pid->min;
}
