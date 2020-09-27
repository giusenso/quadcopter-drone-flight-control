/**
 * @file drone.h
 * @author Giuseppe Sensolini (https://github.com/giusenso)
 * @brief Drone essential structure and functions. 
 * @version 0.1
 * @date 2020-09-26
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#pragma once
#ifndef DRONE_H
#define DRONE_H

#include <stdint.h>
#include "PID.h"
#include "MPU9250.h"

#define N 5 // number of stored samples
#define ESC_ARM_SIGNAL 1000
#define ESC_ARM_TIME 7000
#define MIN_PWM_VALUE 1000
#define MAX_PWM_VALUE 1980
#define FLUCTUATION_TOLERANCE 9
#define VARIATION_TOLERANCE 200
#define AV_TOLERANCE 200
#define CONTROLLABILITY_THRESHOLD 1080


/**
 * @brief contains essential paramenters for a quadcopter drone
 * 
 */
typedef struct drone{
    uint8_t state;

    int16_t av_throttle;
    int16_t av_roll;
    int16_t av_pitch;
    int16_t av_yaw;

    int16_t throttle[N];
    int16_t roll[N];
    int16_t pitch[N];
    int16_t yaw[N];
    int16_t ch5;

    float roll_coeff;
    float pitch_coeff;
    float yaw_coeff;

    uint16_t m1;
    uint16_t m2;
    uint16_t m3;
    uint16_t m4;

    IMU* imu;   // imu data for control
    PID* pid_x; // pid control on gyro_x
    PID* pid_y; // pid control on gyro_y
} drone;

enum state{
    disarmed = 0, 
    armed = 1,
    active_control = 2
};

// Function signatures 
uint8_t drone_init(drone* d, float r_coeff, float p_coeff, float y_coeff, float Kp, float Ki, float Kd);
uint8_t compute_motor_signal(drone* d);
inline int16_t signal_average(int16_t* a);
uint8_t update_needed(drone* d, drone* prev_d);
void compute_control_action(drone* d);


#endif