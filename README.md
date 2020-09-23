# autonomous-drone

## Table of contents
* [Project overview](#project-overview)
* [Roadmap](#roadmap)
* [Hardware](#hardware)
* [Technologies](#technologies)
* [Data Structures](#data-structures)

## Project Overview

Drone Configuration:
```
          Front
     cw  (1) (2)  ccw      x
           \ /           z ↑
            X             \|
           / \             +----→ y
    ccw  (4) (3)  cw
    
```
| Channel | Command  |
|:-------:|:--------:|
|    1    |   yaw    |
|    2    |   pitch  |
|    3    | throttle |
|    4    |   roll   |


## Roadmap
#### 1.  Non-autonomous Drone
- [x] Define basic data structures and workflow
- [x] Implement *Tx --> Rx* communication using interrups
- [x] Map commands [*Throttle, Roll, Pitch, Yaw*] into motor signals [*m1, m2, m3, m4*] 
- [x] Signal smoothening
- [ ] Implement a safe arm/disarm routine
- [ ] Testing

#### 2.  Hovering
- [ ] Read and store IMU data
- [ ] Implement a real time PID algorithm
- [ ] Implement a simple gyro-based hovering maneuver
- [ ] Testing
- [ ] Implement a real time Kalman Filter Algorithm
- [ ] Implement a full hovering maneuver
- [ ] Testing

#### 3.  Other Autonomous Maneuver
- [ ] Automatic landing and take-off
- [ ] Trajectory tracking


## Hardware
- Frame: S500 glass fiber 480mm
- Motors: Emax 2216 810kv
- Propellers: Emax 1045
- ESC: HAKRC 35A
- MCU: ATmega 328 (Arduino Nano)
- IMU: mpu9250 + barometer (10dof sensor)
- Lipo battery: 3S 5000mAh
- Radio Tx: Flysky FS-i6X
- Radio Rx: Flysky X6B

## Technologies
- Language: C/C++, Matlab
- IDE: Visual Studio Code w\ PlatformIO
- Libs: Arduino.h

## Data Structures
**drone** object:
```
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
} drone;
```
