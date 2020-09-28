# autonomous-drone-flight-control

## Table of contents
* [Project overview](#project-overview)
* [Roadmap](#roadmap)
* [Hardware](#hardware)
* [Electronic Schematics](#electronic-schematics)
* [Software](#software)

## Project Overview

... 

**Drone Configuration**
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
|    5    |   vra    |


## Roadmap
#### 1.  Non-autonomous Drone
- [x] Define basic data structures and workflow
- [x] pcb design
- [x] Implement *Tx --> Rx* communication using interrups
- [x] Map commands <*Throttle, Roll, Pitch, Yaw*> into motor signals <*m1, m2, m3, m4*> 
- [x] Signal smoothening
- [ ] Implement a safe arm/disarm routine
- [ ] Testing

#### 2.  Hovering
- [ ] Implement an IMU class for MPU9250
- [x] Implement a PID class and algorithm
- [ ] Implement a gyro-based hovering maneuver
- [ ] Testing
- [ ] Implement a real time Kalman Filter Algorithm
- [ ] Implement a full hovering maneuver
- [ ] Testing

#### 3.  Other Autonomous Maneuver
- [ ] Autonomous take-off and landing


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


## Electronic schematics

**Pin Mapping**
```
                       ATmega328
                    +--------------+           
                    | D13      D12 |<-------------|Ch5| (Radio Rx)
                    | 3.3v     D11 |----> (M1)
                    | Ref      D10 |----> (M2)
                    | A0        D9 |----> (M3)
                    | A1        D8 |
                    | A2        D7 |<-------------|Ch1| (Radio Rx)
                    | A3        D6 |<-------------|Ch2| (Radio Rx)
(MPU9250) |SDA|---->| A4        D5 |<-------------|Ch3| (Radio Rx)
(MPU9250) |SCL|---->| A5        D4 |<-------------|Ch4| (Radio Rx)
                    | A6        D3 |----> (M4)
                    | A7        D2 |
                    | 5v       GND |
                    | RST      RST |
           GND -----| GND       Rx |
          3.3v -----| VIN       Tx |
                    +--------------+
```
| Pin |   Port   | Data |
|:---:|:--------:|:-------:|
| A4  |   PC4    | SDA |
| A5  |   PC5    | SCL |
| D4  |   PD4    | Ch4 |
| D5  |   PD5    | Ch3 |
| D6  |   PD6    | Ch2 |
| D7  |   PD7    | Ch1 |
| D9  |   PB1    | M3 |
| D10 |   PB2    | M2 |
| D11 |   PB3    | M1 |
| D12 |   PB4    | Ch5 |

                 
**Power Busses**
```
                                 +---------+        +-----------+
                                 | MPU9250 |        | Brushless |
                                 +---+--+--+        |  Motors   |
                                     |  |           +---+--+----+
+======+                             |  |               |  |
|      |-------------- GND ----------+--|----+----------+  |
| LIPO |----(buck)---- 3.3v ------------+    |             |
|  3S  |----(buck)---- 7.3v -----------------|--+          |
|      |------------ 11.1/12.6v -------------|--|----------+
+======+                                     |  |  
                                             |  |   
                                         +---+--+---+ 
                                         | Radio Rx | 
                                         +----------+     
```


## Software
- Language: C/C++, Matlab
- IDE: Visual Studio Code w\ PlatformIO
- Additional libs: Arduino.h, Servo.h


**Repository structure**
```
.
├── include
│   ├── atmega328_pin_mapping.h
│   └── README
├── lib
│   ├── drone
│   │   ├── drone.cpp
│   │   └── drone.h
│   ├── MPU9250
│   │   ├── MPU9250.cpp
│   │   └── MPU9250.h
│   ├── PID
│   │   ├── PID.cpp
│   │   └── PID.h
│   └── README
├── platformio.ini
├── README.md
├── src
│   └── main.cpp
└── test
    └── README
```

**Data Structures**

*Drone*:
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

    IMU* imu;   // imu data for control
    PID* pid_x; // pid control on gyro_x
    PID* pid_y; // pid control on gyro_y
} drone;

```

*PID controller:*
```
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

```
