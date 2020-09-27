/**
 * @file main.cpp
 * @author Giuseppe Sensolini (https://github.com/giusenso)
 * @brief Quadcopter drone flight controller design.
 * @repository: https://github.com/giusenso/autonomous-drone-flight-control
 * @version 0.2
 * @date 2020-09-26
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include <stdio.h>
#include <stdint.h>

#include <Arduino.h>
#include <Servo.h>

#include "drone.h"
#include "atmega328_pin_mapping.h"

// Drone flight coefficients
#define ROLL_COEFF  0.3
#define PITCH_COEFF 0.3
#define YAW_COEFF   0.3

// DATA STRUCTURES
drone d;
MPU9250 mpu9250(Wire,0x68);
Servo ESC[4];
uint64_t timer[NUM_CHANNEL];
uint8_t last_channel[NUM_CHANNEL];
uint8_t update = 0;
uint8_t status;

// FUNCTION SIGNATURE
uint8_t arm_drone(Servo* ESC);
void print_drone(drone* d);
void print_drone_csv(drone* d);
void print_drone_setup(drone* d);

/*###################################################################################*/
/*::: SETUP :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/*###################################################################################*/

void setup() {
  
  // INPUT PINS SETUP
  pinMode(INPUT_CH1,INPUT);
  pinMode(INPUT_CH2,INPUT);
  pinMode(INPUT_CH3,INPUT);
  pinMode(INPUT_CH4,INPUT);
  pinMode(INPUT_CH5,INPUT);

  // PWM SETUP
  ESC[0].attach(OUTPUT_M1, MIN_PWM_VALUE, MAX_PWM_VALUE);
  ESC[1].attach(OUTPUT_M2, MIN_PWM_VALUE, MAX_PWM_VALUE);
  ESC[2].attach(OUTPUT_M3, MIN_PWM_VALUE, MAX_PWM_VALUE);
  ESC[3].attach(OUTPUT_M4, MIN_PWM_VALUE, MAX_PWM_VALUE);

  // INTERRUPS SETUP
  PCICR |= (1 << PCIE0);    // enable interrupts on port B
  PCMSK0 |= (1 << PCINT4);  // turn on PB4
  
  PCICR |= (1 << PCIE2);    // enable interrupts on port D
  PCMSK2 |= (1 << PCINT23); // turn on PD7
  PCMSK2 |= (1 << PCINT22); // turn on PD6
  PCMSK2 |= (1 << PCINT21); // turn on PD5 
  PCMSK2 |= (1 << PCINT20); // turn on PD4

  // SERIAL COMMUNICATION SETUP
  Serial.begin(115200);

  // IMU SETUP
  status = mpu9250.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
  mpu9250.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  mpu9250.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  mpu9250.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  mpu9250.setSrd(19);

  // DRONE INITIALIZATION
  Serial.println("Initialize Drone...");
  drone_init(&d, ROLL_COEFF, PITCH_COEFF, YAW_COEFF, 0, 0, 0);
  print_drone_setup(&d);
  Serial.println("\nSIGNAL PROCESSING TEST:");
  _delay_ms(3000);
}

/*###################################################################################*/
/*::: M A I N :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/*###################################################################################*/

void loop() {

  if( d.state==disarmed && d.throttle[0]<=ESC_ARM_SIGNAL ){
    d.state = arm_drone(ESC);   
  }

  if(d.state==armed){

    // READ IMU DATA (to be fixed)
    /*
    mpu9250.readSensor();
    d.imu->gyro_x = mpu9250.getGyroX_rads();
    d.imu->gyro_y = mpu9250.getGyroY_rads();
    */

    // update iteration timestamp dt ...

    // PID control action
    compute_control_action(&d);

    // MOTOR PWM SIGNAL
    update = compute_motor_signal(&d);

    //update = update_needed(&d);   // avoid unnecessary computations
    if( update ){
      // GENERATE PWM SIGNAL for ESC
      ESC[0].write(d.m1);
      ESC[1].write(d.m2);
      ESC[2].write(d.m3);
      ESC[3].write(d.m4);
      //memcpy(&prev_d, &d, sizeof(drone));
    }
    print_drone(&d);
  }
}




//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

/**
 * @brief Interrupt Service Routine on Port D, Pins 4,5,6,7. Update channels 1(yaw), 2(pitch), 3(throttle), 4(roll).
 * 
 */
ISR(PCINT2_vect){
  timer[0] = micros();

  // read channel 1 YAW --------------------------
  if( last_channel[0]==0 && PIND & B10000000 ){
    last_channel[0] = 1;
    timer[1] = timer[0];
  }
  else if( last_channel[0]==1 && !(PIND & B10000000) ){
    last_channel[0] = 0;
    d.yaw[4] = d.yaw[3];
    d.yaw[3] = d.yaw[2];
    d.yaw[2] = d.yaw[1];
    d.yaw[1] = d.yaw[0];
    d.yaw[0] = timer[0] - timer[3];  //update yaw
  }
  
  // read channel 2 PITCH ------------------------
  if( last_channel[1]==0 && PIND & B01000000 ){
    last_channel[1] = 1;
    timer[1] = timer[0];
  }
  else if( last_channel[1]==1 && !(PIND & B01000000) ){
    last_channel[1] = 0;
    d.pitch[4] = d.pitch[3];
    d.pitch[3] = d.pitch[2];
    d.pitch[2] = d.pitch[1];
    d.pitch[1] = d.pitch[0];
    d.pitch[0] = timer[0] - timer[3];  //update pitch
  }

  // read channel 3 THRUST -----------------------
  if( last_channel[2]==0 && PIND & B00100000 ){
    last_channel[2] = 1;
    timer[3] = timer[0];
  }
  else if( last_channel[2]==1 && !(PIND & B00100000) ){
    last_channel[2] = 0;
    d.throttle[4] = d.throttle[3];
    d.throttle[3] = d.throttle[2];
    d.throttle[2] = d.throttle[1];
    d.throttle[1] = d.throttle[0];
    d.throttle[0] = timer[0] - timer[3];  //update throttle
  }
  
  // read channel 4 ROLL -------------------------
  if( last_channel[3]==0 && PIND & B00010000 ){
    last_channel[3] = 1;
    timer[4] = timer[0];
  }
  else if( last_channel[3]==1 && !(PIND & B00010000) ){
    last_channel[3] = 0;
    d.roll[4] = d.roll[3];
    d.roll[3] = d.roll[2];
    d.roll[2] = d.roll[1];
    d.roll[1] = d.roll[0];
    d.roll[0] = timer[0] - timer[3];  //update roll
  }
}

/**
 * @brief Interrupt Service Routine on Port B, Pin 4. Update channel 5 (vra)
 * 
 */
ISR(PCINT0_vect){
  timer[0] = micros();
  // read channel 4 VRA -------------------------
  if( last_channel[4]==0 && PINB & B00010000 ){
    last_channel[4] = 1;
    timer[5] = timer[0];
  }
  else if( last_channel[4]==1 && !(PINB & B00010000) ){
    last_channel[4] = 0;
    d.ch5 = timer[0] - timer[5];  //update roll
  }
}

/**
 * @brief Drone arming routine (to be moved in drone.cpp)
 * 
 * @param ESC array of Servo object.
 * @return 1 if arming routine completed.
 */
uint8_t arm_drone(Servo* ESC){
  Serial.println("\n# ESC arming ...");
  ESC[0].write(ESC_ARM_SIGNAL);
  ESC[1].write(ESC_ARM_SIGNAL);
  ESC[2].write(ESC_ARM_SIGNAL);
  ESC[3].write(ESC_ARM_SIGNAL);
  unsigned long now = millis();
  while (millis() < now + ESC_ARM_TIME){}
  digitalWrite(13,HIGH);
  Serial.println("# DRONE ARMED: Ready to fly.\n\n");
  now = millis();
  while (millis() < now + 1000){}
  return 1;
}

/**
 * @brief Print drone data (to be moved in drone.cpp)
 * 
 * @param d drone struct
 */
void print_drone(drone* d){
  Serial.print("[ T, R, P, Y, ch5 ]:  [ ");
  Serial.print(d->av_throttle); Serial.print(" | ");
  Serial.print(d->av_roll); Serial.print(" | ");
  Serial.print(d->av_pitch); Serial.print(" | ");
  Serial.print(d->av_yaw); Serial.print(" | ");
  Serial.print(d->ch5); Serial.println(" ]");
  Serial.print("[ M1, M2, M3, M4 ]:   [ ");
  Serial.print(d->m1); Serial.print(" | ");
  Serial.print(d->m2); Serial.print(" | ");
  Serial.print(d->m3); Serial.print(" | ");
  Serial.print(d->m4); Serial.println(" ]\n");
}


/**
 * @brief Print drone data in csv format (to be moved in drone.cpp)
 * 
 * @param d drone struct
 */
void print_drone_csv(drone* d){
  Serial.print(d->av_throttle);
  Serial.print(",");
  Serial.print(d->av_roll);
  Serial.print(",");
  Serial.print(d->av_pitch);
  Serial.print(",");
  Serial.print(d->av_yaw);
  Serial.print(",");
  Serial.print(d->m1);
  Serial.print(",");
  Serial.print(d->m2);
  Serial.print(",");
  Serial.print(d->m3);
  Serial.print(",");
  Serial.print(d->m4);
  Serial.print(",");
  Serial.println(d->ch5);
}

/**
 * @brief print drone initial setup (to be moved in drone.cpp)
 * 
 * @param d drone struct
 */
void print_drone_setup(drone* d){
  Serial.print("# State:  ");
  if (d->state==disarmed) Serial.println("disarmed");
  else if (d->state==armed) Serial.println("armed");
  Serial.print("# Roll coefficient:  ");
  Serial.println(d->roll_coeff);
  Serial.print("# Pitch coefficient: ");
  Serial.println(d->pitch_coeff);
  Serial.print("# Yaw coefficient:   ");
  Serial.println(d->yaw_coeff);
}
