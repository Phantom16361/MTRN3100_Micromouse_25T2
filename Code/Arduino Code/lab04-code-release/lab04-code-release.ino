#include "DualEncoder.hpp"
#include "EncoderOdometry.hpp"
#include "IMUOdometry.hpp"
#include "Wire.h"
#include <MPU6050_light.h>

#include "Motor.hpp"

#define MOT1PWM 11
#define MOT1DIR 12
#define MOT2PWM 9
#define MOT2DIR 10

using namespace mtrn3100;

MPU6050 mpu(Wire);

#define EN_1_A 2 //These are the pins for the PCB encoder
#define EN_1_B 7 //These are the pins for the PCB encoder
#define EN_2_A 3 //These are the pins for the PCB encoder
#define EN_2_B 8 //These are the pins for the PCB encoder

mtrn3100::DualEncoder encoder(EN_1_A, EN_1_B,EN_2_A, EN_2_B);
mtrn3100::EncoderOdometry encoder_odometry(16.5, 93.0, false, false); //TASK1 TODO: IDENTIFY THE WHEEL RADIUS AND AXLE LENGTH
mtrn3100::IMUOdometry IMU_odometry;



void setup() {
    Serial.begin(115200);
    Wire.begin();


    //Set up the IMU
    byte status = mpu.begin();
    Serial.print(F("MPU6050 status: "));
    Serial.println(status);
    while(status!=0){ } // stop everything if could not connect to MPU6050
    
    Serial.println(F("Calculating offsets, do not move MPU6050"));
    delay(1000);
    mpu.calcOffsets(true,true);
    Serial.println("Done!\n");

    pinMode(MOT1PWM, OUTPUT);
    pinMode(MOT1DIR, OUTPUT);
    pinMode(MOT2PWM, OUTPUT);
    pinMode(MOT2DIR, OUTPUT);

    Motor motor1(MOT1PWM, MOT1DIR);
    Motor motor2(MOT2PWM, MOT2DIR);

    motor1.setPWM(150);
    motor2.setPWM(150);
}


void loop() {

//UNCOMMENT FOR TASK 2: 
//THE DELAY IS REQUIRED OTHERWISE THE ENCODER DIFFERENCE IS TOO SMALL
    delay(50);
    encoder_odometry.update(encoder.getLeftRotation(),encoder.getRightRotation());

//UNCOMMET FOR TASK 3:
//NOTE: IMU ODOMETRY IS REALLY BAD, THIS TASK EXISTS TO TEACH YOU WHY IMU ODOMETRY SUCKS, DO NOT SPEND TOO LONG ON IT
    //mpu.update();
    //IMU_odometry.update(mpu.getAccX(),mpu.getAccY());
    

    Serial.print("ODOM:\t\t");
    Serial.print(encoder_odometry.getX());
    Serial.print(",\t\t");
    Serial.print(encoder_odometry.getY());
    Serial.print(",\t\t");
    Serial.print(encoder_odometry.getH());
    Serial.println();
}
