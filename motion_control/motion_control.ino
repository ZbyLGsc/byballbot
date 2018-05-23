/* Copyright (C) 2012 Kristian Lauszus, TKJ Electronics. All rights reserved.

 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").

 Contact information
 -------------------

 Kristian Lauszus, TKJ Electronics
 Web      :  http://www.tkjelectronics.com
 e-mail   :  kristianl@tkjelectronics.com
 */

#include <Wire.h>
#include "Kalman.h"  // Source: https://github.com/TKJElectronics/KalmanFilter


#include <ros.h>
#include <std_msgs/Float64.h>

// kalman filter caochao
#define KALMAN_CC_FILTER
#ifdef KALMAN_CC_FILTER
#include "kalman_filter.h"
#endif

//#define USE_LOWPASS_FILTER 0

#ifdef USE_LOWPASS_FILTER
#include <Filters.h>
#endif

#define RESTRICT_PITCH  // Comment out to restrict roll to ±90deg instead - please read:
                        // http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

#define INFINITE (unsigned long)1000000
#define P 0.15   
#define D 0.03
#define V_upper_limit 3.0f
#define ACC_upper_limit 5000000.0f

const int array_size = 1;
double sum_kalAngleX, sum_kalAngleY;
double pre_kalAngleX[array_size];
double pre_kalAngleY[array_size];

uint32_t Now = 0;
uint32_t lastUpdate = 0;
float deltat = 0.0f;
float sum = 0.0f;
uint32_t count = 0;
uint32_t sumCount = 0;
uint32_t delt_t = 0;

Kalman kalmanX;  // Create the Kalman instances
Kalman kalmanY;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle;  // Angle calculate using the gyro only
double compAngleX, compAngleY;  // Calculated angle using a complementary filter
double kalAngleX, kalAngleY;    // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14];  // Buffer for I2C data

float vs1 = 0.0, vs2 = 0.0, vs3 = 0.0;
float as1 = 0.0, as2 = 0.0, as3 = 0.0;
long delay1, delay2, delay3;
unsigned long v_last_time = 0;
float fRad2Deg = 57.295779513f;
bool init_pid_pitch = false;
bool init_pid_roll = false;

uint32_t last_print_t = 0;
uint32_t print_t = 0;

// TODO: Make calibration routine

// ROS stuff
// ros::NodeHandle nh;
// std_msgs::Float64 imu_pitch_msg;
// std_msgs::Float64 imu_roll_msg;
// ros::Publisher imu_pitch_pub("imu_pitch", &imu_pitch_msg);
// ros::Publisher imu_roll_pub("imu_roll", &imu_roll_msg);

// std_msgs::Float64 acc_pitch_msg;
// std_msgs::Float64 acc_roll_msg;
// ros::Publisher acc_pitch_pub("acc_pitch", &acc_pitch_msg);
// ros::Publisher acc_roll_pub("acc_roll", &acc_roll_msg);

// std_msgs::Float64 gyro_pitch_msg;
// std_msgs::Float64 gyro_roll_msg;
// ros::Publisher gyro_pitch_pub("gyro_pitch", &gyro_pitch_msg);
// ros::Publisher gyro_roll_pub("gyro_roll", &gyro_roll_msg);

// std_msgs::Float64 acc_x_msg;
// std_msgs::Float64 acc_y_msg;
// std_msgs::Float64 acc_z_msg;
// ros::Publisher acc_x_pub("acc_x", &acc_x_msg);
// ros::Publisher acc_y_pub("acc_y", &acc_y_msg);
// ros::Publisher acc_z_pub("acc_z", &acc_z_msg);

// stepper motor stuff
uint16_t counter_m[2];
uint16_t period_m[2];
uint8_t dir_m[2];

// motor 0 step: pin32 => PORTC 5
// motor 0 dir:  pin33 => PORTC 4
// motor 2 step: pin30 => PORTC 7
// motor 2 dir:  pin31 => PORTC 6
// motor 1 step: pin34 => PORTC 3
// motor 1 dir:  pin35 => PORTC 2
// motor 3 step: pin36 => PORTC 1
// motor 3 dir:  pin37 => PORTC 0
#define CLR(x, y) (x &= (~(1 << y)))
#define SET(x, y) (x |= (1 << y))

const uint8_t motor_step_0 = 5;
const uint8_t motor_dir_0 = 4;
const uint8_t motor_step_2 = 7;
const uint8_t motor_dir_2 = 6;
const uint8_t motor_step_1 = 3;
const uint8_t motor_dir_1 = 2;
const uint8_t motor_step_3 = 1;
const uint8_t motor_dir_3 = 0;

// PID stuff
float kp_pitch = P;
float kd_pitch = D;
float kp_roll = P;
float kd_roll = D;

float error_pitch = 0.0;
float pre_error_pitch = 0.0;
float error_roll = 0.0;
float pre_error_roll = 0.0;

float dt = 0.002;

float pid_output;

float velocity_pitch = 0.0;
float velocity_roll = 0.0;
float acc_pitch = 0.0;
float acc_roll = 0.0;

float setpoint_pitch = -3.3;
float setpoint_roll = -1.4;

// lowpass filter stuff
#ifdef USE_LOWPASS_FILTER
// filters out changes faster than 10hz
float filterFrequency = 1.5;

// create a one pole (RC) lowpass filter
FilterOnePole lowpassFilter_roll(LOWPASS, filterFrequency);
FilterOnePole lowpassFilter_pitch(LOWPASS, filterFrequency);

float filter_acc_roll;
float filter_acc_pitch;

std_msgs::Float64 filter_acc_pitch_msg;
std_msgs::Float64 filter_acc_roll_msg;
ros::Publisher filter_acc_pitch_pub("filter_acc_pitch", &filter_acc_pitch_msg);
ros::Publisher filter_acc_roll_pub("filter_acc_roll", &filter_acc_roll_msg);
#endif

#ifdef KALMAN_CC_FILTER
float kf_P = 1;
float kf_Q = 0.7;  // bigger tighter
float kf_R = 1;    // smaller tighter

float kf_acc_roll = 0;
float kf_acc_pitch = 0;
float kf_gyro_x_val = 0;
float kf_gyro_y_val = 0;

float kf_acc_x_val = 0;
float kf_acc_y_val = 0;
float kf_acc_z_val = 0;

float original_accX = 0;
float original_accY = 0;
float original_accZ = 0;

KalmanFilter *kf_roll;
KalmanFilter *kf_pitch;

KalmanFilter *kf_gyro_x;
KalmanFilter *kf_gyro_y;

KalmanFilter *kf_acc_x;
KalmanFilter *kf_acc_y;
KalmanFilter *kf_acc_z;

// std_msgs::Float64 kf_acc_pitch_msg;
// std_msgs::Float64 kf_acc_roll_msg;
// ros::Publisher kf_acc_pitch_pub("kf_acc_pitch", &kf_acc_pitch_msg);
// ros::Publisher kf_acc_roll_pub("kf_acc_roll", &kf_acc_roll_msg);

// std_msgs::Float64 kf_gyro_x_msg;
// std_msgs::Float64 kf_gyro_y_msg;
// ros::Publisher kf_gyro_x_pub("kf_gyro_x", &kf_gyro_x_msg);
// ros::Publisher kf_gyro_y_pub("kf_gyro_y", &kf_gyro_y_msg);

// std_msgs::Float64 kf_acc_x_msg;
// std_msgs::Float64 kf_acc_y_msg;
// std_msgs::Float64 kf_acc_z_msg;
// ros::Publisher kf_acc_x_pub("kf_acc_x", &kf_acc_x_msg);
// ros::Publisher kf_acc_y_pub("kf_acc_y", &kf_acc_y_msg);
// ros::Publisher kf_acc_z_pub("kf_acc_z", &kf_acc_z_msg);
#endif

float pid_pitch(float pv)
{
    error_pitch = setpoint_pitch - pv;

    if(!init_pid_pitch)
    {
        pre_error_pitch = error_pitch;
        init_pid_pitch = true;
    }

    pid_output = kp_pitch * error_pitch + kd_pitch * (error_pitch - pre_error_pitch) / dt;
    pre_error_pitch = error_pitch;
    if(fabs(pid_output) > ACC_upper_limit)
    {
        pid_output = pid_output > 0 ? ACC_upper_limit : -ACC_upper_limit;
    }
    return pid_output;
}

float pid_roll(float pv)
{
    error_roll = setpoint_roll - pv;

    if(!init_pid_roll)
    {
        pre_error_roll = error_roll;
        init_pid_roll = true;
    }

    pid_output = kp_roll * error_roll + kd_roll * (error_roll - pre_error_roll) / dt;
    pre_error_roll = error_roll;
    if(fabs(pid_output) > ACC_upper_limit)
    {
        pid_output = pid_output > 0 ? ACC_upper_limit : -ACC_upper_limit;
    }
    return pid_output;
}

ISR(TIMER1_COMPA_vect)
{
    for(int i = 0; i < 2; i++)
    {
        counter_m[i]++;
    }
    // Serial.print(counter_m[0]);
    if(counter_m[0] >= period_m[0])
    {
        counter_m[0] = 0;
        if(dir_m[0] == 0)
        {
            CLR(PORTC, motor_dir_2);
            SET(PORTC, motor_dir_3);
        }
        else
        {
            SET(PORTC, motor_dir_2);
            CLR(PORTC, motor_dir_3);
        }
        SET(PORTC, motor_step_2);
        SET(PORTC, motor_step_3);
        delayMicroseconds(1);
        CLR(PORTC, motor_step_2);
        CLR(PORTC, motor_step_3);
    }
    if(counter_m[1] >= period_m[1])
    {
        counter_m[1] = 0;
        if(dir_m[1] == 0)
        {
            CLR(PORTC, motor_dir_0);
            SET(PORTC, motor_dir_1);
        }
        else
        {
            SET(PORTC, motor_dir_0);
            CLR(PORTC, motor_dir_1);
        }
        SET(PORTC, motor_step_0);
        SET(PORTC, motor_step_1);
        delayMicroseconds(1);
        CLR(PORTC, motor_step_0);
        CLR(PORTC, motor_step_1);
    }
}

void setMotorSpeed(int motor_idx, float tspeed)
{
    float absspeed;
    if(tspeed > 0)
    {
        dir_m[motor_idx] = 0;
        absspeed = tspeed;
    }
    else
    {
        dir_m[motor_idx] = 1;
        absspeed = -tspeed;
    }
    period_m[motor_idx] = (int)(1000 / absspeed);
}

void set_delay_pitch(float acc)
{
    velocity_pitch += acc * dt;
    if(fabs(velocity_pitch) > V_upper_limit)
    {
        velocity_pitch = velocity_pitch > 0 ? V_upper_limit : -V_upper_limit;
    }
    if(fabs(velocity_pitch) > 0.002)
    {
        if(velocity_pitch > 0)
        {
            dir_m[0] = 0;
        }
        else
        {
            dir_m[0] = 1;
        }
        period_m[0] = int(fabs(1000 / velocity_pitch));
    }
    else
    {
        period_m[0] = INFINITE;
    }
}

void set_delay_roll(float acc)
{
    velocity_roll += acc * dt;
    if(fabs(velocity_roll) > V_upper_limit)
    {
        velocity_roll = velocity_roll > 0 ? V_upper_limit : -V_upper_limit;
    }
    if(fabs(velocity_roll) > 0.002)
    {
        if(velocity_roll > 0)
        {
            dir_m[1] = 1;
        }
        else
        {
            dir_m[1] = 0;
        }
        period_m[1] = int(fabs(1000 / velocity_roll));
    }
    else
    {
        period_m[1] = INFINITE;
    }
}

void setup()
{
    Serial.begin(115200);
    Wire.begin();
    TWBR = ((F_CPU / 400000L) - 16) / 2;  // Set I2C frequency to 400kHz

    i2cData[0] = 3;     // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
    i2cData[1] = 0x00;  // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
    i2cData[2] = 0x00;  // Set Gyro Full Scale Range to ±250deg/s
    i2cData[3] = 0x00;  // Set Accelerometer Full Scale Range to ±2g
    while(i2cWrite(0x19, i2cData, 4, false))
        ;  // Write to all four registers at once
    while(i2cWrite(0x6B, 0x01, true))
        ;  // PLL with X axis gyroscope reference and disable sleep mode

    while(i2cRead(0x75, i2cData, 1))
        ;
    if(i2cData[0] != 0x68)
    {  // Read "WHO_AM_I" register
        Serial.print(F("Error reading sensor"));
        while(1)
            ;
    }

    delay(100);  // Wait for sensor to stabilize

    /* Set kalman and gyro starting angle */
    while(i2cRead(0x3B, i2cData, 6))
        ;
    accX = (i2cData[0] << 8) | i2cData[1];
    accY = (i2cData[2] << 8) | i2cData[3];
    accZ = (i2cData[4] << 8) | i2cData[5];

    // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
    // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
    // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH  // Eq. 25 and 26
    double roll = atan2(accY, accZ) * RAD_TO_DEG;
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else  // Eq. 28 and 29
    double roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

    kalmanX.setAngle(roll);  // Set starting angle
    kalmanY.setAngle(pitch);
    gyroXangle = roll;
    gyroYangle = pitch;
    compAngleX = roll;
    compAngleY = pitch;

    timer = micros();

    // initialize ros
    // nh.initNode();
    // nh.advertise(imu_pitch_pub);
    // nh.advertise(imu_roll_pub);
    // nh.advertise(acc_pitch_pub);
    // nh.advertise(acc_roll_pub);
    // nh.advertise(gyro_pitch_pub);
    // nh.advertise(gyro_roll_pub);
    // nh.advertise(acc_x_pub);
    // nh.advertise(acc_y_pub);
    // nh.advertise(acc_z_pub);

#ifdef USE_LOWPASS_FILTER
    nh.advertise(filter_acc_pitch_pub);
    nh.advertise(filter_acc_roll_pub);
#endif

#ifdef KALMAN_CC_FILTER
    kf_roll = new KalmanFilter(kf_P, kf_Q, kf_R);
    kf_pitch = new KalmanFilter(kf_P, kf_Q, kf_R);

    kf_gyro_x = new KalmanFilter(kf_P, kf_Q, kf_R);
    kf_gyro_y = new KalmanFilter(kf_P, kf_Q, kf_R);

    // kf_Q = 0.7 kf_R = 1
    kf_acc_x = new KalmanFilter(kf_P, 0.7, 1);
    kf_acc_y = new KalmanFilter(kf_P, 0.7, 1);
    kf_acc_z = new KalmanFilter(kf_P, 0.7, 1);

    kf_roll->init();
    kf_pitch->init();

    kf_gyro_x->init();
    kf_gyro_y->init();

    kf_acc_x->init();
    kf_acc_y->init();
    kf_acc_z->init();

    // nh.advertise(kf_acc_pitch_pub);
    // nh.advertise(kf_acc_roll_pub);

    // nh.advertise(kf_gyro_x_pub);
    // nh.advertise(kf_gyro_y_pub);

    // nh.advertise(kf_acc_x_pub);
    // nh.advertise(kf_acc_y_pub);
    // nh.advertise(kf_acc_z_pub);
#endif

    // initialize motors
    pinMode(30, OUTPUT);
    pinMode(31, OUTPUT);
    pinMode(32, OUTPUT);
    pinMode(33, OUTPUT);
    pinMode(34, OUTPUT);
    pinMode(35, OUTPUT);
    pinMode(36, OUTPUT);
    pinMode(37, OUTPUT);

    // initialize timer
    noInterrupts();  // disable all interrupts
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;

    OCR1A = 10;               // compare match register 16MHz/256/2Hz
    TCCR1B |= (1 << WGM12);   // CTC mode
    TCCR1B |= (1 << CS12);    // 256 prescaler
    TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
    interrupts();             // enable all interrupts
                              //  setMotorSpeed(0, 100);
                              //  setMotorSpeed(1, -500);

    for(int i = 0; i < array_size; i++)
    {
        pre_kalAngleX[i] = 0;
        pre_kalAngleY[i] = 0;
    }

    // boyu
    String s = "<" + String(10000000) + ":" + String(10000000) + ":" + String(10000000) + ">";
    Serial.write(s.c_str());
    delay(2000);
    v_last_time = micros();
    last_print_t = millis();
    count = millis();
}

void loop()
{
    /* Update all the values */
    // read acceleration and angle rate from sensor
    while(i2cRead(0x3B, i2cData, 14))
        ;
    accX = ((i2cData[0] << 8) | i2cData[1]);
    accY = ((i2cData[2] << 8) | i2cData[3]);
    accZ = ((i2cData[4] << 8) | i2cData[5]);
    tempRaw = (i2cData[6] << 8) | i2cData[7];
    gyroX = (i2cData[8] << 8) | i2cData[9];
    gyroY = (i2cData[10] << 8) | i2cData[11];
    gyroZ = (i2cData[12] << 8) | i2cData[13];

    double dt = (double)(micros() - timer) / 1000000;  // Calculate delta time
    timer = micros();

    // use kalman filter to update sensor data
#ifdef KALMAN_CC_FILTER
    kf_gyro_x->update(gyroX);
    kf_gyro_y->update(gyroY);
    kf_gyro_x_val = kf_gyro_x->state();
    kf_gyro_y_val = kf_gyro_y->state();

    kf_acc_x->update(accX);
    kf_acc_y->update(accY);
    kf_acc_z->update(accZ);
    kf_acc_x_val = kf_acc_x->state();
    kf_acc_y_val = kf_acc_y->state();
    kf_acc_z_val = kf_acc_z->state();

    original_accX = accX;
    original_accY = accY;
    original_accZ = accZ;

    accX = kf_acc_x_val;
    accY = kf_acc_y_val;
    accZ = kf_acc_z_val;

    double gyroXrate = kf_gyro_x_val / 131.0;  // Convert to deg/s
    double gyroYrate = kf_gyro_y_val / 131.0;  // Convert to deg/s
#else
    double gyroXrate = gyroX / 131.0;  // Convert to deg/s
    double gyroYrate = gyroY / 131.0;  // Convert to deg/s
#endif

    // compute roll and pitch

    // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
    // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
    // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH  // Eq. 25 and 26
    double roll = atan2(accY, accZ) * RAD_TO_DEG;
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else  // Eq. 28 and 29
    double roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

#ifdef RESTRICT_PITCH
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90))
    {
        kalmanX.setAngle(roll);
        compAngleX = roll;
        kalAngleX = roll;
        gyroXangle = roll;
    }
    else
        kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt);  // Calculate the angle using a Kalman filter

    if(abs(kalAngleX) > 90)
        gyroYrate = -gyroYrate;  // Invert rate, so it fits the restriced accelerometer reading
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90))
    {
        kalmanY.setAngle(pitch);
        compAngleY = pitch;
        kalAngleY = pitch;
        gyroYangle = pitch;
    }
    else
        kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);  // Calculate the angle using a Kalman filter

    if(abs(kalAngleY) > 90)
        gyroXrate = -gyroXrate;  // Invert rate, so it fits the restriced accelerometer reading
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt);  // Calculate the angle using a Kalman filter
#endif

    gyroXangle += gyroXrate * dt;  // Calculate gyro angle without any filter
    gyroYangle += gyroYrate * dt;
    // gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
    // gyroYangle += kalmanY.getRate() * dt;

    compAngleX = 0.93 * (compAngleX + gyroXrate * dt) +
                 0.07 * roll;  // Calculate the angle using a Complimentary filter
    compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

    // Reset the gyro angle when it has drifted too much
    if(gyroXangle < -180 || gyroXangle > 180) gyroXangle = kalAngleX;
    if(gyroYangle < -180 || gyroYangle > 180) gyroYangle = kalAngleY;

        /* Print Data */
#if 0  // Set to 1 to activate
  Serial.print(accX); Serial.print("\t");
  Serial.print(accY); Serial.print("\t");
  Serial.print(accZ); Serial.print("\t");

  Serial.print(gyroX); Serial.print("\t");
  Serial.print(gyroY); Serial.print("\t");
  Serial.print(gyroZ); Serial.print("\t");

  Serial.print("\t");
#endif

    delt_t = millis() - count;
    if(delt_t > 500)
    {
        // Serial.print("x plane: ");
        // // Serial.print(roll); Serial.print(" ");
        // // Serial.print("gyro: "); Serial.print(gyroXangle); Serial.print(" ");
        // // Serial.print("comp: ");Serial.print(compAngleX); Serial.print(" ");
        // //    Serial.print("kal: ");
        // Serial.print(kalAngleX);
        // Serial.print(" ");

        // Serial.print(" ");

        // Serial.print("y plane: ");
        // // Serial.print(pitch); Serial.print(" ");
        // // Serial.print("gyro: ");Serial.print(gyroYangle); Serial.print(" ");
        // // Serial.print("comp: ");Serial.print(compAngleY); Serial.print(" ");
        // // Serial.print("kal: ");
        // Serial.print(kalAngleY);
        // Serial.print(" ");

        Serial.print("rate = ");
        Serial.print((float)sumCount / sum, 2);
        Serial.println(" Hz");

        count = millis();
    }

    Now = micros();
    deltat = ((Now - lastUpdate) / 1000000.0f);
    lastUpdate = Now;

    sum += deltat;
    sumCount++;

    sum_kalAngleX = 0;
    sum_kalAngleY = 0;
    for(int i = 0; i < array_size - 1; i++)
    {
        sum_kalAngleX += pre_kalAngleX[i];
        pre_kalAngleX[i] = pre_kalAngleX[i + 1];
        sum_kalAngleY += pre_kalAngleY[i];
        pre_kalAngleY[i] = pre_kalAngleY[i + 1];
    }

    sum_kalAngleX += kalAngleX;
    pre_kalAngleX[array_size - 1] = kalAngleX;
    sum_kalAngleY += kalAngleY;
    pre_kalAngleY[array_size - 1] = kalAngleY;

    acc_pitch = pid_pitch(sum_kalAngleY / array_size);
    acc_roll = pid_roll(sum_kalAngleX / array_size);

    // boyu, need to convert acceleration of ball to motors
    float a_x = -acc_pitch;
    float a_y = acc_roll;

    // Convert ball acceleration to motors' acceleration
    const float c_phi = cos(45 / fRad2Deg), kz = -0.1 * sin(45 / fRad2Deg);
    const float w_z = 0.0;
    as1 = -a_y * c_phi + kz * w_z;
    as2 = (sqrt(3) / 2 * a_x + 0.5 * a_y) * c_phi + kz * w_z;
    as3 = (-sqrt(3) / 2 * a_x + 0.5 * a_y) * c_phi + kz * w_z;

    // calculate motors' velocity
    unsigned long v_cur_time = micros();
    float dtv = double(v_cur_time - v_last_time) / 1000000.0;

    vs1 += as1 * dtv;
    vs2 += as2 * dtv;
    vs3 += as3 * dtv;

    // limit the velocity
    if(fabs(vs1) > V_upper_limit) vs1 = vs1 > 0.0 ? V_upper_limit : -V_upper_limit;
    if(fabs(vs2) > V_upper_limit) vs2 = vs2 > 0.0 ? V_upper_limit : -V_upper_limit;
    if(fabs(vs3) > V_upper_limit) vs3 = vs3 > 0.0 ? V_upper_limit : -V_upper_limit;

    // dead zone
    double k = 2500.0 / 8 / 2;
    delay1 = fabs(vs1) > 0.00001 ? k / vs1 : INFINITE;
    delay2 = fabs(vs2) > 0.00001 ? k / vs2 : INFINITE;
    delay3 = fabs(vs3) > 0.00001 ? k / vs3 : INFINITE;

    v_last_time = v_cur_time;

    print_t = millis() - last_print_t;
    if(print_t > 500)
    {
        // Serial.println("Vel:");
        // Serial.println(vs1);
        // Serial.println(vs2);
        // Serial.println(vs3);
        // Serial.println("");

        Serial.println("Acc:");
        Serial.println(as1);
        Serial.println(as2);
        Serial.println(as3);
        Serial.println("");

        Serial.println("delay:");
        Serial.println(delay1);
        Serial.println(delay2);
        Serial.println(delay3);
        Serial.println("");

        last_print_t = millis();
    }

    // send to motor control board
    String s = "<" + String(delay1) + ":" + String(delay2) + ":" + String(delay3) + ">";
    Serial.write(s.c_str());

    //  imu_pitch_msg.data = kalAngleY;
    //  imu_roll_msg.data = kalAngleX;
    // imu_pitch_msg.data = sum_kalAngleY / array_size;
    // imu_roll_msg.data = sum_kalAngleX / array_size;
    // imu_pitch_pub.publish(&imu_pitch_msg);
    // imu_roll_pub.publish(&imu_roll_msg);

    // acc_pitch_msg.data = acc_pitch / 100;
    // acc_roll_msg.data = acc_roll / 100;
    // acc_pitch_pub.publish(&acc_pitch_msg);
    // acc_roll_pub.publish(&acc_roll_msg);

    // gyro_pitch_msg.data = gyroX;
    // gyro_roll_msg.data = gyroY;
    // gyro_pitch_pub.publish(&gyro_pitch_msg);
    // gyro_roll_pub.publish(&gyro_roll_msg);

#ifdef USE_LOWPASS_FILTER
    lowpassFilter_roll.input(acc_roll);
    filter_acc_roll = lowpassFilter_roll.output();

    lowpassFilter_pitch.input(acc_pitch);
    filter_acc_pitch = lowpassFilter_pitch.output();

    filter_acc_roll_msg.data = filter_acc_roll / 100;
    filter_acc_roll_pub.publish(&filter_acc_roll_msg);

    filter_acc_pitch_msg.data = filter_acc_pitch / 100;
    filter_acc_pitch_pub.publish(&filter_acc_pitch_msg);

    set_delay_pitch(filter_acc_pitch);
    set_delay_roll(filter_acc_roll);
#else
    // boyu
    // set_delay_pitch(acc_pitch);
    // set_delay_roll(acc_roll);
#endif

#ifdef KALMAN_CC_FILTER
    kf_roll->update(acc_roll);
    kf_pitch->update(acc_pitch);
    kf_acc_roll = kf_roll->state();
    kf_acc_pitch = kf_pitch->state();

    // kf_acc_roll_msg.data = kf_acc_roll / 100;
    // kf_acc_roll_pub.publish(&kf_acc_roll_msg);

    // kf_acc_pitch_msg.data = kf_acc_pitch / 100;
    // kf_acc_pitch_pub.publish(&kf_acc_pitch_msg);

    // kf_gyro_x_msg.data = kf_gyro_x_val;
    // kf_gyro_x_pub.publish(&kf_gyro_x_msg);

    // kf_gyro_y_msg.data = kf_gyro_y_val;
    // kf_gyro_y_pub.publish(&kf_gyro_y_msg);

    // kf_acc_x_msg.data = kf_acc_x_val;
    // kf_acc_x_pub.publish(&kf_acc_x_msg);

    // kf_acc_y_msg.data = kf_acc_y_val;
    // kf_acc_y_pub.publish(&kf_acc_y_msg);

    // kf_acc_z_msg.data = kf_acc_z_val;
    // kf_acc_z_pub.publish(&kf_acc_z_msg);

    // acc_x_msg.data = original_accX;
    // acc_x_pub.publish(&acc_x_msg);

    // acc_y_msg.data = original_accY;
    // acc_y_pub.publish(&acc_y_msg);

    // acc_z_msg.data = original_accZ;
    // acc_z_pub.publish(&acc_z_msg);

#else

    acc_x_msg.data = accX;
    acc_x_pub.publish(&acc_x_msg);

    acc_y_msg.data = accY;
    acc_y_pub.publish(&acc_y_msg);

    acc_z_msg.data = accZ;
    acc_z_pub.publish(&acc_z_msg);

#endif

    // nh.spinOnce();

    //  Serial.println(period_m[0]);
    //  Serial.println(period_m[1]);
}
