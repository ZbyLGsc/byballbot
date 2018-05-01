//连线方法
// MPU-UNO
// VCC-VCC
// GND-GND
// SCL-21
// SDA-20
// INT-2 (Optional)

#include <FlexiTimer2.h>
#include <Kalman.h>
#include <Wire.h>
#include <math.h>

#define MOTOR1_DIR_PIN 46
#define MOTOR2_DIR_PIN 22
#define MOTOR3_DIR_PIN 34

#define MOTOR1_VEL_PIN 50
#define MOTOR2_VEL_PIN 26
#define MOTOR3_VEL_PIN 38

// IMU parameter
float fRad2Deg = 57.295779513f;  //将弧度转为角度的乘数
const int MPU = 0x68;            // MPU-6050的I2C地址
const int nValCnt = 7;           //一次读取寄存器的数量

const int nCalibTimes = 1000;  //校准时读数的次数
int calibData[nValCnt];        //校准数据

unsigned long nLastTime = 0;  //上一次读数的时间
float fLastRoll = 0.0f;       //上一次滤波得到的Roll角
float fLastPitch = 0.0f;      //上一次滤波得到的Pitch角
Kalman kalmanRoll;            // Roll角滤波器
Kalman kalmanPitch;           // Pitch角滤波器

// motor control
float vs1 = 0.0, vs2 = 0.0, vs3 = 0.0;
long delay1, delay2, delay3;
const unsigned long delay_imu = 2000;
unsigned long last_step_imu = 0;

int count = 0;

void setup()
{
    Serial.begin(115200);  //初始化串口，指定波特率
    Serial.println("start:");

    // setup IMU
    Wire.begin();          //初始化Wire库
    WriteMPUReg(0x6B, 0);  //启动MPU6050设备
    Calibration();         //执行校准
    nLastTime = micros();  //记录当前时间
}

void loop()
{
    // imu
    unsigned long now = micros();
    if(last_step_imu + delay_imu <= now)
    {
        // Serial.println("imu");
        last_step_imu += delay_imu;
        float roll, pitch, roll_rate, pitch_rate;
        getAngleAndRate(roll, pitch, roll_rate, pitch_rate);

        // get the needed motor output
        getControlOutput(roll, pitch, roll_rate, pitch_rate, vs1, vs2, vs3);
        // vs1 = 1.0;
        // vs2 = 0.3;
        // vs3 = 1.0;

        // if(vs1 == 0.0)
        //     delay1 = 1000000;
        // else
        //     delay1 = int(2500.0 / 8 / vs1);

        // if(vs2 == 0.0)
        //     delay2 = 1000000;
        // else
        //     delay2 = int(2500.0 / 8 / vs2);

        // if(vs3 == 0.0)
        //     delay3 = 1000000;
        // else
        //     delay3 = int(2500.0 / 8 / vs3);

        // String s = "<" + String(delay1) + ":" + String(delay2) + ":" + String(delay3) + ">";
        String s = "<" + String(vs1) + ":" + String(vs2) + ":" + String(vs3) + ">";
        Serial.write(s.c_str());

        count++;
        if(count >= 50)
        {
            Serial.println("angle:");
            Serial.println(-pitch);
            Serial.println(-roll);

            Serial.println("Vel:");
            Serial.println(vs1);
            Serial.println(vs2);
            Serial.println(vs3);

            Serial.println("delay:");
            Serial.println(delay1);
            Serial.println(delay2);
            Serial.println(delay3);
            Serial.println("");

            count = 0;
        }
    }
}

// get needed control output
void getControlOutput(const float roll, const float pitch, const float roll_rate, const float pitch_rate,
                      float &vs1, float &vs2, float &vs3)
{
    // convert roll and pitch to plannar theta
    float theta_x = -pitch;
    float theta_x_dot = -pitch_rate;
    float theta_y = -roll;
    float theta_y_dot = -roll_rate;

    // Then the required accerleration of control can be computed
    const float ka = 0.2, kav = 0.003;
    float a_x, a_y;
    // if(abs(theta_x) < 5)
    //     a_x = 0.0;
    // else
    a_x = ka * theta_x + kav * theta_x_dot;

    // if(abs(theta_y) < 5)
    //     a_y = 0.0;
    // else
    a_y = ka * theta_y + kav * theta_y_dot;

    // Convert ball velocity to motor velocity
    const float c_phi = cos(45 / fRad2Deg), kz = -0.1 * sin(45 / fRad2Deg);
    float w_z = 0.0;
    vs1 = -a_y * c_phi + kz * w_z;
    vs2 = (sqrt(3) / 2 * a_x + 0.5 * a_y) * c_phi + kz * w_z;
    vs3 = (-sqrt(3) / 2 * a_x + 0.5 * a_y) * c_phi + kz * w_z;
    //
    if(abs(vs1) < 0.04) vs1 = 0;
    if(abs(vs2) < 0.04) vs2 = 0;
    if(abs(vs3) < 0.04) vs3 = 0;

    const float limit = 0.4;
    if(abs(vs1) > limit) vs1 = vs1 > 0.0 ? limit : -limit;
    if(abs(vs2) > limit) vs2 = vs2 > 0.0 ? limit : -limit;
    if(abs(vs3) > limit) vs3 = vs3 > 0.0 ? limit : -limit;

    // bound
    // float bound = 0.3;
    // if(vs1 > 0 && vs1 < bound)
    //     vs1 = bound;
    // else if(vs1 < 0 && vs1 > -bound)
    //     vs1 = -bound;

    // if(vs2 > 0 && vs2 < bound)
    //     vs2 = bound;
    // else if(vs2 < 0 && vs2 > -bound)
    //     vs2 = -bound;

    // if(vs3 > 0 && vs3 < bound)
    //     vs3 = bound;
    // else if(vs3 < 0 && vs3 > -bound)
    //     vs3 = -bound;
}

// get roll and pitch (also rate)
void getAngleAndRate(float &roll, float &pitch, float &roll_rate, float &pitch_rate)
{
    int readouts[nValCnt];
    ReadAccGyr(readouts);  //读出测量值

    float realVals[7];
    Rectify(readouts, realVals);  //根据校准的偏移量进行纠正

    //计算加速度向量的模长，均以g为单位
    float fNorm = sqrt(realVals[0] * realVals[0] + realVals[1] * realVals[1] + realVals[2] * realVals[2]);
    float fRoll = GetRoll(realVals, fNorm);  //计算Roll角
    if(realVals[1] > 0) fRoll = -fRoll;
    float fPitch = GetPitch(realVals, fNorm);  //计算Pitch角
    if(realVals[0] < 0) fPitch = -fPitch;

    //计算两次测量的时间间隔dt，以秒为单位
    unsigned long nCurTime = micros();
    float dt = (double)(nCurTime - nLastTime) / 1000000.0;
    //对Roll角和Pitch角进行卡尔曼滤波
    float fNewRoll = kalmanRoll.getAngle(fRoll, realVals[4], dt);
    float fNewPitch = kalmanPitch.getAngle(fPitch, realVals[5], dt);
    //跟据滤波值计算角度速
    float fRollRate = (fNewRoll - fLastRoll) / dt;
    float fPitchRate = (fNewPitch - fLastPitch) / dt;

    //更新Roll角和Pitch角
    fLastRoll = fNewRoll;
    fLastPitch = fNewPitch;
    //更新本次测的时间
    nLastTime = nCurTime;

    // output
    roll = fNewRoll;
    pitch = fNewPitch;
    roll_rate = fRollRate;
    pitch_rate = fPitchRate;
}

//向MPU6050写入一个字节的数据
//指定寄存器地址与一个字节的值
void WriteMPUReg(int nReg, unsigned char nVal)
{
    Wire.beginTransmission(MPU);
    Wire.write(nReg);
    Wire.write(nVal);
    Wire.endTransmission(true);
}

//从MPU6050读出一个字节的数据
//指定寄存器地址，返回读出的值
unsigned char ReadMPUReg(int nReg)
{
    Wire.beginTransmission(MPU);
    Wire.write(nReg);
    Wire.requestFrom(MPU, 1, true);
    Wire.endTransmission(true);
    return Wire.read();
}

//从MPU6050读出加速度计三个分量、温度和三个角速度计
//保存在指定的数组中
void ReadAccGyr(int *pVals)
{
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.requestFrom(MPU, nValCnt * 2, true);
    Wire.endTransmission(true);
    for(long i = 0; i < nValCnt; ++i)
    {
        pVals[i] = Wire.read() << 8 | Wire.read();
    }
}

//对大量读数进行统计，校准平均偏移量
void Calibration()
{
    float valSums[7] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0 };
    //先求和
    for(int i = 0; i < nCalibTimes; ++i)
    {
        int mpuVals[nValCnt];
        ReadAccGyr(mpuVals);
        for(int j = 0; j < nValCnt; ++j)
        {
            valSums[j] += mpuVals[j];
        }
    }
    //再求平均
    for(int i = 0; i < nValCnt; ++i)
    {
        calibData[i] = int(valSums[i] / nCalibTimes);
    }
    calibData[2] += 16384;  //设芯片Z轴竖直向下，设定静态工作点。
}

//算得Roll角。算法见文档。
float GetRoll(float *pRealVals, float fNorm)
{
    float fNormXZ = sqrt(pRealVals[0] * pRealVals[0] + pRealVals[2] * pRealVals[2]);
    float fCos = fNormXZ / fNorm;
    return acos(fCos) * fRad2Deg;
}

//算得Pitch角。算法见文档。
float GetPitch(float *pRealVals, float fNorm)
{
    float fNormYZ = sqrt(pRealVals[1] * pRealVals[1] + pRealVals[2] * pRealVals[2]);
    float fCos = fNormYZ / fNorm;
    return acos(fCos) * fRad2Deg;
}

//对读数进行纠正，消除偏移，并转换为物理量。公式见文档。
void Rectify(int *pReadout, float *pRealVals)
{
    for(int i = 0; i < 3; ++i)
    {
        pRealVals[i] = (float)(pReadout[i] - calibData[i]) / 16384.0f;
    }
    pRealVals[3] = pReadout[3] / 340.0f + 36.53;
    for(int i = 4; i < 7; ++i)
    {
        pRealVals[i] = (float)(pReadout[i] - calibData[i]) / 131.0f;
    }
}
