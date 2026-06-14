#include "BNO055.h"
#include <math.h>

static Adafruit_BNO055 bno(55);

static float roll_deg  = 0.0f;
static float pitch_deg = 0.0f;
static float yaw_deg   = 0.0f;

bool initBNO055()
{
    if (!bno.begin())
    {
        Serial.println("BNO055 not found");
        return false;
    }

    delay(1000);
    bno.setExtCrystalUse(true);

    return true;
}

void updateIMU()
{
    imu::Quaternion q = bno.getQuat();

    float qw = q.w();
    float qx = q.x();
    float qy = q.y();
    float qz = q.z();

    float sinr_cosp = 2.0f * (qw * qx + qy * qz);
    float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
    roll_deg = atan2(sinr_cosp, cosr_cosp) * 180.0f / PI;

    float sinp = 2.0f * (qw * qy - qz * qx);

    if (fabs(sinp) >= 1.0f)
        pitch_deg = copysign(90.0f, sinp);
    else
        pitch_deg = asin(sinp) * 180.0f / PI;

    float siny_cosp = 2.0f * (qw * qz + qx * qy);
    float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);

    yaw_deg = atan2(siny_cosp, cosy_cosp) * 180.0f / PI;

    if (yaw_deg < 0)
        yaw_deg += 360.0f;
}

float getRoll()
{
    return roll_deg;
}

float getPitch()
{
    return pitch_deg;
}

float getYaw()
{
    return yaw_deg;
}