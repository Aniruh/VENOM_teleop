#ifndef BNO055_H
#define BNO055_H

#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

bool initBNO055();
void updateIMU();

float getRoll();
float getPitch();
float getYaw();

#endif