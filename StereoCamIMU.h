#pragma once

#include <opencv2/opencv.hpp>
//#include <windows.h>
#include <thread>
#include "serial/serial.h"

cv::Mat frame;
cv::Mat grayleft;
cv::Mat grayright;
cv::Mat left;
cv::Mat right;

bool newFrameAvailable;
bool newIMUAvailable;
bool cameraOn = false;
bool turnOnIMU;
bool getNewFrame = false;

std::string IMUstring;
std::vector<std::string> words;
double read_vals[7];
int frametype;
double accel[3];
double gyro[3];
