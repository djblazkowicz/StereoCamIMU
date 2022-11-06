#pragma once

#include <opencv2/opencv.hpp>
//#include <windows.h>
#include <thread>
#include "serial/serial.h"

cv::Mat frame;
cv::Mat left;
cv::Mat right;

bool newFrameAvailable;
bool newIMUAvailable;
bool cameraOn;
bool turnOnIMU;

std::string IMUstring;
std::vector<std::string> words;
double read_vals[6];
double accel[3];
double gyro[3];

void getFrame();
void getImu();




