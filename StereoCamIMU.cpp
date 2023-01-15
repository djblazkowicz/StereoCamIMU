#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <windows.h>
#include "StereoCamIMU.h"
#include <thread>
#include "serial/serial.h"
#include <chrono>
#include <string>

int main()
{

    int codec = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
    cv::VideoCapture cap1(0, cv::CAP_DSHOW);
    cap1.set(cv::CAP_PROP_FOURCC, codec);
    cap1.set(cv::CAP_PROP_AUTO_EXPOSURE, 0);
    cap1.set(cv::CAP_PROP_EXPOSURE, -6);
    cap1.set(cv::CAP_PROP_MONOCHROME, 1);
    cap1.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap1.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    cv::VideoCapture cap2(1, cv::CAP_DSHOW);
    cap2.set(cv::CAP_PROP_FOURCC, codec);
    cap2.set(cv::CAP_PROP_AUTO_EXPOSURE, 0);
    cap2.set(cv::CAP_PROP_EXPOSURE, -6);
    cap2.set(cv::CAP_PROP_MONOCHROME, 1);
    cap2.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap2.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    cap1.set(cv::CAP_PROP_SETTINGS, 0);
    cap2.set(cv::CAP_PROP_SETTINGS, 0);

    cv::namedWindow("full frame");
    cv::namedWindow("left image");
    cv::namedWindow("right image");

    serial::Serial IMU;
    bool IMUavailable = false;

    IMU.setPort("COM8");
    IMU.setBaudrate(115200);
    try {
        IMU.open();
    }
    catch (...)
    {
    }
    while (true)
    {
        newIMUAvailable = false;
        if (IMU.isOpen())
        {
            try
            {
                IMUstring = IMU.readline();
                std::string raw_str = IMUstring;

                if (raw_str.size() > 0)
                {

                    if (raw_str[0] != 0) {
                        if (raw_str.size() > 3) {

                            read_vals[0] = 0;
                            read_vals[1] = 0;
                            read_vals[2] = 0;
                            read_vals[3] = 0;
                            read_vals[4] = 0;
                            read_vals[5] = 0;
                            read_vals[6] = 0;

                            size_t pos = 0;
                            pos = raw_str.find(',');
                            read_vals[0] = std::stod(raw_str.substr(0, pos));
                            raw_str.erase(0, pos + 1);

                            pos = raw_str.find(',');
                            read_vals[1] = std::stod(raw_str.substr(0, pos));
                            raw_str.erase(0, pos + 1);

                            pos = raw_str.find(',');
                            read_vals[2] = std::stod(raw_str.substr(0, pos));
                            raw_str.erase(0, pos + 1);

                            pos = raw_str.find(',');
                            read_vals[3] = std::stod(raw_str.substr(0, pos));
                            raw_str.erase(0, pos + 1);

                            pos = raw_str.find(',');
                            read_vals[4] = std::stod(raw_str.substr(0, pos));
                            raw_str.erase(0, pos + 1);

                            pos = raw_str.find(',');
                            read_vals[5] = std::stod(raw_str.substr(0, pos));
                            raw_str.erase(0, pos + 1);

                            read_vals[6] = std::stod(raw_str);

                        }
                    }

                }

                frametype = read_vals[0];
                gyro[0] = read_vals[1];
                gyro[1] = read_vals[2];
                gyro[2] = read_vals[3];
                accel[0] = read_vals[4];
                accel[1] = read_vals[5];
                accel[2] = read_vals[6];

                std::cout << frametype << "," << gyro[0] << "," << gyro[1] << "," << gyro[2] << accel[0] << "," << accel[1] << "," << accel[2] << "\n";

                if (frametype == 2)
                {
                    if (!cameraOn) { cameraOn = true; }
                    cap1 >> right;
                    cap2 >> left;
                    cv::cvtColor(left, grayleft,cv::COLOR_BGR2GRAY);
                    cv::cvtColor(right, grayright, cv::COLOR_BGR2GRAY);

                    cv::hconcat(grayleft, grayright, frame);
                    cv::imshow("full frame", frame);
                    cv::imshow("left image", grayleft);
                    cv::imshow("right image", grayright);
                }

            }
            catch (...)
            {
                //pass
            }

        }
        char c = (char)cv::waitKey(1);
    }
    return 0;
}
