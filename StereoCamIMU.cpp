#include <iostream>
#include <opencv2/opencv.hpp>
//#include <windows.h>
#include "StereoCamIMU.h"
#include <thread>
#include "serial/serial.h"
#include <chrono>
#include <string>

void getIMU()
{
    using namespace std::literals::chrono_literals;
    newIMUAvailable = false;


    serial::Serial IMU;
    bool IMUavailable = false;

    IMU.setPort("COM12");
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


                            read_vals[5] = std::stod(raw_str);

                        }
                    }

                }

                gyro[0] = read_vals[0];
                gyro[1] = read_vals[1];
                gyro[2] = read_vals[2];
                accel[0] = read_vals[3];
                accel[1] = read_vals[4];
                accel[2] = read_vals[5];

                newIMUAvailable = true;
                std::this_thread::sleep_for(5ms);
            }
            catch (...)
            {
                newIMUAvailable = false;
            }
        }
        else
        {
            newIMUAvailable = false;
        }

    }
}


void getFrame()
{
    using namespace std::literals::chrono_literals;

    

    double delta = 16;
    double elapsed_time;

    int codec = cv::VideoWriter::fourcc('m', 'j', 'p', '2');
    cv::VideoCapture cap(0);
    cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);
    //cap.set(cv::CAP_PROP_EXPOSURE, -6);
    cap.set(cv::CAP_PROP_FOURCC, codec);
    cap.set(cv::CAP_PROP_FPS, 60);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    
    
    cv::namedWindow("full frame");
    cv::namedWindow("left image");
    cv::namedWindow("right image");

    auto frameStart = std::chrono::system_clock::now();
    bool tookFirstFrame = false;
    while (true)
    {

        newFrameAvailable = false;
        cap >> frame;
        if (!tookFirstFrame)
        {
            auto frameStart = std::chrono::system_clock::now();
        }

        cameraOn = true;

        cv::imshow("full frame", frame);

        auto stopwatch = std::chrono::system_clock::now();
        elapsed_time = double(std::chrono::duration_cast <std::chrono::milliseconds> (stopwatch - frameStart).count());
        if (elapsed_time >= delta)
        {
            left = frame(cv::Rect(0, 0, 640, 480));
            right = frame(cv::Rect(640, 0, 640, 480));
            cv::imshow("left image", left);
            cv::imshow("right image", right);
            newFrameAvailable = true;
            tookFirstFrame = true;
            auto frameStart = std::chrono::system_clock::now();
        }
        char c = (char)cv::waitKey(1);
    }        
}

int main()
{
    cameraOn = false;
    using namespace std::chrono_literals;
   
    auto start = std::chrono::system_clock::now();
    double elapsed_time;

    std::thread camWorker(getFrame);
    std::cout << "waiting for camera to initialize..." << std::endl;

    while(!cameraOn)
    {
        std::this_thread::sleep_for(1ms);
    }

    std::thread imuWorker(getIMU);

    while (true)
    {
        if (newFrameAvailable)
        {
            auto end = std::chrono::system_clock::now();
            elapsed_time = double(std::chrono::duration_cast <std::chrono::milliseconds> (end - start).count());
            std::cout << "frame timestamp:" << elapsed_time / 1000.0 << std::endl;
        }
        if (newIMUAvailable)
        {
            auto end = std::chrono::system_clock::now();
            elapsed_time = double(std::chrono::duration_cast <std::chrono::milliseconds> (end - start).count());
            std::cout << "imu timestamp:" << elapsed_time / 1000.0 << std::endl;
        }
    }

    return 0;
}
