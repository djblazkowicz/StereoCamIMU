#pragma once
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <thread>
#include <vector>


class MyCap {
public:
	MyCap();
	void run();
	bool keepRunning = true; // Will be changed by the external program.
	static void RunThreads(MyCap* cap);
	static std::vector<std::thread> capThreads;
	static MyCap* getInstance();
private:
	static MyCap* instance;
};