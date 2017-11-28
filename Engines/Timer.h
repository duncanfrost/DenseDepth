/*
 * Basic timer. Self explanatory
 */
#pragma once
#include <sys/time.h>
#include <iostream>
#include <string>

class Timer {
public:
    Timer(std::string name="") {
        this->name = name;
        this->display = true;
    }
    void start() {
        gettimeofday(&startTime, NULL);
    }
    void stop() {
        timeval endTime;
        gettimeofday(&endTime, NULL);
        elapsedTime = (endTime.tv_sec - startTime.tv_sec) * 1000.0;
        elapsedTime += (endTime.tv_usec - startTime.tv_usec) / 1000.0;
        if (display)
            std::cout << "Time "<<name<<": "<<elapsedTime << " ms.\n";
    }
    bool display;
    double elapsedTime;
private:
    timeval startTime;
    std::string name;
};
