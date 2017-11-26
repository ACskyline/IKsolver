//------------------------------------------------------------------------
// Copyright (C) 2015 by Aline Normoyle

#include "aTimer.h"

bool ATimer::gInit = false;
LARGE_INTEGER ATimer::gFrequency;

ATimer::ATimer()
{
    if (!gInit)
    {
        QueryPerformanceFrequency(&gFrequency);
        if (gFrequency.QuadPart == 0)
        {
            std::cout << "This computer does not support High Resolution Performance counter" << std::endl;
        }
        else
        {
            std::cout << "Timer frequency = " << double(gFrequency.QuadPart) << std::endl;
        }
        gInit = true;
    }   
    mTotalElapsedTime = 0.0;
    mStarted = false;
}

ATimer::~ATimer()
{
}

void ATimer::reset()
{
    mTotalElapsedTime = 0.0;
    mStarted = false;
}

void ATimer::start()
{
    if (!mStarted) QueryPerformanceCounter(&mStartTime);
    mStarted = true;
}

void ATimer::pause()
{
    if (mStarted)
    {
        LARGE_INTEGER currentTime;
        QueryPerformanceCounter(&currentTime);
        mTotalElapsedTime += double(currentTime.QuadPart - mStartTime.QuadPart) / double(gFrequency.QuadPart);
        mStarted = false;
    }
}

double ATimer::totalElapsedTime()
{
    if (mStarted)
    {
        LARGE_INTEGER currentTime;
        QueryPerformanceCounter(&currentTime);
        mTotalElapsedTime += double(currentTime.QuadPart - mStartTime.QuadPart) / double(gFrequency.QuadPart);
        mStartTime = currentTime;
    }
    return mTotalElapsedTime;
}

void ATimer::restart()
{
    reset();
    start();
}
