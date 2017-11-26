//------------------------------------------------------------------------
// Copyright (C) 2015 by Aline Normoyle

#ifndef Timer_H_
#define Timer_H_

#include <iostream>
#include <windows.h>

class ATimer
{
public:
    ATimer(void);
    ~ATimer(void);
    void reset();
    void start();
    void pause();
    void restart();
    double totalElapsedTime();

protected:
    double mTotalElapsedTime;
    LARGE_INTEGER mStartTime;
    bool mStarted;

    static LARGE_INTEGER gFrequency;
    static bool gInit;
};

#endif
