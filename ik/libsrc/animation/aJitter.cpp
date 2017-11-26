#pragma once

#include "aJitter.h"

double AJitterVal(const AJitter& range)
{
    double x = ((double)rand()) / RAND_MAX;
    return range.first + x * (range.second - range.first);
}

vec3 AJitterVec(const AJitter& range)
{
    return vec3(AJitterVal(range), AJitterVal(range), AJitterVal(range));
}
