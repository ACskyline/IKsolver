#pragma once
#include "aVector.h"

typedef std::pair<double, double> AJitter; // min, max 
extern double AJitterVal(const AJitter& range);
extern vec3 AJitterVec(const AJitter& range);
