#include "average.h"

ExponentialMovingAverage::ExponentialMovingAverage() {  }

ExponentialMovingAverage::~ExponentialMovingAverage() {   }

ExponentialMovingAverage::ExponentialMovingAverage(float a) {
    oldValue = -1;
    alpha = a;
}

float ExponentialMovingAverage::average(float value) {
    if (oldValue == -1) {
        oldValue = value;
        return value;
    }
    float newValue = oldValue + alpha * (value - oldValue);
    oldValue = newValue;
    
    return newValue;
}