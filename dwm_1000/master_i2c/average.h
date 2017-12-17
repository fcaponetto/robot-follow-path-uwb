#ifndef _EXP_AVG_
#define _EXP_AVG_

class ExponentialMovingAverage {
private:
    float alpha;
    float oldValue;
   
public:
    ExponentialMovingAverage();
    ExponentialMovingAverage(float);
    ~ExponentialMovingAverage();
    float average(float);
};
#endif