#ifndef _ERROR_MEASURE_H_
#define _ERROR_MEASURE_H_

#include <list>
#include <cstdlib>
#include <ctime>
#include "mbed.h"

#define NUMBER_POINTS 100
#define ERROR_MEASURE 0.1f

typedef struct{
  float distance_left;
  float distance_right;
}tipobaseList;

class ErrorMeasure{
private:
  float distance_left;
  float distance_right;
  std::list<tipobaseList> points;
  void DoPointsCloud();
public:
  ErrorMeasure();
  ErrorMeasure(float,float);
  ~ErrorMeasure();
  void DeletePoints(float, float);
  void PrintPoints(Serial *);
  void MakeAverage(float *, float *);
};

#endif