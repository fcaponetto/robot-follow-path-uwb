#ifndef _TRIANGULATION_TWO_STEPS_H_
#define _TRIANGULATION_TWO_STEPS_H_

#include "errormeasure.h"
#include "master.h"

class TriangulationTwoSteps : Master{
  ErrorMeasure *err;
  int steps;
  float x_old;
  float x_new;
  float y_new;

  uint8 StateMachine();
  void ResetStateMachine(float);
  void PrintInformation();
  void FilterMeasure();
public:
  TriangulationTwoSteps();
  ~TriangulationTwoSteps();
  void loop();
};

#endif