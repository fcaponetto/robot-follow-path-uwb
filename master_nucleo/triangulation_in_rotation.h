#ifndef _ROTATION_H_
#define _ROTATION_H_

#include "errormeasure.h"
#include "master.h"

class TriangulationInRotation : Master{
private:
  ErrorMeasure *err;
  Timer timeElapsed;
  
  uint32 _timeNew;
  uint32 _timeOld;
  uint32 oldGiro;
  uint32 newGiro;
  uint8 circle;

  float _realAngle;
  float _fixedAngle;
  float _timeMeasure;
  float _timeCircle;
  
  float x_old;
  float x_new;
  float y_new;
  int start;
  
  uint8 StateMachine();
  void ResetStateMachine(float);
  void PrintInformation();
  void CompleteRotation();
  void FilterMeasure();
public:
  TriangulationInRotation();
  ~TriangulationInRotation();
  void loop();
 
};

#endif