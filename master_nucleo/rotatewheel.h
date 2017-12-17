#ifndef _ROTATE_WHEEL_H
#define _ROTATE_WHEEL_H

#include <stdlib.h>     /* abs */
#include "mbed.h"

/*
  This class perform the rotation of motor steps
*/
class RotateWheel{
private:
  DigitalOut * dir_pin;
  DigitalOut * step_pin;
public:
  RotateWheel(DigitalOut *, DigitalOut *);
  ~RotateWheel();
  void DoRotation(float, float);
  void RotateDeg(float, float);
};

#endif