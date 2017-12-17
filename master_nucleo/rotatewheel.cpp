#include "rotatewheel.h"

RotateWheel::RotateWheel(DigitalOut * dir, DigitalOut * step) 
        : dir_pin(dir), step_pin(step){
}

RotateWheel::~RotateWheel(){
}

void RotateWheel::DoRotation(float x_new, float x_old){
  float diff = abs(x_new - x_old);
    if(diff > 0.02){
      if(diff < 0.05){
        if(x_new > 0 )
          RotateDeg((1)*10, 00);
        else
          RotateDeg(-1*10, 200);
      }
      else if(diff < 0.10){
        if(x_new > 0 )
          RotateDeg((2)*10, 200);
        else
          RotateDeg(-2*10, 200);
      }
      else if(diff < 0.15){
        if(x_new > 0 )
          RotateDeg((3)*10, 300);
        else
          RotateDeg(-3*10, 300);
      }
      else if(diff < 0.20){
        if(x_new > 0 )
          RotateDeg((4)*10, 400);
        else
          RotateDeg(-4*10, 400);
      }
      else if(diff < 0.25){
        if(x_new > 0 )
          RotateDeg((5)*10, 500);
        else
          RotateDeg(-5*10, 500);
      }
      else if(diff < 0.30){
        if(x_new > 0 )
          RotateDeg((6)*10, 600);
        else
          RotateDeg(-6*10, 600);
      }
    }
}

void RotateWheel::RotateDeg(float deg, float wait_next_step){
  //pc.printf("Rotation: %f\n", deg);
  //rotate a specific number of degrees (negitive for reverse movement)
  int dir = (deg > 0)? 1:0;
  *dir_pin = dir;
  
  int steps_smooth = abs(0.5 * 10)*(1/0.225); // 360 / 0.225 = 1600 steps
  int steps = abs(deg)*(1/0.225); // 360 / 0.225 = 1600 steps

  for(;(steps - steps_smooth) != 0; steps_smooth++){
      *step_pin = 1;
      wait_us(wait_next_step); 

      *step_pin = 0;
      wait_us(wait_next_step);
  }
}