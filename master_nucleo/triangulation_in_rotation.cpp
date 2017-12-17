#include "triangulation_in_rotation.h"
#include "def_rotation.h"
#include <math.h>

TriangulationInRotation::TriangulationInRotation() : Master(){
  x_old = 0.0f;
  x_new = 0.0f;
  y_new  = 0.0f;
  //const float alpha = 0.1;
  start = 1;
}

TriangulationInRotation::~TriangulationInRotation(){
}

void TriangulationInRotation::loop(){
   //SetupInitialPosition();
    
    wait(2);
    timeElapsed.start();
    while(1) {
      if (circle == 5) {
        _timeNew = timeElapsed.read_ms();
        if (_timeNew - _timeOld >= _timeMeasure) {
          //Serial.println(_timeElapsedPoll - _timerPoll);
          _realAngle += (360 * (_timeNew - _timeOld)) / _timeCircle;
          _timeOld = _timeNew;

          if (_realAngle >= _fixedAngle && (_fixedAngle < 360)) {
              _fixedAngle += ANGLE;
              if(StateMachine() == TRUE)
                FilterMeasure();
          }
        }
      }
        
      if(ir_led)
        CompleteRotation();
    }
}

uint8 TriangulationInRotation::StateMachine(){
  { //Reset
    make_measure_left = true;
    make_measure_right = false;
    complete_left = false;
    complete_right = false;
  }
  
  while(1){
   if (make_measure_left) {
        //pc.printf("Make measure1 \t");
        make_measure_left = false;
        i2c.write(addr1, cmd, 1);
        timer_reset_tag.reset();
        timer_reset_tag.start();
    }

    if (complete_left) {
        complete_left = false;
        timer_reset_tag.stop();
        make_measure_right = true;
        read_measure(&distance_left, addr1);
    }
    
    if(make_measure_right) {
        make_measure_right = false;
        i2c.write(addr2, cmd, 1);
        timer_reset_tag.reset();
        timer_reset_tag.start();
    }

    if(complete_right){
        complete_right = false;
        timer_reset_tag.stop();
        make_measure_left = true;
        read_measure(&distance_right, addr2);
        
        x_new = (pow(distance_left, 2) - pow(distance_right, 2)) / (2 * BASE);
        y_new = 0.5 * sqrt( (-(pow(pow(distance_left,2) - pow(distance_right,2), 2)) / BASE)  + (2 * pow(distance_left,2)) + (2 * pow(distance_right,2))  - (pow(BASE, 2)));
        
        x_old = x_new;
        return TRUE;
    }
        
   if(timer_reset_tag.read() >= 0.125){
      ResetStateMachine(timer_reset_tag.read());
      return FALSE;
   }
  }
}

void TriangulationInRotation::FilterMeasure(){
    if(_realAngle == 0){
      if(err)
        delete err;
      err = new ErrorMeasure(distance_left, distance_right);
#ifdef VERBOSE
      PrintInformation();
      err->PrintPoints(&pc);
#endif
    }
    else{
        err->DeletePoints(distance_left, distance_right);
#ifdef VERBOSE
        PrintInformation();
        err->PrintPoints(&pc);
#endif
    }
     
}

void TriangulationInRotation::ResetStateMachine(float time_elapsed){
#ifdef VERBOSE
    pc.printf("RESET!!. The time taken was %f seconds\n", time_elapsed);
#endif
    /* clear the I2C buffer if it is not empty */
    i2c.read(0);
    //read_measure(&distance_left);
    //read_measure(&distance_right);
    
    make_measure_left = true;
    make_measure_right = false;
    complete_left = false;
    complete_right = false;
}

void TriangulationInRotation::PrintInformation(){
    pc.printf("Left: %f \t",distance_left);
    pc.printf("Right: %f\t",distance_right);
    pc.printf("X: %f\t", x_new);
    pc.printf("Y: %f\n", y_new);
}


void TriangulationInRotation::CompleteRotation() {
    _realAngle = 0;
    _fixedAngle = ANGLE;
    
    if (circle < 5) circle++;

    if (circle == 4) oldGiro = timeElapsed.read_ms();

    if (circle == 5) {
        if(StateMachine() == TRUE)
          FilterMeasure();
        newGiro = timeElapsed.read_ms();
        _timeCircle = (newGiro - oldGiro);
        _timeMeasure = ceil((newGiro - oldGiro) / 360);
        oldGiro = newGiro;
    }
}