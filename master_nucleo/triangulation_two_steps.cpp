#include "triangulation_two_steps.h"
#define MICRO_STEP 250

TriangulationTwoSteps::TriangulationTwoSteps() : Master(), steps(0){
  x_old = 0.0f;
  x_new = 0.0f;
  y_new  = 0.0f;
}

TriangulationTwoSteps::~TriangulationTwoSteps(){
}

uint8 TriangulationTwoSteps::StateMachine(){
  if (make_measure_left) {
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
        
        if(steps == 0){
          if(err)
            delete err;
          err = new ErrorMeasure(distance_left, distance_right);
#ifdef VERBOSE
          pc.printf("***********************\n");
          pc.printf("Posizione Centrale\n");
          PrintInformation();
#endif
          err->PrintPoints(&pc);
          rot.RotateDeg(25 * 10,MICRO_STEP);
          steps++;
        }
        else{
          if(steps == 1){
#ifdef VERBOSE
            pc.printf("Posizione Destra\n");
            PrintInformation();
#endif
            err->DeletePoints(distance_left, distance_right);
            err->PrintPoints(&pc);
            rot.RotateDeg((-45) * 10,MICRO_STEP);
            steps++;
          }
          else{
            steps = 0;
#ifdef VERBOSE
            pc.printf("Posizione Sinistra\n");
            PrintInformation();
#endif
            err->DeletePoints(distance_left, distance_right);
            err->PrintPoints(&pc);
            err->MakeAverage(&distance_left, &distance_right);
            x_new = (pow(distance_left, 2) - pow(distance_right, 2)) / (2 * BASE);
            y_new = 0.5 * sqrt( (-(pow(pow(distance_left,2) - pow(distance_right,2), 2)) / BASE)  + (2 * pow(distance_left,2)) + (2 * pow(distance_right,2))  - (pow(BASE, 2)));
            pc.printf("%f\n", x_new);
            pc.printf("%f\n", y_new);
#ifdef VERBOSE
            pc.printf("***********************\n");
            pc.printf("\n");
            pc.printf("\n");
            pc.printf("\n");
#endif
            while(!ir_led){
              rot.RotateDeg(5*10, MICRO_STEP);
            }
          }   
        }
        if(ir_led == 1)
          ir_led = 0;
                       
        //rot.DoRotation(x_new, x_old);
        x_old = x_new;
        
    }
  
  if(timer_reset_tag.read() >= 0.125){
      ResetStateMachine(timer_reset_tag.read());
      return FALSE;
   }
  return TRUE;
}

void TriangulationTwoSteps::FilterMeasure(){
}

void TriangulationTwoSteps::loop(){
    SetupInitialPosition();
 
    wait(2);
    while(1) {
      StateMachine();
    }
}

void TriangulationTwoSteps::PrintInformation(){
}

void TriangulationTwoSteps::ResetStateMachine(float time){
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