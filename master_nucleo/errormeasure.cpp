#include "errormeasure.h"

ErrorMeasure::ErrorMeasure(){
}

ErrorMeasure::ErrorMeasure(float d_l, float d_r) : distance_left(d_l), distance_right(d_r){
  DoPointsCloud();
}

ErrorMeasure::~ErrorMeasure(){
}

void ErrorMeasure::DoPointsCloud(){
  std::srand(std::time(0)); // use current time as seed for random generator
  for (int n=0; n < NUMBER_POINTS; ++n){
      float random = ((float)(std::rand() % 20+ (-10)))/((float)100);
      tipobaseList elem = {distance_left + random, distance_right+random};
      points.push_back(elem);
  }
}

void ErrorMeasure::DeletePoints(float new_distance_left, float new_distance_right){
  for (std::list<tipobaseList>::iterator it=points.begin(); it != points.end();){
    if(it->distance_left > (new_distance_left + ERROR_MEASURE))
      it = points.erase(it);
    else
      ++it;
    if(it != points.end()){
      if(it->distance_right > (new_distance_right + ERROR_MEASURE))
         it = points.erase(it);
      ++it;
    }
    else
      break;
  }  
}

void ErrorMeasure::PrintPoints(Serial * pc){
  //pc->printf("\tSize List: %d\n", points.size());
  /*
  for (std::list<tipobaseList>::iterator it=points.begin(); it != points.end(); ++it){
      (*pc).printf("%f\t", (*it).distance_left);
      (*pc).printf("%f\n", (*it).distance_right);
  }
  */
}

void ErrorMeasure::MakeAverage(float * d_l, float * d_r){
  for (std::list<tipobaseList>::iterator it=points.begin(); it != points.end(); ++it){
    *d_l += it->distance_left;
    *d_r += it->distance_right;
  }
  *d_l /= points.size();
  *d_r /= points.size();
}

