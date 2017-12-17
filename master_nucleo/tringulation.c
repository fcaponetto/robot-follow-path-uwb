#include "triangulation.h"
float alfa_old;
float beta_old;
float gamma_old;
float height;
float median;
float miu;
float alfa_new;
float beta_new;
float gamma_new;

float temp;
float temp1;
float temp3;
float temp4;
float temp5;
float ipotenuse;

static float calculate_height_triangle(float a, float b, float c){
    return (sqrt((pow((pow(a,2) + pow(b,2) + pow(c,2)),2)) - (2 * (pow(a,4) + pow(b,4) + pow(c,4)))))/(2 * a);
}

static float calculate_hypotenuse(float b, float h){
  return sqrt(pow(b,2) + pow(h,2));
}

static float calculate_median(float a, float b, float c){
    return 0.5 * (sqrt((2 * (pow(b,2) + pow(c,2))) - pow(a,2)));
}
                  
static float calculate_miu(float mediana, float lato, float old_angolo){
    return asin((lato/mediana) * sin(old_angolo * PI/180.0)) * 180.0/PI;
}

/*
  a = base,             alfa = angle opposite
  b = left side         beta = angle opposite
  c= right side         gamma = angle opposite
*/
float calculate_angle_rotation(float a, float b, float c){
    alfa_old = acos(((pow(b,2) + pow(c,2) - pow(a,2))/ (2.0*b*c))) * 180.0 / PI;
    beta_old = acos((pow(c,2) + pow(a,2) - pow(b,2))/(2.0*c*a)) * 180.0 / PI;
    gamma_old = 180 - alfa_old - beta_old;
    
    //height = calculate_height_triangle(a, b, c);
    median = calculate_median(a, b, c);
    //ipotenuse = calculate_hypotenuse(a/2.0,height);
    
    if(b - c > 0){
        miu = calculate_miu(median, c, beta_old);
    }
    else{
         miu = calculate_miu(median, b, gamma_old);
    }
    
    return 90 - miu;
    
    /*
    gamma_new = 90.0;
    beta_new = acos((pow(calculate_hypotenuse(a/2.0,height), 2) + pow(a/2,2) - pow((height),2))/(2.0 * (a/2) * calculate_hypotenuse(a/2.0,height))) * 180.0 / PI;
    alfa_new = 180 - gamma_new - beta_new;
    
    if(b - c > 0){      //clockwise
        return 90 - miu;
    }
    else{               //counterclockwise
        return (-90 - miu);
    }
    */
}

