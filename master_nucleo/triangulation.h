#ifndef _TRIANG_H
#define _TRING_H

#define PI 3.14159265

#include <math.h>

/*
*/
static float calculate_height_triangle(float a, float b, float c);

/*
*/
static float calculate_hypotenuse(float b, float h);

/*
*/
static float calculate_median(float a, float b, float c);

/*
*/
float calculate_miu(float mediana, float lato, float old_angolo);

/*
  a = base,             alfa = angle opposite
  b = left side         beta = angle opposite
  c= right side         gamma = angle opposite
*/
float calculate_angle_rotation(float a, float b, float c);

#endif