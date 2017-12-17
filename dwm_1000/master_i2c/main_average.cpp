#include "mbed.h"
#include "average.h"
extern "C"{
  #include "triangulation.h"
  #include "buffershift.h"
}
#define DISTANCE_OF_RADIO 0.0046917639786159f
#define BASE 0.81f

typedef unsigned long long uint64;

volatile bool complete_left = false;
volatile bool complete_right = false;
volatile bool reset_tag = false;
volatile bool ir_led = false;
bool make_measure_left = true;
bool make_measure_right = false;
char _TOF[5];
uint64 _timeOfFlight = 0;
float distance_left = 0;            //from TAG1 (left)
float distance_right = 0;            //from TAG2 (right)
ExponentialMovingAverage avg(0.2);

Timer timer_reset_tag;
I2C i2c(PB_7, PB_6);            //SDA=D4, SCL=D5
Serial pc(USBTX, USBRX);        //Tx, Rx
InterruptIn int_tag1(PB_0);     //D3
InterruptIn int_tag2(PA_12);    //D2
InterruptIn int_ir_led(PF_1);   //ir_led - D8
DigitalOut dir_pin(PB_1);       //D6
DigitalOut step_pin(PF_0);     //D7

const int addr1 = 0x02;         //TAG_LEFT
const int addr2 = 0x04;         //TAG_RIGHT

int addr;

float time_elapsed = 0.0f;
float x_old = 0.0f;
float x_new = 0.0f;
float ema = 0.0f;
const float alpha = 0.1;
bool start = true;
float diff;
float ref = 0.05f;
int measure = 0;
bool compute = false;

buffer distances_left;
buffer distances_right;

void complete_measure_left()
{
    complete_left = true;
}

void complete_measure_right()
{
    complete_right = true;
}

void Array64BigEndianToInt64LittleEndian(char input[], unsigned int length, uint64 *output)
{
    for (int j = length - 1; j >= 0; j--) {
        *output = (*output << 8) + input[j];
    }
}

void rotateDeg(float deg, float wait_next_step){ 
  //pc.printf("Rotation: %f\n", deg);
  //rotate a specific number of degrees (negitive for reverse movement)
  int dir = (deg > 0)? 1:0;
  dir_pin = dir;
  
  int steps_smooth = abs(0.5 * 10)*(1/0.225); // 360 / 0.225 = 1600 steps
  int steps = abs(deg)*(1/0.225); // 360 / 0.225 = 1600 steps

  for(;(steps - steps_smooth) != 0; steps_smooth++){
      step_pin = 1;
      wait_us(wait_next_step); 

      step_pin = 0;
      wait_us(wait_next_step);
  }
}

void read_measure(float *distance){
    memset(_TOF, 0x00, 5);
    i2c.read(addr, _TOF, 5); // request 5 bytes from slave device #1
    //pc.printf("\t");

    Array64BigEndianToInt64LittleEndian(_TOF, 5, &_timeOfFlight);
    _timeOfFlight -= 16384;  // Antenna delay
    //*distance = (double) (((unsigned int)_timeOfFlight) * DISTANCE_OF_RADIO);
    //pc.printf("\%f\n",distance);
}

int i = 0;
void read_measure_average(buffer * coda){
    memset(_TOF, 0x00, 5);
    i2c.read(addr, _TOF, 5); // request 5 bytes from slave device #1
    //pc.printf("\t");

    Array64BigEndianToInt64LittleEndian(_TOF, 5, &_timeOfFlight);
    _timeOfFlight -= 16384;  // Antenna delay

    float distance = (double) (((unsigned int)_timeOfFlight) * DISTANCE_OF_RADIO);
    
    Push(coda, distance);
    
    //pc.printf("\%f\n",*distance);
    //distances[i] = *distance;
    i++;
    if(compute){
      i = 0;
      compute = false;
      //*distance = (distances[0] + distances[1] + distances[2]) / 3;
      //pc.printf("Media: %f \n", *distance);
    }
    //pc.printf("\%f\n",distance);
}

void MakeAverageDistances(buffer * coda, float * distance){
  boolean end = 0;
  double temp;
  while(!end)
    temp += ReadValues(coda, &end);
  
  *distance = temp / 16;
}

void handle_reset_tags(){
  reset_tag = true;
}

void handle_ir_led(){
  ir_led = true;
}

void setup_initial_position(){
  
  while(!ir_led){
      rotateDeg(5*10, 300);
  }
  
}

int main()
{
    char cmd[1] = {0x01};
    int_tag1.rise(&complete_measure_left);  // attach the address of the flip function to the rising edge
    int_tag2.rise(&complete_measure_right);
    int_ir_led.fall(&handle_ir_led);
    MakeNullBuffer(&distances_left);
    MakeNullBuffer(&distances_right);
    
    /*
    for(int i = 0; i < 1000;){
      rotateDeg((5)*10, 400);
      wait(1);
    }
    */
    
    setup_initial_position();
    
    wait(2);
    while(1) {
        if (make_measure_left) {
            //pc.printf("Make measure1 \t");
            make_measure_left = false;
            addr = addr1;
            i2c.write(addr1, cmd, 1);
            timer_reset_tag.reset();
            timer_reset_tag.start();
        }

        if (complete_left) {
            complete_left = false;
            timer_reset_tag.stop();
            //pc.printf("Measure1\t");
            make_measure_right = true;
            read_measure(&distance_left);
            ///read_measure_average(&distances_left);
            pc.printf("%f \t",distance_left);
        }
        
        if(make_measure_right) {
            make_measure_right = false;
            addr = addr2;
            i2c.write(addr2, cmd, 1);
            timer_reset_tag.reset();
            timer_reset_tag.start();
        }
    
        if(complete_right){
            complete_right = false;
            timer_reset_tag.stop();
            //pc.printf("Measure2\t");
            make_measure_left = true;
            read_measure(&distance_right);
            //read_measure_average(&distances_right);
            pc.printf("%f \t",distance_right);
            
            
            //if(FullBuffer(distances_left) == 1 && FullBuffer(distances_right) == 1){
              //MakeAverageDistances(&distances_left, &distance_left);
              //MakeAverageDistances(&distances_right, &distance_right);
              x_new = (pow(distance_left, 2) - pow(distance_right, 2)) / (2 * BASE);
              float y_new = 0.5 * sqrt( (-(pow(pow(distance_left,2) - pow(distance_right,2), 2)) / BASE)  + (2 * pow(distance_left,2)) + (2 * pow(distance_right,2))  - (pow(BASE, 2)));
              pc.printf("%f\n", x_new);
              pc.printf("%f\n", y_new);
              ema = (alpha * x_new) + ((1-alpha) * x_old);
               
              diff = abs(x_new - x_old);
              //if(diff > 0.02){
                if(diff < 0.05){
                  if(x_new > 0 )
                    rotateDeg((2)*10, 400);
                  else
                    rotateDeg(-2*10, 400);
                }
                else if(diff < 0.10){
                  if(x_new > 0 )
                    rotateDeg((4)*10, 400);
                  else
                    rotateDeg(-4*10, 400);
                }
                else if(diff < 0.15){
                  if(x_new > 0 )
                    rotateDeg((6)*10, 400);
                  else
                    rotateDeg(-6*10, 400);
                }
                else if(diff < 0.20){
                  if(x_new > 0 )
                    rotateDeg((8)*10, 400);
                  else
                    rotateDeg(-8*10, 400);
                }
                else if(diff < 0.25){
                  if(x_new > 0 )
                    rotateDeg((10)*10, 300);
                  else
                    rotateDeg(-10*10, 300);
                }
                else if(diff < 0.30){
                  if(x_new > 0 )
                    rotateDeg((12)*10, 300);
                  else
                    rotateDeg(-12*10, 300);
                }
              //}
              x_old = x_new;
              //pc.printf("\tGradi: %f\n", avg.average(calculate_angle_rotation(0.3, distance_left, distance_right)));
              //pc.printf("\tGradi: %f\n", calculate_angle_rotation(0.26, distance_left, distance_right));
              //pc.printf("Gradi: %f\n", calculate_angle_rotation(0.3, 1.54, 1.44));
            //}
        }
        
        time_elapsed = timer_reset_tag.read();
        
        if(time_elapsed >= 0.125)
        {
            //pc.printf("RESET!!. The time taken was %f seconds\n", time_elapsed);
            /* clear the I2C buffer if it is not empty */
            i2c.read(0);
            //read_measure(&distance_left);
            //read_measure(&distance_right);
            
            make_measure_left = true;
            make_measure_right = false;
            complete_left = false;
            complete_right = false;
            //timer_reset_tag.stop();
        }        
    }

}