#include "mbed.h"
#include "errormeasure.h"
#include "rotatewheel.h"

#define DISTANCE_OF_RADIO 0.0046917639786159f
#define BASE 0.81f

volatile bool complete_left = false;
volatile bool complete_right = false;
volatile bool reset_tag = false;
volatile bool ir_led = false;
bool make_measure_left = true;
bool make_measure_right = false;
char _TOF[5];
unsigned long long _timeOfFlight = 0;
float distance_left = 0;            //from TAG1 (left)
float distance_right = 0;            //from TAG2 (right)

Timer timer_reset_tag;
I2C i2c(PB_7, PB_6);            //SDA=D4, SCL=D5
Serial pc(USBTX, USBRX);        //Tx, Rx
InterruptIn int_tag1(PB_0);     //D3
InterruptIn int_tag2(PA_12);    //D2
InterruptIn int_ir_led(PF_1);   //ir_led - D8
DigitalOut dir_pin(PB_1);       //D6
DigitalOut step_pin(PF_0);      //D7

ErrorMeasure *err;
RotateWheel rot(&dir_pin, &step_pin);

const int addr1 = 0x02;         //TAG_LEFT
const int addr2 = 0x04;         //TAG_RIGHT
char cmd[1];

float x_old = 0.0f;
float x_new = 0.0f;
float y_new  = 0.0f;
float ema = 0.0f;
//const float alpha = 0.1;
int start = 1;

int addr;

void complete_measure_left()
{
    complete_left = true;
}

void complete_measure_right()
{
    complete_right = true;
}

void Array64BigEndianToInt64LittleEndian(char input[], unsigned int length, unsigned long long *output)
{
    for (int j = length - 1; j >= 0; j--) {
        *output = (*output << 8) + input[j];
    }
}

void read_measure(float *distance){
    memset(_TOF, 0x00, 5);
    i2c.read(addr, _TOF, 5); // request 5 bytes from slave device #1

    Array64BigEndianToInt64LittleEndian(_TOF, 5, &_timeOfFlight);
    _timeOfFlight -= 16384;  // Antenna delay

    *distance = (double) (((unsigned int)_timeOfFlight) * DISTANCE_OF_RADIO);
    //pc.printf("\%f\n",distance);
}


void PrintInformation(){
  pc.printf("Left: %f \t",distance_left);
  pc.printf("Right: %f\t",distance_right);
  pc.printf("X: %f\t", x_new);
  pc.printf("Y: %f\n", y_new);
}

int steps = 0;
void StateMachine(){
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
        make_measure_right = true;
        read_measure(&distance_left);
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
        make_measure_left = true;
        read_measure(&distance_right);
        
        x_new = (pow(distance_left, 2) - pow(distance_right, 2)) / (2 * BASE);
        y_new = 0.5 * sqrt( (-(pow(pow(distance_left,2) - pow(distance_right,2), 2)) / BASE)  + (2 * pow(distance_left,2)) + (2 * pow(distance_right,2))  - (pow(BASE, 2)));
        
        if(steps == 0){
          pc.printf("***********************\n");
          pc.printf("Posizione Centrale\n");
          if(err)
            delete err;
          err = new ErrorMeasure(distance_left, distance_right);
          PrintInformation();
          err->PrintPoints(&pc);
//          rot.RotateDeg(55 * 10,400);
          steps++;
        }
        else{
          if(steps == 1){
            pc.printf("Posizione Destra\n");
            PrintInformation();
            err->DeletePoints(distance_left, distance_right);
            err->PrintPoints(&pc);
//            rot.RotateDeg((-105) * 10,400);
            steps++;
          }
          else{
            steps = 0;
//            while(!ir_led){
//              rot.RotateDeg(5*10, 400);
//            }
            pc.printf("Posizione Sinistra\n");
            PrintInformation();
            err->DeletePoints(distance_left, distance_right);
            err->PrintPoints(&pc);
            pc.printf("***********************\n");
            pc.printf("\n");
            pc.printf("\n");
            pc.printf("\n");
          }   
        }
        if(ir_led == 1)
          ir_led = 0;
                       
        //rot.DoRotation(x_new, x_old);
        x_old = x_new;
    }
    
    float time_elapsed = timer_reset_tag.read();
    
    if(time_elapsed >= 0.125)
    {
        pc.printf("RESET!!. The time taken was %f seconds\n", time_elapsed);
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

void handle_reset_tags(){
  reset_tag = true;
}

void handle_ir_led(){
  ir_led = true;
}

void SetupInitialPosition(){
  
  while(!ir_led){
      rot.RotateDeg(5*10, 200);
  }
  ir_led = false;
}

int main()
{
    int_tag1.rise(&complete_measure_left);  // attach the address of the flip function to the rising edge
    int_tag2.rise(&complete_measure_right);
    int_ir_led.fall(&handle_ir_led);
    
    //SetupInitialPosition();
    
    *cmd = 'r';
    i2c.write(addr1, cmd, 1);
    i2c.write(addr2, cmd, 1);
    *cmd = 's';
    wait(2);
    while(1) {
       StateMachine();
    }
}