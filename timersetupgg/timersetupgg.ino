#include <Timer5.h>

volatile int counter = 0;

int f = 50;
int sig[1000];
float fs = 500.0;
float t;

void setup(){
   Serial.begin(9600);
   pinMode(10, OUTPUT);

   MyTimer5.start();
   MyTimer5.begin(500);
   MyTimer5.attachInterrupt(CounterIncrease);

   for(int i = 0; i <= 500; i++){
    t = (float)i/fs;
    sig[i] = (int)1023 * (1/2 * (sin(2*PI*f*t)+1));
    }
  }

void counterIncrease
