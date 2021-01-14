
#include <Timer5.h>

volatile int counter = 0;

int f = 50;
int sig[1000];
float fs = 500.0;
float t;

void setup() {
<<<<<<< Updated upstream
  Serial.begin(115200);
  pinMode(10, OUTPUT);
 
  MyTimer5.begin(10000);
=======
  Serial.begin(9600);

  MyTimer5.begin(1000);
>>>>>>> Stashed changes
  MyTimer5.attachInterrupt(counterIncrease);
  MyTimer5.start();

  //first wave yeye
  for (int i = 0; i < 1000; i++) {
    t = (float)i / fs;
    sig[i] = (0.5 * (sin(2 * PI * f * t) + 1));
  }



}

void counterIncrease() {
  if (counter < 1000){
    counter++;
    }else{
      counter = 0;
      }
}

void loop() {
  Serial.println(counter);
}
