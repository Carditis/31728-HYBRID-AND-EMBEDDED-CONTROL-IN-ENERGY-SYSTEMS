
#include <Timer5.h>


volatile int counter = 0;

float mess1 = 0;

int annMess1[100];

void setup() {
  Serial.begin(115200);
  pinMode(10, OUTPUT);

  // MyTimer5.begin(10000);
  // MyTimer5.attachInterrupt(counterIncrease);



}

void counterIncrease() {
  analogReadResolution(10); //readings will now be done in 10 bits
  mess1 = analogRead(A1); //
  for (int i = 0; i = 100, i++) {
    if(mess1 != annMess1[i-1]){
      annMess1[i] = mess1;
      }
  }

  //


  counter++;
}

void loop() {
  analogReadResolution(10);






}
