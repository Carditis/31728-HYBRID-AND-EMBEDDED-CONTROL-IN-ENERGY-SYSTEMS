

#include <Timer5.h>


volatile int counter = 0;

int mess1 = 0;

int annMess1[100];
int voltage[100];

void setup() {
  Serial.begin(115200);
  pinMode(10, OUTPUT);

  // MyTimer5.begin(10000);
  // MyTimer5.attachInterrupt(counterIncrease);



}

void counterIncrease() {
  analogReadResolution(10); //readings will now be done in 10 bits
  for (int i = 0; i < 100; i++) {
    mess1 = analogRead(A1);
    if (i == 99) {
      annMess1[99] = mess1;
    } else {
      annMess1[i] = annMess1[i + 1];
    }
    annMess1[i] = voltage[i];
  }


  counter++;
}

void loop() {

for(int i =0; i < 100; i++){
  Serial.println(annMess1[i]);
  }


}
