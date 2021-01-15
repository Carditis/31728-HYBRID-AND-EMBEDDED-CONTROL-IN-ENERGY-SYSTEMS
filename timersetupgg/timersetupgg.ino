

#include <Timer5.h>

volatile int counter = 0;
int processedCounter = 0;

volatile int mess1 = 0;

float currentY = 0.0;
float oldY = 0.0;
float cutoff = 50.0;
float RC = 1.0 / (2.0 * PI*cutoff);
float dt = 1.0 / 10000.0;
float alpha = dt / dt * RC;

float crossOffPoint = 176.0; //
int zeroCrossCounter = 0;


void setup() {
  Serial.begin(9600);
  Serial.print("det virker");
  
  analogReadResolution(10); //readings will now be done in 10 bits
  analogWriteResolution(10);

  MyTimer5.begin(10000);
  MyTimer5.attachInterrupt(takingMeasurements);
}

void takingMeasurements() {

  mess1 = analogRead(A1);
  oldY = currentY;
  currentY = alpha * mess1 + (1 - alpha) * oldY;  //lowpassing here
  if (oldY < crossOffPoint and currentY >= crossOffPoint) {
    zeroCrossCounter++;
  }

  //zero cross here

  /*
    mess1 = analogRead(A1);
    for (int i = 0; i < 100; i++) {
    if (i == 99) {
      annMess1[99] = mess1;
    } else {
      annMess1[i] = annMess1[i + 1];
    }
    }
  */

  counter++;
}



void loop() {
  Serial.println("loop");
  analogWrite(A0, mess1);
  if (processedCounter < counter) {
    Serial.println(counter);
    processedCounter = counter;
    Serial.println((mess1 / 1023.0) * 3.3);

  }
  /*
    while (processedCounter < counter) {
    // mess1 = analogRead(A1);
    for (int i = 0; i < 100; i++) {
      if (i == 99) {
        annMess1[99] = mess1;
      } else {
        annMess1[i] = annMess1[i + 1];
      }
    }
    fvoltage = (mess1 / 1023.0) * 3.3;
    Serial.println(fvoltage);
    processedCounter++;
    }
  */
  /*
    noInterrupts();

    memcpy(voltage, annMess1, sizeof(annMess1[0]) * 100); //we copy a array to another array, so it does not get effected by the interrupter.
    interrupts();
*/
}
