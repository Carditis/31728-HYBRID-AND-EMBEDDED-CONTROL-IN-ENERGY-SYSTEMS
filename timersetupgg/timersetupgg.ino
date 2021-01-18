

#include <Timer5.h>

#include <LiquidCrystal.h>

LiquidCrystal lcd(7, 6, 5, 4, 3, 2); //pins connected to LCD

float voltage = 0;

volatile int counter = 0;
int processedCounter = 0;

volatile int mess1 = 0;

//lowpass and zerocross variables
float currentY = 0.0;
float oldY = 0.0;
float cutoff = 50.0;
float RC = 1.0 / (2.0 * PI*cutoff);
float dt = 1.0 / 10000.0;
float alpha = dt / dt * RC;

float crossOffPoint = 1023.0/2; //
int zeroCrossCounter = 0;

//frequnce variables
unsigned long oldTime = 0;
unsigned long newTime = 0;
int zeroCrossInterval = 100;
float scaleFactor = 1.00015;
double averageIntervalTime = 0.0;


void setup() {
  Serial.begin(115200);

  AdcBooster();

  analogReadResolution(10); //readings will now be done in 10 bits
  analogWriteResolution(10);

  MyTimer5.begin(10000);
  MyTimer5.attachInterrupt(takingMeasurements);

  oldTime = millis();
}

void takingMeasurements() {

  mess1 = analogRead(A2);
  //analogWrite(A0, mess1);
  oldY = currentY;
  currentY = alpha * mess1 + (1 - alpha) * oldY;  //lowpassing here
  analogWrite(A0, currentY);

  if (oldY > crossOffPoint and currentY <= crossOffPoint) {
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

void AdcBooster()
{
  ADC->CTRLA.bit.ENABLE = 0;                     // Disable ADC
  while ( ADC->STATUS.bit.SYNCBUSY == 1 );       // Wait for synchronization
  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV64 |   // Divide Clock by 64.
                   ADC_CTRLB_RESSEL_12BIT;       // Result on 12 bits
  ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_1 |   // 1 sample
                     ADC_AVGCTRL_ADJRES(0x00ul); // Adjusting result by 0
  ADC->SAMPCTRL.reg = 0x00;                      // Sampling Time Length = 0
  ADC->CTRLA.bit.ENABLE = 1;                     // Enable ADC
  while ( ADC->STATUS.bit.SYNCBUSY == 1 );       // Wait for synchronization
} // AdcBooster



void loop() {
  //analogWrite(A0, mess1);
  //Serial.println("loop");
  if (processedCounter < counter) {
    processedCounter = counter;
    voltage = (mess1 / 1023.0) * 3.3;
    //Serial.println(mess1);
    //Serial.println(voltage);
  }
  if (zeroCrossCounter == zeroCrossInterval) {
    Serial.println(freqCalculator());
  }

  //digitalPins for writing on the LCD, and making LED turn on and off
  //need a check for if the frequency is correct.

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

double freqCalculator() {
  newTime = millis();
  averageIntervalTime = ((double)(newTime - oldTime) / (double)(zeroCrossInterval));
  oldTime = newTime;
  zeroCrossCounter = 0;
  return ((1 / averageIntervalTime) * 1000) * scaleFactor ;
}
