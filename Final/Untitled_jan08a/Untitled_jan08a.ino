#include "arduino_secrets.h"
#include <Timer5.h>
#include <LiquidCrystal.h>
#include "thingProperties.h"


LiquidCrystal lcd(7, 6, 5, 4, 3, 2); //pins connected to LCD

//variables for calculating the RMS voltage
float squaredVoltage = 0.0;
float voltageSummed = 0.0;
float RMSVoltage = 0.0;
int RMSCounter = 0;

//main couter and mesuament variable
volatile int counter = 0;
volatile int mess1 = 0;

//PWM Charging Signal Variables
float k = 0.002702702702702741;       //Droop constant (Given in slides)
float f0 = 49.883783783783784;        //Frequency intercept (Given in slides)
float a1 = 0.6133333333333333;        //PWM slop 1 (46/75)
float b1 = -0.13333333333332575;      //PWM intercept 1 (6-(10*a1))
float a2 = 2.5;                       //PWM slope 2 (27.5/11)
float b2 = -160.0;                    //PWM intercept 2 (52,5-(85*a2))
float PWMSignal;


//LEDs
int chargingLED = 0;
int dechargingLED = 1;

//lowpass and zerocross variables
float currentY = 0.0;
float oldY = 0.0;
float cutoff = 50.0;
float RC = 1.0 / (2.0 * PI*cutoff);
float dt = 1.0 / 10000.0;
float alpha = dt / (dt + RC);
float minusAlpha = 1 - alpha;
float crossOffPoint = 1023.0 / 2; //
volatile int zeroCrossCounter = 0;

//frequnce variables
int zeroCrossInterval = 100;
float scalingFactor = 1.092; //used for stabelizing the freq to its know value
float averageIntervalTime = 0.0;
float freq = 0.0;


void setup() {
  Serial.begin(115200);
  //setting up the LEDs
  pinMode(chargingLED, OUTPUT);
  pinMode(dechargingLED, OUTPUT);

  // Defined in thingProperties.h
  initProperties();

  // Connect to Arduino IoT Cloud
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);

  //ready the LCD screen
  lcd.begin(16, 2);
  lcd.clear();
  lcd.print("Freq: ");

  AdcBooster();

  analogReadResolution(10); //readings will now be done in 10 bits
  analogWriteResolution(10); //same

  MyTimer5.begin(10000); //the timer will interrupt the system 10000 per seacond. This is made possible with the Adcbooster
  MyTimer5.attachInterrupt(takingMeasurements);
}

void takingMeasurements() {
  //for cheking if any interrups are skipped, the freq meassured from this, kan be used to determen if the interrupt runs correctly
  //digitalWrite(interruptCheckPin, HIGH);
  //digitalWrite(interruptCheckPin, LOW);

  mess1 = analogRead(A2); //taking the anlog meassument

  currentY = alpha * mess1 + (minusAlpha) * oldY;  //lowpassing here
  //analogWrite(A0, currentY);

  //zero cross gets cheked for here
  if (oldY > crossOffPoint and currentY <= crossOffPoint) {
    zeroCrossCounter++;
  }
  oldY = currentY;
  counter++;
}
void loop() {
  //updating the cloud too often, result in a slow running program. This if statement makes sure that does not happen
  if (counter > 20000) {
    ArduinoCloud.update();
  }

  //only calculating the freq after a surtain amount of zero crosses, to metigate errors
  if (zeroCrossCounter >= zeroCrossInterval) {
    freq = freqCalculator();
    lcd.setCursor(7, 0);
    lcd.print(freq, 3);
    frequency = freq;
  }
  //visually showing if the frequency is in the correct interval, using LEDs on the breadbord
  if (freq < 49.9) {
    digitalWrite(chargingLED, LOW);
    digitalWrite(dechargingLED, HIGH);
  } else if (freq > 50.1) {
    digitalWrite(chargingLED, HIGH);
    digitalWrite(dechargingLED, LOW);
  } else {
    digitalWrite(chargingLED, LOW);
    digitalWrite(dechargingLED, LOW);
  }

  //RMS Voltage calculations
  squaredVoltage = pow(((mess1 - 511.0) / 1023.0) * 3.3, 2);
  voltageSummed = voltageSummed + squaredVoltage;
  RMSCounter++;

  if (RMSCounter >= 1000) {
    RMSVoltage = (sqrt((voltageSummed / RMSCounter))) * 240 * 0.85;
    RMSCounter = 0;
    voltageSummed = 0.0;
    lcd.setCursor(0, 1);
    lcd.print("RMS V:");
    lcd.setCursor(7, 1);
    lcd.print(RMSVoltage);
    rMSVALUE = RMSVoltage;
  }

  PWMChargingAmount(); //this function calculates the PWM signal

  //here we write it to the ociliscope, to confirm its square wave pattern
  analogWrite(A3, (PWMSignal / 100) * 1023);
}
//calculating the frequancy based on the amount of zero crosses and time gone by
float freqCalculator() {
  float tempFreq = (zeroCrossInterval / (counter * 0.0001)) * scalingFactor;
  zeroCrossCounter = 0;
  counter = 0;
  return tempFreq;
}

void PWMChargingAmount() {
  //calculate the allowed charging amp from the network frequency, or set it to the slider
  float chargingAmp;
  if (manualOverWrite == 1) {
    chargingAmp = manualChargingAmp; //just return the manual slider value, it is predefined to only go from 6 to 80
  } else {
    chargingAmp = (freq - f0) / k;
  }
  //Setting the boundaries for the chaging current
  if (chargingAmp >= 80.0) {
    chargingAmp = 80.0;
  } else if (chargingAmp <= 6.0) {
    chargingAmp = 0.0;
  }
  //calculating the corresponding PWM signal, from the slopes in IEC61851
  if (chargingAmp == 0.0) {
    PWMSignal = 0.0;
  } else if (chargingAmp < 52.0 and chargingAmp > 6) {
    PWMSignal = (((chargingAmp - b1) / a1));
  } else if (chargingAmp > 52.5 and chargingAmp <= 80.0) {
    PWMSignal = (((chargingAmp - b2) / a2));
  } else if (chargingAmp >= 52.0 and chargingAmp < 52.5) {
    PWMSignal = 85 ;
  }
  currentChargingAmp = "Charge current: " + String(chargingAmp) + "   PWM Signal: " + String(PWMSignal); //this will get displayed on the dashboard
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
}
void onManualOverWriteChange() {
  // Do something
}

void onManualChargingAmpChange() {
  // Do something
}
