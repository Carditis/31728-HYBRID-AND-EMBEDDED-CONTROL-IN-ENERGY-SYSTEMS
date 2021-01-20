//Libraries
#include "Timer5.h"
#include "LiquidCrystal.h"
#include "WiFi101.h"
#include "BlynkSimpleWiFiShield101.h"
#include "SPI.h"

#define BLYNK_PRINT SerialUSB

//Blynk online access
char ssid[] = "Arduino_router1";
char pass[] = "Y1_c0urse_r1";
char auth[] = "IDYjZKy6CrMyNqyya7F_kPxyLtNYBD7I";

//LCD
LiquidCrystal lcd(0, 1, 5, 4, 3, 2);

//Constants
const int analogIn = A1;            //Read from wave generator
const int analogOut = A0;
const int greenLED = 8;             //Connection to green LED
const int blueLED = 9;              //Connection to blue LED
const int redLED = 6;               //Connection to red LED
const int remotePin = 10;
const int btnPin = 13;
const int PWMOut = 7;

//Volatile variables
volatile int ADCSignal;
volatile int count = 0;
volatile int counterCross = 0;
volatile float yOld;
volatile float yNew;


//Float
float alpha = 0.03045902795; //Constant calculated from sampling interval and RC
float minusAlpha = 0.9695409720; //Constant calculated from 1-alpha, so save memory
float freq;
float Input;
float volRMS = 0.0;
float powerOut = 0.0;
float percentagePWM;
float ASet = 0.0;
float ASetMan = 0.0;
float ASetPWM = 0.0;
float k = 0.002702702702702741;       //Droop constant (Given in slides)
float f0 = 49.883783783783784;        //Frequency intercept (Given in slides)
float a1 = 0.6133333333333333;        //PWM slop 1 (46/75)
float b1 = -0.13333333333332575;      //PWM intercept 1 (6-(10*a1))
float a2 = 2.5;                       //PWM slope 2 (27.5/11)
float b2 = -160.0;                    //PWM intercept 2 (52,5-(85*a2))
float vSquared = 0.0;
float vSquaredSum = 0.0;

//Int
int remoteControl;                    //
int btnState;                         //
int btnPrevState = 0;                 //
int z;                                //
int zeroCrossNumber = 150;
int counterRMS = 0;

BLYNK_WRITE(V5) {                     //Sync slider from Blink with variable ASetMan
  int z = param.asInt();
  ASetMan = z;
}

void setup() {
  Serial.begin(115200);                     //Starts loop at a baud rate of 115200

  Blynk.begin(auth, ssid, pass);            //Connects to Blynk

  //Timer
  MyTimer5.begin(10000);                    //Starts timer at a rate of 10000 times per second  MyTimer5.attachInterrupt(interrupt);      //Attaches the rate to the function "interrupt"
  MyTimer5.attachInterrupt(interrupt);
  ADCBooster();                             //Speeds up the analogRead

  //LED's
  pinMode(redLED, OUTPUT);
  pinMode(blueLED, OUTPUT);
  pinMode(greenLED, OUTPUT);

  //Control remotely
  pinMode(btnPin, INPUT);
  pinMode(remotePin, OUTPUT);

  pinMode(PWMOut, OUTPUT);

  //LCD
  lcd.begin(16, 2);
}

//Functions defined to be used in loop later on
//Enables ADCBooster
void ADCBooster() {
  ADC->CTRLA.bit.ENABLE = 0; // Disable ADC
  while ( ADC->STATUS.bit.SYNCBUSY == 1 ); // Wait for synchronization
  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV16 | // Divide Clock by 16.
                   ADC_CTRLB_RESSEL_10BIT; // Result on 10 bits
  ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_1 | // 1 sample
                     ADC_AVGCTRL_ADJRES(0x00ul); // Adjusting result by 0
  ADC->SAMPCTRL.reg = 0x00; // Sampling Time Length = 0
  ADC->CTRLA.bit.ENABLE = 1; // Enable ADC
  while ( ADC->STATUS.bit.SYNCBUSY == 1 ); // Wait for synchronization
}


//Initialises the Low Pass Filter
float LowPassFilter(float Input, float yOld) {
  float yTemporary;                                 //Creates a temporary variable
  yTemporary = alpha * Input + minusAlpha * yOld;   //Calculates the value after the low pass filter
  return yTemporary;                                //Returns the value
}

//The timer, which runs 10000 times a second
void interrupt() {                        //INTERRUPT SERVICE ROUTINE
  analogReadResolution(10);               //Sets the read resolution to 10 bit
  ADCSignal = analogRead(A1);             //Reads the current input

  //Implementing low pass filter
  yNew = LowPassFilter(ADCSignal, yOld);  //CURRENTLY NOT WORKING WITH INPUT FROM ARDUINO


  if (yOld < 512 && yNew >= 512) {        //Checks for cross at zero (512)
    counterCross++;                       //Counts times the wave crosses 512 going upward.
  }

  yOld = yNew;                            //Stores the data so it can be used in next iteration
  count++;                                //Increases the counter
  analogWriteResolution(10);
  analogWrite(analogOut, yNew);
}


//Calcualtes frequency
int freqCalc() {
  if (counterCross >= zeroCrossNumber) { //counterCross instead of counterRMS
    freq = ((zeroCrossNumber - 1) / (count * 0.0001)) * 1.135; //9.151642719868216e-05);       //zeroCrossNumber is 50(for 50 hertz, but could be anything(every 1 sec right now)). Count is number of times interrupt is run before it crosses. One could put a scaling factor here at the end. (1.004)
    counterCross = 0;
    count = 0;
    lcd.setCursor(0, 0);
    lcd.print("Freq: ");
    lcd.print(freq, 3);
    lcd.print(" HZ");
  }
}

//Calculates vSquared, volRMS, percentagePWM & powerOUt
int calculations() {
  percentagePWM = (ASetPWM / 1023.0) * 100;                   //Calculates the percentage PWM [%]

  vSquared = pow((( (ADCSignal - 511.0) / 1023.0) * 3.3), 2); //- 511 making a new 0 line
  vSquaredSum = vSquaredSum + vSquared;
  counterRMS++;

  if (counterRMS >= 10000) {                                  // /1.017 er skalering så vi får ca vRMS fra wavegen (1.65 Vpp / sqrt(2) = 1.166)
    //volRMS = (sqrt(vSquaredSum / counterRMS));
    volRMS = (sqrt(vSquaredSum / counterRMS) * 240) * 0.856;  //Calculates RMS voltage in [V]
    counterRMS = 0;
    vSquaredSum = 0.0;
    lcd.setCursor(0, 1);
    lcd.print("V-RMS: ");
    lcd.print(volRMS, 3);
    lcd.print(" V");

    powerOut = (ASet * volRMS) / 1000;                          //Calculates the power out in [kW]
  }
}

//Controls LED based upon the frequency
int LED() {
  if (freq >= 49.975 && freq <= 50.025) {       //Normal frequency is within normal range
    digitalWrite(greenLED, HIGH);
  }
  else {
    digitalWrite(greenLED, LOW);
  }

  if (freq > 50.025) {                        //Indicates frequency is too high
    digitalWrite(redLED, HIGH);
  }
  else {
    digitalWrite(redLED, LOW);
  }

  if (freq < 49.975) {                        //Indicates frequency is too low
    digitalWrite(blueLED, HIGH);
  }
  else {
    digitalWrite(blueLED, LOW);
  }
}

//Remote control
int control() {
  if (remoteControl == HIGH) {
    digitalWrite(remotePin, HIGH);
  } else {
    digitalWrite(remotePin, LOW);
  }
}
//Toggles the switch
int toggle() {
  btnState = digitalRead(btnPin);
  if (btnPrevState == 0 && btnState == 1) {
    remoteControl = !remoteControl;
  }
  btnPrevState = btnState;
}

//Grid Services
int gridService() {
  if (remoteControl == 1) {             //(DROOP CONTROLLER WITH PWM) BETTER DESCRIPTION
    ASet = (freq - f0) / k;
  } else {                            //(DEACTIVATING GRID SERVICES)
    ASet = ASetMan;
  }
  if (ASet >= 80.0) {                  //80A is the upper limit of PWM
    ASet = 80.0;
  } else if (ASet <= 6.0) {           //6A is the lower limit
    ASet = 0.0;
  }
  if (ASet == 0) {
    ASetPWM = 0;
  } else if (ASet < 52.0 && ASet > 6.0) {     //For current on slope 1
    ASetPWM = (((ASet - b1) / a1) / 100) * 1023.0; //Calculating the PWM for slope 1
  } else if (ASet > 52.5 && ASet <= 80.0) {    //For current on slope 2
    ASetPWM = (((ASet - b2) / a2) / 100) * 1023.0; //Calcualting the PWM for slope 2
  } else if (ASet >= 52.0 && ASet < 52.5) {   //For current between slopes
    ASetPWM = 0.85 * 1023.0;                    //Calculating the PWM between slopes
  }
}

//Sends data to BLYNK app
int blynk() {
  Blynk.virtualWrite(V1, powerOut);
  Blynk.virtualWrite(V2, volRMS);
  Blynk.virtualWrite(V3, freq);
  Blynk.virtualWrite(V4, percentagePWM);
  Blynk.virtualWrite(V6, ASet);
  Blynk.virtualWrite(V7, ASetPWM);
}

void loop() {
  calculations();

  freqCalc();

  LED();

  control();

  toggle();

  gridService();
  analogWriteResolution(10);
  analogWrite(PWMOut, ASetPWM);

  if (count >= 30000) {
    blynk();
  }
}
