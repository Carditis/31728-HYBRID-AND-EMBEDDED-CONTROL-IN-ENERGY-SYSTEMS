int redPin = 10;
int greenPin = 9;
int bluePin = 8;
String myColor;
String msg = "What Color LED? ";

void setup() {
  Serial.begin(9600);
}

void loop() {
  Serial.println(msg);
  while (Serial.available() == 0) {

  }
  myColor = Serial.readString();

  if (myColor == "red") {
    digitalWrite(redPin, HIGH);
    digitalWrite(greenPin, LOW);
    digitalWrite(bluePin, LOW);
    Serial.println("I am now red");
  } else if (myColor == "green") {
    digitalWrite(redPin, LOW);
    digitalWrite(greenPin, HIGH);
    digitalWrite(bluePin, LOW);
    Serial.println("I am now green");
  } else if (myColor == "blue") {
    digitalWrite(redPin, LOW);
    digitalWrite(greenPin, LOW);
    digitalWrite(bluePin, HIGH);
    Serial.println("I am now blue");
  } else {
    digitalWrite(redPin, HIGH);
    digitalWrite(greenPin, HIGH);
    digitalWrite(bluePin, HIGH);
  }
}
