void setup() {
  Serial.begin(9600);
  
 
}

void loop() {
  
  Serial.write(analogRead(A1)/4);

  //this will not work, as it will send the input as a string
  //Serial.println(analogRead(A1)/4);
  delay(100);
}
