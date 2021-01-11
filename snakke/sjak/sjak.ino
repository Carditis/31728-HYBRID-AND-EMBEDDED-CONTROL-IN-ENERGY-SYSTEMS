
#include <LiquidCrystal.h>
#include <string.h>

LiquidCrystal lcd(7, 6, 5, 4, 3, 2); //pins connected to LCD

int len = 0;

void setup()
{
  Serial.begin(9600);

  pinMode(2, INPUT);
  lcd.begin(16, 2);
  lcd.print("Ready");
}

//String msg = "";

void loop() {

  String msg = Serial.readString();
  if (msg != "") {
    lcd.clear();
    
    if (msg.length() >= 16) {
      //print first 16 chars on first line, then shift to next
      lcd.print(msg.substring(0, 16).c_str());
      lcd.setCursor(0, 1);
      lcd.print(msg.substring(16).c_str());
    } else {
      lcd.print(msg);
    }




  }
}
