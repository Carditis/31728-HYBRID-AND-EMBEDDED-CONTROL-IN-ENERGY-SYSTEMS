
import processing.serial.*; 
Serial myPort;

PImage logo;

int bgcolor;


void setup(){ 
  colorMode(HSB, 255);
   
  logo = loadImage("http://arduino.cc/logo.png");
  
 
  size(500, 500);
  
  println("Available seial ports:"); 
  println(Serial.list());
  delay(10);
  myPort = new Serial(this, Serial.list()[1], 9600);
}

void draw(){
  frameRate(10);
  if (myPort.available() > 0){ 
    bgcolor = myPort.read(); 
    println(bgcolor); 
  }
  background(bgcolor, 255, 255); 
  image(logo, 0, 0);
}
