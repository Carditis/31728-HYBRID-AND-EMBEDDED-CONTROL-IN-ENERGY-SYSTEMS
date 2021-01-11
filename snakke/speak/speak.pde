
import processing.net.*;
import websockets.*;
import processing.serial.*; 

Serial myPort;

WebsocketServer socket;

void setup() {
  socket = new WebsocketServer(this, 1337, "/p5websocket");
   
  println("Available sreial ports:"); 
  println(Serial.list());
  
  myPort = new Serial(this, Serial.list()[0], 9600); 
}

void draw() {
  background(0);
  //myPort.write(65);
}

void webSocketServerEvent(String msg){
 println(msg);
 myPort.write(msg);
}
