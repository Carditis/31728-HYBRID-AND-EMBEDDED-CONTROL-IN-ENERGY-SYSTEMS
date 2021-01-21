// Code generated by Arduino IoT Cloud, DO NOT EDIT.

#include <ArduinoIoTCloud.h>
#include <Arduino_ConnectionHandler.h>


const char THING_ID[] = "44a30508-236c-42f4-a2f8-6536d5f27e3f";

const char SSID[]     = SECRET_SSID;    // Network SSID (name)
const char PASS[]     = SECRET_PASS;    // Network password (use for WPA, or use as key for WEP)

void onManualChargingAmpChange();
void onManualOverWriteChange();

float frequency;
float rMSVALUE;
String currentChargingAmp;
float manualChargingAmp;
bool manualOverWrite;

void initProperties(){

  ArduinoCloud.setThingId(THING_ID);
  ArduinoCloud.addProperty(frequency, READ, ON_CHANGE, NULL, 0.01);
  ArduinoCloud.addProperty(rMSVALUE, READ, ON_CHANGE, NULL, 0.01);
  ArduinoCloud.addProperty(manualChargingAmp, READWRITE, ON_CHANGE, onManualChargingAmpChange, 0.01);
  ArduinoCloud.addProperty(currentChargingAmp, READ, 1 * SECONDS, NULL);
  ArduinoCloud.addProperty(manualOverWrite, READWRITE, ON_CHANGE, onManualOverWriteChange);

}

WiFiConnectionHandler ArduinoIoTPreferredConnection(SSID, PASS);
