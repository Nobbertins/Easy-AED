#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
  #include <TFMPlus.h>
  #include <TFLI2C.h>      // TFLuna-I2C Library v.0.2.0
  #include <SoftwareSerial.h>
  #include <TinyGPSPlus.h>
// Choose two Arduino pins to use for software serial
int RXPin = 2;
int TXPin = 3;
//Default baud of NEO-6M is 9600
int GPSBaud = 9600;
// The TinyGPSPlus object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);
  TFLI2C tflI2C;
  int16_t  tfDist;    // distance in centimeters
int16_t  tfAddr = TFL_DEF_ADR;  // use this default I2C address or
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// Wiring Setup:
// Vin - 5V
// GND - GND
// SDA - A4
// SCL - A5

void setup(void) 
{
  Serial.begin(9600);
  //Serial.println("Full Sensor Test"); Serial.println("");
      Wire.begin();           // initialize Wire library
      pinMode(13, OUTPUT);
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    //Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
    delay(1000);
    
  bno.setExtCrystalUse(true);
    // Start the software serial port at the GPS's default baud
  ss.begin(GPSBaud);
}
String postData = "55.2944,29.2359 ";
bool blinking = false;
void displayInfo() {
	//Serial.print(F("Location: ")); 
	if (gps.location.isValid()) {
		// Serial.print(gps.location.lat(), 6);
		// Serial.print(F(","));
		// Serial.print(gps.location.lng(), 6);
    postData = String(gps.location.lng(), 6)+","+String(gps.location.lat(), 6)+" ";
    digitalWrite(13, blinking);
blinking = !blinking;
    //Serial.println(postData);
    // Serial.println("Coords: "+postData);
    
	} else {
		// Serial.print(F("INVALID"));
	}
}
void loop(void) 
{
    // Displays information when new sentence is available.
  	while (ss.available() > 0) {
		if (gps.encode(ss.read())) {
			displayInfo();
		}
	}
    if(postData != "0,0"){
    for(int i = 0; i < postData.length(); i++){
Serial.write(postData[i]);
    }
    }
	if (millis() > 5000 && gps.charsProcessed() < 10) {
		//Serial.println(F("No GPS detected: check wiring."));
		while(true);
	}
  
      if( tflI2C.getData( tfDist, tfAddr)) // If read okay...
    {
        // Serial.print("Dist: ");
        // Serial.println(tfDist);          // print the data...
    }
    //else tflI2C.printStatus();           // else, print error.
  sensors_event_t event; 
  bno.getEvent(&event);
  
  // Serial.print("X: ");
  // Serial.print(event.orientation.x, 4);
  // Serial.print("\tY: ");
  // Serial.print(event.orientation.y, 4);
  // Serial.print("\tZ: ");
  // Serial.print(event.orientation.z, 4);
  // Serial.println("");
  
  delay(1000);
  
}