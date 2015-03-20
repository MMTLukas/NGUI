/* Ping))) Sensor
 
 This sketch reads a PING))) ultrasonic rangefinder and returns the
 distance to the closest object in range. To do this, it sends a pulse
 to the sensor to initiate a reading, then listens for a pulse 
 to return.  The length of the returning pulse is proportional to 
 the distance of the object from the sensor.
 
 The circuit:
 	* +V connection of the PING))) attached to +5V
 	* GND connection of the PING))) attached to ground
 	* SIG connection of the PING))) attached to digital pin 7
 */
 
#include <Adafruit_NeoPixel.h>
#include <avr/power.h>

#define PIN_STRIP 6
#define PIN_SENSOR 7

Adafruit_NeoPixel strip = Adafruit_NeoPixel(16, PIN_STRIP, NEO_GRB + NEO_KHZ800);

void setup() {
  Serial.begin(9600);
  
  strip.begin();
  strip.show();
}

void loop(){
  triggerSensor();
  int distance = readSensor();
  changeColor(distance);
  printDistance(distance);
  
  delay(50);
}

long changeColor(int distance){
  int maxDistance = 200;
  int minDistance = 50;
  
  distance = max(minDistance, distance);
  distance = min(maxDistance, distance);

  int value = 255.00 / (maxDistance - minDistance) * (distance - minDistance);
  int green = value;
  int red = 255 - value;

  uint32_t c = strip.Color(red, green, 0);

  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
  }
  
  strip.show();
}

long readSensor(){
  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(PIN_SENSOR, INPUT);
  long duration = pulseIn(PIN_SENSOR, HIGH);
  
  // convert the time into a distance
  return microsecondsToCentimeters(duration);
}

void printDistance(int distance){
  Serial.print(distance);
  Serial.print("cm");
  Serial.println();
}

void triggerSensor(){
  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(PIN_SENSOR, OUTPUT);
  digitalWrite(PIN_SENSOR, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_SENSOR, HIGH);
  delayMicroseconds(5);
  digitalWrite(PIN_SENSOR, LOW);  
}

int microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}

