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
#include <Servo.h> 
#include <avr/power.h>

#define PIN_STRIP 6
#define PIN_SENSOR 7
#define PIN_SERVO 8
#define NUMBER_LEDS 48
#define NUMBER_STRIPS 3

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMBER_LEDS, PIN_STRIP, NEO_GRB + NEO_KHZ800);
Servo servo;

int servoPos = 0;
int servoSteps = 6;
int servoMinAngle = 0;
int servoMaxAngle = 90;
int servoStepAngle = (servoMaxAngle-servoMinAngle)/(servoSteps-1);
int servoMSPerDegree = 5;

int distanceValues[servoSteps];

void setup() {
  for(int i=0; i<servoSteps; i++){
    distanceValues[i] = 0;
  }
  
  Serial.begin(9600);
  
  strip.begin();
  strip.setBrightness(64);
  strip.show();
  
  servo.attach(PIN_SERVO);
  servo.write(servoPos);
  delay(500);
}

void loop(){
  var i = 0;
  for(servoPos = servoMinAngle; servoPos < servoMaxAngle; servoPos += servoStepAngle){
    execute(servoPos, i);
    i+=1;
  }
  
  for(servoPos = servoMaxAngle; servoPos>servoMinAngle; servoPos-=servoStepAngle)
  {                
    execute(servoPos, i);
    i-=1;
  }
}

void execute(servoPos){
  servo.write(servoPos, i);
  
  triggerSensor();
  int distance = readSensor();
  printDistance(distance);
 
  distanceValues[i] = distance;
  visualizeCircle();
  
  delay(servoMSPerDegree*servoStepAngle);
}

void visualizeCircle() {
  float maxDistance = 50;
  float minDistance = 10;
  
  distance = (distanceValues[0] + distanceValues[1]) / 2;
  
  distance = max(minDistance, distance);
  distance = min(maxDistance, distance);
    
  int height = NUMBER_LEDS/NUMBER_STRIPS - (NUMBER_LEDS/NUMBER_STRIPS * ( (distance - minDistance) / (maxDistance - minDistance) ) );    
  
  int single_strip = NUMBER_PINS / 3;
  
  strip.setBrightness(height * 5);
  
  for(uint16_t i=0; i<single_strip/2; i++) {
    if(i <= height/3 && height/3 != 0){
      for(uint16_t j=0; j < 3; j++) {
          strip.setPixelColor(single_strip/2 + single_strip*j + i, strip.Color(255, 0, 0));
          strip.setPixelColor(single_strip/2 + single_strip*j - i -1, strip.Color(255, 0, 0));
        }
     }
    else{
      for(uint16_t j=0; j < 3; j++) {
          strip.setPixelColor(single_strip/2 + single_strip*j + i, strip.Color(0, 0, 0));
          strip.setPixelColor(single_strip/2 + single_strip*j - i -1, strip.Color(0, 0, 0));
        }
    }    
  }
  strip.show();
} 

void changeColorLinear(int distance){
  float maxDistance = 100;
  float minDistance = 10;

  uint32_t leds[strip.numPixels()];
  
  
  for(int i=0; i<strip.numPixels(); i++){
    int value = min(i*20,255);
    int green = 255-value;
    int red = value;
    leds[i] = strip.Color(red, green, 0);
  }
  
  distance = max(minDistance, distance);
  distance = min(maxDistance, distance);
  
  int height = NUMBER_PINS - (NUMBER_PINS * ( (distance - minDistance) / (maxDistance - minDistance) ) );  

  for(uint16_t i=0; i<strip.numPixels(); i++) {
    if(i <= height){
      strip.setPixelColor(i, leds[i]);
    }
    else{
      strip.setPixelColor(i, strip.Color(0,0,0));
    }    
  }
  
  strip.show();
}

long changeColorAll(int distance){
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
  Serial.print(" cm");
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


