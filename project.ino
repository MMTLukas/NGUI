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
int servoMSPerDegree = 200;

int distanceValues[6];

void setup() {
  for(int i=0; i<servoSteps; i+=1){
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
  int i = 0;
  for(servoPos = servoMinAngle; servoPos < servoMaxAngle; servoPos += servoStepAngle){
    readSensorWriteLEDs(servoPos, i);
    i+=1;
  }
  
  for(servoPos = servoMaxAngle; servoPos>servoMinAngle; servoPos-=servoStepAngle)
  {                
    readSensorWriteLEDs(servoPos, i);
    i-=1;
  }
}

void readSensorWriteLEDs(int servoPos, int i){
  servo.write(servoPos);
  
  triggerSensor();
  int distance = readSensor();;
  printDistance(distance);
 
  distanceValues[i] = distance; 
  visualizeCircle();
  
  delay(servoMSPerDegree*servoStepAngle);
}

/**
 * VISUALIZATION TYPE 1
 *
 *    LED-STRIPS:    OBJECT: comming from right
 *                           near at the right
 *    [ ][ ][ ]              nearer at the center
 *    [ ][X][ ]      
 *    [X][X][ ]  =   [o][o][O][O][ ][ ]
 *    [ ][X][ ]      
 *    [ ][ ][ ]      
 * 
 **/
 
void visualizeCircle() {
  float maxDistance = 50;
  float minDistance = 10;
  
  //Calculate the color and the amount of leds for every strip
  for(int i=0; i<NUMBER_STRIPS; i++){
  
    //to get the with 1,2,3 the needed indices we use i*2 and i*2+1
    //0 = 0,1 / 1 = 2,3 / 2 = 4,5   
    int distance = (distanceValues[i*2] + distanceValues[i*2+1]) / 2;
    distance = max(minDistance, distance);
    distance = min(maxDistance, distance);
    
    int heightStrip = NUMBER_LEDS/NUMBER_STRIPS;
    
    //how many of the leds of one strip should be switched on
    int heightToLight = heightStrip * ( (distance - minDistance) / (maxDistance - minDistance) );   
    
    //Calculate the color and the amount of leds for the single strip i
    for(int j=0; j<heightStrip; j++) {
      int centerOfStrip = heightStrip/2;
      int farestLEDFromCenter = heightToLight/2
      
      /**
       *   [ ]  When the j is not over or under the heightStrip half plus/minus
       *   [X]  the height calculated with the distance than it should be switched on
       *   [X]  <- Center
       *   [X]  
       *   [ ]  OVerwise it should be off (hear the most top and bottom one)
       **/
       
      if(j <= centerOfStrip+farestLEDFromCenter && j >= centerOfStrip+farestLEDFromCenter){
        strip.setPixelColor(j+(i*heightStrip), strip.Color(255, 0, 0));
        strip.setPixelColor(j+(i*heightStrip), strip.Color(255, 0, 0));
      }
      else{
        strip.setPixelColor(j+(i*heightStrip), strip.Color(0, 0, 0));
        strip.setPixelColor(j+(i*heightStrip), strip.Color(0, 0, 0));
      } 
    }
  }
  
  strip.show();
} 

/** 
 *
 * DISTANCE SENSOR
 *
 **/

long readSensor(){
  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(PIN_SENSOR, INPUT);
  long duration = pulseIn(PIN_SENSOR, HIGH);
  
  // convert the time into a distance
  return microsecondsToCentimeters(duration);
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

/** 
 * 
 * HELPER FUNCTIONS
 *
 **/
 
void printDistance(int distance){
  Serial.print(distance);
  Serial.print(" cm");
  Serial.println();
}
 
int microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}


