#include <Adafruit_NeoPixel.h>
#include <Servo.h> 
#include <avr/power.h>

//Hardware constants
#define PIN_STRIP 6
#define PIN_SENSOR 7
#define PIN_SERVO 8

#define NUMBER_LEDS 48
#define NUMBER_STRIPS 3

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMBER_LEDS, PIN_STRIP, NEO_GRB + NEO_KHZ800);
Servo servo;

//Servo variables
int servoPos = 0;
int servoSteps = 6;
int servoMinAngle = 0;
int servoMaxAngle = 90;
int servoStepAngle = (servoMaxAngle-servoMinAngle)/(servoSteps-1);
int servoMSPerDegree = 5;

//Max and Min value for the distance sensor
float maxDistance = 50;
float minDistance = 10;

//Store of the measured distances of each servo step
int distanceValues[6];

void setup() {
  //Init distance store to far away
  for(int i=0; i<servoSteps; i+=1){
    distanceValues[i] = 0;
  }
  
  //For console output
  Serial.begin(9600);
  
  //Init led strips
  strip.begin();
  strip.setBrightness(64);
  strip.show();
  
  //Set servo to init position
  //And wait for the servo reaching the position
  servo.attach(PIN_SERVO);
  servo.write(servoPos);
  delay(5*360);
}

/**
 *
 *  MAIN
 *  Move servo, read sensor and write leds
 *
 */
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

void readSensorWriteLEDs(int servoPosition, int i){
  //servo.write(servoPosition);
  
  triggerSensor();
  int distance = readSensor();;
  printDistance(distance);
 
  distanceValues[i] = distance;
  visualizeLinear();

  delay(servoMSPerDegree*servoStepAngle);
}


/**
 * VISUALIZATION TYPE 2
 *
 *    LED-STRIPS:    OBJECT: comming from right
 *                           near at the right
 *    [ ][ ][ ]              nearer at the center
 *    [ ][*][ ]      
 *    [*][x][ ]  =   [o][o][O][O][ ][ ]
 *    [x][X][ ]      
 *    [X][X][ ]      
 * 
 *    bottom = green  
 *    top = red
 **/
 
void visualizeLinear() {
  //Calculate the color and the amount of leds for every strip
  for(int i=0; i<NUMBER_STRIPS; i++){
  
    //to get the with 1,2,3 the needed indices we use i*2 and i*2+1
    //0 = 0,1 / 1 = 2,3 / 2 = 4,5   
    int distance = (distanceValues[i*2] + distanceValues[i*2+1]) / 2;
    distance = max(minDistance, distance);
    distance = min(maxDistance, distance);
    
    int heightStrip = NUMBER_LEDS/NUMBER_STRIPS;
    
    //how many of the leds of one strip should be switched on
    //more distances = less leds
    int heightToLight = heightStrip - heightStrip * ( (distance - minDistance) / (maxDistance - minDistance) );   
    
    //Calculate the color and the amount of leds for the single strip i
    for(int j=0; j<heightStrip; j++) {
      
      //Workaround for the second led strip, which is upside down
      int idx = 0;
      if(i == 1){
        idx = 2*i*heightStrip-j-1;
      }
      else{
        idx = j+(i*heightStrip);
      }
      
      //Check if the leds is one of the leds which should be switched on
      if(j <= heightToLight){          
        int delta = 255*0.33/heightStrip*j;
        
        //Most green for low j's, more red for higher j's
        strip.setPixelColor(idx, strip.Color(255*0.66 + delta, 255*0.33 - delta, 0));
      }
      else{
        strip.setPixelColor(idx, strip.Color(0, 0, 0));
      } 
    }
  }
  
  strip.show();
} 

/**
 * VISUALIZATION TYPE 1
 *
 *    LED-STRIPS:    OBJECT: comming from right
 *                           near at the right
 *    [ ][ ][ ]              nearer at the center
 *    [ ][X][ ]      
 *    [X][X][ ]  =   [o][o][O][O][ ][ ]
 *    [X][X][ ]      
 *    [ ][X][ ]      
 *    [ ][ ][ ]  
 * 
 **/
 
void visualizeWithDirection() {
  //Calculate the color and the amount of leds for every strip
  for(int i=0; i<NUMBER_STRIPS; i++){
  
    //to get the with 1,2,3 the needed indices we use i*2 and i*2+1
    //0 = 0,1 / 1 = 2,3 / 2 = 4,5   
    int distance = (distanceValues[i*2] + distanceValues[i*2+1]) / 2;
    distance = max(minDistance, distance);
    distance = min(maxDistance, distance);
    
    int heightStrip = NUMBER_LEDS/NUMBER_STRIPS;
    
    //how many of the leds of one strip should be switched on
    //more distances = less leds
    int heightToLight = heightStrip - heightStrip * ( (distance - minDistance) / (maxDistance - minDistance) );   
    
    int centerOfStrip = heightStrip/2;
    int farestLEDFromCenter = heightToLight/2;
    
    //Calculate the color and the amount of leds for the single strip i
    for(int j=0; j<heightStrip; j++) {
      /**
       *   [ ]  When the j is not over or under the heightStrip half plus/minus
       *   [X]  the height calculated with the distance than it should be switched on
       *   [X]  <- Center
       *   [X]  
       *   [ ]  OVerwise it should be off (hear the most top and bottom one)
       **/       
       
      /**
       * For 16 leds high strip, when farestLEDFromCenter = 0
       * [ ][ ][ ][ ][ ][ ][ ][ ][ ][ ][ ][ ][ ][ ][ ][ ]
       *
       * For 16 leds high strip, when farestLEDFromCenter = 3
       * [ ][ ][ ][ ][ ][x][x][x][x][x][x][ ][ ][ ][ ][ ]
       **/
      if(j < centerOfStrip+farestLEDFromCenter && j >= centerOfStrip-farestLEDFromCenter){      
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

void printDistanceValues(){
  for(int i=0; i<servoSteps; i++){
    Serial.print(distanceValues[i]);
    Serial.print(" ");
  }
  Serial.println();
}
 
int microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}


