#include <Adafruit_NeoPixel.h>
#include <Servo.h> 
#include <avr/power.h>
#include <NewPing.h>

//Max and Min value for the distance sensor
float maxDistance = 200;
float minDistance = 10;

// # Cable management sensors #
//   SUPPLY  |   S1    |   S2    |   S3
// GND | +5V | E1 | T1 | E2 | T2 | E3 | T3

//Define 3 distance sensors
NewPing sonar1(11, 12, maxDistance); // Sensor 1: trigger pin, echo pin, maximum distance in cm
NewPing sonar2(9, 10, maxDistance); // Sensor 2: trigger pin, echo pin, maximum distance in cm
NewPing sonar3(7, 8, maxDistance); // Sensor 3: trigger pin, echo pin, maximum distance in cm

#define pingSpeed 100 // Ping frequency (in milliseconds), fastest we should ping is about 35ms per sensor
unsigned long pingTimer1, pingTimer2, pingTimer3;

//Hardware constants
#define PIN_STRIP 5

#define NUMBER_LEDS 48
#define NUMBER_STRIPS 3

#define PIN_SWITCH 2

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMBER_LEDS, PIN_STRIP, NEO_GRB + NEO_KHZ800);


//Store of the measured distances of each servo step
  int distanceValues[3];
  int distanceValuesFade[3];


void setup() {  
  //Init distance store to far away
  for(int i=0; i < 3; i++){
    distanceValues[i] = maxDistance;
    distanceValuesFade[i] = maxDistance;
  }
  
  pinMode(PIN_SWITCH, INPUT_PULLUP); // set pin to input with internal pullup resistor
  
  pingTimer1 = millis() + pingSpeed; // Sensor 1 fires after 100ms (pingSpeed)
  pingTimer2 = pingTimer1 + (pingSpeed / 2); // Sensor 2 fires 50ms later
  pingTimer3 = pingTimer2 + (pingSpeed / 2); // Sensor 3 fires 50ms later
  
  //For console output
  Serial.begin(9600);
  
  //Init led strips
  strip.begin();
  strip.setBrightness(64);
  strip.show();
}
/**
 *
 *  MAIN
 *  Move servo, read sensor and write leds
 *
 */
void loop(){  
 if (millis() >= pingTimer1) {
   pingTimer1 += pingSpeed; // Make sensor 1 fire again 100ms later (pingSpeed)
   distanceValues[0] = sonar1.ping_cm();
   Serial.print("Ping 1: ");
   Serial.print(distanceValues[0]); // Convert ping time to distance and print result (0 = outside set distance range, no ping echo)
   Serial.println("cm");  
 }
 if (millis() >= pingTimer2) {
   pingTimer2 = pingTimer1 + (pingSpeed / 2); // Make sensor 2 fire again 50ms after sensor 1 fires
   distanceValues[1] = sonar2.ping_cm();
   // Both sensors pinged, process results here
   Serial.print("Ping 2: ");
   Serial.print(distanceValues[1]); // Convert ping time to distance and print result (0 = outside set distance range, no ping echo)
   Serial.println("cm");
 }
 if (millis() >= pingTimer3) {
   pingTimer3 = pingTimer2 + (pingSpeed / 2); // Make sensor 2 fire again 50ms after sensor 1 fires
   distanceValues[2] = sonar3.ping_cm();
   // All three sensors pinged, process results here
   Serial.print("Ping 3: ");
   Serial.print(distanceValues[2]); // Convert ping time to distance and print result (0 = outside set distance range, no ping echo)
   Serial.println("cm");
   
   Serial.print("PIN_SWITCH: ");
   Serial.println(digitalRead(PIN_SWITCH));
   
   for(int i=0; i < 3; i++) {
     if(distanceValues[i] < minDistance) {
      if(distanceValues[i] == 0) {
        distanceValues[i] = maxDistance; 
      }  
      else {
        distanceValues[i] = minDistance;
      }
     }
     if(distanceValues[i] < distanceValuesFade[i]) {
        distanceValuesFade[i] = distanceValues[i];
     }
     else {
      distanceValuesFade[i] = distanceValuesFade[i] + (distanceValues[i] / 6) ;
      
     }
     Serial.print("dist values FADE: ");
     Serial.println(distanceValuesFade[i]);
   }
   
   if(digitalRead(PIN_SWITCH)) {
      visualizeWithDirection();
    }
    else {
      visualizeLinear();
    }
  }
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
    //int distance = (distanceValues[i*2] + distanceValues[i*2+1]) / 2;  
    int distance = distanceValuesFade[i];
    //Serial.println(distance);
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
    //int distance = (distanceValues[i*2] + distanceValues[i*2+1]) / 2;
    int distance = distanceValuesFade[i];
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


//void printDistance(int distance){
//  Serial.print(distance);
//  Serial.print(" cm");
//  Serial.println();
//}
//
//void printDistanceValues(){
//  for(int i=0; i<servoSteps; i++){
//    Serial.print(distanceValues[i]);
//    Serial.print(" ");
//  }
//  Serial.println();
//}
 
//int microsecondsToCentimeters(long microseconds)
//{
//  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
//  // The ping travels out and back, so to find the distance of the
//  // object we take half of the distance travelled.
//  return microseconds / 29 / 2;
//}


