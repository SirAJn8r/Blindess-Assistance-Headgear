#include <Arduino.h>
#include <Wire.h>
#include <TFLI2C.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>

#define leftTFL 0x12 // I2C address of left TF-Luna
#define centerTFL 0x11 // I2C address of center TF-Luna
#define rightTFL 0x10 // I2C address of right TF-Luna

#define leftBuz 7 // pin for left-side buzzer
#define centerBuz 8 // pin for center buzzer
#define rightBuz 9 // pin for right buzzer

#define leftVib 4 // pin for left-side vibration motor
#define centerVib 5 // pin for center vibration motor
#define rightVib 6 // pin for right vibration motor

#define leftVibMod 0.35 // adjust left vibrator power
#define centerVibMod 0.55 // adjust center vibrator power
#define rightVibMod 1 // adjust right vibrator power

// Bucketing
#define maxDistance 600 // After maxDistance (cm), no buzzer/vibration output
#define scalar 0.8 // Outermost bucket = 20%, 2nd is outer 20% of remaining, etc.
#define numBuckets 16 // Number of buckets to categorize distance into
#define bucketMap(dist) map(bucketize(dist), 15, 0, 0, 255)

// Focus on 1 distance if it is much closer than either other distance
#define closeFactor 0.5
#define maxClose maxDistance / 4
#define singleClose(dist, distA, distB) ((dist < maxClose) && (dist < distA * closeFactor) && (dist < distB * closeFactor))

// Photocell
#define photocellPin A1 // pin for photocell sensor
#define minBrightness 10
#define maxBrightness 1000
#define photocellMap(brightness) map(brightness, maxBrightness, minBrightness, 0, 255)

// Compass
#define compassAddr 0x20
#define northBound 10 // how many degrees off from north still counts

enum ActiveMode{
  off = 0,
  distance,
  photocell,
  compass,
};

enum ActuatorMode {
  none = 0,
  vibration,
  buzzer,
  all,
};

// A specific Vibration Motor/Buzzer pair
typedef struct{
  uint8_t vib;
  uint8_t buz;
  float vibModifier;
}actuatorSet;

int16_t distL, distC, distR;
uint64_t getDataTimer;
sensors_event_t compassData;
double compassHeading;

TFLI2C tfl;
ActiveMode activeMode;
ActuatorMode actuatorMode;
actuatorSet leftSet, centerSet, rightSet;

Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);

void setup()
{
  distL = distC = distR = 0;
  getDataTimer = 0;

  activeMode = distance;
  actuatorMode = vibration;
  
  leftSet.vib = leftVib;
  leftSet.buz = leftBuz;
  leftSet.vibModifier = leftVibMod;
  centerSet.vib = centerVib;
  centerSet.buz = centerBuz;
  centerSet.vibModifier = centerVibMod;
  rightSet.vib = rightVib;
  rightSet.buz = rightBuz;
  rightSet.vibModifier = rightVibMod;

  pinMode(leftVib, OUTPUT);
  pinMode(centerVib, OUTPUT);
  pinMode(rightVib, OUTPUT);
  pinMode(leftBuz, OUTPUT);
  pinMode(centerBuz, OUTPUT);
  pinMode(centerBuz, OUTPUT);
  pinMode(photocell, INPUT);

  Wire.begin();

  // while(!mag.begin()) ; // wait for mag to come online
  mag.enableAutoRange(true);
}

void loop()
{
  switch(activeMode) {
    case off:
      break;
    case distance:
      runDistance();
      break;
    case photocell:
      runPhotocell();
      break;
    case compass:
      runCompass();
      break;
  }
}

void runDistance() {
  uint64_t millisS = millis();

  // Collect distance data only from whoever's turn it is
  if (millisS - getDataTimer > 99) {
    getDataTimer = millisS;

  } else if (millisS - getDataTimer > 66) {
    if(tfl.getData(distL, leftTFL)) 
      actuatorOutput(bucketMap(distL), leftSet);   

  } else if (millisS - getDataTimer > 33) {
    if(tfl.getData(distC, centerTFL))
      actuatorOutput(bucketMap(distC), centerSet);

  } else {
    if (tfl.getData(distR, rightTFL))
      actuatorOutput(bucketMap(distR), rightSet); 

  }

  // If one is way closer than the other two, focus on it exclusively
  if (singleClose(distL, distC, distR)) {
    actuatorOutput(0, centerSet);
    actuatorOutput(0, rightSet);
  } 
  else if (singleClose(distC, distL, distR)) {
    actuatorOutput(0, leftSet);
    actuatorOutput(0, rightSet);
  } 
  else if (singleClose(distR, distL, distC)) {    
    actuatorOutput(0, leftSet);
    actuatorOutput(0, centerSet);
  }
}

void runPhotocell() {
  actuatorOutput(0, leftSet);
  actuatorOutput(photocellMap(analogRead(photocellPin)), centerSet);
  actuatorOutput(0, rightSet);
}

void runCompass() {
  mag.getEvent(&compassData);
  compassHeading = atan2(compassData.magnetic.y, compassData.magnetic.x) * 180 / PI;

  if(compassHeading < northBound && compassHeading > -northBound) {
    actuatorOutput(0, leftSet);
    actuatorOutput(255, centerSet);
    actuatorOutput(0, rightSet);
  } else {
    // if north is left, then left output 255, right 0. Else, vice versa
    uint8_t northIsLeft = compassHeading < 0 ? 255 : 0;
    actuatorOutput(northIsLeft, leftSet);
    actuatorOutput(0, centerSet);
    actuatorOutput(255 - northIsLeft, rightSet);
  }
}

/* Writes the output to the actuator(s) currently chosen by the mode
 * For the vibration motors, adjusts their output by their custom modifier
 */
void actuatorOutput(uint8_t output, actuatorSet actSet) {
  switch(actuatorMode) {
    case none:
      break;
    case vibration:
      analogWrite(actSet.vib, (uint8_t)(actSet.vibModifier * output));
      break;
    case buzzer:
      analogWrite(actSet.buz, output);
      break;
    case all:
      analogWrite(actSet.vib, (uint8_t)(actSet.vibModifier * output));
      analogWrite(actSet.buz, output);
      break;
  }
}

/* Bucketing group distances 0 - maxDistance into numBuckets buckets, larger being further 
 * Find the bucket of a given distance
 */
uint8_t bucketize(int16_t dist) {
    uint8_t bucket = numBuckets - 1;
    float maxDist = maxDistance;

    while(dist < maxDist && bucket > 0) {
      bucket--;
      maxDist *= scalar;
    }

    return bucket;
}