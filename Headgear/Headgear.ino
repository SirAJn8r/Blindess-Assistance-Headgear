#include <Arduino.h>
#include <Wire.h>
#include <TFLI2C.h>

#define leftTFL 0x12 // I2C address of left TF-Luna
#define centerTFL 0x11 // I2C address of center TF-Luna
#define rightTFL 0x10 // I2C address of right TF-Luna

#define leftVib 4 // pin for left-side vibration motor
#define centerVib 5 // pin for center vibration motor
#define rightVib 6 // pin for right vibration motor

#define leftBuz 7 // pin for left-side buzzer
#define centerBuz 8 // pin for center buzzer
#define rightBuz 9 // pin for right buzzer

#define maxDistance 600 // After maxDistance (cm), no buzzer/vibration output
#define scalar 0.8 // Outermost bucket = 20%, 2nd is outer 20% of remaining, etc.
#define numBuckets 16 // Number of buckets to categorize distance into

// Focus on 1 distance if it is much closer than either other distance
#define singleClose(dist, distA, distB) ((dist < maxDistance / 4) && (dist < distA * 0.5) && (dist < distB * 0.5))

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
  float modifier;
}actuatorSet;

int16_t tfDist, distL, distC, distR;
unsigned long getDataTimer;

TFLI2C tfl;
ActiveMode activeMode;
ActuatorMode actuatorMode;
actuatorSet leftSet, centerSet, rightSet;

void setup()
{
  distL = distC = distR = 0;
  getDataTimer = 0;
  activeMode = distance;
  actuatorMode = vibration;
  
  leftSet.vib = leftVib;
  leftSet.buz = leftBuz;
  centerSet.vib = centerVib;
  centerSet.buz = centerBuz;
  rightSet.vib = rightVib;
  rightSet.buz = rightBuz;

  leftSet.modifier = .35;
  centerSet.modifier = .55;
  rightSet.modifier = 1;

  //Serial.begin(9600);
  Wire.begin();

  pinMode(leftVib, OUTPUT);
  pinMode(centerVib, OUTPUT);
  pinMode(rightVib, OUTPUT);
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
      break;
    case compass:
      break;
  }
}

void runDistance() {
  unsigned long millisS = millis();

  // Collect distance data only from whoever's turn it is
  if (millisS - getDataTimer > 99) {
    getDataTimer = millisS;

  } else if (millisS - getDataTimer > 66) {
    if(tfl.getData(distL, leftTFL)) 
      actuateOutput(distL, leftSet);   

  } else if (millisS - getDataTimer > 33) {
    if(tfl.getData(distC, centerTFL))
      actuateOutput(distC, centerSet);

  } else {
    if (tfl.getData(distR, rightTFL))
      actuateOutput(distR, rightSet); 

  }

  // If one is way closer than the other two, focus on it exclusively
  if (singleClose(distL, distC, distR)) {
    actuateOutput(0, centerSet);
    actuateOutput(0, rightSet);
  } 
  else if (singleClose(distC, distL, distR)) {
    actuateOutput(0, leftSet);
    actuateOutput(0, rightSet);
  } 
  else if (singleClose(distR, distL, distC)) {    
    actuateOutput(0, leftSet);
    actuateOutput(0, centerSet);
  }
}

void actuateOutput(uint8_t output, actuatorSet actSet) {
  switch(actuatorMode) {
    case none:
      break;
    case vibration:
      analogWrite(actSet.vib, (uint8_t)(actSet.modifier * map(bucketize(output), 15, 0, 0, 255)));
      break;
    case buzzer:
      analogWrite(actSet.buz, map(bucketize(output), 15, 0, 0, 255));
      break;
    case all:
      analogWrite(actSet.vib, (uint8_t)(actSet.modifier * map(bucketize(output), 15, 0, 0, 255)));
      analogWrite(actSet.buz, map(bucketize(output), 15, 0, 0, 255));
      break;
  }
}

/* Bucketing group distances 0 - maxDistance into numBuckets buckets, larger being further 
 * Find the bucket of a given distance
 */
uint8_t bucketize(float dist) {
    uint8_t bucket = numBuckets - 1;
    float maxDist = maxDistance;

    while(dist < maxDist && bucket > 0) {
      bucket--;
      maxDist *= scalar;
    }

    return bucket;
}