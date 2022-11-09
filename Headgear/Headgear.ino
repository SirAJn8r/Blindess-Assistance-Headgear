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
#define singleClose(dist, distA, distB) ((dist < distA * 0.5) && (dist < distB * 0.5))

enum ActiveMode{
  off = 0,
  distance,
  photocell,
  compass
};

enum ActuatorMode {
  none = 0,
  vibration,
  buzzer,
  all,
};

// Stuff for output to a specific Vibration Motor/Buzzer pair
typedef struct{
  uint8_t output;
  uint8_t vib;
  uint8_t buz;
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

  //Serial.begin(9600);
  Wire.begin();

  pinMode(leftVib, OUTPUT);
  pinMode(centerVib, OUTPUT);
  pinMode(rightVib, OUTPUT);

  //pinMode(LED_BUILTIN, OUTPUT);
}


void loop()
{
  unsigned long millisS = millis();

  // Collect distance data only from whoever's turn it is
  if (millisS - getDataTimer > 99){
    getDataTimer = millisS;
  } else if (millisS - getDataTimer > 66){
    if (tfl.getData(tfDist, leftTFL)) {
      distL = tfDist;
      //Serial.print("  left: ");
      //Serial.print(tfDist);
    }
  } else if (millisS - getDataTimer > 33){
    if (tfl.getData(tfDist, centerTFL)) {
      distC = tfDist;
      //Serial.print("\tcenter: ");
      //Serial.print(tfDist);
    }
  } else {
    if (tfl.getData(tfDist, rightTFL)) {
      distR = tfDist;
      //Serial.print("\tright: ");
      //Serial.print(tfDist);
    }
  }

  // If one is much closer than the other 2, focus on that distance sensor
  bool foundClosePoint = false;
  if (singleClose(distL, distC, distR)) { // left is closest
    analogWrite(leftVib, map(bucketize(distL), 0, 15, 0, 255));
    digitalWrite(centerVib, LOW);
    digitalWrite(rightVib, LOW);
    foundClosePoint = true;
  } else if (singleClose(distC, distL, distR)) { // center is closest
    digitalWrite(leftVib, LOW);
    analogWrite(centerVib, map(bucketize(distC), 0, 15, 0, 255));
    digitalWrite(rightVib, LOW);
    foundClosePoint = true;
  } else if (singleClose(distR, distL, distC)) { // right is closest
    digitalWrite(leftVib, LOW);
    digitalWrite(centerVib, LOW);
    analogWrite(rightVib, map(bucketize(distR), 0, 15, 0, 255));
    foundClosePoint = true;
  }
  /*
  if (!foundClosePoint){
    int distAvg = (distL + distC + distR) / 3;
    int distAUL = distAvg * (distAvg * 0.2);
    if (distL < distAUL){
      analogWrite(leftVib, map(bucketize(distL), 0, 15, 0, 255)));
      foundClosePoint = true;
    } else {
      digitalWrite(leftVib, LOW);
    }
    if (distC < distAUL){
      analogWrite(centerVib, map(bucketize(distC), 0, 15, 0, 255)));
      foundClosePoint = true;
    } else {
      digitalWrite(centerVib, LOW);
    }
    if (distR < distAUL){
      analogWrite(rightVib, map(bucketize(distR), 0, 15, 0, 255)));
      foundClosePoint = true;
    } else {
      digitalWrite(rightVib, LOW);
    }
  }
  if (!foundClosePoint){
    analogWrite(leftVib, map(bucketize(distL), 0, 15, 0, 255));
    analogWrite(centerVib, map(bucketize(distC), 0, 15, 0, 255));
    analogWrite(rightVib, map(bucketize(distR), 0, 15, 0, 255));
  }*/
  if (!foundClosePoint){
    leftSet.output = distL;
    centerSet.output = distC;
    rightSet.output = distR;
    actuateOutput(leftSet);
    actuateOutput(centerSet);
    actuateOutput(rightSet);
  }
}

void actuateOutput(actuatorSet actSet) {
  switch(actuatorMode) {
    case none:
      break;
    case vibration:
      analogWrite(actSet.vib, map(bucketize(actSet.output), 0, 15, 0, 255));
      break;
    case buzzer:
      analogWrite(actSet.buz, map(bucketize(actSet.output), 0, 15, 0, 255));
      break;
    case all:
      analogWrite(actSet.vib, map(bucketize(actSet.output), 0, 15, 0, 255));
      analogWrite(actSet.buz, map(bucketize(actSet.output), 0, 15, 0, 255));
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