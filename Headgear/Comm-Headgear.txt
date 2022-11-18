#include <Arduino.h>
#include <Wire.h>
#include <TFLI2C.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include "RF24.h"
#include "nRF24L01.h"
#include "SPI.h"

#define leftTFL 0x12 // I2C address of left TF-Luna
#define centerTFL 0x11 // I2C address of center TF-Luna
#define rightTFL 0x10 // I2C address of right TF-Luna

#define leftBuz 7 // pin for left-side buzzer
#define centerBuz 8 // pin for center buzzer
#define rightBuz 9 // pin for right buzzer

#define leftVib 4 // pin for left-side vibration motor
#define centerVib 5 // pin for center vibration motor
#define rightVib 6 // pin for right vibration motor

// Vibration motor power adjustments if needed (don't go above 1)
#define leftVibMod 0.35 // adjust left vibrator power
#define centerVibMod 0.55 // adjust center vibrator power
#define rightVibMod 1 // adjust right vibrator power
#define maxBuzz 10000
#define minBuzz 100
#define buzMap(n) map(n, 0, 255, minBuzz, maxBuzz)

// Focus on distance(s) that are much closer than the other distance(s)
#define closeFactor 0.5
#define maxClose maxDistance / 4
#define singleClose(dist, distA, distB) ((dist < maxClose) && (dist < distA * closeFactor) && (dist < distB * closeFactor))
#define singleFar(dist, distA, distB) ((dist > maxClose) && (dist * closeFactor > distA) && (dist * closeFactor > distB))

// Bucketing
#define maxDistance 600 // After maxDistance (cm), no buzzer/vibration output
#define scalar 0.8 // Outermost bucket = 20%, 2nd is outer 20% of remaining, etc.
#define numBuckets 16 // Number of buckets to categorize distance into
#define bucketMap(dist) map(bucketize(dist), 15, 0, 0, 255)

// Photocell
#define photocellPin A1 // pin for photocell sensor
#define minBrightness 10
#define maxBrightness 1000
#define photocellMap(brightness) map(brightness, maxBrightness, minBrightness, 0, 255)

// Compass
#define compassAddr 0x20
#define northBound 10 // how many degrees off from north still counts

// Communication
#define CE_PIN 9
#define CSN_PIN 10
#define commCycleListenDelay 50
#define commCycleListenTime 100 // 50 + listenDelay
#define commCycleSendTime 150 // 50 + listenTime + listenDelay

#define activeModeSize 4
#define actuatorModeSize 4
#define validActiveMode(n) (n >= 0 && n < activeModeSize)
#define validActuatorMode(n) (n >= 0 && n < actuatorModeSize)

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

struct sensorPayload {
  uint8_t activeMode;
  uint8_t actuatorMode;
  int16_t data1;
  int16_t optData2; // for center TFL, when in distance mode
  int16_t optData3; // for right TFL, when in distance mode
}outPayload;

struct terminalRequestPayload {
  uint8_t activeMode; // set/update the active mode
  uint8_t actuatorMode; // set/update the actuator mode
}inPayload;

uint64_t currentDataCycle, startDataTime, currentCommCycle, startListeningTime;
sensors_event_t compassData;
int16_t distL, distC, distR, luxBrightness, compassHeading;
bool isListening;

const byte headToWristAddr[6] = "00001";
const byte wristToHeadAddr[6] = "00002";

TFLI2C tfl;
ActiveMode activeMode;
ActuatorMode actuatorMode;
actuatorSet leftSet, centerSet, rightSet;
RF24 radio(CE_PIN, CSN_PIN);
Adafruit_LSM303_Mag_Unified mag(12345);

void setup() {
  distL = distC = distR = maxDistance;
  luxBrightness = compassHeading = 0;
  startDataTime = 0;

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
  pinMode(rightBuz, OUTPUT);
  pinMode(photocell, INPUT);

  Wire.begin();

  radio.begin();
  radio.setAutoAck(false); // append Ack packet
  radio.setDataRate(RF24_2MBPS); // transmission rate
  radio.setPALevel(RF24_PA_MAX); // distance and energy consump.
  radio.openWritingPipe(headToWristAddr);
  radio.openReadingPipe(0, wristToHeadAddr);

  isListening = true;
  radio.startListening();
  startListeningTime = millis();

  // while(!mag.begin()) ; // wait for mag to come online
  // mag.enableAutoRange(true);
}

void loop() {
  communicate();

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

void communicate() {
  currentCommCycle = millis() - startListeningTime;

  if (currentCommCycle < commCycleListenDelay) ;
    // Delay so radio can start listening properly
  else if (currentCommCycle < commCycleListenTime) {
    if(radio.available() > 0)
      readInPayload();
  }
  else if (currentCommCycle < commCycleSendTime) {
    if(isListening) {    
      isListening = false;
      radio.stopListening();
    } else
      sendOutPayload();
  }
  else {
    isListening = true;
    radio.startListening();
    startListeningTime = millis();
  }
}

void readInPayload() {
  radio.read(&inPayload, sizeof(inPayload));

  if(validActiveMode(inPayload.activeMode))
    activeMode = inPayload.activeMode;
  if(validActuatorMode(inPayload.actuatorMode))
    actuatorMode = inPayload.actuatorMode;
}

void sendOutPayload() {
  if (activeMode == off)
    return;

  outPayload.activeMode = activeMode;
  outPayload.actuatorMode = actuatorMode;

  switch(activeMode) {
    case distance:
      outPayload.data1 = distL;
      outPayload.optData2 = distC;
      outPayload.optData3 = distR;
      break;
    case photocell:
      outPayload.data1 = luxBrightness;
      break;
    case compass:
      outPayload.data1 = compassHeading;
      break;
  }
  
  radio.write(&outPayload, sizeof(outPayload));
}

void runDistance() {
  currentDataCycle = millis() - startDataTime;

  // Collect distance data only from whoever's turn it is
  if (currentDataCycle > 150) {
    startDataTime = currentDataCycle;

  } else if (currentDataCycle > 100) {
    if(tfl.getData(distL, leftTFL)) 
      actuatorOutput(bucketMap(distL), leftSet);   

  } else if (currentDataCycle > 50) {
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
  // If two are way closer than the last one, focus on them exclusively
  else if (singleFar(distL, distC, distR)) {
    actuatorOutput(0, leftSet);
  }
  else if (singleFar(distC, distL, distR)) {
    actuatorOutput(0, centerSet);
  }
  else if (singleFar(distR, distL, distC)) {
    actuatorOutput(0, rightSet);
  }
}

void runPhotocell() {
  luxBrightness = photocellMap(analogRead(photocellPin)); 
  actuatorOutput(0, leftSet);
  actuatorOutput(luxBrightness, centerSet);
  actuatorOutput(0, rightSet);
}

void runCompass() {
  mag.getEvent(&compassData);
  compassHeading = round(atan2(compassData.magnetic.y, compassData.magnetic.x) * 180 / PI);

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
      tone(actSet.buz, buzMap(output));
      break;
    case all:
      analogWrite(actSet.vib, (uint8_t)(actSet.vibModifier * output));
      tone(actSet.buz, buzMap(output));
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