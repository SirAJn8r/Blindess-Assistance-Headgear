#include <Arduino.h>
#include <Wire.h>
#include <TFLI2C.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include "RF24.h"
#include "nRF24L01.h"
#include "SPI.h"

#define _DEBUG_VALUES false

#define leftTFL 0x12 // I2C address of left TF-Luna
#define centerTFL 0x11 // I2C address of center TF-Luna
#define rightTFL 0x10 // I2C address of right TF-Luna

#define leftVib 3 // pin for left-side vibration motor
#define centerVib 4 // pin for center vibration motor
#define rightVib 5 // pin for right vibration motor

#define leftBuz 6 // pin for left-side buzzer
#define centerBuz 7 // pin for center buzzer
#define rightBuz 8 // pin for right buzzer

#define CE_PIN 9
#define CSN_PIN 10

// Buzzers
#define minBuz 100
#define maxBuz 10000
#define buzMap(val) map(val, 0, 255, minBuz, maxBuz)

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
#define minLux 0
#define maxLux 900
#define photocellMap(lux) map(lux, minLux, maxLux, 0, 255)

// Compass
#define compassAddr 0x20
#define northBound 15 // how many degrees off from north still counts
#define compassAdjust -50

//Communication
#define commCycleListenDelay 50
#define commCycleListenTime 150 // 100 + listenDelay
#define commCycleSendTime 600 // 450 + listenTime + listenDelay

enum ActiveMode{
  readAll = 0,
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
}actuatorSet;

struct sensorPayload {
  int16_t distL;
  int16_t distC;
  int16_t distR;
  int16_t compassHeading;
  int16_t lux;

  ActiveMode activeMode;
  ActuatorMode actuatorMode;
}outPayload;

struct terminalRequestPayload {
  ActiveMode activeMode; // set/update the active mode
  ActuatorMode actuatorMode; // set/update the actuator mode
}inPayload;

uint64_t currentCommCycleTime, startListeningTime, messageCountTimer, lastRecvTime;
uint32_t sentMessageCount;
sensors_event_t compassData;
int16_t distL, distC, distR, lux, compassHeading; // lux = brightness
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
  lux = compassHeading = 0;

  activeMode = readAll;
  actuatorMode = vibration;
  
  leftSet.vib = leftVib;
  leftSet.buz = leftBuz;
  centerSet.vib = centerVib;
  centerSet.buz = centerBuz;
  rightSet.vib = rightVib;
  rightSet.buz = rightBuz;

  pinMode(leftVib, OUTPUT);
  pinMode(centerVib, OUTPUT);
  pinMode(rightVib, OUTPUT);
  pinMode(leftBuz, OUTPUT);
  pinMode(centerBuz, OUTPUT);
  pinMode(rightBuz, OUTPUT);
  pinMode(photocell, INPUT);

  Serial.begin(9600);
  Wire.begin();
  while(!mag.begin()) Serial.println("Cannot connect to compass");
  while(!radio.begin()) Serial.println("Cannot connect to radio");
  Serial.println("Wire, Compass, and Radio connections made.");
 
  mag.enableAutoRange(true);

  radio.setAutoAck(true); // append Ack packet
  radio.setDataRate(RF24_2MBPS); // transmission rate
  radio.setPALevel(RF24_PA_MAX); // distance and energy consump.
  radio.openWritingPipe(headToWristAddr);
  radio.openReadingPipe(0, wristToHeadAddr);

  isListening = true;
  radio.startListening();
  startListeningTime = messageCountTimer = lastRecvTime = millis();
  sentMessageCount = 0;
}

void loop() {
  readAllSensors(); 
  communicate();

  switch(activeMode) {
    case readAll: break; // reads all anyways
    case distance: runDistance(); break;
    case photocell: runPhotocell(); break;
    case compass: runCompass(); break;
  }
  
  #if _DEBUG_VALUES
  Serial.print(" 1 ");
  Serial.print(distL);
  Serial.print(" | 2 ");
  Serial.print(distC);
  Serial.print(" | 3 ");
  Serial.print(distR);
  Serial.print(" | 4 ");
  Serial.print(compassHeading);
  Serial.print(" | 5 ");
  Serial.println(lux);
  #endif
}

void communicate() {
  currentCommCycleTime = millis() - startListeningTime;

  if (currentCommCycleTime < commCycleListenDelay) ;
    // Delay so radio can start listening properly
  else if (currentCommCycleTime < commCycleListenTime) {
    if(radio.available() > 0)
      readInPayload();
  }
  else if (currentCommCycleTime < commCycleSendTime) {
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

  // Debug info
  currentCommCycleTime = millis();
  Serial.print("Last Recv Seconds Ago = ");
  Serial.print((uint32_t)(currentCommCycleTime - lastRecvTime) / 1000);
  Serial.print("  |  Sent Per Second = ");
  Serial.println(sentMessageCount * 1000 / (uint32_t)(currentCommCycleTime - messageCountTimer));
  
  if(currentCommCycleTime - messageCountTimer > 3000) {
    sentMessageCount = 0;
    messageCountTimer = millis();
  }
}

void readInPayload() {
  radio.read(&inPayload, sizeof(inPayload));

  activeMode = inPayload.activeMode;
  actuatorMode = inPayload.actuatorMode;

  lastRecvTime = millis();

  /*
  // Debugging
  Serial.print("Set active mode to ");
  Serial.println(inPayload.activeMode);
  Serial.print("Set actuator mode to ");
  Serial.println(inPayload.actuatorMode);
  */
}

void sendOutPayload() {
  outPayload.distL = distL;
  outPayload.distC = distC;
  outPayload.distR = distR;
  outPayload.compassHeading = compassHeading;
  outPayload.lux = lux;
  
  radio.write(&outPayload, sizeof(outPayload));

  sentMessageCount++;
}

void readAllSensors() {
  tfl.getData(distL, leftTFL);   
  tfl.getData(distC, centerTFL);
  tfl.getData(distR, rightTFL);

  lux = analogRead(photocellPin);
  
  mag.getEvent(&compassData);
  compassHeading = round(atan2(compassData.magnetic.y, compassData.magnetic.x) * 180 / PI) + compassAdjust;
}

void runDistance() {
  // If one is way closer than the other two, focus on it exclusively
  if (singleClose(distL, distC, distR)) {
    actuatorOutput(bucketMap(distL), leftSet);   
    actuatorOutput(0, centerSet);
    actuatorOutput(0, rightSet);
  } 
  else if (singleClose(distC, distL, distR)) {
    actuatorOutput(0, leftSet);
    actuatorOutput(bucketMap(distC), centerSet);
    actuatorOutput(0, rightSet);
  } 
  else if (singleClose(distR, distL, distC)) {    
    actuatorOutput(0, leftSet);
    actuatorOutput(0, centerSet);
    actuatorOutput(bucketMap(distR), rightSet);
  } 
  // If two are way close than the last one, focus on them exclusively
  else if (singleFar(distL, distC, distR)) {
    actuatorOutput(0, leftSet);
    actuatorOutput(bucketMap(distC), centerSet);
    actuatorOutput(bucketMap(distR), rightSet);
  }
  else if (singleFar(distC, distL, distR)) {
    actuatorOutput(bucketMap(distL), leftSet);   
    actuatorOutput(0, centerSet);
    actuatorOutput(bucketMap(distR), rightSet);
  }
  else if (singleFar(distR, distL, distC)) {
    actuatorOutput(bucketMap(distL), leftSet);   
    actuatorOutput(bucketMap(distC), centerSet);    
    actuatorOutput(0, rightSet);
  }
  // If they're comparable 
  else {
    actuatorOutput(bucketMap(distL), leftSet);   
    actuatorOutput(bucketMap(distC), centerSet);    
    actuatorOutput(bucketMap(distR), rightSet);
  }
}

void runPhotocell() {
  int16_t tempLux;
  tempLux = lux < minLux ? minLux : lux;
  tempLux = tempLux > maxLux ? maxLux : tempLux;

  actuatorOutput(0, leftSet);
  actuatorOutput(photocellMap(lux), centerSet);
  actuatorOutput(0, rightSet);
}

void runCompass() {
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
      analogWrite(actSet.vib, output);
      break;
    case buzzer:
      tone(actSet.buz, buzMap(output));
      break;
    case all:
      analogWrite(actSet.vib, output);
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
