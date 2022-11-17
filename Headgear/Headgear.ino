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

#define leftVib 3 // pin for left-side vibration motor
#define centerVib 4 // pin for center vibration motor
#define rightVib 5 // pin for right vibration motor

#define leftBuz 6 // pin for left-side buzzer
#define centerBuz 7 // pin for center buzzer
#define rightBuz 8 // pin for right buzzer

#define CE_PIN 9
#define CSN_PIN 10

// Vibration motor power adjustments if needed (don't go above 1)
#define leftVibMod 1 // adjust left vibrator power
#define centerVibMod 1 // adjust center vibrator power
#define rightVibMod 1 // adjust right vibrator power

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
uint16_t lux; // brightness

TFLI2C tfl;
ActiveMode activeMode;
ActuatorMode actuatorMode;
actuatorSet leftSet, centerSet, rightSet;

Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);

//---------- for comms below-----------------

#define commCycleListenDelay 50
#define commCycleListenTime 100 // 50 + listenDelay
#define commCycleSendTime 1250 // 150 + listenTime + listenDelay

RF24 radio(CE_PIN, CSN_PIN);
const byte headToWristAddr[6] = "00001";
const byte wristToHeadAddr[6] = "00002";

struct sensorPayload {
  float sensorData;
  float data2;
  float data3;
  float data4;
  float data5;
  uint8_t sensorNumber; //0-2 = left-right LiDAR, 3 = photocell, 4 = compass

  // For ack'ing
  uint8_t activeMode;
  uint8_t actuatorMode;
  bool isPeriodical;
};

struct terminalRequestPayload {
  uint8_t activeMode; // set/update the active mode
  uint8_t actuatorMode; // set/update the actuator mode
  bool isPeriodical; // true = periodical, false = on-demand
};

sensorPayload outPayload;
terminalRequestPayload inPayload;
uint64_t startListeningTime, currentCycleTime;
bool readNotWrite;
uint32_t sendCount;

float data[5] = {0,0,0,0,0}; //0-2 = left-right LiDAR, 3 = photocell, 4 = compass

void setup()
{
  
  Serial.begin(9600);
  distL = distC = distR = maxDistance;
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
  pinMode(rightBuz, OUTPUT);
  pinMode(photocell, INPUT);

  Wire.begin();

  // while(!mag.begin()) ; // wait for mag to come online
  // mag.enableAutoRange(true);

  // comms below ----------------
  while(!radio.begin()) Serial.println("failed comms wiring");
  Serial.println("comms wiring valid");
  radio.setAutoAck(false); // append Ack packet
  radio.setDataRate(RF24_250KBPS); // transmission rate
  radio.setPALevel(RF24_PA_LOW); // distance and energy consump.

  radio.openWritingPipe(headToWristAddr);
  radio.openReadingPipe(0, wristToHeadAddr);

  readNotWrite = true;
  radio.startListening();
  startListeningTime = millis();

  outPayload.data2 = 1;
  outPayload.data3 = 2;
  outPayload.data4 = 3;
  outPayload.data5 = 4;

  pinMode(30, OUTPUT);
  pinMode(32, OUTPUT);
  pinMode(34, OUTPUT);
  digitalWrite(30, HIGH);
  
  digitalWrite(32, HIGH);
  
  digitalWrite(34, HIGH);
}

void loop()
{
  comms();
  actuate();
  data[0] = float(distL);
  data[1] = float(distC);  
  data[2] = float(distR);
  data[3] = float(compassHeading);
  data[4] = float(lux);
  Serial.print(" 1 ");
  Serial.print(data[0]);
  Serial.print(" 2 ");
  Serial.print(data[1]);
  Serial.print(" 3 ");
  Serial.print(data[2]);
  Serial.print(" 4 ");
  Serial.print(data[3]);
  Serial.print(" 5 ");
  Serial.println(data[4]);
}

void comms() {
  currentCycleTime = millis() - startListeningTime;

  if (currentCycleTime < commCycleListenDelay) {
    // wait
  }

  else if (currentCycleTime < commCycleListenTime) {
    if(radio.available() > 0) {
      radio.read(&inPayload, sizeof(inPayload));
      Serial.print("Set active mode to ");
      Serial.println(inPayload.activeMode);
      Serial.print("Set actuator mode to ");
      Serial.println(inPayload.actuatorMode);
      Serial.print("Set isPeriodical to ");
      Serial.println(inPayload.isPeriodical);
      outPayload.activeMode = inPayload.activeMode;
      outPayload.actuatorMode = inPayload.actuatorMode;
      outPayload.isPeriodical = inPayload.isPeriodical;
      sendCount = 0;
    }
  }

  else if (currentCycleTime < commCycleSendTime) {
    if(readNotWrite) {    
      readNotWrite = false;
      radio.stopListening();
    } else {
      if(outPayload.isPeriodical || sendCount < 10 || true) {
        //outPayload.sensorData = data[sendCount % 5];
        //outPayload.sensorNumber = sendCount % 5;
        outPayload.sensorData = data[0];
        outPayload.data2 = data[1];
        outPayload.data3 = data[2];
        outPayload.data4 = data[3];
        outPayload.data5 = data[4];
        radio.write(&outPayload, sizeof(outPayload));
        sendCount++;
        //Serial.print(".");
      }
    }
  }
  else {
    readNotWrite = true;
    radio.startListening();
    startListeningTime = millis();    
  }
}

void actuate(){
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
  if (millisS - getDataTimer > 150) {
    getDataTimer = millisS;

  } else if (millisS - getDataTimer > 100) {
    if(tfl.getData(distL, leftTFL)) 
      actuatorOutput(bucketMap(distL), leftSet);   

  } else if (millisS - getDataTimer > 50) {
    if(tfl.getData(distC, centerTFL))
      actuatorOutput(bucketMap(distC), centerSet);
  } 
  if (millisS - getDataTimer <= 50) {
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
  // If two are way close than the last one, focus on them exclusively
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
  //Serial.println(output);
  switch(actuatorMode) {
    case none:
      break;
    case vibration:
      analogWrite(actSet.vib, (uint8_t)(actSet.vibModifier * output));
      break;
    case buzzer:
      tone(actSet.buz, map(output, 0, 255, 100, 10000));
      break;
    case all:
      analogWrite(actSet.vib, (uint8_t)(actSet.vibModifier * output));
      tone(actSet.buz, map(output, 0, 255, 100, 10000));
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
