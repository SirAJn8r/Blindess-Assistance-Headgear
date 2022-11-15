#include "RF24.h"
#include "nRF24L01.h"
#include "SPI.h"

#define CE_PIN 9
#define CSN_PIN 10
#define commCycleListenDelay 50
#define commCycleListenTime 200 // 150 + listenDelay
#define commCycleSendTime 250 // 50 + listenTime + listenDelay

RF24 radio(CE_PIN, CSN_PIN);
const byte headToWristAddr[6] = "00001";
const byte wristToHeadAddr[6] = "00002";

struct sensorPayload {
  float sensorData;
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

sensorPayload inPayload;
terminalRequestPayload outPayload;
uint64_t startListeningTime, currentCycleTime;
bool readNotWrite;

void setup() {
  Serial.begin(9600);
  radio.begin();

  radio.setAutoAck(false); // append Ack packet
  radio.setDataRate(RF24_250KBPS); // transmission rate
  radio.setPALevel(RF24_PA_LOW); // distance and energy consump.

  radio.openWritingPipe(wristToHeadAddr);
  radio.openReadingPipe(0, headToWristAddr);

  readNotWrite = false;
  radio.stopListening();
  startListeningTime = millis() + commCycleListenTime;
}

void loop() {
  currentCycleTime = millis() - startListeningTime;

  if (currentCycleTime < commCycleListenDelay) {
    // wait
  }

  else if (currentCycleTime < commCycleListenTime) {
    if(radio.available() > 0) {
      radio.read(&inPayload, sizeof(inPayload));
      
      Serial.print("Sensor Data is ");
      Serial.println(inPayload.sensorData);
      Serial.print("Sensor Number is ");
      Serial.println(inPayload.activeMode);
      Serial.print("Active mode is ");
      Serial.println(inPayload.activeMode);
      Serial.print("Actuator mode is ");
      Serial.println(inPayload.actuatorMode);
      Serial.print("isPeriodical is ");
      Serial.println(inPayload.isPeriodical);
    }
  }

  else if (currentCycleTime < commCycleSendTime) {
    if(readNotWrite) {    
      readNotWrite = false;
      radio.stopListening();
    } else {
      outPayload.activeMode = 0;
      outPayload.actuatorMode = 1;
      outPayload.isPeriodical = true;
      radio.write(&outPayload, sizeof(outPayload));
    }
  }

  else {
    readNotWrite = true;
    radio.startListening();
    startListeningTime = millis();    
  }
}
