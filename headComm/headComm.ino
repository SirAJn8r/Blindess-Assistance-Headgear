#include "RF24.h"
#include "nRF24L01.h"
#include "SPI.h"

#define CE_PIN 9
#define CSN_PIN 10
#define commCycleListenDelay 50
#define commCycleListenTime 100 // 50 + listenDelay
#define commCycleSendTime 150 // 50 + listenTime + listenDelay

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

sensorPayload outPayload;
terminalRequestPayload inPayload;
uint64_t startListeningTime, currentCycleTime;
bool readNotWrite;
uint32_t sendCount;

void setup() {
  Serial.begin(9600);
  radio.begin();

  radio.setAutoAck(false); // append Ack packet
  radio.setDataRate(RF24_250KBPS); // transmission rate
  radio.setPALevel(RF24_PA_LOW); // distance and energy consump.

  radio.openWritingPipe(headToWristAddr);
  radio.openReadingPipe(0, wristToHeadAddr);

  readNotWrite = true;
  radio.startListening();
  startListeningTime = millis();
}

void loop() {
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
      if(outPayload.isPeriodical || sendCount < 10) {
        outPayload.sensorData = 1.23f;
        outPayload.sensorNumber = 1;
        radio.write(&outPayload, sizeof(outPayload));
        sendCount++;
      }
    }
  }

  else {
    readNotWrite = true;
    radio.startListening();
    startListeningTime = millis();    
  }
}
