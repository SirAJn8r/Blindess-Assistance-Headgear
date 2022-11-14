#include "RF24.h"
#include "nRF24L01.h"
#include "SPI.h"

#define CE_PIN 9
#define CSN_PIN 10
#define INTERVAL_MS_SIGNAL_LOST 1000
#define INTERVAL_MS_SIGNAL_RETRY 250

RF24 radio(CE_PIN, CSN_PIN);
const byte address[6] = "00001";
struct payload {
  byte data1;
  char data2;
};
payload payload;

unsigned long lastSignalMillis = 0;

void setup() {
  Serial.begin(9600);
  
  radio.begin();
  radio.setAutoAck(false); // appeend Ack packet
  radio.setDataRate(RF24_250KBPS); // transmission rate
  radio.setPALevel(RF24_PA_MAX); // distance and energy consump.
  radio.setPayloadSize(sizeof(payload)); // default 32 bytes
  radio.openReadingPipe(0, address); // be a receiver
  radio.startListening();
}

void loop() {
  unsigned long currentMillis = millis();
  
  if(radio.available() > 0) {
    radio.read(&payload, sizeof(payload));
    Serial.println("Received");
    Serial.print("Data1:");
    Serial.println(payload.data1);
    Serial.print("Data2:");
    Serial.println(payload.data2);
    lastSignalMillis = currentMillis;
  }

  if(currentMillis - lastSignalMillis > INTERVAL_MS_SIGNAL_LOST) {
    Serial.println("We have lost connection, preventing unwanted behavior");
    delay(INTERVAL_MS_SIGNAL_RETRY);
  }
}
