#include "RF24.h"
#include "nRF24L01.h"
#include "SPI.h"

#define CE_PIN 9
#define CSN_PIN 10
#define INTERVAL_MS_TRANSMISSION 250

RF24 radio(CE_PIN, CSN_PIN);
const byte address[6] = "00001";
struct payload {
  byte data1;
  char data2;
};
payload payload;

void setup() {
  Serial.begin(9600);
  radio.begin();

  radio.setAutoAck(false); // appeend Ack packet
  radio.setDataRate(RF24_250KBPS); // transmission rate
  radio.setPALevel(RF24_PA_MAX); // distance and energy consump.
  radio.setPayloadSize(sizeof(payload)); // default 32 bytes
  radio.openWritingPipe(address); // be a transmitter
  radio.stopListening();
}

void loop() {
  payload.data1 = 123;
  payload.data2 = 'x';
  radio.write(&payload, sizeof(payload));

  Serial.print("Data1:");
  Serial.println(payload.data1);
  Serial.print("Data2:");
  Serial.println(payload.data2);
  Serial.println("Sent");

  delay(INTERVAL_MS_TRANSMISSION);
}
