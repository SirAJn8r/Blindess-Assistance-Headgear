#include "RF24.h"
#include "nRF24L01.h"
#include "SPI.h"

#define CE_PIN 9
#define CSN_PIN 10
#define INTERVAL_MS_TRANSMISSION 250

/*
 For Uno:
 1 (GND) - GND
 2 (VCC) - 3.3V
 3 (CE) = pin 9
 4 (CSN) = pin 10
 5 (SCK) = pin 13
 6 (MISO) = pin 12
 7 (MOSI) = pin 11
*/

RF24 radio(CE_PIN, CSN_PIN);
const byte address[][6] = {"00001", "00002"};
struct payload {
  uint8_t data1;
  uint8_t bounceCount;
};
payload payload;

unsigned long lastSignalMillis = 0;

void setup() {
  Serial.begin(9600);
  radio.begin();

  radio.setAutoAck(true); // append Ack packet
  radio.setDataRate(RF24_250KBPS); // transmission rate
  radio.setPALevel(RF24_PA_LOW); // distance and energy consump.
  radio.setPayloadSize(sizeof(payload)); // up to 32 bytes

  radio.openWritingPipe(address[0]); // write to A
  radio.openReadingPipe(0, address[1]); // read from B
}

void loop() {
  // Try to read
  bool readData = false;
  
  radio.startListening();
  unsigned long currentMillis = millis();
  while (millis() - currentMillis < 50) ;

  if(radio.available() > 0) {
    radio.read(&payload, sizeof(payload)); 
    readData = true;
  }
  radio.stopListening();

  // Try to respond
  if(readData) {
    Serial.print("Received ");
    Serial.println(payload.data1);
  }

  // Try to send
  //if(Serial.available()) {
    if(currentMillis - lastSignalMillis > 10) {
    //payload.data1 = Serial.readString().toInt();
    payload.data1 = 4;
    payload.bounceCount = 0;
    Serial.print("Sending ");
    Serial.println(payload.data1);
    radio.write(&payload, sizeof(payload));
    lastSignalMillis = currentMillis;
  }

  //delay(INTERVAL_MS_TRANSMISSION);
}
