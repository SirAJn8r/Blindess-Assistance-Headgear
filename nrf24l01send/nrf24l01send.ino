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

void setup() {
  Serial.begin(9600);
  radio.begin();

  radio.setAutoAck(false); // appeend Ack packet
  radio.setDataRate(RF24_250KBPS); // transmission rate
  radio.setPALevel(RF24_PA_MAX); // distance and energy consump.
  radio.setPayloadSize(sizeof(payload)); // default 32 bytes

  radio.openWritingPipe(address[1]);
  radio.openReadingPipe(1, address[0]);
}

void loop() {
  bool readData = false;
  
  radio.startListening();
  if(radio.available() > 0) {
    radio.read(&payload, sizeof(payload)); 
    readData = true;
  }
  radio.stopListening();

  if(readData) {
    Serial.print("Received ");
    Serial.println(payload.data1);
    
    if(bounceCount < 5) {
      payload.data1 += 5;
      payload.bounceCount++;
      Serial.print("Responding with ");
      Serial.println(payload.data1);
      radio.write(&payload, sizeof(payload));
    } else
      Serial.println("Bounce count too high, no response");
  }

  if(Serial.available()) {
    payload.data1 = Serial.readString().toInt();
    payload.bounceCount = 0;
    Serial.print("\nSending ");
    Serial.println(payload.data1);
    radio.write(&payload, sizeof(payload));
  }
  //delay(INTERVAL_MS_TRANSMISSION);
}
