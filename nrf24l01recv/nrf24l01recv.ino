#include "RF24.h"
#include "nRF24L01.h"
#include "SPI.h"

#define CE_PIN 9
#define CSN_PIN 10
#define INTERVAL_MS_SIGNAL_LOST 1000
#define INTERVAL_MS_SIGNAL_RETRY 250

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

  radio.openWritingPipe(address[1]); // write to B
  radio.openReadingPipe(0, address[0]); // read from A
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
    Serial.print("Received: ");
    Serial.println(payload.data1);
    if(payload.bounceCount < 5) {
      payload.data1 += 10;
      payload.bounceCount++;
      Serial.print("Responding with ");
      Serial.println(payload.data1);
      radio.write(&payload, sizeof(payload));
    } else
      Serial.println("Bounce count too high, no response");
  }

  /*if(currentMillis - lastSignalMillis > INTERVAL_MS_SIGNAL_LOST) {
    Serial.println("We have lost connection, preventing unwanted behavior");
    delay(INTERVAL_MS_SIGNAL_RETRY);
  }*/
}