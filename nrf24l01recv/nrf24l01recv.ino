#include "RF24.h"
#include "nRF24L01.h"
#include "SPI.h"

#define CE_PIN 9
#define CSN_PIN 10
#define INTERVAL_MS_SIGNAL_LOST 1000
#define INTERVAL_MS_SIGNAL_RETRY 250

/*
 For Mega:
 1 (GND) - GND
 2 (VCC) - 3.3V
 3 (CE) = pin 9
 4 (CSN) = pin 10
 5 (SCK) = pin 52
 6 (MISO) = pin 50
 7 (MOSI) = pin 51
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
  radio.setAutoAck(false); // appeend Ack packet
  radio.setDataRate(RF24_250KBPS); // transmission rate
  radio.setPALevel(RF24_PA_MAX); // distance and energy consump.
  radio.setPayloadSize(sizeof(payload)); // default 32 bytes

  radio.openWritingPipe(address[0]);
  radio.openReadingPipe(1, address[1]);
}

void loop() {
  //unsigned long currentMillis = millis();
  bool readData = false;
  
  radio.startListening();
  if(radio.available() > 0) {
    radio.read(&payload, sizeof(payload)); 
    readData = true;
  }
  radio.stopListening();

  if(readData) {
    Serial.print("Received: ");
    Serial.println(payload.data1);
    //lastSignalMillis = currentMillis;

    if(bounceCount < 5) {
      payload.data1 += 10;
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

  /*if(currentMillis - lastSignalMillis > INTERVAL_MS_SIGNAL_LOST) {
    Serial.println("We have lost connection, preventing unwanted behavior");
    delay(INTERVAL_MS_SIGNAL_RETRY);
  }*/
}
