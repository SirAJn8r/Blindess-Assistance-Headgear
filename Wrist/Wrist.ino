#include "RF24.h"
#include "nRF24L01.h"
#include "SPI.h"
#include <LiquidCrystal_I2C.h>

#define button1 6
#define button2 5
#define debounceTime 300

#define CE_PIN 7
#define CSN_PIN 8
#define listenDelay 50
#define sendTime 700

// Printing info
#define cs 3 // compass shift
#define solidDotChar char(0)
#define hollowDotChar char(1)
#define backslashChar char(2)
#define rightArrowChar char(0b01111110)
#define leftArrowChar char(0b01111111)
#define degreeChar char(0b11011111)
#define blackSquareChar char(0b11111111)

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

struct sensorPayload {
  int16_t distL;
  int16_t distC;
  int16_t distR;
  int16_t compassHeading;
  int16_t lux;

  ActiveMode activeMode;
  ActuatorMode actuatorMode;
}inPayload;

struct terminalRequestPayload {
  ActiveMode activeMode; // set/update the active mode
  ActuatorMode actuatorMode; // set/update the actuator mode
}outPayload;

byte backSlash[8] = {
  0b00000,
  0b10000,
  0b01000,
  0b00100,
  0b00010,
  0b00001,
  0b00000,
  0b00000
};
byte customCharDot[8] = {
  0b00000,
  0b00000,
  0b00000,
  0b01110,
  0b01110,
  0b01110,
  0b00000,
  0b00000
};
byte customCharDotSel[8] = {
  0b00000,
  0b00000,
  0b11111,
  0b10001,
  0b10001,
  0b10001,
  0b11111,
  0b00000
};

uint64_t cycleStartTime, messageCountTimer, lastSentTime, lastButtonUseTime;
uint32_t recvMessageCount;
int16_t distL, distC, distR, lux, compassHeading; // lux = brightness
bool sendMessage, isListening;

const byte headToWristAddr[6] = "00001";
const byte wristToHeadAddr[6] = "00002";

ActiveMode activeMode;
ActuatorMode actuatorMode;

RF24 radio(CE_PIN, CSN_PIN);
LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display

void setup() {
  distL = distC = distR = lux = compassHeading = 0;

  activeMode = readAll;
  actuatorMode = vibration;

  Serial.begin(9600);
  while(!radio.begin()) Serial.println("Could not connect to radio");
  Serial.println("Everything connected");

  radio.setAutoAck(true); // append Ack packet
  radio.setDataRate(RF24_2MBPS); // transmission rate
  radio.setPALevel(RF24_PA_MAX); // distance and energy consump.
  radio.openWritingPipe(wristToHeadAddr);
  radio.openReadingPipe(0, headToWristAddr);

  sendMessage = false;
  isListening = true;
  radio.startListening();
  cycleStartTime = millis();
  messageCountTimer = cycleStartTime;
  lastSentTime = cycleStartTime;
  lastButtonUseTime = cycleStartTime;
  recvMessageCount = 0;

  lcd.init();
  lcd.backlight();
  lcd.createChar(0, customCharDot);
  lcd.createChar(1, customCharDotSel);
  lcd.createChar(2, backSlash);
  lcd.setCursor(3, 0);

  pinMode(button1, INPUT_PULLUP);
  pinMode(button2, INPUT_PULLUP);
}

void loop() {
  checkButtons();
  communicate();

  switch(activeMode) {
    case 0: printAll(); break;
    case 1: printLidar(); break;
    case 2: printPhotoCell(); break;
    case 3: printCompass(); break;
  }
}

void checkButtons() {
  uint64_t timeSinceButton = millis() - lastButtonUseTime;

  if(timeSinceButton > debounceTime) {
    if(digitalRead(button1) == LOW) {
      activeMode = (activeMode + 1) % 4;
      sendMessage = true;
      lastButtonUseTime = millis();
    }
    if(digitalRead(button2) == LOW) {
      actuatorMode = (actuatorMode + 1) % 4;
      sendMessage = true;
      lastButtonUseTime = millis();    
    }
  }
}

void communicate() {
  uint64_t currentCycleTime = millis() - cycleStartTime;

  if(currentCycleTime > 3000)
    sendMessage = true;

  if (!sendMessage) {
    if(!isListening) {
      radio.startListening();
      cycleStartTime = millis();
      isListening = true;
    } 
    
    else if (currentCycleTime > listenDelay) {
      if(radio.available() > 0)
        readInPayload();
    }
  
  } else {
    if (isListening) {
      radio.stopListening();
      cycleStartTime = millis();
      isListening = false;
    }
    
    else if(currentCycleTime < sendTime) {
      outPayload.activeMode = activeMode;
      outPayload.actuatorMode = actuatorMode;
      radio.write(&outPayload, sizeof(outPayload));
      lastSentTime = millis();
    } 
    
    else
      sendMessage = false;
  }

  // Debug info
  currentCycleTime = millis();
  Serial.print("Received Per Second = ");
  Serial.print(recvMessageCount * 1000 / (uint32_t)(currentCycleTime - messageCountTimer));
  Serial.print("  |  Last Sent Seconds Ago = ");
  Serial.println((uint32_t)(currentCycleTime - lastSentTime) / 1000);
  
  if(currentCycleTime - messageCountTimer > 3000) {
    recvMessageCount = 0;
    messageCountTimer = millis();
  }
}

void readInPayload() {
  radio.read(&inPayload, sizeof(inPayload));

  distL = inPayload.distL;
  distC = inPayload.distC;
  distR = inPayload.distR;
  compassHeading = inPayload.compassHeading;
  lux = inPayload.lux;

  recvMessageCount++;
}

void printAll() {
  String dist;
  int16_t cursor, i, luxL;

  lcd.setCursor(3, 0);
  lcd.print(leftArrowChar);
  lcd.setCursor(9, 0);
  lcd.print("^");
  lcd.setCursor(15, 0);
  lcd.print(rightArrowChar);
  
  dist = String(simplifyNum(distL));
  cursor = 4 - dist.length();
  lcd.setCursor(0, 1);
  for (i = 0; i < cursor; i++) lcd.print(" ");
  lcd.print(dist);
  lcd.print("cm");

  dist = String(simplifyNum(distC));
  cursor = 10 - dist.length();
  lcd.setCursor(6, 1);
  for (i = 6; i < cursor; i++) lcd.print(" ");
  lcd.print(dist);
  lcd.print("cm");

  dist = String(simplifyNum(distR));
  cursor = 16 - dist.length();
  lcd.setCursor(12, 1);
  for (i = 12; i < cursor; i++) lcd.print(" ");
  lcd.print(dist);
  lcd.print("cm");

  luxL = String(lux).length();
  lcd.setCursor(2 - luxL, 2);
  lcd.print("   ");
  lcd.setCursor(5 - luxL, 2);
  lcd.print(String(simplifyNum(lux)));
  lcd.print(" Lux");

  lcd.setCursor(11, 2);
  lcd.print("  ");
  lcd.setCursor(13, 2);
  lcd.print(String((compassHeading / 10) * 10));
  lcd.print(degreeChar);
  lcd.print("  ");
  
  lcd.setCursor(15, 3);
  lcd.print(hollowDotChar);
  lcd.print(solidDotChar);
  lcd.print(solidDotChar);
  lcd.print(solidDotChar);
}

void printPhotoCell() {
  int16_t luxL, bars, i;

  luxL = String(lux).length();
  lcd.setCursor(7 - luxL, 2);
  lcd.print("   ");
  lcd.setCursor(10 - luxL, 2);
  lcd.print(String(simplifyNum(lux)));
  lcd.print(" Lux");

  //print brightness bar
  bars = (lux * 9 / 1000) + 1;
  lcd.setCursor(4, 1);
  lcd.print("|");
  for (i = 0 ; i < bars; i++) lcd.print(blackSquareChar);
  for (i = bars ; i <= 9; i++) lcd.print(" ");
  lcd.print("|");

  lcd.setCursor(15, 3);
  lcd.print(solidDotChar);
  lcd.print(solidDotChar);
  lcd.print(solidDotChar);
  lcd.print(hollowDotChar);
}

void printLidar() {
  int16_t b, row, i;

  lcd.setCursor(9, 3);
  lcd.print(blackSquareChar);

  //left wall
  b = getDistBucket4(distL);
  row = 3 - b;
  if (row >= 0) {
    lcd.setCursor(8 - b, row);
    lcd.print("/");
    for (i = 0; i < row; i++) {
      lcd.setCursor(8 - (3 - i), i);
      lcd.print(" ");
    }
    for (i = 3; i > row; i--) {
      lcd.setCursor(8 - (3 - i), i);
      lcd.print(" ");
    }
  }
  
  //middle wall
  b = getDistBucket4(distC);
  row = 2 - b;
  if (row >= 0) {
    lcd.setCursor(9, row);
    lcd.print("_");
    for (i = 0; i < row; i++) {
      lcd.setCursor(9,i);
      lcd.print(" ");
    }
    for (i = 2; i > row; i--) {
      lcd.setCursor(9,i);
      lcd.print(" ");
    }
  }
  
  //right wall
  b = getDistBucket4(distR);
  row = 3 - b;
  if (row >= 0){
    lcd.setCursor(10 + b, row);
    lcd.print(backslashChar);
    for (i = 0; i < row; i++) {
      lcd.setCursor(10 + (3 - i), i);
      lcd.print(" ");
    }
    for (i = 3; i > row; i--) {
      lcd.setCursor(10 + (3 - i), i);
      lcd.print(" ");
    }
  }

  lcd.setCursor(15, 3);
  lcd.print(solidDotChar);
  lcd.print(hollowDotChar);
  lcd.print(solidDotChar);
  lcd.print(solidDotChar);
}

int16_t getDistBucket4(int16_t dist){
  if (dist < 50) return 0;
  if (dist < 100) return 1;
  if (dist < 200) return 2;
  if (dist < 300) return 3;
  return 4;
}

void printCompass() {
  lcd.setCursor(11, 1);
  lcd.print("  ");
  lcd.setCursor(13, 1);
  lcd.print((compassHeading / 10) * 10);
  lcd.print(degreeChar);
  lcd.print("  ");

  // direction
  if (compassHeading >= -23 && compassHeading <= 23) { // north
    printCompassOutline();
    lcd.setCursor(cs + 2, 0);
    lcd.print(" N ");
    lcd.setCursor(cs + 1, 1);
    lcd.print("  |  ");
    lcd.setCursor(cs + 1, 2);
    lcd.print("     ");
    lcd.setCursor(cs + 2, 3);
    lcd.print("   ");
  } 
  else if (compassHeading >= 24 && compassHeading <= 66) { // north east
    printCompassOutline();
    lcd.setCursor(cs + 2, 0);
    lcd.print("   ");
    lcd.setCursor(cs + 1, 1);
    lcd.print("   / ");
    lcd.setCursor(cs + 1, 2);
    lcd.print("     ");
    lcd.setCursor(cs + 2, 3);
    lcd.print("   ");
  }
  else if (compassHeading >= 67 && compassHeading <= 113) { // east
    printCompassOutline();
    lcd.setCursor(cs + 2, 0);
    lcd.print("   ");
    lcd.setCursor(cs + 1, 1);
    lcd.print("   __");
    lcd.setCursor(cs + 1, 2);
    lcd.print("     ");
    lcd.setCursor(cs + 2, 3);
    lcd.print("   ");
  }
  else if (compassHeading >= 114 && compassHeading <= 156) { // south east
    printCompassOutline();
    lcd.setCursor(cs + 2, 0);
    lcd.print("   ");
    lcd.setCursor(cs + 1, 1);
    lcd.print("     ");
    lcd.setCursor(cs + 1, 2);
    lcd.print("   ");
    lcd.print(backslashChar);
    lcd.print(" ");
    lcd.setCursor(cs + 2, 3);
    lcd.print("   ");
  }
  else if (compassHeading > 157 || compassHeading < -157) { // south
    printCompassOutline();
    lcd.setCursor(cs + 2, 0);
    lcd.print("   ");
    lcd.setCursor(cs + 1, 1);
    lcd.print("     ");
    lcd.setCursor(cs + 1, 2);
    lcd.print("  |  ");
    lcd.setCursor(cs + 2, 3);
    lcd.print(" | ");
  }
  else if (compassHeading >= -66 && compassHeading <= -24) { // north west
    printCompassOutline();
    lcd.setCursor(cs + 2, 0);
    lcd.print("   ");
    lcd.setCursor(cs + 1, 1);
    lcd.print(" ");
    lcd.print(backslashChar);
    lcd.print("   ");
    lcd.setCursor(cs + 1, 2);
    lcd.print("     ");
    lcd.setCursor(cs + 2, 3);
    lcd.print("   ");
  }
  else if (compassHeading >= -113 && compassHeading <= -67) { // west
    printCompassOutline();
    lcd.setCursor(cs + 2, 0);
    lcd.print("   ");
    lcd.setCursor(cs + 1, 1);
    lcd.print("__   ");
    lcd.setCursor(cs + 1, 2);
    lcd.print("     ");
    lcd.setCursor(cs + 2, 3);
    lcd.print("   ");
  }
  else if (compassHeading >= -156 && compassHeading <= -114) { // south west
    printCompassOutline();
    lcd.setCursor(cs + 2, 0);
    lcd.print("   ");
    lcd.setCursor(cs + 1, 1);
    lcd.print("     ");
    lcd.setCursor(cs + 1, 2);
    lcd.print(" /   ");
    lcd.setCursor(cs + 2, 3);
    lcd.print("   ");
  }

  lcd.setCursor(15, 3);
  lcd.print(solidDotChar);
  lcd.print(solidDotChar);
  lcd.print(hollowDotChar);
  lcd.print(solidDotChar);
}

void printCompassOutline() {
  lcd.setCursor(cs + 1, 0);
  lcd.print("/");
  lcd.setCursor(cs + 5, 0);
  lcd.print(backslashChar);
  lcd.setCursor(cs, 1);
  lcd.print("|");
  lcd.setCursor(cs + 6, 1);
  lcd.print("|");
  lcd.setCursor(cs, 2);
  lcd.print("|");
  lcd.setCursor(cs + 6, 2);
  lcd.print("|");
  lcd.setCursor(cs + 1, 3);
  lcd.print(backslashChar);
  lcd.setCursor(cs + 5, 3);
  lcd.print("/");
}

int16_t simplifyNum(int16_t n) {
  if (n < 10) return n;
  else if (n < 100) return (n / 10) * 10;
  else if (n < 1000) return (n / 100) * 100;
  return (n / 1000) * 1000;
}
