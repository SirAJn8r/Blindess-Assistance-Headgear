#include <LiquidCrystal_I2C.h>
#include "RF24.h"
#include "nRF24L01.h"
#include "SPI.h"

#define CE_PIN 9
#define CSN_PIN 10
#define listenDelay 50
#define sendTime 150

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

struct sensorPayload {
  uint8_t activeMode;
  uint8_t actuatorMode;
  int16_t data1;
  int16_t optData2; // for center TFL, when in distance mode
  int16_t optData3; // for right TFL, when in distance mode
}inPayload;

struct terminalRequestPayload {
  uint8_t activeMode; // set/update the active mode
  uint8_t actuatorMode; // set/update the actuator mode
}outPayload;

LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display
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

const byte headToWristAddr[6] = "00001";
const byte wristToHeadAddr[6] = "00002";
RF24 radio(CE_PIN, CSN_PIN);

ActiveMode activeMode;
ActuatorMode actuatorMode;

uint64_t changeTimer, cycleStartTime, currentCycleTime;
int32_t cs;
int16_t distL, distC, distR, luxBrightness, compassHeading;
bool readNotWrite, isListening;

void setup() {  
  distL = distC = distR = luxBrightness = compassHeading = 0;

  cs = 3;
  changeTimer = 10000;

  activeMode = distance;
  actuatorMode = vibration;

  lcd.init(); 
  lcd.backlight();
  lcd.createChar(0, customCharDot);
  lcd.createChar(1, customCharDotSel);
  lcd.createChar(2, backSlash);

  lcd.setCursor(3,0);
  //lcd.print("Hello, world!");
  //lcd.setCursor(2,1);
  //lcd.print("Ywrobot Arduino!");
  //lcd.setCursor(0,2);
  //lcd.print("Arduino LCM IIC 2004");
  //lcd.setCursor(2,3);
  //lcd.print("Power By Ec-yuan!");

  radio.begin();
  radio.setAutoAck(false); // append Ack packet
  radio.setDataRate(RF24_2MBPS); // transmission rate
  radio.setPALevel(RF24_PA_MAX); // distance and energy consump.
  radio.openWritingPipe(wristToHeadAddr);
  radio.openReadingPipe(0, headToWristAddr);

  readNotWrite = true;
  isListening = false;
  radio.stopListening();
  cycleStartTime = millis();
}

void loop() {
  communicate();

  /* For Testing: Roll through data */
  //printCompass();
  if (millis() % 8 == 0) {
    luxBrightness += 20;
  }
  if (millis() % 50 == 0) {
    distL += 10;
    distC += 10;
    distR += 10;
  }
  if (millis() % 3 == 0) {
    compassHeading += 5;
  }
  
  if (distL > 180) distL = 0;
  if (distC > 180) distC = 0;
  if (distR > 180) distR = 0;
  if (compassHeading > 180) compassHeading = -180;
  if (luxBrightness > 1000) luxBrightness = 0;
  /* End For Testing */

  switch(activeMode) {
    case off:
      printAll(); break;
    case distance:
      printLidar(); break;
    case photocell:
      printPhotoCell(); break;
    case compass:
      printCompass(); break;
  }

  /* For Testing: Roll through modes */
  if (millis() > changeTimer){
    changeTimer = millis() + 10000;
    activeMode = (activeMode + 1) % 4;
    readNotWrite = false; // write mode, tell headband to switch modes
    lcd.clear();
  }
  /* End For Testing */
}

void communicate() {
  currentCycleTime = millis() - cycleStartTime;

  if (readNotWrite) {
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
    } 
    
    else
      readNotWrite = true;
  }
}

void readInPayload() {
  radio.read(&inPayload, sizeof(inPayload));
  
  switch(inPayload.activeMode) {
    case distance:
      distL = inPayload.data1; break;
      distC = inPayload.optData2; break;
      distR = inPayload.optData3; break;
    case photocell:
      luxBrightness = inPayload.data1; break;
    case compass:
      compassHeading = inPayload.data1; break;
  }
}

void printAll() {
  lcd.setCursor(3,0);
  lcd.print(char(0b01111111));
  lcd.setCursor(9,0);
  lcd.print("^");
  lcd.setCursor(15,0);
  lcd.print(char(0b01111110));
  
  String dist1 = String(distL);
  int cursor1 = 4 - dist1.length();
  lcd.setCursor(cursor1, 1);
  lcd.print(dist1);
  lcd.print("cm");

  String dist2 = String(distC);
  int cursor2 = 10 - dist2.length();
  lcd.setCursor(cursor2, 1);
  lcd.print(dist2);
  lcd.print("cm");

  String dist3 = String(distR);
  int cursor3 = 16 - dist3.length();
  lcd.setCursor(cursor3, 1);
  lcd.print(dist3);
  lcd.print("cm");



  int luxL = String(luxBrightness).length();
  lcd.setCursor(2-luxL,2);
  lcd.print("   ");
  lcd.setCursor(5-luxL,2);
  lcd.print(luxBrightness);
  lcd.print(" Lux");



  lcd.setCursor(11,2);
  lcd.print("  ");
  String degS = String((compassHeading/10) * 10);
  lcd.setCursor(13,2);
  lcd.print(degS);
  lcd.print(char(0b11011111));
  lcd.print("  ");
  

  
  lcd.setCursor(15,3);
  lcd.print(char(1));
  lcd.print(char(0));
  lcd.print(char(0));
  lcd.print(char(0));
}

void printPhotoCell() {
  int luxL = String(luxBrightness).length();
  lcd.setCursor(7-luxL,2);
  lcd.print("   ");
  lcd.setCursor(10-luxL,2);
  lcd.print(luxBrightness);
  lcd.print(" Lux");

  //print brightness bar
  int bars = (luxBrightness * 9 / 1000)+1;
  lcd.setCursor(4,1);
  lcd.print("|");
  for (int i = 0 ; i < bars; i++)
    lcd.print(char(0b11111111));
  for (int i = bars ; i <= 9; i++)
    lcd.print(" ");
  lcd.print("|");

  lcd.setCursor(15,3);
  lcd.print(char(0));
  lcd.print(char(0));
  lcd.print(char(0));
  lcd.print(char(1));
}

void printLidar() {
  lcd.setCursor(3,0);
  lcd.print(char(0b01111111));
  lcd.setCursor(9,0);
  lcd.print("^");
  lcd.setCursor(15,0);
  lcd.print(char(0b01111110));

  String dist1 = String(distL);
  int cursor1 = 4 - dist1.length();
  lcd.setCursor(cursor1, 1);
  lcd.print(dist1);
  lcd.print("cm");

  String dist2 = String(distC);
  int cursor2 = 10 - dist2.length();
  lcd.setCursor(cursor2, 1);
  lcd.print(dist2);
  lcd.print("cm");

  String dist3 = String(distR);
  int cursor3 = 16 - dist3.length();
  lcd.setCursor(cursor3, 1);
  lcd.print(dist3);
  lcd.print("cm");

  lcd.setCursor(15,3);
  lcd.print(char(0));
  lcd.print(char(1));
  lcd.print(char(0));
  lcd.print(char(0));
}

void printCompass() {
  lcd.setCursor(11,1);
  lcd.print("  ");
  String degS = String((compassHeading/10) * 10);
  lcd.setCursor(13,1);
  lcd.print(degS);
  lcd.print(char(0b11011111));
  lcd.print("  ");

  // direction
  if (compassHeading >= -23 && compassHeading <= 23) { // north
    printCompassOutline();
    lcd.setCursor(cs + 2,0);
    lcd.print(" N ");
    lcd.setCursor(cs + 1,1);
    lcd.print("  |  ");
    lcd.setCursor(cs + 1,2);
    lcd.print("     ");
    lcd.setCursor(cs + 2,3);
    lcd.print("   ");
  } 
  else if (compassHeading >= 24 && compassHeading <= 66) { // north east
    printCompassOutline();
    lcd.setCursor(cs + 2,0);
    lcd.print("   ");
    lcd.setCursor(cs + 1,1);
    lcd.print("   / ");
    lcd.setCursor(cs + 1,2);
    lcd.print("     ");
    lcd.setCursor(cs + 2,3);
    lcd.print("   ");
  }
  else if (compassHeading >= 67 && compassHeading <= 113) { // east
    printCompassOutline();
    lcd.setCursor(cs + 2,0);
    lcd.print("   ");
    lcd.setCursor(cs + 1,1);
    lcd.print("   __");
    lcd.setCursor(cs + 1,2);
    lcd.print("     ");
    lcd.setCursor(cs + 2,3);
    lcd.print("   ");
  }
  else if (compassHeading >= 114 && compassHeading <= 156) { // south east
    printCompassOutline();
    lcd.setCursor(cs + 2,0);
    lcd.print("   ");
    lcd.setCursor(cs + 1,1);
    lcd.print("     ");
    lcd.setCursor(cs + 1,2);
    lcd.print("   ");
    lcd.print(char(2));
    lcd.print(" ");
    lcd.setCursor(cs + 2,3);
    lcd.print("   ");
  }
  else if (compassHeading > 157 || compassHeading < -157) { // south
    printCompassOutline();
    lcd.setCursor(cs + 2,0);
    lcd.print("   ");
    lcd.setCursor(cs + 1,1);
    lcd.print("     ");
    lcd.setCursor(cs + 1,2);
    lcd.print("  |  ");
    lcd.setCursor(cs + 2,3);
    lcd.print(" | ");
  }
  else if (compassHeading >= -66 && compassHeading <= -24) { // north west
    printCompassOutline();
    lcd.setCursor(cs + 2,0);
    lcd.print("   ");
    lcd.setCursor(cs + 1,1);
    lcd.print(" ");
    lcd.print(char(2));
    lcd.print("   ");
    lcd.setCursor(cs + 1,2);
    lcd.print("     ");
    lcd.setCursor(cs + 2,3);
    lcd.print("   ");
  }
  else if (compassHeading >= -113 && compassHeading <= -67) { // west
    printCompassOutline();
    lcd.setCursor(cs + 2,0);
    lcd.print("   ");
    lcd.setCursor(cs + 1,1);
    lcd.print("__   ");
    lcd.setCursor(cs + 1,2);
    lcd.print("     ");
    lcd.setCursor(cs + 2,3);
    lcd.print("   ");
  }
  else if (compassHeading >= -156 && compassHeading <= -114) { // south west
    printCompassOutline();
    lcd.setCursor(cs + 2,0);
    lcd.print("   ");
    lcd.setCursor(cs + 1,1);
    lcd.print("     ");
    lcd.setCursor(cs + 1,2);
    lcd.print(" /   ");
    lcd.setCursor(cs + 2,3);
    lcd.print("   ");
  }

  lcd.setCursor(15,3);
  lcd.print(char(0));
  lcd.print(char(0));
  lcd.print(char(1));
  lcd.print(char(0));
}

void printCompassOutline() {
  lcd.setCursor(cs + 1,0);
  lcd.print("/");
  lcd.setCursor(cs + 5,0);
  lcd.print(char(2));
  lcd.setCursor(cs,1);
  lcd.print("|");
  lcd.setCursor(cs + 6,1);
  lcd.print("|");
  lcd.setCursor(cs,2);
  lcd.print("|");
  lcd.setCursor(cs + 6,2);
  lcd.print("|");
  lcd.setCursor(cs + 1,3);
  lcd.print(char(2));
  lcd.setCursor(cs + 5,3);
  lcd.print("/");
}
