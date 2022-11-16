//YWROBOT
//Compatible with the Arduino IDE 1.0
//Library version:1.1
#include <LiquidCrystal_I2C.h>

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

int dist = 0;
int deg = 0;
int cs = 3; //compass shift
int lux = 0; // brightness
int mode = 0;

long long changeTimer = 10000;
void setup()
{

  
  lcd.init();                      // initialize the lcd 
  // Print a message to the LCD.
  lcd.backlight();
  lcd.createChar(0, customCharDot); // create a new custom character
  lcd.createChar(1, customCharDotSel); // create a new custom character
  lcd.createChar(2, backSlash); // create a new custom character
  lcd.setCursor(3,0);
  //lcd.print("Hello, world!");
  //lcd.setCursor(2,1);
  //lcd.print("Ywrobot Arduino!");
  // lcd.setCursor(0,2);
  //lcd.print("Arduino LCM IIC 2004");
  // lcd.setCursor(2,3);
  //lcd.print("Power By Ec-yuan!");
}


void loop()
{
  //printCompass();
  if (millis() % 8 == 0) {
    lux += 20;
  }
  if (millis() % 50 == 0) {
    dist += 10;
  }
  if (millis() % 3 == 0) {
    deg += 5;
  }
  
  if (dist > 180) dist = 0;
  if (deg > 180) deg = -180;
  if (lux > 1000) lux = 0;

  if (mode == 0){
    printAll();
  } else if (mode == 1){
    printLidar();
  } else if (mode == 2){
    printCompass();
  } else if (mode == 3){
    printPhotoCell();
  }
  if (millis() > changeTimer){
    changeTimer = millis() + 10000;
    mode++;
    if (mode > 3) mode = 0;
    lcd.clear();
  }
}


void printAll() {
  lcd.setCursor(3,0);
  lcd.print(char(0b01111111));
  lcd.setCursor(9,0);
  lcd.print("^");
  lcd.setCursor(15,0);
  lcd.print(char(0b01111110));
  
  String dist1 = String(dist);
  int cursor1 = 4 - dist1.length();
  lcd.setCursor(cursor1, 1);
  lcd.print(dist1);
  lcd.print("cm");

  String dist2 = String(dist);
  int cursor2 = 10 - dist2.length();
  lcd.setCursor(cursor2, 1);
  lcd.print(dist2);
  lcd.print("cm");

  String dist3 = String(dist);
  int cursor3 = 16 - dist3.length();
  lcd.setCursor(cursor3, 1);
  lcd.print(dist3);
  lcd.print("cm");



  int luxL = String(lux).length();
  lcd.setCursor(2-luxL,2);
  lcd.print("   ");
  lcd.setCursor(5-luxL,2);
  lcd.print(lux);
  lcd.print(" Lux");



  lcd.setCursor(11,2);
  lcd.print("  ");
  String degS = String((deg/10) * 10);
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
  int luxL = String(lux).length();
  lcd.setCursor(7-luxL,2);
  lcd.print("   ");
  lcd.setCursor(10-luxL,2);
  lcd.print(lux);
  lcd.print(" Lux");

  //print brightness bar
  int bars = (lux * 9 / 1000)+1;
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

  String dist1 = String(dist);
  int cursor1 = 4 - dist1.length();
  lcd.setCursor(cursor1, 1);
  lcd.print(dist1);
  lcd.print("cm");

  String dist2 = String(dist);
  int cursor2 = 10 - dist2.length();
  lcd.setCursor(cursor2, 1);
  lcd.print(dist2);
  lcd.print("cm");

  String dist3 = String(dist);
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
  String degS = String((deg/10) * 10);
  lcd.setCursor(13,1);
  lcd.print(degS);
  lcd.print(char(0b11011111));
  lcd.print("  ");

  // direction
  if (deg >= -23 && deg <= 23) { // north
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
  else if (deg >= 24 && deg <= 66) { // north east
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
  else if (deg >= 67 && deg <= 113) { // east
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
  else if (deg >= 114 && deg <= 156) { // south east
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
  else if (deg > 157 || deg < -157) { // south
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
  else if (deg >= -66 && deg <= -24) { // north west
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
  else if (deg >= -113 && deg <= -67) { // west
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
  else if (deg >= -156 && deg <= -114) { // south west
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

void printCompassOutline(){
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
