#include <Arduino.h>

// DHT REQUIRES the following Arduino libraries:
// - DHT Sensor Library: https://github.com/adafruit/DHT-sensor-library
// - Adafruit Unified Sensor Lib: https://github.com/adafruit/Adafruit_Sensor
// Wiring see docs folder in the project
// Connect pin 1 (on the left) of the sensor to +5V
// NOTE: If using a board with 3.3V logic like an Arduino Due connect pin 1
// to 3.3V instead of 5V!
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

// LCD HD44780 REQUIRES LiquidCrystal library
// Wiring see docs folder in the project
// If your display has no backlight led resistor use an additional 220ohm for pin15 LCD

// BMP180GY REQUIRES Adafruit BMP085 library 
// Wiring see docs folder in the project
// Connect VCC of the BMP085 sensor to 3.3V (NOT 5.0V!)
// Connect GND to Ground
// Connect SCL to i2c clock - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 5
// Connect SDA to i2c data - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 4
// EOC is not used, it signifies an end of conversion
// XCLR is a reset pin, also not used here

// i2c_scanner - kod uzyty w funkcji scannerI2C
//
// Version 1
//    This program (or code that looks like it)
//    can be found in many places.
//    For example on the Arduino.cc forum.
//    The original author is not know.
// Version 2, Juni 2012, Using Arduino 1.0.1
//     Adapted to be as simple as possible by Arduino.cc user Krodal
// Version 3, Feb 26  2013
//    V3 by louarnold
// Version 4, March 3, 2013, Using Arduino 1.0.3
//    by Arduino.cc user Krodal.
//    Changes by louarnold removed.
//    Scanning addresses changed from 0...127 to 1...119,
//    according to the i2c scanner by Nick Gammon
//    http://www.gammon.com.au/forum/?id=10896
// Version 5, March 28, 2013
//    As version 4, but address scans now to 127.
//    A sensor seems to use address 120.
// Version 6, November 27, 2015.
//    Added waiting for the Leonardo serial communication.
//
//
// This sketch tests the standard 7-bit addresses
// Devices with higher bit address might not be seen properly.

#include <LiquidCrystal.h>      // biblioteka odpowiedzialna za LCD
#include <DHT.h>                // biblioteka odpowiedzialna za DHT
#include <Wire.h>               // biblioteka odpowiedzialna za I2C
#include <LiquidCrystal_I2C.h>  // biblioteka odpowiedzialna za LCD I2C

// Uncomment whatever type DHT you're using!
//#define DHTTYPE DHT11   // DHT 11
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

#define DHTPIN 8          // Digital pin na Arduino do korego jest przylaczony DHT

/* Fragment odpowiedzialny za wyswietlacz bez I2C
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;   // piny do których podłączony jest wyświetlacz - sterownik HD44780
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);                    // tworzę obiekt klasy LiquidCrystal o powyzszych paramatrach
*/

DHT dht(DHTPIN, DHTTYPE);                                     // tworzę obiekt klasy DHT o nazwie dht o podanych parametrach
LiquidCrystal_I2C lcd(0x27,20,4);                             // tworze obiekt klasy LCD o adresie 0x27 I2C 20 znakow 4 wiersze

void blink();             // definicja funkcji do migania dioda - uzywana w celach testowych PCB
void scannerI2C();        // definicja funkcji skanowania magistrali I2C w poszukiwaniu adresu urzadzen


// How to write symbols on LCD - see docs folder
byte termometr[8] = {0b00100,0b00110,0b00100,0b00110,0b00100,0b01110,0b01110,0b00000};
byte kropla[8] = {0b00100,0b00100,0b01110,0b01110,0b10111,0b10111,0b01110,0b00000};
byte stopnie[8] = {0b00010,0b00101,0b00010,0b00000,0b00000,0b00000,0b00000,0b00000};
byte serce[8] = {0b00000,0b01010,0b11111,0b11111,0b01110,0b00100,0b00000};
byte bell[8]  = {0b00100,0b01110,0b01110,0b01110,0b11111,0b00000,0b00100};
byte note[8]  = {0b00010,0b00011,0b00010,0b01110,0b11110,0b01100,0b00000};
byte clock[8] = {0b00000,0b01110,0b10101,0b10111,0b10001,0b01110,0b00000};
byte duck[8]  = {0b00000,0b01100,0b11101,0b01111,0b01111,0b00110,0b00000};
byte check[8] = {0b00000,0b00001,0b00011,0b10110,0b11100,0b01000,0b00000};
byte cross[8] = {0b00000,0b11011,0b01110,0b00100,0b01110,0b11011,0b00000};
byte retarrow[8] = {0b00001,0b00001,0b00101,0b01001,0b11111,0b01000,0b00100};

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);     // uzywane do migania diody tylko do celow testowych

  Serial.begin(9600);               // inicjacja terminala
  dht.begin();                      // inicjalizacja czujnika
  // lcd.begin(16, 2);              // inicjalizacja LCD bez I2C 16 znakow, 2 wiersze
  lcd.begin();                      // inicjalizacja LCD I2C - tu nie podajemy liczby wieszy i kolumn bo jest w definicji
  Wire.begin();                     // inicjalizacja I2C
  lcd.createChar(0, termometr);
  lcd.createChar(1, kropla);
  lcd.createChar(2, stopnie);
  lcd.createChar(3, serce);
  lcd.createChar(4, clock);
  
  lcd.home();
  lcd.print("Pomiar ..."); 
  // lcd.noBacklight();            // wylaczenie podswietlenia LCD I2C

  while (!Serial);                 // Arduino: wait for serial monitor
  Serial.println("\nI2C Scanner");
}

void loop() {
  // blink();                         // uruchomienie migania diody uzywane w celach testowych PCB
  // scannerI2C();                    // uruchomienie skanera magistrali I2C w celu wyswietlenia adresow urzadzen

  delay(2000);                        // oczekiwanie na pomiar przez czujnik - normalnie zajmuje ok. 250 msek do nawet 2 sek
  float temp = dht.readTemperature(); // tworzę zmienną typu float przechowującą pomiar temperatury
  float wilg = dht.readHumidity();    // tworzę zmienną typu float przechowującą pomiar wilgotności
  if (isnan(temp) || isnan(wilg)) {   // sprawdzam czy nie ma błędu odczytu
    lcd.home(); // kursor na 0, 0
    lcd.print("Blad odczytu");        // wypisuję napis na LCD w przypadku bledu
    return; // wychodzę z pętli
  }
 
  // wyswietlam dane na LCD
  lcd.home();                         // ustawiam kursor na 0, 0 LCD
  lcd.write((byte)0);                 // rzutuję ikonę termometr na typ byte i wyświetlam ją na lcd
  lcd.print(" ");
  lcd.print(temp);                    // wypisuję zmienną temp
  lcd.print(" ");
  lcd.write((byte)2);                 // rzutuję ikonę stopnie na typ byte i wyświetlam ją na lcd
  lcd.print("C");
  lcd.setCursor(0, 1);                // ustawian kursor na drugi wiersz
  lcd.write((byte)1);                 // rzutuję ikonę kropla na typ byte i wyświetlam ją na lcd
  lcd.print(" ");
  lcd.print(wilg);                    // wypisuję zmienną wilg
  lcd.print("  %");
  lcd.setCursor(0, 2);                // ustawian kursor na trzeci wiersz
  lcd.write((byte)4); 
  lcd.setCursor(0, 3);                // ustawian kursor na czwarty wiersz
  lcd.print("I");
  lcd.write((byte)3);  
  lcd.print(" Arduino");
  
  // wyswitlam dane na terminalu
  Serial.print("T: ");
  Serial.print(temp);
  Serial.println(" stC");
  Serial.print("W: ");
  Serial.print(wilg);
  Serial.println(" %");
}

// definicja funkcji migania diody na PCB
void blink() {
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
}

// definicja funkcji do skanowania I2C w celu poszukiwania adresow urzadzen
void scannerI2C() {
  byte error, address;
  int nDevices;
 
  Serial.println("Scanning...");
 
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
 
      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
 
  delay(5000);           // wait 5 seconds for next scan
}