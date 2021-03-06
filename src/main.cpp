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

#include <LiquidCrystal.h>      // biblioteka odpowiedzialna za LCD
#include <DHT.h>                // biblioteka odpowiedzialna za DHT
#include <Wire.h>               // biblioteka odpowiedzialna za I2C
#include <LiquidCrystal_I2C.h>  // biblioteka odpowiedzialna za LCD I2C
#include <Rtc_Pcf8563.h>        // biblioteka odpowidzialna za RTC PCF8563

// Uncomment whatever type DHT you're using!
//#define DHTTYPE DHT11   // DHT 11
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

#define DHTPIN 8          // Digital pin na Arduino do korego jest przylaczony DHT
Rtc_Pcf8563 rtc;          // Init RTC 

/* Fragment odpowiedzialny za wyswietlacz bez I2C
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;   // piny do których podłączony jest wyświetlacz - sterownik HD44780
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);                    // tworzę obiekt klasy LiquidCrystal o powyzszych paramatrach
*/

DHT dht(DHTPIN, DHTTYPE);                                     // tworzę obiekt klasy DHT o nazwie dht o podanych parametrach
LiquidCrystal_I2C lcd(0x27,20,4);                             // tworze obiekt klasy LCD o adresie 0x27 I2C 20 znakow 4 wiersze

// How to write symbols on LCD - see docs folder
byte thermo[8] = {0b00100,0b00110,0b00100,0b00110,0b00100,0b01110,0b01110,0b00000};
byte drop[8] = {0b00100,0b00100,0b01110,0b01110,0b10111,0b10111,0b01110,0b00000};
byte degrees[8] = {0b00010,0b00101,0b00010,0b00000,0b00000,0b00000,0b00000,0b00000};
byte heart[8] = {0b00000,0b01010,0b11111,0b11111,0b01110,0b00100,0b00000};
byte bell[8]  = {0b00100,0b01110,0b01110,0b01110,0b11111,0b00000,0b00100};
byte note[8]  = {0b00010,0b00011,0b00010,0b01110,0b11110,0b01100,0b00000};
byte clock[8] = {0b00000,0b01110,0b10101,0b10111,0b10001,0b01110,0b00000};
byte duck[8]  = {0b00000,0b01100,0b11101,0b01111,0b01111,0b00110,0b00000};
byte check[8] = {0b00000,0b00001,0b00011,0b10110,0b11100,0b01000,0b00000};
byte cross[8] = {0b00000,0b11011,0b01110,0b00100,0b01110,0b11011,0b00000};
byte retarrow[8] = {0b00001,0b00001,0b00101,0b01001,0b11111,0b01000,0b00100};
byte smile[8] = {0b00000,0b01010,0b00000,0b00000,0b10001,0b01110,0b00000,0b00000};
byte neutral[8] = {0b00000,0b01010,0b00000,0b00000,0b00000,0b11111,0b00000,0b00000};
byte sad[8] = {0b00000,0b01010,0b00000,0b00000,0b01110,0b10001,0b00000,0b00000};

int redPin = 9;
int greenPin = 10;
int bluePin = 11;
int pomiarSwiatla;     

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);     // uzywane do migania diody tylko do celow testowych
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);

  Serial.begin(9600);               // inicjacja terminala
  dht.begin();                      // inicjalizacja czujnika
  // lcd.begin(16, 2);              // inicjalizacja LCD bez I2C 16 znakow, 2 wiersze
  lcd.begin();                      // inicjalizacja LCD I2C - tu nie podajemy liczby wieszy i kolumn bo jest w definicji
  Wire.begin();                     // inicjalizacja I2C

  lcd.createChar(0, thermo);
  lcd.createChar(1, drop);
  lcd.createChar(2, degrees);
  lcd.createChar(3, heart);
  lcd.createChar(4, clock);
  lcd.createChar(5, smile);
  lcd.createChar(6, neutral);
  lcd.createChar(7, sad);
  
  lcd.home();
  lcd.print("Pomiar ..."); 

  // rtc.initClock();                  // wyczyszczenie rejestru zegara - odkomentuj w przypadku konieczności resetu
  //set a time to start with - uncomment if you need new settings
  // Set date (day, weekday, month, century(1=1900, 0=2000), year(0-99))
  // rtc.setDate(2, 1, 6, 0, 20);
  // Set time (hr, min, sec)
  // rtc.setTime(16, 58, 0);
}

void loop() {
  pomiarSwiatla = analogRead(A0);     // pomiar swiatla z fotorezystora 
  Serial.println(pomiarSwiatla);    
  if (pomiarSwiatla < 350){
    lcd.noBacklight();
  } else{
    lcd.backlight();
  }

  delay(1000);                        // oczekiwanie na pomiar przez czujnik - normalnie zajmuje ok. 250 msek do nawet 2 sek
  float temp = dht.readTemperature(); // tworzę zmienną typu float przechowującą pomiar temperatury
  float wilg = dht.readHumidity();    // tworzę zmienną typu float przechowującą pomiar wilgotności
  if (isnan(temp) || isnan(wilg)) {   // sprawdzam czy nie ma błędu odczytu
    lcd.home(); // kursor na 0, 0
    lcd.print("Blad odczytu");        // wypisuję napis na LCD w przypadku bledu
    return; // wychodzę z pętli
  }

  digitalWrite(2, LOW);
  digitalWrite(3, LOW);
  digitalWrite(4, LOW);
  if (wilg<35){
    digitalWrite(2, HIGH);
    analogWrite(redPin, 0);
    analogWrite(greenPin, 255);
    analogWrite(bluePin, 255);
    lcd.setCursor(0, 3);                // ustawiam kursor na czwarty wiersz
    lcd.write((byte)7);  
    lcd.print(" humidity");  
  }
  if (wilg>35 and wilg<40){
    digitalWrite(3, HIGH);
    analogWrite(redPin, 255);
    analogWrite(greenPin, 255);
    analogWrite(bluePin, 0);
    lcd.setCursor(0, 3);                // ustawiam kursor na czwarty wiersz
    lcd.write((byte)6);  
    lcd.print(" humidity");  
  }
  if (wilg>40) {
    digitalWrite(4, HIGH); 
    analogWrite(redPin, 255);
    analogWrite(greenPin, 0);
    analogWrite(bluePin, 255);  
    lcd.setCursor(0, 3);                // ustawiam kursor na czwarty wiersz
    lcd.write((byte)5);  
    lcd.print(" humidity");                                                
  }

  // wyswietlam dane na LCD
  lcd.home();                         // ustawiam kursor na 0,0 LCD
  lcd.write((byte)0);                 // rzutuje ikonę termometr na typ byte i wyświetlam ją na lcd
  lcd.print(" ");
  lcd.print(temp);                    // wypisuje zmienną temp
  lcd.print(" ");
  lcd.write((byte)2);                 // rzutuje ikonę stopnie na typ byte i wyświetlam ją na lcd
  lcd.print("C");
  lcd.setCursor(0, 1);                // ustawiam kursor na drugi wiersz
  lcd.write((byte)1);                 // rzutuje ikonę kropla na typ byte i wyświetlam ją na lcd
  lcd.print(" ");
  lcd.print(wilg);                    // wypisuje zmienną wilg
  lcd.print("  %");
  lcd.setCursor(0, 2);                // ustawiam kursor na trzeci wiersz
  lcd.write((byte)4); 
  lcd.print(" ");
  lcd.print(rtc.formatTime());
}