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
// If your display has no backlight led resistor use an additional 220ohm for pin15

#include <LiquidCrystal.h>  // biblioteka odpowiedzialna za LCD
#include <DHT.h>            // biblioteka odpowiedzialna za DHT

#define DHTPIN 8          // Digital pin na Arduino do korego jest przylaczony DHT

// Uncomment whatever type DHT you're using!
//#define DHTTYPE DHT11   // DHT 11
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;   // piny do których podłączony jest wyświetlacz - sterownik HD44780
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);                    // tworzę obiekt klasy LiquidCrystal o powyzszych paramatrach

DHT dht(DHTPIN, DHTTYPE); // tworzę obiekt klasy DHT o nazwie dht o podanych parametrach

void blink();             // definicja funkcji do migania dioda - uzywana w celach testowych PCB

void setup() {
  Serial.begin(9600);               // inicjacja terminala
  dht.begin();                      // inicjalizacja czujnika
  lcd.begin(16, 2);                 // inicjalizacja LCD 16 znakow, 2 wiersze

  pinMode(LED_BUILTIN, OUTPUT);     // uzywane tylko do migania diody
  //Napis kontrolny startu programu
  Serial.println("Start pomiaru");  
  lcd.home();
  lcd.print("Pomiar ..."); 
}

void loop() {
  blink();                            // uruchomienie migania diody uzywane w celach testowych PCB
  delay(2000);                        //oczekiwanie na pomiar przez czujnik - normalnie zajmuje ok. 250 msek do nawet 2 sek
  float temp = dht.readTemperature(); // tworzę zmienną typu float przechowującą pomiar temperatury
  float wilg = dht.readHumidity();    // tworzę zmienną typu float przechowującą pomiar wilgotności

  if (isnan(temp) || isnan(wilg)) {   // sprawdzam czy nie ma błędu odczytu
    lcd.home(); // kursor na 0, 0
    lcd.print("Blad odczytu");        // wypisuję napis na LCD w przypadku bledu
    return; // wychodzę z pętli
  }
  
  // wyswietlam dane na LCD
  lcd.home(); // kursor na 0, 0
  lcd.print("T: ");
  lcd.print(temp);                    // wypisuję zmienną temp
  lcd.print(" stC");
  lcd.setCursor(0, 1);                // kursor na drugi wiersz
  lcd.print("W: ");
  lcd.print(wilg);                    // wypisuję zmienną wilg
  lcd.print("  %");
 
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