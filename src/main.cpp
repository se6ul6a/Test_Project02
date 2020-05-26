#include <Arduino.h>

#include <LiquidCrystal.h>  // biblioteka odpowiedzialna za LCD
#include <DHT.h>            // biblioteka odpowiedzialna za DHT

#define DHTPIN 8         // pin do korego jestem przylaczony
#define DHTTYPE DHT22     // typ czujnika

const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;   // piny do których podłączony jest wyświetlacz
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);                    // tworzę obiekt klasy LiquidCrystal o powyzszych paramatrach

DHT dht(DHTPIN, DHTTYPE); // tworzę obiekt klasy DHT o nazwie dht o podanych parametrach

void blink();

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
  Serial.println("Start pomiaru");  //Napis kontrolny startu programu
  
  dht.begin();        // inicjalizacja czujnika
  lcd.begin(16, 2);   // inicjalizacja LCD
  
  lcd.home();
  lcd.print("Pomiar ..."); 
}

int x = 6;
int y = 4;

void loop() {
  // blink(); // uruchomienie migania diody
  delay(2000);  //oczekiwanie na pomiar przez czujnik - normalnie zajmuje ok. 250 msek do nawet 2 sek
  float temp = dht.readTemperature(); // tworzę zmienną typu float przechowującą pomiar temperatury
  float wilg = dht.readHumidity(); // tworzę zmienną typu float przechowującą pomiar wilgotności

  if (isnan(temp) || isnan(wilg)) { // sprawdzam czy nie ma błędu odczytu
    lcd.home(); // kursor na 0, 0
    lcd.print("Blad odczytu"); // wypisuję napis na lcd
    return; // wychodzę z pętli
  }
  
  // wyswietlam dane na LCD
  lcd.home(); // kursor na 0, 0
  lcd.print("T: ");
  lcd.print(temp); // wypisuję zmienną temp
  lcd.print(" stC");
  lcd.setCursor(0, 1); // kursor na drugi wiersz
  lcd.print("W: ");
  lcd.print(wilg); // wypisuję zmienną wilg
  lcd.print("  %");
 
  // wyswitlam dane na terminalu
  Serial.print("T: ");
  Serial.print(temp);
  Serial.println(" stC");
  Serial.print("W: ");
  Serial.print(wilg);
  Serial.println(" %");


  /*
  for (int licznik = 0; licznik < 10; licznik++)
  {
    Serial.println("Dodawanie: ");
    Serial.println(x + y);
    Serial.println(licznik);
    Serial.println();
  delay(2000);
  }
  */
}

// definicja funkcji migania diody na PCB Uno
void blink() {
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
// Serial.println("loop completed"); // przy pierwszym uruchomieniu wypisywałem komunikat kończący pętlę
}