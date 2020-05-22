#include <Arduino.h>

#include <LiquidCrystal.h>

const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

void blink();

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
  Serial.println("Start"); //Napis kontrolny startu programu
  //Zabawa wyświetlaczem
  lcd.begin(16, 2);
  lcd.home();
  lcd.print("Dziala");
}

int x = 6;
int y = 4;

void loop() {
  // put your main code here, to run repeatedly:
  // blink(); // uruchomienie migania diody
  for (int licznik = 0; licznik < 10; licznik++)
  {
    Serial.println("Dodawanie: ");
    Serial.println(x + y);
    Serial.println(licznik);
    Serial.println();
  delay(2000);
  }
  
}

// definicja funkcji migania diody na PCB Uno
void blink() {
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
// Serial.println("loop completed"); // przy pierwszym uruchomieniu wypisywałem komunikat kończący pętlę
}