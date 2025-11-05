#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <string.h>

const byte ROWS = 4;
const byte COLS = 4;

char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};

byte rowPins[ROWS] = {9, 8, 7, 6};
byte colPins[COLS] = {5, 4, 3, 2};

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

LiquidCrystal_I2C lcd_1(0x27, 16, 2);

const char CODE[] = "123456";
const byte PIN_LEN = 6;

char pin[PIN_LEN + 1];
byte pos = 0;
bool canWrite = false;
bool doorOpen = false;
bool itemClaimed = false;

const int buzzerPin = 10;

void deactivateWriting() {
  lcd_1.noBlink();
  canWrite = false;
}

void activateWriting() {
  lcd_1.blink();
  canWrite = true;
}

void resetLcd() {
  lcd_1.clear();
  lcd_1.setCursor(0, 0);
}

void EmmitBuzzer(int frequency, int rep, int duration, int sleep) {
  for (int i = 0; i < rep; i++) {
    noTone(buzzerPin);
    tone(buzzerPin, frequency, duration);
    if (delay > 0) {
      delay(sleep);
    }
  }
}

void showPinPrompt() {
  resetLcd();
  lcd_1.print("Entrez PIN:");
  lcd_1.setCursor(0, 1);
  lcd_1.print("______");
  lcd_1.setCursor(0, 1);
}

void resetPinBuffer() {
  memset(pin, 0, sizeof(pin));
  pos = 0;
}

void startPinEntry() {
  resetPinBuffer();
  showPinPrompt();
  activateWriting();
}

void showMessage(const char* l1, const char* l2, uint16_t ms=1000) {
  lcd_1.clear();
  lcd_1.setCursor(0, 0);
  lcd_1.print(l1);
  lcd_1.setCursor(0, 1);
  if (l2) lcd_1.print(l2);
  delay(ms);
}

void validatePin() {
  deactivateWriting();

  if (pos < PIN_LEN) {
    showMessage("PIN incomplet", "6 chiffres svp", 1200);
    startPinEntry();
    return;
  }

  pin[PIN_LEN] = '\0';

  showMessage("VALIDATION...", nullptr, 500);

  if (strcmp(pin, CODE) == 0) {
    doorPage();
  } else {
    showMessage("CODE INVALIDE", "Reessayer", 1200);
  }

  startPinEntry();
}

void doorPage() {
  resetLcd();
  lcd_1.print("Ouvrez la porte");
  lcd_1.setCursor(0, 1);
  lcd_1.print("Num. #1");
  EmmitBuzzer(2500, 2, 300, 350);
  return;
}

void setup() {
  lcd_1.init();
  lcd_1.backlight();
  startPinEntry();
}

void loop() {
  if (!canWrite) return;

  char key = keypad.getKey();
  if (key == NO_KEY) return;

  if (key >= '0' && key <= '9') {
    if (pos < PIN_LEN) {
      pin[pos++] = key;
      lcd_1.print(key);
      if (pos == PIN_LEN) {
        validatePin(); return;
      }
    }
  }
  else if (key == '*') {
    if (pos > 0) {
      pos--;
      pin[pos] = '\0';
      lcd_1.setCursor(pos, 1);
      lcd_1.print("_");
      lcd_1.setCursor(pos, 1);
    }
  }
  else if (key == '#') {
    validatePin();
  }
}
