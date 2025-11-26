#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <string.h>
#include <avr/io.h>
#include <avr/wdt.h>  // Watchdog (AVR Uno/Nano/ProMini)
#include <Servo.h>

// =================== CONFIG ===================
static const uint32_t BAUDRATE = 115200;

// Communication / protocole
static const uint16_t VALIDATION_TIMEOUT_MS = 5000;   // attente réponse Pi
static const uint8_t  MAX_RETRIES = 3;                // ré-essais si timeout

// Saisie / UX
static const byte     PIN_LEN = 6;
static const bool     MASK_PIN_ON_LCD = true;         // affiche '*' au lieu des chiffres
static const uint32_t ENTRY_IDLE_TIMEOUT_MS = 15000;  // reset saisie si inactif 15s
static const uint32_t BACKLIGHT_SLEEP_MS = 20000;     // backlight off si inactif 20s

// Sécurité
static const uint8_t  MAX_ATTEMPTS = 3;               // essais avant lockout
static const uint32_t LOCKOUT_MS = 30000;             // durée du lockout

// Relais Porte
static const uint8_t  DOOR_RELAY_PIN = 11;            // <— adapte à ton câblage
static const bool     RELAY_ACTIVE_HIGH = true;       // HIGH = actif ?
static const uint32_t DOOR_OPEN_MS = 5000;            // durée d’ouverture

// Servo
static const uint8_t  SERVO_PIN = 13;                 // broche servo (change si nécessaire)
static const uint8_t  SERVO_CLOSED_ANGLE = 0;         // position fermée
static const uint8_t  SERVO_OPEN_ANGLE = 90;          // position ouverte
static const uint16_t SERVO_SWEEP_MS = 300;          // durée du balayage ouvr/ferme (ms)

// Watchdog
static const bool     USE_WATCHDOG = true;            // ✅ watchdog activé proprement

// --- Watchdog helpers ---
#define WDT_FEED() do { if (USE_WATCHDOG) wdt_reset(); } while(0)
void delay_with_wdt(uint16_t ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) { WDT_FEED(); delay(5); }
}

// ================= Servomoteur ================
Servo myservo;

// ================= CLAVIER & LCD ==============
const byte ROWS = 4, COLS = 4;
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

// ================== ETATS =====================
char    pinBuf[PIN_LEN + 1];
byte    pos = 0;
bool    canWrite = false;
bool    waitingValidation = false;
uint8_t retryCount = 0;

unsigned long waitStart = 0;
unsigned long lastKeyTs = 0;
unsigned long backlightTs = 0;

uint8_t  respPos = 0;
char     respBuf[160];

uint32_t seqCounter = 1;  // ID séquence croissant
uint32_t lastSentSeq = 0; // pour retries
uint32_t lastHandledSeq = 0; // dernière réponse traitée (dédoublonnage)

uint8_t  failedAttempts = 0;
bool     lockout = false;
unsigned long lockoutStart = 0;

// ================= BUZZER =====================
const int buzzerPin = 10;
static inline void Beep(int freq, int ms) { noTone(buzzerPin); tone(buzzerPin, freq, ms); }

// =============== UTILS GEN ===================
// CRC32 (polynôme 0xEDB88320) pour PIN+SEQ
uint32_t crc32_update(uint32_t crc, uint8_t data) {
  crc ^= data;
  for (uint8_t i = 0; i < 8; i++)
    crc = (crc >> 1) ^ (0xEDB88320UL & (-(int32_t)(crc & 1)));
  return crc;
}
uint32_t crc32_pin_seq(const char* pin, uint32_t seq) {
  uint32_t crc = 0xFFFFFFFFUL;
  // pin
  for (const char* p = pin; *p; ++p) crc = crc32_update(crc, (uint8_t)*p);
  // seq (4 octets little endian)
  for (uint8_t i = 0; i < 4; ++i) crc = crc32_update(crc, (uint8_t)((seq >> (8*i)) & 0xFF));
  return ~crc;
}

// =============== BACKLIGHT HELPERS ============
void lcdWake()   { lcd_1.backlight(); backlightTs = millis(); }
void lcdSleep()  { lcd_1.noBacklight(); }
void lcdTouch()  { backlightTs = millis(); if (!canWrite) lcdWake(); }

// =============== LCD & RELAIS =================
void relaySet(bool on) {
  digitalWrite(DOOR_RELAY_PIN, (RELAY_ACTIVE_HIGH ? (on?HIGH:LOW) : (on?LOW:HIGH)));
}

void deactivateWriting() { lcd_1.noBlink(); canWrite = false; }
void activateWriting()   { lcd_1.blink();  canWrite = true;  }
void resetLcd()          { lcd_1.clear(); lcd_1.setCursor(0, 0); }

void EmmitBuzzer(int frequency, int rep, int durationMs, int sleepMs) {
  for (int i = 0; i < rep; i++) {
    noTone(buzzerPin);
    tone(buzzerPin, frequency, durationMs);
    lcdTouch(); // activité légère = prolonge le timer
    if (sleepMs > 0) delay_with_wdt(sleepMs);   // ✅ WDT-safe
  }
}

void showMessage(const char* l1, const char* l2 = nullptr, uint16_t ms = 1000) {
  lcdTouch();
  lcd_1.clear();
  lcd_1.setCursor(0, 0); lcd_1.print(l1);
  lcd_1.setCursor(0, 1); if (l2) lcd_1.print(l2);
  if (ms) delay_with_wdt(ms);                   // ✅ WDT-safe
}

void showPinPrompt() {
  lcdTouch();
  resetLcd();
  lcd_1.print("Entrez PIN:");
  lcd_1.setCursor(0, 1);
  lcd_1.print("______");
  lcd_1.setCursor(0, 1);
}

// Smooth move servo from 'fromAngle' to 'toAngle' over 'durationMs' (feeds WDT)
void servo_smooth_move(uint8_t fromAngle, uint8_t toAngle, uint16_t durationMs) {
  if (fromAngle == toAngle) return;
  int steps = abs((int)toAngle - (int)fromAngle);
  if (steps == 0) return;
  uint32_t stepDelay = durationMs / (uint32_t)steps;
  int dir = (toAngle > fromAngle) ? 1 : -1;
  int angle = fromAngle;
  for (int i = 0; i < steps; ++i) {
    angle += dir;
    myservo.write(angle);
    delay_with_wdt((uint16_t)stepDelay);
  }
}

// Door open sequence: open with servo, sound, switch relay, wait, then close
void doorOpenSequence() {
  resetLcd();
  lcd_1.print("Ouvrez la porte");
  lcd_1.setCursor(0, 1);
  lcd_1.print("Num. #1");
  EmmitBuzzer(2500, 2, 300, 250);

  // Move servo to open smoothly
  myservo.attach(SERVO_PIN);              // ensure attached
  servo_smooth_move(SERVO_CLOSED_ANGLE, SERVO_OPEN_ANGLE, SERVO_SWEEP_MS);

  // Activate relay (if present)
  relaySet(true);

  // Hold door open for configured ms, feeding WDT
  unsigned long t0 = millis();
  while (millis() - t0 < DOOR_OPEN_MS) {
    WDT_FEED();
    delay(10); // small sleep to reduce CPU spin
  }

  // Deactivate relay and return servo to closed
  relaySet(false);
  servo_smooth_move(SERVO_OPEN_ANGLE, SERVO_CLOSED_ANGLE, SERVO_SWEEP_MS);

  // Detach servo to avoid jitter and reduce current draw
  myservo.detach();
}

// ================ PIN MGMT ===================
void resetPinBuffer() { memset(pinBuf, 0, sizeof(pinBuf)); pos = 0; }

void startPinEntry() {
  waitingValidation = false;
  retryCount = 0;
  respPos = 0;
  resetPinBuffer();
  showPinPrompt();
  activateWriting();
  lastKeyTs = millis();      // <<< important : reset du timer d’inactivité
  backlightTs = millis();
}

static inline void printDigit(char ch) {
  if (MASK_PIN_ON_LCD) lcd_1.print('*'); else lcd_1.print(ch);
}

// ============== COM / PROTOCOLE ==============
// REQ:  {"event":"pin_entered","pin":"123456","seq":42,"crc32":12345678}\n
// RESP: {"event":"pin_result","valid":true,"seq":42}\n

void purgeSerialInput() { while (Serial.available()) (void)Serial.read(); }

void sendPinToPi(uint32_t seq) {
  purgeSerialInput();
  uint32_t crc = crc32_pin_seq(pinBuf, seq);
  Serial.print(F("{\"event\":\"pin_entered\",\"pin\":\""));
  Serial.print(pinBuf);
  Serial.print(F("\",\"seq\":"));
  Serial.print(seq);
  Serial.print(F(",\"crc32\":"));
  Serial.print(crc);
  Serial.println(F("}"));
  Serial.flush();
}

bool strContains(const char* hay, const char* needle) { return (strstr(hay, needle) != nullptr); }

bool tryParseBoolValid(const char* json, bool &outValid) {
  const char* p = strstr(json, "\"valid\"");
  if (!p) return false;
  p = strchr(p, ':'); if (!p) return false;
  ++p; while (*p==' '||*p=='\t') ++p;
  if (strncmp(p, "true", 4)==0)  { outValid = true;  return true; }
  if (strncmp(p, "false",5)==0)  { outValid = false; return true; }
  return false;
}
bool tryParseSeq(const char* json, uint32_t &outSeq) {
  const char* p = strstr(json, "\"seq\"");
  if (!p) return false;
  p = strchr(p, ':'); if (!p) return false;
  ++p; while (*p==' '||*p=='\t') ++p;
  uint32_t v=0; bool any=false;
  while (*p>='0' && *p<='9') { v=v*10+(*p-'0'); ++p; any=true; }
  if (!any) return false;
  outSeq=v; return true;
}

void handleResponseLine() {
  respBuf[respPos] = '\0';

  if (!strContains(respBuf, "\"event\":\"pin_result\"")) { respPos=0; return; }

  bool isValid=false;
  if (!tryParseBoolValid(respBuf, isValid)) { respPos=0; return; }

  uint32_t gotSeq=0;
  bool hasSeq = tryParseSeq(respBuf, gotSeq);
  if (hasSeq) {
    if (gotSeq != lastSentSeq || gotSeq <= lastHandledSeq) { respPos=0; return; }
    lastHandledSeq = gotSeq; // dédoublonnage
  }

  // Réponse acceptée
  waitingValidation=false; respPos=0; retryCount=0;

  if (isValid) {
    failedAttempts = 0;   // reset anti-brute
    Beep(2000, 120);
    doorOpenSequence();
  } else {
    failedAttempts++;
    Beep(400, 150);
    showMessage("CODE INVALIDE", "Reessayer", 1000);
    if (failedAttempts >= MAX_ATTEMPTS) {
      lockout = true; lockoutStart = millis();
      showMessage("Trop d'essais", "Verrouillage...", 800);
      // alerte sonore
      EmmitBuzzer(600, 3, 150, 120);
    }
  }
  startPinEntry();
}

void beginValidation() {
  if (lockout) return; // ignore en lockout

  deactivateWriting();

  if (pos < PIN_LEN) {
    Beep(400, 120);
    showMessage("PIN incomplet", "6 chiffres svp", 1000);
    startPinEntry();
    return;
  }

  pinBuf[PIN_LEN] = '\0';

  resetLcd();
  lcd_1.print("VALIDATION...");
  lcd_1.setCursor(0, 1); lcd_1.print("Veuillez patienter");

  waitingValidation = true;
  waitStart = millis();
  respPos = 0;
  retryCount = 0;

  lastSentSeq = seqCounter++;
  sendPinToPi(lastSentSeq);
}

// =============== SELF-TEST ===================
void selfTest() {
  // LCD
  lcd_1.init(); lcdWake();
  lcd_1.print("Self-test...");
  // Buzzer
  EmmitBuzzer(1200,1,120,80);
  EmmitBuzzer(1800,1,120,80);
  // Relais
  pinMode(DOOR_RELAY_PIN, OUTPUT);
  relaySet(true);  delay_with_wdt(150);
  relaySet(false); delay_with_wdt(300);
  lcd_1.clear();
}

// ============ SETUP / LOOP ===================
void setup() {
  // ✅ Empêcher un reset en boucle si WDT actif avant
  MCUSR = 0;
  wdt_disable();

  Serial.begin(BAUDRATE);
  delay(400);

  pinMode(buzzerPin, OUTPUT);
  pinMode(DOOR_RELAY_PIN, OUTPUT);

  // Inits lentes AVANT d'activer le WDT
  selfTest();

  // ✅ Réactiver le watchdog seulement après les inits lentes
  if (USE_WATCHDOG) wdt_enable(WDTO_2S);

  // Initialise le servo en position fermée mais detaché (on attachera à l'ouverture)
  myservo.attach(SERVO_PIN);
  myservo.write(SERVO_CLOSED_ANGLE);
  myservo.detach();

  startPinEntry();
}

void loop() {
  if (USE_WATCHDOG) wdt_reset();

  unsigned long now = millis();

  // Backlight auto-sleep
  if (canWrite && (now - backlightTs > BACKLIGHT_SLEEP_MS)) {
    lcdSleep();
  }

  // Lockout
  if (lockout) {
    // affichage compte à rebours simple
    lcdWake();
    resetLcd();
    lcd_1.print("Verrouillage...");
    uint32_t rem = (LOCKOUT_MS - (now - lockoutStart))/1000;
    lcd_1.setCursor(0,1); lcd_1.print("Attendre "); lcd_1.print(rem); lcd_1.print("s");
    if (now - lockoutStart >= LOCKOUT_MS) {
      lockout = false; failedAttempts = 0;
      startPinEntry();
    }
    delay_with_wdt(250);   // ✅ WDT-safe
    return;
  }

  // Phase attente réponse
  if (waitingValidation) {
    while (Serial.available()) {
      WDT_FEED();  // ✅ nourrit le WDT pendant la lecture série
      char c = (char)Serial.read();
      if (c=='\r') continue;
      if (c=='\n') { handleResponseLine(); break; }
      if (respPos < sizeof(respBuf)-1) respBuf[respPos++] = c;
      else respPos = 0; // overflow -> purge
    }
    if (waitingValidation && (now - waitStart > VALIDATION_TIMEOUT_MS)) {
      if (retryCount < MAX_RETRIES) {
        retryCount++; waitStart = now;
        sendPinToPi(lastSentSeq); Beep(900, 80);
      } else {
        waitingValidation=false;
        Beep(350, 180);
        showMessage("Pas de reponse", "du Pi (timeout)", 1000);
        startPinEntry();
      }
    }
    return;
  }

  // Inactivité saisie -> seulement si l'utilisateur a commencé à taper (pos > 0)
  if (canWrite && pos > 0 && (now - lastKeyTs > ENTRY_IDLE_TIMEOUT_MS)) {
    Beep(700, 80);
    showMessage("Timeout saisie", nullptr, 600);
    startPinEntry();
    return;
  }

  // Lecture clavier
  char key = keypad.getKey();
  if (key != NO_KEY) {
    lcdWake();
    lastKeyTs = now;        // on rafraîchit le timer d’inactivité à chaque touche
  } else {
    return;
  }

  if (!canWrite) return;

  if (key >= '0' && key <= '9') {
    if (pos < PIN_LEN) {
      pinBuf[pos++] = key;
      printDigit(key);
      if (pos == PIN_LEN) beginValidation();
    } else {
      Beep(800, 60);
    }
  } else if (key == '*') {
    // effacement intelligent: courte pression -> 1 char, longue -> clear
    unsigned long t0 = millis();
    while (keypad.getState() == HOLD) { if (millis() - t0 > 600) break; }
    if (millis() - t0 > 600) { // long press -> clear total
      resetPinBuffer();
      lcd_1.setCursor(0,1); lcd_1.print("______"); lcd_1.setCursor(0,1);
      Beep(650, 90);
    } else if (pos > 0) { // backspace
      pos--;
      pinBuf[pos] = '\0';
      lcd_1.setCursor(0, 1);
      for (byte i=0;i<PIN_LEN;i++) lcd_1.print((i<pos)?(MASK_PIN_ON_LCD?'*':pinBuf[i]) : '_');
      lcd_1.setCursor(pos,1);
      Beep(700, 60);
    }
  } else if (key == '#') {
    beginValidation();
  } // A/B/C/D ignorées
}
