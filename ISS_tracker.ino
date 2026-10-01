/*****************************************************************************/
/*
  ISS-tracker
  (c) Hans Schneider 2026
  
  V1.5
  01.10.2026
  
  Based on a project of Arpan Mondal
  https://www.instructables.com/ISS-Antenna-Indicates-When-the-ISS-Is-Overhead  
  
  This code includes numerous improvements over the original. When the LED
  lights up yellow (distance between 500 km and 1000 km), a tone sounds 
  periodically in sync with the flashing LED; when the LED flashes green
  (distance less or equal 500 km), a siren sounds.
  
  To amplify the sound, a small audio amplifier and a speaker have been 
  integrated instead of a buzzer. The amplifier draws its supply voltage from
  the board, and the audio signal is fed from pin D8 via a voltage divider. 
  The amplifier can be switched on and off using a small switch.
  
  
  Board: Lolin(Wemos) D1 R2 & mini oder D1 mini pro
*/
/*****************************************************************************/

#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ArduinoJson.h>

#define RED_PIN    D5
#define GREEN_PIN  D6
#define BLUE_PIN   D7
#define BUZZER_PIN D8

// ---- USER CONFIG ----
const char* WIFI_SSID = "DEIN_WLAN";      	           // hier deine Zugangsdaten eintragen
const char* WIFI_PASS = "DEIN_PASSWORT";

const double MY_LAT = 50.698;                          // hier deinen Standort eintragen
const double MY_LON = 6.096;

const double VISIBLE_RADIUS      = 500.0;              // km
const double SLIGHTLY_FAR_RADIUS = 1000.0;             // km

const char* ISS_HOST = "api.open-notify.org";
const unsigned long HTTP_INTERVAL_MS   = 5000;         // Abstand zwischen Abrufen
const unsigned long REQUEST_TIMEOUT_MS = 4000;         // Abbruch hängender Abrufe

// ---- MUSTER-PARAMETER (gemeinsame Zeitbasis für Licht + Ton) ----
const unsigned long OUTPUT_INTERVAL_MS = 20;
const unsigned long PULSE_MS           = 1000;
const int           PWM_MAX            = 1023;
const int           PWM_MIN_PULSE      = 40;

const unsigned long SIREN_ON_MS    = 3000;
const unsigned long SIREN_CYCLE_MS = 5000;
const int SIREN_MIN_FREQ = 700;
const int SIREN_MAX_FREQ = 1200;

const unsigned long BEEP_MS       = 200;
const unsigned long BEEP_CYCLE_MS = 5000;
const int BEEP_FREQ = 700;

enum LedMode { LED_OFF, LED_STEADY_RED, LED_PULSING_YELLOW, LED_PULSING_GREEN };
LedMode currentMode = LED_OFF;
unsigned long modeStart = 0;
unsigned long lastOutputUpdate = 0;
long lastBeepCycle = -1;

// ---- ASYNC-ABRUF: Zustand ----
AsyncClient* aClient = nullptr;                        // wird einmal in setup() angelegt, NIE gelöscht
char rxBuf[1024];
size_t rxLen = 0;
bool requestRunning = false;
volatile bool requestFinished = false;                 // von Callbacks gesetzt (Disconnect/Fehler)
unsigned long requestStart = 0;

// ---- HELPER ----
double deg2rad(double deg) { return deg * (PI / 180.0); }

double haversine(double lat1, double lon1, double lat2, double lon2) {
  double dLat = deg2rad(lat2 - lat1);
  double dLon = deg2rad(lon2 - lon1);
  lat1 = deg2rad(lat1);
  lat2 = deg2rad(lat2);
  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos(lat1) * cos(lat2) * sin(dLon / 2) * sin(dLon / 2);
  return 6371.0 * 2 * atan2(sqrt(a), sqrt(1 - a));
}

// Schreibt nur, wenn sich der Wert wirklich ändert (vermeidet Glitches)
int lastR = -1, lastG = -1, lastB = -1;
void setRGB(int r, int g, int b) {
  r = constrain(r, 0, PWM_MAX);
  g = constrain(g, 0, PWM_MAX);
  b = constrain(b, 0, PWM_MAX);
  if (r != lastR) { analogWrite(RED_PIN, r);   lastR = r; }
  if (g != lastG) { analogWrite(GREEN_PIN, g); lastG = g; }
  if (b != lastB) { analogWrite(BLUE_PIN, b);  lastB = b; }
}

int lastFreq = 0;
void setBuzzer(int freq) {                             // 0 = aus
  if (freq == lastFreq) return;
  if (freq > 0) tone(BUZZER_PIN, freq);
  else          noTone(BUZZER_PIN);
  lastFreq = freq;
}

void setMode(LedMode m) {
  if (m == currentMode) return;
  currentMode = m;
  modeStart = millis();                                // Licht und Ton starten gemeinsam bei Phase 0
  lastBeepCycle = -1;
  setBuzzer(0);
}

// Dreieckswelle: PWM_MIN_PULSE -> PWM_MAX -> PWM_MIN_PULSE, Maximum bei PULSE_MS/2
int pulseLevel(unsigned long t) {
  unsigned long half = PULSE_MS / 2;
  unsigned long ph = t % PULSE_MS;
  unsigned long up = (ph < half) ? ph : (PULSE_MS - ph);
  return PWM_MIN_PULSE + (up * (unsigned long)(PWM_MAX - PWM_MIN_PULSE)) / half;
}

// ---- EINZIGE STELLE, die LED und Buzzer ansteuert ----
void updateOutputs() {
  unsigned long now = millis();
  if (now - lastOutputUpdate < OUTPUT_INTERVAL_MS) return;
  lastOutputUpdate = now;

  unsigned long t = now - modeStart;

  switch (currentMode) {
    case LED_STEADY_RED:
      setRGB(512, 0, 0);
      setBuzzer(0);
      break;

    case LED_PULSING_YELLOW: {
      int level = pulseLevel(t);
      setRGB(level, level / 10, 0);

      long cycle = t / BEEP_CYCLE_MS;
      unsigned long tc = t % BEEP_CYCLE_MS;
      // Piep beginnt kurz vor dem Helligkeitsmaximum, genau einmal pro Zyklus
      if (cycle != lastBeepCycle && tc >= PULSE_MS / 2 - BEEP_MS / 2) {
        lastBeepCycle = cycle;
        tone(BUZZER_PIN, BEEP_FREQ, BEEP_MS);
      }
      break;
    }

    case LED_PULSING_GREEN: {
      int level = pulseLevel(t);
      setRGB(0, level, 0);
      bool sirenOn = (t % SIREN_CYCLE_MS) < SIREN_ON_MS;
      // Tonhöhe folgt der Helligkeit -> immer synchron
      setBuzzer(sirenOn ? map(level, PWM_MIN_PULSE, PWM_MAX, SIREN_MIN_FREQ, SIREN_MAX_FREQ) : 0);
      break;
    }

    default:
      setRGB(0, 0, 0);
      setBuzzer(0);
      break;
  }
}

// ---- ASYNC ISS-ABRUF ----

// true, sobald das JSON vollständig angekommen ist (geschweifte Klammern ausgeglichen)
bool bodyComplete() {
  if (rxLen == 0) return false;
  rxBuf[rxLen] = 0;
  const char* s = strchr(rxBuf, '{');
  if (!s) return false;
  int depth = 0;
  for (const char* p = s; *p; p++) {
    if (*p == '{') depth++;
    else if (*p == '}' && --depth == 0) return true;
  }
  return false;
}

// Einmal in setup() aufrufen: Objekt und Callbacks werden nur ein einziges Mal angelegt
void setupAsyncClient() {
  aClient = new AsyncClient();

  aClient->onConnect([](void* arg, AsyncClient* c) {
    c->write("GET /iss-now.json HTTP/1.0\r\n"
             "Host: api.open-notify.org\r\n"
             "Connection: close\r\n\r\n");
  }, nullptr);

  aClient->onData([](void* arg, AsyncClient* c, void* data, size_t len) {
    size_t space = sizeof(rxBuf) - 1 - rxLen;
    if (len > space) len = space;
    memcpy(rxBuf + rxLen, data, len);
    rxLen += len;
  }, nullptr);

  aClient->onDisconnect([](void* arg, AsyncClient* c) {
    requestFinished = true;
  }, nullptr);

  aClient->onError([](void* arg, AsyncClient* c, int8_t error) {
    requestFinished = true;
  }, nullptr);
}

void startIssRequest() {
  if (requestRunning || WiFi.status() != WL_CONNECTED) return;

  // Alte Verbindung noch nicht vollständig beendet? Dann abbrechen und beim nächsten Intervall neu versuchen
  if (!aClient->disconnected()) {
    aClient->close(true);
    return;
  }

  rxLen = 0;
  requestFinished = false;
  requestRunning = true;
  requestStart = millis();

  if (!aClient->connect(ISS_HOST, 80)) {
    requestRunning = false;
    Serial.println("ISS: connect fehlgeschlagen");
  }
}

void processResponse() {
  rxBuf[rxLen] = 0;
  char* s = strchr(rxBuf, '{');
  char* e = strrchr(rxBuf, '}');
  if (!s || !e || e < s) {
    Serial.println("ISS: keine gueltige Antwort");
    return;                                            // letzter Modus läuft weiter
  }

  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, s, e - s + 1)) {
    Serial.println("ISS: JSON-Fehler");
    return;
  }

  double issLat = doc["iss_position"]["latitude"].as<double>();
  double issLon = doc["iss_position"]["longitude"].as<double>();
  double dist = haversine(MY_LAT, MY_LON, issLat, issLon);
  // double dist = 300.0;  // TEST 300, 600, 800, 1200

  Serial.print("Entfernung: ");
  Serial.print(dist);
  Serial.println(" km");

  if      (dist <= VISIBLE_RADIUS)      setMode(LED_PULSING_GREEN);
  else if (dist <= SLIGHTLY_FAR_RADIUS) setMode(LED_PULSING_YELLOW);
  else                                  setMode(LED_STEADY_RED);
}

void handleIssRequest() {
  if (!requestRunning) return;

  if (bodyComplete()) {                                // Daten komplett: auswerten, Verbindung selbst schließen
    requestRunning = false;
    processResponse();
    aClient->close();                                  // sauber schließen, Objekt bleibt bestehen
    return;
  }

  if (requestFinished) {                               // Verbindung zu Ende, aber keine vollständigen Daten
    requestRunning = false;
    Serial.println("ISS: Verbindung ohne gueltige Daten beendet");
    return;
  }

  if (millis() - requestStart > REQUEST_TIMEOUT_MS) {
    requestRunning = false;
    Serial.print("ISS: Timeout, empfangene Bytes: ");
    Serial.println(rxLen);
    aClient->close(true);                              // nur schließen, NICHT löschen
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println(ESP.getResetInfo());                  // zeigt den Grund des letzten Resets
  Serial.print("Free heap: ");
  Serial.println(ESP.getFreeHeap());

  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  analogWriteRange(PWM_MAX);
  analogWriteFreq(1000);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
    delay(250);
  }
  Serial.println(WiFi.status() == WL_CONNECTED ? "\nConnected!" : "\nWiFi failed");

  setupAsyncClient();
  startIssRequest();                                   // erster Abruf sofort
}

void loop() {
  updateOutputs();
  handleIssRequest();

  static unsigned long lastCall = 0;
  if (!requestRunning && millis() - lastCall >= HTTP_INTERVAL_MS) {
    lastCall = millis();
    startIssRequest();
  }
}