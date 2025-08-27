/*-
 * Copyright (c) 2025 Thomas Müller
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/************************************************************************************
 * Dual-PID Controller Firmware (Zeitproportional)
 * Version (Siehe unten im Code)
 *
 * Dieses Programm steuert Wasser- und Dampftemperatur über zwei unabhängige PID-Regler
 * mit zeitproportionaler Ansteuerung der SSRs.
 * Sensoren: MAX6675 (Dampf), NTC (Wasser).
 * Funktionen: Eco-Modus, Profile, Fast-Heat-Up, WLAN-Weboberfläche, AutoTune,
 * Wartungserinnerung, Werkseinstellungen, Hostname-Konfiguration.
 *
 * Compiler- und Hardware-Umgebung:
 * - ESP8266: Wemos D1 mini, 4MB
 * - ESP32: D1 mini
 ************************************************************************************/
 
/************************************************************************************
 * WAAGEN-KONFIGURATION
 ************************************************************************************/
#define SCALE_I2C 1
#define SCALE_ESPNOW 2

/************************************************************************************
 * NACH BEDARF ANPASSEN - HARDWARE-SPEZIFIKATIONEN
 ************************************************************************************/
// Die Waage auskommentieren, welche NICHT verwendet wird!
#define SCALE_TYPE SCALE_ESPNOW
// #define SCALE_TYPE SCALE_I2C

// Nachfolgende Zeile auskommentieren, falls die Steuerung ohne Display verwendet wird!
#define ENABLE_DISPLAY

/************************************************************************************
 * Vorwärtsdeklarationen benutzerdefinierter Strukturen
 * Erforderlich, damit die Arduino-Präprozessor-Prototypen gültige Typen sehen
 ************************************************************************************/
struct WiFiConfig;
struct TemperatureProfile;
struct ShotStats;

/************************************************************************************
 * Includes & Bibliotheken
 ************************************************************************************/
#include <Wire.h>

#ifdef ENABLE_DISPLAY
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#endif // ENABLE_DISPLAY

#include <EEPROM.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>  // Modifizierte AutoTune-Funktion - Muss als .zip eingebunden werden
#include "max6675.h"
#include <math.h>      // Für isnan()
#include <FS.h>        // Dateisystem-Basis
#include <LittleFS.h>  // LittleFS-Implementierung (für ESP32 ggf. explizit nötig)
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <time.h>  // Für Zeitfunktionen (struct tm, mktime, etc.)
#include <vector> // Erforderlich für std::vector
#include <pgmspace.h>

// Plattformspezifische Includes
// Für asynchronen Webserver
#ifdef ESP32
  #include <WiFi.h>         // ESP32 WiFi Core
  #include <AsyncTCP.h>     // TCP Stack für AsyncWebServer
  #include <ESPAsyncWebServer.h> // Asynchroner Webserver
  #include <ESPmDNS.h>      // ESP32 mDNS Implementation
  #include <Update.h>       // Für Firmware-Updates
  #include <esp_adc_cal.h>  // Für ADC-Kalibrierung
  #define LED_BUILTIN 2     // Standard LED Pin für viele ESP32 Boards (optional, falls genutzt)
  
  // --- KONDITIONALE WAAGEN-INCLUDES ---
  #if (SCALE_TYPE == SCALE_I2C)
    #include "UNIT_SCALES.h" // Für I2C-Waage
  #elif (SCALE_TYPE == SCALE_ESPNOW)
    #include <esp_now.h>     // Für ESP-NOW-Waage
  #endif
  // --- ENDE KONDITIONALE WAAGEN-INCLUDES ---

#else // ESP8266
  #include <ESP8266WiFi.h>
  #include <ESPAsyncTCP.h>       // TCP Stack für AsyncWebServer
  #include <ESPAsyncWebServer.h> // Asynchroner Webserver
  #include <ESP8266mDNS.h>
  // LED_BUILTIN ist für ESP8266 meistens vordefiniert (oft 2 oder DX Pin)
  #ifndef LED_BUILTIN
  #define LED_BUILTIN 2  // Fallback falls nicht vordefiniert
  #endif
#endif // Ende des #ifdef ESP32 / #else Blocks

/************************************************************************************
 * Webserver und zugehörige Objekte
 ************************************************************************************/
// Gemeinsamer asynchroner Webserver für ESP32 und ESP8266
AsyncWebServer asyncServer(80);


unsigned long scheduledRestart = 0;
void scheduleRestart(uint32_t delayMs) {
  scheduledRestart = millis() + delayMs;
}


/************************************************************************************
 * Hardware-Pinbelegung (Plattformabhängig)
 ************************************************************************************/

#ifdef ESP32
  // --- ESP32 Pinout ---
#define SSR_WASSER_PIN 21  // SSR für Wasser-Heizung (GPIO21)
#define SSR_DAMPF_PIN 22   // SSR für Dampf-Heizung (GPIO22)
#define SHOT_TIMER_PIN 5   // Input für Shot-Timer (GPIO5)
#define ANALOG_NTC_PIN 36  // Analoger Eingang für NTC (ADC1_CH0 - VP) - Nur ADC1 Pins (GPIO 32-39) sind sicher im WiFi Betrieb!
#define IIC_5V_SDA 17       // SDA-Leitung für OLED (GPIO17)
#define IIC_5V_SCK 16       // SCK-Leitung für OLED (GPIO16)

 // Piezo für ESP
#define PIEZO_PIN 33    // Piezo-Buzzer an GPIO35

 // Pins für Pumpe und Ventil (Nur ESP32)
#define PUMP_PIN 4      // SSR für Pumpe (GPIO4)
#define VALVE_PIN 2     // SSR für Magnetventil (GPIO2)

// MAX6675 (Dampf-Temperatursensor) - SPI Pins für ESP32
#define MAX6675_SCK 18  // Serial Clock (GPIO23) - Standard HSPI SCK
#define MAX6675_CS 19   // Chip Select (GPIO19) - Frei wählbar
#define MAX6675_SO 23   // Serial Out (GPIO18) - Standard HSPI MISO (oft als 'SDO' oder 'SO' bezeichnet) \
                        // Mosi (SDI) vom MAX6675 wird nicht benötigt, da nur gelesen wird.

#else                      // ESP8266 \
                           // --- ESP8266 Pinout (Original) ---
#define SSR_WASSER_PIN D2  // SSR für Wasser-Heizung
#define SSR_DAMPF_PIN D1   // SSR für Dampf-Heizung
#define SHOT_TIMER_PIN D8  // Input für Shot-Timer
#define ANALOG_NTC_PIN A0  // Analoger Eingang für den NTC-Sensor

// ENABLE_DISPLAY
#ifdef ENABLE_DISPLAY
  #define IIC_5V_SDA D3       // SDA-Leitung für OLED
  #define IIC_5V_SCK D4       // SCK-Leitung für OLED
#endif 

// MAX6675 (Dampf-Temperatursensor) - SPI Pins für ESP8266
#define MAX6675_SCK D7     // Serial Clock (GPIO13 / D7) - Kann auch D5 (GPIO14/SCK) sein
#define MAX6675_CS D6      // Chip Select (GPIO12 / D6) - Kann auch D8 (GPIO15/CS) sein
#define MAX6675_SO D5      // Serial Out (GPIO14 / D5) - Kann auch D6 (GPIO12/MISO) sein
#define A0 A0              // Explizite Definition für ESP8266 falls analogRead(A0) direkt verwendet wird
#endif

/************************************************************************************
 * MAX6675 (Dampf-Temperatursensor)
 ************************************************************************************/
// Die Pins werden oben plattformabhängig definiert.
MAX6675 thermocouple(MAX6675_SCK, MAX6675_CS, MAX6675_SO);

/************************************************************************************
 * Ultraschallsensor (Wasserstand) - I2C Variante
 ************************************************************************************/
// --- I2C Adresse Ultraschallsensor ---
// !!! WICHTIG: Adresse prüfen und ggf. anpassen! 0x57 ist eine häufige, aber nicht garantierte Adresse. !!!
#define ULTRASONIC_I2C_ADDR 0x57
// --- Globale Variablen für Ultraschallsensor ---
bool ultrasonicEnabled = false;           // Ist die Messung per Ultraschall aktiviert?
bool ultrasonicSensorConnected = false;   // Wird automatisch beim Start geprüft
float tankHeightCm = 20.0;                // Höhe des Tanks in cm (Beispielwert, konfigurierbar)
float waterLevelCm = -1.0;                // Berechneter Wasserstand in cm (-1 = ungültig/nicht gemessen)
float minLevelWarnCm = 5.0;               // Minimaler Wasserstand für Warnung (Beispielwert, konfigurierbar)
unsigned long lastUltrasonicReadTime = 0; // Zeitpunkt der letzten Messung
const unsigned long ULTRASONIC_READ_INTERVAL = 2000; // Intervall für Messung in ms (z.B. alle 2 Sekunden)
const unsigned int ULTRASONIC_MIN_DIST_MM = 180; // 18 cm Mindestabstand laut Datenblatt
const unsigned int ULTRASONIC_MAX_DIST_MM = 6000; // 600 cm Maximalabstand laut Datenblatt

/************************************************************************************
 * Display-Objekt (Adafruit SH1106G)
 ************************************************************************************/
#ifdef ENABLE_DISPLAY
/************************************************************************************
 * Display-Objekt (Adafruit SH1106G)
 ************************************************************************************/
// Dieser Block muss vorhanden sein, wenn Display aktiviert!
Adafruit_SH1106G display = Adafruit_SH1106G(128, 64, &Wire);
#endif // ENABLE_DISPLAY

/************************************************************************************
 * Firmware-Informationen
 ************************************************************************************/

String version = "3.6.2";
String versionHersteller = "Thomas M&uuml;ller";
String versionHerstellerMail =
  "<a href='mailto:thomas@mueller.black' class='info-link'>thomas@mueller.black</a>";
String versionHerstellerWeb =
  "<a href='https://raw-designs.de/' target='_blank' class='info-link'>https://raw-designs.de/</a> | "
  "<a href='https://mueller.black/' target='_blank' class='info-link'>https://mueller.black/</a>";
String versionHerstellerGitHub =
  "<a href='https://github.com/thomas-michael-mueller/dual-pid' target='_blank' class='info-link'>https://github.com/thomas-michael-mueller/dual-pid</a>";

// Demo-Modus: Hier können bestimmte Funktionen (WLAN-Anpassungen etc.) eingeschränkt werden.
// Entweder hier pauschal auf true setzen oder über IP-Adresse im Setup()-Teil auf true setzen lassen.
bool demoModus = false;
IPAddress demoModusIP(192, 168, 178, 88);

// Dieses Kennwort muss in einer Firmware-Binärdatei vorhanden sein, damit ein Update gültig ist.
static const char FIRMWARE_PASSWORD[] = "FWKennwort123";
static bool passwordFound = false;
static int passMatchPos = 0;
const int passLength = sizeof(FIRMWARE_PASSWORD) - 1;

/************************************************************************************
 * Geräte-Infos
 ************************************************************************************/

char infoHersteller[50];
char infoModell[50];
char infoZusatz[50];

/************************************************************************************
 * NTC-Sensor-Parameter (Wasser)
 * - A0 wird verwendet, um die Temperatur über einen Spannungsteiler zu messen
 ************************************************************************************/

const double V_SUPPLY = 3.3;    // Versorgungsspannung
const double ADC_REF = 3.3;     // ADC-Referenz
const double R_FIXED = 3000.0;  // Festwiderstand 3 kΩ

// Daten des NTC
const double NTC_T0 = 298.15;   // Referenztemperatur in Kelvin (25°C)
const double NTC_R0 = 50000.0;  // NTC-Widerstand bei 25°C
const double beta = 3950.0;     // Beta-Wert

// Globale Variable für ADC-Kalibrierungsdaten
#ifdef ESP32
static esp_adc_cal_characteristics_t adc_chars;
#endif

/************************************************************************************
 * Waagen-Instanz & Variablen (Konditional)
 ************************************************************************************/
#ifdef ESP32
// --- Gemeinsame Waagen-Variablen ---
bool scaleConnected = false;      // Zeigt an, ob eine Waage verbunden ist (egal welcher Typ)
bool scaleModeActive = false;     // Für Anzeige der Waage im Display
volatile float currentWeightReading = 0.0f; // Aktueller Messwert der Waage
volatile float brewStartWeight = 0.0f;
const float WEIGHT_CHANGE_THRESHOLD = 0.1f; // Schwelle in g für die erste Gewichtsänderung
unsigned long firstWeightChangeTime = 0;     // Zeitpunkt der ersten Gewichtsänderung
bool firstWeightChangeDetected = false;      // Flag, ob bereits eine Änderung erkannt wurde
unsigned long lastShotFirstWeightChangeTime = 0; // Gespeicherter Zeitpunkt für den letzten Shot

#if (SCALE_TYPE == SCALE_I2C)
  // --- I2C-spezifische Variablen ---
  UNIT_SCALES scales; // Instanz für die I2C-Waage
  
  // Button-Logik für die I2C-Waage
  unsigned long scaleButtonPressStartTime = 0;
  bool scaleButtonIsCurrentlyPressed = false;
  uint8_t lastScaleButtonRawState = 1;
  const unsigned long longPressThresholdScale = 1000;
  const unsigned long debounceDelayScale = 50;
  unsigned long lastScaleButtonDebounceTime = 0;
  uint8_t currentScaleButtonDebouncedState = 1;
  uint8_t lastScaleButtonDebouncedState = 1;

  // Verzögertes Tarieren für die I2C-Waage
  bool tareScaleAfterDelay = false;
  unsigned long tareScaleDelayStartTime = 0;
  const unsigned long TARE_SCALE_DELAY_DURATION = 1000;

  // Nachlaufzeit nach dem Tarieren (nicht-blockierend)
  bool tareScaleSettling = false;
  unsigned long tareScaleSettlingStartTime = 0;
  const unsigned long TARE_SCALE_SETTLING_DURATION = 50;

#elif (SCALE_TYPE == SCALE_ESPNOW)
  // --- ESP-NOW-spezifische Variablen ---
  
  // Datenstruktur für empfangene Nachrichten (muss mit dem Sender übereinstimmen)
  typedef struct struct_espnow_scale_message {
    float weight_g;
    uint8_t status_flags;
    uint8_t battery_percentage;
  } struct_espnow_scale_message;

  // Flags für die Status-Nachrichten
  #define ESPNOW_SCALE_FLAG_JUST_TARED  (1 << 0)
  #define ESPNOW_SCALE_FLAG_TOGGLE_MODE (1 << 1)
  #define ESPNOW_SCALE_FLAG_AWOKE       (1 << 2)

  // Timeout-Handling
  unsigned long lastScaleMessageTime = 0;
  const unsigned long SCALE_TIMEOUT = 5000; // 5 Sekunden Timeout

#endif // Ende SCALE_TYPE Check
#endif // Ende ESP32 Check

/************************************************************************************
 * Temperatur-Profil-Struktur (ERWEITERT)
 * - Speichert nahezu alle konfigurierbaren Einstellungen pro Profil
 ************************************************************************************/
// VERSIONSNUMMER
const uint16_t CURRENT_PROFILE_VERSION = 3; // Versionsnummer für die Struktur

struct TemperatureProfile {
    // --- Identifikation & Version ---
    uint16_t profileVersion; // Zur Kompatibilitätsprüfung beim Laden
    char profileName[31];    // Angezeigter Name (etwas mehr Platz + Nullterminator)

    // --- PID Einstellungen (Seite: /) ---
    double setpointWasser;
    double setpointDampf;
    double offsetWasser;
    double offsetDampf;
    double kpWasser;
    double kiWasser;
    double kdWasser;
    double kpDampf;
    double kiDampf;
    double kdDampf;
    bool boostWasserActive;
    bool boostDampfActive;
    bool preventHeatAboveSetpointWasser;
    bool preventHeatAboveSetpointDampf;
    unsigned long windowSizeWasser;
    unsigned long windowSizeDampf;
    double maxTempWasser;
    double maxTempDampf;

    // --- PID Tuning Parameter (Seite: /PID-Tuning) ---
    double tuningStepWasser;
    double tuningNoiseWasser;
    double tuningStartValueWasser;
    unsigned int tuningLookBackWasser;
    double tuningStepDampf;
    double tuningNoiseDampf;
    double tuningStartValueDampf;
    unsigned int tuningLookBackDampf;

    // --- ECO Modus (Seite: /ECO) ---
    int ecoModeMinutes;
    int ecoModeTempWasser;
    int ecoModeTempDampf;
    bool dynamicEcoActive;
    int dampfVerzoegerung; // Dampfverzögerung ist auf ECO-Seite
    bool steamDelayOverrideBySwitch;

    // --- Fast-Heat-Up (Seite: /Fast-Heat-Up) ---
    bool fastHeatUpAktiv;

    // --- Brew Control Einstellungen
    bool brewByTimeEnabled;
    float brewByTimeTargetSeconds; // Geändert
    bool preInfusionEnabled;
    float preInfusionDurationSeconds; // Geändert
    float preInfusionPauseSeconds;    // Geändert
    bool brewByWeightEnabled;
    float brewByWeightTargetGrams;
    float brewByWeightOffsetGrams; // Offset zum früheren Stoppen

    // --- Systemtöne / PIEZO ---
    bool piezoEnabled; // Status der Piezo-Töne
};

/************************************************************************************
 * PID-Regler-Konfiguration & Zeitproportionale Steuerung
 ************************************************************************************/

// PID Input/Output/Setpoint Variablen
double SetpointDampf, InputDampf, OutputDampf;
double SetpointWasser, InputWasser, OutputWasser;
double OffsetDampf = 0.0, OffsetWasser = 0.0;

// PID-Konstanten (Regelungsparameter)
double KpDampf = 3.0, KiDampf = 0.1, KdDampf = 0.5;
double KpWasser = 5.0, KiWasser = 0.2, KdWasser = 1.0;

// Zeitproportionale Steuerung
unsigned long windowSizeWasser;       // Fenstergröße für Wasser-SSR (in ms)
unsigned long windowSizeDampf;        // Fenstergröße für Dampf-SSR (in ms)
unsigned long windowStartTimeWasser;  // Startzeit des aktuellen Fensters für Wasser
unsigned long windowStartTimeDampf;   // Startzeit des aktuellen Fensters für Dampf

// PID-Regler Instanzen
// WICHTIG: Die Output-Grenzen werden in setup() auf die Fenstergröße gesetzt!
PID pidDampf(&InputDampf, &OutputDampf, &SetpointDampf, KpDampf, KiDampf, KdDampf, DIRECT);
PID pidWasser(&InputWasser, &OutputWasser, &SetpointWasser, KpWasser, KiWasser, KdWasser, DIRECT);

// Variablen für Sicherheitsabschaltung
double maxTempWasser = 110.0;  // Standardwert, wie bisher hardcoded
double maxTempDampf = 180.0;   // Standardwert, wie bisher hardcoded

// --- NTP Client Konfiguration ---
WiFiUDP ntpUDP;
// Zeitverschiebung UTC+1 (Berlin/CET), automatische Sommerzeit (CEST) wird über TZ_INFO gehandhabt
// Offset 0 hier, da TZ_INFO die Regeln enthält. Update-Intervall 60000ms = 1 Minute.
NTPClient timeClient(ntpUDP, "pool.ntp.org", 0, 60000);
bool timeSynced = false;

// Zeitzonen-String für Deutschland (CET/CEST)
const char* TZ_INFO = "CET-1CEST,M3.5.0,M10.5.0/3";  // Germany

// --- Für Shot-Statistik ---
struct ShotStats {
  unsigned long totalShotsLogged = 0;
  unsigned long totalDurationMs = 0;
  unsigned long shotsToday = 0;
  unsigned long shotsThisWeek = 0;
  unsigned long shotsThisMonth = 0;
  double averageDurationSec = 0.0;
  time_t firstShotTimestamp = 0;
  time_t lastShotTimestamp = 0;
  bool historyAvailable = false;  // Flag, ob Daten gelesen werden konnten
  unsigned long shotsYesterday = 0;
  unsigned long shotsLastWeek = 0;   // Kalenderwoche
  unsigned long shotsLastMonth = 0;  // Kalendermonat
  double avgShotsPerDay = 0.0;       // Durchschnitt seit erstem Log
};

/************************************************************************************
 * Brew Control Variablen & Defaults
 ************************************************************************************/
#ifdef ESP32
enum PreInfusionState { PI_INACTIVE, PI_PRE_BREW, PI_PAUSE, PI_MAIN_BREW };
PreInfusionState currentPreInfusionState = PI_INACTIVE;
unsigned long preInfusionPhaseStartTime = 0;
#endif

// Globale Variablen für den aktuellen Zustand
bool brewByTimeEnabled = false;
float brewByTimeTargetSeconds = 30.0f; 
bool preInfusionEnabled = false;
float preInfusionDurationSeconds = 5.0f; 
float preInfusionPauseSeconds = 3.0f;  
bool brewByWeightEnabled = false;
float brewByWeightTargetGrams = 36.0f;
float brewByWeightOffsetGrams = 0.0f; 
// Die Variable "currentWeightReading" ist jetzt im konditionalen Waagen-Block definiert.

// Standard-Werte für Brew-Control
const bool defaultBrewByTimeEnabled = false;
const float defaultBrewByTimeTargetSeconds = 30.0f;
const bool defaultPreInfusionEnabled = false;
const float defaultPreInfusionDurationSeconds = 5.0f;
const float defaultPreInfusionPauseSeconds = 3.0f;
const bool defaultBrewByWeightEnabled = false;
const float defaultBrewByWeightTargetGrams = 36.0f;
const float defaultBrewByWeightOffsetGrams = 0.0f; // Standard kein Offset

/************************************************************************************
 * Standardwerte und Default-Einstellungen
 ************************************************************************************/

double defaultSetpointDampf = 165.0;
double defaultSetpointWasser = 93.0;
double defaultOffsetDampf = 0.0;
double defaultOffsetWasser = 0.0;
double defaultKpDampf = 3.0;
double defaultKiDampf = 0.1;
double defaultKdDampf = 0.5;
double defaultKpWasser = 5.0;
double defaultKiWasser = 0.2;
double defaultKdWasser = 1.0;
int defaultEcoModeMinutes = 0;
int defaultEcoModeTempWasser = 60;
int defaultEcoModeTempDampf = 60;
String defaultInfoHersteller = "";
String defaultInfoModell = "";
String defaultInfoZusatz = "";
int defaultDampfVerzoegerung = 0;
int dampfVerzoegerung = 0;
int defaultMaintenanceInterval = 0;
String defaultHostname = "Dual-PID";
unsigned long defaultWindowSizeWasser = 2000;
unsigned long defaultWindowSizeDampf = 2000;
const bool defaultSteamDelayOverrideBySwitchEnabled = false; 
bool steamDelayOverrideBySwitchEnabled = false;

// MagicValue zum Erkennen bereits gespeicherter EEPROM-Daten
const char storageMagicValue[5] = "MGVE";

/************************************************************************************
 * EEPROM Adressen für das Speichern/Laden von Werten
 * KORRIGIERT: Adressen ab Hostname / Windowsize angepasst (Overlap entfernt)
 * Kommentare für Endadressen überprüft/korrigiert
 ************************************************************************************/
const int EEPROM_SIZE = 1024;  // Gesamtgröße des verwendeten EEPROM-Speichers

// --- Allgemeine Einstellungen ---
const int EEPROM_ADDR_MAGICVALUE = 0;                     // 5 Bytes (char[5]) -> Ende 4
const int EEPROM_ADDR_SETPOINT_DAMPF = 5;                 // 8 Bytes (double) -> Ende 12
const int EEPROM_ADDR_SETPOINT_WASSER = 13;               // 8 Bytes -> Ende 20
const int EEPROM_ADDR_OFFSET_DAMPF = 21;                  // 8 Bytes -> Ende 28
const int EEPROM_ADDR_OFFSET_WASSER = 29;                 // 8 Bytes -> Ende 36
const int EEPROM_ADDR_KP_DAMPF = 37;                      // 8 Bytes -> Ende 44
const int EEPROM_ADDR_KI_DAMPF = 45;                      // 8 Bytes -> Ende 52
const int EEPROM_ADDR_KD_DAMPF = 53;                      // 8 Bytes -> Ende 60
const int EEPROM_ADDR_KP_WASSER = 61;                     // 8 Bytes -> Ende 68
const int EEPROM_ADDR_KI_WASSER = 69;                     // 8 Bytes -> Ende 76
const int EEPROM_ADDR_KD_WASSER = 77;                     // 8 Bytes -> Ende 84
const int EEPROM_ADDR_ECOMODE_MINUTES = 85;               // 4 Bytes (int) -> Ende 88
const int EEPROM_ADDR_ECOMODE_TEMP_WASSER = 89;           // 4 Bytes -> Ende 92
const int EEPROM_ADDR_ECOMODE_TEMP_DAMPF = 93;            // 4 Bytes -> Ende 96
const int EEPROM_ADDR_INFO_HERSTELLER = 97;               // 50 Bytes (char[50]) -> Ende 146
const int EEPROM_ADDR_INFO_MODELL = 147;                  // 50 Bytes -> Ende 196
const int EEPROM_ADDR_INFO_ZUSATZ = 197;                  // 50 Bytes -> Ende 246
const int EEPROM_ADDR_FASTHEATUP_DATA = 247;              // 1 Byte (bool) -> Ende 247
const int EEPROM_ADDR_RUNTIME = 248;                      // 4 Bytes (unsigned long) -> Ende 251
const int EEPROM_ADDR_SHOTCOUNTER = 252;                  // 4 Bytes (unsigned long) -> Ende 255
const int EEPROM_ADDR_DYNAMIC_ECO_MODE = 256;             // 1 Byte (bool) -> Ende 256
// --- AutoTune Parameter Wasser --- (Start bei 357 -> Lücke vorhanden)
const int EEPROM_ADDR_TUNING_STEP_WASSER = 357;           // 8 Bytes (double) -> Ende 364
const int EEPROM_ADDR_TUNING_NOISE_WASSER = 365;          // 8 Bytes (double) -> Ende 372
const int EEPROM_ADDR_TUNING_STARTVALUE_WASSER = 373;     // 8 Bytes (double) -> Ende 380
const int EEPROM_ADDR_TUNING_LOOKBACK_WASSER = 381;       // 4 Bytes (unsigned int) -> Ende 384
// --- WiFi Konfiguration (Feldweise) ---
const int EEPROM_ADDR_WIFI_CONFIG_MAGIC = 385;            // 5 Bytes (char[5]) -> Ende 389
const int EEPROM_ADDR_WIFI_SSID         = 390;            // 32 Bytes -> Ende 421
const int EEPROM_ADDR_WIFI_PASSWORD     = 422;            // 64 Bytes -> Ende 485
const int EEPROM_ADDR_WIFI_USE_STATIC   = 486;            // 1 Byte   -> Ende 486
const int EEPROM_ADDR_WIFI_STATIC_IP    = 487;            // 4 Bytes -> Ende 490
const int EEPROM_ADDR_WIFI_GATEWAY      = 491;            // 4 Bytes -> Ende 494
const int EEPROM_ADDR_WIFI_SUBNET       = 495;            // 4 Bytes -> Ende 498
const int EEPROM_ADDR_WIFI_DNS          = 499;            // 4 Bytes -> Ende 502
// --- Sonstige Einstellungen (Lücke vorhanden) ---
const int EEPROM_ADDR_STEAM_DELAY = 527;                  // 4 Bytes (int) -> Ende 530
// --- AutoTune Parameter Dampf ---
const int EEPROM_ADDR_TUNING_STEP_DAMPF = 531;            // 8 Bytes (double) -> Ende 538
const int EEPROM_ADDR_TUNING_NOISE_DAMPF = 539;           // 8 Bytes (double) -> Ende 546
const int EEPROM_ADDR_TUNING_STARTVALUE_DAMPF = 547;      // 8 Bytes (double) -> Ende 554
const int EEPROM_ADDR_TUNING_LOOKBACK_DAMPF = 555;        // 4 Bytes (unsigned int) -> Ende 558
// --- Adressen für Übertemperatursicherung ---
const int EEPROM_ADDR_MAX_TEMP_WASSER = 559;              // 8 Bytes (double) -> Ende 566
const int EEPROM_ADDR_MAX_TEMP_DAMPF = 567;               // 8 Bytes (double) -> Ende 574
// Zusätzliche EEPROM-Adressen
const int EEPROM_ADDR_MAINTENANCE_INTERVAL = 575;         // 4 Bytes (int) -> Ende 578
const int EEPROM_ADDR_MAINTENANCE_INTERVAL_COUNTER = 579; // 4 Bytes (int) -> Ende 582 // Kommentar korrigiert
const int EEPROM_ADDR_HOSTNAME = 583;                     // 33 Bytes (char[33]) -> Ende 615

// --- KORRIGIERTE / VERSCHOBENE ADRESSEN (Start nach Hostname) ---
const int EEPROM_ADDR_WINDOWSIZE_WASSER = 616;            // WAR 614 -> 4 Bytes (unsigned long) -> Ende 619
const int EEPROM_ADDR_WINDOWSIZE_DAMPF = 620;             // WAR 618 -> 4 Bytes (unsigned long) -> Ende 623
const int EEPROM_ADDR_BOOST_WASSER_ACTIVE = 624;          // WAR 612 -> 1 Byte (bool) -> Ende 624
const int EEPROM_ADDR_BOOST_DAMPF_ACTIVE = 625;           // WAR 613 -> 1 Byte (bool) -> Ende 625
const int EEPROM_ADDR_PREVENTHEAT_WASSER = 626;           // WAR 622 -> 1 Byte (bool) -> Ende 626
const int EEPROM_ADDR_PREVENTHEAT_DAMPF = 627;            // WAR 623 -> 1 Byte (bool) -> Ende 627
// --- BREW CONTROL (Start nach Prevent Heat) ---
const int EEPROM_ADDR_BREWBYTIME_ENABLED = 628;           // WAR 624 -> 1 Byte (bool) -> Ende 628
// ACHTUNG: brewByTimeTargetSeconds ist float (4 Bytes), nicht unsigned long
const int EEPROM_ADDR_BREWBYTIME_SECONDS = 629;           // WAR 625 -> 4 Bytes (float) -> Ende 632
const int EEPROM_ADDR_PREINF_ENABLED = 633;               // WAR 629 -> 1 Byte (bool) -> Ende 633
// ACHTUNG: preInfusionDurationSeconds/preInfusionPauseSeconds sind float (4 Bytes)
const int EEPROM_ADDR_PREINF_DUR_SEC = 634;               // WAR 630 -> 4 Bytes (float) -> Ende 637
const int EEPROM_ADDR_PREINF_PAUSE_SEC = 638;             // WAR 634 -> 4 Bytes (float) -> Ende 641
const int EEPROM_ADDR_BREWBYWEIGHT_ENABLED = 642;         // WAR 638 -> bool (1 byte) -> Ende 642
const int EEPROM_ADDR_BREWBYWEIGHT_TARGET  = 643;         // WAR 639 -> float (4 bytes) -> Ende 646
const int EEPROM_ADDR_BREWBYWEIGHT_OFFSET  = 647;         // WAR 643 -> float (4 bytes) -> Ende 650
// --- PIEZO (Start nach Brew By Weight) ---
const int EEPROM_ADDR_PIEZO_ENABLED = 651;                // WAR 647 -> 1 Byte (bool) -> Ende 651
// --- Dampfverzögerung über ECO-Einstellungen temporär überspringen
const int EEPROM_ADDR_STEAM_DELAY_OVERRIDE_SWITCH = 652;  // 1 Byte (bool) -> Ende 652
// Nächste freie Adresse: 653 (Innerhalb EEPROM_SIZE=1024)

/************************************************************************************
 * Eco-Mode Variablen
 ************************************************************************************/

unsigned long lastShotTime = 0;  // Wann wurde zuletzt ein Shot beendet
int ecoModeMinutes = 0;          // Zeit (Minuten) bis Eco-Modus
bool ecoModeAktiv = 0;           // Flag, ob Eco läuft
int ecoModeTempWasser = 50;      // Zieltemp Wasser im Eco
int ecoModeTempDampf = 50;       // Zieltemp Dampf im Eco
bool dynamicEcoActive = false;
unsigned long ecoModeActivatedTime = 0;

/************************************************************************************
 * Wartungsmodus Variable
 ************************************************************************************/
bool wartungsModusAktiv = false;  // Ist der Reinigungs-/Wartungsmodus aktiv?
int wartungsModusTemp = 35;       // Zieltemp Wasser und Dampf im Wartungsmodus

/************************************************************************************
 * Fast-Heat-Up Variablen
 ************************************************************************************/

bool fastHeatUpAktiv = 0;    // Ist Fast-Heat-Up aktiviert?
bool fastHeatUpHeating = 0;  // Wird gerade hochgeheizt (Wasser >130)?
int fastHeatUpSetpoint = 130;

/************************************************************************************
 * Sicherheits- und Fehlerstatus
 ************************************************************************************/

bool wasserSensorError = false;  // Flag für Fehler am Wassertemperatursensor
bool dampfSensorError = false;   // Flag für Fehler am Dampftemperatursensor
bool wasserSafetyShutdown = false;
bool dampfSafetyShutdown = false;

/************************************************************************************
 * Auto-Tune-Einstellungen
 ************************************************************************************/

// Parameter für Wasser
double tuningStepWasser;
double tuningNoiseWasser;
double tuningStartValueWasser;
unsigned int tuningLookBackWasser;
const double defaultTuningStepWasser = 500.0;
const double defaultTuningNoiseWasser = 1.0;
const double defaultTuningStartValueWasser = 1000.0;
const unsigned int defaultTuningLookBackWasser = 60;

// Parameter für Dampf
double tuningStepDampf;
double tuningNoiseDampf;
double tuningStartValueDampf;
unsigned int tuningLookBackDampf;
const double defaultTuningStepDampf = 300.0;
const double defaultTuningNoiseDampf = 1.0;
const double defaultTuningStartValueDampf = 1000.0;
const unsigned int defaultTuningLookBackDampf = 30;
// Globale Variable für temporäre Dampfverzögerungs-Überbrückung
bool steamDelayOverridden = false;

PID_ATune* autoTuneWasser;
PID_ATune* autoTuneDampf;
bool autoTuneWasserActive = false;
bool autoTuneDampfActive = false;

// Boost-Feature
bool boostWasserActive = false;
const bool defaultBoostWasserActive = false;
bool boostDampfActive = false;
const bool defaultBoostDampfActive = false;

// Heizen oberhalb Setpoint verhindern
bool preventHeatAboveSetpointWasser = false;  // Standardmäßig AUS
const bool defaultPreventHeatAboveSetpointWasser = false;
bool preventHeatAboveSetpointDampf = false;  // Standardmäßig AUS
const bool defaultPreventHeatAboveSetpointDampf = false;

/************************************************************************************
 * Shot-Timer Variablen
 ************************************************************************************/

unsigned long shotStartTime = 0;
unsigned long shotEndTime = 0;
unsigned long lastShotStartTime = 0; // Startzeit des letzten Shots
bool shotActive = false;
unsigned long lastShotDurationMillis = 0;  // Speichert die Dauer des letzten Shots in ms
bool manualSwitchReleasedAfterAutoStop = true; // Startwert: true, bereit für manuellen Start

/************************************************************************************
 * Shot-Zähler und Betriebszeit
 ************************************************************************************/

unsigned long shotCounter = 0;                                        // Shots > 20 Sekunden
unsigned long totalRuntime = 0;                                       // Gesamt-Betriebszeit in Sekunden
unsigned long lastRuntimeSave = 0;                                    // Wann zuletzt gespeichert
const unsigned long initialRuntimeSaveDelay = 10 * 60 * 1000UL;       // 10 Minuten
const unsigned long subsequentRuntimeSaveInterval = 5 * 60 * 1000UL;  // 5 Minuten
bool initialRuntimeSaveDone = false;
// VARIABLEN ZUM VERZÖGERTEN SPEICHERN VON SHOTS, NACH DESSEN BEZUG
bool shotNeedsToBeSaved = false;
unsigned long savedShotDuration = 0;
// --- Zusätzliche Variablen für das Speichern des Gewichts ---
bool savedShotWasByWeight = false;      // War der zu speichernde Shot ein "By-Weight"-Shot?
float savedShotFinalWeight = 0.0f;      // Das finale Gewicht des zu speichernden Shots

/************************************************************************************
 * Wartungserinnerung Variablen
 ************************************************************************************/
int maintenanceInterval = 0;                    // Intervall in Shots (0 = deaktiviert)
unsigned long maintenanceIntervalCounter = 0;   // Zähler für Wartungsintervall
bool displayMaintenanceMessage = false;         // Flag, ob Meldung aktiv angezeigt wird
unsigned long maintenanceMessageStartTime = 0;  // Startzeit der Meldungsanzeige

/************************************************************************************
 * Zustandsmaschine für die Anzeige nach dem Shot
 ************************************************************************************/
#define POST_SHOT_IDLE 0          // Standardzustand, nichts zu tun
#define POST_SHOT_SHOW_DURATION 1 // Zustand: Zeige Bezugsdauer an
#define POST_SHOT_SHOW_MAINTENANCE 2 // Zustand: Zeige Wartungsmeldung an
#define POST_SHOT_SHOW_WEIGHT_RESULT 3 // Zustand: Zeige Gewicht und Dauer an (für By-Weight)

uint8_t postShotDisplayState = POST_SHOT_IDLE; // Unsere neue Variable, die den aktuellen Zustand speichert
float lastShotFinalNetWeight = 0.0f;           // Globale Variable für das finale Nettogewicht

/************************************************************************************
 * Systemtöne / Piezo
 ************************************************************************************/
bool piezoEnabled = true; // Standardmäßig aktiviert
const bool defaultPiezoEnabled = true;

/************************************************************************************
 * Verzögerungen für Anzeigen
 ************************************************************************************/

int delayInit1 = 2000;               // Display-Bild am Anfang
int delayInit2 = 3000;               // Warte für Anzeige
unsigned long previousMillis = 0;    // für Intervall z.B. 250ms
const unsigned long interval = 250;  // PID-Berechnungsintervall

/************************************************************************************
 * LittleFS
 ************************************************************************************/
File fsUploadFile;  // Globale Variable für die Datei während des Uploads
String uploadStatusMessage = "";       // Statusmeldung für Upload-Ergebnisse
String uploadStatusClass = "";         // "status-success" oder "status-error"

/************************************************************************************
 * Hostname Variable
 ************************************************************************************/
char hostname[33] = "Dual-PID";  // Max 32 Zeichen + Nullterminator


/************************************************************************************
 * Profile
 ************************************************************************************/
String profileStatusMessage = ""; // Für Nachrichten auf der Profil-Seite

/************************************************************************************
 * WiFi Signal Bitmap (Display-Anzeige)
 ************************************************************************************/
#ifdef ENABLE_DISPLAY
static const unsigned char PROGMEM wifiSymbol[] = {
  0x1f, 0xc0, 0x20, 0x20, 0x4f, 0x90, 0x90, 0x48,
  0x27, 0x20, 0x08, 0x80, 0x02, 0x00
};
#endif // ENABLE_DISPLAY

/************************************************************************************
 * Gemeinsame CSS-Styles (PROGMEM) - Unverändert
 ************************************************************************************/

static const char commonStyle[] PROGMEM = R"rawliteral(
<style>
  /* Farben und Grundvariablen */
  :root {
    --primary-bg: #000000;     /* Schwarz */
    --secondary-bg: #1a1a1a;   /* Dunkles Grau/Schwarz */
    --accent-color: #d89904;   /* Gold-/Kupferton */
    --text-color: #ffffff;     /* Weiß für Schrift */
    --card-bg-opacity: 0.15;   /* Opazität für Form-Karten-Hintergrund */
  }

  /* Seitenhintergrund: Farbverlauf */
  body {
    margin: 0;
    padding: 0;
    font-family: "Segoe UI", Tahoma, Arial, sans-serif;
    background: linear-gradient(120deg, #000000 0%, #1a1a1a 100%);
    background-repeat: no-repeat;
    background-attachment: fixed;
    background-size: cover;
    min-height: 100vh;
    color: #ffffff;
  }

  /************************************************************************
   * Links / Anker-Tags
   ************************************************************************/
  a,
  a:visited {
    text-decoration: none;
    transition: color 0.2s ease, background-color 0.2s ease;
  }
  a:hover,
  a:focus {
    text-decoration: none;
  }

  .info-link,
  .info-link:visited {
      color: #d89904; /* Oder var(--accent-color), falls das die Akzentfarbe ist */
      text-decoration: none; /* Unterstreichung entfernen (wie schon bei 'a') */
      /* !important ist hier wahrscheinlich nicht mehr nötig, da die Klasse spezifischer ist als 'a' */
  }

  .info-link:hover {
      /* z.B. Helligkeit ändern oder doch unterstreichen */
      filter: brightness(1.2);
      /* text-decoration: underline; */
  }

  /************************************************************************
   * Navigation (Sticky Top Bar)
   ************************************************************************/
  nav {
    background: rgba(0, 0, 0, 0.5);
    backdrop-filter: blur(6px);
    padding: 10px 0;
    text-align: center;
    position: sticky;
    top: 0;
    /* <b style='color: #FFCC00;'>WICHTIG:</b>: z-index erhöhen, damit das X-Icon klickbar bleibt */
    z-index: 1000;
    position: relative; /* Basis für absolute Positionierung von Kindern */
  }

  /* Wrapper für die Navigationslinks */
  .nav-links {
    display: inline;
  }

  /* Desktop Navigationslinks */
  .nav-links a {
    color: #fff;
    font-weight: 500;
    text-decoration: none;
    margin: 0 10px;
    padding: 10px 20px;
    border-radius: 8px;
    transition: background 0.3s ease, color 0.3s ease;
    display: inline-block;
  }

  /* Desktop Hover/Focus/Active States */
  .nav-links a:hover,
  .nav-links a:focus,
  .nav-links a:active,
  .nav-links a.active {
    background: var(--accent-color);
    color: #000000;
  }

  /* Hamburger-Icon / Close-Icon Styling */
  .hamburger {
    display: none;
    font-size: 30px; /* Etwas größer für bessere Klickbarkeit */
    font-weight: bold; /* Macht das 'X' oft etwas klarer */
    line-height: 1;
    position: absolute;
    /* Anpassung - weiter nach unten verschoben */
    top: calc(50% + 25px); /* Passe '25px' nach Bedarf an */
    transform: translateY(-50%);
    right: 20px;
    color: var(--text-color);
    cursor: pointer;
    padding: 5px; /* Etwas Padding für größere Klickfläche */
    z-index: 1001; /* Sicherstellen, dass der Button über dem Overlay liegt */
  }
  .hamburger:hover {
    color: #ddd;
  }


  /************************************************************************
   * Formulare (Card-Design, Glas-Effekt)
   ************************************************************************/
  form {
    background: rgba(255, 255, 255, var(--card-bg-opacity));
    backdrop-filter: blur(8px);
    border-radius: 12px;
    padding: 20px;
    margin: 20px auto;
    width: 90%;
    max-width: 600px;
    box-shadow: 0 8px 24px rgba(0, 0, 0, 0.3);
    color: var(--text-color);
  }

  /* Überschriften */
  h1 {
    text-align: center;
    margin-top: 20px;
    font-size: 2rem;
  }
  h3 {
    margin-top: 0;
    font-weight: 600;
    padding-bottom: 3px;
    border-bottom: 1px solid rgba(255, 255, 255, 0.3);
  }

  /* Labels */
  label {
    display: block;
    margin: 10px 0 5px;
    font-weight: 600;
  }

  /************************************************************************
   * Eingabefelder (Inputs, Select, Textarea) MIT FOKUS-EFFEKT
   ************************************************************************/
  input[type="text"],
  input[type="password"],
  input[type="number"],
  select,
  textarea {
    width: auto;
    min-width: 200px;
    max-width: 300px;
    display: block;
    margin-bottom: 5px;
    background: rgba(0, 0, 0, 0.4);
    color: var(--text-color);
    border: 1px solid #555; /* Standard-Rand */
    border-radius: 8px;
    padding: 10px;
    box-shadow: inset 0 4px 8px rgba(0, 0, 0, 0.2);
    transition: background 0.2s ease, border-color 0.2s ease, box-shadow 0.2s ease;
  }

  /* Fokus-Effekt FÜR ALLE INPUTS/SELECT/TEXTAREA */
  input[type="text"]:focus,
  input[type="password"]:focus,
  input[type="number"]:focus,
  select:focus,
  textarea:focus {
    outline: none;
    background: rgba(0, 0, 0, 0.6);
    /* Orangener Rand und Schein bei Fokus */
    border-color: var(--accent-color, #d89904);
    box-shadow: 0 0 8px rgba(216, 153, 4, 0.3);
  }

 /* --- CSS zum Ausblenden der Pfeile bei number Inputs --- */
 input[type=number]::-webkit-outer-spin-button,
 input[type=number]::-webkit-inner-spin-button {
   -webkit-appearance: none;
   margin: 0;
 }
 input[type=number] {
   -moz-appearance: textfield;
 }

  /************************************************************************
   * Buttons
   ************************************************************************/
  input[type="submit"],
  button {
    background: var(--accent-color);
    color: #000000;
    border: none;
    border-radius: 30px;
    padding: 10px 20px;
    margin-top: 12px;
    width: auto;
    min-width: 100px;
    max-width: 200px;
    cursor: pointer;
    font-weight: 600;
    box-shadow: 0 4px 12px rgba(216, 153, 4, 0.3);
    transition: all 0.3s ease;
  }
  input[type="submit"]:hover,
  button:hover {
    opacity: 0.85;
    transform: translateY(-2px);
  }

  /* Style für Danger-Button (Werkseinstellungen) */
  button.danger {
      background: #dc3545; /* Roter Hintergrund */
      color: #ffffff; /* Weiße Schrift */
      box-shadow: 0 4px 12px rgba(220, 53, 69, 0.3); /* Roter Schatten */
  }
  button.danger:hover {
      background: #c82333; /* Dunkleres Rot bei Hover */
      opacity: 1; /* Opazität zurücksetzen, da Hintergrund dunkler wird */
      transform: translateY(-2px);
  }

  /************************************************************************
   * Fehlermeldungen / Sensorfehler
   ************************************************************************/
  .sensor-error-message {
    color: #FFCC00; /* Auffälliges Gelb/Gold für Warnungen im Dark Mode */
    text-align: center;
    margin-top: 5px;  /* Weniger Abstand nach oben */
    margin-bottom: 15px; /* Abstand zum Chart-Container */
    padding: 8px 10px;
    background-color: rgba(255, 204, 0, 0.1); /* Sehr dezenter gelber Hintergrund */
    border: 1px solid rgba(255, 204, 0, 0.2); /* Sehr dezenter gelber Rand */
    border-radius: 8px;
    font-weight: 500; /* Etwas fetter als normal */
    font-size: 0.9em;
    max-width: 600px; /* Verhindert zu breite Box auf großen Screens */
    margin-left: auto; /* Zentriert die Box selbst */
    margin-right: auto;
  }


  /************************************************************************
   * Responsive Design - MOBILE OVERLAY HIER IMPLEMENTIERT
   ************************************************************************/
  @media (max-width: 768px) {
    .hamburger {
      display: block; /* Hamburger anzeigen */
    }

    /* Der Link-Wrapper wird zum absolut positionierten Overlay */
    .nav-links {
      display: none;
      position: absolute;
      top: 100%; /* Unter der Nav-Leiste */
      left: 0;
      width: 100%;
      background: rgba(26, 26, 26, 0.95);
      backdrop-filter: blur(8px);
      padding: 10px 0;
      box-shadow: 0 5px 15px rgba(0, 0, 0, 0.3);
      border-top: 1px solid rgba(255, 255, 255, 0.1);
      /* <b style='color: #FFCC00;'>WICHTIG:</b>: z-index niedriger als der Hamburger/Close-Button */
      z-index: 1000;
    }

    /* Wenn Menü aktiv, Wrapper sichtbar */
    nav.active .nav-links {
      display: block;
    }

    /* Styling der Links im mobilen Overlay */
    nav .nav-links a {
      display: block;
      margin: 0;
      padding: 15px 25px;
      text-align: center; /* Links zentrieren */
      border-radius: 0;
      color: #eee;
      border-bottom: 1px solid rgba(255, 255, 255, 0.08);
    }
    nav .nav-links a:last-child {
        border-bottom: none;
    }

    /* Hover/Active für mobile Links */
    nav .nav-links a:hover,
    nav .nav-links a:active,
    nav .nav-links a.active {
      background: var(--accent-color);
      color: #000000;
    }

  } /* Ende @media (max-width: 768px) */


  @media (max-width: 480px) {
    h1 {
      font-size: 1.4rem;
    }
    /* Buttons auf 100% Breite */
    input[type="submit"],
    button {
      width: 100%;
      max-width: none;
    }
    /* Eingabefelder evtl. auch anpassen */
    input[type="text"],
    input[type="password"],
    select,
    textarea {
        max-width: 90%;
        min-width: 150px;
    }
  } /* Ende @media (max-width: 480px) */

  /************************************************************************
   * Toggle-Switches
   ************************************************************************/

/* --- Toggle Switch Styles --- */
.toggle-switch-container {
  display: flex; /* Elemente nebeneinander anordnen */
  align-items: center; /* Vertikal zentrieren */
  margin-bottom: 10px; /* Abstand nach unten */
  min-height: 30px; /* Mindesthöhe für Konsistenz */
  gap: 10px; /* Abstand zwischen Text und Schalter */
}

.toggle-switch-label-text {
  /* Standard Label-Text Stil (kann angepasst werden) */
  /* display: block; */ /* Nicht mehr block, da flex verwendet wird */
  /* margin: 10px 0 5px; */ /* Margin wird durch Container/Gap geregelt */
  font-weight: 600;
  flex-grow: 1; /* Lässt den Text den verfügbaren Platz einnehmen */
}

.toggle-switch {
  position: relative;
  display: inline-block;
  width: 50px;  /* Breite des Schalters */
  height: 26px; /* Höhe des Schalters */
  flex-shrink: 0; /* Verhindert, dass der Schalter schrumpft */
}

/* Verstecke die eigentliche Checkbox */
.toggle-switch input {
  opacity: 0;
  width: 0;
  height: 0;
  position: absolute; /* Aus dem Layout nehmen */
}

/* Der Slider (Hintergrund des Schalters) */
.toggle-slider {
  position: absolute;
  cursor: pointer;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  background-color: #555; /* Dunkler Hintergrund im "Aus"-Zustand */
  transition: .3s;
  border-radius: 26px; /* Abgerundete Ecken */
  border: 1px solid #666; /* Leichter Rand */
}

/* Der Knubbel (der bewegliche Teil) */
.toggle-slider:before {
  position: absolute;
  content: "";
  height: 20px; /* Höhe des Knubbels */
  width: 20px;  /* Breite des Knubbels */
  left: 3px;   /* Startposition von links */
  bottom: 2px;  /* Startposition von unten */
  background-color: white;
  transition: .3s;
  border-radius: 50%; /* Macht ihn rund */
  box-shadow: 0 2px 4px rgba(0,0,0,0.2);
}

/* Styling, wenn die Checkbox aktiviert (checked) ist */
.toggle-switch input:checked + .toggle-slider {
  background-color: var(--accent-color); /* Akzentfarbe für "An"-Zustand */
  border-color: var(--accent-color);
}

/* Knubbel-Position im "An"-Zustand */
.toggle-switch input:checked + .toggle-slider:before {
  transform: translateX(23px); /* Verschiebt den Knubbel nach rechts */
}

/* Fokus-Styling für Barrierefreiheit (optional aber empfohlen) */
.toggle-switch input:focus + .toggle-slider {
  box-shadow: 0 0 1px var(--accent-color);
}

/* Optional: Disabled State */
.toggle-switch input:disabled + .toggle-slider {
  cursor: not-allowed;
  background-color: #444;
  opacity: 0.6;
}
.toggle-switch input:disabled + .toggle-slider:before {
  background-color: #ccc;
}

/* Hilfsklasse für Beschreibungen unter dem Toggle */
.toggle-description {
  display: block; /* Eigene Zeile */
  font-size: 0.85em;
  color: #ccc;
  margin-top: -20px; /* Näher an den Toggle rücken */
  margin-bottom: 5px; /* Abstand zum nächsten Element */
}

/* Hilfsklasse für Beschreibungen unter dem Toggle unterhalb von Input-Feldern */
.toggle-description-input {
  display: block; /* Eigene Zeile */
  font-size: 0.85em;
  color: #ccc;
  margin-top: 0px; /* Näher an den Toggle rücken */
  margin-bottom: 5px; /* Abstand zum nächsten Element */
}

</style>
<script>
// Angepasste JavaScript-Funktion für Icon-Wechsel
function toggleMenu() {
  var nav = document.querySelector('nav');
  var hamburger = document.querySelector('.hamburger');
  nav.classList.toggle('active');

  // Prüfen, ob das Menü JETZT aktiv ist und Icon entsprechend ändern
  if (nav.classList.contains('active')) {
    hamburger.innerHTML = '&times;'; // Ändert zu einem 'X' (Schließen-Symbol)
  } else {
    hamburger.innerHTML = '&#9776;'; // Ändert zurück zum Hamburger-Symbol
  }
}

// OPTIONAL: Schließen des Menüs bei Klick außerhalb (Verbessert Usability)
document.addEventListener('click', function(event) {
  var nav = document.querySelector('nav');
  var hamburger = document.querySelector('.hamburger');
  var navLinks = document.querySelector('.nav-links');

  // Prüfen ob Menü überhaupt offen ist
  if (nav.classList.contains('active')) {
    // Prüfen ob Klick außerhalb des Menüs UND außerhalb des Buttons war
    var isClickInsideNavLinks = navLinks.contains(event.target);
    var isClickOnHamburger = hamburger.contains(event.target);

    if (!isClickInsideNavLinks && !isClickOnHamburger) {
      toggleMenu(); // Schließe das Menü
    }
  }
});

// Bestätigungsdialog für Werkseinstellungen
function confirmResetDefaults() {
    return confirm("Sicher, dass Sie alle Einstellungen auf die Werkseinstellungen geladen werden sollen?");
}

</script>
)rawliteral";


/************************************************************************************
 * Gemeinsame Navigation (PROGMEM) 
 ************************************************************************************/
#ifdef ESP32
static const char commonNav[] PROGMEM = R"rawliteral(
  <nav>
    <span class="hamburger" onclick="toggleMenu()">&#9776;</span>
    <div class="nav-links">
      <a href="/">Dashboard</a>         
      <a href="/PID">PID-Einstellung</a> 
      <a href="/PID-Tuning">PID-Tuning</a>
      <a href="/Profile">Profile</a>
      <a href="/Brew-Control">Brew Control</a>
      <a href="/Chart">Chart</a>
      <a href="/Fast-Heat-Up">Fast-Heat-Up</a>
      <a href="/ECO">Eco</a>
      <a href="/Info">Info</a>
      <a href="/Service">Service</a>
      <a href="/Netzwerk">WiFi</a>
      <a href="/Firmware">Firmware</a>
    </div>
  </nav>
)rawliteral";
#else
static const char commonNav[] PROGMEM = R"rawliteral(
  <nav>
    <span class="hamburger" onclick="toggleMenu()">&#9776;</span>
    <div class="nav-links">
      <a href="/">Dashboard</a>         
      <a href="/PID">PID-Einstellung</a> 
      <a href="/PID-Tuning">PID-Tuning</a>
      <a href="/Profile">Profile</a>
      <a href="/Chart">Chart</a>
      <a href="/Fast-Heat-Up">Fast-Heat-Up</a>
      <a href="/ECO">Eco</a>
      <a href="/Info">Info</a>
      <a href="/Service">Service</a>
      <a href="/Netzwerk">WiFi</a>
      <a href="/Firmware">Firmware</a>
    </div>
  </nav>
)rawliteral";
#endif

/************************************************************************************
 * WiFi-Konfigurationsstrukturen und Funktionen
 ************************************************************************************/

struct WiFiConfig {
    char ssid[32] = "";
    char password[64] = "";
    bool useStaticIP = false;
    IPAddress staticIP = IPAddress(192, 168, 4, 1);
    IPAddress gateway = IPAddress(192, 168, 4, 1);
    IPAddress subnet = IPAddress(255, 255, 255, 0);
    IPAddress dns = IPAddress(8, 8, 8, 8);
    // Hostname in WiFi-Config (obwohl global gespeichert, hier für Vollständigkeit)
    // Wird nicht direkt hier gespeichert, sondern über globale Variable `hostname`
};

// Liest die WLAN-Konfiguration aus dem EEPROM (Feld für Feld, IPs manuell)
// Gibt true zurück, wenn eine gültige Konfiguration (Magic Value OK und SSID nicht leer) geladen wurde
bool loadWiFiConfig(WiFiConfig& config) { // config wird per Referenz übergeben
    char magic[5] = { 0 };
    EEPROM.get(EEPROM_ADDR_WIFI_CONFIG_MAGIC, magic);

    // Lade Hostname
    EEPROM.get(EEPROM_ADDR_HOSTNAME, hostname);
    hostname[sizeof(hostname) - 1] = '\0';
    if (strlen(hostname) == 0) {
        strncpy(hostname, defaultHostname.c_str(), sizeof(hostname) - 1);
        hostname[sizeof(hostname) - 1] = '\0';
        EEPROM.put(EEPROM_ADDR_HOSTNAME, hostname);
        EEPROM.commit();
    }

    // Prüfe Magic Value
    if (strncmp(magic, storageMagicValue, 4) != 0) {
        // Serial.println("DEBUG Boot - Magic Value MISMATCH! Setze WiFi Defaults."); // Debug entfernt
        strcpy(config.ssid, "");
        strcpy(config.password, "");
        config.useStaticIP = false;
        config.staticIP = IPAddress(192, 168, 4, 1); // Default AP Mode IP
        config.gateway = IPAddress(192, 168, 4, 1); // Default AP Mode GW
        config.subnet = IPAddress(255, 255, 255, 0);
        config.dns = IPAddress(8, 8, 8, 8);         // Default Public DNS
        return false;
    }

    // Magic Value war korrekt, fahre mit dem Lesen fort
    // Serial.println("DEBUG Boot - Magic Value OK."); // Debug entfernt
    byte ipBytes[4]; // Puffer für IP-Bytes

    // Laden der einfachen Felder
    EEPROM.get(EEPROM_ADDR_WIFI_SSID, config.ssid);
    EEPROM.get(EEPROM_ADDR_WIFI_PASSWORD, config.password);
    EEPROM.get(EEPROM_ADDR_WIFI_USE_STATIC, config.useStaticIP);

    // --- MANUELLES LESEN FÜR IPAddress FELDER ---
    for(int i=0; i<4; i++) ipBytes[i] = EEPROM.read(EEPROM_ADDR_WIFI_STATIC_IP + i);
    config.staticIP = IPAddress(ipBytes[0], ipBytes[1], ipBytes[2], ipBytes[3]);

    for(int i=0; i<4; i++) ipBytes[i] = EEPROM.read(EEPROM_ADDR_WIFI_GATEWAY + i);
    config.gateway = IPAddress(ipBytes[0], ipBytes[1], ipBytes[2], ipBytes[3]);

    for(int i=0; i<4; i++) ipBytes[i] = EEPROM.read(EEPROM_ADDR_WIFI_SUBNET + i);
    config.subnet = IPAddress(ipBytes[0], ipBytes[1], ipBytes[2], ipBytes[3]);

    for(int i=0; i<4; i++) ipBytes[i] = EEPROM.read(EEPROM_ADDR_WIFI_DNS + i);
    config.dns = IPAddress(ipBytes[0], ipBytes[1], ipBytes[2], ipBytes[3]);
    // --- ENDE MANUELLES LESEN ---

    // Nullterminierung sicherstellen
    config.ssid[sizeof(config.ssid) - 1] = '\0';
    config.password[sizeof(config.password) - 1] = '\0';

    // Normale Log-Ausgabe der geladenen Werte
    // Serial.println("WiFi Konfiguration geladen:");
    // Serial.print("  SSID: "); Serial.println(config.ssid);
    // Serial.print("  Static IP Mode: "); Serial.println(config.useStaticIP);
    // if(config.useStaticIP) {
    //     Serial.print("  Static IP: "); Serial.println(config.staticIP);
    //     Serial.print("  Gateway: "); Serial.println(config.gateway);
    //     Serial.print("  Subnet: "); Serial.println(config.subnet);
    //     Serial.print("  DNS: "); Serial.println(config.dns);
    // }

    return (strlen(config.ssid) > 0);
}

// Schreibt die WLAN-Konfiguration ins EEPROM (Feld für Feld, IPs manuell byte-weise)
void saveWiFiConfig(const WiFiConfig& config) {
    // Serial.println("Schreibe WiFi Konfiguration Feld für Feld (manuelles Schreiben für IPs)..."); // Debug entfernt
    EEPROM.put(EEPROM_ADDR_WIFI_CONFIG_MAGIC, storageMagicValue);

    // Einfache Felder mit EEPROM.put
    EEPROM.put(EEPROM_ADDR_WIFI_SSID, config.ssid);
    EEPROM.put(EEPROM_ADDR_WIFI_PASSWORD, config.password);
    EEPROM.put(EEPROM_ADDR_WIFI_USE_STATIC, config.useStaticIP);

    // --- IPAddress Felder manuell mit EEPROM.write() schreiben ---
    // Serial.print("  Schreibe Static IP bytes: "); Serial.println(config.staticIP); // Debug entfernt
    for(int i=0; i<4; i++) {
        EEPROM.write(EEPROM_ADDR_WIFI_STATIC_IP + i, config.staticIP[i]);
    }

    // Serial.print("  Schreibe Gateway bytes: "); Serial.println(config.gateway); // Debug entfernt
    for(int i=0; i<4; i++) {
        EEPROM.write(EEPROM_ADDR_WIFI_GATEWAY + i, config.gateway[i]);
    }

    // Serial.print("  Schreibe Subnet bytes: "); Serial.println(config.subnet); // Debug entfernt
    for(int i=0; i<4; i++) {
        EEPROM.write(EEPROM_ADDR_WIFI_SUBNET + i, config.subnet[i]);
    }

    // Serial.print("  Schreibe DNS bytes: "); Serial.println(config.dns); // Debug entfernt
    for(int i=0; i<4; i++) {
        EEPROM.write(EEPROM_ADDR_WIFI_DNS + i, config.dns[i]);
    }
    // --- ENDE Manuelles Schreiben ---

    // Commit der Änderungen
    EEPROM.commit();
    }

// Aktiviert den Access Point Modus und zeigt IP an
void startAPMode() {
  WiFi.hostname(hostname);
  WiFi.softAP("Dual-PID", "QuickMill");  // Setzt SSID "Dual-PID" und Passwort "QuickMill"
  // Serial.print(F("AP-Mode gestartet. SSID: Dual-PID, Passwort: QuickMill, IP: "));
  // Serial.println(WiFi.softAPIP());
}

/************************************************************************************
 * Webserver-Handler: WLAN-Konfigurations-Seite - Hostname hinzugefügt
 ************************************************************************************/

// === PROGMEM Teile definieren (Neue Reihenfolge: IP, Gateway, DNS, Subnet) ===
static const char wifiPageChunk1[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><title>WLAN-Konfiguration</title><meta name='theme-color' content='#000000'><meta name='viewport' content='width=device-width, initial-scale=1.0'><meta name='theme-color' content='#000000'><meta name='viewport' content='width=device-width, initial-scale=1.0'>
)rawliteral";
static const char wifiPageChunk2[] PROGMEM = R"rawliteral(</head>)rawliteral";  // Head Ende
// Teil 3a: Body bis VOR Signalstärke-Wert
static const char wifiPageChunk3_a[] PROGMEM = R"rawliteral(
<body><h1>WLAN-Konfiguration</h1><form action='/saveWiFiConfig' method='POST'><h3>Netzwerk</h3><div style='margin-bottom: 15px; padding-bottom: 10px; border-bottom: 1px solid rgba(255,255,255,0.1);'><label style='margin-right: 5px;'>Aktuelle Signalst&auml;rke:</label><span>)rawliteral";  // Endet vor dem Wert
// Teil 3b: NACH Signalstärke-Wert bis VOR Hostname Value
static const char wifiPageChunk3_b[] PROGMEM = R"rawliteral(</span></div><label for='hostname'>Hostname:</label><input type='text' id='hostname' name='hostname' value=')rawliteral";  // Endet mit value='
// Teil 3c: NACH Hostname Value bis VOR SSID Value
static const char wifiPageChunk3_c[] PROGMEM = R"rawliteral('' required><label for='ssid'>SSID:</label><input type='text' id='ssid' name='ssid' value=')rawliteral";  // Beginnt mit ', endet mit value='
// Teil 4: NACH SSID Value bis VOR Passwort Value
static const char wifiPageChunk4[] PROGMEM = R"rawliteral('' required><label for='password'>Passwort:</label><input type='password' id='password' name='password' value=')rawliteral";  // Type=password, Beginnt mit ', endet mit value='
// Teil 5: NACH Passwort Value bis staticFields div Start (formatiert)
static const char wifiPageChunk5_format[] PROGMEM = R"rawliteral(''><h3>IP-Einstellungen</h3><label><input type='radio' name='ipType' value='dhcp' %s> DHCP</label><label><input type='radio' name='ipType' value='static' %s> Statische IP</label><div id='staticFields' style='display:%s'>)rawliteral";  // Beginnt mit '', endet mit display:%s'>
// Teil 6: IP-Label und Input Start
static const char wifiPageChunk6_ip[] PROGMEM = R"rawliteral(<label for='ip'>IP-Adresse:</label><input type='text' id='ip' name='ip' value=')rawliteral";  // Beginnt normal, endet mit value='
// Teil 7: NACH IP Value bis Gateway Value
static const char wifiPageChunk7_gateway[] PROGMEM = R"rawliteral('><label for='gateway'>Gateway:</label><input type='text' id='gateway' name='gateway' value=')rawliteral";  // Beginnt mit ', endet mit value='
// Teil 8 NACH Gateway Value bis DNS Value
static const char wifiPageChunk8_dns[] PROGMEM = R"rawliteral('><label for='dns'>DNS-Server:</label><input type='text' id='dns' name='dns' value=')rawliteral";  // Beginnt mit '>, endet mit value='
// Teil 9 NACH DNS Value bis Subnet Value
static const char wifiPageChunk9_subnet[] PROGMEM = R"rawliteral('><label for='subnet'>Subnetzmaske:</label><input type='text' id='subnet' name='subnet' value=')rawliteral";  // Beginnt mit '>, endet mit value='
// Teil 10 (Rest): NACH Subnet Value (schließt Input), schließendes Div für staticFields, Submit-Button, AP-Form, Script, Ende body/html
static const char wifiPageChunk10_rest[] PROGMEM = R"rawliteral(''></div><input type='submit' value='Speichern'></form><form action='/forceAPMode' method='POST' style='margin-top: 20px;'><h3>AP-Modus</h3>Verwendung im Access Point-Modus (AP).<br><br>Netzwerk-Name: Dual-PID<br>Passwort: QuickMill<br>IP-Adresse: 192.168.4.1<br><input type='submit' value='AP-Modus verwenden'></form><script>document.querySelectorAll('input[name="ipType"]').forEach(radio => {radio.addEventListener('change', () => {document.getElementById('staticFields').style.display = radio.value === 'static' ? 'block' : 'none';});});document.addEventListener('DOMContentLoaded', () => { const initialIpType = document.querySelector('input[name="ipType"]:checked').value; document.getElementById('staticFields').style.display = initialIpType === 'static' ? 'block' : 'none'; });</script></body></html>)rawliteral";  // Beginnt mit '>, dann </div>, Rest bis Ende

// --- Hilfsfunktion zur Umrechnung von RSSI in Prozent (Approximation) ---
int calculateSignalPercentage(int rssi) {
  int quality = 0;
  if (rssi <= -100) {
    quality = 0;
  } else if (rssi >= -50) {  // Guter Empfang
    quality = 100;
  } else {
    // Lineare Skalierung zwischen -100dBm (0%) und -50dBm (100%)
    quality = 2 * (rssi + 100);
  }
  return quality;
}

void handleWiFiConfig(AsyncWebServerRequest *request) {
    WiFiConfig currentConfig;
    char magic[5] = { 0 };
    EEPROM.get(EEPROM_ADDR_WIFI_CONFIG_MAGIC, magic);
    bool configValid = (strncmp(magic, storageMagicValue, 4) == 0);

    if (configValid) {
        // Serial.println("Lese WiFi Konfiguration für WebUI Feld für Feld (manuelles Lesen für IPs)..."); // Debug entfernt
        byte ipBytes[4]; // Puffer

        // Laden der einfachen Felder
        EEPROM.get(EEPROM_ADDR_WIFI_SSID, currentConfig.ssid);
        EEPROM.get(EEPROM_ADDR_WIFI_PASSWORD, currentConfig.password);
        EEPROM.get(EEPROM_ADDR_WIFI_USE_STATIC, currentConfig.useStaticIP);

        // --- MANUELLES LESEN FÜR IPAddress FELDER (ohne Byte-Logging) ---
        for(int i=0; i<4; i++) ipBytes[i] = EEPROM.read(EEPROM_ADDR_WIFI_STATIC_IP + i);
        currentConfig.staticIP = IPAddress(ipBytes[0], ipBytes[1], ipBytes[2], ipBytes[3]);

        for(int i=0; i<4; i++) ipBytes[i] = EEPROM.read(EEPROM_ADDR_WIFI_GATEWAY + i);
        currentConfig.gateway = IPAddress(ipBytes[0], ipBytes[1], ipBytes[2], ipBytes[3]);

        for(int i=0; i<4; i++) ipBytes[i] = EEPROM.read(EEPROM_ADDR_WIFI_SUBNET + i);
        currentConfig.subnet = IPAddress(ipBytes[0], ipBytes[1], ipBytes[2], ipBytes[3]);

        for(int i=0; i<4; i++) ipBytes[i] = EEPROM.read(EEPROM_ADDR_WIFI_DNS + i);
        currentConfig.dns = IPAddress(ipBytes[0], ipBytes[1], ipBytes[2], ipBytes[3]);
        // --- ENDE MANUELLES LESEN ---

        // Nullterminierung
        currentConfig.ssid[sizeof(currentConfig.ssid) - 1] = '\0';
        currentConfig.password[sizeof(currentConfig.password) - 1] = '\0';

    } else {
        // Defaults setzen
        // Serial.println("WiFi Magic Value nicht gefunden (in handleWiFiConfig). Setze Defaults für UI."); // Debug entfernt
        strcpy(currentConfig.ssid, "");
        strcpy(currentConfig.password, "");
        currentConfig.useStaticIP = false;
        currentConfig.staticIP = IPAddress(192, 168, 4, 1);
        currentConfig.gateway = IPAddress(192, 168, 4, 1);
        currentConfig.subnet = IPAddress(255, 255, 255, 0);
        currentConfig.dns = IPAddress(8, 8, 8, 8);
    }
    // Hostname ist global verfügbar

    // --- Signalstärke ermitteln ---
    char signalBuffer[50];
    if (WiFi.getMode() == WIFI_STA && WiFi.status() == WL_CONNECTED) {
        long rssi = WiFi.RSSI();
        int percentage = calculateSignalPercentage(rssi);
        snprintf(signalBuffer, sizeof(signalBuffer), "%d dBm (%d%%)", (int)rssi, percentage);
    } else if (WiFi.getMode() == WIFI_AP) {
        strcpy(signalBuffer, "AP-Modus (keine Messung)");
    } else {
        strcpy(signalBuffer, "Nicht verbunden");
    }

    // Temporäre Puffer für Senden
    char formatBuffer[256];
    char ipBuffer[16];

    AsyncResponseStream *response = request->beginResponseStream("text/html; charset=utf-8");

    // --- Senden der HTML Chunks (unverändert) ---
    response->print(FPSTR(wifiPageChunk1));
    response->print(FPSTR(commonStyle));
    response->print(FPSTR(wifiPageChunk2));
    response->print(FPSTR(commonNav));
    response->print(FPSTR(wifiPageChunk3_a));
    response->print(signalBuffer);
    response->print(FPSTR(wifiPageChunk3_b));
    response->print(hostname);
    response->print(FPSTR(wifiPageChunk3_c));

    // SSID einfügen
    if (strlen(currentConfig.ssid) == 0) { response->print(F("SSID eingeben")); }
    else { response->print(currentConfig.ssid); }
    response->print(FPSTR(wifiPageChunk4));

    // Passwort einfügen
    if (!demoModus) {
        if (strlen(currentConfig.password) == 0) { response->print(F("Passwort eingeben")); }
        else { response->print(currentConfig.password); }
    } else { response->print(F("********")); }

    // IP-Einstellungen (Radios + Div)
    snprintf_P(formatBuffer, sizeof(formatBuffer), wifiPageChunk5_format,
               currentConfig.useStaticIP ? "" : "checked",
               currentConfig.useStaticIP ? "checked" : "",
               currentConfig.useStaticIP ? "block" : "none");
    response->print(formatBuffer);

    // Statische IP-Felder füllen
    response->print(FPSTR(wifiPageChunk6_ip)); // IP Label+Input
    snprintf(ipBuffer, sizeof(ipBuffer), "%d.%d.%d.%d", currentConfig.staticIP[0], currentConfig.staticIP[1], currentConfig.staticIP[2], currentConfig.staticIP[3]);
    response->print(ipBuffer); // IP Wert

    response->print(FPSTR(wifiPageChunk7_gateway)); // GW Label+Input
    snprintf(ipBuffer, sizeof(ipBuffer), "%d.%d.%d.%d", currentConfig.gateway[0], currentConfig.gateway[1], currentConfig.gateway[2], currentConfig.gateway[3]);
    response->print(ipBuffer); // GW Wert

    response->print(FPSTR(wifiPageChunk8_dns)); // DNS Label+Input
    snprintf(ipBuffer, sizeof(ipBuffer), "%d.%d.%d.%d", currentConfig.dns[0], currentConfig.dns[1], currentConfig.dns[2], currentConfig.dns[3]);
    response->print(ipBuffer); // DNS Wert

    response->print(FPSTR(wifiPageChunk9_subnet)); // Subnet Label+Input
    snprintf(ipBuffer, sizeof(ipBuffer), "%d.%d.%d.%d", currentConfig.subnet[0], currentConfig.subnet[1], currentConfig.subnet[2], currentConfig.subnet[3]);
    response->print(ipBuffer); // Subnet Wert

    // Rest der Seite
    response->print(FPSTR(wifiPageChunk10_rest));

    request->send(response);
}

/************************************************************************************
 * Handler, um den AP-Modus per Knopfdruck zu erzwingen - Heap-Optimiert
 ************************************************************************************/

// Die PROGMEM-Definitionen für die Bestätigungsseite bleiben bestehen
static const char forceAPHtml[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>AP-Modus aktivieren</title><meta name='theme-color' content='#000000'><meta name='viewport' content='width=device-width, initial-scale=1.0'><meta name='theme-color' content='#000000'><meta name='viewport' content='width=device-width, initial-scale=1.0'>
)rawliteral";

static const char forceAPHtml2[] PROGMEM = R"rawliteral(
</head>
)rawliteral";

static const char forceAPHtml3[] PROGMEM = R"rawliteral(
<body>
<h1>AP-Modus wird aktiviert...</h1>
<form><p>Die Einstellungen wurden gespeichert. Neustart im AP-Modus ...</p></form>
</body>
</html>
)rawliteral";


// --- Überarbeitete Funktion handleForceAPMode ---
void handleForceAPMode(AsyncWebServerRequest *request) {
  // Leer-Konfiguration speichern, damit beim nächsten Start AP aktiv wird
  WiFiConfig emptyConfig;
  // Wichtig: Hostname NICHT entfernen!
  if (!demoModus) { saveWiFiConfig(emptyConfig); }

  AsyncResponseStream *response = request->beginResponseStream("text/html; charset=utf-8");
  response->print(FPSTR(forceAPHtml));   // Chunk 1 (Head Start)
  response->print(FPSTR(commonStyle));   // Common CSS
  response->print(FPSTR(forceAPHtml2));  // Chunk 2 (Head End)
  response->print(FPSTR(commonNav));     // Common Navigation
  response->print(FPSTR(forceAPHtml3));  // Chunk 3 (Body)
  request->send(response);

  // --- Logik nach dem Senden (BLEIBT UNVERÄNDERT) ---
  scheduleRestart(1000);
}

/************************************************************************************
 * Handler zum Speichern der WLAN-Konfiguration - Heap-Optimiert (Bestätigungsseite)
 ************************************************************************************/

// Die PROGMEM-Definitionen für die Bestätigungsseite bleiben bestehen
static const char saveConfigHtml[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>Einstellungen gespeichert</title><meta name='theme-color' content='#000000'><meta name='viewport' content='width=device-width, initial-scale=1.0'><meta name='theme-color' content='#000000'><meta name='viewport' content='width=device-width, initial-scale=1.0'>
)rawliteral";

static const char saveConfigHtml2[] PROGMEM = R"rawliteral(
</head>
)rawliteral";

static const char saveConfigHtml3[] PROGMEM = R"rawliteral(
<body>
<h1>Einstellungen gespeichert</h1>
<form><p>Die Einstellungen wurden gespeichert. Neustart ...</p></form>
</body>
</html>
)rawliteral";

// --- Funktion handleSaveWiFiConfig ---
void handleSaveWiFiConfig(AsyncWebServerRequest *request) {
    // --- Daten validieren und auslesen ---
    if (!request->hasArg(F("ssid"))) {
        request->send(400, F("text/plain"), F("Fehler: SSID fehlt!"));
        return;
    }

    // --- Konfigurationsstruktur füllen ---
    WiFiConfig newConfig;

    // SSID und Passwort kopieren (mit Längenbegrenzung und Nullterminierung)
    strncpy(newConfig.ssid, request->arg(F("ssid")).c_str(), sizeof(newConfig.ssid) - 1);
    newConfig.ssid[sizeof(newConfig.ssid) - 1] = '\0';

    if (request->hasArg(F("password"))) {
        strncpy(newConfig.password, request->arg(F("password")).c_str(), sizeof(newConfig.password) - 1);
        newConfig.password[sizeof(newConfig.password) - 1] = '\0';
    } else {
        newConfig.password[0] = '\0';
    }

    // IP-Typ bestimmen
    newConfig.useStaticIP = (request->arg(F("ipType")) == "static");

    // Statische IP-Einstellungen parsen, falls ausgewählt
    if (newConfig.useStaticIP) {
        if (!request->hasArg(F("ip")) || !request->hasArg(F("gateway")) || !request->hasArg(F("subnet")) || !request->hasArg(F("dns"))) {
            request->send(400, F("text/plain"), F("Fehler: Fehlende Felder für statische IP!"));
            return;
        }

        bool ipOK = newConfig.staticIP.fromString(request->arg(F("ip")));
        bool gwOK = newConfig.gateway.fromString(request->arg(F("gateway")));
        bool subOK = newConfig.subnet.fromString(request->arg(F("subnet")));
        bool dnsOK = newConfig.dns.fromString(request->arg(F("dns")));
    }

    // Hostname speichern (separat, da nicht Teil der WiFiConfig im EEPROM)
    if (request->hasArg(F("hostname"))) {
        strncpy(hostname, request->arg(F("hostname")).c_str(), sizeof(hostname) - 1);
        hostname[sizeof(hostname) - 1] = '\0'; // Sicherstellen der Nullterminierung
        if (strlen(hostname) == 0) { // Wenn leer, Default verwenden
            strncpy(hostname, defaultHostname.c_str(), sizeof(hostname) - 1);
            hostname[sizeof(hostname) - 1] = '\0';
        }
        EEPROM.put(EEPROM_ADDR_HOSTNAME, hostname); // Hostname direkt ins EEPROM schreiben
        EEPROM.commit();
    }


    // --- Speichern (wenn nicht im Demo-Modus) ---
    if (!demoModus) {
        // Serial.println("Schreibe WiFi-Konfiguration ins EEPROM...");
        saveWiFiConfig(newConfig); // Ruft die Funktion auf, die Feld für Feld schreibt
                                   // EEPROM.commit() wird jetzt innerhalb von saveWiFiConfig aufgerufen.
    }

    // --- Senden der Bestätigungsseite (Heap-Optimiert) ---
    AsyncResponseStream *response = request->beginResponseStream("text/html; charset=utf-8");
    response->print(FPSTR(saveConfigHtml));    // Chunk 1 (Head Start)
    response->print(FPSTR(commonStyle));       // Common CSS
    response->print(FPSTR(saveConfigHtml2));   // Chunk 2 (Head End)
    response->print(FPSTR(commonNav));         // Common Navigation
    response->print(FPSTR(saveConfigHtml3));   // Chunk 3 (Body)
    request->send(response);

    // --- Neustart einleiten ---
    scheduleRestart(5000); // Browser kann Bestätigung empfangen
  }

// Prüfung der WiFi-Verbindung (Reconnect falls getrennt) - Unverändert
void checkWifiConnection() {
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.reconnect();
    // if (WiFi.waitForConnectResult() != WL_CONNECTED) {Serial.println(F("WiFi-Verbindung kann nicht hergestellt werden!")); }
    }
}

#ifdef ENABLE_DISPLAY
// WiFi-Symbol aufs Display zeichnen 
void drawwifiSymbol(int16_t x, int16_t y) {
  display.drawBitmap(x, y, wifiSymbol, 13, 7, SH110X_WHITE);
}
#endif // ENABLE_DISPLAY

/************************************************************************************
 * Handler zum Rücksetzen der Betriebszeit - URL angepasst
 ************************************************************************************/

void handleResetRuntime(AsyncWebServerRequest *request) {
  totalRuntime = 0;
  EEPROM.put(EEPROM_ADDR_RUNTIME, totalRuntime);
  EEPROM.commit();
  AsyncWebServerResponse *response = request->beginResponse(303);
  response->addHeader(F("Location"), F("/Info"));
  request->send(response);
}

/************************************************************************************
 * Handler zum Rücksetzen des Shotzählers - URL angepasst
 ************************************************************************************/

void handleResetShots(AsyncWebServerRequest *request) {
  shotCounter = 0;
  EEPROM.put(EEPROM_ADDR_SHOTCOUNTER, shotCounter);
  EEPROM.commit();
  AsyncWebServerResponse *response = request->beginResponse(303);
  response->addHeader(F("Location"), F("/Info"));
  request->send(response);
}

/************************************************************************************
 * Handler zum Rücksetzen des Wartungszählers (über Shot-Zähler)
 ************************************************************************************/
void handleResetMaintenance(AsyncWebServerRequest *request) {
  maintenanceIntervalCounter = 0;  // Wartungszähler-Reset auf 0
  EEPROM.put(EEPROM_ADDR_MAINTENANCE_INTERVAL_COUNTER, maintenanceIntervalCounter);
  EEPROM.commit();
  displayMaintenanceMessage = false;                // Meldung ggf. ausblenden
  AsyncWebServerResponse *response = request->beginResponse(303);
  response->addHeader(F("Location"), F("/Service"));
  request->send(response);
}


/************************************************************************************
 * Handler für den Export der EEPROM-Einstellungen - Unverändert
 ************************************************************************************/
void handleExportSettings(AsyncWebServerRequest *request) {
  const size_t bufferSize = 64;
  byte buffer[bufferSize];

  AsyncResponseStream *response = request->beginResponseStream("application/octet-stream");
  response->addHeader("Content-Disposition", "attachment; filename=\"dual_pid_settings.bin\"");
  response->setContentLength(EEPROM_SIZE);

  for (int i = 0; i < EEPROM_SIZE; i += bufferSize) {
    size_t currentChunkSize = (EEPROM_SIZE - i < bufferSize) ? (EEPROM_SIZE - i) : bufferSize;
    for (size_t j = 0; j < currentChunkSize; j++) {
      buffer[j] = EEPROM.read(i + j);
      yield();
    }
    response->write(buffer, currentChunkSize);
    yield();
  }

  request->send(response);
}


/************************************************************************************
 * Handler für den Import der EEPROM-Einstellungen (Upload-Verarbeitung) - Unverändert
 ************************************************************************************/
// Globale Variablen bleiben gleich
static int eepromWriteAddress = 0;
static bool importSuccessful = false;
static String importStatusMessage = "";

void handleImportUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
  (void)request;
  (void)filename;

  if (demoModus) {
    if (index == 0) {
      importStatusMessage = F("Import im Demo-Modus nicht erlaubt!");
      importSuccessful = false;
    }
    return;
  }

  if (index == 0) {
    eepromWriteAddress = 0;
    importSuccessful = false;
    importStatusMessage = "";
  }

  if (len) {
    if (importStatusMessage.length() == 0) {
      for (size_t i = 0; i < len; i++) {
        if (eepromWriteAddress < EEPROM_SIZE) {
          EEPROM.write(eepromWriteAddress, data[i]);
          eepromWriteAddress++;
        } else {
          importStatusMessage = F("Importfehler: Empfangene Datei zu gro&szlig;!");
          break;
        }
        yield();
      }
    }
  }

  if (final) {
    if (importStatusMessage.length() > 0) {
      importSuccessful = false;
    } else if (eepromWriteAddress == EEPROM_SIZE) {
      if (EEPROM.commit()) {
        importSuccessful = true;
        importStatusMessage = F("Einstellungen erfolgreich importiert! Neustart wird durchgef&uuml;hrt...");
      } else {
        importSuccessful = false;
        importStatusMessage = F("Import fehlgeschlagen: EEPROM Commit Fehler!");
      }
    } else {
      importSuccessful = false;
      importStatusMessage = F("Import fehlgeschlagen: Falsche Dateigr&ouml;&szlig;e!");
    }
    eepromWriteAddress = 0;
  }
}

/************************************************************************************
 * Handler für die Import-Seite (nach dem Upload) - Heap-Optimiert
 ************************************************************************************/

// PROGMEM Chunks für die Seite
static const char importDoneHtmlStart[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><title>Einstellungen Import</title><meta name='theme-color' content='#000000'><meta name='viewport' content='width=device-width, initial-scale=1.0'><meta name='theme-color' content='#000000'><meta name='viewport' content='width=device-width, initial-scale=1.0'>
)rawliteral";
static const char importDoneHtmlHeadEnd[] PROGMEM = R"rawliteral(</head>)rawliteral";
static const char importDoneHtmlBodyStart[] PROGMEM = R"rawliteral(
<body><h1>Einstellungen Import</h1>
<form><p>)rawliteral";  // Endet vor {status_message}
static const char importDoneHtmlBodyEnd[] PROGMEM = R"rawliteral(</p></form>
</body></html>
)rawliteral";           // Beginnt nach {status_message}

void handleImportSettings(AsyncWebServerRequest *request) {
  AsyncResponseStream *response = request->beginResponseStream("text/html");
  response->write(importDoneHtmlStart, strlen_P(importDoneHtmlStart));
  response->write(commonStyle, strlen_P(commonStyle));
  response->write(importDoneHtmlHeadEnd, strlen_P(importDoneHtmlHeadEnd));
  response->write(commonNav, strlen_P(commonNav));
  response->write(importDoneHtmlBodyStart, strlen_P(importDoneHtmlBodyStart));
  if (importStatusMessage.length() > 0) {
    response->print(importStatusMessage);
  }
  response->write(importDoneHtmlBodyEnd, strlen_P(importDoneHtmlBodyEnd));
  request->send(response);

  if (importSuccessful) {
    scheduleRestart(5000);
  }
}

/************************************************************************************
 * Funktion zum Zurücksetzen auf Werkseinstellungen
 * Setzt auch Brew Control und Brew-by-Weight Werte zurück
 ************************************************************************************/
void resetToDefaults() {

  // PID & Offset etc. (Bestehende Werte)
  EEPROM.put(EEPROM_ADDR_SETPOINT_DAMPF, defaultSetpointDampf);
  EEPROM.put(EEPROM_ADDR_SETPOINT_WASSER, defaultSetpointWasser);
  EEPROM.put(EEPROM_ADDR_OFFSET_DAMPF, defaultOffsetDampf);
  EEPROM.put(EEPROM_ADDR_OFFSET_WASSER, defaultOffsetWasser);
  EEPROM.put(EEPROM_ADDR_KP_DAMPF, defaultKpDampf);
  EEPROM.put(EEPROM_ADDR_KI_DAMPF, defaultKiDampf);
  EEPROM.put(EEPROM_ADDR_KD_DAMPF, defaultKdDampf);
  EEPROM.put(EEPROM_ADDR_KP_WASSER, defaultKpWasser);
  EEPROM.put(EEPROM_ADDR_KI_WASSER, defaultKiWasser);
  EEPROM.put(EEPROM_ADDR_KD_WASSER, defaultKdWasser);
  EEPROM.put(EEPROM_ADDR_WINDOWSIZE_WASSER, defaultWindowSizeWasser);
  EEPROM.put(EEPROM_ADDR_WINDOWSIZE_DAMPF, defaultWindowSizeDampf);
  EEPROM.put(EEPROM_ADDR_BOOST_WASSER_ACTIVE, defaultBoostWasserActive);
  EEPROM.put(EEPROM_ADDR_BOOST_DAMPF_ACTIVE, defaultBoostDampfActive);
  EEPROM.put(EEPROM_ADDR_PREVENTHEAT_WASSER, defaultPreventHeatAboveSetpointWasser);
  EEPROM.put(EEPROM_ADDR_PREVENTHEAT_DAMPF, defaultPreventHeatAboveSetpointDampf);
  EEPROM.put(EEPROM_ADDR_MAX_TEMP_WASSER, maxTempWasser); // Sollte ggf. eigene Defaults haben
  EEPROM.put(EEPROM_ADDR_MAX_TEMP_DAMPF, maxTempDampf);   // Sollte ggf. eigene Defaults haben

  // Eco Modus
  EEPROM.put(EEPROM_ADDR_ECOMODE_MINUTES, defaultEcoModeMinutes);
  EEPROM.put(EEPROM_ADDR_ECOMODE_TEMP_WASSER, defaultEcoModeTempWasser);
  EEPROM.put(EEPROM_ADDR_ECOMODE_TEMP_DAMPF, defaultEcoModeTempDampf);
  EEPROM.put(EEPROM_ADDR_DYNAMIC_ECO_MODE, false);
  EEPROM.put(EEPROM_ADDR_STEAM_DELAY, defaultDampfVerzoegerung);

  // Geräteinfo
  char tempInfo[50];
  strncpy(tempInfo, defaultInfoHersteller.c_str(), sizeof(tempInfo) - 1); tempInfo[sizeof(tempInfo) - 1] = '\0'; EEPROM.put(EEPROM_ADDR_INFO_HERSTELLER, tempInfo);
  strncpy(tempInfo, defaultInfoModell.c_str(), sizeof(tempInfo) - 1); tempInfo[sizeof(tempInfo) - 1] = '\0'; EEPROM.put(EEPROM_ADDR_INFO_MODELL, tempInfo);
  strncpy(tempInfo, defaultInfoZusatz.c_str(), sizeof(tempInfo) - 1); tempInfo[sizeof(tempInfo) - 1] = '\0'; EEPROM.put(EEPROM_ADDR_INFO_ZUSATZ, tempInfo);

  // Fast Heat Up
  EEPROM.put(EEPROM_ADDR_FASTHEATUP_DATA, false);

  // AutoTune Parameter
  EEPROM.put(EEPROM_ADDR_TUNING_STEP_WASSER, defaultTuningStepWasser);
  EEPROM.put(EEPROM_ADDR_TUNING_NOISE_WASSER, defaultTuningNoiseWasser);
  EEPROM.put(EEPROM_ADDR_TUNING_STARTVALUE_WASSER, defaultTuningStartValueWasser);
  EEPROM.put(EEPROM_ADDR_TUNING_LOOKBACK_WASSER, defaultTuningLookBackWasser);
  EEPROM.put(EEPROM_ADDR_TUNING_STEP_DAMPF, defaultTuningStepDampf);
  EEPROM.put(EEPROM_ADDR_TUNING_NOISE_DAMPF, defaultTuningNoiseDampf);
  EEPROM.put(EEPROM_ADDR_TUNING_STARTVALUE_DAMPF, defaultTuningStartValueDampf);
  EEPROM.put(EEPROM_ADDR_TUNING_LOOKBACK_DAMPF, defaultTuningLookBackDampf);

  // Hostname auf Standard
  strncpy(hostname, defaultHostname.c_str(), sizeof(hostname) - 1); hostname[sizeof(hostname) - 1] = '\0';
  EEPROM.put(EEPROM_ADDR_HOSTNAME, hostname);

  // Wartungsintervall zurücksetzen
  EEPROM.put(EEPROM_ADDR_MAINTENANCE_INTERVAL_COUNTER, 0UL);
  EEPROM.put(EEPROM_ADDR_MAINTENANCE_INTERVAL, defaultMaintenanceInterval);

  // Brew Control auf Defaults im EEPROM schreiben
  EEPROM.put(EEPROM_ADDR_BREWBYTIME_ENABLED, defaultBrewByTimeEnabled);
  EEPROM.put(EEPROM_ADDR_BREWBYTIME_SECONDS, defaultBrewByTimeTargetSeconds);
  EEPROM.put(EEPROM_ADDR_PREINF_ENABLED, defaultPreInfusionEnabled);
  EEPROM.put(EEPROM_ADDR_PREINF_DUR_SEC, defaultPreInfusionDurationSeconds);
  EEPROM.put(EEPROM_ADDR_PREINF_PAUSE_SEC, defaultPreInfusionPauseSeconds);
  EEPROM.put(EEPROM_ADDR_BREWBYWEIGHT_ENABLED, defaultBrewByWeightEnabled);
  EEPROM.put(EEPROM_ADDR_BREWBYWEIGHT_TARGET, defaultBrewByWeightTargetGrams);
  EEPROM.put(EEPROM_ADDR_BREWBYWEIGHT_OFFSET, defaultBrewByWeightOffsetGrams);

  // Piezo auf Default im EEPROM schreiben
  EEPROM.put(EEPROM_ADDR_PIEZO_ENABLED, defaultPiezoEnabled);

  // Dampfverzögerungs-Override auf Default im EEPROM schreiben
  EEPROM.put(EEPROM_ADDR_STEAM_DELAY_OVERRIDE_SWITCH, defaultSteamDelayOverrideBySwitchEnabled);
 
  // Magic Value für allgemeine Einstellungen setzen (wichtig!)
  EEPROM.put(EEPROM_ADDR_MAGICVALUE, storageMagicValue);

  // Alle Änderungen ins EEPROM schreiben
  EEPROM.commit();

  // WICHTIG: Globale Variablen auch auf Default setzen
  SetpointWasser = defaultSetpointWasser; SetpointDampf = defaultSetpointDampf;
  OffsetWasser = defaultOffsetWasser; OffsetDampf = defaultOffsetDampf;
  KpWasser = defaultKpWasser; KiWasser = defaultKiWasser; KdWasser = defaultKdWasser;
  KpDampf = defaultKpDampf; KiDampf = defaultKiDampf; KdDampf = defaultKdDampf;
  ecoModeMinutes = defaultEcoModeMinutes; ecoModeTempWasser = defaultEcoModeTempWasser; ecoModeTempDampf = defaultEcoModeTempDampf;
  dynamicEcoActive = false; fastHeatUpAktiv = false; dampfVerzoegerung = defaultDampfVerzoegerung;
  tuningStepWasser = defaultTuningStepWasser; tuningNoiseWasser = defaultTuningNoiseWasser; tuningStartValueWasser = defaultTuningStartValueWasser; tuningLookBackWasser = defaultTuningLookBackWasser;
  tuningStepDampf = defaultTuningStepDampf; tuningNoiseDampf = defaultTuningNoiseDampf; tuningStartValueDampf = defaultTuningStartValueDampf; tuningLookBackDampf = defaultTuningLookBackDampf;
  maintenanceInterval = defaultMaintenanceInterval; maintenanceIntervalCounter = 0;
  boostWasserActive = defaultBoostWasserActive; boostDampfActive = defaultBoostDampfActive;
  preventHeatAboveSetpointWasser = defaultPreventHeatAboveSetpointWasser; preventHeatAboveSetpointDampf = defaultPreventHeatAboveSetpointDampf;
  windowSizeWasser = defaultWindowSizeWasser; windowSizeDampf = defaultWindowSizeDampf;
  strncpy(hostname, defaultHostname.c_str(), sizeof(hostname) - 1); hostname[sizeof(hostname) - 1] = '\0';

  // Globale Brew Control Variablen auf Defaults setzen
  brewByTimeEnabled = defaultBrewByTimeEnabled; brewByTimeTargetSeconds = defaultBrewByTimeTargetSeconds;
  preInfusionEnabled = defaultPreInfusionEnabled; preInfusionDurationSeconds = defaultPreInfusionDurationSeconds; preInfusionPauseSeconds = defaultPreInfusionPauseSeconds;

  // Globale Brew-by-Weight Variablen auf Defaults setzen
#ifdef ESP32
  brewByWeightEnabled = defaultBrewByWeightEnabled; brewByWeightTargetGrams = defaultBrewByWeightTargetGrams; brewByWeightOffsetGrams = defaultBrewByWeightOffsetGrams;
#endif

  // Globale Piezo Variable auf Default setzen
  piezoEnabled = defaultPiezoEnabled; 
  
  // Globale Variable für Dampfverzögerungs-Override auf Default setzen
  steamDelayOverrideBySwitchEnabled = defaultSteamDelayOverrideBySwitchEnabled;
  
  // PID Limits neu setzen (falls Fenstergröße geändert wurde)
  pidWasser.SetOutputLimits(0, windowSizeWasser);
  pidDampf.SetOutputLimits(0, windowSizeDampf);
}

/************************************************************************************
 * Handler für das Zurücksetzen auf Werkseinstellungen - Heap-Optimiert
 ************************************************************************************/

// PROGMEM Chunks für die Seite
static const char resetDoneHtmlStart[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><title>Werkseinstellungen</title><meta name='theme-color' content='#000000'><meta name='viewport' content='width=device-width, initial-scale=1.0'><meta name='theme-color' content='#000000'><meta name='viewport' content='width=device-width, initial-scale=1.0'>
)rawliteral";
static const char resetDoneHtmlHeadEnd[] PROGMEM = R"rawliteral(</head>)rawliteral";
static const char resetDoneHtmlBody[] PROGMEM = R"rawliteral(
<body><h1>Werkseinstellungen</h1>
<form><p>Alle Einstellungen wurden auf die Werkseinstellungen zur&uuml;ckgesetzt. Das Ger&auml;t wird neu gestartet...</p></form>
</body></html>
)rawliteral";

// Überarbeitete Funktion
void handleResetDefaults(AsyncWebServerRequest *request) {
  if (!demoModus) {
    resetToDefaults();
  } else {
    request->send(403, "text/plain", "Reset im Demo-Modus nicht erlaubt.");
    return;
  }

  AsyncResponseStream *response = request->beginResponseStream("text/html");
  response->write(resetDoneHtmlStart, strlen_P(resetDoneHtmlStart));
  response->write(commonStyle, strlen_P(commonStyle));
  response->write(resetDoneHtmlHeadEnd, strlen_P(resetDoneHtmlHeadEnd));
  response->write(commonNav, strlen_P(commonNav));
  response->write(resetDoneHtmlBody, strlen_P(resetDoneHtmlBody));
  request->send(response);

  scheduleRestart(5000);
}

/************************************************************************************
 * Piezo-Hilfsfunktionen (Nur ESP32)
 ************************************************************************************/
#ifdef ESP32 // Funktionen nur für ESP32 kompilieren

// Erzeugt einen kurzen Piepton
void beepShort(int frequency = 3000, int duration = 1000) {
    if (!piezoEnabled) return;
    tone(PIEZO_PIN, frequency, duration);
    // delay(duration + 10); // Kurze Pause danach, falls nötig, aber `tone` mit Dauer ist blockierend
}

// Spezifischer Piepton für die Wartungserinnerung
void beepMaintenanceAlert(int frequency = 3000, int duration = 3000) {
    if (!piezoEnabled) return;
    tone(PIEZO_PIN, frequency, duration);
    // delay(duration + 10); // Kurze Pause danach, falls nötig, aber `tone` mit Dauer ist blockierend
}
#endif // ESP32


/************************************************************************************
 *  ESP-NOW Empfangsfunktion (Nur ESP32)
 ************************************************************************************/
#if (defined(ESP32) && (SCALE_TYPE == SCALE_ESPNOW))
/**
 * @brief Callback-Funktion, die bei Empfang einer ESP-NOW Nachricht aufgerufen wird.
 * Verarbeitet die Daten der Waage.
 * KORREKTE SIGNATUR FÜR AKTUELLE ESP32-CORES
 */
void OnDataRecv(const esp_now_recv_info * info, const uint8_t *incomingData, int len) {
  // Der erste Parameter ist jetzt eine Struktur, wir ignorieren sie aber, da wir nur die Daten brauchen.
  // Die MAC-Adresse wäre z.B. über "info->src_addr" erreichbar.
  
  // Prüfen, ob die Länge der Nachricht der erwarteten Struktur entspricht
  if (len == sizeof(struct_espnow_scale_message)) {
    struct_espnow_scale_message receivedData;
    memcpy(&receivedData, incomingData, sizeof(receivedData));

    // Globale Variablen aktualisieren
    currentWeightReading = receivedData.weight_g;
    lastScaleMessageTime = millis(); // Zeit des letzten Empfangs für Timeout merken

    // Verbindung als hergestellt markieren, falls sie es noch nicht war
    if (!scaleConnected) {
      scaleConnected = true;
      // Serial.println("INFO: ESP-NOW Waage verbunden und erste Daten empfangen.");
    }

    // Status-Flags auswerten
    if (receivedData.status_flags & ESPNOW_SCALE_FLAG_TOGGLE_MODE) {
      scaleModeActive = !scaleModeActive;
      // Serial.print("INFO: Waagen-Anzeigemodus via ESP-NOW umgeschaltet: ");
      // Serial.println(scaleModeActive ? "AN" : "AUS");
      if (piezoEnabled) {
        beepShort(scaleModeActive ? 3500 : 3200, 70); // Unterschiedliche Töne für an/aus
      }
    }
    if (receivedData.status_flags & ESPNOW_SCALE_FLAG_JUST_TARED) {
      // Serial.println("INFO: Waage wurde via ESP-NOW tariert.");
      if (piezoEnabled) {
        beepShort(2800, 50); // Bestätigungston
      }
    }
    if (receivedData.status_flags & ESPNOW_SCALE_FLAG_AWOKE) {
      // Serial.println("INFO: ESP-NOW Waage ist aus dem Schlaf aufgewacht.");
    }
  }
}
#endif

/************************************************************************************
 * Variable für Systemstartzeit (relevant für die Dampf-Verzögerung) - Unverändert
 ************************************************************************************/

unsigned long startupTime;

#ifdef ESP32
/************************************************************************************
 * Stoppt den Brühvorgang (Pumpe/Ventil) und verarbeitet Shot-Daten (ESP32)
 * Wird von updateShotTimer() oder aus loop() (BrewByTime/Weight) aufgerufen.
 * Setzt das manualSwitchReleasedAfterAutoStop Flag entsprechend.
 ************************************************************************************/
void stopBrewSequence(unsigned long durationMillis, bool manualStop = false, bool stoppedByWeight = false) {
    if (!shotActive) return; // Verhindert mehrfaches Ausführen

    lastShotDurationMillis = durationMillis; // Dauer speichern
    shotEndTime = millis();                  // Endzeitpunkt speichern
    lastShotFirstWeightChangeTime = firstWeightChangeTime;
    lastShotStartTime = shotStartTime;
    firstWeightChangeDetected = false;
    firstWeightChangeTime = 0;

    // Wenn durch Gewicht gestoppt, das finale Gewicht für die Anzeige speichern
    if (stoppedByWeight) {
        lastShotFinalNetWeight = currentWeightReading - brewStartWeight;
    }

    // Statusvariablen zurücksetzen
    shotActive = false;
    currentPreInfusionState = PI_INACTIVE;
    lastShotTime = millis(); // Wichtig für Eco-Modus

    // Hardware stoppen
    digitalWrite(PUMP_PIN, LOW);
    digitalWrite(VALVE_PIN, LOW); // Ventil öffnen -> Druck ablassen

    // --- Sperre das System, wenn es ein AUTO-Stopp war ---
    if (!manualStop) { // Prüft den übergebenen Parameter
        manualSwitchReleasedAfterAutoStop = false; // Sperren, bis der Schalter losgelassen wird 
        // Hinweiston ausgeben
        beepShort();
    } else {
        manualSwitchReleasedAfterAutoStop = true; // Sicherstellen, dass nach manuellem Stopp entsperrt ist
    }
    // --- Ende Sperrlogik ---

    // Shot zählen und loggen (nur wenn > 20 Sekunden)
    if (durationMillis > 20000 && !wartungsModusAktiv) {
        // Langsames Speichern wird in die Hauptschleife ausgelagert.
        // Hier werden jetzt alle relevanten Daten für den Log zwischengespeichert.
        shotNeedsToBeSaved = true;
        savedShotDuration = durationMillis;
        savedShotWasByWeight = stoppedByWeight; // Kennzeichnen, ob der Bezug gewichtsbasierend war
        if(stoppedByWeight) {
            savedShotFinalWeight = lastShotFinalNetWeight  + brewByWeightOffsetGrams; ; // Gewicht (inkl. Offset) für den Log merken
        } else {
            savedShotFinalWeight = 0.0f; // Sicherstellen, dass kein altes Gewicht geloggt wird
        }
    }

    // ZUSTANDSMASCHINE STARTEN (JETZT KONDITIONAL)
    if (stoppedByWeight) {
        postShotDisplayState = POST_SHOT_SHOW_WEIGHT_RESULT; // Setze neuen Zustand für Gewichtsanzeige
    } else {
        postShotDisplayState = POST_SHOT_SHOW_DURATION; // Setze alten Zustand für Zeit/manuell
    }

    // Fast-Heat-Up eventuell deaktivieren
    fastHeatUpHeating = false;

}
#endif // ESP32

/************************************************************************************
 * setup(): Initialisierung des Systems
 * - Angepasst für optionale Display-Unterstützung via ENABLE_DISPLAY
 * - Korrekte Wire.begin() Logik für Display/Waage
 * - Zusätzliche Serial-Ausgaben für Headless-Betrieb
 ************************************************************************************/
void setup() {
  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE); // EEPROM initialisieren

  // --- PinModes initialisieren ---
  pinMode(SSR_DAMPF_PIN, OUTPUT);
  pinMode(SSR_WASSER_PIN, OUTPUT);
#ifdef ESP32
  pinMode(SHOT_TIMER_PIN, INPUT_PULLDOWN); // ESP32 uses Pulldown for HIGH active  
  // Sicherstellen, dass nicht durch vorherigen Bezug, welcher mit Brew-Control beendet wurde noch der Bezugsschalter auf "EIN" ist
  // und dadurch direkt nach dem Einschalten ein Bezug erfolgt - Nur Relevant für ESP32-Variante!
  if (digitalRead(SHOT_TIMER_PIN) == HIGH) {
    manualSwitchReleasedAfterAutoStop = false; // Behandle es so, als ob gerade ein Auto-Stopp erfolgt wäre
  }

  pinMode(PUMP_PIN, OUTPUT);
  pinMode(VALVE_PIN, OUTPUT);

  // Piezo-Pin als Ausgang konfigurieren <<<
  pinMode(PIEZO_PIN, OUTPUT);
  digitalWrite(PIEZO_PIN, LOW); // Sicherstellen, dass er initial aus ist

  // ADC für NTC-Pin konfigurieren und kalibrieren (nur ESP32)
  analogSetPinAttenuation(ANALOG_NTC_PIN, ADC_11db);
  analogReadResolution(12);
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
  // if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
  //     Serial.println("  Kalibrierung: eFuse Vref verwendet.");
  // } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
  //     Serial.println("  Kalibrierung: Two Point eFuse Werte verwendet.");
  // } else {
  //     Serial.println("  Kalibrierung: Default Vref verwendet.");
  // }
  // *** ENDE ADC Konfiguration/Kalibrierung ***

#else // ESP8266
  pinMode(SHOT_TIMER_PIN, INPUT_PULLUP); // ESP8266 uses Pullup for HIGH active
#endif

  // Initialisiere SSRs als AUS
  digitalWrite(SSR_DAMPF_PIN, LOW);
  digitalWrite(SSR_WASSER_PIN, LOW);
#ifdef ESP32
  // Pumpe & Ventil Pins initialisieren (AUS)
  digitalWrite(PUMP_PIN, LOW);
  digitalWrite(VALVE_PIN, LOW);
#endif

  // --- Wire (I2C) initialisieren, wenn benötigt (Display oder Waage) ---
#if defined(ENABLE_DISPLAY) || defined(ESP32) // Wenn Display aktiviert ODER ESP32 (für Waage)
    #ifdef ENABLE_DISPLAY
        // Priorisiere Display-Pins, falls aktiviert
        Wire.begin(IIC_5V_SDA, IIC_5V_SCK);
    #elif defined(ESP32) // Nur Waage auf ESP32 (kein Display)
        // Nutze Standard I2C Pins oder spezifische Waagen-Pins falls anders definiert
        Wire.begin(IIC_5V_SDA, IIC_5V_SCK); // Oder Wire.begin(SCALE_SDA, SCALE_SCL); falls vorhanden
    #endif
#endif


  // --- EEPROM-Werte laden oder Defaults setzen ---
  char storedMagicValue[5] = {0};
  bool magicValueVorhanden = false;
  EEPROM.get(EEPROM_ADDR_MAGICVALUE, storedMagicValue);

  if (strncmp(storedMagicValue, storageMagicValue, 4) == 0) {
    magicValueVorhanden = true;
  } else {
    magicValueVorhanden = false;
  }

  // Werte aus dem EEPROM laden oder Defaults setzen (kompletter Block)
  if (magicValueVorhanden) {
    // --- Bestehende EEPROM Ladevorgänge ---
    EEPROM.get(EEPROM_ADDR_SETPOINT_DAMPF, SetpointDampf);
    EEPROM.get(EEPROM_ADDR_SETPOINT_WASSER, SetpointWasser);
    EEPROM.get(EEPROM_ADDR_OFFSET_DAMPF, OffsetDampf);
    EEPROM.get(EEPROM_ADDR_OFFSET_WASSER, OffsetWasser);
    EEPROM.get(EEPROM_ADDR_KP_DAMPF, KpDampf);
    EEPROM.get(EEPROM_ADDR_KI_DAMPF, KiDampf);
    EEPROM.get(EEPROM_ADDR_KD_DAMPF, KdDampf);
    EEPROM.get(EEPROM_ADDR_KP_WASSER, KpWasser);
    EEPROM.get(EEPROM_ADDR_KI_WASSER, KiWasser);
    EEPROM.get(EEPROM_ADDR_KD_WASSER, KdWasser);
    EEPROM.get(EEPROM_ADDR_ECOMODE_MINUTES, ecoModeMinutes);
    EEPROM.get(EEPROM_ADDR_ECOMODE_TEMP_WASSER, ecoModeTempWasser);
    EEPROM.get(EEPROM_ADDR_ECOMODE_TEMP_DAMPF, ecoModeTempDampf);
    EEPROM.get(EEPROM_ADDR_INFO_HERSTELLER, infoHersteller);
    EEPROM.get(EEPROM_ADDR_INFO_MODELL, infoModell);
    EEPROM.get(EEPROM_ADDR_INFO_ZUSATZ, infoZusatz);
    EEPROM.get(EEPROM_ADDR_DYNAMIC_ECO_MODE, dynamicEcoActive);
    EEPROM.get(EEPROM_ADDR_FASTHEATUP_DATA, fastHeatUpAktiv);
    EEPROM.get(EEPROM_ADDR_STEAM_DELAY, dampfVerzoegerung);
    EEPROM.get(EEPROM_ADDR_TUNING_STEP_WASSER, tuningStepWasser);
    EEPROM.get(EEPROM_ADDR_TUNING_NOISE_WASSER, tuningNoiseWasser);
    EEPROM.get(EEPROM_ADDR_TUNING_STARTVALUE_WASSER, tuningStartValueWasser);
    EEPROM.get(EEPROM_ADDR_TUNING_LOOKBACK_WASSER, tuningLookBackWasser);
    EEPROM.get(EEPROM_ADDR_TUNING_STEP_DAMPF, tuningStepDampf);
    EEPROM.get(EEPROM_ADDR_TUNING_NOISE_DAMPF, tuningNoiseDampf);
    EEPROM.get(EEPROM_ADDR_TUNING_STARTVALUE_DAMPF, tuningStartValueDampf);
    EEPROM.get(EEPROM_ADDR_TUNING_LOOKBACK_DAMPF, tuningLookBackDampf);
    EEPROM.get(EEPROM_ADDR_MAX_TEMP_WASSER, maxTempWasser);
    EEPROM.get(EEPROM_ADDR_MAX_TEMP_DAMPF, maxTempDampf);
    EEPROM.get(EEPROM_ADDR_MAINTENANCE_INTERVAL, maintenanceInterval);
    EEPROM.get(EEPROM_ADDR_BOOST_WASSER_ACTIVE, boostWasserActive);
    EEPROM.get(EEPROM_ADDR_BOOST_DAMPF_ACTIVE, boostDampfActive);
    EEPROM.get(EEPROM_ADDR_PREVENTHEAT_WASSER, preventHeatAboveSetpointWasser);
    EEPROM.get(EEPROM_ADDR_PREVENTHEAT_DAMPF, preventHeatAboveSetpointDampf);
    EEPROM.get(EEPROM_ADDR_WINDOWSIZE_WASSER, windowSizeWasser);
    EEPROM.get(EEPROM_ADDR_WINDOWSIZE_DAMPF, windowSizeDampf);

    // Brew Control Einstellungen laden
    EEPROM.get(EEPROM_ADDR_BREWBYTIME_ENABLED, brewByTimeEnabled);
    EEPROM.get(EEPROM_ADDR_BREWBYTIME_SECONDS, brewByTimeTargetSeconds);
    EEPROM.get(EEPROM_ADDR_PREINF_ENABLED, preInfusionEnabled);
    EEPROM.get(EEPROM_ADDR_PREINF_DUR_SEC, preInfusionDurationSeconds);
    EEPROM.get(EEPROM_ADDR_PREINF_PAUSE_SEC, preInfusionPauseSeconds);
    EEPROM.get(EEPROM_ADDR_BREWBYWEIGHT_ENABLED, brewByWeightEnabled);
    EEPROM.get(EEPROM_ADDR_BREWBYWEIGHT_TARGET, brewByWeightTargetGrams);
    EEPROM.get(EEPROM_ADDR_BREWBYWEIGHT_OFFSET, brewByWeightOffsetGrams);

    // Piezo-Status laden
    EEPROM.get(EEPROM_ADDR_PIEZO_ENABLED, piezoEnabled);

    // Status für Dampfverzögerungs-Override per Schalter laden
    EEPROM.get(EEPROM_ADDR_STEAM_DELAY_OVERRIDE_SWITCH, steamDelayOverrideBySwitchEnabled);
    
    // Sicherstellen, dass Strings nullterminiert sind
    infoHersteller[sizeof(infoHersteller) - 1] = '\0';
    infoModell[sizeof(infoModell) - 1] = '\0';
    infoZusatz[sizeof(infoZusatz) - 1] = '\0';

  } else { // Magic Value fehlt -> ALLES auf Default setzen und speichern
    // Serial.println("Keine gültigen Einstellungen im EEPROM gefunden. Setze Werkseinstellungen.");
    resetToDefaults(); // Diese Funktion setzt globale Variablen UND schreibt ins EEPROM
  }

  // --- WiFi Konfiguration ---
  WiFiConfig wifiConfig;
  bool hasWiFiConfig = loadWiFiConfig(wifiConfig); // Lädt auch Hostname
  WiFi.hostname(hostname); // Hostname setzen VOR begin/softAP

  if (hasWiFiConfig) {
    WiFi.mode(WIFI_STA);
    #ifdef ESP32 // Sicherstellen, dass dies nur für ESP32 gilt
      WiFi.setSleep(false);
    #endif

    if (wifiConfig.useStaticIP) {
        // Serial.println(F("Verwende statische IP-Konfiguration:")); // Optional für Debugging
        // Serial.print(F("  IP: ")); Serial.println(wifiConfig.staticIP); // Optional
        // Serial.print(F("  Gateway: ")); Serial.println(wifiConfig.gateway); // Optional
        // Serial.print(F("  Subnet: ")); Serial.println(wifiConfig.subnet); // Optional
        // Serial.print(F("  DNS: ")); Serial.println(wifiConfig.dns); // Optional

        // Für ESP8266 und ESP32 ist die Reihenfolge der primären DNS-Parameter leicht unterschiedlich,
        // aber ein einzelner DNS-Server sollte funktionieren.
        // ESP8266: WiFi.config(ip, gateway, subnet, dns1, dns2_optional)
        // ESP32:   WiFi.config(ip, gateway, subnet, primaryDNS_optional, secondaryDNS_optional)
        // Wenn Sie nur einen DNS-Server haben, ist `wifiConfig.dns` für beide Plattformen in Ordnung.
        if (!WiFi.config(wifiConfig.staticIP, wifiConfig.gateway, wifiConfig.subnet, wifiConfig.dns)) {
            Serial.println(F("FEHLER: Statische IP-Konfiguration fehlgeschlagen!")); // Optional
        }
    // } else {
    //     Serial.println(F("Verwende DHCP.")); // Optional für Debugging
    }

    // Serial.printf("Verbinde mit SSID: %s, Hostname: %s\n", wifiConfig.ssid, hostname);
    WiFi.begin(wifiConfig.ssid, wifiConfig.password);

    // Warte auf Verbindung (mit Timeout)
    int retries = 0;
    while (WiFi.status() != WL_CONNECTED && retries < 20) {
      delay(500); 
      retries++; 
      yield();
    }

    if (WiFi.status() == WL_CONNECTED) {
      // Serial.print(F("\nErfolgreich verbunden! IP-Adresse: ")); Serial.println(WiFi.localIP());
      // NTP Client und Zeitzone konfigurieren
      timeClient.begin(); 
      // Serial.println("NTP Client gestartet.");
      configTzTime(TZ_INFO, "pool.ntp.org"); 
      // Serial.printf("Zeitzone gesetzt: %s\n", TZ_INFO);

      // Demo Modus Check
      if (WiFi.localIP() == demoModusIP) {
        demoModus = true;
        // Serial.println("INFO: Demo-Modus aktiviert (basierend auf IP-Adresse).");
      }
      // mDNS starten
      if (MDNS.begin(hostname)) {
        // Serial.println("mDNS Responder gestartet");
        MDNS.addService("http", "tcp", 80);
      }
    } else {
      // Serial.println(F("\nKonnte keine Verbindung herstellen! Starte AP-Modus..."));
      startAPMode(); // Funktion startet Soft AP und gibt Infos aus
    }
  } else {
    // Serial.println("Keine WLAN-Konfiguration gefunden. Starte AP-Modus...");
    startAPMode(); // Funktion startet Soft AP und gibt Infos aus
  }
  // --- Ende WiFi Konfiguration ---


  // --- Wichtige Infos auf Serial ausgeben (Alternativ zur Displayausgabe) ---
  // Serial.println("---------------------------------------------");
  // Serial.print("Firmware Version: "); Serial.println(version);

// #ifdef ENABLE_DISPLAY
//   Serial.println("Display: Aktiviert");
// #else
//   Serial.println("Display: Deaktiviert");
// #endif
  // if (WiFi.getMode() == WIFI_AP || WiFi.status() != WL_CONNECTED) {
  //     Serial.println(F("WiFi Modus: Access Point (AP)"));
  //     Serial.print(F("SSID: ")); Serial.println(WiFi.softAPSSID());
  //     Serial.print(F("Passwort: QuickMill")); // Oder was in startAPMode() gesetzt wird
  //     Serial.print(F("AP IP Addresse: ")); Serial.println(WiFi.softAPIP());
  // } else {
  //     Serial.println(F("WiFi Modus: Station (STA)"));
  //     Serial.print(F("Verbunden mit SSID: ")); Serial.println(WiFi.SSID());
  //     Serial.print(F("IP Addresse: ")); Serial.println(WiFi.localIP());
  //     Serial.print(F("Hostname: ")); Serial.print(hostname); Serial.println(".local");
  //     Serial.print(F("Signalstärke: ")); Serial.print(WiFi.RSSI()); Serial.println(" dBm");
  // }
  // Serial.println("---------------------------------------------");
  // --- Ende Serial Ausgabe ---


  // --- Weitere Initialisierungen ---
  // Wartungszähler, Betriebszeit und Shotcounter aus EEPROM laden
  EEPROM.get(EEPROM_ADDR_RUNTIME, totalRuntime);
  if (totalRuntime == ULONG_MAX || totalRuntime > (86400UL * 365 * 10)) { totalRuntime = 0; EEPROM.put(EEPROM_ADDR_RUNTIME, totalRuntime); EEPROM.commit(); }
  EEPROM.get(EEPROM_ADDR_SHOTCOUNTER, shotCounter);
  if (shotCounter == ULONG_MAX || shotCounter > 1000000UL) { shotCounter = 0; EEPROM.put(EEPROM_ADDR_SHOTCOUNTER, shotCounter); EEPROM.commit(); }
  EEPROM.get(EEPROM_ADDR_MAINTENANCE_INTERVAL_COUNTER, maintenanceIntervalCounter);
  if (maintenanceIntervalCounter == ULONG_MAX || maintenanceIntervalCounter > 1000000UL) { maintenanceIntervalCounter = 0; EEPROM.put(EEPROM_ADDR_MAINTENANCE_INTERVAL_COUNTER, maintenanceIntervalCounter); EEPROM.commit(); }

  // Fast Heat Up Status setzen
  if (fastHeatUpAktiv) {
    fastHeatUpHeating = true;
  }


  // --- Display-Initialisierung (nur wenn aktiviert) ---
#ifdef ENABLE_DISPLAY
  if (!display.begin(0x3C, true)) { // display.begin benutzt das ggf. schon initialisierte Wire
    Serial.println(F("Display konnte nicht initialisiert werden!"));
  } else {
      Serial.println(F("Display initialisiert."));
      // Display-Ausgabe am Start (1)
      display.clearDisplay();
      display.setCursor(0, 0);
      display.setTextColor(SH110X_WHITE);
      display.setTextSize(1);
      display.println(infoHersteller);
      display.println(infoModell);
      display.println(infoZusatz);
      display.println("");
      display.println(F("Dual-PID"));
      display.print(F("Version: "));
      display.println(version);
      display.println(F("von Thomas M\201ller")); // \201 ist ü für manche Displays
      display.display();
      delay(delayInit1); // Verzögerung nur bei aktiviertem Display

      // Display-Ausgabe am Start (2) - WiFi Info
      display.clearDisplay();
      display.setCursor(0, 0);
      display.setTextColor(SH110X_WHITE);
      display.setTextSize(1);
      display.println(infoHersteller);
      display.println(infoModell);
      display.println(infoZusatz);
      display.println("");
      if (WiFi.status() != WL_CONNECTED) {
        display.println(F("WiFi: AP-Modus"));
        display.print(F("SSID: ")); display.println(WiFi.softAPSSID());
        display.print(F("IP: ")); display.println(WiFi.softAPIP());
      } else {
        display.println(F("Webserver gestartet"));
        display.print(F("IP: ")); display.println(WiFi.localIP());
        display.print(F("Host: ")); display.print(hostname); display.println(".local");
      }
      display.display();
      delay(delayInit2); // Verzögerung nur bei aktiviertem Display
  }
#endif // ENABLE_DISPLAY


  // --- Waage Initialisierung (nur ESP32) ---
#ifdef ESP32
  #if (SCALE_TYPE == SCALE_I2C)
    // Serial.println("Initialisiere I2C Waage...");
    if (!scales.begin(&Wire)) {
      // Serial.println("FEHLER: I2C Waage nicht gefunden oder Initialisierung fehlgeschlagen!");
      scaleConnected = false;
    } else {
      // Serial.println("I2C Waage erfolgreich initialisiert.");
      scaleConnected = true;
    }
  #elif (SCALE_TYPE == SCALE_ESPNOW)
    // Serial.println("Initialisiere ESP-NOW für Waagen-Empfang...");
    // WiFi ist bereits im STA Modus, was für ESP-NOW Empfang notwendig ist.
    if (esp_now_init() != ESP_OK) {
      // Serial.println("FEHLER: ESP-NOW Initialisierung fehlgeschlagen!");
    } else {
      esp_now_register_recv_cb(OnDataRecv);
      // Serial.println("ESP-NOW für Empfang von Waage bereit.");
      // `scaleConnected` wird `true`, sobald die erste Nachricht empfangen wird.
    }
  #endif
#endif


  // --- PID-Regler Konfiguration ---
  // Plausibilitätscheck für Fenstergrößen
  if (windowSizeWasser == 0 || windowSizeWasser > 60000) {
    // Serial.printf("WARNUNG: Ungültige windowSizeWasser (%lu), setze auf Default (%lu).\n", windowSizeWasser, defaultWindowSizeWasser);
    windowSizeWasser = defaultWindowSizeWasser;
    EEPROM.put(EEPROM_ADDR_WINDOWSIZE_WASSER, windowSizeWasser); // Korrigieren
    EEPROM.commit();
  }
  if (windowSizeDampf == 0 || windowSizeDampf > 60000) {
     // Serial.printf("WARNUNG: Ungültige windowSizeDampf (%lu), setze auf Default (%lu).\n", windowSizeDampf, defaultWindowSizeDampf);
    windowSizeDampf = defaultWindowSizeDampf;
    EEPROM.put(EEPROM_ADDR_WINDOWSIZE_DAMPF, windowSizeDampf); // Korrigieren
    EEPROM.commit();
  }
  pidWasser.SetOutputLimits(0, windowSizeWasser);
  pidDampf.SetOutputLimits(0, windowSizeDampf);
  pidWasser.SetTunings(KpWasser, KiWasser, KdWasser);
  pidDampf.SetTunings(KpDampf, KiDampf, KdDampf);
  pidWasser.SetMode(AUTOMATIC);
  pidDampf.SetMode(AUTOMATIC);
  windowStartTimeWasser = millis();
  windowStartTimeDampf = millis();
  // Serial.printf("PID Wasser: Kp=%.2f, Ki=%.2f, Kd=%.2f, Win=%lu\n", KpWasser, KiWasser, KdWasser, windowSizeWasser);
  // Serial.printf("PID Dampf : Kp=%.2f, Ki=%.2f, Kd=%.2f, Win=%lu\n", KpDampf, KiDampf, KdDampf, windowSizeDampf);


  // --- LittleFS Initialisierung ---
  // Serial.println("Initialisiere LittleFS...");
  if (!LittleFS.begin()) {
    // Serial.println("--------------------------------------------------");
    // Serial.println("FEHLER: LittleFS Mount fehlgeschlagen!");
    
    // Versuch zu formatieren (Achtung: löscht alle Daten!)
    // Serial.println("Versuche LittleFS zu formatieren...");
    if (!LittleFS.format()) {
    // Serial.println("FEHLER: LittleFS Formatierung fehlgeschlagen!");
    } else {
        // Serial.println("LittleFS formatiert. Versuche erneut zu mounten...");
        if (!LittleFS.begin()) {
          // Serial.println("LittleFS Mount auch nach Formatierung fehlgeschlagen!");
        } else {
            // Serial.println("LittleFS nach Formatierung erfolgreich gemountet.");
        }
    }
  } else {
    // Serial.println("LittleFS erfolgreich gemountet.");
    // Prüfe/Erstelle /Profile Verzeichnis
    if (!LittleFS.exists("/Profile")) {
      if (LittleFS.mkdir("/Profile")) {
        // Serial.println("Verzeichnis '/Profile' erstellt.");
      } else {
        // Serial.println("FEHLER: Konnte Verzeichnis '/Profile' nicht erstellen!");
      }
    } else {
      // Serial.println("Verzeichnis '/Profile' gefunden.");
    }
    // Optional: FS Info ausgeben
    // #ifdef ESP32
    //     uint64_t totalBytes = LittleFS.totalBytes();
    //     uint64_t usedBytes = LittleFS.usedBytes();
    //     Serial.printf("LittleFS Info: Total= %llu, Used= %llu, Free= %llu Bytes\n", totalBytes, usedBytes, totalBytes - usedBytes);
    // #else
    //     FSInfo fs_info;
    //     if(LittleFS.info(fs_info)) {
    //          Serial.printf("LittleFS Info: Total= %u, Used= %u, Free= %u Bytes\n", fs_info.totalBytes, fs_info.usedBytes, fs_info.totalBytes - fs_info.usedBytes);
    //     }
    // #endif
  } // Ende des LittleFS-Blocks


  // --- Routen für Webserver definieren ---
  // Serial.println("Definiere Webserver Routen...");
  asyncServer.on("/", HTTP_GET, handleDashboard); // Dashboard als Hauptseite
  asyncServer.on("/dashboard-data", HTTP_GET, handleDashboardData);
  asyncServer.on("/dashboard-action", HTTP_POST, handleDashboardAction);
  asyncServer.on("/PID", HTTP_GET, handlePidSettings);
  asyncServer.on("/updateSettings", HTTP_POST, handleSettingsUpdate);
  asyncServer.on("/Info", HTTP_GET, handleInfo);
  asyncServer.on("/updateInfoSettings", HTTP_POST, handleInfoUpdate);
  asyncServer.on("/downloadNutzungsstatistik", handleDownloadNutzungsstatistik);
  asyncServer.on("/deleteStatistics", HTTP_POST, handleDeleteStatistics);
  asyncServer.on("/toggleWartungsmodus", HTTP_POST, handleToggleWartungsmodus); // In Service integriert
  asyncServer.on("/Service", HTTP_GET, handleService); // Eigene Seite für Wartung, Reset etc.
  asyncServer.on("/AutoTune-Wasser", handleAutoTuneWasser);
  asyncServer.on("/AutoTune-Dampf", handleAutoTuneDampf);
  asyncServer.on("/updateAutotuneSettings", HTTP_POST, handleUpdateAutotuneSettings);
  asyncServer.on("/PID-Tuning-Abbruch", HTTP_POST, handlePidSettingsTuningAbbruch);
  asyncServer.on("/PID-Tuning", handlePidSettingsTuning); // Eigene Seite für AutoTune Start & Parameter
  asyncServer.on("/ECO", HTTP_GET, handleEco);
  asyncServer.on("/updateEcoSettings", HTTP_POST, handleEcoUpdate);
  asyncServer.on("/Firmware", HTTP_GET, handleFirmware); // Kombinierte Firmware & Settings Seite
  asyncServer.on("/update", HTTP_POST, handleFirmwareUpdateResponse, handleFirmwareUploadProgress);
  asyncServer.on("/Fast-Heat-Up", HTTP_GET, handleFastHeatUp);
  asyncServer.on("/updateFast-Heat-Up-Settings", HTTP_POST, handleFastHeatUpSettings);
  asyncServer.on("/Dateimanager", HTTP_GET, handleFileManager);
  asyncServer.on("/Datei-Upload", HTTP_POST, handleFileUploaded, handleFileUpload);
  asyncServer.on("/Datei-Entfernen", HTTP_GET, handleFileDelete);
  asyncServer.on("/Datei-Download", HTTP_GET, handleFileDownload);
  asyncServer.on("/Verzeichnis-Erstellen", HTTP_POST, handleCreateDirectory);
  asyncServer.on("/Chart-Daten", HTTP_GET, handleChartData);
  asyncServer.on("/Chart", HTTP_GET, handleCharts); // Eigene Seite für die Charts
  asyncServer.on("/Netzwerk", HTTP_GET, handleWiFiConfig); // Eigene Seite für WiFi
  asyncServer.on("/saveWiFiConfig", HTTP_POST, handleSaveWiFiConfig);
  asyncServer.on("/forceAPMode", HTTP_GET, handleForceAPMode);
  asyncServer.on("/Profile", HTTP_GET, handleProfilesPage); // Eigene Seite für Profile
  asyncServer.on("/loadProfile", HTTP_POST, handleLoadProfile);
  asyncServer.on("/saveProfile", HTTP_POST, handleSaveProfile);
  asyncServer.on("/deleteProfile", HTTP_POST, handleDeleteProfile);
  asyncServer.on("/exportSettings", HTTP_GET, handleExportSettings); // In Firmware-Seite integriert
  asyncServer.on("/importSettings", HTTP_POST, handleImportSettings, handleImportUpload); // In Firmware-Seite integriert
  asyncServer.on("/resetDefaults", HTTP_POST, handleResetDefaults); // In Firmware-Seite integriert
  asyncServer.on("/updatePiezoSettings", HTTP_POST, handleUpdatePiezoSettings); // Service-Seite
  asyncServer.on("/updateIntervalSettings", HTTP_POST, handleUpdateIntervalSettings); // Service-Seite
  asyncServer.on("/resetRuntime", HTTP_POST, handleResetRuntime); // In Info-Seite integriert
  asyncServer.on("/resetShots", HTTP_POST, handleResetShots); // In Info-Seite integriert
  asyncServer.on("/resetMaintenance", HTTP_POST, handleResetMaintenance); // In Service-Seite integriert
  asyncServer.on("/restartDevice", HTTP_POST, handleRestartDevice); // Neustart des Microcontrollers


#ifdef ESP32
  // Routen für Brew Control (Nur ESP32)
  asyncServer.on("/Brew-Control", HTTP_GET, handleBrewControl);
  asyncServer.on("/saveBrewControl", HTTP_POST, handleSaveBrewControl);
#endif

  // Webserver starten
  asyncServer.begin();
  yield(); // Dem System kurz Zeit geben

  // Serial.println(F("Webserver gestartet und bereit."));
  // Serial.println("---------------------------------------------");
  // Serial.println("Setup abgeschlossen. Hauptschleife beginnt.");
  // Serial.println("---------------------------------------------");

  startupTime = millis(); // Startzeit für Dampfverzögerung speichern
  yield(); // Dem System kurz Zeit geben
} // Ende setup()

void handleAutoTune(AsyncWebServerRequest *request = nullptr);

/************************************************************************************
 * loop(): Hauptschleife
 * Enthält Logik für Pre-Infusion Ablauf und Brew-By-Time Check (Nur ESP32)
 ************************************************************************************/
void loop() {
  unsigned long currentMillis = millis();

  if (scheduledRestart && currentMillis >= scheduledRestart) {
    ESP.restart();
  }

  // Prüfen, ob Zeit bereits synchronisiert wurde
  if (!timeSynced && WiFi.status() == WL_CONNECTED && timeClient.update()) { timeSynced = true; }

  // --- Temperaturmessung und PID-Berechnung (im Intervall) ---
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Temperaturmessung
    InputDampf = round(thermocouple.readCelsius() + OffsetDampf);
    InputWasser = round(readNTCTemperature() + OffsetWasser);
    yield();

    // --- Angepasste Sicherheitsüberprüfung Wasser ---
    if (InputWasser < 0.0 || isnan(InputWasser)) {
      wasserSensorError = true; 
      wasserSafetyShutdown = false; 
      OutputWasser = 0;
      // Ton ausgeben zur Warnung
      #ifdef ESP32 // Nur piepen, wenn auf ESP32
        beepShort();
      #endif
      // Serial.println("WARNUNG: Wassertemperatursensor liefert ungültigen Wert!"); // Optional Log
    } else if (InputWasser > maxTempWasser) {
      wasserSensorError = false; 
      wasserSafetyShutdown = true; 
      OutputWasser = 0;
      // Ton ausgeben zur Warnung
      #ifdef ESP32 // Nur piepen, wenn auf ESP32
        beepShort();
      #endif
      // Serial.printf("WARNUNG: Wassertemperatur über Limit! Shutdown.\n"); // Optional Log
    } else {
      if (wasserSensorError) Serial.println("INFO: Wassertemperatursensor wieder OK."); wasserSensorError = false;
      if (wasserSafetyShutdown) Serial.println("INFO: Wassertemperatur wieder im sicheren Bereich."); wasserSafetyShutdown = false;
      bool activateBoost = boostWasserActive && !wartungsModusAktiv && !ecoModeAktiv && !autoTuneWasserActive && (InputWasser < (SetpointWasser * 0.95));
      if (activateBoost) { OutputWasser = windowSizeWasser; } else if (!autoTuneWasserActive) { pidWasser.Compute(); }
    }
    yield();

    // --- Angepasste Sicherheitsüberprüfung Dampf ---
    if (isnan(InputDampf) || InputDampf < 0.0) {
        dampfSensorError = true; 
        dampfSafetyShutdown = false; 
        OutputDampf = 0;
      // Ton ausgeben zur Warnung
      #ifdef ESP32 // Nur piepen, wenn auf ESP32
        beepShort();
      #endif
        // Serial.println("WARNUNG: Dampftemperatursensor liefert ungültigen Wert!"); // Optional Log
    } else if (InputDampf > maxTempDampf) {
        dampfSensorError = false;
        dampfSafetyShutdown = true; 
        OutputDampf = 0;
      // Ton ausgeben zur Warnung
      #ifdef ESP32 // Nur piepen, wenn auf ESP32
        beepShort();
      #endif
        // Serial.printf("WARNUNG: Dampftemperatur über Limit! Shutdown.\n"); // Optional Log
    } else {
        if (dampfSensorError) Serial.println("INFO: Dampftemperatursensor wieder OK."); dampfSensorError = false;
        if (dampfSafetyShutdown) Serial.println("INFO: Dampftemperatur wieder im sicheren Bereich."); dampfSafetyShutdown = false;
        bool steamDelayActive = (dampfVerzoegerung > 0 && (millis() - startupTime < (unsigned long)dampfVerzoegerung * 60000UL));
        if (steamDelayActive && !wartungsModusAktiv) { OutputDampf = 0; } else {
            bool activateBoostDampf = boostDampfActive && !wartungsModusAktiv && !ecoModeAktiv && !autoTuneDampfActive && (InputDampf < (SetpointDampf * 0.90));
            if (activateBoostDampf) { OutputDampf = windowSizeDampf; } else if (!autoTuneDampfActive) { pidDampf.Compute(); }
        }
    }
    yield();

    // Prüfen der Startverzögerung für Dampf
    bool steamDelayActiveSystem = (dampfVerzoegerung > 0 && (millis() - startupTime < (unsigned long)dampfVerzoegerung * 60000UL));
    // Prüfe, ob WIRKLICH verzögert werden soll (System will UND kein Override aktiv)
    bool shouldDelaySteamPID = steamDelayActiveSystem && !steamDelayOverridden;

    // Nur fortfahren, wenn kein Sensorfehler und keine Sicherheitsabschaltung aktiv ist
    if (!dampfSensorError && !dampfSafetyShutdown)
    {
          if (shouldDelaySteamPID && !wartungsModusAktiv) {
                // Wenn verzögert werden soll, erzwinge PID-Output auf 0
                OutputDampf = 0;
                // Serial.println("Dampf-PID-Berechnung wegen Verzögerung übersprungen."); // Optional Debug
          } else {
                // Keine Verzögerung ODER Override aktiv -> PID normal berechnen/Boost prüfen
                bool activateBoostDampf = boostDampfActive && !wartungsModusAktiv && !ecoModeAktiv && !autoTuneDampfActive && (InputDampf < (SetpointDampf * 0.90));
                if (activateBoostDampf) {
                    OutputDampf = windowSizeDampf; // Boost hat Vorrang
                } else if (!autoTuneDampfActive) {
                    pidDampf.Compute(); // PID normal berechnen lassen (jetzt sicher)
                }
                // Wenn AutoTune aktiv ist, wird OutputDampf dort gesetzt (in handleAutoTune)
          }
    }

#ifdef ESP32
  #if (SCALE_TYPE == SCALE_I2C)
    // Waage nur bei I2C-Typ aktiv auslesen
    if (scaleConnected && (scaleModeActive || (shotActive && brewByWeightEnabled))) {
        currentWeightReading = scales.getWeight();
    }
  #elif (SCALE_TYPE == SCALE_ESPNOW)
    // Für ESP-NOW passiert hier nichts, der Wert wird automatisch
    // im Hintergrund durch die OnDataRecv-Funktion aktualisiert.
  #endif

    if (!scaleConnected) {
        currentWeightReading = 0.0f; // Dies gilt für beide Typen
    }
#endif

  // AutoTune-Logik
  handleAutoTune();

  } // Ende des Temperaturmessungs-/PID-Berechnungsintervalls
  yield();

  // --- Zeitproportionale SSR-Ansteuerung (wird kontinuierlich geprüft) ---
  unsigned long now = millis();
  // Wasser SSR
  if (wasserSensorError || wasserSafetyShutdown) { digitalWrite(SSR_WASSER_PIN, LOW); } else {
    if (now - windowStartTimeWasser > windowSizeWasser) { windowStartTimeWasser += windowSizeWasser; }
    bool pidWantsHeat = (OutputWasser > 0) && (now < (windowStartTimeWasser + (unsigned long)OutputWasser));
    bool preventOverheatActive = (preventHeatAboveSetpointWasser && (InputWasser > SetpointWasser)) || (ecoModeAktiv && (InputWasser > SetpointWasser));
    if (pidWantsHeat && !preventOverheatActive) { digitalWrite(SSR_WASSER_PIN, HIGH); } else { digitalWrite(SSR_WASSER_PIN, LOW); }
  }
  yield();

  // Dampf SSR
  if (dampfSensorError || dampfSafetyShutdown) {
      digitalWrite(SSR_DAMPF_PIN, LOW);
  } else {
      if (now - windowStartTimeDampf > windowSizeDampf) { windowStartTimeDampf += windowSizeDampf; }

      // Prüfe, ob die Verzögerung aktiv UND NICHT überschrieben ist
      bool steamDelayActiveSystem = (dampfVerzoegerung > 0 && (millis() - startupTime < (unsigned long)dampfVerzoegerung * 60000UL));
      bool shouldDelaySteam = steamDelayActiveSystem && !steamDelayOverridden;

      if (shouldDelaySteam && !wartungsModusAktiv) { // Wenn verzögert werden soll (und kein Wartungsmodus)
          OutputDampf = 0; // PID Output überschreiben (wird unten zum Abschalten führen)
          digitalWrite(SSR_DAMPF_PIN, LOW); // Sicherstellen, dass SSR aus ist
          // Optional: Logge, dass Verzögerung aktiv ist
          // Serial.println("Dampfverzögerung aktiv.");
      } else { // Keine Verzögerung ODER überschrieben -> Normale PID-Logik
          bool pidWantsHeat = (OutputDampf > 0) && (now < (windowStartTimeDampf + (unsigned long)OutputDampf));
          bool preventOverheatActive = (preventHeatAboveSetpointDampf && (InputDampf > SetpointDampf)) || (ecoModeAktiv && (InputDampf > SetpointDampf));

          if (pidWantsHeat && !preventOverheatActive && !wartungsModusAktiv) { // Zusätzliche Prüfung auf Wartungsmodus hier
                digitalWrite(SSR_DAMPF_PIN, HIGH);
          } else {
                digitalWrite(SSR_DAMPF_PIN, LOW);
          }
          // PID muss normal weiterlaufen, auch wenn SSR aus ist, daher keine Output-Manipulation mehr hier nötig,
          // außer wenn explizit verzögert wird (siehe oben).
      }
  }
  yield();

#ifdef ESP32
  // *** Brüh-Logik (Pre-Infusion, Brew-by-Time, Brew-by-Weight) ***
  if (shotActive) {
      unsigned long currentShotTimeMillis = millis() - shotStartTime;

      // --- Waage lesen ---
    #if (SCALE_TYPE == SCALE_I2C)
      if (scaleConnected) {
          currentWeightReading = scales.getWeight(); // Lese Rohwert aktiv nur für I2C
      }
    #endif
    // Für ESP-NOW wird currentWeightReading im Hintergrund aktualisiert.

      if (!scaleConnected) { // Gemeinsame Logik für "nicht verbunden"
          currentWeightReading = 0.0f;
      }

      // --- Stopp-Bedingungen prüfen ---
      float netWeight = currentWeightReading - brewStartWeight; // Berechne das Nettogewicht des Kaffees
      if (!firstWeightChangeDetected && scaleConnected && netWeight >= WEIGHT_CHANGE_THRESHOLD) {
          firstWeightChangeDetected = true;
          firstWeightChangeTime = millis();
      }
      float targetWeightToStopAt = brewByWeightTargetGrams - brewByWeightOffsetGrams;
      bool stopForTime = brewByTimeEnabled && (currentShotTimeMillis >= (unsigned long)(brewByTimeTargetSeconds * 1000.0f));
      bool stopForWeight = brewByWeightEnabled && scaleConnected && (netWeight >= targetWeightToStopAt);

      if (stopForTime || stopForWeight) {
          // if (stopForTime) Serial.println("Brew-By-Time Limit erreicht. Stoppe Bezug...");
          // if (stopForWeight) Serial.printf("Brew-By-Weight Limit erreicht (Aktuell: %.1fg, Ziel: %.1fg). Stoppe Bezug...\n", currentWeightReading, targetWeightToStopAt);
          stopBrewSequence(currentShotTimeMillis, false, stopForWeight); // Übergibt die aktuelle Dauer und den Grund des Stopps
      }

      // --- Pre-Infusion Zustandsmaschine (Nur ausführen, wenn noch nicht gestoppt wurde) ---
      else if (preInfusionEnabled) {
          unsigned long currentPhaseTimeMillis = millis() - preInfusionPhaseStartTime;

          // Wechsel von Pre-Infusion zu Pause
          if (currentPreInfusionState == PI_PRE_BREW && (currentPhaseTimeMillis >= (unsigned long)(preInfusionDurationSeconds * 1000.0f))) {
              // Serial.println("Pre-Infusion -> Pause");
              digitalWrite(PUMP_PIN, LOW);
              currentPreInfusionState = PI_PAUSE;
              preInfusionPhaseStartTime = millis();
          }
          // Wechsel von Pause zu Hauptbezug
          else if (currentPreInfusionState == PI_PAUSE && (currentPhaseTimeMillis >= (unsigned long)(preInfusionPauseSeconds * 1000.0f))) {
              // Serial.println("Pause -> Main Brew");
              digitalWrite(PUMP_PIN, HIGH);
              currentPreInfusionState = PI_MAIN_BREW;
              preInfusionPhaseStartTime = millis();
          }
      }
      // Kein else hier -> Entweder Stopp, oder PreInfusion-Check, oder normaler Bezug ohne PreInfusion läuft weiter
  }
#endif // ESP32

  // Shot-Timer (Diese Funktion initiiert nur den Start oder stoppt bei manuellem Ende)
  updateShotTimer(); // Ruft Waage tarren auf, wenn der Shot beginnt

  // Eco-Modus, wenn kein AutoTune oder Wartungsmodus
  if (!autoTuneWasserActive && !autoTuneDampfActive && !wartungsModusAktiv) {
      if (ecoModeMinutes > 0 && !shotActive) { // Eco nur wenn kein Shot läuft
          unsigned long currentTimeEco = millis();
          if (currentTimeEco - lastShotTime > (ecoModeMinutes * 60UL * 1000UL)) {
              if (!ecoModeAktiv) { ecoModeActivatedTime = currentTimeEco; }
              ecoModeAktiv = true;        
              if (dynamicEcoActive) {
                  int minutesPassed = (currentTimeEco - ecoModeActivatedTime) / (60UL * 1000UL);
                  SetpointWasser = max(0, ecoModeTempWasser - minutesPassed); SetpointDampf = max(0, ecoModeTempDampf - minutesPassed);
              } else {
                  SetpointWasser = ecoModeTempWasser; SetpointDampf = ecoModeTempDampf;
              }
          } else {
              if (ecoModeAktiv) { EEPROM.get(EEPROM_ADDR_SETPOINT_WASSER, SetpointWasser); EEPROM.get(EEPROM_ADDR_SETPOINT_DAMPF, SetpointDampf); }
              ecoModeAktiv = false; ecoModeActivatedTime = 0;
          }
      } else if (ecoModeAktiv) {
          ecoModeAktiv = false; ecoModeActivatedTime = 0;
          EEPROM.get(EEPROM_ADDR_SETPOINT_WASSER, SetpointWasser); EEPROM.get(EEPROM_ADDR_SETPOINT_DAMPF, SetpointDampf);
      }
      yield();
  } else if (ecoModeAktiv) {
      ecoModeAktiv = false; ecoModeActivatedTime = 0;
      EEPROM.get(EEPROM_ADDR_SETPOINT_WASSER, SetpointWasser); EEPROM.get(EEPROM_ADDR_SETPOINT_DAMPF, SetpointDampf);
  }

// Display regelmäßig updaten, wenn aktiviert / angeschlossen
#ifdef ENABLE_DISPLAY
  updateDisplay(); 
#endif // ENABLE_DISPLAY

// --- Waagen-spezifische Logik in der Hauptschleife ---
#ifdef ESP32
  #if (SCALE_TYPE == SCALE_I2C)
    // Logik nur für I2C-Waage
    if (scaleConnected) {
      handleScaleButtonPress(nullptr);
    }
    // Verzögertes Tarieren für I2C-Waage ausführen
    if (tareScaleAfterDelay && scaleConnected && (currentMillis - tareScaleDelayStartTime >= TARE_SCALE_DELAY_DURATION)) {
      scales.setOffset();
      tareScaleAfterDelay = false;
      tareScaleSettling = true;
      tareScaleSettlingStartTime = currentMillis;
    }
    if (tareScaleSettling && scaleConnected && (currentMillis - tareScaleSettlingStartTime >= TARE_SCALE_SETTLING_DURATION)) {
      currentWeightReading = scales.getWeight();
      tareScaleSettling = false;
      Serial.println(F("Waage nach Verzögerung tariert."));
      if (piezoEnabled) {
        beepShort(2800, 50);
      }
    }
  #elif (SCALE_TYPE == SCALE_ESPNOW)
    // Logik nur für ESP-NOW-Waage: Timeout-Prüfung
    if (scaleConnected && (millis() - lastScaleMessageTime > SCALE_TIMEOUT)) {
      scaleConnected = false;
      scaleModeActive = false; // Modus ebenfalls zurücksetzen
      currentWeightReading = 0.0;
      Serial.println("WARNUNG: Verbindung zur ESP-NOW Waage verloren (Timeout).");
    }
  #endif
#endif

  // AsyncWebServer verarbeitet Clients im Hintergrund, kein handleClient erforderlich

#ifdef ESP8266
  MDNS.update();
#endif
  yield();

  // Betriebszeit zählen
  static unsigned long lastRuntimeSecond = 0;
  if (millis() - lastRuntimeSecond >= 1000) { totalRuntime++; lastRuntimeSecond = millis(); }

  // Speichern der Betriebszeit und WiFi-Check
  if (!demoModus) {
    unsigned long currentMillisLoop = millis();
    unsigned long intervalToUse = !initialRuntimeSaveDone ? initialRuntimeSaveDelay : subsequentRuntimeSaveInterval;
    if (currentMillisLoop - lastRuntimeSave >= intervalToUse) {
      EEPROM.put(EEPROM_ADDR_RUNTIME, totalRuntime);
      EEPROM.commit();
      lastRuntimeSave = currentMillisLoop;
      if (!initialRuntimeSaveDone) initialRuntimeSaveDone = true;
      if (WiFi.getMode() == WIFI_STA) checkWifiConnection();
    }
  }
  yield();

  // Zur Speicherung des Bezugszählers
  if (!shotActive) {
    handleShotSaving(nullptr);
  }
  yield();

} // Ende loop()

/************************************************************************************
 * Liest die Temperatur vom NTC-Sensor (Wasser) über den Spannungsteiler (ADC)
 * - Verwendet jetzt ANALOG_NTC_PIN für Plattformkompatibilität
 ************************************************************************************/
double readNTCTemperature() {
    double sensorVoltage = NAN; // Mit Fehlerwert initialisieren
    int adcValue = analogRead(ANALOG_NTC_PIN);

    #ifdef ESP32
        // --- ESP32 Logik ---
        const double MAX_ADC_VALUE = 4095.0; // ESP32 Standard 12-bit

        // Prüfen auf ungültige ADC-Werte (ESP32)
        // Kann für Debugging auskommentiert werden - Kommentiert, damit Demo-Controller ohne permanente Meldungen aufgesetzt werden kann
        // if (adcValue <= 0 || adcValue >= MAX_ADC_VALUE) {
        //     Serial.print("WARNUNG (ESP32): Ungültiger ADC-Wert vom NTC-Sensor: ");
        //     Serial.println(adcValue);
        //     return NAN; // Not-a-Number als Fehlerwert zurückgeben
        // }

        // Versuche kalibrierte Spannung zu erhalten (adc_chars muss vorher initialisiert sein!)
        uint32_t voltage_mv = esp_adc_cal_raw_to_voltage(adcValue, &adc_chars);

        if (voltage_mv > 0) { // Prüfen, ob Kalibrierung erfolgreich / plausibel war
             sensorVoltage = voltage_mv / 1000.0; // Kalibrierte Spannung nutzen
             // Serial.print("ESP32 Calibrated Voltage: "); 
             // Serial.println(sensorVoltage); // Zum Debuggen
        } else {
             // Fallback auf lineare Berechnung, wenn Kalibrierung nicht geht/konfiguriert ist
             // Serial.println("WARNUNG (ESP32): ADC Kalibrierung nicht verfügbar/erfolgreich. Nutze lineare Schätzung.");
             // ADC_REF muss definiert sein (z.B. 3.3 für 3.3V Referenz)
             sensorVoltage = adcValue * (ADC_REF / MAX_ADC_VALUE);
             // Serial.print("ESP32 Linear Voltage: "); 
             // Serial.println(sensorVoltage); // Zum Debuggen
        }
        // Optional: ADC-Konfiguration hier oder global setzen (z.B. Attenuation für vollen Range)
        // analogSetAttenuation(ADC_11db);
        // analogReadResolution(12);

    #else // ESP8266 Logik
        // --- ESP8266 Logik ---
        const double MAX_ADC_VALUE = 1023.0; // ESP8266 10-bit

        // Prüfen auf ungültige ADC-Werte (ESP8266)
        // Kann für Debugging auskommentiert werden - Kommentiert, damit Demo-Controller ohne permanente Meldungen aufgesetzt werden kann
        // if (adcValue <= 0 || adcValue >= MAX_ADC_VALUE) {
        //     Serial.print("WARNUNG (ESP8266): Ungültiger ADC-Wert vom NTC-Sensor: ");
        //     Serial.println(adcValue);
        //     return NAN; // Not-a-Number als Fehlerwert zurückgeben
        // }

        // Lineare Berechnung für ESP8266
        // WICHTIG: Stelle sicher, dass ADC_REF korrekt ist!
        // Wenn der Spannungsteiler mit V_SUPPLY (z.B. 3.3V) betrieben wird, sollte ADC_REF dieser Spannung entsprechen.
        // Siehe Kommentar im alten Code bzgl. interner 1.0V Referenz vs. externer Beschaltung.
        // Annahme hier: ADC_REF ist die Spannung bei MAX_ADC_VALUE für die externe Schaltung.
        sensorVoltage = adcValue * (ADC_REF / MAX_ADC_VALUE); // ADC_REF muss definiert sein (z.B. 3.3)
         // Serial.print("ESP8266 Linear Voltage: "); Serial.println(sensorVoltage); // Zum Debuggen

    #endif

    // --- Gemeinsame NTC-Berechnung & Fehlerprüfungen ---

    // Prüfen, ob sensorVoltage gültig berechnet wurde
    if (isnan(sensorVoltage)) {
         // Serial.println("FEHLER: sensorVoltage konnte nicht ermittelt werden.");
         return NAN;
    }

    // Vermeide Division durch Null oder ungültige Bedingungen
    if (sensorVoltage <= 0) { // <= 0 fängt auch den Fall ab, der R_ntc negativ machen würde
       // Serial.println("WARNUNG: NTC-Spannung ist 0 oder negativ nach Berechnung.");
       return NAN;
    }
     if (sensorVoltage >= V_SUPPLY) {
       // Serial.println("WARNUNG: NTC-Spannung entspricht Versorgungsspannung, R_ntc wäre unendlich.");
       return NAN; // Fehler
    }

    // NTC Widerstand berechnen
    double R_ntc = R_FIXED * ((V_SUPPLY / sensorVoltage) - 1.0);

    if (R_ntc <= 0) { // Sollte durch die Prüfungen oben eigentlich nicht mehr vorkommen, aber sicher ist sicher
        // Serial.print("WARNUNG: Berechneter NTC-Widerstand ungültig: ");
        // Serial.println(R_ntc);
        return NAN; // Fehler
    }

    // Temperatur über Beta-Gleichung berechnen
    double log_R_NTC_R0 = log(R_ntc / NTC_R0); // log() braucht positiven Wert, ist durch R_ntc > 0 sichergestellt
    double temp_inv_K = (1.0 / NTC_T0) + (1.0 / beta) * log_R_NTC_R0;

    if (temp_inv_K == 0) { // Vermeide Division durch Null
        // Serial.println("WARNUNG: Kehrwert der Kelvin-Temperatur ist 0.");
        return NAN; // Fehler
    }

    double temperatureK = 1.0 / temp_inv_K;
    double temperatureC = temperatureK - 273.15;

    // Optional: Plausibilitätsprüfung
    // if (temperatureC < -50 || temperatureC > 200) { ... }

    return temperatureC;
}

/************************************************************************************
 * Aktualisiert das Display (Temperaturanzeige, Shot-Timer, Wartung etc.)
 * Priorität: PID Tuning > Fast Heat Up > Aktiver Shot > Letzte Shot-Zeit > Wartung > Temp/Shutdown/Fehler
 * Verwendet eine Zustandsmaschine für die Anzeigen nach einem Shot.
 ************************************************************************************/
#ifdef ENABLE_DISPLAY
void updateDisplay() {
  // =================================================================================
  // HÖCHSTE PRIORITÄT: Spezielle Vollbild-Anzeigen, die alles andere überschreiben
  // =================================================================================

#ifdef ESP32 
  // --- Anzeige: Bezugsschalter nach Auto-Stopp zurückstellen ---
  if (!manualSwitchReleasedAfterAutoStop) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 0);
    display.println(F("Brew Control:"));
    display.println(F("")); 
    display.println(F("Bitte Bezugsschalter"));
    display.println(F("zur\201ckstellen."));
    if (WiFi.status() == WL_CONNECTED) {
      drawwifiSymbol(113, 0);
    }
    display.display();
    return; // WICHTIG: Funktion hier beenden
  }

  // --- Anzeige: Waage-Modus aktiv ---
if (scaleModeActive) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 0);
    display.println(F("Waage:"));
    display.println(F(""));
    display.setTextSize(3);
    display.setCursor(0, 20);
    if (scaleConnected) {
        // --- START ÄNDERUNG ---
        float weightToShow = currentWeightReading;
        // Verhindert die Anzeige von "-0.0" bei sehr kleinen negativen Werten
        if (weightToShow < 0.0 && weightToShow > -0.05) {
            weightToShow = 0.0;
        }
        
        char weightBuffer[10];
        // Benutzen Sie die neue Variable für die Anzeige
        snprintf(weightBuffer, sizeof(weightBuffer), "%.1f g", weightToShow);
        // --- ENDE ÄNDERUNG ---

        int16_t x1, y1;
        uint16_t w, h;
        display.getTextBounds(weightBuffer, 0, 0, &x1, &y1, &w, &h);
        display.setCursor((display.width() - w) / 2, 20);
        display.println(weightBuffer);
    } else {
        display.setTextSize(1);
        display.setCursor(0, 20);
        display.println(F("Waage nicht"));
        display.setCursor(0, 30);
        display.println(F("verbunden!"));
    }
    display.setTextSize(1);
    if (WiFi.status() == WL_CONNECTED) {
        drawwifiSymbol(113, 0);
    }
    display.display();
    return; // WICHTIG: Funktion hier beenden
  }
#endif // ESP32

  // --- Anzeige: Wartungsmodus aktiv ---
  if (wartungsModusAktiv) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.println(F("Wartungsmodus"));
    display.println(F("ist aktiv!"));
    display.println(F(""));
    display.println(F("Temperaturen werden"));
    display.println(F("auf "));
    display.print(wartungsModusTemp);
    display.print(F(" "));
    display.print((char)247);
    display.print(F("C gehalten."));
    if (WiFi.status() == WL_CONNECTED) {
      drawwifiSymbol(113, 0);
    }
    display.display();
    return; // Funktion hier beenden
  }

  // --- Anzeige: PID-Tuning aktiv ---
  if (autoTuneWasserActive || autoTuneDampfActive) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.println(F("PID-Tuning aktiv:"));
    display.println(autoTuneWasserActive ? F("Wasser-PID") : F("Dampf-PID"));
    display.println("");
    int currentPeakCount = 0;
    int currentPeakType = 0;
    if (autoTuneWasserActive && autoTuneWasser != nullptr) {
      currentPeakCount = autoTuneWasser->getPeakCount();
      currentPeakType = autoTuneWasser->getPeakType();
    } else if (autoTuneDampfActive && autoTuneDampf != nullptr) {
      currentPeakCount = autoTuneDampf->getPeakCount();
      currentPeakType = autoTuneDampf->getPeakType();
    }
    display.print(F("Cycle: "));
    display.print( (currentPeakCount < 0 || currentPeakCount > 10) ? "0" : String(currentPeakCount) );
    display.println(F(" von 5-10"));
    display.print(F("Phase: "));
    if (currentPeakType == 1) { display.print(F("Heizen")); }
    else if (currentPeakType == -1) { display.print(F("K\201hlen")); }
    else { display.print(F("Init")); }
    display.println();
    display.print("Temperatur:");
    display.print((int)(autoTuneWasserActive ? InputWasser : InputDampf));
    display.print(F(" "));
    display.print((char)247);
    display.print(F("C"));
    if (WiFi.status() == WL_CONNECTED) {
      drawwifiSymbol(113, 0);
    }
    display.display();
    return; // Funktion hier beenden
  }

  // =================================================================================
  // STANDARD-ANZEIGEN-LOGIK
  // =================================================================================
  display.clearDisplay();
  if (WiFi.status() == WL_CONNECTED) {
    drawwifiSymbol(113, 0);
  }
  display.setCursor(0, 0);
  display.setTextColor(SH110X_WHITE);
  display.setTextSize(1);

  // --- Anzeige: Fast-Heat-Up aktiv ---
  if (fastHeatUpHeating && !shotActive && !wasserSensorError && ((int)InputWasser < fastHeatUpSetpoint)) {
    SetpointWasser = fastHeatUpSetpoint;
    display.println(F("Fast-Heat-Up aktiv:"));
    display.println("");
    display.println(F("20 Sek. Flushen,"));
    display.println(F("wenn Temp. erreicht!"));
    display.println(F(" "));
    display.println(F("Temperatur: "));
    display.print((int)InputWasser);
    display.print(F(" / "));
    display.print((int)SetpointWasser);
    display.print(F(" "));
    display.print((char)247);
    display.print(F("C"));
  }
  else {
    //
    // --- ZUSTANDSMASCHINE FÜR ANZEIGEN NACH DEM SHOT ---
    //
    if (postShotDisplayState != POST_SHOT_IDLE) {
      switch (postShotDisplayState) {
        
        case POST_SHOT_SHOW_WEIGHT_RESULT: {
            // --- Werte berechnen ---
            float finalWeight = lastShotFinalNetWeight + brewByWeightOffsetGrams;
            float totalDurationSec = lastShotDurationMillis / 1000.0;
            float activeDurationSec = 0.0f;
            if (lastShotFirstWeightChangeTime > 0 && shotEndTime > lastShotFirstWeightChangeTime) {
                activeDurationSec = (shotEndTime - lastShotFirstWeightChangeTime) / 1000.0f;
                if (preInfusionEnabled) {
                    unsigned long pauseStart = lastShotStartTime + (unsigned long)(preInfusionDurationSeconds * 1000.0f);
                    if (lastShotFirstWeightChangeTime < pauseStart) {
                        activeDurationSec -= preInfusionPauseSeconds;
                    }
                }
            } else {
                activeDurationSec = totalDurationSec;
                if (preInfusionEnabled) {
                    activeDurationSec -= preInfusionPauseSeconds;
                }
            }
            if (activeDurationSec < 0) activeDurationSec = 0;
            float flowRate = 0.0;
            if (activeDurationSec > 0) {
                flowRate = finalWeight / activeDurationSec;
            }

            // --- Buffer für die formatierten Strings ---
            char weightBuffer[12];
            char totalTimeBuffer[12];
            char activeTimeBuffer[12];
            char flowBuffer[15];

            // --- Werte in Strings formatieren ---
            snprintf(weightBuffer, sizeof(weightBuffer), "%.1f g", finalWeight);
            snprintf(totalTimeBuffer, sizeof(totalTimeBuffer), "%.1f s", totalDurationSec);
            snprintf(activeTimeBuffer, sizeof(activeTimeBuffer), "%.1f s", activeDurationSec);
            snprintf(flowBuffer, sizeof(flowBuffer), "%.2f g/s", flowRate);

            // --- Strukturiertes Layout mit allen Details ---
            display.setTextSize(1); 
            
            // Titel anzeigen
            display.setCursor(0, 0);
            display.print("Bezug beendet");

            // Layout-Positionen definieren
            const int LABEL_X = 0;
            const int VALUE_X = 58;
            const int Y_LINE_1 = 14;
            const int Y_LINE_2 = 24;
            const int Y_LINE_3 = 34;
            const int Y_LINE_4 = 44;

            // Zeile: Gewicht
            display.setCursor(LABEL_X, Y_LINE_1);
            display.print("Gewicht:");
            display.setCursor(VALUE_X, Y_LINE_1);
            display.print(weightBuffer);

            // Zeile: Gesamtdauer
            display.setCursor(LABEL_X, Y_LINE_2);
            display.print("Gesamt:");
            display.setCursor(VALUE_X, Y_LINE_2);
            display.print(totalTimeBuffer);
            
            // Zeile: Aktive Dauer (nur wenn Pre-Infusion aktiv war)
            if (preInfusionEnabled) {
                display.setCursor(LABEL_X, Y_LINE_3);
                display.print("Aktiv:");
                display.setCursor(VALUE_X, Y_LINE_3);
                display.print(activeTimeBuffer);
            }

            // Zeile: Flow-Rate
            display.setCursor(LABEL_X, Y_LINE_4);
            display.print("Flow:");
            display.setCursor(VALUE_X, Y_LINE_4);
            display.print(flowBuffer);


            // --- Übergangslogik zum nächsten Zustand (bleibt unverändert) ---
            if (millis() > shotEndTime + 5000) { // Anzeigezeit auf 5s erhöht für mehr Infos
                if (maintenanceInterval > 0 && maintenanceIntervalCounter >= maintenanceInterval) {
                    postShotDisplayState = POST_SHOT_SHOW_MAINTENANCE;
                    maintenanceMessageStartTime = millis();
                    #ifdef ESP32
                    if (piezoEnabled) {
                        beepMaintenanceAlert();
                    }
                    #endif
                } else {
                    postShotDisplayState = POST_SHOT_IDLE;
                }
            }
            break;
        }

        case POST_SHOT_SHOW_DURATION: {
          // --- Zustand 2: Bezugsdauer anzeigen ---
          display.println(F("Bezugszeit:"));
          display.println("");
          display.setTextSize(3);
          char buffer[6];
          snprintf(buffer, sizeof(buffer), "%.1f", lastShotDurationMillis / 1000.0);
          display.println(buffer);
          display.setTextSize(1);
          display.println(F("Sekunden"));

          // Prüfen, ob 2 Sekunden um sind
          if (millis() > shotEndTime + 2000) {
            // Prüfen, ob eine Wartung fällig ist
            if (maintenanceInterval > 0 && maintenanceIntervalCounter >= maintenanceInterval) {
              // JA -> Gehe zu Zustand 2 (Wartung anzeigen)
              postShotDisplayState = POST_SHOT_SHOW_MAINTENANCE;
              maintenanceMessageStartTime = millis(); // Timer für Meldung JETZT starten
#ifdef ESP32
              if (piezoEnabled) {
                beepMaintenanceAlert(); // Piepton perfekt synchron zur Meldung auslösen!
              }
#endif
            } else {
              // NEIN -> Sequenz beendet
              postShotDisplayState = POST_SHOT_IDLE;
            }
          }
          break; // Wichtig!
        }

        case POST_SHOT_SHOW_MAINTENANCE: {
          // --- Zustand 3: Wartungsmeldung anzeigen ---
          display.println(F("Erinnerung:"));
          display.println(F(""));
          display.println(F("Reinigung /"));
          display.println(F("Wartung"));

          // Prüfen, ob 6 Sekunden um sind
          if (millis() > maintenanceMessageStartTime + 6000) {
            displayMaintenanceMessage = false; // Flag für alle Fälle zurücksetzen
            postShotDisplayState = POST_SHOT_IDLE; // Sequenz beendet
          }
          break; // Wichtig!
        }
      }
    }
    // --- ENDE DER ZUSTANDSMASCHINE ---

    // Wenn die Zustandsmaschine nicht aktiv ist, zeige den laufenden Shot oder die Temperaturen
    else if (shotActive) {
      // Priorität: Shot läuft *aktuell*?
#ifdef ESP32
  if (brewByWeightEnabled && scaleConnected) {
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println(F("Bezug aktiv:"));
    unsigned long elapsed = millis() - shotStartTime;
    char timeBuffer[8];
    snprintf(timeBuffer, sizeof(timeBuffer), "%.1fs", elapsed / 1000.0);
    display.setCursor(0, 15);
    display.println(timeBuffer);

    // --- START ÄNDERUNG ---
    float netWeight = currentWeightReading - brewStartWeight;
    // Verhindert die Anzeige von "-0.0" für das Nettogewicht
    if (netWeight < 0.0 && netWeight > -0.05) {
        netWeight = 0.0;
    }
    char weightBuffer[15];
    snprintf(weightBuffer, sizeof(weightBuffer), "%.1f/%.1fg", netWeight, brewByWeightTargetGrams);
    // --- ENDE ÄNDERUNG ---

    display.setCursor(0, 30);
    display.println(weightBuffer);
    display.println("");
    if(preInfusionEnabled && currentPreInfusionState == PI_PRE_BREW) {display.println(F("Pre-Infusion ..."));}
    if(preInfusionEnabled && currentPreInfusionState == PI_PAUSE) {display.println(F("Pause ..."));}
    if(preInfusionEnabled && currentPreInfusionState == PI_MAIN_BREW) {display.println(F("Pre-Infusion beendet."));}
  } else {
#endif
        unsigned long elapsed = millis() - shotStartTime;
        display.println(F("Shot-Timer:"));
        display.println("");
        display.setTextSize(3);
        char buffer[6];
        snprintf(buffer, sizeof(buffer), "%.1f", elapsed / 1000.0);
        display.println(buffer);
        display.setTextSize(1);
        display.println(F("Sekunden"));
#ifdef ESP32
        if(preInfusionEnabled && currentPreInfusionState == PI_PRE_BREW) {display.println(F("Pre-Infusion ..."));}
        if(preInfusionEnabled && currentPreInfusionState == PI_PAUSE) {display.println(F("Pause ..."));}
        if(preInfusionEnabled && currentPreInfusionState == PI_MAIN_BREW) {display.println(F("Pre-Infusion beendet."));}
      }
#endif
      fastHeatUpHeating = false;
    }
    else {
      // Standardanzeige: Temperaturen / Fehler / Sicherheitsabschaltung (Fallback, wenn nichts anderes zu tun ist)
      display.println(F("Temperaturen:"));
      display.println("");
      display.println(F("Wasser: "));
      if (wasserSafetyShutdown) { display.print(F("Sicherheitsabsch.")); }
      else if (wasserSensorError) { display.print(F("Sensorfehler!")); }
      else {
        display.print((int)InputWasser);
        display.print(F(" / "));
        display.print((int)SetpointWasser);
        display.print(F(" "));
        display.print((char)247);
        display.print(F("C"));
        if (ecoModeAktiv && dynamicEcoActive) display.print(F(" (ECO+)"));
        else if (ecoModeAktiv && !dynamicEcoActive) display.print(F(" (ECO)"));
      }
      display.println("");
      display.println("");
      display.println(F("Dampf:  "));
      if (dampfSafetyShutdown) { display.print(F("Sicherheitsabsch.")); }
      else if (dampfSensorError) { display.print(F("Sensorfehler!")); }
      else {
        bool steamDelayActiveSystem = (dampfVerzoegerung > 0 && (millis() - startupTime < (unsigned long)dampfVerzoegerung * 60000UL));
        bool shouldShowDelayMessage = steamDelayActiveSystem && !steamDelayOverridden;
        if (!shouldShowDelayMessage) {
          display.print((int)InputDampf);
          display.print(F(" / "));
          display.print((int)SetpointDampf);
          display.print(F(" "));
          display.print((char)247);
          display.print(F("C"));
          if (ecoModeAktiv && dynamicEcoActive) display.print(F(" (ECO+)"));
          else if (ecoModeAktiv && !dynamicEcoActive) display.print(F(" (ECO)"));
        } else {
          display.print(F("Startverz\x94gerung"));
        }
      }
    }
  }

  yield();
  display.display(); // Display *immer* am Ende aktualisieren
}
#endif // ENABLE_DISPLAY

/************************************************************************************
 * Verwaltet den Shot-Timer basierend auf dem physischen Schalter.
 * Berücksichtigt den Status 'manualSwitchReleasedAfterAutoStop', um ein
 * sofortiges Neustarten nach automatischem Stopp zu verhindern.
 * Initiiert Pre-Infusion (ESP32).
 * Ruft stopBrewSequence auf (ESP32) für manuelle Stopps.
 ************************************************************************************/
void updateShotTimer() {
    bool shotInputActive = (digitalRead(SHOT_TIMER_PIN) == HIGH);

    // --- Shot beginnt (Eingang ist HIGH) ---
    if (shotInputActive && !shotActive && manualSwitchReleasedAfterAutoStop) {

        // Prüfen, ob die Dampfverzögerung per Schalter übersprungen werden soll
        if (steamDelayOverrideBySwitchEnabled) {
            steamDelayOverridden = true;
            // Serial.println("INFO: Dampfverzögerung wird wegen Bezugsschalter-Aktivierung übersprungen.");
        }

        #ifdef ESP32 // Sicherstellen, dass dies nur den ESP32 betrifft
            scaleModeActive = false; // Manuellen Waage-Anzeigemodus beim Start des Bezugs beenden
        #endif
        shotActive = true;
        shotStartTime = millis(); // Startzeit für interne Logik (z.B. Dauer), aber kein Logging/Zählen im Wartungsmodus
        lastShotDurationMillis = 0;
        shotEndTime = 0;
        firstWeightChangeDetected = false;
        firstWeightChangeTime = 0;

        if (!wartungsModusAktiv) { // Nur ausführen, wenn NICHT im Wartungsmodus
            displayMaintenanceMessage = false; // Wartungsmeldung ausblenden

            // Eco Modus direkt beenden, falls aktiv
            if (ecoModeAktiv) {
                ecoModeAktiv = false;
                ecoModeActivatedTime = 0;
                // Normale Sollwerte wiederherstellen (wichtig!)
                EEPROM.get(EEPROM_ADDR_SETPOINT_WASSER, SetpointWasser);
                EEPROM.get(EEPROM_ADDR_SETPOINT_DAMPF, SetpointDampf);
                pidWasser.SetMode(AUTOMATIC); // Sicherstellen, dass PID läuft
                pidDampf.SetMode(AUTOMATIC);
            }
        }

#ifdef ESP32
        if (!wartungsModusAktiv) { 
          #if (SCALE_TYPE == SCALE_I2C)
            // Waage nur bei I2C-Typ automatisch auf Null setzen
            if (scaleConnected) {
                scales.setOffset();
                delay(50);
                currentWeightReading = scales.getWeight();
            }
          #else
            // "Software-Tara" für ESP-NOW-Waage: Speichere das aktuelle Gewicht als Startpunkt für die Differenzrechnung
            brewStartWeight = currentWeightReading;
          #endif

            // Pre-Infusion oder direkten Start behandeln
            if (preInfusionEnabled) {
                currentPreInfusionState = PI_PRE_BREW;
                preInfusionPhaseStartTime = millis();
                digitalWrite(PUMP_PIN, HIGH); // Pumpe AN für Pre-Infusion
                digitalWrite(VALVE_PIN, HIGH); // Ventil ZU für Pre-Infusion
            } else {
                currentPreInfusionState = PI_MAIN_BREW; // Direkt zur Hauptphase
                preInfusionPhaseStartTime = millis(); // Timer trotzdem starten (für Konsistenz)
                digitalWrite(PUMP_PIN, HIGH); // Pumpe AN für Hauptbezug
                digitalWrite(VALVE_PIN, HIGH); // Ventil ZU für Hauptbezug
            }
        } else { // Im Wartungsmodus: Pumpe und Ventil direkt schalten
            digitalWrite(PUMP_PIN, HIGH);
            digitalWrite(VALVE_PIN, HIGH); // Annahme: Ventil soll ZU sein, während Pumpe läuft (wie bei Bezug)
                                          // Ggf. anpassen, falls für Reinigung das Ventil offen sein soll, während Pumpe läuft.
            // Serial.println("Wartungsmodus: Pumpe AN, Ventil ZU (Bezugsschalter)");
        }
#endif // ESP32
    }
    // --- Shot endet (Eingang ist LOW - manuelles Stoppen) ---
    else if (!shotInputActive && shotActive) {
#ifdef ESP32
        unsigned long shotDuration = millis() - shotStartTime;
        if (!wartungsModusAktiv) { // Normale Stopp-Sequenz nur wenn NICHT im Wartungsmodus
            stopBrewSequence(shotDuration, true, false); // true für manual stop, false für "nicht durch Gewicht gestoppt"
        } else { // Im Wartungsmodus: Pumpe und Ventil direkt schalten, keine Statistik etc.
            digitalWrite(PUMP_PIN, LOW);
            digitalWrite(VALVE_PIN, LOW); // Ventil ÖFFNEN zum Druck ablassen/Spülen beenden
            // Serial.println("Wartungsmodus: Pumpe AUS, Ventil AUF (Bezugsschalter losgelassen)");
            // Für den Wartungsmodus werden keine Shot-Daten geloggt oder Zähler erhöht.
            // lastShotDurationMillis und shotEndTime könnten hier optional gesetzt werden,
            // wenn man die Dauer im Display anzeigen wollte, aber es wird nicht geloggt.
            lastShotDurationMillis = shotDuration;
            shotEndTime = millis();
        }
#else // ESP8266 - Logik (...)
    if (!wartungsModusAktiv) {
        unsigned long shotDuration = millis() - shotStartTime;
        lastShotDurationMillis = shotDuration;
        shotEndTime = millis();

        postShotDisplayState = POST_SHOT_SHOW_DURATION;

        if (shotDuration > 20000) {
            shotNeedsToBeSaved = true;
            savedShotDuration = shotDuration;
        }
        lastShotTime = millis();
        fastHeatUpHeating = false;
    }
#endif  // ESP32 / ESP8266

        shotActive = false; // Wichtig, um den Zustand zurückzusetzen (für beide Modi)

        // Logik für manualSwitchReleasedAfterAutoStop bleibt wichtig
        if (!manualSwitchReleasedAfterAutoStop) {
            manualSwitchReleasedAfterAutoStop = true;
        }
    }
    // --- Behandelt den Fall, dass der Schalter losgelassen wird, *nachdem* der Shot bereits automatisch gestoppt wurde ---
    else if (!shotInputActive && !shotActive && !manualSwitchReleasedAfterAutoStop) {
        manualSwitchReleasedAfterAutoStop = true;
        shotEndTime = millis();
    }
    // --- Optional: Wenn Schalter gehalten wird, aber System gesperrt ist (nach Auto-Stopp) ---
    else if (shotInputActive && !shotActive && !manualSwitchReleasedAfterAutoStop) {
        // Hier passiert nichts, der Timer startet nicht.
#ifdef ESP32
        // Sicherstellen, dass Pumpe/Ventil aus bleiben, wenn System gesperrt ist
        digitalWrite(PUMP_PIN, LOW);
        digitalWrite(VALVE_PIN, LOW);
#endif
    }
}

/************************************************************************************
 * Startet AutoTune für Wasser-PID (Angepasst für Zeitproportional)
 ************************************************************************************/

void startAutoTuneWasser() {
  // Eco-Modus deaktivieren, falls aktiv
  if (ecoModeAktiv) {
    ecoModeAktiv = false;
    // Ursprünglichen Setpoint wiederherstellen (aus EEPROM oder Profil)
    EEPROM.get(EEPROM_ADDR_SETPOINT_WASSER, SetpointWasser);
    // Optional: Auch Dampf-Setpoint wiederherstellen, falls Eco beide beeinflusst
    EEPROM.get(EEPROM_ADDR_SETPOINT_DAMPF, SetpointDampf);
  }

  // Stelle sicher, dass der PID im Automatikmodus ist bevor Tuning beginnt
  pidWasser.SetMode(AUTOMATIC);

  // Sicherstellen, dass die Output-Grenzen korrekt gesetzt sind
  pidWasser.SetOutputLimits(0, windowSizeWasser);

  autoTuneWasser = new PID_ATune(&InputWasser, &OutputWasser);
  // Verwende die Wasser-spezifischen Parameter
  autoTuneWasser->SetOutputStep(tuningStepWasser);            // Angepasst
  autoTuneWasser->SetControlType(1);                          // DIRECT control (heating)
  autoTuneWasser->SetNoiseBand(tuningNoiseWasser);            // Angepasst
  autoTuneWasser->SetLookbackSec((int)tuningLookBackWasser);  // Angepasst (Cast zu int für Funktion)

  // Setze den Startwert für den Output
  // WICHTIG: AutoTune manipuliert OutputWasser direkt. Die zeitproportionale
  // Logik in loop() wird diesen Wert verwenden, um den SSR zu steuern.
  OutputWasser = tuningStartValueWasser;  // Angepasst

  // KEIN direktes digitalWrite mehr hier! Die loop() übernimmt das.

  autoTuneWasserActive = true;
  // Serial.println("Starte AutoTune für Wasser (Zeitproportional)...");
}

/************************************************************************************
 * Startet AutoTune für Dampf-PID (Angepasst für Zeitproportional)
 ************************************************************************************/

void startAutoTuneDampf() {
  // Eco-Modus deaktivieren, falls aktiv
  if (ecoModeAktiv) {
    ecoModeAktiv = false;
    // Ursprünglichen Setpoint wiederherstellen (aus EEPROM oder Profil)
    EEPROM.get(EEPROM_ADDR_SETPOINT_DAMPF, SetpointDampf);
    // Optional: Auch Wasser-Setpoint wiederherstellen, falls Eco beide beeinflusst
    EEPROM.get(EEPROM_ADDR_SETPOINT_WASSER, SetpointWasser);
  }

  // Stelle sicher, dass der PID im Automatikmodus ist bevor Tuning beginnt
  pidDampf.SetMode(AUTOMATIC);

  // Sicherstellen, dass die Output-Grenzen korrekt gesetzt sind
  pidDampf.SetOutputLimits(0, windowSizeDampf);

  autoTuneDampf = new PID_ATune(&InputDampf, &OutputDampf);
  // Verwende die Dampf-spezifischen Parameter
  autoTuneDampf->SetOutputStep(tuningStepDampf);            // Angepasst
  autoTuneDampf->SetControlType(1);                         // DIRECT control (heating)
  autoTuneDampf->SetNoiseBand(tuningNoiseDampf);            // Angepasst
  autoTuneDampf->SetLookbackSec((int)tuningLookBackDampf);  // Angepasst (Cast zu int für Funktion)

  // Setze den Startwert für den Output
  // WICHTIG: AutoTune manipuliert OutputDampf direkt. Die zeitproportionale
  // Logik in loop() wird diesen Wert verwenden, um den SSR zu steuern.
  OutputDampf = tuningStartValueDampf;  // Angepasst

  // KEIN direktes digitalWrite mehr hier! Die loop() übernimmt das.

  autoTuneDampfActive = true;
  // Serial.println("Starte AutoTune für Dampf (Zeitproportional)...");
}

/************************************************************************************
 * Stoppt AutoTune für Wasser - Unverändert
 ************************************************************************************/

void stopAutoTuneWasser() {
  if (autoTuneWasserActive) {
    autoTuneWasserActive = false;
    delete autoTuneWasser;
    autoTuneWasser = nullptr;
    // Setze PID zurück in den Normalbetrieb (ggf. Output auf 0)
    pidWasser.SetMode(AUTOMATIC);  // Sicherstellen, dass PID wieder normal läuft
    OutputWasser = 0;              // Optional: Output zurücksetzen
    // Stelle sicher, dass SSR ausgeschaltet ist
    digitalWrite(SSR_WASSER_PIN, LOW);
    // Serial.println("AutoTune Wasser gestoppt.");
  }
}

/************************************************************************************
 * Stoppt AutoTune für Dampf - Unverändert
 ************************************************************************************/

void stopAutoTuneDampf() {
  if (autoTuneDampfActive) {
    autoTuneDampfActive = false;
    delete autoTuneDampf;
    autoTuneDampf = nullptr;
    // Setze PID zurück in den Normalbetrieb (ggf. Output auf 0)
    pidDampf.SetMode(AUTOMATIC);  // Sicherstellen, dass PID wieder normal läuft
    OutputDampf = 0;              // Optional: Output zurücksetzen
    // Stelle sicher, dass SSR ausgeschaltet ist
    digitalWrite(SSR_DAMPF_PIN, LOW);
    // Serial.println("AutoTune Dampf gestoppt.");
  }
}

/************************************************************************************
 * Überwacht die AutoTune-Prozesse (Dampf/Wasser)
 * - Prüft auf Sensorfehler ODER Sicherheitsabschaltung vor Ausführung
 * - Display-Logik ist in updateDisplay() ausgelagert
 ************************************************************************************/
void handleAutoTune(AsyncWebServerRequest *request) {
  (void)request;
  // --- AutoTune Wasser ---
  if (autoTuneWasserActive) {
    // Erweiterte Prüfung auf Sensorfehler ODER Sicherheitsabschaltung während des Tunings
    if (wasserSensorError || wasserSafetyShutdown) {
      // Logge den spezifischen Grund für den Abbruch
      // if (wasserSensorError) {
      //   Serial.println("FEHLER: Wassertemperatursensor während AutoTune ausgefallen! Breche Tuning ab.");
      // } else {  // Muss wasserSafetyShutdown sein
      //   Serial.println("WARNUNG: Sicherheitslimit Wasser während AutoTune überschritten! Breche Tuning ab.");
      // }
      stopAutoTuneWasser();  // Ruft Funktion auf, die Flag zurücksetzt, Speicher freigibt und SSR ausschaltet
      return;                // Verlasse die Funktion für diesen Durchlauf, um Runtime() nicht auszuführen
    }

    // Nur wenn kein Fehler/Shutdown vorliegt, führe AutoTune->Runtime() aus.
    // Diese Funktion berechnet und SETZT OutputWasser für den nächsten Schritt!
    if (autoTuneWasser->Runtime() == 1)  // Runtime() gibt 1 zurück, wenn Tuning abgeschlossen
    {
      // Tuning abgeschlossen, Werte holen
      KpWasser = autoTuneWasser->GetKp();
      KiWasser = autoTuneWasser->GetKi();
      KdWasser = autoTuneWasser->GetKd();

      // WICHTIG: PID-Tunings sofort anwenden
      pidWasser.SetTunings(KpWasser, KiWasser, KdWasser);

      // Neue Werte im EEPROM speichern
      EEPROM.put(EEPROM_ADDR_KP_WASSER, KpWasser);
      EEPROM.put(EEPROM_ADDR_KI_WASSER, KiWasser);
      EEPROM.put(EEPROM_ADDR_KD_WASSER, KdWasser);
      EEPROM.commit();

      // Serial.println("AutoTune Wasser erfolgreich abgeschlossen.");
      // Serial.printf("Neue Werte: Kp=%.2f, Ki=%.2f, Kd=%.2f\n", KpWasser, KiWasser, KdWasser);

      // AutoTune-Prozess korrekt beenden
      stopAutoTuneWasser();
    }
    // Hinweis: Die Display-Anzeige für laufendes Tuning wird in updateDisplay() gehandhabt.
  }

  // --- AutoTune Dampf ---
  if (autoTuneDampfActive) {
    // Erweiterte Prüfung auf Sensorfehler ODER Sicherheitsabschaltung während des Tunings
    if (dampfSensorError || dampfSafetyShutdown) {
      // Logge den spezifischen Grund für den Abbruch
      // if (dampfSensorError) {
      //   Serial.println("FEHLER: Dampftemperatursensor während AutoTune ausgefallen! Breche Tuning ab.");
      // } else {  // Muss dampfSafetyShutdown sein
      //   Serial.println("WARNUNG: Sicherheitslimit Dampf während AutoTune überschritten! Breche Tuning ab.");
      // }
      stopAutoTuneDampf();  // Ruft Funktion auf, die Flag zurücksetzt, Speicher freigibt und SSR ausschaltet
      return;               // Verlasse die Funktion für diesen Durchlauf, um Runtime() nicht auszuführen
    }

    // Nur wenn kein Fehler/Shutdown vorliegt, führe AutoTune->Runtime() aus.
    // Diese Funktion berechnet und SETZT OutputDampf für den nächsten Schritt!
    if (autoTuneDampf->Runtime() == 1)  // Runtime() gibt 1 zurück, wenn Tuning abgeschlossen
    {
      // Tuning abgeschlossen, Werte holen
      KpDampf = autoTuneDampf->GetKp();
      KiDampf = autoTuneDampf->GetKi();
      KdDampf = autoTuneDampf->GetKd();

      // WICHTIG: PID-Tunings sofort anwenden
      pidDampf.SetTunings(KpDampf, KiDampf, KdDampf);

      // Neue Werte im EEPROM speichern
      EEPROM.put(EEPROM_ADDR_KP_DAMPF, KpDampf);
      EEPROM.put(EEPROM_ADDR_KI_DAMPF, KiDampf);
      EEPROM.put(EEPROM_ADDR_KD_DAMPF, KdDampf);
      EEPROM.commit();

      // Serial.println("AutoTune Dampf erfolgreich abgeschlossen.");
      // Serial.printf("Neue Werte: Kp=%.2f, Ki=%.2f, Kd=%.2f\n", KpDampf, KiDampf, KdDampf);

      // AutoTune-Prozess korrekt beenden
      stopAutoTuneDampf();
    }
    // Hinweis: Die Display-Anzeige für laufendes Tuning wird in updateDisplay() gehandhabt.
  }
}


/************************************************************************************
 * Handler für die Brew Control Seite (GET /Brew-Control) - Nur ESP32
 ************************************************************************************/
#ifdef ESP32 // Handler sind nur für ESP32 sinnvoll

// --- Angepasste PROGMEM Chunks für die Brew Control Seite mit Toggle Switches ---

// Head und Body Start bleiben gleich
static const char brewControlHead[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><title>Brew Control</title><meta name='theme-color' content='#000000'><meta name='viewport' content='width=device-width, initial-scale=1.0'>
)rawliteral";
static const char brewControlHeadEnd[] PROGMEM = R"rawliteral(</head>)rawliteral";
static const char brewControlBodyStart[] PROGMEM = R"rawliteral(
<body><h1>Brew Control</h1>
<form action='/saveBrewControl' method='POST'>
Steuert das automatische Beenden des Br&uuml;hvorgangs basierend auf Zeit oder Gewicht.
Wenn beide aktiviert sind, stoppt der Bezug, sobald die erste Bedingung (Zeit oder Gewicht) erreicht ist.
Der Bezug kann aber weiterhin manuell &uuml;ber den Bezugs-Schalter beendet werden.<br>
Ebenso kann eine Pre-Infusion aktiviert und eingestellt werden.<br><br>
)rawliteral";

// --- Brew-By-Time (BBT) Chunks ---
static const char brewControlBBT_H3[] PROGMEM = R"rawliteral(<h3>Brew-By-Time</h3>)rawliteral";
static const char brewControlBBT_ToggleContainerStart[] PROGMEM = R"rawliteral(<div class="toggle-switch-container"><span class="toggle-switch-label-text">Aktivieren:</span><label class="toggle-switch"><input type="checkbox" id="bbtEnabled" name="bbtEnabled" value="1")rawliteral"; // Endet VOR checked Attribut
static const char brewControlBBT_ToggleContainerEnd[] PROGMEM = R"rawliteral(><span class="toggle-slider"></span></label></div>)rawliteral"; // Schließt Input, Span, Label, Div
static const char brewControlBBT_Fields[] PROGMEM = R"rawliteral(
    <label for='bbtSecs'>Zieldauer (Sekunden):</label>
    <input type='number' min='5.0' max='120.0' step='0.1' id='bbtSecs' name='bbtSecs' value='{BBT_SECS}' required>
    <small class="toggle-description-input">(Stoppt nach dieser Zeit)</small><br>
)rawliteral"; // Beschreibung hat jetzt toggle-description Klasse

// --- Brew-By-Weight (BBW) Chunks ---
static const char brewControlBBW_H3[] PROGMEM = R"rawliteral(<h3>Brew-By-Weight</h3>)rawliteral";
static const char brewControlBBW_ToggleContainerStart[] PROGMEM = R"rawliteral(<div class="toggle-switch-container"><span class="toggle-switch-label-text">Aktivieren:</span><label class="toggle-switch"><input type="checkbox" id="bbwEnabled" name="bbwEnabled" value="1")rawliteral"; // Endet VOR checked/disabled Attribut(en)
static const char brewControlBBW_ToggleContainerEnd[] PROGMEM = R"rawliteral(><span class="toggle-slider"></span></label></div>)rawliteral"; // Schließt Input, Span, Label, Div
static const char brewControlBBW_Fields[] PROGMEM = R"rawliteral(
    <label for='bbwTarget'>Zielgewicht (Gramm):</label>
    <input type='number' min='10.0' max='70.0' step='0.1' id='bbwTarget' name='bbwTarget' value='{BBW_TARGET}' required {SCALE_DISABLED}>
    <label for='bbwOffset'>Offset (Gramm):</label>
    <input type='number' min='0.0' max='10.0' step='0.1' id='bbwOffset' name='bbwOffset' value='{BBW_OFFSET}' required {SCALE_DISABLED}>
    <small class="toggle-description-input">(Stoppt x Gramm fr&uuml;her)</small>
    {SCALE_STATUS_MSG}
    <br>
)rawliteral";

// --- Pre-Infusion (PI) Chunks ---
static const char brewControlPI_H3[] PROGMEM = R"rawliteral(<h3>Pre-Infusion</h3>)rawliteral";
static const char brewControlPI_ToggleContainerStart[] PROGMEM = R"rawliteral(<div class="toggle-switch-container"><span class="toggle-switch-label-text">Aktivieren:</span><label class="toggle-switch"><input type="checkbox" id="piEnabled" name="piEnabled" value="1")rawliteral"; // Endet VOR checked Attribut
static const char brewControlPI_ToggleContainerEnd[] PROGMEM = R"rawliteral(><span class="toggle-slider"></span></label></div>)rawliteral"; // Schließt Input, Span, Label, Div
static const char brewControlPI_Fields[] PROGMEM = R"rawliteral(
    <label for='piDurSecs'>Pre-Infusion Dauer (Sekunden):</label>
    <input type='number' min='0.5' max='20.0' step='0.1' id='piDurSecs' name='piDurSecs' value='{PI_DUR_SECS}' required>
    <label for='piPauseSecs'>Pause nach Pre-Infusion (Sekunden):</label>
    <input type='number' min='0.0' max='20.0' step='0.1' id='piPauseSecs' name='piPauseSecs' value='{PI_PAUSE_SECS}' required>
    <small class="toggle-description-input">(Pumpe AN -> Pumpe AUS -> Pumpe AN)</small><br>
)rawliteral";

// Formular Ende bleibt gleich
static const char brewControlFormEnd[] PROGMEM = R"rawliteral(
    <input type='submit' value='Einstellungen speichern'>
</form>
</body></html>
)rawliteral";

// --- Ende der PROGMEM Chunks ---

// Handler für die Brew Control Seite (GET /Brew-Control) - Mit Toggle Switches
void handleBrewControl(AsyncWebServerRequest *request) {
    char buffer[10]; // Puffer für Zahlen
    AsyncResponseStream *response = request->beginResponseStream("text/html; charset=utf-8");

    response->print(FPSTR(brewControlHead));
    response->print(FPSTR(commonStyle));     // Dein globaler Style mit den Toggle-CSS-Regeln
    response->print(FPSTR(brewControlHeadEnd));
    response->print(FPSTR(commonNav));
    response->print(FPSTR(brewControlBodyStart));
    yield();

    // --- Brew-By-Time Sektion ---
    response->print(FPSTR(brewControlBBT_H3));
    response->print(FPSTR(brewControlBBT_ToggleContainerStart)); // Beginnt Toggle bis vor 'checked'
    if (brewByTimeEnabled) {
        response->print(F(" checked")); // Fügt 'checked' hinzu, wenn aktiv
    }
    response->print(FPSTR(brewControlBBT_ToggleContainerEnd)); // Schließt den Toggle HTML-Teil

    // Felder für BBT rendern
    String bbtFieldsHtml = FPSTR(brewControlBBT_Fields);
    snprintf(buffer, sizeof(buffer), "%.1f", brewByTimeTargetSeconds);
    bbtFieldsHtml.replace("{BBT_SECS}", buffer);
    response->print(bbtFieldsHtml);
    yield();

    // --- Brew-By-Weight Sektion ---
    response->print(FPSTR(brewControlBBW_H3));
    response->print(FPSTR(brewControlBBW_ToggleContainerStart)); // Beginnt Toggle bis vor 'checked'/'disabled'

    if (brewByWeightEnabled) {
        response->print(F(" checked")); // Fügt 'checked' hinzu, wenn aktiv
    }
    if (!scaleConnected) {
         response->print(F(" disabled")); // Fügt 'disabled' hinzu, wenn Waage nicht verbunden
    }
    response->print(FPSTR(brewControlBBW_ToggleContainerEnd)); // Schließt den Toggle HTML-Teil

    // Felder für BBW rendern
    String bbwFieldsHtml = FPSTR(brewControlBBW_Fields);
    snprintf(buffer, sizeof(buffer), "%.1f", brewByWeightTargetGrams);
    bbwFieldsHtml.replace("{BBW_TARGET}", buffer);
    snprintf(buffer, sizeof(buffer), "%.1f", brewByWeightOffsetGrams);
    bbwFieldsHtml.replace("{BBW_OFFSET}", buffer);

    // Deaktiviere Felder und zeige Statusmeldung, wenn Waage nicht verbunden ist
    if (!scaleConnected) {
        bbwFieldsHtml.replace("{SCALE_DISABLED}", "disabled");
        bbwFieldsHtml.replace("{SCALE_STATUS_MSG}", "<b><small style='color: #FFCC00;'>Waage nicht verbunden!</small><br></b>");
    } else {
        bbwFieldsHtml.replace("{SCALE_DISABLED}", "");
        bbwFieldsHtml.replace("{SCALE_STATUS_MSG}", ""); // Keine Meldung wenn ok
    }
    response->print(bbwFieldsHtml);
    yield();

    // --- Pre-Infusion Sektion ---
    response->print(FPSTR(brewControlPI_H3));
    response->print(FPSTR(brewControlPI_ToggleContainerStart)); // Beginnt Toggle bis vor 'checked'
    if (preInfusionEnabled) {
        response->print(F(" checked")); // Fügt 'checked' hinzu, wenn aktiv
    }
    response->print(FPSTR(brewControlPI_ToggleContainerEnd)); // Schließt den Toggle HTML-Teil

    // Felder für PI rendern
    String piFieldsHtml = FPSTR(brewControlPI_Fields);
    snprintf(buffer, sizeof(buffer), "%.1f", preInfusionDurationSeconds);
    piFieldsHtml.replace("{PI_DUR_SECS}", buffer);
    snprintf(buffer, sizeof(buffer), "%.1f", preInfusionPauseSeconds);
    piFieldsHtml.replace("{PI_PAUSE_SECS}", buffer);
    response->print(piFieldsHtml);
    yield();

    // Formular Ende
    response->print(FPSTR(brewControlFormEnd));

    request->send(response);
}

/************************************************************************************
 * Handler zum Speichern der Brew Control Einstellungen (POST /saveBrewControl) - Nur ESP32
 ************************************************************************************/

void handleSaveBrewControl(AsyncWebServerRequest *request) {
    bool changed = false;

    // Brew-By-Time Auswerten (Unverändert, verwendet float)
    bool newBBTEnabled = request->hasArg("bbtEnabled");
    if (newBBTEnabled != brewByTimeEnabled) { brewByTimeEnabled = newBBTEnabled; EEPROM.put(EEPROM_ADDR_BREWBYTIME_ENABLED, brewByTimeEnabled); changed = true; }
    if (request->hasArg("bbtSecs")) {
        float newBBTSecs = request->arg("bbtSecs").toFloat();
        if (newBBTSecs >= 5.0f && newBBTSecs <= 120.0f) { if (abs(newBBTSecs - brewByTimeTargetSeconds) > 0.01f) { brewByTimeTargetSeconds = newBBTSecs; EEPROM.put(EEPROM_ADDR_BREWBYTIME_SECONDS, brewByTimeTargetSeconds); changed = true; }
        }
    }

    // *** Brew-By-Weight Auswerten ***
    bool newBBWEnabled = request->hasArg("bbwEnabled");
    if (newBBWEnabled != brewByWeightEnabled) { brewByWeightEnabled = newBBWEnabled; EEPROM.put(EEPROM_ADDR_BREWBYWEIGHT_ENABLED, brewByWeightEnabled); changed = true; }
    if (request->hasArg("bbwTarget")) {
        float newBBWTarget = request->arg("bbwTarget").toFloat();
        if (newBBWTarget >= 10.0f && newBBWTarget <= 70.0f) {
            if (abs(newBBWTarget - brewByWeightTargetGrams) > 0.01f) { brewByWeightTargetGrams = newBBWTarget; EEPROM.put(EEPROM_ADDR_BREWBYWEIGHT_TARGET, brewByWeightTargetGrams); changed = true; }
        }
    }
    if (request->hasArg("bbwOffset")) {
        float newBBWOffset = request->arg("bbwOffset").toFloat();
        if (newBBWOffset >= 0.0f && newBBWOffset <= 10.0f) {
            if (abs(newBBWOffset - brewByWeightOffsetGrams) > 0.01f) { brewByWeightOffsetGrams = newBBWOffset; EEPROM.put(EEPROM_ADDR_BREWBYWEIGHT_OFFSET, brewByWeightOffsetGrams); changed = true; }
        }
    }

    // Pre-Infusion Auswerten (Unverändert, verwendet float)
    bool newPIEnabled = request->hasArg("piEnabled");
    if (newPIEnabled != preInfusionEnabled) { preInfusionEnabled = newPIEnabled; EEPROM.put(EEPROM_ADDR_PREINF_ENABLED, preInfusionEnabled); changed = true; }
    if (request->hasArg("piDurSecs")) {
        float newPIDur = request->arg("piDurSecs").toFloat();
        if (newPIDur >= 0.5f && newPIDur <= 20.0f) { if(abs(newPIDur - preInfusionDurationSeconds) > 0.01f) { preInfusionDurationSeconds = newPIDur; EEPROM.put(EEPROM_ADDR_PREINF_DUR_SEC, preInfusionDurationSeconds); changed = true; } }
    }
    if (request->hasArg("piPauseSecs")) {
        float newPIPause = request->arg("piPauseSecs").toFloat();
        if (newPIPause >= 0.0f && newPIPause <= 20.0f) { if(abs(newPIPause - preInfusionPauseSeconds) > 0.01f) { preInfusionPauseSeconds = newPIPause; EEPROM.put(EEPROM_ADDR_PREINF_PAUSE_SEC, preInfusionPauseSeconds); changed = true; } }
    }

    if (changed) {
      EEPROM.commit();
    }

    AsyncWebServerResponse *response = request->beginResponse(303);
    response->addHeader("Location", "/Brew-Control");
    request->send(response);
}

#endif // ESP32 - Ende der Handler für Brew Control

/************************************************************************************
 * PROGMEM Chunks für die Info-Seite (/Info)
 ************************************************************************************/

// --- Kopfzeile ---
static const char infoHtmlHead[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>Info</title><meta name='theme-color' content='#000000'><meta name='viewport' content='width=device-width, initial-scale=1.0'><meta name='theme-color' content='#000000'><meta name='viewport' content='width=device-width, initial-scale=1.0'>
)rawliteral";
static const char infoHtmlHeadEnd[] PROGMEM = R"rawliteral(
</head>
)rawliteral";

// --- Geräteinfo Formular ---
static const char infoChunk_BodyStart[] PROGMEM = R"rawliteral(
<body>
  <h1>Info</h1>
  <form action='/updateInfoSettings' method='POST'>
    <h3>Ger&auml;teinfo</h3>
    Die Ger&auml;teinformationen werden beim Start der Maschine, bzw. PID-Controllers im Display angezeigt.
    <br><br>
    <label for='hersteller'>Hersteller:</label>
    <input type='text' id='hersteller' name='hersteller' value=')rawliteral";  // Endet vor Hersteller-Wert
static const char infoChunk_AfterHersteller[] PROGMEM = R"rawliteral('>
    <label for='modell'>Modell:</label>
    <input type='text' id='modell' name='modell' value=')rawliteral";          // Endet vor Modell-Wert
static const char infoChunk_AfterModell[] PROGMEM = R"rawliteral('>
    <label for='zusatz'>Zusatz (z.B. Limited Edition):</label>
    <input type='text' id='zusatz' name='zusatz' value=')rawliteral";          // Endet vor Zusatz-Wert
static const char infoChunk_AfterZusatz[] PROGMEM = R"rawliteral('>
    <input type='submit' value='Einstellungen speichern'>
  </form>
)rawliteral";                                                                  // Ende Geräteinfo-Formular

// --- Betriebszeit Reset Formular ---
static const char infoChunk_RuntimeResetForm[] PROGMEM = R"rawliteral(
  <form action='/resetRuntime' method='POST'>
    <h3>Betriebszeit:</h3>
    <b>Betriebszeit gesamt:</b><br>
)rawliteral";  // Endet vor formatierter Betriebszeit
static const char infoChunk_AfterRuntime[] PROGMEM = R"rawliteral(<br><br>
    <b>Betriebszeit seit Einschalten:</b><br>)rawliteral";
static const char infoChunk_AfterCurrentRuntime[] PROGMEM = R"rawliteral(<br>
    <input type='submit' value='Zur&uuml;cksetzen'>
  </form>
)rawliteral";  // Ende Betriebszeit-Reset-Formular

// --- Shot Zähler Reset Formular ---
static const char infoChunk_ShotCounterResetForm[] PROGMEM = R"rawliteral(
  <form action='/resetShots' method='POST'>
    <h3>Shots: (Bez&uuml;ge &uuml;ber 20 Sekunden)</h3>
    Anzahl: )rawliteral";  // Endet vor Shot-Zähler-Wert
static const char infoChunk_ShotCounterFormEnd[] PROGMEM = R"rawliteral(<br>
    <input type='submit' value='Zur&uuml;cksetzen'>
  </form>
)rawliteral";              // Ende Shot-Zähler-Reset-Formular

// --- Statistik Chunks ---
static const char infoChunk_StatsStart[] PROGMEM = R"rawliteral(
  <form method='GET' action='/downloadNutzungsstatistik' target='_blank'enctype='multipart/form-data'>
    <h3>Statistik</h3>
    <p>
)rawliteral";  // Start Statistik-Sektion
static const char infoChunk_StatsTotal[] PROGMEM = R"rawliteral(Bez&uuml;ge in der Statistik: )rawliteral";
static const char infoChunk_StatsAvg[] PROGMEM = R"rawliteral(<br>Durchschnittliche Bezugsdauer: )rawliteral";
static const char infoChunk_StatsAvgPerDay[] PROGMEM = R"rawliteral(<br>Durchschnittliche Bez&uuml;ge pro Tag: )rawliteral";

static const char infoChunk_StatsToday[] PROGMEM = R"rawliteral(<br><br><b>Anzahl Bez&uuml;ge:</b></br>Heute: )rawliteral";
static const char infoChunk_StatsYesterday[] PROGMEM = R"rawliteral(<br>Gestern: )rawliteral";
static const char infoChunk_StatsWeek[] PROGMEM = R"rawliteral(<br>Diese Woche: )rawliteral";
static const char infoChunk_StatsLastWeek[] PROGMEM = R"rawliteral(<br>Letzte Woche: )rawliteral";
static const char infoChunk_StatsMonth[] PROGMEM = R"rawliteral(<br>Dieses Monat: )rawliteral";
static const char infoChunk_StatsLastMonth[] PROGMEM = R"rawliteral(<br>Letzes Monat: )rawliteral";
static const char infoChunk_StatsFirst[] PROGMEM = R"rawliteral(<br><br>Erster geloggter Bezug: )rawliteral";

static const char infoChunk_StatsLast[] PROGMEM = R"rawliteral(<br>Letzter geloggter Bezug: )rawliteral";
static const char infoChunk_StatsTimeError[] PROGMEM = R"rawliteral(<br><small>(Datum/Zeit-basierte Z&auml;hler ben&ouml;tigen aktive WLAN-Verbindung und NTP-Synchronisation)</small>)rawliteral";
// --- Statistik Download
static const char infoChunk_StatsDownloadButton[] PROGMEM = R"rawliteral(
    <br><button type='submit' >Nutzungsverlauf herunterladen</button>
)rawliteral";
// --- Statistik entfernen
static const char infoChunk_DeleteStatsForm[] PROGMEM = R"rawliteral(
  <form action='/deleteStatistics' method='POST' onsubmit='return confirm("Sicher, dass die aktuelle Nutzungsstatistik (Nutzungsstatistik.csv) entfernt werden soll?");'>
    <h3>Nutzungsstatistik zur&uuml;cksetzen</h3>
    Die aktuelle Statistik wird gel&ouml;scht und es kann eine neue Statistik begonnen werden.</br>
    <button type='submit' class='danger'>Zur&uuml;cksetzen</button>
  </form>
)rawliteral";
static const char infoChunk_StatsEnd[] PROGMEM = R"rawliteral(
  </form>
)rawliteral";  // Ende Statistik-Sektion (mit Verlauf)
static const char infoChunk_StatsNotAvailable[] PROGMEM = R"rawliteral(
  <form>
    <h3>Shot-Verlauf Statistik</h3>
    <p>Kein Shot-Verlauf gefunden oder Datei ist leer.</p>
    <p><small>(Bez&uuml;ge werden erst nach erfolgreicher NTP-Zeitsynchronisation geloggt.<br>Hierzu wird eine Internetverbindung ben&ouml;tigt!)</small></p>
  </form>
)rawliteral";  // Meldung, falls kein Verlauf da

// --- Systeminfo Chunks (Angepasst für NTP-Status) ---
static const char infoChunk_SysInfoStart[] PROGMEM = R"rawliteral(
  <form>
    <h3>Systeminfo</h3>
    Freier Heap: )rawliteral";  // Endet VOR dem Heap-Wert
// Nach dem Heap-Wert und vor dem NTP-Statuswert
static const char infoChunk_AfterFreeHeap[] PROGMEM = R"rawliteral( Bytes<br>
    NTP Zeit synchronisiert: )rawliteral";  // Endet VOR dem NTP-Status (Ja/Nein)
// Nach dem NTP-Statuswert und vor der SDK-Version
static const char infoChunk_AfterNTPStatus[] PROGMEM = R"rawliteral(<br>
    SDK-Version: )rawliteral";  // Endet VOR dem SDK-Wert

// Bestehende Chunks für restliche Systeminfos
static const char infoChunk_AfterSDK[] PROGMEM = R"rawliteral(<br>
    Core-Version: )rawliteral";
static const char infoChunk_AfterCore[] PROGMEM = R"rawliteral(<br>
    Boot-Version: )rawliteral";
static const char infoChunk_AfterBoot[] PROGMEM = R"rawliteral(<br>
    Chip-ID: )rawliteral";
static const char infoChunk_MacAddress[] PROGMEM = R"rawliteral(<br>
    MAC-Adresse (WLAN): )rawliteral";
static const char infoChunk_AfterChipID[] PROGMEM = R"rawliteral(<br>
    CPU-Takt: )rawliteral";
static const char infoChunk_AfterCPU[] PROGMEM = R"rawliteral( MHz<br>
    Letzter Reset-Grund: )rawliteral";
static const char infoChunk_AfterResetReason[] PROGMEM = R"rawliteral(<br>
    <br>
    Sketch-Gr&ouml;&szlig;e: )rawliteral";
static const char infoChunk_AfterSketchSize[] PROGMEM = R"rawliteral( Bytes<br>
    Nutzbarer Sketch-Speicher: )rawliteral";
static const char infoChunk_AfterFreeSketch[] PROGMEM = R"rawliteral( Bytes<br>
    Flash-Gr&ouml;&szlig;e (Chip): )rawliteral";
static const char infoChunk_End[] PROGMEM = R"rawliteral( Bytes<br>
    </form>
 </body>
 </html>
)rawliteral";  // Ende Systeminfo und HTML-Seite

/************************************************************************************
 * Handler für die Info-Seite - Inkl. NTP-Status, Download-Button,
 * neuer Statistiken, Lösch-Button und plattformabhängiger Systeminfos
 ************************************************************************************/

void handleInfo(AsyncWebServerRequest *request)  // URL: /Info
{ 
  AsyncResponseStream *response = request->beginResponseStream(F("text/html"));
  response->setCode(200);
  char buffer[40];                // Allzweck-Puffer für Zahlen etc.
  char dateBuffer[25];            // Puffer für Datumsformatierung
  char tempStringCopyBuffer[50];  // Temp Puffer für String-Kopien


  // --- Kopfzeile, Styles, Navigation senden ---
  response->print(FPSTR(infoHtmlHead));
  response->print(FPSTR(commonStyle));  // Globale Styles nicht vergessen
  response->print(FPSTR(infoHtmlHeadEnd));
  response->print(FPSTR(commonNav));  // Globale Navigation nicht vergessen
  yield();

  // --- Geräteinfo Formular ---
  response->print(FPSTR(infoChunk_BodyStart));  // Start Body, H1, Start Geräteinfo-Form
  // Sicherstellen, dass Strings nicht über Puffer laufen und terminiert sind
  strncpy(tempStringCopyBuffer, infoHersteller, sizeof(tempStringCopyBuffer) - 1);
  tempStringCopyBuffer[sizeof(tempStringCopyBuffer) - 1] = '\0';
  if (strlen(tempStringCopyBuffer) > 0) { response->print(tempStringCopyBuffer); }  // Wert einfügen
  response->print(FPSTR(infoChunk_AfterHersteller));                                     // Label+Input für Modell
  yield();

  strncpy(tempStringCopyBuffer, infoModell, sizeof(tempStringCopyBuffer) - 1);
  tempStringCopyBuffer[sizeof(tempStringCopyBuffer) - 1] = '\0';
  if (strlen(tempStringCopyBuffer) > 0) { response->print(tempStringCopyBuffer); }  // Wert einfügen
  response->print(FPSTR(infoChunk_AfterModell));                                         // Label+Input für Zusatz
  yield();

  strncpy(tempStringCopyBuffer, infoZusatz, sizeof(tempStringCopyBuffer) - 1);
  tempStringCopyBuffer[sizeof(tempStringCopyBuffer) - 1] = '\0';
  if (strlen(tempStringCopyBuffer) > 0) { response->print(tempStringCopyBuffer); }  // Wert einfügen
  response->print(FPSTR(infoChunk_AfterZusatz));                                         // Ende Geräteinfo-Form
  yield();

  // --- Betriebszeit Reset Formular ---
  response->print(FPSTR(infoChunk_RuntimeResetForm));  // Start Betriebszeit-Reset-Form, endet vor formatierter Zeit
  // Formatierte Betriebszeit berechnen und senden
  unsigned long totalSeconds = totalRuntime;
  unsigned long days = totalSeconds / 86400;
  unsigned long hours = (totalSeconds % 86400) / 3600;
  unsigned long minutes = (totalSeconds % 3600) / 60;
  snprintf(buffer, sizeof(buffer), "%lu", days);
  response->print(buffer);
    response->print(F(" Tag(e), "));
  snprintf(buffer, sizeof(buffer), "%lu", hours);
  response->print(buffer);
  response->print(F(" Stunde(n) und "));
  snprintf(buffer, sizeof(buffer), "%lu", minutes);
  response->print(buffer);
  response->print(F(" Minuten"));
  response->print(FPSTR(infoChunk_AfterRuntime));
  unsigned long currentSeconds = millis() / 1000UL;
  unsigned long currentDays = currentSeconds / 86400UL;
  unsigned long currentHours = (currentSeconds % 86400UL) / 3600UL;
  unsigned long currentMinutes = (currentSeconds % 3600UL) / 60UL;
  snprintf(buffer, sizeof(buffer), "%lu", currentDays);
  response->print(buffer);
    response->print(F(" Tag(e), "));
  snprintf(buffer, sizeof(buffer), "%lu", currentHours);
  response->print(buffer);
  response->print(F(" Stunde(n) und "));
  snprintf(buffer, sizeof(buffer), "%lu", currentMinutes);
  response->print(buffer);
  response->print(F(" Minuten"));
  response->print(FPSTR(infoChunk_AfterCurrentRuntime));  // Ende Betriebszeit-Reset-Form
  yield();

  // --- Shot Zähler Reset Formular ---
  response->print(FPSTR(infoChunk_ShotCounterResetForm));  // Start Shot-Reset-Form, endet vor Zähler-Wert
  snprintf(buffer, sizeof(buffer), "%lu", shotCounter);  // Wert einfügen
  response->print(buffer);
  response->print(FPSTR(infoChunk_ShotCounterFormEnd));  // Ende Shot-Reset-Form
  yield();

  // --- Shot-Verlauf Statistik ---
  ShotStats stats = calculateShotStatistics();  // Berechne Statistiken
  yield();                                      // Nach der (potenziell) längeren Berechnung

  if (stats.historyAvailable) {
    response->print(FPSTR(infoChunk_StatsStart));  // Start des Statistik-Blocks (<form><h3><p>)

    // Bestehende allgemeine Statistiken
    response->print(FPSTR(infoChunk_StatsTotal));  // "Geloggte Bezüge gesamt: "
    snprintf(buffer, sizeof(buffer), "%lu", stats.totalShotsLogged);
    response->print(buffer);
    yield();

    response->print(FPSTR(infoChunk_StatsAvg));  // "<br>Durchschnittliche Bezugsdauer: "
    snprintf(buffer, sizeof(buffer), "%.1f", stats.averageDurationSec);
    response->print(buffer);
    response->print(F(" s"));  // Einheit hinzufügen
    yield();

    // Durchschnitt pro Tag
    response->print(FPSTR(infoChunk_StatsAvgPerDay));  // "<br>Durchschnittliche Bez&uuml;ge pro Tag (seit Log): "
    snprintf(buffer, sizeof(buffer), "%.1f", stats.avgShotsPerDay);
    response->print(buffer);
    yield();

    // Bestehende Zeitfenster
    response->print(FPSTR(infoChunk_StatsToday));  // " s<br><br>Bez&uuml;ge Heute: "
    snprintf(buffer, sizeof(buffer), "%lu", stats.shotsToday);
    response->print(buffer);
    yield();

    response->print(FPSTR(infoChunk_StatsYesterday));  // "<br>Bez&uuml;ge Gestern: "
    snprintf(buffer, sizeof(buffer), "%lu", stats.shotsYesterday);
    response->print(buffer);
    yield();

    response->print(FPSTR(infoChunk_StatsWeek));  // "<br>Bez&uuml;ge in dieser Woche: "
    snprintf(buffer, sizeof(buffer), "%lu", stats.shotsThisWeek);
    response->print(buffer);
    yield();

    response->print(FPSTR(infoChunk_StatsLastWeek));  // "<br>Bez&uuml;ge Letzte Woche (Mo-So): "
    snprintf(buffer, sizeof(buffer), "%lu", stats.shotsLastWeek);
    response->print(buffer);
    yield();

    response->print(FPSTR(infoChunk_StatsMonth));  // "<br>Bez&uuml;ge in diesem Monat: "
    snprintf(buffer, sizeof(buffer), "%lu", stats.shotsThisMonth);
    response->print(buffer);
    yield();

    response->print(FPSTR(infoChunk_StatsLastMonth));  // "<br>Bez&uuml;ge Letzter Monat: "
    snprintf(buffer, sizeof(buffer), "%lu", stats.shotsLastMonth);
    response->print(buffer);
    yield();  // Nach mehreren Ausgaben


    // Ersten und letzten Shot anzeigen, falls vorhanden (wie bisher)
    if (stats.firstShotTimestamp > 0) {
      response->print(FPSTR(infoChunk_StatsFirst));  // "<br><br>Erster geloggter Bezug: "
      struct tm first_tm;
      localtime_r(&stats.firstShotTimestamp, &first_tm);
      strftime(dateBuffer, sizeof(dateBuffer), "%d.%m.%Y %H:%M", &first_tm);  // Datum formatieren
      response->print(dateBuffer);                                         // Wert senden
      response->print(F(" Uhr"));
      yield();
    }
    if (stats.lastShotTimestamp > 0) {
      response->print(FPSTR(infoChunk_StatsLast));  // "<br>Letzter geloggter Bezug: "
      struct tm last_tm;
      localtime_r(&stats.lastShotTimestamp, &last_tm);
      strftime(dateBuffer, sizeof(dateBuffer), "%d.%m.%Y %H:%M", &last_tm);  // Datum formatieren
      response->print(dateBuffer);                                        // Wert senden
      response->print(F(" Uhr"));
      yield();
    }

    // Hinweis auf NTP anzeigen, wenn aktuelle Zeit ungültig erscheint (wie bisher)
    time_t now_t_stats;
    time(&now_t_stats);
    struct tm now_tm_stats;
    localtime_r(&now_t_stats, &now_tm_stats);
    if (now_tm_stats.tm_year <= (2023 - 1900)) {       // Prüft ob aktuelles Jahr > 2023
      response->print(FPSTR(infoChunk_StatsTimeError));  // Sendet den Hinweis
    }

    // Download Button hinzufügen (wie bisher)
    response->print(FPSTR(infoChunk_StatsDownloadButton));
    response->print(FPSTR(infoChunk_StatsEnd));  // Ende des Statistik-Blocks (</p></form>)
    yield();

  } else {
    // Alternative Meldung, falls keine Statistik verfügbar (wie bisher)
    response->print(FPSTR(infoChunk_StatsNotAvailable));
  }
  // --- Ende Statistikbereich ---
  yield();

  response->print(FPSTR(infoChunk_DeleteStatsForm));

  // --- Systeminfo (mit plattformabhängigen Teilen) ---
  response->print(FPSTR(infoChunk_SysInfoStart));  // Start Systeminfo <form>, endet vor Heap-Wert

  snprintf(buffer, sizeof(buffer), "%u", ESP.getFreeHeap());
  response->print(buffer);                     // Heap-Wert senden
  response->print(FPSTR(infoChunk_AfterFreeHeap));  // Sendet " Bytes<br> NTP Zeit synchronisiert: "
  yield();

  // NTP Status senden (Ja/Nein)
  response->print(timeSynced ? F("Ja") : F("Nein"));
  response->print(FPSTR(infoChunk_AfterNTPStatus));  // Sendet "<br> SDK-Version: "
  yield();

  // --- SDK Version ---
  const char* sdkVersion = ESP.getSdkVersion();
  if (sdkVersion && strlen(sdkVersion) > 0) {
    response->print(sdkVersion);
  } else {
    response->print(F("N/A"));
  }
  // --- Core Version ---
  response->print(FPSTR(infoChunk_AfterSDK));  // "<br> Core-Version: "
  String coreVersionStr = ESP.getCoreVersion();
  yield();
  strncpy(tempStringCopyBuffer, coreVersionStr.c_str(), sizeof(tempStringCopyBuffer) - 1);
  tempStringCopyBuffer[sizeof(tempStringCopyBuffer) - 1] = '\0';
  if (strlen(tempStringCopyBuffer) > 0) response->print(tempStringCopyBuffer);

  // --- Boot Version (Plattformabhängig) ---
  response->print(FPSTR(infoChunk_AfterCore));  // "<br> Boot-Version: "
#ifdef ESP32
  response->print(F("N/A"));  // ESP32 hat keine getBootVersion()
#else                            // ESP8266
  snprintf(buffer, sizeof(buffer), "%u", ESP.getBootVersion());
  response->print(buffer);
#endif

  // --- Chip ID (Plattformabhängig) ---
  response->print(FPSTR(infoChunk_AfterBoot));  // "<br> Chip-ID: "
#ifdef ESP32
  uint64_t chipid_64 = ESP.getEfuseMac();
  uint32_t chipid_32 = (uint32_t)(chipid_64 >> 16);
  snprintf(buffer, sizeof(buffer), "%08X", chipid_32);
#else  // ESP8266
  snprintf(buffer, sizeof(buffer), "%08X", ESP.getChipId());
#endif
  response->print(buffer);  // Formatierten Chip-ID Wert senden

  // MAC-Adresse SICHER ausgeben
  response->print(FPSTR(infoChunk_MacAddress));    // Label "MAC-Adresse (WLAN): " senden
  if (WiFi.status() == WL_CONNECTED) {
      response->print(WiFi.macAddress());         // Die MAC-Adresse als String senden
  } else {
      response->print(F("N/A (nicht verbunden)"));
  }
 
  // --- CPU Takt ---
  response->print(FPSTR(infoChunk_AfterChipID));  // "<br> CPU-Takt: "
  snprintf(buffer, sizeof(buffer), "%u", ESP.getCpuFreqMHz());
  response->print(buffer);

  // --- Reset Grund (Plattformabhängig & KORRIGIERT) ---
  response->print(FPSTR(infoChunk_AfterCPU));  // " MHz<br> Letzter Reset-Grund: "
#ifdef ESP32
  esp_reset_reason_t reason = esp_reset_reason();
  switch (reason) {
    case ESP_RST_UNKNOWN: response->print(F("Unbekannt")); break;
    case ESP_RST_POWERON: response->print(F("Power On")); break;
    case ESP_RST_EXT: response->print(F("External Pin")); break;
    case ESP_RST_SW: response->print(F("Software")); break;
    case ESP_RST_PANIC: response->print(F("Panic/Exception")); break;
    case ESP_RST_INT_WDT: response->print(F("Interrupt Watchdog")); break;
    case ESP_RST_TASK_WDT: response->print(F("Task Watchdog")); break;
    case ESP_RST_WDT: response->print(F("Other Watchdog")); break;
    case ESP_RST_DEEPSLEEP: response->print(F("Deep Sleep Wakeup")); break;
    case ESP_RST_BROWNOUT: response->print(F("Brownout")); break;
    case ESP_RST_SDIO: response->print(F("SDIO")); break;
    default:
      // *** KORREKTUR: String-Verknüpfung vermeiden ***
      response->print(F("Anderer Grund ("));  // Teil 1
      response->print(String(reason));        // Teil 2 (Grund als Zahl)
      response->print(F(")"));                // Teil 3
      break;
  }
#else  // ESP8266
  String resetReasonStr = ESP.getResetReason();
  strncpy(tempStringCopyBuffer, resetReasonStr.c_str(), sizeof(tempStringCopyBuffer) - 1);
  tempStringCopyBuffer[sizeof(tempStringCopyBuffer) - 1] = '\0';
  if (strlen(tempStringCopyBuffer) > 0) response->print(tempStringCopyBuffer);
#endif

  // --- Sketch Größe ---
  response->print(FPSTR(infoChunk_AfterResetReason));  // "<br><br> Sketch-Gr&ouml;&szlig;e: "
  snprintf(buffer, sizeof(buffer), "%u", ESP.getSketchSize());
  response->print(buffer);
  yield();

  // --- Freier Sketch Speicher ---
  response->print(FPSTR(infoChunk_AfterSketchSize));  // " Bytes<br> Freier Sketch-Speicher: "
  snprintf(buffer, sizeof(buffer), "%u", ESP.getFreeSketchSpace());
  response->print(buffer);

  // --- Flash Größe ---
  response->print(FPSTR(infoChunk_AfterFreeSketch));  // " Bytes<br> Flash-Gr&ouml;&szlig;e (Chip): "
  snprintf(buffer, sizeof(buffer), "%u", ESP.getFlashChipSize());
  response->print(buffer);
  yield();                              // Vor dem letzten Chunk
  response->print(FPSTR(infoChunk_End));  // Ende </body></html>
  yield();
  request->send(response);
}

/************************************************************************************
 * PROGMEM Chunks für die Service-Seite (/Service)
 ************************************************************************************/

// --- Kopfzeile ---
static const char serviceHtmlHead[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>Service</title><meta name='theme-color' content='#000000'><meta name='viewport' content='width=device-width, initial-scale=1.0'><meta name='theme-color' content='#000000'><meta name='viewport' content='width=device-width, initial-scale=1.0'>
)rawliteral";
// </head> bleibt gleich
static const char serviceHtmlHeadEnd[] PROGMEM = R"rawliteral(
</head>
)rawliteral";

static const char serviceChunk_BodyStart[] PROGMEM = R"rawliteral(
<body>
  <h1>Service & Wartung</h1>
)rawliteral";

// --- Systemtöne / Piezo ---
static const char serviceChunk_PiezoToggleStart[] PROGMEM = R"rawliteral(
    <form action='/updatePiezoSettings' method='POST'>
      <h3>Systemt&ouml;ne</h3>
      <div class="toggle-switch-container">
          <span class="toggle-switch-label-text">Aktivieren:</span>
          <label class="toggle-switch">
              <input type="checkbox" id="piezoEnabled" name="piezoEnabled" value="1")rawliteral"; // Endet VOR checked
static const char serviceChunk_PiezoToggleEnd[] PROGMEM = R"rawliteral(>
              <span class="toggle-slider"></span>
          </label>
      </div>
      <small class="toggle-description">
        Aktiviert oder deaktiviert die akustischen Signale des Piezo-Lautsprechers (falls angeschlossen).<br>
        Hierdurch werden bei Sensorfehlern, Sicherheitsabschaltung, &Uuml;bergang in den Eco-Modus, ... Hinweist&ouml;ne ausgegeben.<br>
      </small>
      <input type='submit' value='Einstellungen speichern'>
  </form>
)rawliteral";

// --- Wartung Intervall Formular (ehemals infoChunk_MaintenanceForm) ---
static const char serviceChunk_MaintenanceForm[] PROGMEM = R"rawliteral(
  <form action='/updateIntervalSettings' method='POST'>
    <h3>Reinigung & Wartung</h3>
    Zeigt nach der eingestellten Anzahl von Bez&uuml;gen eine Reinigungs-/ Wartungserinnerung im Display an.<br>
    <label for='maintenanceInterval'>Anzahl an Bez&uuml;gen bis Erinnerung: (0 = deaktiviert)</label>
    <input type='number' min='0' step='1' id='maintenanceInterval' name='maintenanceInterval' value=')rawliteral";  // Endet vor Wartungsintervall-Wert
static const char serviceChunk_AfterMaintInterval[] PROGMEM = R"rawliteral('>
    <input type='submit' value='Einstellungen speichern'>
  </form>
)rawliteral";                                                                                    // Ende Wartungsintervall-Formular

// --- Wartungsmodus Sektion (ehemals infoChunk_Wartung...) ---
static const char serviceChunk_WartungStart[] PROGMEM = R"rawliteral(
  <form action='/toggleWartungsmodus' method='POST'>
    <h3>Wartungsmodus (Entkalkung)</h3>
    <p>Aktiviert einen Modus zur einfachen Entkalkung der Maschine.<br>
      Im Wartungsmodus ist das Heizen von Wasser und Dampf deaktiviert.<br>
      Bez&uuml;ge werden in diesem Modus nicht gez&auml;hlt.</p>
    <p><b>Aktueller Status:</b> )rawliteral";          // Endet vor dem Status (Aktiv/Inaktiv)
static const char serviceChunk_WartungButton[] PROGMEM = R"rawliteral(</p>
    <button type='submit'>Wartungsmodus )rawliteral";  // Endet vor dem Button-Text (Aktivieren/Deaktivieren)
static const char serviceChunk_WartungEnd[] PROGMEM = R"rawliteral(</button>
  </form>
)rawliteral";                                          // Ende Wartungsmodus-Formular

// --- Wartungszähler Reset Formular (ehemals infoChunk_MaintenanceResetForm) ---
static const char serviceChunk_MaintenanceResetForm[] PROGMEM = R"rawliteral(
  <form action='/resetMaintenance' method='POST'>
    <h3>Wartungsz&auml;hler zur&uuml;cksetzen</h3>
    Aktueller Z&auml;hlerstand: )rawliteral";  // Endet vor Wartungszähler-Wert
static const char serviceChunk_AfterMaintCounter[] PROGMEM = R"rawliteral(<br>
    <br>
    Setzt den Z&auml;hler f&uuml;r die Reinigungs- & Wartungserinnerung zur&uuml;ck.<br>
    Dies ist nach der erfolgreichen Durchf&uuml;hrung notwendig,<br>
    damit die Meldung nicht nach jedem Bezug erneut scheint.<br>
    <input type='submit' value='Zur&uuml;cksetzen'>
  </form>
)rawliteral";                                  // Ende Wartungszähler-Reset-Formular

static const char serviceChunk_BodyEnd[] PROGMEM = R"rawliteral(
</body>
</html>
)rawliteral";

/************************************************************************************
 * Handler für die Service-Seite (/Service)
 ************************************************************************************/
void handleService(AsyncWebServerRequest *request) {
  AsyncResponseStream *response = request->beginResponseStream("text/html");
  char buffer[20];  // Puffer für Zahlen

  // --- Kopfzeile, Styles, Navigation senden ---
  response->print(FPSTR(serviceHtmlHead));
  response->print(FPSTR(commonStyle));  // Globale Styles nicht vergessen
  response->print(FPSTR(serviceHtmlHeadEnd));
  response->print(FPSTR(commonNav));  // Globale Navigation nicht vergessen
  yield();

  // --- Body Start ---
  response->print(FPSTR(serviceChunk_BodyStart));
  yield();

  // --- Systemtöne / Piezo ---
  response->print(FPSTR(serviceChunk_PiezoToggleStart)); // Bis vor 'checked'
  if (piezoEnabled) {
    response->print(F(" checked")); // Fügt 'checked' hinzu, wenn aktiv
  }
  response->print(FPSTR(serviceChunk_PiezoToggleEnd)); // Schließt den Toggle HTML-Teil
  yield();

  // --- Wartung Intervall Formular ---
  response->print(FPSTR(serviceChunk_MaintenanceForm));  // Start Wartungs-Form, endet vor Intervall-Wert
  snprintf(buffer, sizeof(buffer), "%d", maintenanceInterval);
  response->print(buffer);                             // Wert einfügen
  response->print(FPSTR(serviceChunk_AfterMaintInterval));  // Ende Wartungs-Form
  yield();

  // --- Wartungsmodus Sektion ---
  response->print(FPSTR(serviceChunk_WartungStart));  // Start der Sektion
  if (wartungsModusAktiv) {
    response->print(F("Aktiv"));  // Zeige "Aktiv"
  } else {
    response->print(F("Inaktiv"));  // Zeige "Inaktiv"
  }
  yield();
  response->print(FPSTR(serviceChunk_WartungButton));  // Bis vor Button-Text
  yield();
  if (wartungsModusAktiv) {
    response->print(F("Deaktivieren"));  // Button zum Deaktivieren
  } else {
    response->print(F("Aktivieren"));  // Button zum Aktivieren
  }
  yield();
  response->print(FPSTR(serviceChunk_WartungEnd));  // Rest des Formulars
  yield();

  // --- Wartungszähler Reset Formular ---
  response->print(FPSTR(serviceChunk_MaintenanceResetForm));  // Start Zähler-Reset-Form, endet vor Zähler-Wert
  snprintf(buffer, sizeof(buffer), "%lu", maintenanceIntervalCounter);
  response->print(buffer);                            // Wert einfügen
  response->print(FPSTR(serviceChunk_AfterMaintCounter));  // Ende Zähler-Reset-Form
  yield();

  // --- Body Ende ---
  response->print(FPSTR(serviceChunk_BodyEnd));
  yield();

  request->send(response);
}

/************************************************************************************
 * Handler für die Eco-Modus-Konfiguration - Heap-Optimiert mit Chunked Sending
 ************************************************************************************/

// --- Angepasste PROGMEM Chunks für handleEco mit neuem Toggle Switch ---

// Head und Body Start bleiben gleich
static const char ecoHtmlHead[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>Eco-Modus</title><meta name='theme-color' content='#000000'><meta name='viewport' content='width=device-width, initial-scale=1.0'><meta name='theme-color' content='#000000'><meta name='viewport' content='width=device-width, initial-scale=1.0'>
)rawliteral";
static const char ecoHtmlHeadEnd[] PROGMEM = R"rawliteral(
</head>
)rawliteral";
static const char ecoChunk_BodyStart[] PROGMEM = R"rawliteral(
<body>
  <h1>Eco-Modus</h1>
  <form action='/updateEcoSettings' method='POST'>
    <h3>Eco-Modus</h3>
    Der Eco-Modus senkt die voreingestellten Temperaturen nach der angegebenen Zeit auf die Eco-Temperaturen ab.<br>
    <label for='ecoMode'>Eco-Modus (Minuten, 0 = deaktiviert):</label>
    <input type='number' min='0' step='1' id='ecoMode' name='ecoMode' value=')rawliteral"; // Endet vor {ECO_MODE}
static const char ecoChunk_AfterEcoMode[] PROGMEM = R"rawliteral('>
    <label for='ecoModeTempWasser'>Eco-Temperatur Wasser:</label>
    <input type='number' min='0' max='150' step='1' id='ecoModeTempWasser' name='ecoModeTempWasser' value=')rawliteral"; // Endet vor {ECO_MODE_TEMP_WASSER}
static const char ecoChunk_AfterTempW[] PROGMEM = R"rawliteral('>
    <label for='ecoModeTempDampf'>Eco-Temperatur Dampf:</label>
    <input type='number' min='0' max='200' step='1' id='ecoModeTempDampf' name='ecoModeTempDampf' value=')rawliteral"; // Endet vor {ECO_MODE_TEMP_DAMPF}

// Chunk: Nach Eco-Temperatur Dampf bis VOR den Toggle Switch für Dynamic Eco
static const char ecoChunk_DynamicEco_Start[] PROGMEM = R"rawliteral('>
    <br><h3>Dynamischer Eco-Modus</h3>
)rawliteral";

// Chunk: Der Toggle Switch Container für Dynamic Eco (Start)
static const char ecoChunk_ToggleDynEco_Start[] PROGMEM = R"rawliteral(
    <div class="toggle-switch-container">
        <span class="toggle-switch-label-text">Dynamischen Eco-Modus aktivieren (ECO+):</span>
        <label class="toggle-switch">
            <input type="checkbox" id="dynamicEcoMode" name="dynamicEcoMode" value="1")rawliteral"; // Endet VOR checked

// Chunk: Ende des Toggle Switch Containers
static const char ecoChunk_ToggleDynEco_End[] PROGMEM = R"rawliteral(>
            <span class="toggle-slider"></span>
        </label>
    </div>
)rawliteral"; // Schließt Input, Span, Label, Div

// Chunk: Beschreibung für Dynamic Eco und Start der Dampfverzögerung
static const char ecoChunk_DynEcoDesc_And_SteamDelay[] PROGMEM = R"rawliteral(
    <div style='margin-top:5px; margin-bottom: 15px;'> <small class="toggle-description">
       Der dynamische Eco-Modus verhindert, dass die Maschine bis in die Unendlichkeit eine gewisse Temperatur aufrecht erh&auml;lt.<br>
       Er senkt die Temperatur pro Minute um ein weiteres Grad ab, bis letzlich das Heizen vollst&auml;ndig beendet wird.<br>
       Der dynamische Eco-Modus ist nur in Kombination mit dem Eco-Modus nutzbar!<br>
       <br>
       <b>Beispiel f&uuml;r eine praktische Anwendung:</b><br>
       Peter hat an seiner Maschine den Eco-Modus auf 45 Minuten gestellt, mit Eco-Temperatur von 60 Grad. Nun kommt ihm etwas dazwischen und er kommt erst bei Minute 60 an die Maschine, um sich einen Espresso zuzubereiten. Die Maschine ist nun allerdings schon auf 60 Grad herabgek&uuml;hlt und er muss erneut das Aufheizen abwarten ...<br>
       Markus passiert das gleiche, doch er hat den dynamischen Eco-Modus mit einer initialen Eco-Temperatur von 95 Grad aktiviert - Seine Maschine ist nun zumindest noch bei 80 Grad.<br>
       Markus muss zwar auch warten, kann jedoch noch vor Peter einen Espresso trinken.
     </small>
    </div>
    <h3>Verz&ouml;gerung f&uuml;r Dampf</h3>
    <label for='dampfDelay'>Aufheizen f&uuml;r Dampf verz&ouml;gern (Minuten, 0 = deaktiviert):</label>
    <input type='number' min='0' step='1' id='dampfDelay' name='dampfDelay' value=')rawliteral"; // Endet vor {DAMPF_DELAY}

static const char ecoChunk_AfterDampfDelay_BeforeToggle[] PROGMEM = R"rawliteral('>
)rawliteral";

static const char ecoChunk_SteamOverrideToggle_Start[] PROGMEM = R"rawliteral(
    <div class="toggle-switch-container" style="margin-top: 5px;">
        <span class="toggle-switch-label-text">Aufheizverz&ouml;gerung per Bezugsschalter &uuml;berspringen:</span>
        <label class="toggle-switch">
            <input type="checkbox" id="steamDelayOverrideBySwitch" name="steamDelayOverrideBySwitch" value="1")rawliteral"; // Endet VOR checked

static const char ecoChunk_SteamOverrideToggle_End[] PROGMEM = R"rawliteral(>
            <span class="toggle-slider"></span>
        </label>
    </div>
    <small class="toggle-description">Wenn aktiviert, startet das Heizen des Dampfkreislaufs sofort, sobald der Bezugsschalter bet&auml;tigt wird, und ignoriert die eingestellte Verz&ouml;gerung.</small>
)rawliteral";

static const char ecoChunk_FinalWarningAndSubmit[] PROGMEM = R"rawliteral(
    <br><br><b style='color: #FFCC00;'>ACHTUNG:</b><br>
    Um Konflikte zu vermeiden, sollte der Wert der Verz&ouml;gerung geringer sein, als der des Eco-Modus, falls dieser aktiviert ist!
    <br><br>
    <input type='submit' value='Einstellungen speichern' style='margin-top:10px;'>
  </form>
</body>
</html>
)rawliteral";


// --- Vollständig überarbeitete Funktion handleEco ---
void handleEco(AsyncWebServerRequest *request)  // URL: /ECO
{
  AsyncResponseStream *response = request->beginResponseStream("text/html");
  char buffer[12];  // Puffer für Zahlenumwandlungen

  // Kopfzeile senden
  response->print(FPSTR(ecoHtmlHead));
  response->print(FPSTR(commonStyle));    // Globaler Style mit Toggle-CSS
  response->print(FPSTR(ecoHtmlHeadEnd));
  response->print(FPSTR(commonNav));      // Navigation

  // Body Start bis Eco Mode Wert
  response->print(FPSTR(ecoChunk_BodyStart));
  snprintf(buffer, sizeof(buffer), "%d", ecoModeMinutes); // Eco Mode Wert (int)
  response->print(buffer);
  yield();

  // Nach Eco Mode bis Eco Temp Wasser
  response->print(FPSTR(ecoChunk_AfterEcoMode));
  snprintf(buffer, sizeof(buffer), "%d", ecoModeTempWasser); // Eco Temp Wasser Wert (int)
  response->print(buffer);
  yield();

  // Nach Eco Temp Wasser bis Eco Temp Dampf
  response->print(FPSTR(ecoChunk_AfterTempW));
  snprintf(buffer, sizeof(buffer), "%d", ecoModeTempDampf); // Eco Temp Dampf Wert (int)
  response->print(buffer);
  yield();

  // Dynamic Eco Start und Toggle
  response->print(FPSTR(ecoChunk_DynamicEco_Start)); // '>' nach Dampf Temp, <br>, H3
  response->print(FPSTR(ecoChunk_ToggleDynEco_Start)); // Toggle HTML bis vor checked
  if (dynamicEcoActive) {
    response->print(F(" checked")); // checked Attribut einfügen
  }
  response->print(FPSTR(ecoChunk_ToggleDynEco_End)); // Rest des Toggles
  yield();

  // Beschreibung Dynamic Eco und Start Dampfverzögerung
  response->print(FPSTR(ecoChunk_DynEcoDesc_And_SteamDelay)); // Beschreibung, H3, Label bis value=
  snprintf(buffer, sizeof(buffer), "%d", dampfVerzoegerung); // Dampf Delay Wert (int)
  response->print(buffer);
  yield();

  // Schließt das Input-Feld der Dampfverzögerung
  response->print(FPSTR(ecoChunk_AfterDampfDelay_BeforeToggle));

  // Neuer Toggle-Switch für den Override
  response->print(FPSTR(ecoChunk_SteamOverrideToggle_Start)); // HTML bis vor 'checked'
  if (steamDelayOverrideBySwitchEnabled) {
    response->print(F(" checked")); // 'checked' Attribut einfügen, wenn die Option aktiv ist
  }
  response->print(FPSTR(ecoChunk_SteamOverrideToggle_End)); // Rest des Toggle-HTMLs mit Beschreibung
  yield();

  // Rest der Seite (Warnung, Button, Ende)
  response->print(FPSTR(ecoChunk_FinalWarningAndSubmit));

  request->send(response);
}

/************************************************************************************
 * Handler für die Fast-Heat-Up Seite - Heap-Optimiert mit Chunked Sending
 ************************************************************************************/

// --- Angepasste PROGMEM Chunks für handleFastHeatUp mit Toggle Switch ---

// Kopfzeile bleibt gleich
static const char fhuHtmlHead[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>Fast-Heat-Up-Modus</title><meta name='theme-color' content='#000000'><meta name='viewport' content='width=device-width, initial-scale=1.0'><meta name='theme-color' content='#000000'><meta name='viewport' content='width=device-width, initial-scale=1.0'>
)rawliteral";
// </head> bleibt gleich
static const char fhuHtmlHeadEnd[] PROGMEM = R"rawliteral(
</head>
)rawliteral";

// Neuer Chunk: Body Start bis vor den Toggle Switch
static const char fhuBodyStart[] PROGMEM = R"rawliteral(
<body>
  <h1>Fast-Heat-Up-Modus</h1>
  <form action='/updateFast-Heat-Up-Settings' method='POST'>
    <h3>Fast-Heat-Up-Modus</h3>
)rawliteral";

// Neuer Chunk: Der Toggle Switch Container Start bis VOR das checked Attribut
static const char fhuToggleContainerStart[] PROGMEM = R"rawliteral(
    <div class="toggle-switch-container">
        <span class="toggle-switch-label-text">Fast-Heat-Up aktivieren:</span>
        <label class="toggle-switch">
            <input type="checkbox" id="fastHeatUpAktiv" name="fastHeatUpAktiv" value="1")rawliteral"; // ID geändert für Konsistenz, endet VOR checked

// Neuer Chunk: Ende des Toggle Switch Containers
static const char fhuToggleContainerEnd[] PROGMEM = R"rawliteral(>
            <span class="toggle-slider"></span>
        </label>
    </div>
)rawliteral"; // Schließt Input, Span, Label, Div

// Neuer Chunk: Beschreibungstext und Formular-Ende
static const char fhuDescriptionAndEnd[] PROGMEM = R"rawliteral(
    <div style='margin-top:5px; margin-bottom: 15px;'> <small class="toggle-description"> Der Fast-Heat-Up-Modus erm&ouml;glicht es, die Maschine noch schneller aufzuheizen.<br>
       Der Kessel wird beim Start auf 130 Grad Celsius erhitzt.<br>
       Nachdem die Temperatur erreicht ist, muss ein Flush von ca. 20 Sekunden durchgef&uuml;hrt werden.<br>
       <br>
       <b style='color: #FFCC00;'>ACHTUNG:</b><br>
       Bitte beachten, dass auf der Seite PID-Einstellung eventuell die max. Wassertemperatur der Sicherheitsabschaltung angepasst werden muss!<br>
     </small>
    </div>
    <input type='submit' value='Einstellungen speichern' style='margin-top:10px;'> </form>
</body>
</html>
)rawliteral";

// --- Ende der PROGMEM Chunks ---


// --- Überarbeitete Funktion handleFastHeatUp mit Toggle Switch ---
void handleFastHeatUp(AsyncWebServerRequest *request)  // URL: /Fast-Heat-Up
{
  AsyncResponseStream *response = request->beginResponseStream("text/html");

  // Kopfzeile senden
  response->print(FPSTR(fhuHtmlHead));
  response->print(FPSTR(commonStyle));     // Globaler Style mit Toggle-CSS
  response->print(FPSTR(fhuHtmlHeadEnd));
  response->print(FPSTR(commonNav));       // Navigation

  // Body Start und Formular-Anfang
  response->print(FPSTR(fhuBodyStart));
  yield();

  // Toggle Switch HTML-Teile senden
  response->print(FPSTR(fhuToggleContainerStart)); // Bis vor 'checked'
  if (fastHeatUpAktiv) {
    response->print(F(" checked")); // Fügt 'checked' hinzu, wenn aktiv
  }
  response->print(FPSTR(fhuToggleContainerEnd)); // Schließt den Toggle-Switch
  yield();

  // Beschreibung und Rest der Seite senden
  response->print(FPSTR(fhuDescriptionAndEnd));

  request->send(response);
}

/************************************************************************************
 * Handler zum Speichern der Fast-Heat-Up-Einstellung - URL angepasst
 ************************************************************************************/

void handleFastHeatUpSettings(AsyncWebServerRequest *request) {
  if (request->hasArg(F("fastHeatUpAktiv"))) {
    fastHeatUpAktiv = true;
  } else {
    fastHeatUpAktiv = false;
  }

  EEPROM.put(EEPROM_ADDR_FASTHEATUP_DATA, fastHeatUpAktiv);
  EEPROM.commit();

  AsyncWebServerResponse *resp = request->beginResponse(303);
  resp->addHeader(F("Location"), F("/Fast-Heat-Up"));
  request->send(resp);
}

/************************************************************************************
 * Handler für Root (PID-Einstellungen) - Heap-Optimiert mit Chunked Sending
 ************************************************************************************/

// --- PROGMEM Chunks für handlePidSettings mit Toggle Switches ---

// Head und Body Start bleiben gleich
static const char rootHtmlHead[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>PID-Einstellung</title><meta name='theme-color' content='#000000'><meta name='viewport' content='width=device-width, initial-scale=1.0'><meta name='theme-color' content='#000000'><meta name='viewport' content='width=device-width, initial-scale=1.0'>
)rawliteral";
static const char rootHtmlHeadEnd[] PROGMEM = R"rawliteral(
</head>
)rawliteral";
static const char rootChunk_BodyStart[] PROGMEM = R"rawliteral(
<body>
  <h1>PID-Einstellung</h1>
)rawliteral";

// --- Wasser Temperatur Sektion ---
static const char rootChunk_WasserTempStart[] PROGMEM = R"rawliteral(
  <form action='/updateSettings' method='POST'>
    <h3>Temperatur:  Wasser / Kessel</h3>
    <label for='wasser'>Setpoint (&deg;C):</label>
    <input type='number' min='0' max='135' step='1' id='wasser' name='wasser' value=')rawliteral"; // Endet vor Wasser Setpoint Wert
static const char rootChunk_WasserOffset[] PROGMEM = R"rawliteral('>
    <label for='offsetWasser'>Offset (&deg;C):</label>
    <input type='number' step='0.01' id='offsetWasser' name='offsetWasser' value=')rawliteral"; // Endet vor Wasser Offset Wert

// --- Wasser Boost Toggle ---
static const char rootChunk_ToggleBoostW_Start[] PROGMEM = R"rawliteral('>
    <div class="toggle-switch-container">
        <span class="toggle-switch-label-text">Boost-Funktion:</span>
        <label class="toggle-switch">
            <input type="checkbox" id="boostWasser" name="boostWasser" value="1")rawliteral"; // Endet VOR checked
static const char rootChunk_ToggleBoostW_End[] PROGMEM = R"rawliteral(>
            <span class="toggle-slider"></span>
        </label>
    </div>
    <small class="toggle-description">(Volle Heizleistung erzwingen bis 95% der Zieltemperatur)</small>
)rawliteral";

// --- Wasser Prevent Heat Toggle ---
static const char rootChunk_TogglePreventW_Start[] PROGMEM = R"rawliteral(
    <div class="toggle-switch-container">
        <span class="toggle-switch-label-text">Heizen oberhalb Setpoint verhindern:</span>
        <label class="toggle-switch">
            <input type="checkbox" id="preventHeatWasser" name="preventHeatWasser" value="1")rawliteral"; // Endet VOR checked
static const char rootChunk_TogglePreventW_End[] PROGMEM = R"rawliteral(>
            <span class="toggle-slider"></span>
        </label>
    </div>
    <small class="toggle-description">(Kein Heizen, wenn Temp. > Sollwert)</small>
    <br>
)rawliteral";

// --- Dampf Temperatur Sektion ---
static const char rootChunk_DampfTempStart[] PROGMEM = R"rawliteral(
    <h3>Temperatur:  Dampf / Thermoblock</h3>
    <label for='dampf'>Setpoint (&deg;C):</label>
    <input type='number' min='0' max='200' step='1' id='dampf' name='dampf' value=')rawliteral"; // Endet vor Dampf Setpoint Wert
static const char rootChunk_DampfOffset[] PROGMEM = R"rawliteral('>
    <label for='offsetDampf'>Offset (&deg;C):</label>
    <input type='number' step='0.01' id='offsetDampf' name='offsetDampf' value=')rawliteral"; // Endet vor Dampf Offset Wert

// --- Dampf Boost Toggle ---
static const char rootChunk_ToggleBoostD_Start[] PROGMEM = R"rawliteral('>
    <div class="toggle-switch-container">
        <span class="toggle-switch-label-text">Boost-Funktion:</span>
        <label class="toggle-switch">
            <input type="checkbox" id="boostDampf" name="boostDampf" value="1")rawliteral"; // Endet VOR checked
static const char rootChunk_ToggleBoostD_End[] PROGMEM = R"rawliteral(>
            <span class="toggle-slider"></span>
        </label>
    </div>
    <small class="toggle-description">(Volle Heizleistung erzwingen bis 90% der Zieltemperatur)</small>
)rawliteral";

// --- Dampf Prevent Heat Toggle ---
static const char rootChunk_TogglePreventD_Start[] PROGMEM = R"rawliteral(
    <div class="toggle-switch-container">
        <span class="toggle-switch-label-text">Heizen oberhalb Setpoint verhindern:</span>
        <label class="toggle-switch">
            <input type="checkbox" id="preventHeatDampf" name="preventHeatDampf" value="1")rawliteral"; // Endet VOR checked
static const char rootChunk_TogglePreventD_End[] PROGMEM = R"rawliteral(>
            <span class="toggle-slider"></span>
        </label>
    </div>
    <small class="toggle-description">(Kein Heizen, wenn Temp. > Sollwert)</small>
    <br>
)rawliteral";

// --- PID Wasser Sektion ---
static const char rootChunk_PIDWasser_Start[] PROGMEM = R"rawliteral(
    <h3>PID-Parameter: Wasser / Kessel</h3>
    <label for='kpWasser'>Kp:</label>
    <input type='number' step='0.01' id='kpWasser' name='kpWasser' value=')rawliteral"; // Endet vor Kp Wasser Wert
static const char rootChunk_PIDWasser_Ki[] PROGMEM = R"rawliteral('>
    <label for='kiWasser'>Ki:</label>
    <input type='number' step='0.01' id='kiWasser' name='kiWasser' value=')rawliteral"; // Endet vor Ki Wasser Wert
static const char rootChunk_PIDWasser_Kd[] PROGMEM = R"rawliteral('>
    <label for='kdWasser'>Kd:</label>
    <input type='number' step='0.01' id='kdWasser' name='kdWasser' value=')rawliteral"; // Endet vor Kd Wasser Wert
static const char rootChunk_PIDWasser_Window[] PROGMEM = R"rawliteral('>
    <label for='windowSizeWasser'>Zeitfenster (ms):</label>
    <input type='number' step='1' id='windowSizeWasser' name='windowSizeWasser' value=')rawliteral"; // Step auf 1 geändert für ms, Endet vor Window Size Wert

// --- PID Dampf Sektion ---
static const char rootChunk_PIDDampf_Start[] PROGMEM = R"rawliteral('>
    <br><br> <h3>PID-Parameter: Dampf / Thermoblock</h3>
    <label for='kpDampf'>Kp:</label>
    <input type='number' step='0.01' id='kpDampf' name='kpDampf' value=')rawliteral"; // Endet vor Kp Dampf Wert
static const char rootChunk_PIDDampf_Ki[] PROGMEM = R"rawliteral('>
    <label for='kiDampf'>Ki:</label>
    <input type='number' step='0.01' id='kiDampf' name='kiDampf' value=')rawliteral"; // Endet vor Ki Dampf Wert
static const char rootChunk_PIDDampf_Kd[] PROGMEM = R"rawliteral('>
    <label for='kdDampf'>Kd:</label>
    <input type='number' step='0.01' id='kdDampf' name='kdDampf' value=')rawliteral"; // Endet vor Kd Dampf Wert
static const char rootChunk_PIDDampf_Window[] PROGMEM = R"rawliteral('>
    <label for='windowSizeDampf'>Zeitfenster (ms):</label>
    <input type='number' step='1' id='windowSizeDampf' name='windowSizeDampf' value=')rawliteral"; // Step auf 1 geändert für ms, Endet vor Window Size Wert

// --- Sicherheitsabschaltung Sektion ---
static const char rootChunk_Safety_Start[] PROGMEM = R"rawliteral('>
    <br><br> <h3>Sicherheitsabschaltung</h3>
    <p style="font-size:0.9em; color: #ccc; margin-bottom:15px;">
      Die Sicherheitsabschaltung bietet eine softwareseitige &Uuml;bertemperatursicherung,<br>
      ersetzt jedoch keine hardwareseitige L&ouml;sung durch z.B. Bimetall Temperaturschalter!<br>
      Bei der Verwendung von Fast-Heat-Up muss der Wert f&uuml;r die Maximaltemperatur angepasst werden!
    </p>
    <label for='maxTempWasser'>Max. Temp Wasser (&deg;C):</label>
    <input type='number' step='0.1' id='maxTempWasser' name='maxTempWasser' value=')rawliteral"; // Step 0.1, Endet vor Max Temp Wasser Wert
static const char rootChunk_Safety_MaxDampf[] PROGMEM = R"rawliteral('>
    <label for='maxTempDampf'>Max. Temp Dampf (&deg;C):</label>
    <input type='number' step='0.1' id='maxTempDampf' name='maxTempDampf' value=')rawliteral"; // Step 0.1, Endet vor Max Temp Dampf Wert

// --- Formular Ende ---
static const char rootChunk_FormEnd[] PROGMEM = R"rawliteral('>
    <br> <input type='submit' value='Einstellungen speichern' style='margin-top:15px;'>
  </form>
</body>
</html>
)rawliteral";

// --- Ende der PROGMEM Chunks ---

// Überarbeitete Funktion handlePidSettings mit Toggle Switches
void handlePidSettings(AsyncWebServerRequest *request) {
  char buffer[20];
  AsyncResponseStream *response = request->beginResponseStream("text/html");

  response->write(rootHtmlHead, strlen_P(rootHtmlHead));
  response->write(commonStyle, strlen_P(commonStyle));
  response->write(rootHtmlHeadEnd, strlen_P(rootHtmlHeadEnd));
  response->write(commonNav, strlen_P(commonNav));

  response->write(rootChunk_BodyStart, strlen_P(rootChunk_BodyStart));
  yield();

  String errorMessageHtml = "";
  if (demoModus) {
    errorMessageHtml = "<div class=\"sensor-error-message\"><b>Demo-Betrieb:</b><br>Es sind keine Sensoren angeschlossen und daher keine Temperaturwerte verf&uuml;gbar.</div>";
  } else if (wasserSensorError && dampfSensorError) {
    errorMessageHtml = "<div class=\"sensor-error-message\">Die Daten der Temperatursensoren sind derzeit nicht verf&uuml;gbar!<br>Bitte Temperatur-Sensoren pr&uuml;fen!</div>";
  } else if (wasserSensorError) {
    errorMessageHtml = "<div class=\"sensor-error-message\">Temperaturwert f&uuml;r Wasser nicht verf&uuml;gbar</div>";
  } else if (dampfSensorError) {
    errorMessageHtml = "<div class=\"sensor-error-message\">Temperaturwert f&uuml;r Dampf nicht verf&uuml;gbar</div>";
  }
  if (errorMessageHtml.length() > 0) {
    response->print(errorMessageHtml);
    yield();
  }

  response->write(rootChunk_WasserTempStart, strlen_P(rootChunk_WasserTempStart));
  snprintf(buffer, sizeof(buffer), "%d", (int)SetpointWasser);
  response->print(buffer);
  response->write(rootChunk_WasserOffset, strlen_P(rootChunk_WasserOffset));
  snprintf(buffer, sizeof(buffer), "%.2f", OffsetWasser);
  response->print(buffer);
  yield();

  response->write(rootChunk_ToggleBoostW_Start, strlen_P(rootChunk_ToggleBoostW_Start));
  if (boostWasserActive) response->print(F(" checked"));
  response->write(rootChunk_ToggleBoostW_End, strlen_P(rootChunk_ToggleBoostW_End));
  yield();

  response->write(rootChunk_TogglePreventW_Start, strlen_P(rootChunk_TogglePreventW_Start));
  if (preventHeatAboveSetpointWasser) response->print(F(" checked"));
  response->write(rootChunk_TogglePreventW_End, strlen_P(rootChunk_TogglePreventW_End));
  yield();

  response->write(rootChunk_DampfTempStart, strlen_P(rootChunk_DampfTempStart));
  snprintf(buffer, sizeof(buffer), "%d", (int)SetpointDampf);
  response->print(buffer);
  response->write(rootChunk_DampfOffset, strlen_P(rootChunk_DampfOffset));
  snprintf(buffer, sizeof(buffer), "%.2f", OffsetDampf);
  response->print(buffer);
  yield();

  response->write(rootChunk_ToggleBoostD_Start, strlen_P(rootChunk_ToggleBoostD_Start));
  if (boostDampfActive) response->print(F(" checked"));
  response->write(rootChunk_ToggleBoostD_End, strlen_P(rootChunk_ToggleBoostD_End));
  yield();

  response->write(rootChunk_TogglePreventD_Start, strlen_P(rootChunk_TogglePreventD_Start));
  if (preventHeatAboveSetpointDampf) response->print(F(" checked"));
  response->write(rootChunk_TogglePreventD_End, strlen_P(rootChunk_TogglePreventD_End));
  yield();

  response->write(rootChunk_PIDWasser_Start, strlen_P(rootChunk_PIDWasser_Start));
  snprintf(buffer, sizeof(buffer), "%.2f", KpWasser);
  response->print(buffer);
  response->write(rootChunk_PIDWasser_Ki, strlen_P(rootChunk_PIDWasser_Ki));
  snprintf(buffer, sizeof(buffer), "%.2f", KiWasser);
  response->print(buffer);
  response->write(rootChunk_PIDWasser_Kd, strlen_P(rootChunk_PIDWasser_Kd));
  snprintf(buffer, sizeof(buffer), "%.2f", KdWasser);
  response->print(buffer);
  response->write(rootChunk_PIDWasser_Window, strlen_P(rootChunk_PIDWasser_Window));
  snprintf(buffer, sizeof(buffer), "%lu", windowSizeWasser);
  response->print(buffer);
  yield();

  response->write(rootChunk_PIDDampf_Start, strlen_P(rootChunk_PIDDampf_Start));
  snprintf(buffer, sizeof(buffer), "%.2f", KpDampf);
  response->print(buffer);
  response->write(rootChunk_PIDDampf_Ki, strlen_P(rootChunk_PIDDampf_Ki));
  snprintf(buffer, sizeof(buffer), "%.2f", KiDampf);
  response->print(buffer);
  response->write(rootChunk_PIDDampf_Kd, strlen_P(rootChunk_PIDDampf_Kd));
  snprintf(buffer, sizeof(buffer), "%.2f", KdDampf);
  response->print(buffer);
  response->write(rootChunk_PIDDampf_Window, strlen_P(rootChunk_PIDDampf_Window));
  snprintf(buffer, sizeof(buffer), "%lu", windowSizeDampf);
  response->print(buffer);
  yield();

  response->write(rootChunk_Safety_Start, strlen_P(rootChunk_Safety_Start));
  snprintf(buffer, sizeof(buffer), "%.1f", maxTempWasser);
  response->print(buffer);
  response->write(rootChunk_Safety_MaxDampf, strlen_P(rootChunk_Safety_MaxDampf));
  snprintf(buffer, sizeof(buffer), "%.1f", maxTempDampf);
  response->print(buffer);
  yield();

  response->write(rootChunk_FormEnd, strlen_P(rootChunk_FormEnd));
  request->send(response);
  yield();
}

/************************************************************************************
 * Schreibt die Werte in den EEPROM (PID/Eco) - Unverändert
 ************************************************************************************/

void handleSettingsUpdate(AsyncWebServerRequest *request) {
  bool valueChanged = false;  // Flag um zu prüfen, ob EEPROM.commit() nötig ist

  // --- Bestehende Werte lesen ---
  // Sicherheitslimits
  if (request->hasArg(F("maxTempWasser"))) {
    float newVal = request->arg(F("maxTempWasser")).toFloat();
    if (newVal != maxTempWasser) {
      maxTempWasser = newVal;
      EEPROM.put(EEPROM_ADDR_MAX_TEMP_WASSER, maxTempWasser);
      valueChanged = true;
    }
  }
  if (request->hasArg(F("maxTempDampf"))) {
    float newVal = request->arg(F("maxTempDampf")).toFloat();
    if (newVal != maxTempDampf) {
      maxTempDampf = newVal;
      EEPROM.put(EEPROM_ADDR_MAX_TEMP_DAMPF, maxTempDampf);
      valueChanged = true;
    }
  }
  // Setpoints
  if (request->hasArg(F("wasser"))) {
    float newVal = request->arg(F("wasser")).toFloat();
    if (newVal != SetpointWasser) {
      SetpointWasser = newVal;
      EEPROM.put(EEPROM_ADDR_SETPOINT_WASSER, SetpointWasser);
      valueChanged = true;
    }
  }
  if (request->hasArg(F("dampf"))) {
    float newVal = request->arg(F("dampf")).toFloat();
    if (newVal != SetpointDampf) {
      SetpointDampf = newVal;
      EEPROM.put(EEPROM_ADDR_SETPOINT_DAMPF, SetpointDampf);
      valueChanged = true;
    }
  }
  // Offsets
  if (request->hasArg(F("offsetWasser"))) {
    float newVal = request->arg(F("offsetWasser")).toFloat();
    if (newVal != OffsetWasser) {
      OffsetWasser = newVal;
      EEPROM.put(EEPROM_ADDR_OFFSET_WASSER, OffsetWasser);
      valueChanged = true;
    }
  }
  if (request->hasArg(F("offsetDampf"))) {
    float newVal = request->arg(F("offsetDampf")).toFloat();
    if (newVal != OffsetDampf) {
      OffsetDampf = newVal;
      EEPROM.put(EEPROM_ADDR_OFFSET_DAMPF, OffsetDampf);
      valueChanged = true;
    }
  }
  // PID Wasser
  if (request->hasArg(F("kpWasser"))) {
    float newVal = request->arg(F("kpWasser")).toFloat();
    if (newVal != KpWasser) {
      KpWasser = newVal;
      EEPROM.put(EEPROM_ADDR_KP_WASSER, KpWasser);
      valueChanged = true;
    }
  }
  if (request->hasArg(F("kiWasser"))) {
    float newVal = request->arg(F("kiWasser")).toFloat();
    if (newVal != KiWasser) {
      KiWasser = newVal;
      EEPROM.put(EEPROM_ADDR_KI_WASSER, KiWasser);
      valueChanged = true;
    }
  }
  if (request->hasArg(F("kdWasser"))) {
    float newVal = request->arg(F("kdWasser")).toFloat();
    if (newVal != KdWasser) {
      KdWasser = newVal;
      EEPROM.put(EEPROM_ADDR_KD_WASSER, KdWasser);
      valueChanged = true;
    }
  }
  // PID Dampf
  if (request->hasArg(F("kpDampf"))) {
    float newVal = request->arg(F("kpDampf")).toFloat();
    if (newVal != KpDampf) {
      KpDampf = newVal;
      EEPROM.put(EEPROM_ADDR_KP_DAMPF, KpDampf);
      valueChanged = true;
    }
  }
  if (request->hasArg(F("kiDampf"))) {
    float newVal = request->arg(F("kiDampf")).toFloat();
    if (newVal != KiDampf) {
      KiDampf = newVal;
      EEPROM.put(EEPROM_ADDR_KI_DAMPF, KiDampf);
      valueChanged = true;
    }
  }
  if (request->hasArg(F("kdDampf"))) {
    float newVal = request->arg(F("kdDampf")).toFloat();
    if (newVal != KdDampf) {
      KdDampf = newVal;
      EEPROM.put(EEPROM_ADDR_KD_DAMPF, KdDampf);
      valueChanged = true;
    }
  }
  // Boost Flags
  bool newBoostWasser = request->hasArg(F("boostWasser"));
  if (newBoostWasser != boostWasserActive) {
    boostWasserActive = newBoostWasser;
    EEPROM.put(EEPROM_ADDR_BOOST_WASSER_ACTIVE, boostWasserActive);
    valueChanged = true;
  }
  bool newBoostDampf = request->hasArg(F("boostDampf"));
  if (newBoostDampf != boostDampfActive) {
    boostDampfActive = newBoostDampf;
    EEPROM.put(EEPROM_ADDR_BOOST_DAMPF_ACTIVE, boostDampfActive);
    valueChanged = true;
  }

  // Prevent Heat Above Setpoint Flags
  bool newPreventHeatWasser = request->hasArg(F("preventHeatWasser"));
  if (newPreventHeatWasser != preventHeatAboveSetpointWasser) {
    preventHeatAboveSetpointWasser = newPreventHeatWasser;
    EEPROM.put(EEPROM_ADDR_PREVENTHEAT_WASSER, preventHeatAboveSetpointWasser);
    valueChanged = true;
    // Serial.printf("PreventHeat Wasser %s\n", preventHeatAboveSetpointWasser ? "aktiviert" : "deaktiviert");
  }
  bool newPreventHeatDampf = request->hasArg(F("preventHeatDampf"));
  if (newPreventHeatDampf != preventHeatAboveSetpointDampf) {
    preventHeatAboveSetpointDampf = newPreventHeatDampf;
    EEPROM.put(EEPROM_ADDR_PREVENTHEAT_DAMPF, preventHeatAboveSetpointDampf);
    valueChanged = true;
    // Serial.printf("PreventHeat Dampf %s\n", preventHeatAboveSetpointDampf ? "aktiviert" : "deaktiviert");
  }

  unsigned long newWindowSizeWasser = windowSizeWasser;  // Startwert mit aktuellem Wert
  if (request->hasArg(F("windowSizeWasser"))) {
    char* endptr;
    // Konvertiere den String-Parameter zu unsigned long
    unsigned long val = strtoul(request->arg(F("windowSizeWasser")).c_str(), &endptr, 10);
    // Prüfe, ob Konvertierung erfolgreich war und Wert sinnvoll ist (z.B. >0 und nicht zu groß)
    if (*endptr == '\0' && val > 0 && val <= 60000) {  // Beispiel: max 60 Sekunden
      newWindowSizeWasser = val;
    // } else {
    //   Serial.printf("WARNUNG: Ungültiger Wert für windowSizeWasser empfangen: %s\n", request->arg(F("windowSizeWasser")).c_str());
    }
  }
  // Wenn sich der Wert geändert hat:
  if (newWindowSizeWasser != windowSizeWasser) {
    windowSizeWasser = newWindowSizeWasser;
    EEPROM.put(EEPROM_ADDR_WINDOWSIZE_WASSER, windowSizeWasser);
    pidWasser.SetOutputLimits(0, windowSizeWasser);  // <-- WICHTIG: PID Limit sofort aktualisieren!
    valueChanged = true;
    // Serial.printf("Fenstergröße Wasser aktualisiert auf: %lu ms\n", windowSizeWasser);
  }

  unsigned long newWindowSizeDampf = windowSizeDampf;  // Startwert mit aktuellem Wert
  if (request->hasArg(F("windowSizeDampf"))) {
    char* endptr;
    unsigned long val = strtoul(request->arg(F("windowSizeDampf")).c_str(), &endptr, 10);
    if (*endptr == '\0' && val > 0 && val <= 60000) {
      newWindowSizeDampf = val;
    // } else {
    //   Serial.printf("WARNUNG: Ungültiger Wert für windowSizeDampf empfangen: %s\n", request->arg(F("windowSizeDampf")).c_str());
    }
  }
  // Wenn sich der Wert geändert hat:
  if (newWindowSizeDampf != windowSizeDampf) {
    windowSizeDampf = newWindowSizeDampf;
    EEPROM.put(EEPROM_ADDR_WINDOWSIZE_DAMPF, windowSizeDampf);
    pidDampf.SetOutputLimits(0, windowSizeDampf);  // <-- WICHTIG: PID Limit sofort aktualisieren!
    valueChanged = true;
    // Serial.printf("Fenstergröße Dampf aktualisiert auf: %lu ms\n", windowSizeDampf);
  }


  // --- EEPROM speichern und PID aktualisieren ---
  if (valueChanged) {  // Nur speichern, wenn sich mindestens ein Wert geändert hat
    EEPROM.commit();
  }

  // PID-Parameter an die Regler übergeben
  pidDampf.SetTunings(KpDampf, KiDampf, KdDampf);
  pidWasser.SetTunings(KpWasser, KiWasser, KdWasser);

  // Zurück zur Hauptseite (bereits vorhanden)
  AsyncWebServerResponse *resp = request->beginResponse(303);
  resp->addHeader(F("Location"), F("/PID"));
  request->send(resp);
}

/************************************************************************************
 * Handler für die Firmware & Einstellungen Seite - Heap-Optimiert
 ************************************************************************************/

// --- PROGMEM Chunks für handleFirmware ---
static const char firmwareHtmlHead[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>Firmware & Einstellungen</title><meta name='theme-color' content='#000000'><meta name='viewport' content='width=device-width, initial-scale=1.0'><meta name='theme-color' content='#000000'><meta name='viewport' content='width=device-width, initial-scale=1.0'>
</head>
)rawliteral";  // Head inklusive </head>

static const char firmwareBodyStart[] PROGMEM = R"rawliteral(
<body>
  <h1>Firmware & Einstellungen</h1>
  <form>
    <h3>Firmware-Info</h3>
    Firmware-Version: )rawliteral";  // Endet vor {FIRMWAREVERSION}
static const char firmwareInfoChunk2[] PROGMEM = R"rawliteral(<br>
    Hersteller: )rawliteral";        // Endet vor {SWHERSTELLER}
static const char firmwareInfoChunk3[] PROGMEM = R"rawliteral(<br>
    E-Mail:  )rawliteral";           // Endet vor {SWHERSTELLERMAIL}
static const char firmwareInfoChunk4[] PROGMEM = R"rawliteral(<br>
    Website:  )rawliteral";          // Endet vor {SWHERSTELLERWEBSITE}
static const char firmwareInfoChunkGitHub[] PROGMEM = R"rawliteral(<br>
    GitHub:  )rawliteral";          // Endet vor {SWGITHUB}
static const char firmwareInfoChunkEnd[] PROGMEM = R"rawliteral(<br>
  </form>
)rawliteral";                        // Ende Firmware-Info Sektion

static const char firmwareUpdateSection[] PROGMEM = R"rawliteral(
  <form method='POST' action='/update' enctype='multipart/form-data'>
    <h3>Firmware-Update</h3>
    Das Firmware-Update kann durch den Upload einer .bin-Datei durchgef&uuml;hrt werden.<br>
    Nach dem Update wird ein automatischer Neustart durchgef&uuml;hrt.<br>
    Sollte der Neustart nicht erfolgen, so kann dieser auch durch kurzzeitiges Trennen der Stromversorgung erfolgen.<br>
    <br>
    <input type='file' name='firmware' accept='.bin'>
    <br>
    <button>Update starten</button>
  </form>
)rawliteral";

static const char firmwareFileManagerSection[] PROGMEM = R"rawliteral(
  <form method='GET' action='/Dateimanager' target='_blank'enctype='multipart/form-data'>
    <h3>Dateimanager</h3>
    Verwaltung von Dateien des Systems.<br>
    <button type='submit' >Dateimanager</button>
  </form>
)rawliteral";

static const char firmwareExportSection[] PROGMEM = R"rawliteral(
  <form method='GET' action='/exportSettings' enctype='multipart/form-data'>
    <h3>Einstellungen exportieren</h3>
    Sicherung aller aktuellen Einstellungen in einer Datei.<br>
    <button type='submit' >Exportieren</button>
  </form>
)rawliteral";

static const char firmwareImportSection[] PROGMEM = R"rawliteral(
  <form method='POST' action='/importSettings' enctype='multipart/form-data'>
    <h3>Einstellungen importieren</h3>
    Alle Einstellungen aus einer zuvor exportierten .bin-Datei wiederherstellen.<br><br>
    <b style='color: #FFCC00;'>ACHTUNG:</b><br>
    Dieser Vorgang &uuml;berschreibt ALLE aktuellen Einstellungen!<br>
    Nach dem Import einfolgt ein automatischer Neustart.<br>
    <br>
    <input type='file' name='settings' accept='.bin' required>
    <br>
    <button>Import & Neustart</button>
  </form>
)rawliteral";

static const char firmwareResetSection[] PROGMEM = R"rawliteral(
  <form action='/resetDefaults' method='POST' onsubmit='return confirmResetDefaults();'>
    <h3>Werkseinstellungen</h3>
    Setzt ALLE Einstellungen (mit Ausnahme der WiFi-Konfiguration, der Shot- und Betriebsstundenz&auml;hler) auf die Standardwerte zur&uuml;ck.<br><br>
    <b style='color: #FFCC00;'>ACHTUNG:</b><br>
    Es ist zu empfehlen, vorher einen Export der Einstellungen durchzuf&uuml;hren.
    <br>
    <button type='submit' class='danger'>Werkseinstellungen laden</button>
  </form>
)rawliteral";

static const char firmwareRestartSection[] PROGMEM = R"rawliteral(
  <form action='/restartDevice' method='POST' onsubmit='return confirm("Soll das System wirklich neu gestartet werden?");'>
    <h3>Neustart</h3>
    F&uuml;hrt einen Neustart des Systems durch.<br>
    Alle nicht gespeicherten Einstellungen gehen verloren.<br>
    Die Verbindung wird kurzzeitig unterbrochen.<br>
    <button type='submit' '>Neustart</button>
  </form>
)rawliteral";

static const char firmwareBodyEnd[] PROGMEM = R"rawliteral(
</body>
</html>
)rawliteral";


// --- Neue Funktion handleFirmware ---
void handleFirmware(AsyncWebServerRequest *request) {  // URL: /Firmware, Methode: GET
  AsyncResponseStream *response = request->beginResponseStream("text/html");

  // Kopfzeile senden
  response->print(FPSTR(firmwareHtmlHead));  // Enthält öffnendes <html> und <head> bis </head>
  response->print(FPSTR(commonStyle));
  // </head> ist im Head-Chunk enthalten
  response->print(FPSTR(commonNav));  // Navigation

  // Body Start und Firmware Info Sektion
  response->print(FPSTR(firmwareBodyStart));  // Enthält <body>, <h1> und Start der <form> für Info
  // Firmware Version
  if (version.length() > 0) {  // Prüfen, ob String nicht leer ist
    response->print(version);
  }
  response->print(FPSTR(firmwareInfoChunk2));  // Enthält <br>Hersteller:
  // Hersteller
  if (versionHersteller.length() > 0) {
    response->print(versionHersteller);
  }
  response->print(FPSTR(firmwareInfoChunk3));  // Enthält <br>E-Mail:
  // Mail (ist HTML, kann direkt gesendet werden)
  if (versionHerstellerMail.length() > 0) {
    response->print(versionHerstellerMail);
  }
  response->print(FPSTR(firmwareInfoChunk4));  // Enthält <br>Website:
                                              // Website (ist HTML, kann direkt gesendet werden)
  if (versionHerstellerWeb.length() > 0) {
    response->print(versionHerstellerWeb);
  }
  response->print(FPSTR(firmwareInfoChunkGitHub));  // Enthält <br>GitHub:
  response->print(versionHerstellerGitHub);
  response->print(FPSTR(firmwareInfoChunkEnd));  // Enthält <br></form>

  // Firmware Update Sektion
  response->print(FPSTR(firmwareUpdateSection));

  // Dateimanager Sektion
  response->print(FPSTR(firmwareFileManagerSection));

  // Einstellungen Export Sektion
  response->print(FPSTR(firmwareExportSection));

  // Einstellungen Import Sektion
  response->print(FPSTR(firmwareImportSection));

  // Werkseinstellungen Sektion
  response->print(FPSTR(firmwareResetSection));

  // Neustart Sektion hinzufügen
  response->print(FPSTR(firmwareRestartSection));

  // Body / HTML Ende
  response->print(FPSTR(firmwareBodyEnd));

  request->send(response);
}

/************************************************************************************
 * Handler für AutTune Wasser
 ************************************************************************************/
// --- PROGMEM Chunks für handleAutoTuneWasser ---
static const char autotuneWasserHtmlHead[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>AutoTune Wasser</title><meta name='theme-color' content='#000000'><meta name='viewport' content='width=device-width, initial-scale=1.0'><meta name='theme-color' content='#000000'><meta name='viewport' content='width=device-width, initial-scale=1.0'>
</head>
)rawliteral";

static const char autotuneWasserBodyStart[] PROGMEM = R"rawliteral(
<body>
  <h1>PID-Tuning</h1>
)rawliteral";

static const char autotuneWasserStartSuccess[] PROGMEM = R"rawliteral(
  <form action='/PID-Tuning-Abbruch' method='POST' >
    <p>AutoTune gestartet: Wasser-PID</p>
    <br>
    <input type='submit' value='PID-Tuning abbrechen'>
  </form>
)rawliteral";

static const char autotuneWasserAlreadyActive[] PROGMEM = R"rawliteral(
  <form action='/PID-Tuning-Abbruch' method='POST' >
    <p style="color: red;">AutoTune ist bereits aktiv!</p>
    <br>
    <input type='submit' value='PID-Tuning abbrechen'>
  </form>
)rawliteral";

static const char autotuneWasserBodyEnd[] PROGMEM = R"rawliteral(
</body>
</html>
)rawliteral";

// --- Neue Funktion handleAutoTuneWasser ---
void handleAutoTuneWasser(AsyncWebServerRequest *request) {
  AsyncResponseStream *response = request->beginResponseStream("text/html; charset=utf-8");

  response->print(FPSTR(autotuneWasserHtmlHead));
  response->print(FPSTR(commonStyle));
  response->print(FPSTR(commonNav));
  response->print(FPSTR(autotuneWasserBodyStart));

  // Prüfen, ob bereits ein Tuning läuft
  if (!autoTuneWasserActive && !autoTuneDampfActive) {
    // Wartungsmodus explizit deaktivieren
    if (wartungsModusAktiv) {
      wartungsModusAktiv = false;
      // Standard-Temperaturwerte laden
      EEPROM.get(EEPROM_ADDR_SETPOINT_WASSER, SetpointWasser);
      EEPROM.get(EEPROM_ADDR_SETPOINT_DAMPF, SetpointDampf);
    }

    startAutoTuneWasser();
    response->print(FPSTR(autotuneWasserStartSuccess));
  } else {
    response->print(FPSTR(autotuneWasserAlreadyActive));
  }

  response->print(FPSTR(autotuneWasserBodyEnd));
  request->send(response);
}

/************************************************************************************
 * Handler für AutTune Dampf
 ************************************************************************************/

// --- PROGMEM Chunks für handleAutoTuneDampf ---
static const char autotuneDampfHtmlHead[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>AutoTune Dampf</title><meta name='theme-color' content='#000000'><meta name='viewport' content='width=device-width, initial-scale=1.0'><meta name='theme-color' content='#000000'><meta name='viewport' content='width=device-width, initial-scale=1.0'>
</head>
)rawliteral";

// BodyStart, AlreadyActive und BodyEnd können von handleAutoTuneWasser wiederverwendet werden
// (autotuneWasserBodyStart, autotuneWasserAlreadyActive, autotuneWasserBodyEnd)

static const char autotuneDampfStartSuccess[] PROGMEM = R"rawliteral(
  <form action='/PID-Tuning-Abbruch' method='POST' >
    <p>AutoTune gestartet: Dampf-PID</p>
    <br>
    <input type='submit' value='PID-Tuning abbrechen'>
  </form>
)rawliteral";

void handleAutoTuneDampf(AsyncWebServerRequest *request) {
  AsyncResponseStream *response = request->beginResponseStream("text/html; charset=utf-8");

  response->print(FPSTR(autotuneDampfHtmlHead));  // Eigener Head-Chunk
  response->print(FPSTR(commonStyle));
  response->print(FPSTR(commonNav));
  response->print(FPSTR(autotuneWasserBodyStart));  // Wiederverwendet

  // Prüfen, ob bereits ein Tuning läuft
  if (!autoTuneWasserActive && !autoTuneDampfActive) {
    // Wartungsmodus explizit deaktivieren
    if (wartungsModusAktiv) {
      wartungsModusAktiv = false;
      EEPROM.get(EEPROM_ADDR_SETPOINT_WASSER, SetpointWasser);
      EEPROM.get(EEPROM_ADDR_SETPOINT_DAMPF, SetpointDampf);
    }

    startAutoTuneDampf();                             // Funktion zum Starten des Tunings aufrufen
    response->print(FPSTR(autotuneDampfStartSuccess));  // Eigener Success-Chunk
  } else {
    response->print(FPSTR(autotuneWasserAlreadyActive));  // Wiederverwendet
  }

  response->print(FPSTR(autotuneWasserBodyEnd));  // Wiederverwendet
  request->send(response);
}

/************************************************************************************
 * Handler zum Speichern der AutoTune-Einstellungen - ausgelagert
 ************************************************************************************/
void handleUpdateAutotuneSettings(AsyncWebServerRequest *request) {
  bool changed = false;  // Flag, um zu prüfen, ob EEPROM.commit() nötig ist

  // --- Wasser Parameter ---
  if (request->hasArg(F("tuningStepWasser"))) {
    tuningStepWasser = request->arg(F("tuningStepWasser")).toFloat();
    EEPROM.put(EEPROM_ADDR_TUNING_STEP_WASSER, tuningStepWasser);
    changed = true;
  }
  if (request->hasArg(F("tuningNoiseWasser"))) {
    tuningNoiseWasser = request->arg(F("tuningNoiseWasser")).toFloat();
    EEPROM.put(EEPROM_ADDR_TUNING_NOISE_WASSER, tuningNoiseWasser);
    changed = true;
  }
  if (request->hasArg(F("tuningStartValueWasser"))) {
    tuningStartValueWasser = request->arg(F("tuningStartValueWasser")).toFloat();
    EEPROM.put(EEPROM_ADDR_TUNING_STARTVALUE_WASSER, tuningStartValueWasser);
    changed = true;
  }
  if (request->hasArg(F("tuningLookBackWasser"))) {
    int lookbackInt = request->arg(F("tuningLookBackWasser")).toInt();
    if (lookbackInt >= 0) {
      tuningLookBackWasser = (unsigned int)lookbackInt;
      EEPROM.put(EEPROM_ADDR_TUNING_LOOKBACK_WASSER, tuningLookBackWasser);
      changed = true;
    }
  }

  // --- Dampf Parameter ---
  if (request->hasArg(F("tuningStepDampf"))) {
    tuningStepDampf = request->arg(F("tuningStepDampf")).toFloat();
    EEPROM.put(EEPROM_ADDR_TUNING_STEP_DAMPF, tuningStepDampf);
    changed = true;
  }
  if (request->hasArg(F("tuningNoiseDampf"))) {
    tuningNoiseDampf = request->arg(F("tuningNoiseDampf")).toFloat();
    EEPROM.put(EEPROM_ADDR_TUNING_NOISE_DAMPF, tuningNoiseDampf);
    changed = true;
  }
  if (request->hasArg(F("tuningStartValueDampf"))) {
    tuningStartValueDampf = request->arg(F("tuningStartValueDampf")).toFloat();
    EEPROM.put(EEPROM_ADDR_TUNING_STARTVALUE_DAMPF, tuningStartValueDampf);
    changed = true;
  }
  if (request->hasArg(F("tuningLookBackDampf"))) {
    int lookbackInt = request->arg(F("tuningLookBackDampf")).toInt();
    if (lookbackInt >= 0) {
      tuningLookBackDampf = (unsigned int)lookbackInt;
      EEPROM.put(EEPROM_ADDR_TUNING_LOOKBACK_DAMPF, tuningLookBackDampf);
      changed = true;
    }
  }

  if (changed) {
    EEPROM.commit();
  }

  AsyncWebServerResponse *response = request->beginResponse(303);
  response->addHeader(F("Location"), F("/PID-Tuning"));
  request->send(response);
}

/************************************************************************************
 * Handler für PID-Tuning
 ************************************************************************************/

// --- PROGMEM Chunks für handlePidSettingsTuning ---
static const char pidTuningPageHtmlHead[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>PID-Tuning</title><meta name='theme-color' content='#000000'><meta name='viewport' content='width=device-width, initial-scale=1.0'><meta name='theme-color' content='#000000'><meta name='viewport' content='width=device-width, initial-scale=1.0'>
</head>
)rawliteral";

static const char pidTuningPageBodyStart[] PROGMEM = R"rawliteral(
<body>
)rawliteral";

static const char pidTuningPageAbortForm[] PROGMEM = R"rawliteral(
  <form action='/PID-Tuning-Abbruch' method='POST' >
    <h3>Laufendes PID-Tuning abbrechen?</h3>
    <input type='submit' value='Abbrechen'>
  </form></br>
)rawliteral";

static const char pidTuningPageIntro[] PROGMEM = R"rawliteral(
  <h1>PID-Tuning</h1>
  <form>Das automatische PID-Tuning dient dazu, die optimalen Parameter f&uuml;r den PID-Regler (Proportional-, Integral- und Differentialanteil) selbst&auml;ndig zu ermitteln. Dabei analysiert das System mithilfe von Algorithmen das Regelverhalten (z. B. Reaktion auf einen Testimpuls) und passt Kp, Ki und Kd so an, dass gew&uuml;nschte Kriterien wie kurze Einschwingzeit, geringe &Uuml;berschwingung und stabile Regelung erreicht werden
  <br><br>
  Ein AutoTune-Vorgang kann durchaus bis zu 60 Minuten in Anspruch nehmen!
  <br><br>
  <b style='color: #FFCC00;'>WICHTIG:</b><br>
  Vor dem Start des PID-Tunings sollte die Maschine die gew&uuml;nschte Betriebstemperatur erreicht haben und stabil sein.<br>
  Andernfalls k&ouml;nnten die automatisch ermittelten PID-Werte f&uuml;r diesen Temperaturbereich ungenau oder nicht optimal sein.
  <br><br>
  </form>
  <form action='/AutoTune-Wasser' method='POST' >
    <h3>Automatisches PID-Tuning f&uuml;r Wasser</h3>
    Automatische Ermittlung der PID-Werte f&uuml;r Wasser / Kessel<br>
    <input type='submit' value='Tuning starten'>
  </form>
  <form action='/AutoTune-Dampf' method='POST' >
    <h3>Automatisches PID-Tuning f&uuml;r Dampf</h3>
    Automatische Ermittlung der PID-Werte f&uuml;r Dampf / Thermoblock<br>
    <input type='submit' value='Tuning starten'>
  </form>
)rawliteral";

// --- Chunks für das Parameter-Formular ---
static const char pidTuningPageParamsFormStart[] PROGMEM = R"rawliteral(
  <form action='/updateAutotuneSettings' method='POST'>
    <h3>AutoTune-Parameter (Zeitproportional)</h3>
    Hier k&ouml;nnen die AutoTune-Parameter ge&auml;ndert werden.<br>&Auml;nderungen sollten mit Vorsicht durchgef&uuml;hrt werden!<br><br>
    <h3>Parameter Wasser / Kessel</h3>
    <label for='tuningStepWasser'>Step (Output 0-)rawliteral";                                         // Endet vor {WINDOW_SIZE_WASSER}
static const char pidTuningPageParamsW1[] PROGMEM = R"rawliteral():</label>
    <input type='number' step='0.01' id='tuningStepWasser' name='tuningStepWasser' value=')rawliteral";              // Endet vor {TUNING_STEP_WASSER}
static const char pidTuningPageParamsW2[] PROGMEM = R"rawliteral('>
    <label for='tuningNoiseWasser'>Noise (&deg;C):</label>
    <input type='number' step='0.01' id='tuningNoiseWasser' name='tuningNoiseWasser' value=')rawliteral";            // Endet vor {TUNING_NOISE_WASSER}
static const char pidTuningPageParamsW3[] PROGMEM = R"rawliteral('>
    <label for='tuningStartValueWasser'>StartValue (Output 0-)rawliteral";                             // Endet vor {WINDOW_SIZE_WASSER}
static const char pidTuningPageParamsW4[] PROGMEM = R"rawliteral():</label>
    <input type='number' step='0.01' id='tuningStartValueWasser' name='tuningStartValueWasser' value=')rawliteral";  // Endet vor {TUNING_STARTVALUE_WASSER}
static const char pidTuningPageParamsW5[] PROGMEM = R"rawliteral('>
    <label for='tuningLookBackWasser'>LookBack (Sekunden):</label>
    <input type='number' step='1' id='tuningLookBackWasser' name='tuningLookBackWasser' value=')rawliteral";      // Endet vor {TUNING_LOOKBACK_WASSER}
static const char pidTuningPageParamsDStart[] PROGMEM = R"rawliteral('>
    <br><h3>Parameter Dampf / Thermoblock</h3>
    <label for='tuningStepDampf'>Step (Output 0-)rawliteral";                                          // Endet vor {WINDOW_SIZE_DAMPF}
static const char pidTuningPageParamsD1[] PROGMEM = R"rawliteral():</label>
    <input type='number' step='0.01' id='tuningStepDampf' name='tuningStepDampf' value=')rawliteral";                // Endet vor {TUNING_STEP_DAMPF}
static const char pidTuningPageParamsD2[] PROGMEM = R"rawliteral('>
    <label for='tuningNoiseDampf'>Noise (&deg;C):</label>
    <input type='number' step='0.01' id='tuningNoiseDampf' name='tuningNoiseDampf' value=')rawliteral";              // Endet vor {TUNING_NOISE_DAMPF}
static const char pidTuningPageParamsD3[] PROGMEM = R"rawliteral('>
    <label for='tuningStartValueDampf'>StartValue (Output 0-)rawliteral";                              // Endet vor {WINDOW_SIZE_DAMPF}
static const char pidTuningPageParamsD4[] PROGMEM = R"rawliteral():</label>
    <input type='number' step='0.01' id='tuningStartValueDampf' name='tuningStartValueDampf' value=')rawliteral";    // Endet vor {TUNING_STARTVALUE_DAMPF}
static const char pidTuningPageParamsD5[] PROGMEM = R"rawliteral('>
    <label for='tuningLookBackDampf'>LookBack (Sekunden):</label>
    <input type='number' step='1' 'tuningLookBackDampf' name='tuningLookBackDampf' value=')rawliteral";        // Endet vor {TUNING_LOOKBACK_DAMPF}
static const char pidTuningPageParamsFormEnd[] PROGMEM = R"rawliteral('>
    <input type='submit' value='Parameter speichern'>
  </form>
)rawliteral";

static const char pidTuningPageExplanation[] PROGMEM = R"rawliteral(
  <br><hr><br>
  <form>
    <h3>Erkl&auml;rung der Parameter (Zeitproportional):</h3>
    <p><strong>Noise (&deg;C):</strong><br>
    Beobachte das "Zittern" der Temperaturanzeige, wenn die Temperatur (nahe am Zielwert) stabil ist.<br>Wenn sie z.B. um +/- 0.5&deg;C schwankt, setze Noise auf <code>1.0</code>.<br>Dieser Wert definiert ein Toleranzband, um das Sensorrauschen zu ignorieren.</p>
    <p><strong>StartValue (ms):</strong><br>
    Sch&auml;tze, wie viel Heizzeit (in Millisekunden, innerhalb des eingestellten Zeitfensters (siehe PID-Einstellung) n&ouml;tig ist, um die Zieltemperatur konstant zu halten.<br>Dies ist die durchschnittliche Heizzeit im stabilen Zustand (z.B. <code>250</code> ms).</p>
    <p><strong>Step (ms):</strong><br>
    Die Gr&ouml;&szlig;e des "Sprungs" der Heizzeit (in ms) nach oben/unten um den <code>StartValue</code>, den AutoTune nutzt, um die Temperatur zum Schwingen zu zwingen.<br>Muss gro&szlig; genug f&uuml;r eine Reaktion sein, aber nicht 0-100% (z.B. <code>500</code> ms).</p>
    <p><strong>LookBack (s):</strong><br>
    AutoTune erzeugt eine langsame Welle der Temperatur. Miss (in einem Testlauf) oder sch&auml;tze, wie lange eine volle Welle dauert (Spitze zu Spitze, in Sekunden).<br>Setze Lookback auf einen Wert, der *deutlich l&auml;nger* ist als diese Dauer (z.B. 1.5x so lang; Welle=130s => Lookback=<code>180</code>s).<br>Dieser Wert muss *vor* dem Start festgelegt werden und sagt AutoTune, wie weit es zur&uuml;ckschauen soll, um die Welle zu messen.</p>
    <p><strong>Kontrolle per Chart:</strong><br>
    W&auml;hrend AutoTune l&auml;uft, beobachte das Temperaturchart.<br>Wenn sich dort eine langsame, regelm&auml;&szlig;ige Welle (wie ein \"langgezogener Sinus\") um den Sollwert entwickelt, ist das ein gutes Zeichen, dass der Prozess funktioniert.</p>
  </form>
</body>
</html>
)rawliteral";

// --- handlePidSettingsTuning ---
void handlePidSettingsTuning(AsyncWebServerRequest *request) {
  char buffer[20];  // Puffer für Zahlenumwandlungen

  AsyncResponseStream *response = request->beginResponseStream("text/html; charset=utf-8");

  response->print(FPSTR(pidTuningPageHtmlHead));
  response->print(FPSTR(commonStyle));
  response->print(FPSTR(commonNav));
  response->print(FPSTR(pidTuningPageBodyStart));

  if (autoTuneWasserActive || autoTuneDampfActive) {
    response->print(FPSTR(pidTuningPageAbortForm));
  }

  response->print(FPSTR(pidTuningPageIntro));

  response->print(FPSTR(pidTuningPageParamsFormStart));
  snprintf(buffer, sizeof(buffer), "%lu", windowSizeWasser);
  response->print(buffer);
  response->print(FPSTR(pidTuningPageParamsW1));
  snprintf(buffer, sizeof(buffer), "%.2f", tuningStepWasser);
  response->print(buffer);
  response->print(FPSTR(pidTuningPageParamsW2));
  snprintf(buffer, sizeof(buffer), "%.2f", tuningNoiseWasser);
  response->print(buffer);
  response->print(FPSTR(pidTuningPageParamsW3));
  snprintf(buffer, sizeof(buffer), "%lu", windowSizeWasser);
  response->print(buffer);
  response->print(FPSTR(pidTuningPageParamsW4));
  snprintf(buffer, sizeof(buffer), "%.2f", tuningStartValueWasser);
  response->print(buffer);
  response->print(FPSTR(pidTuningPageParamsW5));
  snprintf(buffer, sizeof(buffer), "%u", tuningLookBackWasser);
  response->print(buffer);

  response->print(FPSTR(pidTuningPageParamsDStart));
  snprintf(buffer, sizeof(buffer), "%lu", windowSizeDampf);
  response->print(buffer);
  response->print(FPSTR(pidTuningPageParamsD1));
  snprintf(buffer, sizeof(buffer), "%.2f", tuningStepDampf);
  response->print(buffer);
  response->print(FPSTR(pidTuningPageParamsD2));
  snprintf(buffer, sizeof(buffer), "%.2f", tuningNoiseDampf);
  response->print(buffer);
  response->print(FPSTR(pidTuningPageParamsD3));
  snprintf(buffer, sizeof(buffer), "%lu", windowSizeDampf);
  response->print(buffer);
  response->print(FPSTR(pidTuningPageParamsD4));
  snprintf(buffer, sizeof(buffer), "%.2f", tuningStartValueDampf);
  response->print(buffer);
  response->print(FPSTR(pidTuningPageParamsD5));
  snprintf(buffer, sizeof(buffer), "%u", tuningLookBackDampf);
  response->print(buffer);

  response->print(FPSTR(pidTuningPageParamsFormEnd));

  response->print(FPSTR(pidTuningPageExplanation));

  request->send(response);
}

/************************************************************************************
 * Handler für die Antwort nach Abbruch des PID-Tunings - Heap-Optimiert
 ************************************************************************************/

// PROGMEM Chunks für die Seite (Namen angepasst für Klarheit)
static const char abortPidTuningHtmlStart[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>Info</title><meta name='theme-color' content='#000000'><meta name='viewport' content='width=device-width, initial-scale=1.0'><meta name='theme-color' content='#000000'><meta name='viewport' content='width=device-width, initial-scale=1.0'>
)rawliteral";
static const char abortPidTuningHtmlHeadEnd[] PROGMEM = R"rawliteral(</head>)rawliteral";
static const char abortPidTuningHtmlBody[] PROGMEM = R"rawliteral(
<body>
  <h1>PID-Tuning Abbruch</h1>
  <form>
    <h3>Das PID-Tuning wurde erfolgreich abgebrochen.</h3>
    <p>Der Normalbetrieb wird nun fortgesetzt.</p>
  </form>
</body>
</html>
)rawliteral";

// --- Neue Funktion handlePidSettingsTuningAbbruch ---
void handlePidSettingsTuningAbbruch(AsyncWebServerRequest *request) {
  AsyncResponseStream *response = request->beginResponseStream("text/html; charset=utf-8");

  response->print(FPSTR(abortPidTuningHtmlStart));
  response->print(FPSTR(commonStyle));
  response->print(FPSTR(abortPidTuningHtmlHeadEnd));
  response->print(FPSTR(commonNav));
  response->print(FPSTR(abortPidTuningHtmlBody));

  request->send(response);

  stopAutoTuneWasser();
  stopAutoTuneDampf();
}

/************************************************************************************
 * Speichert Eco-Einstellungen - URL angepasst
 ************************************************************************************/

void handleEcoUpdate(AsyncWebServerRequest *request) {
  if (request->hasArg(F("ecoMode"))) ecoModeMinutes = request->arg(F("ecoMode")).toInt();
  if (request->hasArg(F("ecoModeTempWasser"))) ecoModeTempWasser = request->arg(F("ecoModeTempWasser")).toInt();
  if (request->hasArg(F("ecoModeTempDampf"))) ecoModeTempDampf = request->arg(F("ecoModeTempDampf")).toInt();

  if (request->hasArg(F("dynamicEcoMode"))) {
    dynamicEcoActive = true;
  } else {
    dynamicEcoActive = false;
  }

  if (request->hasArg(F("dampfDelay"))) {
    dampfVerzoegerung = request->arg(F("dampfDelay")).toInt();
  }

  if (request->hasArg("steamDelayOverrideBySwitch")) {
    steamDelayOverrideBySwitchEnabled = true;
  } else {
    steamDelayOverrideBySwitchEnabled = false;
  }

  EEPROM.put(EEPROM_ADDR_ECOMODE_MINUTES, ecoModeMinutes);
  EEPROM.put(EEPROM_ADDR_ECOMODE_TEMP_WASSER, ecoModeTempWasser);
  EEPROM.put(EEPROM_ADDR_ECOMODE_TEMP_DAMPF, ecoModeTempDampf);
  EEPROM.put(EEPROM_ADDR_DYNAMIC_ECO_MODE, dynamicEcoActive);
  EEPROM.put(EEPROM_ADDR_STEAM_DELAY, dampfVerzoegerung);
  EEPROM.commit();

  // Wenn Eco deaktiviert, Setpoints zurückladen
  if (ecoModeMinutes == 0) {
    ecoModeAktiv = false;
    ecoModeActivatedTime = 0;
    EEPROM.get(EEPROM_ADDR_SETPOINT_WASSER, SetpointWasser);
    EEPROM.get(EEPROM_ADDR_SETPOINT_DAMPF, SetpointDampf);
  }

  AsyncWebServerResponse *resp = request->beginResponse(303);
  resp->addHeader(F("Location"), F("/ECO"));  // URL angepasst
  request->send(resp);
}

/************************************************************************************
 * Speichert Geräte-Infos 
 ************************************************************************************/

void handleInfoUpdate(AsyncWebServerRequest *request) {
  bool changed = false;
  if (request->hasArg(F("hersteller"))) {
    strncpy(infoHersteller, request->arg(F("hersteller")).c_str(), sizeof(infoHersteller) - 1);
    infoHersteller[sizeof(infoHersteller) - 1] = '\0';
    EEPROM.put(EEPROM_ADDR_INFO_HERSTELLER, infoHersteller);
    changed = true;
  }
  if (request->hasArg(F("modell"))) {
    strncpy(infoModell, request->arg(F("modell")).c_str(), sizeof(infoModell) - 1);
    infoModell[sizeof(infoModell) - 1] = '\0';
    EEPROM.put(EEPROM_ADDR_INFO_MODELL, infoModell);
    changed = true;
  }
  if (request->hasArg(F("zusatz"))) {
    strncpy(infoZusatz, request->arg(F("zusatz")).c_str(), sizeof(infoZusatz) - 1);
    infoZusatz[sizeof(infoZusatz) - 1] = '\0';
    EEPROM.put(EEPROM_ADDR_INFO_ZUSATZ, infoZusatz);
    changed = true;
  }

  if (changed) {
    EEPROM.commit();
  }

  AsyncWebServerResponse *response = request->beginResponse(303);
  response->addHeader(F("Location"), F("/Info"));
  request->send(response);
}

/************************************************************************************
 * Speichert NUR die Einstellung für die Systemtöne (Piezo)
 ************************************************************************************/
void handleUpdatePiezoSettings(AsyncWebServerRequest *request) {
    bool changed = false;
    // Piezo-Status speichern
    bool newPiezoState = request->hasArg("piezoEnabled"); // Checkbox ist da, wenn checked
    if (newPiezoState != piezoEnabled) {
        piezoEnabled = newPiezoState;
        EEPROM.put(EEPROM_ADDR_PIEZO_ENABLED, piezoEnabled);
        changed = true;
    }

    if (changed) {
        EEPROM.commit();
    }

    AsyncWebServerResponse *resp = request->beginResponse(303);
    resp->addHeader("Location", "/Service");
    request->send(resp);
}

/************************************************************************************
 * Speichert NUR die Einstellung für das Wartungsintervall
 ************************************************************************************/
void handleUpdateIntervalSettings(AsyncWebServerRequest *request) {
    bool changed = false;
    // Wartungsintervall speichern
    if (request->hasArg("maintenanceInterval")) {
        int newInterval = request->arg("maintenanceInterval").toInt();
        if (newInterval < 0) newInterval = 0; // Sicherstellen, dass nicht negativ
        if (newInterval != maintenanceInterval) {
            maintenanceInterval = newInterval;
            EEPROM.put(EEPROM_ADDR_MAINTENANCE_INTERVAL, maintenanceInterval);
            changed = true;
        }
    }

    if (changed) {
        EEPROM.commit();
    }

    AsyncWebServerResponse *resp = request->beginResponse(303);
    resp->addHeader("Location", "/Service");
    request->send(resp);
}

/************************************************************************************
 * Handler für JSON-Chart-Daten (/Chart-Daten) - Heap-Optimiert
 ************************************************************************************/
void handleChartData(AsyncWebServerRequest *request) {
  AsyncResponseStream *response = request->beginResponseStream("application/json");
  char buffer[20];

  response->print(F("{\"wasser\":"));
  snprintf(buffer, sizeof(buffer), "%.1f", InputWasser);
  response->print(buffer);

  response->print(F(",\"setpointwasser\":"));
  snprintf(buffer, sizeof(buffer), "%.1f", SetpointWasser);
  response->print(buffer);

  response->print(F(",\"dampf\":"));
  snprintf(buffer, sizeof(buffer), "%.1f", InputDampf);
  response->print(buffer);

  response->print(F(",\"setpointdampf\":"));
  snprintf(buffer, sizeof(buffer), "%.1f", SetpointDampf);
  response->print(buffer);

  response->print(F("}"));
  request->send(response);
}

/************************************************************************************
 * Handler für Temperatur-Charts (Live) - URL angepasst (/Chart)
 * Lädt chart.js bevorzugt aus LittleFS, sonst CDN, sonst Fehler.
 * Auswahl für maximale Datenpunkte hinzugefügt.
 * Vollbild-Funktion hinzugefügt (Layout korrigiert: Label über Select).
 ************************************************************************************/

// --- PROGMEM Chunks für die Chart-Seite ---

// Teil 1: HTML-Start, Meta-Tags
static const char chartsHtml_HeadStart[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="de">
<head>
  <title>Temperatur-Chart</title>
  <meta charset="UTF-8">
  <meta name='theme-color' content='#111111'>
  <meta name='viewport' content='width=device-width, initial-scale=1.0'>
)rawliteral";

// Teil 1b: Spezifische Styles (Normal-Layout: Button mittig zu den Blöcken)
static const char chartsHtml_SpecificStyles[] PROGMEM = R"rawliteral(
  <style>
    /* Body und Haupt-Wrapper für Flexbox */
    html { height: 100%; }
    body {
      margin: 0; display: flex; flex-direction: column; min-height: 100vh;
    }
    main.chart-page-content {
      flex: 1; display: flex; flex-direction: column; padding: 15px;
      box-sizing: border-box; overflow: auto;
      max-width: 1600px; margin-left: auto; margin-right: auto; width: 100%;
    }
    h1.chart-title {
    }
    /* Container für Selektoren & Button im Normalzustand */
    .chart-controls-container {
      display: flex;
      flex-wrap: wrap;
      justify-content: center;
      align-items: center; /* <<< ZURÜCK ZU CENTER für vertikale Mitte */
      gap: 25px;
      margin: 10px auto 20px auto;
      position: relative;
      z-index: 10;
      transition: all 0.3s ease-in-out;
    }
    /* Container für einzelne Controls (Label+Select) im Normalzustand */
    .chart-controls-container > div {
        text-align: left;
    }
    .chart-controls-container label {
       display: block;
       margin-bottom: 5px;
       color: #ccc; font-size: 0.9em;
    }
    .chart-controls-container select {
      background: #252525; color: #e0e0e0; border: 1px solid #555;
      padding: 8px 12px; border-radius: 8px; font-size: 0.9em; cursor: pointer;
      min-width: 150px; height: 38px; box-sizing: border-box;
    }
     .chart-controls-container select:focus {
      outline: none; border-color: var(--accent-color, #d89904);
      box-shadow: 0 0 8px rgba(216, 153, 4, 0.3);
    }

     /* Style für den Vollbild-Button im Normalzustand */
     #chart-fullscreen-btn {
        padding: 8px 12px; border-radius: 8px; font-size: 0.9em; cursor: pointer;
        height: 38px; box-sizing: border-box; min-width: 100px;
        display: inline-block;
        transition: all 0.3s ease-in-out;
        margin-top: 30px;
     }
     #chart-fullscreen-btn:hover { 
        background: #d89904; color: #000000; border: 1px solid #555;
      }
     #chart-fullscreen-btn:focus { outline: none; border-color: var(--accent-color, #d89904); box-shadow: 0 0 8px rgba(216, 153, 4, 0.3); }

    /* Restliche Styles bleiben unverändert */
    .chart-container { width: 95%; height: 65vh; min-height: 350px; margin: 15px auto; background: rgba(0, 0, 0, 0.2); box-shadow: 0 6px 20px rgba(0, 0, 0, 0.4); border-radius: 15px; position: relative; border: 1px solid rgba(255, 255, 255, 0.1); padding: 20px; box-sizing: border-box; transition: all 0.3s ease-in-out; }
    .chart-container canvas { width: 100% !important; height: 100% !important; }
    .sensor-error-message { color: #FFCC00; text-align: center; margin-top: 5px; margin-bottom: 15px; padding: 8px 10px; background-color: rgba(255, 204, 0, 0.1); border: 1px solid rgba(255, 204, 0, 0.2); border-radius: 8px; font-weight: 500; font-size: 0.9em; max-width: 600px; margin-left: auto; margin-right: auto; }
    .chart-error-box { margin: 20px auto; padding: 20px; max-width: 600px; background-color: rgba(255, 221, 221, 0.9); border: 1px solid #dc3545; color: #333; border-radius: 8px; text-align: center; }
    .chart-error-box h2 { color: #dc3545; margin-bottom: 10px;}
    .chart-error-box a { color: #0056b3; text-decoration: underline; }

    /* --- Vollbild-Styles (Unverändert zur funktionierenden Version) --- */
     body.chart-fullscreen-active { overflow: hidden; }
     body.chart-fullscreen-active nav, body.chart-fullscreen-active h1.chart-title, body.chart-fullscreen-active footer { display: none !important; }
     body.chart-fullscreen-active main.chart-page-content { padding: 0; max-width: none; height: 100vh; overflow: hidden; }
     body.chart-fullscreen-active .chart-container { position: fixed; top: 0; left: 0; width: 100vw; height: 100vh; z-index: 9999; margin: 0; padding: 10px; border-radius: 0; border: none; background: var(--primary-bg, #000000); box-shadow: none; }
     body.chart-fullscreen-active .chart-controls-container { position: fixed; top: 10px; right: 15px; z-index: 10000; margin: 0; padding: 0; width: auto; background: none; box-shadow: none; gap: 0; display: block; }
     body.chart-fullscreen-active .chart-controls-container > div { display: none !important; }
     body.chart-fullscreen-active #chart-fullscreen-btn { border: 1px solid #666; margin: 0; }

    /* Mobile Styles (Unverändert zur letzten Version) */
    @media (max-width: 600px) {
       h1.chart-title {}
       .chart-container { width: 98%; padding: 10px; height: 65vh; }
       .chart-controls-container { flex-direction: column; gap: 15px; align-items: center; }
       .chart-controls-container > div { width: 100%; max-width: 250px; text-align: center; }
       .chart-controls-container select { width: 100%; }
       #chart-fullscreen-btn { margin-top: 10px; margin-left: 0; width: 100%; max-width: 250px; }
       .sensor-error-message { width: 90%; font-size: 0.85em; }
       .chart-error-box { width: 90%; padding: 15px;}
       body.chart-fullscreen-active .chart-controls-container { top: 5px; right: 5px; }
       body.chart-fullscreen-active #chart-fullscreen-btn { padding: 6px 10px; font-size: 0.8em; }
    }
  </style>
)rawliteral";

// Teil 2: Schließendes Head-Tag
static const char chartsHtml_HeadEnd[] PROGMEM = "</head>\n";

// Teil 3: Selektoren-UI (HTML-Struktur angepasst: Label über Select)
static const char chartsHtml_SelectorsUI[] PROGMEM = R"rawliteral(
  <div class="chart-controls-container">
    <div> <label for="updateRate">Aktualisierungsrate:</label>
      <select id="updateRate">
        <option value="1000">1 Sekunde</option> <option value="2000">2 Sekunden</option> <option value="3000">3 Sekunden</option> <option value="5000">5 Sekunden</option> <option value="10000">10 Sekunden</option> <option value="15000" selected>15 Sekunden</option> <option value="30000">30 Sekunden</option> <option value="45000">45 Sekunden</option> <option value="60000">60 Sekunden</option>
      </select>
    </div>
    <div> <label for="datapointLimit">Max. Datenpunkte:</label>
      <select id="datapointLimit">
        <option value="30">30</option> <option value="50">50</option> <option value="100" selected>100</option> <option value="150">150</option> <option value="200">200</option> <option value="300">300</option> <option value="500">500</option>
      </select>
    </div>
    <button id="chart-fullscreen-btn">Vollbild</button>
  </div>
)rawliteral";


// Teil 4: Chart.js Code (Unverändert zur letzten Version)
static const char chartsHtml_ChartJSCode[] PROGMEM = R"rawliteral(
  const ctx = document.getElementById('combinedChart').getContext('2d');
  let currentSetpointDampf = 0; let currentSetpointWasser = 0;
const wasserColor = '#00AEEF';
const dampfColor = '#F7941D';
const setpointWasserColor = 'rgba(0, 174, 239, 0.7)';
const setpointDampfColor = 'rgba(247, 148, 29, 0.7)';
const gridColor = 'rgba(255, 255, 255, 0.1)';
const textColor = '#E0E0E0';
  let maxDataPoints = parseInt(document.getElementById('datapointLimit').value, 10) || 100;
  const combinedChart = new Chart(ctx, { type: 'line', data: { labels: [], datasets: [ { label: 'Wasser IST', data: [], borderColor: wasserColor, backgroundColor: 'rgba(0, 174, 239, 0.1)', borderWidth: 2, fill: 'start', tension: 0.4, pointRadius: 0, pointHoverRadius: 6, pointHitRadius: 10, pointHoverBackgroundColor: wasserColor }, { label: 'Dampf IST', data: [], borderColor: dampfColor, backgroundColor: 'rgba(247, 148, 29, 0.1)', borderWidth: 2, fill: 'start', tension: 0.4, pointRadius: 0, pointHoverRadius: 6, pointHitRadius: 10, pointHoverBackgroundColor: dampfColor }, { label: 'Wasser SOLL', data: [], borderColor: setpointWasserColor, borderWidth: 1.5, borderDash: [6, 3], fill: false, tension: 0.1, pointRadius: 0, pointHoverRadius: 0 }, { label: 'Dampf SOLL', data: [], borderColor: setpointDampfColor, borderWidth: 1.5, borderDash: [6, 3], fill: false, tension: 0.1, pointRadius: 0, pointHoverRadius: 0 } ] }, options: { responsive: true, maintainAspectRatio: false, animation: false, interaction: { intersect: false, mode: 'index' }, scales: { x: { type: 'category', title: { display: true, text: 'Uhrzeit', color: textColor, font: { size: 13, weight: '300' } }, ticks: { color: textColor, font: { size: 11 }, maxRotation: 0, autoSkip: true, maxTicksLimit: 8 }, grid: { color: gridColor, drawBorder: false } }, y: { suggestedMin: 0, suggestedMax: 180, title: { display: true, text: 'Temperatur (°C)', color: textColor, font: { size: 13, weight: '300' } }, ticks: { color: textColor, font: { size: 11 }, stepSize: 20, callback: function(value, index, values) { return value + ' °C'; } }, grid: { color: gridColor, drawBorder: false } } }, plugins: { legend: { position: 'bottom', align: 'center', labels: { color: textColor, font: { size: 12 }, usePointStyle: true, pointStyle: 'rectRounded', boxWidth: 15, padding: 15 } }, tooltip: { enabled: true, mode: 'index', intersect: false, backgroundColor: 'rgba(0, 0, 0, 0.85)', titleColor: '#ffffff', titleFont: { size: 13, weight: 'bold' }, bodyColor: '#dddddd', bodyFont: { size: 12 }, bodySpacing: 4, borderColor: 'rgba(255, 255, 255, 0.1)', borderWidth: 1, padding: 10, cornerRadius: 8, displayColors: true, boxPadding: 4, callbacks: { label: function(context) { let label = context.dataset.label || ''; if (label) { label += ': '; } if (context.parsed.y !== null) { label += context.parsed.y.toFixed(1) + ' °C'; } return label; } } } } } });
  let updateInterval = parseInt(document.getElementById('updateRate').value, 10) || 15000; let intervalId = null;
  function fetchData() { fetch('/Chart-Daten').then(response => { if (!response.ok) { console.error('Netzwerkantwort war nicht ok:', response.statusText); throw new Error('Network response was not ok'); } return response.json(); }).then(data => { if (data.setpointwasser !== undefined) currentSetpointWasser = data.setpointwasser; if (data.setpointdampf !== undefined) currentSetpointDampf = data.setpointdampf;
const now = new Date();
const timeLabel = now.toLocaleTimeString('de-DE');
const labels = combinedChart.data.labels;
const datasets = combinedChart.data.datasets; labels.push(timeLabel); datasets[0].data.push(data.wasser); datasets[1].data.push(data.dampf); datasets[2].data.push(currentSetpointWasser); datasets[3].data.push(currentSetpointDampf); while (labels.length > maxDataPoints) { labels.shift(); datasets.forEach(dataset => { dataset.data.shift(); }); } combinedChart.update('none'); }).catch(err => { console.error("Fehler beim Abrufen der Daten:", err); }); }
  function startFetching() { if (intervalId) { clearInterval(intervalId); intervalId = null; } fetchData(); intervalId = setInterval(fetchData, updateInterval); console.log(`Chart-Aktualisierung gestartet: Intervall ${updateInterval / 1000} Sekunden.`); }
  document.getElementById('updateRate').addEventListener('change', function(event) { const newInterval = parseInt(event.target.value, 10); if (!isNaN(newInterval) && newInterval > 0) { updateInterval = newInterval; startFetching(); } });
  document.getElementById('datapointLimit').addEventListener('change', function(event) { const newLimit = parseInt(event.target.value, 10); if (!isNaN(newLimit) && newLimit > 0) { maxDataPoints = newLimit; console.log(`Maximale Datenpunkte geändert auf: ${maxDataPoints}`); } });
 const fullscreenBtn = document.getElementById('chart-fullscreen-btn');
const bodyElement = document.body;
 fullscreenBtn.addEventListener('click', () => { bodyElement.classList.toggle('chart-fullscreen-active'); if (bodyElement.classList.contains('chart-fullscreen-active')) { fullscreenBtn.textContent = 'Verlassen'; } else { fullscreenBtn.textContent = 'Vollbild'; } setTimeout(() => { if (typeof combinedChart !== 'undefined' && combinedChart) { combinedChart.resize(); } }, 50); });
  startFetching();
)rawliteral";

// Teil 5: Chunk für den Fehlerfall
static const char chartsHtml_ErrorBox[] PROGMEM = R"rawliteral(
  <div class='chart-error-box'> <h2>Chart kann nicht angezeigt werden</h2> <p>Die Chart-Bibliothek (chart.umd.min.js) wurde weder im Dateisystem gefunden, noch besteht eine Internetverbindung zum Laden vom CDN.</p> <p>Bitte laden Sie die Datei <a href='https://cdn.jsdelivr.net/npm/chart.js@4.4.2/dist/chart.umd.min.js' target='_blank'>chart.umd.min.js</a> herunter und laden Sie sie über den <a href='/Dateimanager' target='_blank'>Dateimanager</a> hoch, oder verbinden Sie das Gerät mit dem Internet.</p> </div>
)rawliteral";

// Teil 6: Schließendes Body- und HTML-Tag
static const char chartsHtml_BodyEnd[] PROGMEM = R"rawliteral(
</body>
</html>
)rawliteral";


// --- handleCharts Funktion ---
void handleCharts(AsyncWebServerRequest *request)  // URL: /Chart
{
  bool chartJsLocal = LittleFS.exists("/chart.umd.min.js");
  bool isConnected = (WiFi.status() == WL_CONNECTED);
  bool canLoadChartJs = false;

  AsyncResponseStream *response = request->beginResponseStream("text/html; charset=utf-8");

  response->print(FPSTR(chartsHtml_HeadStart));
  response->print(FPSTR(commonStyle));
  response->print(FPSTR(chartsHtml_SpecificStyles)); // CSS mit korrektem Normal-Layout

  if (chartJsLocal) {
    File chartFile = LittleFS.open("/chart.umd.min.js", "r");
    if (chartFile) {
      canLoadChartJs = true;
      response->print(F("<script>\n"));
      uint8_t buf[256];
      while (chartFile.available()) {
        size_t len = chartFile.read(buf, sizeof(buf));
        response->write(buf, len);
      }
      chartFile.close();
      response->print(F("\n</script>\n"));
      yield();
    }
  } else if (isConnected) {
    response->print(F("<script src=\"https://cdn.jsdelivr.net/npm/chart.js@4.4.2/dist/chart.umd.min.js\"></script>\n"));
    canLoadChartJs = true;
    yield();
  }
  response->print(FPSTR(chartsHtml_HeadEnd));

  response->print("<body>\n");
  response->print(FPSTR(commonNav));
  response->print("<main class=\"chart-page-content\">\n");

  response->print("<h1 class=\"chart-title\">Temperatur-Chart</h1>\n");

  if (canLoadChartJs) {
    response->print(FPSTR(chartsHtml_SelectorsUI)); // HTML mit Label über Select
    yield();

    // Sensorfehler-Meldungen
    String errorMessageHtml = "";
    if (demoModus) { errorMessageHtml = "<div class=\"sensor-error-message\"><b>Demo-Betrieb:</b><br>Es sind keine Sensoren angeschlossen.</div>"; }
    else if (wasserSensorError && dampfSensorError) { errorMessageHtml = "<div class=\"sensor-error-message\">Temperatursensoren nicht verf&uuml;gbar! Bitte pr&uuml;fen!</div>"; }
    else if (wasserSensorError) { errorMessageHtml = "<div class=\"sensor-error-message\">Temperaturwert Wasser nicht verf&uuml;gbar</div>"; }
    else if (dampfSensorError) { errorMessageHtml = "<div class=\"sensor-error-message\">Temperaturwert Dampf nicht verf&uuml;gbar</div>"; }
    if (errorMessageHtml.length() > 0) { response->print(errorMessageHtml); }
    yield();

    response->print("<div class=\"chart-container\">\n");
    response->print("<canvas id=\"combinedChart\"></canvas>\n");
    response->print("</div>\n");
    yield();

    response->print("<script>\n");
    response->print(FPSTR(chartsHtml_ChartJSCode));
    response->print("\n</script>\n");
    yield();

  } else {
    response->print(FPSTR(chartsHtml_ErrorBox));
    yield();
  }

  response->print("</main>\n");
  response->print(FPSTR(chartsHtml_BodyEnd));

  request->send(response);
} // Ende handleCharts()

/************************************************************************************
 * Hilfsfunktionen für Profile
 ************************************************************************************/

// Hilfsfunktion zum Bereinigen von Profilnamen für Dateinamen
String sanitizeProfileName(String profileName) {
    profileName.trim(); // Leerzeichen am Anfang/Ende entfernen
    if (profileName.length() == 0) {
        profileName = "Unbenannt";
    }
    // Ersetze ungültige Zeichen (Beispiel: Leerzeichen, /, \, :, *, ?, ", <, >, |)
    profileName.replace(" ", "_");
    profileName.replace("/", "_");
    profileName.replace("\\", "_");
    profileName.replace(":", "_");
    profileName.replace("*", "_");
    profileName.replace("?", "_");
    profileName.replace("\"", "_");
    profileName.replace("<", "_");
    profileName.replace(">", "_");
    profileName.replace("|", "_");

    // Maximale Länge begrenzen (z.B. 30 Zeichen + .txt)
    if (profileName.length() > 30) {
        profileName = profileName.substring(0, 30);
    }
    return profileName;
}

// Listet alle Profile (.prof Dateien) im /Profile Verzeichnis auf
std::vector<String> listProfiles() {
    std::vector<String> profileNames;
    // Serial.println("Listing profiles (.prof) in /Profile:"); // Debug-Meldung angepasst
    #ifdef ESP32
        File root = LittleFS.open("/Profile");
        if (!root || !root.isDirectory()) {
            // Serial.println("  Failed to open /Profile directory.");
            if(root) root.close();
            return profileNames; // Leere Liste zurückgeben
        }
        File file = root.openNextFile();
        while(file){
            String filePath = file.path(); // ESP32 gibt vollen Pfad inkl. /Profile/ zurück
            if (!file.isDirectory() && filePath.endsWith(".prof")) { // *** GEÄNDERT ZU .prof ***
                // Extrahiere nur den Namen ohne Pfad und Endung
                int lastSlash = filePath.lastIndexOf('/');
                int lastDot = filePath.lastIndexOf('.'); // Sollte .prof sein
                if (lastSlash != -1 && lastDot != -1 && lastDot > lastSlash) {
                    String profileName = filePath.substring(lastSlash + 1, lastDot); // Extrahiert den Namen
                    profileNames.push_back(profileName);
                    // Serial.printf("  - Found profile: %s\n", profileName.c_str());
                // } else {
                //     Serial.printf("  - Ignoring file (invalid format?): %s\n", filePath.c_str());
                }
            // } else if (!file.isDirectory()) {
            //      Serial.printf("  - Ignoring file (not .prof): %s\n", filePath.c_str()); // Meldung angepasst
            // } else {
            //     Serial.printf("  - Ignoring directory: %s\n", filePath.c_str());
            }
            file.close(); // Wichtig!
            file = root.openNextFile();
            yield(); // Wichtig bei vielen Dateien
        }
        root.close();
    #else // ESP8266
        Dir dir = LittleFS.openDir("/Profile");
        while (dir.next()) {
            String fileName = dir.fileName(); // Gibt nur Dateiname.prof zurück
            if (!dir.isDirectory() && fileName.endsWith(".prof")) { // *** GEÄNDERT ZU .prof ***
                int lastDot = fileName.lastIndexOf('.'); // Sollte .prof sein
                if (lastDot != -1) {
                    String profileName = fileName.substring(0, lastDot); // Extrahiert den Namen
                    profileNames.push_back(profileName);
                //     Serial.printf("  - Found profile: %s\n", profileName.c_str());
                // } else {
                //      Serial.printf("  - Ignoring file (no extension?): %s\n", fileName.c_str());
                }
            // } else if (!dir.isDirectory()) {
            //      Serial.printf("  - Ignoring file (not .prof): %s\n", fileName.c_str()); // Meldung angepasst
            // } else {
            //     Serial.printf("  - Ignoring directory: %s\n", fileName.c_str());
            }
             yield(); // Wichtig bei vielen Dateien
        }
        // dir schließt automatisch
    #endif
    // Serial.printf("Found %d profiles.\n", profileNames.size());
    return profileNames;
}


// Speichert die übergebenen Profileinstellungen als BINÄRE Datei (.prof)
bool saveProfile(const String& profileName, const TemperatureProfile& profileData) {
    String sanitizedName = sanitizeProfileName(profileName);
    if (sanitizedName.length() == 0) return false; // Kein gültiger Name nach Bereinigung

    // Dateiendung auf .prof ändern, um Text von Binär zu unterscheiden
    String filePath = "/Profile/" + sanitizedName + ".prof";
    // Serial.printf("Saving profile (binary) to: %s\n", filePath.c_str());

    File profileFile = LittleFS.open(filePath, "w"); // "w" für (binäres) Schreiben
    if (!profileFile) {
        // Serial.println("  ERROR: Failed to open file for writing.");
        return false;
    }

    // Stelle sicher, dass die Versionsnummer korrekt ist, bevor gespeichert wird
    // Das sollte im Konstruktor oder in getCurrentSettingsAsProfile passieren,
    // aber zur Sicherheit hier nochmal setzen:
    TemperatureProfile dataToSave = profileData; // Kopie erstellen
    dataToSave.profileVersion = CURRENT_PROFILE_VERSION; // Sicherstellen!

    // Schreibe die gesamte Struktur als Bytes
    size_t bytesWritten = profileFile.write((uint8_t*)&dataToSave, sizeof(TemperatureProfile));

    profileFile.close(); // Datei schließen

    if (bytesWritten == sizeof(TemperatureProfile)) {
        // Serial.println("  Profile saved successfully (Binary Format).");
        return true;
    } else {
        // Serial.printf("  ERROR: Failed to write complete profile data! Bytes written: %d, Expected: %d\n", bytesWritten, sizeof(TemperatureProfile));
        // Versuch, die unvollständige Datei zu löschen
        LittleFS.remove(filePath);
        return false;
    }
}

// Lädt ein Profil aus einer BINÄREN Datei (.prof) in die übergebene Struktur
bool loadProfile(const String& profileName, TemperatureProfile& profileData) {
    String sanitizedName = sanitizeProfileName(profileName); // Verwende den Namen aus der Liste
    if (sanitizedName.length() == 0) return false;

    // Dateiendung auf .prof ändern
    String filePath = "/Profile/" + sanitizedName + ".prof";
    // Serial.printf("Loading profile (binary) from: %s\n", filePath.c_str());

    if (!LittleFS.exists(filePath)) {
        //  Serial.println("  ERROR: Profile file not found.");
         return false;
    }

    File profileFile = LittleFS.open(filePath, "r"); // "r" für (binäres) Lesen
    if (!profileFile) {
        // Serial.println("  ERROR: Failed to open profile file for reading.");
        return false;
    }

    // --- Validierung der Dateigröße ---
    size_t fileSize = profileFile.size();
    if (fileSize != sizeof(TemperatureProfile)) {
        // Serial.printf("  ERROR: Profile file size mismatch! Expected %d bytes, got %d bytes.\n", sizeof(TemperatureProfile), fileSize);
        // Serial.println("         Profile might be corrupted or from an incompatible version.");
        profileFile.close();
        return false;
    }

    // Lese die Struktur-Daten
    size_t bytesRead = profileFile.read((uint8_t*)&profileData, sizeof(TemperatureProfile));
    profileFile.close(); // Datei nach dem Lesen schließen

    if (bytesRead != sizeof(TemperatureProfile)) {
        //  Serial.printf("  ERROR: Failed to read complete profile data! Bytes read: %d\n", bytesRead);
         return false;
    }

    // --- Validierung der Profil-Version ---
    if (profileData.profileVersion != CURRENT_PROFILE_VERSION) {
        // Serial.printf("  ERROR: Profile version mismatch! Expected version %d, got version %d.\n", CURRENT_PROFILE_VERSION, profileData.profileVersion);
        // Serial.println("         Cannot load profile from incompatible version.");
        // Optional: Hier könnte man versuchen, alte Versionen zu konvertieren
        return false;
    }

    // Serial.println("  Profile loaded successfully (Binary Format).");
    // Der in der Datei gespeicherte `profileName` ist jetzt in profileData.profileName
    // Serial.printf("  Loaded Profile Name from file: %s\n", profileData.profileName);
    return true;
}

/************************************************************************************
 * Wendet ALLE geladenen Profileinstellungen auf die aktuellen Systemvariablen und EEPROM an
 * Inklusive Brew-by-Weight und Versionscheck.
 ************************************************************************************/
void applyProfileSettings(const TemperatureProfile& profileData) {
    // Serial.printf("Applying settings from loaded profile: %s (Version %d)\n", profileData.profileName, profileData.profileVersion);

    // --- Prüfen der Profilversion (SEHR WICHTIG!) ---
    if (profileData.profileVersion != CURRENT_PROFILE_VERSION) {
        //  Serial.printf("  FEHLER: Kann Profil '%s' nicht anwenden. Version %d ist inkompatibel mit erwarteter Version %d.\n",
        //                profileData.profileName, profileData.profileVersion, CURRENT_PROFILE_VERSION);
         profileStatusMessage = "FEHLER: Profil '" + String(profileData.profileName) + "' hat eine inkompatible Version ("
                                + String(profileData.profileVersion) + ", erwartet " + String(CURRENT_PROFILE_VERSION) + ").";
         return; // Funktion hier abbrechen
    }

    // --- Globale Variablen direkt aktualisieren ---
    SetpointWasser = profileData.setpointWasser; SetpointDampf = profileData.setpointDampf;
    OffsetWasser = profileData.offsetWasser; OffsetDampf = profileData.offsetDampf;
    KpWasser = profileData.kpWasser; KiWasser = profileData.kiWasser; KdWasser = profileData.kdWasser;
    KpDampf = profileData.kpDampf; KiDampf = profileData.kiDampf; KdDampf = profileData.kdDampf;
    boostWasserActive = profileData.boostWasserActive; boostDampfActive = profileData.boostDampfActive;
    preventHeatAboveSetpointWasser = profileData.preventHeatAboveSetpointWasser; preventHeatAboveSetpointDampf = profileData.preventHeatAboveSetpointDampf;
    windowSizeWasser = profileData.windowSizeWasser; windowSizeDampf = profileData.windowSizeDampf;
    maxTempWasser = profileData.maxTempWasser; maxTempDampf = profileData.maxTempDampf;
    tuningStepWasser = profileData.tuningStepWasser; tuningNoiseWasser = profileData.tuningNoiseWasser; tuningStartValueWasser = profileData.tuningStartValueWasser; tuningLookBackWasser = profileData.tuningLookBackWasser;
    tuningStepDampf = profileData.tuningStepDampf; tuningNoiseDampf = profileData.tuningNoiseDampf; tuningStartValueDampf = profileData.tuningStartValueDampf; tuningLookBackDampf = profileData.tuningLookBackDampf;
    ecoModeMinutes = profileData.ecoModeMinutes; ecoModeTempWasser = profileData.ecoModeTempWasser; ecoModeTempDampf = profileData.ecoModeTempDampf;
    dynamicEcoActive = profileData.dynamicEcoActive; dampfVerzoegerung = profileData.dampfVerzoegerung;
    fastHeatUpAktiv = profileData.fastHeatUpAktiv;

    // Brew Control Einstellungen anwenden
    brewByTimeEnabled = profileData.brewByTimeEnabled; brewByTimeTargetSeconds = profileData.brewByTimeTargetSeconds;
    preInfusionEnabled = profileData.preInfusionEnabled; preInfusionDurationSeconds = profileData.preInfusionDurationSeconds; preInfusionPauseSeconds = profileData.preInfusionPauseSeconds;

    // Brew-by-Weight Einstellungen anwenden
#ifdef ESP32
    brewByWeightEnabled = profileData.brewByWeightEnabled; brewByWeightTargetGrams = profileData.brewByWeightTargetGrams; brewByWeightOffsetGrams = profileData.brewByWeightOffsetGrams;
#endif

    // Piezo Status anwenden
    piezoEnabled = profileData.piezoEnabled;

    // Dampfverzögerungs-Override anwenden
    steamDelayOverrideBySwitchEnabled = profileData.steamDelayOverrideBySwitch;

    // --- Werte ins EEPROM schreiben (an die korrekten Adressen) ---
    EEPROM.put(EEPROM_ADDR_SETPOINT_WASSER, profileData.setpointWasser); EEPROM.put(EEPROM_ADDR_SETPOINT_DAMPF, profileData.setpointDampf);
    EEPROM.put(EEPROM_ADDR_OFFSET_WASSER, profileData.offsetWasser); EEPROM.put(EEPROM_ADDR_OFFSET_DAMPF, profileData.offsetDampf);
    EEPROM.put(EEPROM_ADDR_KP_WASSER, profileData.kpWasser); EEPROM.put(EEPROM_ADDR_KI_WASSER, profileData.kiWasser); EEPROM.put(EEPROM_ADDR_KD_WASSER, profileData.kdWasser);
    EEPROM.put(EEPROM_ADDR_KP_DAMPF, profileData.kpDampf); EEPROM.put(EEPROM_ADDR_KI_DAMPF, profileData.kiDampf); EEPROM.put(EEPROM_ADDR_KD_DAMPF, profileData.kdDampf);
    EEPROM.put(EEPROM_ADDR_BOOST_WASSER_ACTIVE, profileData.boostWasserActive); EEPROM.put(EEPROM_ADDR_BOOST_DAMPF_ACTIVE, profileData.boostDampfActive);
    EEPROM.put(EEPROM_ADDR_PREVENTHEAT_WASSER, profileData.preventHeatAboveSetpointWasser); EEPROM.put(EEPROM_ADDR_PREVENTHEAT_DAMPF, profileData.preventHeatAboveSetpointDampf);
    EEPROM.put(EEPROM_ADDR_WINDOWSIZE_WASSER, profileData.windowSizeWasser); EEPROM.put(EEPROM_ADDR_WINDOWSIZE_DAMPF, profileData.windowSizeDampf);
    EEPROM.put(EEPROM_ADDR_MAX_TEMP_WASSER, profileData.maxTempWasser); EEPROM.put(EEPROM_ADDR_MAX_TEMP_DAMPF, profileData.maxTempDampf);
    EEPROM.put(EEPROM_ADDR_TUNING_STEP_WASSER, profileData.tuningStepWasser); EEPROM.put(EEPROM_ADDR_TUNING_NOISE_WASSER, profileData.tuningNoiseWasser); EEPROM.put(EEPROM_ADDR_TUNING_STARTVALUE_WASSER, profileData.tuningStartValueWasser); EEPROM.put(EEPROM_ADDR_TUNING_LOOKBACK_WASSER, profileData.tuningLookBackWasser);
    EEPROM.put(EEPROM_ADDR_TUNING_STEP_DAMPF, profileData.tuningStepDampf); EEPROM.put(EEPROM_ADDR_TUNING_NOISE_DAMPF, profileData.tuningNoiseDampf); EEPROM.put(EEPROM_ADDR_TUNING_STARTVALUE_DAMPF, profileData.tuningStartValueDampf); EEPROM.put(EEPROM_ADDR_TUNING_LOOKBACK_DAMPF, profileData.tuningLookBackDampf);
    EEPROM.put(EEPROM_ADDR_ECOMODE_MINUTES, profileData.ecoModeMinutes); EEPROM.put(EEPROM_ADDR_ECOMODE_TEMP_WASSER, profileData.ecoModeTempWasser); EEPROM.put(EEPROM_ADDR_ECOMODE_TEMP_DAMPF, profileData.ecoModeTempDampf);
    EEPROM.put(EEPROM_ADDR_DYNAMIC_ECO_MODE, profileData.dynamicEcoActive); EEPROM.put(EEPROM_ADDR_STEAM_DELAY, profileData.dampfVerzoegerung);
    EEPROM.put(EEPROM_ADDR_FASTHEATUP_DATA, profileData.fastHeatUpAktiv);

    // Brew Control Einstellungen ins EEPROM schreiben
    EEPROM.put(EEPROM_ADDR_BREWBYTIME_ENABLED, profileData.brewByTimeEnabled); EEPROM.put(EEPROM_ADDR_BREWBYTIME_SECONDS, profileData.brewByTimeTargetSeconds);
    EEPROM.put(EEPROM_ADDR_PREINF_ENABLED, profileData.preInfusionEnabled); EEPROM.put(EEPROM_ADDR_PREINF_DUR_SEC, profileData.preInfusionDurationSeconds); EEPROM.put(EEPROM_ADDR_PREINF_PAUSE_SEC, profileData.preInfusionPauseSeconds);

    // Brew-by-Weight Einstellungen ins EEPROM schreiben
#ifdef ESP32
    EEPROM.put(EEPROM_ADDR_BREWBYWEIGHT_ENABLED, profileData.brewByWeightEnabled); EEPROM.put(EEPROM_ADDR_BREWBYWEIGHT_TARGET, profileData.brewByWeightTargetGrams); EEPROM.put(EEPROM_ADDR_BREWBYWEIGHT_OFFSET, profileData.brewByWeightOffsetGrams);
#endif

    // Systemtöne / Piezo
    EEPROM.put(EEPROM_ADDR_PIEZO_ENABLED, profileData.piezoEnabled); // <-- DIESE ZEILE HINZUFÜGEN

    // Dampfverzögerungs-Override
    EEPROM.put(EEPROM_ADDR_STEAM_DELAY_OVERRIDE_SWITCH, profileData.steamDelayOverrideBySwitch);

    EEPROM.commit();

    // --- PID-Regler sofort aktualisieren ---
    pidWasser.SetTunings(KpWasser, KiWasser, KdWasser); pidDampf.SetTunings(KpDampf, KiDampf, KdDampf);
    pidWasser.SetOutputLimits(0, windowSizeWasser); pidDampf.SetOutputLimits(0, windowSizeDampf);
    // Serial.println("  PID controllers reconfigured with loaded settings.");

    // Eco-Status zurücksetzen
    ecoModeAktiv = false; ecoModeActivatedTime = 0;
    // Serial.println("  Eco mode status reset.");
}


/************************************************************************************
 * Holt ALLE aktuellen relevanten Einstellungen aus den globalen Variablen
 * und packt sie in eine Profilstruktur (inkl. Brew-by-Weight).
 ************************************************************************************/
TemperatureProfile getCurrentSettingsAsProfile() {
    TemperatureProfile currentProfile; // Ruft den Konstruktor auf

    // Setze die aktuelle Versionsnummer in das zu speichernde Profil
    currentProfile.profileVersion = CURRENT_PROFILE_VERSION;

    // --- Globale Variablen in die Struktur kopieren ---
    // PID (/), ECO (/ECO), FastHeatUp (/Fast-Heat-Up)
    currentProfile.setpointWasser = SetpointWasser; currentProfile.setpointDampf = SetpointDampf;
    currentProfile.offsetWasser = OffsetWasser; currentProfile.offsetDampf = OffsetDampf;
    currentProfile.kpWasser = KpWasser; currentProfile.kiWasser = KiWasser; currentProfile.kdWasser = KdWasser;
    currentProfile.kpDampf = KpDampf; currentProfile.kiDampf = KiDampf; currentProfile.kdDampf = KdDampf;
    currentProfile.boostWasserActive = boostWasserActive; currentProfile.boostDampfActive = boostDampfActive;
    currentProfile.preventHeatAboveSetpointWasser = preventHeatAboveSetpointWasser; currentProfile.preventHeatAboveSetpointDampf = preventHeatAboveSetpointDampf;
    currentProfile.windowSizeWasser = windowSizeWasser; currentProfile.windowSizeDampf = windowSizeDampf;
    currentProfile.maxTempWasser = maxTempWasser; currentProfile.maxTempDampf = maxTempDampf;
    currentProfile.ecoModeMinutes = ecoModeMinutes; currentProfile.ecoModeTempWasser = ecoModeTempWasser; currentProfile.ecoModeTempDampf = ecoModeTempDampf;
    currentProfile.dynamicEcoActive = dynamicEcoActive; currentProfile.dampfVerzoegerung = dampfVerzoegerung;
    currentProfile.steamDelayOverrideBySwitch = steamDelayOverrideBySwitchEnabled;
    currentProfile.fastHeatUpAktiv = fastHeatUpAktiv;

    // PID Tuning (/PID-Tuning)
    currentProfile.tuningStepWasser = tuningStepWasser; currentProfile.tuningNoiseWasser = tuningNoiseWasser; currentProfile.tuningStartValueWasser = tuningStartValueWasser; currentProfile.tuningLookBackWasser = tuningLookBackWasser;
    currentProfile.tuningStepDampf = tuningStepDampf; currentProfile.tuningNoiseDampf = tuningNoiseDampf; currentProfile.tuningStartValueDampf = tuningStartValueDampf; currentProfile.tuningLookBackDampf = tuningLookBackDampf;

    // Brew Control Einstellungen in Struktur kopieren
    currentProfile.brewByTimeEnabled = brewByTimeEnabled; currentProfile.brewByTimeTargetSeconds = brewByTimeTargetSeconds;
    currentProfile.preInfusionEnabled = preInfusionEnabled; currentProfile.preInfusionDurationSeconds = preInfusionDurationSeconds; currentProfile.preInfusionPauseSeconds = preInfusionPauseSeconds;

    // Brew-by-Weight Einstellungen in Struktur kopieren
#ifdef ESP32
    currentProfile.brewByWeightEnabled = brewByWeightEnabled; currentProfile.brewByWeightTargetGrams = brewByWeightTargetGrams; currentProfile.brewByWeightOffsetGrams = brewByWeightOffsetGrams;
#else
    // Setze auf Defaults für ESP8266, falls das Profil dort erstellt wird (unwahrscheinlich, aber sicher)
    currentProfile.brewByWeightEnabled = defaultBrewByWeightEnabled;
    currentProfile.brewByWeightTargetGrams = defaultBrewByWeightTargetGrams;
    currentProfile.brewByWeightOffsetGrams = defaultBrewByWeightOffsetGrams;
#endif

    // Piezo Status in Struktur kopieren
    currentProfile.piezoEnabled = piezoEnabled;

    // Profilname bleibt vorerst leer, wird in handleSaveProfile gesetzt
    strncpy(currentProfile.profileName, "", sizeof(currentProfile.profileName));

    return currentProfile;
}

// Löscht eine Profildatei (.prof)
bool deleteProfile(const String& profileName) {
    String sanitizedName = sanitizeProfileName(profileName); // Verwende den Namen aus der Liste
     if (sanitizedName.length() == 0) return false;

    // Dateiendung .prof verwenden
    String filePath = "/Profile/" + sanitizedName + ".prof"; // *** GEÄNDERT ZU .prof ***
    // Serial.printf("Deleting profile: %s\n", filePath.c_str());

    if (LittleFS.exists(filePath)) {
        if (LittleFS.remove(filePath)) {
            // Serial.println("  Profile deleted successfully.");
            return true;
        } else {
            // Serial.println("  ERROR: Failed to delete profile file.");
            return false;
        }
    } else {
        // Serial.println("  ERROR: Profile file not found, cannot delete.");
        return false;
    }
}

// *** ENDE PROFIL-HILFS-FUNKTIONEN ***

    // --- PROGMEM Chunks für die neue Profil-Seite ---
    static const char profilesPageHead[] PROGMEM = R"rawliteral(
    <!DOCTYPE html><html><head><title>Profile</title><meta name='theme-color' content='#000000'><meta name='viewport' content='width=device-width, initial-scale=1.0'>)rawliteral";
    static const char profilesPageStyle[] PROGMEM = R"rawliteral(
    <style>
        /* Zusätzliche Styles für die Profilliste und Speicher-Form */
        .profiles-form {
          background: none;
          backdrop-filter: none;
          border-radius: 12px;
          padding: 0px;
          margin: 0px;
          width: 90%;
          max-width: 600px;
          box-shadow: none;
          color: var(--text-color);
        }

        .profile-list, .save-profile-form {
            max-width: 650px;
            margin-left: auto;
            margin-right: auto;
            background: rgba(255, 255, 255, var(--card-bg-opacity));
            backdrop-filter: blur(8px);
            border-radius: 12px;
            padding: 20px 25px; /* Haupt-Innenabstand der Karten */
            box-shadow: 0 8px 24px rgba(0, 0, 0, 0.3);
        }
        .profile-list {
            list-style: none;
            padding-top: 15px;
            padding-bottom: 15px;
            margin-top: 25px;
            margin-bottom: 0;
        }
        .save-profile-form {
            margin-top: 15px;
            margin-bottom: 25px;
        }

        .profile-list h3, .save-profile-form h3 {
            margin-top: 0;
            margin-bottom: 18px;
            font-weight: 600;
            padding-bottom: 8px;
            border-bottom: 1px solid rgba(255, 255, 255, 0.3);
        }
        .profile-list li {
            display: flex;
            justify-content: space-between;
            align-items: center;
            background: rgba(0,0,0, 0.2);
            padding: 8px 15px; /* <<< Horizontalen Innenabstand reduziert (war 18px) */
            margin-bottom: 10px;
            border-radius: 8px;
            border: 1px solid rgba(255, 255, 255, 0.15);
            transition: background-color 0.2s ease, transform 0.2s ease;
        }
        .profile-list li:hover {
            background: rgba(255, 255, 255, 0.15);
            transform: scale(1.015);
        }
        .profile-list li.no-profiles i {
            color: #aaa;
            display: block;
            width: 100%;
            text-align: center;
            font-size: 0.95em;
        }
        .profile-list li.no-profiles:hover {
            background: rgba(0,0,0, 0.2);
            transform: none;
        }

        .profile-list span {
            font-weight: 500;
            font-size: 1.05em;
            margin-right: 10px; /* <<< Abstand zum Button-Bereich leicht reduziert */
            flex-grow: 1;
            color: #ffffff;
            overflow: hidden;
            text-overflow: ellipsis;
            white-space: nowrap;
        }
        .profile-actions {
            display: flex;
            align-items: center;
            flex-shrink: 0;
            gap: 5px; /* <<< Abstand zwischen Buttons weiter reduziert (war 6px) */
        }
        .profile-actions form {
            display: inline-block;
        }
        /* Buttons in der Liste werden kompakter */
        .profile-actions button {
            padding: 5px 10px; /* <<< Button Padding weiter reduziert */
            font-size: 0.85em;
            cursor: pointer;
            border-radius: 18px;
            border: none;
            font-weight: 600;
            transition: all 0.2s ease;
            /* min-width entfernt, damit Buttons schmaler werden können */
            text-align: center;
            line-height: 1.2;

            border: none;
            border-radius: 30px;
            padding: 10px 20px;
            margin-top: 0px;
            width: auto;
            min-width: 100px;
            max-width: 200px;
            cursor: pointer;
            font-weight: 600;
            box-shadow: none;
            transition: all 0.3s ease;
        }
        .profile-actions button:hover {
            opacity: 0.85;
            transform: translateY(-1px);
        }
        .profile-actions .load-button { background-color: #28a745; color: white; }
        .profile-actions .delete-button { background-color: #dc3545; color: white; }

        /* Speichern-Button bleibt unverändert */
        #save-profile-button {
            background-color: var(--accent-color);
            color: #000000;
            width: 100%;
            max-width: 300px;
            margin-left: auto;
            margin-right: auto;
            padding: 8px 15px;
            font-size: 0.9em;
            border-radius: 20px;
            border: none; /* Sicherstellen, dass kein Rand da ist */
            font-weight: 600; /* Fettschrift */
            transition: all 0.2s ease; /* Übergang für Hover */
            cursor: pointer; /* Mauszeiger */
            line-height: 1.2; /* Vertikale Textzentrierung verbessern */
        }
        #save-profile-button:hover {
            opacity: 0.85;
            transform: translateY(-1px);
        }

        /* Style für das Eingabefeld */
        .save-profile-form input[type="text"] {
            width: calc(100% - 22px);
            display: block;
            margin-bottom: 10px;
            background: rgba(0, 0, 0, 0.4);
            color: var(--text-color);
            border: 1px solid #555;
            border-radius: 8px;
            padding: 10px;
            box-shadow: inset 0 4px 8px rgba(0, 0, 0, 0.2);
            transition: background 0.2s ease, border-color 0.2s ease, box-shadow 0.2s ease;
        }
        .save-profile-form input[type="text"]:focus {
            outline: none;
            background: rgba(0, 0, 0, 0.6);
            border-color: var(--accent-color, #d89904);
            box-shadow: 0 0 8px rgba(216, 153, 4, 0.3);
        }

        .status-message {
          text-align: center;
          padding: 8px 10px; /* Angepasst an Vorlage */
          margin: 15px auto; /* Behält oberen/unteren Abstand und Zentrierung */
          max-width: 600px; /* Angepasst an Vorlage */
          border-radius: 8px;
          color: white; /* Beibehalten für guten Kontrast */
          font-weight: 500; /* Angepasst an Vorlage (war bold) */
          font-size: 0.9em; /* Angepasst an Vorlage */
          border: 1px solid transparent; /* Basis-Rahmen, Farbe wird unten überschrieben */
          background-color: transparent; /* Basis-Hintergrund, wird unten überschrieben */
        }

        /* Spezifische Styles für Erfolgsmeldungen */
        .status-success {
          background-color: rgba(40, 167, 69, 0.15); /* Dezentes Grün-BG (RGB von Bootstrap .btn-success) */
          border-color: rgba(40, 167, 69, 0.3); /* Etwas sichtbarer grüner Rahmen */
          color: #62c34b
        }

        /* Spezifische Styles für Fehlermeldungen */
        .status-error {
          background-color: rgba(220, 53, 69, 0.15); /* Dezentes Rot-BG (RGB von Bootstrap .btn-danger) */
          border-color: rgba(220, 53, 69, 0.3); /* Etwas sichtbarer roter Rahmen */
          color: #f8d7da
        }

        @media (max-width: 600px) {
            .profile-list li { flex-direction: column; align-items: stretch; padding: 10px 15px; /* Padding für Mobile anpassen */}
            .profile-list span { margin-bottom: 10px; white-space: normal; margin-right: 0;}
            .profile-actions { margin-top: 10px; width: 100%; display: flex; justify-content: flex-end; gap: 8px; /* Etwas mehr Lücke auf Mobile */}
            .profile-actions button { padding: 6px 10px; font-size: 0.8em; /* Ggf. noch kleiner auf Mobile */ }
            .save-profile-form input[type="text"] { width: calc(100% - 22px); }
            #save-profile-button { max-width: none; }
        }

    </style></head>)rawliteral"; // Schließt </head>
    static const char profilesPageBodyStart[] PROGMEM = R"rawliteral(
    <body><h1>Temperaturprofile</h1>)rawliteral";
    static const char profilesPageStatusPlaceholder[] PROGMEM = R"rawliteral({STATUS_MESSAGE_PLACEHOLDER})rawliteral";
    static const char profilesPageListStart[] PROGMEM = R"rawliteral(
    <div class="profile-list"><h3>Gespeicherte Profile</h3><ul>)rawliteral";
    static const char profilesPageListItemStart[] PROGMEM = R"rawliteral(<li><span>)rawliteral"; // Nach <span>
    static const char profilesPageListItemEnd[] PROGMEM = R"rawliteral(</span><div class="profile-actions">
    <form action="/loadProfile" method="POST" class="profiles-form"><input type="hidden" name="profileName" value="{PROFILE_NAME}"><button type="submit" class="load-button">Laden</button></form>
    <form action="/deleteProfile" method="POST" onsubmit="return confirm('Profil \'{PROFILE_NAME}\' wirklich entfernen?');" class="profiles-form"><input type="hidden" name="profileName" value="{PROFILE_NAME}"><button type="submit" class="delete-button">Entfernen</button></form>
    </div></li>)rawliteral"; // Ersetzt {PROFILE_NAME} mehrfach
    static const char profilesPageNoProfiles[] PROGMEM = R"rawliteral(<li class="no-profiles"><i>Keine Profile gespeichert.</i></li>)rawliteral";
    static const char profilesPageListEnd[] PROGMEM = R"rawliteral(</ul></div>)rawliteral";
    static const char profilesPageSaveForm[] PROGMEM = R"rawliteral(
    <form action="/saveProfile" method="POST" class="save-profile-form">
        <h3>Aktuelle Einstellungen speichern</h3>
        <label for="newProfileName">Profilname:</label>
        <input type="text" id="newProfileName" name="profileName" placeholder="Neuer Profilname" required maxlength="30">
        <p style="font-size:0.9em; ">(Wird als neues Profil gespeichert oder &uuml;berschreibt ein bestehendes mit gleichem Namen)</p>
        <button type="submit" id="save-profile-button">Aktuelle Einstellungen speichern</button>
    </form>
    </body></html>)rawliteral";
    // ----------- Ende PROGMEM Chunks -----------

// Zeigt die Profil-Verwaltungsseite an
void handleProfilesPage(AsyncWebServerRequest *request) {
    std::vector<String> profiles = listProfiles(); // Hole die Liste der Profile

    AsyncResponseStream *response = request->beginResponseStream("text/html");
    response->print(FPSTR(profilesPageHead));
    response->print(FPSTR(commonStyle)); // Globale Styles
    response->print(FPSTR(profilesPageStyle)); // Seiten-spezifische Styles und </head>
    response->print(FPSTR(commonNav)); // Navigation
    response->print(FPSTR(profilesPageBodyStart)); // <body> und <h1>

    // Statusmeldung einfügen (falls vorhanden)
    if (profileStatusMessage.length() > 0) {
        String statusDiv = "<div class=\"status-message ";
        // Annahme: Nachrichten, die mit "FEHLER" beginnen, sind Fehler
        if (profileStatusMessage.startsWith("FEHLER")) {
            statusDiv += "status-error";
        } else {
            statusDiv += "status-success";
        }
        statusDiv += "\">" + profileStatusMessage + "</div>";
        response->print(statusDiv);
        profileStatusMessage = ""; // Nachricht entfernen, nachdem sie angezeigt wurde
    }

    // Profilliste generieren
    response->print(FPSTR(profilesPageListStart)); // <ul>
    if (profiles.empty()) {
        response->print(FPSTR(profilesPageNoProfiles));
    } else {
        for (const String& name : profiles) {
            response->print(FPSTR(profilesPageListItemStart)); // <li><span>
            response->print(name); // Profilnamen einfügen

            String itemEnd = FPSTR(profilesPageListItemEnd);
            itemEnd.replace("{PROFILE_NAME}", name); // Platzhalter im Template ersetzen
            response->print(itemEnd); // Rest des <li> Elements
            yield(); // Bei vielen Profilen
        }
    }
    response->print(FPSTR(profilesPageListEnd)); // </ul>

    // Speicherformular
    response->print(FPSTR(profilesPageSaveForm)); // Formular und </body></html>

    request->send(response);
}

// Lädt ein ausgewähltes Profil und wendet es an
void handleLoadProfile(AsyncWebServerRequest *request) {
    if (request->hasArg("profileName")) {
        String profileToLoad = request->arg("profileName");
        TemperatureProfile loadedData;

        if (loadProfile(profileToLoad, loadedData)) {
            applyProfileSettings(loadedData);
            // *** KORRIGIERT: Erfolgsmeldung hier ***
            profileStatusMessage = "Profil '" + String(loadedData.profileName) + "' erfolgreich geladen und angewendet.";
        } else {
             // *** KORRIGIERT: Fehlermeldung hier ***
            profileStatusMessage = "FEHLER: Profil '" + profileToLoad + "' konnte nicht geladen werden, da es eventuell zu dieser Firmware-Version nicht mehr kompatibel ist.";
        }
    } else {
        profileStatusMessage = "FEHLER: Kein Profilname zum Laden &uuml;bermittelt.";
    }
    // Redirect zurück zur Profilseite, um die Nachricht anzuzeigen
    AsyncWebServerResponse *response = request->beginResponse(303);
    response->addHeader("Location", "/Profile");
    request->send(response);
}

// Speichert die aktuellen Einstellungen unter einem neuen Namen
void handleSaveProfile(AsyncWebServerRequest *request) {
    if (request->hasArg("profileName")) {
        String newName = request->arg("profileName");
        String sanitizedName = sanitizeProfileName(newName);

        if (sanitizedName.length() == 0) {
             profileStatusMessage = "FEHLER: Ung&uuml;ltiger oder leerer Profilname angegeben.";
        } else {
            TemperatureProfile currentData = getCurrentSettingsAsProfile();
            // Setze den vom Benutzer gewünschten (originalen) Namen in die Struktur
            strncpy(currentData.profileName, newName.c_str(), sizeof(currentData.profileName) - 1);
            currentData.profileName[sizeof(currentData.profileName) - 1] = '\0';

            // Speichere unter dem bereinigten Dateinamen
            if (saveProfile(sanitizedName, currentData)) {
                 profileStatusMessage = "Aktuelle Einstellungen erfolgreich als Profil '" + String(currentData.profileName) + "' gespeichert.";
            } else {
                 profileStatusMessage = "FEHLER: Einstellungen konnten nicht als Profil '" + String(currentData.profileName) + "' gespeichert werden.";
            }
        }
    } else {
        profileStatusMessage = "FEHLER: Kein Profilname zum Speichern übermittelt.";
    }
    // Redirect zurück zur Profilseite
    AsyncWebServerResponse *response = request->beginResponse(303);
    response->addHeader("Location", "/Profile");
    request->send(response);
}

// Löscht ein ausgewähltes Profil
void handleDeleteProfile(AsyncWebServerRequest *request) {
    if (request->hasArg("profileName")) {
        String profileToDelete = request->arg("profileName");

        if (deleteProfile(profileToDelete)) {
             profileStatusMessage = "Profil '" + profileToDelete + "' erfolgreich entfernt.";
        } else {
             profileStatusMessage = "FEHLER: Profil '" + profileToDelete + "' konnte nicht entfernt werden.";
        }
    } else {
        profileStatusMessage = "FEHLER: Kein Profilname zum entfernen übermittelt.";
    }
    // Redirect zurück zur Profilseite
    AsyncWebServerResponse *response = request->beginResponse(303);
    response->addHeader("Location", "/Profile");
    request->send(response);
}

// *** ENDE WEB UI HANDLER ***

/************************************************************************************
 * Handler für die Dateimanager-Seite (/Dateimanager) - Heap-Optimiert
 * - Plattformabhängige FS-API berücksichtigt
 * - Unterstützt jetzt Verzeichnisnavigation
 ************************************************************************************/

// --- PROGMEM Chunks für handleFileManager ---
static const char fileMgrHead[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html lang='de'><head><meta charset='UTF-8'><title>Dateimanager</title>
<style>
body { font-family: sans-serif; background-color: #f0f0f0; margin: 15px; }
h1, h2 { color: #333; }
ul { list-style: none; padding: 0; }
li { margin-bottom: 8px; background: #fff; padding: 8px 12px; border: 1px solid #ccc; border-radius: 4px; display: flex; align-items: center; justify-content: space-between; flex-wrap: wrap; /* Für kleine Bildschirme */ }
li span { flex-grow: 1; margin-right: 10px; word-break: break-all; /* Lange Namen umbrechen */ }
.file-info { display: inline-block; min-width: 80px; text-align: right; color: #666; font-size: 0.9em; margin-left: 10px; } /* Dateigröße */
.item-name { font-weight: bold; }
.dir-name { color: #0056b3; } /* Blaue Farbe für Ordner */
.file-actions { display: flex; align-items: center; gap: 5px; flex-shrink: 0; /* Verhindert Schrumpfen */ margin-left:auto; /* Schiebt Aktionen nach rechts */ padding-left: 10px; /* Etwas Abstand */}
a { text-decoration: none; color: #007bff; }
a:hover { text-decoration: underline; }
a.button-like, input[type='submit'], button.button-like { text-decoration: none; background: #eee; padding: 4px 8px; border: 1px solid #ccc; border-radius: 3px; color: #333; cursor: pointer; font-size: 0.9em; display: inline-block; /* Wichtig für korrekte Darstellung */}
a.button-like:hover, input[type='submit']:hover, button.button-like:hover { background: #ddd; }
form { margin: 0; padding: 0; border: none; display: inline; } /* Forms in Aktionen */
hr { margin: 20px 0; border: 0; border-top: 1px solid #ccc; }
.upload-form, .create-dir-form { margin-top: 20px; background: #fff; padding: 15px; border: 1px solid #ccc; border-radius: 4px; }
.upload-form input[type='file'] { margin-bottom: 10px; }
.status-message { padding: 10px; margin-bottom: 15px; border-radius: 4px; color: #fff; }
.status-error { background-color: #dc3545; }
.status-success { background-color: #28a745; }
/* Responsive Anpassungen */
@media (max-width: 600px) {
  li { flex-direction: column; align-items: flex-start; }
  .file-actions { width: 100%; margin-top: 8px; margin-left: 0; justify-content: flex-end; /* Buttons nach rechts */}
  .file-info { text-align: left; margin-left: 0; margin-top: 4px;}
  li span { margin-right: 0; }
}
</style>
</head><body>
)rawliteral"; // Ende des Head+Style Blocks, vor <body>

// Platzhalter für Pfad und Fehlermeldung
static const char fileMgrTitle[] PROGMEM = R"rawliteral(<h1>Dateimanager</h1><h2>Aktueller Pfad: {CURRENT_PATH}</h2>{STATUS_MESSAGE})rawliteral";

// List Start
static const char fileMgrListStart[] PROGMEM = "<ul>";

// List Item: Zurück ( .. )
static const char fileMgrListItemUp[] PROGMEM = R"rawliteral(<li><a href='/Dateimanager?path={PARENT_PATH}' class='item-name dir-name'>[ .. Zur&uuml;ck]</a></li>)rawliteral";

// List Item: Verzeichnis
static const char fileMgrListItemDir[] PROGMEM = R"rawliteral(<li><a href='/Dateimanager?path={DIR_PATH}' class='item-name dir-name'>[ {DIR_NAME} ]</a><div class='file-actions'><form action='/Datei-Entfernen' method='GET' onsubmit='return confirm(\"Sicher, dass das Verzeichnis {DIR_NAME} und sein Inhalt gelöscht werden soll?\");'><input type='hidden' name='name' value='{DIR_PATH}'><input type='hidden' name='isDir' value='1'><input type='submit' value='Entfernen'></form></div></li>)rawliteral"; // DIR_PATH ist der volle Pfad, DIR_NAME nur der Name

// List Item: Datei (wie bisher, aber mit vollem Pfad bei name=...)
static const char fileMgrListItemFileStart[] PROGMEM = R"rawliteral(<li><span class='item-name'>)rawliteral"; // Start + Span für Name
static const char fileMgrListItemFileMid1[] PROGMEM = R"rawliteral(</span><span class='file-info'>)rawliteral"; // Nach Name, vor Größe
static const char fileMgrListItemFileMid2[] PROGMEM = R"rawliteral( Bytes</span><div class='file-actions'><a href='/Datei-Download?name=)rawliteral"; // Nach Größe, bis vor Download-Pfad
static const char fileMgrListItemFileMid3[] PROGMEM = R"rawliteral(' class='button-like' download>Download</a><form action='/Datei-Entfernen' method='GET' onsubmit='return confirm(\"Sicher, dass die Datei {FILE_NAME} gelöscht werden soll?\");'><input type='hidden' name='name' value=')rawliteral"; // Nach Download-Pfad, bis vor Delete-Pfad
static const char fileMgrListItemFileEnd[] PROGMEM = R"rawliteral('><input type='submit' value='Entfernen'></form></div></li>)rawliteral"; // Nach Delete-Pfad, bis Ende <li>

// Meldungen für leere Liste oder Fehler
static const char fileMgrNoFiles[] PROGMEM = R"rawliteral(<li><i>(Verzeichnis ist leer)</i></li>)rawliteral";
static const char fileMgrListError[] PROGMEM = R"rawliteral(<li><i>Fehler beim Auflisten des Verzeichnisses.</i></li>)rawliteral";
static const char fileMgrOpenError[] PROGMEM = R"rawliteral(<li><i>Fehler beim Öffnen des Verzeichnisses: Pfad nicht gefunden oder keine Berechtigung.</i></li>)rawliteral";

// Ende der Liste + Trennlinie
static const char fileMgrListEnd[] PROGMEM = R"rawliteral(</ul><hr>)rawliteral";

// Upload Formular (mit verstecktem Pfad)
static const char fileMgrUploadForm[] PROGMEM = R"rawliteral(
<div class='upload-form'>
<h2>Datei hochladen (in aktuellen Pfad)</h2>
<form method='POST' action='/Datei-Upload' enctype='multipart/form-data'>
<input type='hidden' name='path' value='{CURRENT_PATH}'>
<input type='file' name='upload' required><br>
<input type='submit' value='Datei Hochladen'>
</form>
</div>
)rawliteral";

// Verzeichnis erstellen Formular
static const char fileMgrCreateDirForm[] PROGMEM = R"rawliteral(
<div class='create-dir-form'>
<h2>Verzeichnis erstellen (im aktuellen Pfad)</h2>
<form method='POST' action='/Verzeichnis-Erstellen'>
<input type='hidden' name='basePath' value='{CURRENT_PATH}'>
<input type='text' name='dirName' placeholder='Neuer Verzeichnisname' required pattern='^[a-zA-Z0-9_.-]+$' title='Nur Buchstaben, Zahlen, _, -, . erlaubt'>
<input type='submit' value='Verzeichnis erstellen'>
</form>
</div>
)rawliteral";


// Dateisystem Info (wie bisher)
static const char fileMgrInfoStart[] PROGMEM = R"rawliteral(<hr><h2>Info</h2>)rawliteral";
static const char fileMgrInfoTotal[] PROGMEM = R"rawliteral(Gesamtspeicher: )rawliteral";
static const char fileMgrInfoUsed[] PROGMEM = R"rawliteral( Bytes<br>Belegt: )rawliteral";
static const char fileMgrInfoFree[] PROGMEM = R"rawliteral( Bytes<br>Frei: )rawliteral";
static const char fileMgrInfoEnd[] PROGMEM = R"rawliteral( Bytes<br>)rawliteral";
static const char fileMgrInfoError[] PROGMEM = R"rawliteral(Fehler beim Abrufen der Dateisystem-Informationen.<br>)rawliteral";

// Body Ende
static const char fileMgrBodyEnd[] PROGMEM = R"rawliteral(</body></html>)rawliteral";

// Globale Variable für Statusmeldungen im Dateimanager
String fileManagerStatusMessage = "";
String fileManagerStatusClass = ""; // "status-success" oder "status-error"

// Hilfsfunktion zum rekursiven Löschen von Verzeichnissen
bool removeDirectoryRecursive(const String& path) {
    // Serial.printf("Attempting to recursively remove directory: %s\n", path.c_str());
    bool success = true;

#ifdef ESP32
    File root = LittleFS.open(path);
    if (!root) {
        // Serial.printf("  ERROR: Failed to open directory %s for removal.\n", path.c_str());
        return false;
    }
    if (!root.isDirectory()) {
        // Serial.printf("  ERROR: %s is not a directory.\n", path.c_str());
        root.close();
        return false; // Oder sollte eine Datei einfach gelöscht werden? Hier nur Verzeichnisse.
    }

    File file = root.openNextFile();
    while (file) {
        String entryPath = path + "/" + file.name(); // Korrekten Pfad bilden
         // Korrektur für ESP32: file.name() liefert nur den Namen, nicht den vollen Pfad relativ zum FS-Root.
         // Wir brauchen den vollen Pfad für die Rekursion und das Löschen.
        entryPath = file.path(); // Besser: ESP32 gibt vollen Pfad zurück.

        if (file.isDirectory()) {
            // Serial.printf("  Recursing into directory: %s\n", entryPath.c_str());
            if (!removeDirectoryRecursive(entryPath)) {
                // Serial.printf("  ERROR: Failed to remove subdirectory %s\n", entryPath.c_str());
                success = false; // Fehler beim Löschen des Unterverzeichnisses
                // Man könnte hier entscheiden abzubrechen oder weiterzumachen
            }
        } else {
            // Serial.printf("  Deleting file: %s\n", entryPath.c_str());
            if (!LittleFS.remove(entryPath)) {
                // Serial.printf("  ERROR: Failed to remove file %s\n", entryPath.c_str());
                success = false; // Fehler beim Löschen der Datei
            }
        }
        file.close(); // Wichtig: Datei schließen
        file = root.openNextFile();
        yield(); // Luft holen
    }
    root.close(); // Root des zu löschenden Verzeichnisses schließen

    // Nachdem der Inhalt (hoffentlich) weg ist, das leere Verzeichnis löschen
    if (success) { // Nur versuchen, wenn bisher alles geklappt hat
        // Serial.printf("  Attempting to remove now (hopefully) empty directory: %s\n", path.c_str());
        if (!LittleFS.rmdir(path)) { // rmdir für Verzeichnisse auf ESP32
            // Serial.printf("  ERROR: Failed to remove directory %s itself.\n", path.c_str());
            success = false;
        // } else {
        //      Serial.printf("  Successfully removed directory %s.\n", path.c_str());
        }
    // } else {
    //      Serial.printf("  Skipping removal of directory %s due to previous errors.\n", path.c_str());
    }

#else // ESP8266
    Dir dir = LittleFS.openDir(path);
    while (dir.next()) {
        String entryName = dir.fileName();
        String entryPath = path + "/" + entryName;
        if (entryName == "." || entryName == "..") continue; // Ignorieren

        if (dir.isDirectory()) {
            //  Serial.printf("  Recursing into directory: %s\n", entryPath.c_str());
            if (!removeDirectoryRecursive(entryPath)) {
                //  Serial.printf("  ERROR: Failed to remove subdirectory %s\n", entryPath.c_str());
                success = false;
            }
        } else {
            // Serial.printf("  Deleting file: %s\n", entryPath.c_str());
            if (!LittleFS.remove(entryPath)) {
                //  Serial.printf("  ERROR: Failed to remove file %s\n", entryPath.c_str());
                success = false;
            }
        }
         yield(); // Luft holen
    }
     // dir schließt automatisch

    if (success) {
        // Serial.printf("  Attempting to remove now (hopefully) empty directory: %s\n", path.c_str());
        if (!LittleFS.rmdir(path)) { // rmdir auch für ESP8266
            // Serial.printf("  ERROR: Failed to remove directory %s itself.\n", path.c_str());
            success = false;
        // } else {
        //     Serial.printf("  Successfully removed directory %s.\n", path.c_str());
        }
    // } else {
    //     Serial.printf("  Skipping removal of directory %s due to previous errors.\n", path.c_str());
    }
#endif

    return success;
}


/************************************************************************************
 * Handler für die Dateimanager-Seite (/Dateimanager) - Heap-Optimiert
 * - Plattformabhängige FS-API berücksichtigt
 * - Unterstützt jetzt Verzeichnisnavigation
 ************************************************************************************/
 
void handleFileManager(AsyncWebServerRequest *request) {
    // Serial.println("Anfrage für /Dateimanager");
    char buffer[20]; // Puffer für Zahlen (Größe, FS Info) - HIER deklariert für die ganze Funktion

    // Aktuellen Pfad aus URL holen, Standard ist "/"
    String currentPath = request->hasArg("path") ? request->arg("path") : "/";

    // Pfad bereinigen und validieren
    if (!currentPath.startsWith("/")) {
        currentPath = "/" + currentPath;
    }
    // Einfache Normalisierung (doppelte Slashes entfernen)
    currentPath.replace("//", "/");
    // Entferne abschließenden Slash, außer bei Root "/"
    if (currentPath.length() > 1 && currentPath.endsWith("/")) {
        currentPath.remove(currentPath.length() - 1);
    }
    // Serial.printf("Aktueller Pfad: %s\n", currentPath.c_str());

    // --- Antwort-Stream erstellen ---
    AsyncResponseStream *response = request->beginResponseStream("text/html; charset=utf-8");

    // --- Header und Style senden ---
    response->print(FPSTR(fileMgrHead)); // Enthält <html>, <head>, <style>, </head>, <body>

    // --- Titel und Statusmeldung ---
    String titleHtml = FPSTR(fileMgrTitle);
    titleHtml.replace("{CURRENT_PATH}", currentPath); // Zeige aktuellen Pfad
    // Füge Statusmeldung ein, falls vorhanden
    if (fileManagerStatusMessage.length() > 0) {
        String statusHtml = "<div class='status-message " + fileManagerStatusClass + "'>" + fileManagerStatusMessage + "</div>";
        titleHtml.replace("{STATUS_MESSAGE}", statusHtml);
        fileManagerStatusMessage = ""; // Nachricht zurücksetzen nach Anzeige
        fileManagerStatusClass = "";
    } else {
        titleHtml.replace("{STATUS_MESSAGE}", ""); // Kein Status -> Platzhalter entfernen
    }
    response->print(titleHtml);
    yield();

    // --- Dateiliste starten ---
    response->print(FPSTR(fileMgrListStart)); // <ul>

    // --- "Zurück"-Link hinzufügen, wenn nicht im Root ---
    if (currentPath != "/") {
        String parentPath = "/"; // Standard-Elternpfad ist Root
        int lastSlash = currentPath.lastIndexOf('/');
        if (lastSlash > 0) { // Wenn Slash nicht am Anfang ist (z.B. bei /subdir)
            parentPath = currentPath.substring(0, lastSlash);
        }
        // parentPath ist "/" wenn currentPath z.B. "/subdir" war
        // parentPath bleibt "/", wenn currentPath "/" ist (wird aber oben schon abgefangen)

        String upLink = FPSTR(fileMgrListItemUp);
        upLink.replace("{PARENT_PATH}", parentPath);
        response->print(upLink);
        yield();
    }

    // --- Verzeichnisinhalt auflisten ---
    int entryCount = 0;      // Zählt gefundene Dateien/Verzeichnisse (außer . und ..)
    bool errorOccurred = false; // Flag für Fehler beim Öffnen (primär für ESP32 relevant)

#ifdef ESP32
    File root = LittleFS.open(currentPath);
    if (!root) {
        response->print(FPSTR(fileMgrOpenError));
        errorOccurred = true;
    } else if (!root.isDirectory()) {
        response->print(F("<li>Fehler: Angegebener Pfad ist keine Verzeichnis.</li>"));
        errorOccurred = true;
        root.close();
    } else {
        File entry = root.openNextFile();
        while (entry) {
            entryCount++;
            String entryName = entry.name(); // ESP32: Gibt nur den Namen zurück
            String entryFullPath = entry.path(); // ESP32: Gibt vollen Pfad zurück

            if (entry.isDirectory()) {
                // Verzeichnis-Eintrag
                String dirItem = FPSTR(fileMgrListItemDir);
                // Ersetze Platzhalter im Template
                dirItem.replace("{DIR_PATH}", entryFullPath);
                dirItem.replace("{DIR_NAME}", entryName);
                dirItem.replace("{DIR_NAME}", entryName); // Erneut für Confirm-Dialog
                response->print(dirItem);
            } else {
                // Datei-Eintrag
                size_t fileSize = entry.size();
                response->print(FPSTR(fileMgrListItemFileStart)); // <li><span class='item-name'>
                response->print(entryName);                  // Dateiname
                response->print(FPSTR(fileMgrListItemFileMid1));  // </span><span class='file-info'>
                snprintf(buffer, sizeof(buffer), "%llu", (uint64_t)fileSize); // ESP32 size_t ist oft 64bit? Sicherer mit ll/u64
                response->print(buffer);                     // Dateigröße
                response->print(FPSTR(fileMgrListItemFileMid2));  // Bytes</span><div ... href='...?name=
                response->print(entryFullPath);              // Voller Pfad für Download
                String fileItemEnd = FPSTR(fileMgrListItemFileMid3);
                fileItemEnd.replace("{FILE_NAME}", entryName);  // Name für Confirm-Dialog
                response->print(fileItemEnd);                // ' class...>...<input value='
                response->print(entryFullPath);              // Voller Pfad für Delete
                response->print(FPSTR(fileMgrListItemFileEnd));   // '><input ...></div></li>
            }
            entry.close(); // Wichtig: Eintrag schließen
            entry = root.openNextFile();
            yield();
        }
        root.close(); // Wichtig: Verzeichnis schließen
    }
#else // ESP8266
    // Für ESP8266: KEINE explizite Prüfung (!dir) hier, verlasse dich auf dir.next()
    Dir dir = LittleFS.openDir(currentPath);

    while (dir.next()) { // Die Schleife wird nicht betreten, wenn openDir fehlschlug oder das Verzeichnis leer ist
        String entryName = dir.fileName();
        // Kleiner Fix: Verhindern, dass "." und ".." aufgelistet werden und gezählt werden
        if (entryName == "." || entryName == "..") {
             continue; // Nicht auflisten
        }
        entryCount++; // Zähle nur gültige Einträge

        String entryFullPath = currentPath;
        if (currentPath == "/") {
            entryFullPath += entryName; // Root: /dateiname
        } else {
            entryFullPath += "/" + entryName; // Subdir: /subdir/dateiname
        }

        if (dir.isDirectory()) {
            // Verzeichnis-Eintrag
            String dirItem = FPSTR(fileMgrListItemDir);
             // Ersetze Platzhalter im Template
            dirItem.replace("{DIR_PATH}", entryFullPath);
            dirItem.replace("{DIR_NAME}", entryName);
             dirItem.replace("{DIR_NAME}", entryName); // Erneut für Confirm-Dialog
            response->print(dirItem);
        } else {
            // Datei-Eintrag
            size_t fileSize = dir.fileSize();
            response->print(FPSTR(fileMgrListItemFileStart)); // <li><span class='item-name'>
            response->print(entryName);                  // Dateiname
            response->print(FPSTR(fileMgrListItemFileMid1));  // </span><span class='file-info'>
            snprintf(buffer, sizeof(buffer), "%u", fileSize); // %u für unsigned size_t auf ESP8266
            response->print(buffer);                     // Dateigröße
            response->print(FPSTR(fileMgrListItemFileMid2));  // Bytes</span><div ... href='...?name=
            response->print(entryFullPath);              // Voller Pfad für Download
             String fileItemEnd = FPSTR(fileMgrListItemFileMid3);
             fileItemEnd.replace("{FILE_NAME}", entryName); // Name für Confirm-Dialog
             response->print(fileItemEnd);               // ' class...>...<input value='
            response->print(entryFullPath);              // Voller Pfad für Delete
            response->print(FPSTR(fileMgrListItemFileEnd));   // '><input ...></div></li>
        }
         yield();
    }
    // dir schließt automatisch auf ESP8266

    // Die Variable errorOccurred wird im ESP8266-Teil nicht gesetzt, da wir uns auf dir.next() verlassen.
    // Die Prüfung auf entryCount == 0 deckt "leer" und "konnte nicht geöffnet werden" ab.

#endif // Ende Plattform-Block

    // --- Meldung, wenn keine Einträge gefunden wurden ---
    if (entryCount == 0 && !errorOccurred) { // errorOccurred ist nur für ESP32 relevant
         response->print(FPSTR(fileMgrNoFiles)); // Zeigt "(Verzeichnis ist leer)"
    }

    // --- Ende der Liste + Trennlinie ---
    response->print(FPSTR(fileMgrListEnd)); // </ul><hr>
    yield();

    // --- Datei-Upload-Formular ---
    String uploadForm = FPSTR(fileMgrUploadForm);
    uploadForm.replace("{CURRENT_PATH}", currentPath); // Aktuellen Pfad für Upload übergeben
    response->print(uploadForm);
    yield();

    // --- Verzeichnis erstellen Formular ---
    String createDirForm = FPSTR(fileMgrCreateDirForm);
    createDirForm.replace("{CURRENT_PATH}", currentPath); // Aktuellen Pfad für Erstellung übergeben
    response->print(createDirForm);
    yield();

    // --- Dateisystem-Informationen ---
    response->print(FPSTR(fileMgrInfoStart)); // <hr><h2>Info</h2>
#ifdef ESP32
    // ESP32 Info Code...
    uint64_t totalBytes = LittleFS.totalBytes();
    uint64_t usedBytes = LittleFS.usedBytes();
    response->print(FPSTR(fileMgrInfoTotal));
    snprintf(buffer, sizeof(buffer), "%llu", totalBytes); response->print(buffer);
    response->print(FPSTR(fileMgrInfoUsed));
    snprintf(buffer, sizeof(buffer), "%llu", usedBytes); response->print(buffer);
    response->print(FPSTR(fileMgrInfoFree));
    snprintf(buffer, sizeof(buffer), "%llu", totalBytes - usedBytes); response->print(buffer);
    response->print(FPSTR(fileMgrInfoEnd));
#else // ESP8266
    // ESP8266 Info Code...
    FSInfo fs_info;
    if (LittleFS.info(fs_info)) {
        response->print(FPSTR(fileMgrInfoTotal));
        snprintf(buffer, sizeof(buffer), "%u", fs_info.totalBytes); response->print(buffer);
        response->print(FPSTR(fileMgrInfoUsed));
        snprintf(buffer, sizeof(buffer), "%u", fs_info.usedBytes); response->print(buffer);
        response->print(FPSTR(fileMgrInfoFree));
        snprintf(buffer, sizeof(buffer), "%u", fs_info.totalBytes - fs_info.usedBytes); response->print(buffer);
        response->print(FPSTR(fileMgrInfoEnd));
    } else {
        response->print(FPSTR(fileMgrInfoError));
    }
#endif
    yield();

    // --- Ende Body/HTML ---
    response->print(FPSTR(fileMgrBodyEnd)); // </body></html>

    request->send(response);

} // Ende handleFileManager()


/************************************************************************************
 * Handler während des Datei-Uploads (angepasst für Pfad)
 ************************************************************************************/
void handleFileUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
    String targetPath = "/";

    if (request->hasArg("path")) {
        targetPath = request->arg("path");
        if (!targetPath.startsWith("/")) { targetPath = "/" + targetPath; }
        targetPath.replace("//", "/");
        if (targetPath.length() > 1 && targetPath.endsWith("/")) {
            targetPath.remove(targetPath.length() - 1);
        }
    }

    if (index == 0) {
        String safeName = filename;
        if (safeName.indexOf('/') != -1) {
            safeName = safeName.substring(safeName.lastIndexOf('/') + 1);
        }
        String fullTargetPath = (targetPath == "/") ? "/" + safeName : targetPath + "/" + safeName;
        fsUploadFile = LittleFS.open(fullTargetPath, "w");
        if (!fsUploadFile) {
            uploadStatusMessage = "FEHLER: Datei '" + safeName + "' konnte nicht erstellt werden (Pfad existiert? Speicherplatz?).";
            uploadStatusClass = "status-error";
        } else {
            uploadStatusMessage = "";
            uploadStatusClass = "";
        }
    }

    if (len) {
        if (fsUploadFile) {
            size_t bytesWritten = fsUploadFile.write(data, len);
            if (bytesWritten != len) {
                fsUploadFile.close();
                uploadStatusMessage = "FEHLER: Schreibfehler während des Uploads von '" + String(fsUploadFile.name()) + "'. Upload abgebrochen.";
                uploadStatusClass = "status-error";
                LittleFS.remove(fsUploadFile.name());
            }
        }
        yield();
    }

    if (final) {
        if (fsUploadFile) {
            String finalName = fsUploadFile.name();
            fsUploadFile.close();
            if (uploadStatusMessage.length() == 0) {
                uploadStatusMessage = "Datei '" + finalName.substring(finalName.lastIndexOf('/') + 1) + "' erfolgreich hochgeladen.";
                uploadStatusClass = "status-success";
            }
        }
    }
}

/************************************************************************************
 * Handler NACHDEM der Upload abgeschlossen (oder fehlgeschlagen) ist
 * Gibt eine Statusmeldung aus und bietet einen Link zurück zum Dateimanager.
 ************************************************************************************/
void handleFileUploaded(AsyncWebServerRequest *request) {
    String redirectPath = "/Dateimanager";
    // Pfad aus dem Formular holen, um zur richtigen Ansicht zurückzukehren
    if (request->hasArg("path")) {
        String targetPath = request->arg("path");
        // Bereinigen (wie im handleFileManager)
        if (!targetPath.startsWith("/")) { targetPath = "/" + targetPath; }
        targetPath.replace("//", "/");
        if (targetPath.length() > 1 && targetPath.endsWith("/")) {
            targetPath.remove(targetPath.length() - 1);
        }
        redirectPath += "?path=" + targetPath; // Pfad an URL anhängen
    }

    AsyncResponseStream *response = request->beginResponseStream("text/html; charset=utf-8");
    response->print(F("<html><body>"));
    if (uploadStatusMessage.length() > 0) {
        response->printf("<div class='status-message %s'>%s</div>", uploadStatusClass.c_str(), uploadStatusMessage.c_str());
        uploadStatusMessage = "";
        uploadStatusClass = "";
    }
    response->printf("<a href='%s'>Zurück</a>", redirectPath.c_str());
    response->print(F("</body></html>"));
    request->send(response);
}


/************************************************************************************
 * Handler zum Herunterladen einer Datei (unverändert, sollte mit Pfaden funktionieren)
 ************************************************************************************/
void handleFileDownload(AsyncWebServerRequest *request) {
    if (!request->hasArg("name") || request->arg("name") == "") {
        fileManagerStatusMessage = "FEHLER: Dateiname zum Herunterladen fehlt!";
        fileManagerStatusClass = "status-error";
        AsyncWebServerResponse *response = request->beginResponse(303);
        response->addHeader("Location", "/Dateimanager");
        request->send(response);
        return;
    }

    String filePath = request->arg("name"); // Enthält bereits den vollen Pfad

    // Zusätzliche Sicherheitsprüfung (optional, aber gut)
    if (filePath.indexOf("..") != -1) {
         fileManagerStatusMessage = "FEHLER: Ungültiger Pfad für Download.";
         fileManagerStatusClass = "status-error";
         AsyncWebServerResponse *response = request->beginResponse(303);
         response->addHeader("Location", "/Dateimanager");
         request->send(response);
         return;
    }
     // Sicherstellen, dass der Pfad mit '/' beginnt
    if (!filePath.startsWith("/")) {
        filePath = "/" + filePath;
    }

    if (LittleFS.exists(filePath)) {
        File file = LittleFS.open(filePath, "r");
        if (file) {
             if (file.isDirectory()) { // Verzeichnisse nicht herunterladen
                 file.close();
                 fileManagerStatusMessage = "FEHLER: Verzeichnisse können nicht heruntergeladen werden.";
                 fileManagerStatusClass = "status-error";
                 String parent = filePath.substring(0, filePath.lastIndexOf('/'));
                 AsyncWebServerResponse *response = request->beginResponse(303);
                 response->addHeader("Location", "/Dateimanager?path=" + parent);
                 request->send(response);
                 return;
             }

            String contentType = "application/octet-stream"; // Standard für Download
            // Dateinamen für den Browser extrahieren (letzter Teil des Pfades)
            String downloadFilename = filePath.substring(filePath.lastIndexOf('/') + 1);

            AsyncWebServerResponse *response = request->beginResponse(LittleFS, filePath, contentType);
            response->addHeader("Content-Disposition", "attachment; filename=\"" + downloadFilename + "\"");
            request->send(response);
            file.close();
            return;
        } else {
            fileManagerStatusMessage = "FEHLER: Datei '" + filePath + "' konnte nicht geöffnet werden.";
            fileManagerStatusClass = "status-error";
        }
    } else {
        fileManagerStatusMessage = "FEHLER: Datei '" + filePath + "' nicht gefunden.";
        fileManagerStatusClass = "status-error";
    }

     // Bei Fehlern zurückleiten
    String parentPath = filePath.substring(0, filePath.lastIndexOf('/'));
    if (parentPath == "") parentPath = "/";
    AsyncWebServerResponse *response = request->beginResponse(303);
    response->addHeader("Location", "/Dateimanager?path=" + parentPath);
    request->send(response);
}

/************************************************************************************
 * Handler zum Löschen einer Datei oder eines Verzeichnisses
 ************************************************************************************/
void handleFileDelete(AsyncWebServerRequest *request) {
    if (!request->hasArg("name") || request->arg("name") == "") {
        fileManagerStatusMessage = "FEHLER: Name zum Löschen fehlt!";
        fileManagerStatusClass = "status-error";
        AsyncWebServerResponse *response = request->beginResponse(303);
        response->addHeader("Location", "/Dateimanager");
        request->send(response);
        return;
    }

    String itemPath = request->arg("name"); // Enthält den vollen Pfad
    bool isDir = request->hasArg("isDir") && request->arg("isDir") == "1"; // Prüfen ob es ein Verzeichnis ist

     // Zusätzliche Sicherheitsprüfung
    if (itemPath.indexOf("..") != -1 || itemPath == "/") {
         fileManagerStatusMessage = "FEHLER: Ungültiger Pfad zum Löschen.";
         fileManagerStatusClass = "status-error";
         AsyncWebServerResponse *response = request->beginResponse(303);
         response->addHeader("Location", "/Dateimanager");
         request->send(response);
         return;
    }
     // Sicherstellen, dass der Pfad mit '/' beginnt
    if (!itemPath.startsWith("/")) {
        itemPath = "/" + itemPath;
    }

    String itemName = itemPath.substring(itemPath.lastIndexOf('/') + 1);
    String parentPath = itemPath.substring(0, itemPath.lastIndexOf('/'));
    if (parentPath == "") parentPath = "/";

    bool success = false;
    if (LittleFS.exists(itemPath)) {
        if (isDir) {
            // --- Rekursives Löschen für Verzeichnisse ---
            success = removeDirectoryRecursive(itemPath);
            if (success) {
                fileManagerStatusMessage = "Verzeichnis '" + itemName + "' und Inhalt erfolgreich gelöscht.";
                fileManagerStatusClass = "status-success";
            } else {
                fileManagerStatusMessage = "FEHLER: Verzeichnis '" + itemName + "' konnte nicht vollständig gelöscht werden.";
                fileManagerStatusClass = "status-error";
            }
        } else {
            // --- Einfaches Löschen für Dateien ---
            if (LittleFS.remove(itemPath)) {
                fileManagerStatusMessage = "Datei '" + itemName + "' erfolgreich gelöscht.";
                fileManagerStatusClass = "status-success";
                success = true;
            } else {
                fileManagerStatusMessage = "FEHLER: Datei '" + itemName + "' konnte nicht gelöscht werden.";
                fileManagerStatusClass = "status-error";
            }
        }
    } else {
        fileManagerStatusMessage = "FEHLER: Element '" + itemName + "' nicht gefunden.";
        fileManagerStatusClass = "status-error";
    }

    // Redirect zurück zum übergeordneten Verzeichnis
    AsyncWebServerResponse *response = request->beginResponse(303);
    response->addHeader("Location", "/Dateimanager?path=" + parentPath);
    request->send(response);
}

// HANDLER zum Erstellen von Verzeichnissen
void handleCreateDirectory(AsyncWebServerRequest *request) {
    String basePath = "/";
    if (request->hasArg("basePath")) {
        basePath = request->arg("basePath");
         // Bereinigen
        if (!basePath.startsWith("/")) { basePath = "/" + basePath; }
        basePath.replace("//", "/");
        if (basePath.length() > 1 && basePath.endsWith("/")) {
            basePath.remove(basePath.length() - 1);
        }
    }

    if (!request->hasArg("dirName") || request->arg("dirName") == "") {
        fileManagerStatusMessage = "FEHLER: Name für neues Verzeichnis fehlt!";
        fileManagerStatusClass = "status-error";
    } else {
        String dirName = request->arg("dirName");
        // Einfache Validierung des Namens (gegen Pfadtrenner etc.)
        if (dirName.indexOf('/') != -1 || dirName.indexOf('\\') != -1 || dirName == "." || dirName == "..") {
             fileManagerStatusMessage = "FEHLER: Ungültiger Verzeichnisname!";
             fileManagerStatusClass = "status-error";
        } else {
            String newDirPath;
            if (basePath == "/") {
                newDirPath = "/" + dirName;
            } else {
                newDirPath = basePath + "/" + dirName;
            }

            if (LittleFS.mkdir(newDirPath)) {
                fileManagerStatusMessage = "Verzeichnis '" + dirName + "' erfolgreich erstellt.";
                fileManagerStatusClass = "status-success";
            } else {
                fileManagerStatusMessage = "FEHLER: Verzeichnis '" + dirName + "' konnte nicht erstellt werden (existiert bereits?).";
                fileManagerStatusClass = "status-error";
            }
        }
    }

    // Redirect zurück zum Basisverzeichnis
    AsyncWebServerResponse *response = request->beginResponse(303);
    response->addHeader("Location", "/Dateimanager?path=" + basePath);
    request->send(response);
}

/************************************************************************************
 * Handler zum Neustarten des Geräts
 ************************************************************************************/

void handleRestartDevice(AsyncWebServerRequest *request) {
    // Serial.println("Neustart über Webinterface ausgelöst...");

    AsyncWebServerResponse *response = request->beginResponse(303);
    response->addHeader("Location", "/");
    request->send(response);

    // Kurz warten, damit der Browser die Antwort verarbeiten kann
    scheduleRestart(2000); // Neustart einplanen
}

/************************************************************************************
 * Handler für den Firmware-Upload-Prozess (während des Uploads)
 * - Berücksichtigt Unterschiede bei Update.begin() für ESP32/ESP8266
 ************************************************************************************/
void handleFirmwareUploadProgress(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
  (void)request;
  (void)filename;

  if (index == 0) {
    passwordFound = false;
    passMatchPos = 0;
    uint32_t maxSketchSpace = 0;
    bool updateStarted = false;
#ifdef ESP32
    updateStarted = Update.begin(UPDATE_SIZE_UNKNOWN, U_FLASH);
#else
    maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
    updateStarted = Update.begin(maxSketchSpace);
#endif
    if (!updateStarted) {
      Update.printError(Serial);
    }
  }

  if (len) {
    for (size_t i = 0; i < len; i++) {
      char c = (char)data[i];
      if (!passwordFound) {
        if (c == FIRMWARE_PASSWORD[passMatchPos]) {
          passMatchPos++;
          if (passMatchPos == passLength) {
            passwordFound = true;
          }
        } else {
          passMatchPos = (c == FIRMWARE_PASSWORD[0]) ? 1 : 0;
        }
      }
    }
    yield();
    if (Update.write(data, len) != len) {
      Update.printError(Serial);
    }
  }

  if (final) {
    if (!passwordFound) {
      Update.end(false);
    } else {
      if (!Update.end(true)) {
        Serial.println("FEHLER: Update.end() fehlgeschlagen!");
        Update.printError(Serial);
      }
    }
  }

  yield();
}

/************************************************************************************
 * Handler für die Antwort nach einem Firmware-Update - Heap-Optimiert
 ************************************************************************************/

static const char firmwareUpdateResultHtmlHead[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><title>Firmware-Update</title><meta charset='utf-8'><meta name='theme-color' content='#000000'><meta name='viewport' content='width=device-width, initial-scale=1.0'><meta http-equiv='refresh' content='5; url=/'>
)rawliteral";

static const char firmwareUpdateResultHtmlHeadEnd[] PROGMEM = R"rawliteral(
</head>
)rawliteral";

static const char firmwareUpdateResultBodyStart[] PROGMEM = R"rawliteral(
<body>
<form>
<h3>Firmware-Update</h3>
)rawliteral";

static const char firmwareUpdateResultBodyEnd[] PROGMEM = R"rawliteral(
  <p>Die Steuerung startet neu...</p>
</form>
</body>
</html>
)rawliteral";

void handleFirmwareUpdateResponse(AsyncWebServerRequest *request) {
  String statusMessage;  // Status wird hier ermittelt
  if (!passwordFound) {
    statusMessage = F("Update abgebrochen! Die Firmware konnte nicht validiert werden.");
  } else {
    if (Update.hasError()) {
      statusMessage = F("Update fehlgeschlagen!");
    } else {
      statusMessage = F("Update erfolgreich!");
    }
  }

  // Statusmeldung auf der seriellen Schnittstelle ausgeben
  if (statusMessage.length() > 0) {
    Serial.println(statusMessage);
  }

  AsyncResponseStream *response = request->beginResponseStream("text/html; charset=utf-8");
  response->print(FPSTR(firmwareUpdateResultHtmlHead));
  response->print(FPSTR(commonStyle));
  response->print(FPSTR(firmwareUpdateResultHtmlHeadEnd));
  response->print(FPSTR(commonNav));
  response->print(FPSTR(firmwareUpdateResultBodyStart));
  response->print(F("<b>"));
  response->print(statusMessage);
  response->print(F("</b>"));
  response->print(FPSTR(firmwareUpdateResultBodyEnd));
  request->send(response);

  // Neustart-Logik
  scheduleRestart(5000);  // Browser hat Zeit, die Meldung anzuzeigen
}

/************************************************************************************
 * Schreibt einen gültigen Shot (Dauer > 20s) in die CSV-Logdatei.
 * Dateiname: Nutzungsstatistik.csv
 * Format: UnixTimestamp,DauerInMs
 ************************************************************************************/
void logShot(unsigned long durationMs, bool wasByWeight, float finalWeight) {
  // Nur loggen, wenn Zeit synchronisiert ist (prüft Jahr > 2023)
  time_t now_t;
  time(&now_t);  // Holt aktuelle Zeit (Unix Timestamp) vom System
  struct tm timeinfo;
  localtime_r(&now_t, &timeinfo);  // Konvertiert in lokale Zeit

  if (timeinfo.tm_year > (2023 - 1900)) {  // tm_year ist Jahre seit 1900
    File logFile = LittleFS.open("/Nutzungsstatistik.csv", "a");  // "a" = Append
    if (!logFile) {
      // Serial.println("FEHLER: Konnte Nutzungsstatistik.csv zum Schreiben nicht öffnen!");
      return;
    }

    String logEntry;
    if (wasByWeight) {
        // Format mit Gewicht: Timestamp,Dauer,Gewicht
        logEntry = String(now_t) + "," + String(durationMs) + "," + String(finalWeight, 1) + "\n";
    } else {
        // Altes Format ohne Gewicht: Timestamp,Dauer
        logEntry = String(now_t) + "," + String(durationMs) + "\n";
    }
    
    if (logFile.print(logEntry)) {
      // Serial.printf("Shot geloggt: %s", logEntry.c_str()); // Optional: Debug-Ausgabe
    // } else {
    //   Serial.println("FEHLER: Konnte nicht in Nutzungsstatistik.csv schreiben!");
    }
    logFile.close();
  // } else {
  //   Serial.println("WARNUNG: NTP Zeit noch nicht synchronisiert, Shot nicht geloggt.");
  }
  yield();
}

// --- Hilfsfunktionen für Datumsvergleiche ---

bool isSameDay(time_t timestamp, time_t now) {
  struct tm ts_tm, now_tm;
  localtime_r(&timestamp, &ts_tm);
  localtime_r(&now, &now_tm);
  return (ts_tm.tm_year == now_tm.tm_year && ts_tm.tm_mon == now_tm.tm_mon && ts_tm.tm_mday == now_tm.tm_mday);
}

bool isSameWeek(time_t timestamp, time_t now) {
  struct tm ts_tm, now_tm;
  localtime_r(&timestamp, &ts_tm);
  localtime_r(&now, &now_tm);
  int ts_wday = (ts_tm.tm_wday == 0) ? 6 : ts_tm.tm_wday - 1;  // Mo=0..So=6
  int now_wday = (now_tm.tm_wday == 0) ? 6 : now_tm.tm_wday - 1;
  time_t ts_monday_start_of_day = timestamp - ts_tm.tm_hour * 3600 - ts_tm.tm_min * 60 - ts_tm.tm_sec - ts_wday * 86400;
  time_t now_monday_start_of_day = now - now_tm.tm_hour * 3600 - now_tm.tm_min * 60 - now_tm.tm_sec - now_wday * 86400;
  return (ts_monday_start_of_day == now_monday_start_of_day);
}

bool isSameMonth(time_t timestamp, time_t now) {
  struct tm ts_tm, now_tm;
  localtime_r(&timestamp, &ts_tm);
  localtime_r(&now, &now_tm);
  return (ts_tm.tm_year == now_tm.tm_year && ts_tm.tm_mon == now_tm.tm_mon);
}

/************************************************************************************
 * Liest die Log-Datei (Nutzungsstatistik.csv) und berechnet Statistiken.
 * Gibt eine ShotStats-Struktur zurück.
 ************************************************************************************/
/************************************************************************************
 * Liest die Log-Datei (Nutzungsstatistik.csv) und berechnet Statistiken.
 * Kann Zeilen mit 2 Werten (Timestamp, Dauer) und 3 Werten (Timestamp, Dauer, Gewicht) verarbeiten.
 * Gibt eine ShotStats-Struktur zurück.
 ************************************************************************************/
ShotStats calculateShotStatistics() {
  ShotStats stats;  // Initialisiert alle Felder mit 0/false

  time_t now_t;
  time(&now_t);
  struct tm timeinfo;
  localtime_r(&now_t, &timeinfo);
  // Prüfe, ob die *aktuelle* Zeit gültig erscheint (wichtig für relative Vergleiche wie Gestern etc.)
  bool timeValid = (timeinfo.tm_year > (2023 - 1900));  // Beispiel: Prüft ob Jahr > 2023

  File logFile = LittleFS.open("/Nutzungsstatistik.csv", "r");
  if (!logFile || logFile.size() == 0) {
    if (logFile) logFile.close();
    stats.historyAvailable = false;
    // Serial.println("Statistik: Log-Datei nicht gefunden oder leer."); // Optional: Debug-Ausgabe
    return stats;
  }

  stats.historyAvailable = true;  // Datei existiert und ist > 0 Bytes

  int lineCounter = 0;  // Zähler für yield()
  while (logFile.available()) {
    String line = logFile.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;

    // --- Flexibles Parsen für 2 oder 3 Spalten ---
    int firstCommaIndex = line.indexOf(',');
    int secondCommaIndex = -1;
    if (firstCommaIndex != -1) {
        // Suche nach dem zweiten Komma NACH dem ersten
        secondCommaIndex = line.indexOf(',', firstCommaIndex + 1);
    }
    
    if (firstCommaIndex != -1) {
      // strtoull verwenden, um mögliche Überläufe bei 32-Bit time_t zu vermeiden
      time_t timestamp = (time_t)strtoull(line.substring(0, firstCommaIndex).c_str(), NULL, 10);
      unsigned long durationMs;

      if(secondCommaIndex != -1) {
        // Neue Zeile mit Gewicht: Dauer ist zwischen den Kommas
        durationMs = strtoul(line.substring(firstCommaIndex + 1, secondCommaIndex).c_str(), NULL, 10);
        // Das Gewicht selbst wird hier nicht für die Statistik benötigt, nur korrekt geparst.
      } else {
        // Alte Zeile ohne Gewicht: Dauer ist nach dem ersten Komma
        durationMs = strtoul(line.substring(firstCommaIndex + 1).c_str(), NULL, 10);
      }
      // --- Ende Flexibles Parsen ---

      if (timestamp > 0 && durationMs > 0) {  // Grundlegende Prüfung der gelesenen Werte
        stats.totalShotsLogged++;
        stats.totalDurationMs += durationMs;

        // Ersten und letzten Zeitstempel aktualisieren
        if (stats.firstShotTimestamp == 0 || timestamp < stats.firstShotTimestamp) {
          stats.firstShotTimestamp = timestamp;
        }
        if (timestamp > stats.lastShotTimestamp) {
          stats.lastShotTimestamp = timestamp;
        }

        // Zeitbasierte Zähler nur füllen, wenn aktuelle Zeit gültig ist
        if (timeValid) {
          if (isSameDay(timestamp, now_t)) { stats.shotsToday++; }
          if (isSameWeek(timestamp, now_t)) { stats.shotsThisWeek++; }
          if (isSameMonth(timestamp, now_t)) { stats.shotsThisMonth++; }

          // Zähler für "letzte Periode"
          if (isYesterday(timestamp, now_t)) { stats.shotsYesterday++; }
          if (isLastWeek(timestamp, now_t)) { stats.shotsLastWeek++; }
          if (isLastMonth(timestamp, now_t)) { stats.shotsLastMonth++; }
        }
      // } else {
      //   Serial.printf("WARNUNG: Ungültige Werte in Log übersprungen: %s\n", line.c_str());
      }
    // } else {
    //   Serial.printf("WARNUNG: Komma in Logzeile nicht gefunden: %s\n", line.c_str());
    }
    lineCounter++;
    if (lineCounter % 50 == 0)
      yield();  // Watchdog bei großen Dateien vermeiden
  }
  logFile.close();

  // Berechnungen nach dem Lesen der Datei
  if (stats.totalShotsLogged > 0) {
    stats.averageDurationSec = (double)stats.totalDurationMs / 1000.0 / stats.totalShotsLogged;

    // --- Durchschnitt Shots/Tag ---
    if (stats.firstShotTimestamp > 0 && timeValid && now_t > stats.firstShotTimestamp) {
      unsigned long secondsElapsed = now_t - stats.firstShotTimestamp;
      unsigned long daysElapsed = secondsElapsed / 86400UL;  // Ganze Tage seit erstem Shot
      if (daysElapsed == 0) { daysElapsed = 1; }             // Mindestens 1 Tag annehmen, um Division durch 0 zu vermeiden
      stats.avgShotsPerDay = (double)stats.totalShotsLogged / daysElapsed;
    }
  }
  return stats;
}

/************************************************************************************
 * Handler zum Erstellen und Senden des formatierten Verlaufs (aus Nutzungsstatistik.csv)
 * als TXT-Datei. Verarbeitet Zeilen mit und ohne Gewichtsangabe.
 ************************************************************************************/
void handleDownloadNutzungsstatistik(AsyncWebServerRequest *request) {
  File logFile = LittleFS.open("/Nutzungsstatistik.csv", "r");

  if (!logFile) {
    request->send(404, "text/plain", "Fehler: Log-Datei nicht gefunden.");
    yield();
    return;
  }
  if (logFile.size() == 0) {
    logFile.close();
    request->send(200, "text/plain", "Nutzungsstatistik ist leer.");
    yield();
    return;
  }

  AsyncResponseStream *response = request->beginResponseStream("text/plain; charset=utf-8");
  response->addHeader("Content-Disposition", "attachment; filename=\"Nutzungsverlauf.txt\"");

  char lineBuffer[120];
  char dateBuffer[25];
  char durationBuffer[10];
  char weightBuffer[10];

  response->print("Datum, Uhrzeit, Dauer (s), Gewicht (g)\n");
  response->print("------------------------------------------\n");

  int lineCount = 0;
  while (logFile.available()) {
    String line = logFile.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;

    int firstCommaIndex = line.indexOf(',');
    int secondCommaIndex = -1;
    if (firstCommaIndex != -1) {
        secondCommaIndex = line.indexOf(',', firstCommaIndex + 1);
    }

    if (firstCommaIndex != -1) {
      time_t timestamp = (time_t)strtoull(line.substring(0, firstCommaIndex).c_str(), NULL, 10);
      unsigned long durationMs;
      float weight = -1.0f;

      if(secondCommaIndex != -1) {
        durationMs = strtoul(line.substring(firstCommaIndex + 1, secondCommaIndex).c_str(), NULL, 10);
        weight = line.substring(secondCommaIndex + 1).toFloat();
      } else {
        durationMs = strtoul(line.substring(firstCommaIndex + 1).c_str(), NULL, 10);
      }

      if (timestamp > 0 && durationMs > 0) {
        struct tm timeinfo;
        localtime_r(&timestamp, &timeinfo);
        strftime(dateBuffer, sizeof(dateBuffer), "%d.%m.%Y, %H:%M:%S", &timeinfo);
        snprintf(durationBuffer, sizeof(durationBuffer), "%.1f", durationMs / 1000.0);

        if (weight >= 0.0) {
            snprintf(weightBuffer, sizeof(weightBuffer), "%.1f", weight);
            snprintf(lineBuffer, sizeof(lineBuffer), "%s, %s, %s\n", dateBuffer, durationBuffer, weightBuffer);
        } else {
            snprintf(lineBuffer, sizeof(lineBuffer), "%s, %s, -\n", dateBuffer, durationBuffer);
        }

        response->print(lineBuffer);
        lineCount++;
        if (lineCount % 20 == 0) { yield(); }
      }
    }
    yield();
  }
  logFile.close();

  request->send(response);
  // Serial.printf("Formatierter Verlauf mit %d Zeilen gesendet.\n", lineCount);
}

/************************************************************************************
* Hilfsfunktionen zum Erstellen der Statistik-Daten
 ************************************************************************************/

// Prüft, ob der timestamp am Vortag von 'now' liegt
bool isYesterday(time_t timestamp, time_t now) {
  time_t yesterday_t = now - 86400UL;  // Zeitstempel für vor 24 Stunden
  struct tm ts_tm, yest_tm;
  localtime_r(&timestamp, &ts_tm);      // Konvertiert Timestamp in lokale Zeit-Struktur
  localtime_r(&yesterday_t, &yest_tm);  // Konvertiert Zeitstempel von Gestern
  yield();
  // Prüfen, ob Jahr, Monat und Tag übereinstimmen
  return (ts_tm.tm_year == yest_tm.tm_year && ts_tm.tm_mon == yest_tm.tm_mon && ts_tm.tm_mday == yest_tm.tm_mday);
}

// Prüft, ob der timestamp in der Kalenderwoche *vor* der von 'now' liegt (Annahme: Woche beginnt Montag)
bool isLastWeek(time_t timestamp, time_t now) {
  struct tm ts_tm, now_tm;
  localtime_r(&timestamp, &ts_tm);
  localtime_r(&now, &now_tm);

  // Berechne den Start der aktuellen Woche (Montag 00:00:00)
  int now_wday = (now_tm.tm_wday == 0) ? 6 : now_tm.tm_wday - 1;  // Korrigiert Wochentag Mo=0..So=6
  time_t startOfThisWeek = now - (now_tm.tm_hour * 3600) - (now_tm.tm_min * 60) - now_tm.tm_sec - (now_wday * 86400UL);

  // Berechne den Start der letzten Woche (Montag 00:00:00)
  time_t startOfLastWeek = startOfThisWeek - (7 * 86400UL);

  // Prüfe, ob der Timestamp in der letzten Woche liegt (>= Start der letzten Woche UND < Start dieser Woche)
  yield();
  return (timestamp >= startOfLastWeek && timestamp < startOfThisWeek);
}


// Prüft, ob der timestamp im Kalendermonat *vor* dem von 'now' liegt
bool isLastMonth(time_t timestamp, time_t now) {
  struct tm ts_tm, now_tm;
  localtime_r(&timestamp, &ts_tm);
  localtime_r(&now, &now_tm);

  int lastMonth = now_tm.tm_mon - 1;   // Monat vor dem aktuellen
  int lastMonthYear = now_tm.tm_year;  // Jahr des letzten Monats (Standard: aktuelles Jahr)

  if (lastMonth < 0) {                   // Sonderfall: Aktueller Monat ist Januar (tm_mon = 0)
    lastMonth = 11;                      // Letzter Monat war Dezember (tm_mon = 11)
    lastMonthYear = now_tm.tm_year - 1;  // im vorherigen Jahr
  }

  // Prüfe, ob Jahr und Monat des Timestamps mit dem letzten Monat übereinstimmen
  yield();
  return (ts_tm.tm_year == lastMonthYear && ts_tm.tm_mon == lastMonth);
}

/************************************************************************************
 * Handler zum entfernen der Nutzungsstatistik-Datei (/Nutzungsstatistik.csv)
 ************************************************************************************/
void handleDeleteStatistics(AsyncWebServerRequest *request) {
  const char* statsFilename = "/Nutzungsstatistik.csv";

  bool success = false;
  if (LittleFS.exists(statsFilename)) {
    if (LittleFS.remove(statsFilename)) {
      success = true;
      shotCounter = 0;
      EEPROM.put(EEPROM_ADDR_SHOTCOUNTER, shotCounter);
      // Andere Statistikvariablen werden beim nächsten Aufruf von calculateShotStatistics neu berechnet
      // maintenanceIntervalCounter NICHT zurücksetzen, das hat einen eigenen Reset.
    }
  } else {
    success = true;
  }

  AsyncWebServerResponse *response = request->beginResponse(303);
  response->addHeader(F("Location"), F("/Info"));
  request->send(response);
  yield();
}

/************************************************************************************
 * Handler zum Umschalten des Wartungsmodus
 ************************************************************************************/
void handleToggleWartungsmodus(AsyncWebServerRequest *request) {
  wartungsModusAktiv = !wartungsModusAktiv;  // Zustand umkehren
  if (!wartungsModusAktiv) {
    // Standard-Temperaturwerte laden
    EEPROM.get(EEPROM_ADDR_SETPOINT_WASSER, SetpointWasser);
    EEPROM.get(EEPROM_ADDR_SETPOINT_DAMPF, SetpointDampf);
  }
  if (wartungsModusAktiv) {
    // Temperaturen auf Wartungsmodus-Temperaturen setzen
    SetpointWasser = wartungsModusTemp;
    SetpointDampf = wartungsModusTemp;
  }
  // Serial.print("Wartungsmodus: ");
  // Serial.println(wartungsModusAktiv ? "aktiviert" : "deaktiviert");

  AsyncWebServerResponse *response = request->beginResponse(303);
  response->addHeader(F("Location"), F("/Service"));
  request->send(response);
}

/************************************************************************************
 * Führt das verzögerte Speichern von Shot-Daten durch, um die loop() nicht zu blockieren.
 ************************************************************************************/
void handleShotSaving(AsyncWebServerRequest *request) {
    (void)request;
    if (shotNeedsToBeSaved) {
        // Flag sofort zurücksetzen, um Mehrfachausführung zu verhindern
        shotNeedsToBeSaved = false;

        // Die langsamen Operationen werden jetzt hier ausgeführt
        shotCounter++;
        maintenanceIntervalCounter++;
        EEPROM.put(EEPROM_ADDR_SHOTCOUNTER, shotCounter);
        EEPROM.put(EEPROM_ADDR_MAINTENANCE_INTERVAL_COUNTER, maintenanceIntervalCounter);
        EEPROM.commit();
        
        // Shot mit aktuellen Parametern protokollieren
        logShot(savedShotDuration, savedShotWasByWeight, savedShotFinalWeight);

        // Zusätzliche Zustandsvariablen zurücksetzen
        savedShotWasByWeight = false;
        savedShotFinalWeight = 0.0f;
    }
}

// =============================================================================
// START: Dashboard Code Block (PROGMEM Strings & Handler Funktionen)
// Version v3.8 - Basiert auf v3.5, mit verfeinerter Farb-Logik im JS
// =============================================================================

// --- PROGMEM Strings für Dashboard ---

static const char dashboardHtmlHead[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html lang="de"><head><title>Dashboard</title><meta charset="UTF-8"><meta name='theme-color' content='#000000'><meta name='viewport' content='width=device-width, initial-scale=1.0'>
)rawliteral";

// Dashboard spezifische CSS-Regeln (v3.6 - Eco Button Farbe angepasst)
static const char dashboardStyles[] PROGMEM = R"rawliteral(
<style>
  /* Grundlayout */
  .dashboard-container { display: flex; flex-wrap: wrap; padding: 15px; gap: 20px; max-width: 1200px; margin: 20px auto; }
  .main-display { flex: 2; min-width: 300px; background: rgba(255, 255, 255, 0.05); border-radius: 12px; padding: 20px; box-shadow: 0 4px 15px rgba(0,0,0,0.2); border: 1px solid rgba(255, 255, 255, 0.1); display: flex; flex-direction: column; gap: 15px; }
  .control-panel { flex: 1; min-width: 280px; background: rgba(255, 255, 255, 0.05); border-radius: 12px; padding: 20px; box-shadow: 0 4px 15px rgba(0,0,0,0.2); border: 1px solid rgba(255, 255, 255, 0.1); display: flex; flex-direction: column; gap: 18px; }

  /* Temperaturanzeige */
  .temp-display { text-align: center; border-bottom: 1px solid rgba(255, 255, 255, 0.1); padding-bottom: 15px; }
  .temp-display h2 { margin: 0 0 5px 0; font-size: 1.1em; color: #ccc; font-weight: 500; }
  .temp-actual { font-size: 3.5em; font-weight: bold; line-height: 1.1; color: #fff; transition: color 0.3s ease; }
  .temp-target { font-size: 1.2em; color: #aaa; }
  .temp-actual.heating { color: #F7941D; } /* Orange */
  .temp-actual.ready { color: #28a745; }   /* Grün */
  .temp-actual.eco { color: #00AEEF; }     /* Blau */
  .temp-actual.error { color: #dc3545; }   /* Rot */
  .temp-actual.maintenance { color: #6c757d; } /* Grau */

  /* Statusanzeige */
  .status-indicator { text-align: center; background: rgba(0,0,0,0.2); padding: 10px; border-radius: 8px; }
  .status-indicator span { font-size: 1.2em; font-weight: 500; color: #eee; }
  .status-indicator .error { color: #FFCC00; font-weight: bold;}

  /* Shot-Anzeige */
  .shot-info { text-align: center; background: rgba(0,0,0,0.2); padding: 15px; border-radius: 8px; display: none; }
  .shot-info h3 { margin: 0 0 8px 0; font-size: 1.1em; color: #ccc; font-weight: 500; }
  .shot-timer-display { font-size: 2.8em; font-weight: bold; color: var(--accent-color); line-height: 1; }
  .shot-weight-display { font-size: 1.5em; color: #ddd; margin-top: 8px; }

  /* Control Panel Allgemein */
  .control-panel h3 { margin: 0 0 10px 0; font-size: 1.2em; color: #eee; font-weight: 600; border-bottom: 1px solid rgba(255, 255, 255, 0.2); padding-bottom: 8px; }
  .control-group { display: flex; align-items: center; gap: 10px; flex-wrap: wrap; }
  .control-group label { flex-basis: 60px; font-size: 0.95em; color: #ccc; text-align: right; }
  .control-group input[type="number"] { flex-grow: 1; width: auto; min-width: 80px; max-width: 100px; }
  .control-group button { padding: 6px 12px; font-size: 0.9em; margin-left: 5px; }
  .control-panel .toggle-switch-container { justify-content: space-between; margin-bottom: 5px; }
  .control-panel .toggle-switch-label-text { flex-grow: 0; font-size: 0.95em; }
  .control-panel select { width: 100%; background: rgba(0, 0, 0, 0.4); color: #fff; border: 1px solid #555; border-radius: 8px; padding: 10px; }

  /* Allgemeiner Action Button Style (Standard: Grau) */
  .control-panel .action-button {
      width: 100%; margin-top: 10px; background-color: #444; color: #eee;
      padding: 8px 12px; border: none; border-radius: 20px;
      font-size: 0.9em; font-weight: 600; cursor: pointer; transition: all 0.2s ease;
      text-align: center;
  }
  .control-panel .action-button:hover:not(:disabled) { background-color: #555; transform: translateY(-1px); }
  .control-panel .action-button:disabled { background-color: #333; color: #777; cursor: not-allowed; transform: none; box-shadow: none; opacity: 0.6; }

  /* Style für Buttons die die Akzentfarbe haben sollen (wenn nicht disabled) */
  .control-panel .profile-load-button:not(:disabled),
  .control-panel button#steam_now_button:not(:disabled),
  .control-panel button#tare_button:not(:disabled),
  .control-panel button#eco_stop_button:not(:disabled) { /* Eco Stop Button jetzt hier enthalten */
      background: var(--accent-color, #d89904);
      color: #000000;
      box-shadow: 0 4px 12px rgba(216, 153, 4, 0.2);
  }
   /* Hover für diese Buttons (bleibt Kupfer) */
  .control-panel .profile-load-button:not(:disabled):hover,
  .control-panel button#steam_now_button:not(:disabled):hover,
  .control-panel button#tare_button:not(:disabled):hover,
  .control-panel button#eco_stop_button:not(:disabled):hover { /* Eco Stop Button jetzt hier enthalten */
      background: var(--accent-color, #d89904);
      opacity: 0.85;
      transform: translateY(-2px);
  }

  /* Dashboard Nachrichten */
  #dashboard-message { text-align: center; padding: 8px; margin-top: 15px; border-radius: 5px; display: none; font-weight: 500;}
  #dashboard-message.success { background-color: rgba(40, 167, 69, 0.2); border: 1px solid rgba(40, 167, 69, 0.4); color: #62c34b; }
  #dashboard-message.error { background-color: rgba(220, 53, 69, 0.2); border: 1px solid rgba(220, 53, 69, 0.4); color: #f8d7da; }
  #dashboard-message.info { background-color: rgba(0, 123, 255, 0.1); border: 1px solid rgba(0,123,255,0.3); color: #85C1E9; }

  /* Responsive Anpassungen */
  @media (max-width: 768px) { /*...*/ }
</style>
</head>
)rawliteral";

static const char dashboardBodyStart[] PROGMEM = R"rawliteral(
<body>
)rawliteral";

static const char dashboardContainerStart[] PROGMEM = R"rawliteral(
<div class="dashboard-container">
  <div class="main-display">
    <div class="temp-display">
      <h2>Wasser</h2>
      <div id="tempW_actual" class="temp-actual">--.-</div>
      <div id="tempW_target" class="temp-target">Soll: --.- &deg;C</div>
    </div>
    <div class="temp-display">
      <h2>Dampf</h2>
      <div id="tempD_actual" class="temp-actual">--.-</div>
      <div id="tempD_target" class="temp-target">Soll: --.- &deg;C</div>
    </div>
    <div class="status-indicator">
      Status: <span id="machine_status">Initialisiere...</span>
    </div>
    <div id="shot-info" class="shot-info">
      <h3>Bezug aktiv</h3>
      <div id="shot-timer-display" class="shot-timer-display">0.0s</div>
)rawliteral";

#ifdef ESP32
static const char dashboardShotWeight[] PROGMEM = R"rawliteral(
      <div id="shot-weight-display" class="shot-weight-display">Gewicht: --.- g</div>
)rawliteral";
#endif

// Angepasster Control Panel HTML String (Eco Toggle -> Button)
static const char dashboardControlPanelHTML[] PROGMEM = R"rawliteral(
    </div> <div id="dashboard-message"></div>
  </div> <div class="control-panel">
    <h3>Schnelleinstellung</h3>
    <div class="control-group">
      <label for="setpointW_input">Wasser:</label>
      <input type="number" id="setpointW_input" min="0" max="135" step="1">
      <button id="setpointW_save" class="save-button">OK</button>
    </div>
    <div class="control-group">
      <label for="setpointD_input">Dampf:</label>
      <input type="number" id="setpointD_input" min="0" max="200" step="1">
      <button id="setpointD_save" class="save-button">OK</button>
    </div>

    <h3>Modus</h3>
    <button id="eco_stop_button" class="action-button" disabled>Eco-Modus beenden</button>
    <div class="toggle-switch-container" style="margin-top: 15px;"> <span class="toggle-switch-label-text">Wartungsmodus:</span>
      <label class="toggle-switch">
        <input type="checkbox" id="maintenance_toggle">
        <span class="toggle-slider"></span>
      </label>
    </div>

    <h3>Boost & Heizung</h3> <div class="toggle-switch-container">
      <span class="toggle-switch-label-text">Boost Wasser:</span>
      <label class="toggle-switch">
        <input type="checkbox" id="boostW_toggle">
        <span class="toggle-slider"></span>
      </label>
    </div>
    <div class="toggle-switch-container">
      <span class="toggle-switch-label-text">Boost Dampf:</span>
      <label class="toggle-switch">
        <input type="checkbox" id="boostD_toggle">
        <span class="toggle-slider"></span>
      </label>
    </div>
    <button id="steam_now_button" class="action-button" style="margin-top: 10px;" disabled>Dampf sofort heizen</button>

    <h3>Profile</h3>
    <select id="profile_select">
      <option value="">Profil wählen...</option>
      {PROFILE_OPTIONS} </select>
    <button id="profile_load_button" class="profile-load-button action-button">Profil laden</button>

    )rawliteral";

#ifdef ESP32
static const char dashboardBrewControlSection[] PROGMEM = R"rawliteral(
    <h3>Brew Control</h3>
     <div class="toggle-switch-container">
      <span class="toggle-switch-label-text">Pre-Infusion:</span>
      <label class="toggle-switch">
        <input type="checkbox" id="pi_toggle">
        <span class="toggle-slider"></span>
      </label>
    </div>
     <div class="toggle-switch-container">
      <span class="toggle-switch-label-text">Brew-By-Time:</span>
      <label class="toggle-switch">
        <input type="checkbox" id="bbt_toggle">
        <span class="toggle-slider"></span>
      </label>
    </div>
     <div class="toggle-switch-container">
      <span class="toggle-switch-label-text">Brew-By-Weight:</span>
      <label class="toggle-switch">
        <input type="checkbox" id="bbw_toggle">
        <span class="toggle-slider"></span>
      </label>
    </div>
    <div class="toggle-switch-container">
    <span class="toggle-switch-label-text">Waage anzeigen:</span>
    <label class="toggle-switch">
      <input type="checkbox" id="scale_mode_toggle" disabled>
      <span class="toggle-slider"></span>
    </label>
    </div>
    <button id="tare_button" class="action-button" disabled>Waage tarieren</button>
)rawliteral";
#endif

static const char dashboardEndContainer[] PROGMEM = R"rawliteral(
  </div> </div> )rawliteral";

// --- Angepasstes JavaScript (v3.10 - Shot Timer Anzeige um 3s verzögert ausblenden) ---
static const char dashboardJavaScript[] PROGMEM = R"rawliteral(
<script>
  let updateInterval = 2000;
  let dashboardIntervalId = null;
  // --- Timer Variablen ---
  let jsShotStartTime = 0;       // Speichert Date.now() WENN Shot beginnt (Browser-Zeit)
  let shotUpdateIntervalId = null; // ID für setInterval
  let hideShotInfoTimeoutId = null;// ID für setTimeout zum Ausblenden

  function sendDashboardAction(action, value = null) {
    const formData = new FormData();
    formData.append('action', action);
    if (value !== null) { formData.append('value', value); }
    showDashboardMessage('Verarbeite...', 'info', 2000);
    fetch('/dashboard-action', { method: 'POST', body: formData })
      .then(response => response.json())
      .then(data => {
        if (data.success) {
          console.log('Aktion erfolgreich:', action, value); // Behalte Konsolen-Log zum Debuggen
          showDashboardMessage(data.message || 'Aktion erfolgreich!', 'success');
          setTimeout(fetchDashboardData, 150); // Kurze Verzögerung vor dem Neuladen
        } else {
          console.error('Aktion fehlgeschlagen:', action, value, data.message);
          showDashboardMessage(data.message || 'Aktion fehlgeschlagen!', 'error');
          setTimeout(fetchDashboardData, 1000);
        }
      })
      .catch(error => {
        console.error('Fehler bei Aktion:', action, error);
        showDashboardMessage('Kommunikationsfehler!', 'error');
        setTimeout(fetchDashboardData, 1000);
      });
  }

  let messageTimeoutId = null;
  function showDashboardMessage(message, type = 'info', duration = 4000) {
       const msgElement = document.getElementById('dashboard-message');
       if (!msgElement) return;
       clearTimeout(messageTimeoutId);
       msgElement.textContent = message;
       msgElement.className = ''; // Reset classes
       if (type === 'success') msgElement.classList.add('success');
       else if (type === 'error') msgElement.classList.add('error');
       else msgElement.classList.add('info');
       msgElement.style.display = 'block';
       messageTimeoutId = setTimeout(() => { msgElement.style.display = 'none'; }, duration);
  }

  // --- Timer Anzeige Logik ---
  function updateShotTimerDisplay() {
      // Prüft, ob die JS-Variable einen Startzeitpunkt enthält
      if (jsShotStartTime > 0) {
          // Berechnet Differenz zur *aktuellen Browser-Zeit*
          const elapsedSeconds = (Date.now() - jsShotStartTime) / 1000;
          const timerDisplay = document.getElementById('shot-timer-display');
          if(timerDisplay) timerDisplay.textContent = elapsedSeconds.toFixed(1) + 's';
      }
  }

  // --- Timer Intervall stoppen ---
  function stopShotTimerInterval() {
      if (shotUpdateIntervalId) {
          clearInterval(shotUpdateIntervalId);
          shotUpdateIntervalId = null;
      }
  }

  function processDashboardData(data) {
    // Elemente holen
    const tempWActual = document.getElementById('tempW_actual'); 
    const tempWTarget = document.getElementById('tempW_target'); 
    const tempDActual = document.getElementById('tempD_actual'); 
    const tempDTarget = document.getElementById('tempD_target');
    const statusSpan = document.getElementById('machine_status');
    const shotInfoDiv = document.getElementById('shot-info'); 
    const weightDisplay = document.getElementById('shot-weight-display'); 
    const timerDisplay = document.getElementById('shot-timer-display');
    const setpointWInput = document.getElementById('setpointW_input'); 
    const setpointDInput = document.getElementById('setpointD_input');
    const ecoStopButton = document.getElementById('eco_stop_button');
    const maintToggle = document.getElementById('maintenance_toggle'); 
    const boostWToggle = document.getElementById('boostW_toggle'); 
    const boostDToggle = document.getElementById('boostD_toggle');
    const steamNowButton = document.getElementById('steam_now_button');
    const piToggle = document.getElementById('pi_toggle');
    const bbtToggle = document.getElementById('bbt_toggle');
    const bbwToggle = document.getElementById('bbw_toggle');
    const scaleModeToggle = document.getElementById('scale_mode_toggle');
    const tareButton = document.getElementById('tare_button');

    // Temperaturen anzeigen & Dampf-Sollwert
    if (tempWActual) tempWActual.textContent = data.tempW.toFixed(1);
    if (tempWTarget) tempWTarget.textContent = 'Soll: ' + data.setW.toFixed(1) + ' \u00B0C'; // \u00B0 ist °
    if (tempDActual) tempDActual.textContent = data.tempD.toFixed(1);
    if (tempDTarget) { if (data.steamDelayActiveNow === true && !data.shotActive) { tempDTarget.textContent = 'Startverz\u00F6gerung'; tempDTarget.style.fontStyle = 'italic'; } else { tempDTarget.textContent = 'Soll: ' + data.setD.toFixed(1) + ' \u00B0C'; tempDTarget.style.fontStyle = 'normal'; } }

    // --- Verfeinerte Farb-Logik START (v3.8) ---
    const setClass = (el, baseClass, stateClass) => { if(el) { el.className = baseClass; if(stateClass) el.classList.add(stateClass); } };
    const waterReadyThreshold = 1.0;
    const waterHeatThreshold = 0.5;
    const steamReadyThreshold = 2.0;
    const steamHeatThreshold = 1.5;

    // Wasser Status
    let waterStateClass = '';
    if (data.errorW || data.safetyW) { waterStateClass = 'error'; }
    else if (data.maintenanceActive) { waterStateClass = 'maintenance'; }
    else if (data.ecoActive) { // Ist der Eco-Kühlmodus gerade aktiv?
        if (Math.abs(data.tempW - data.setW) < waterReadyThreshold) { waterStateClass = 'eco'; } // Nahe Eco-Temp -> blau
        else if (data.tempW > data.setW) { waterStateClass = 'eco'; } // Zeigt blau während des Abkühlens
        else { waterStateClass = 'heating'; } // Zeigt orange, wenn es zur Eco-Temp aufheizt
    }
    else if (data.tempW < data.setW - waterHeatThreshold) { waterStateClass = 'heating'; } // Normales Heizen
    else if (Math.abs(data.tempW - data.setW) < waterReadyThreshold) { waterStateClass = 'ready'; } // Normal Bereit
    setClass(tempWActual, 'temp-actual', waterStateClass);

    // Dampf Status
    let steamStateClass = '';
    if (data.errorD || data.safetyD) { steamStateClass = 'error'; }
    else if (data.maintenanceActive) { steamStateClass = 'maintenance'; }
    else if (data.ecoActive) { // Ist der Eco-Kühlmodus gerade aktiv?
        if (Math.abs(data.tempD - data.setD) < steamReadyThreshold) { steamStateClass = 'eco'; } // Nahe Eco-Temp
        else if (data.tempD > data.setD) { steamStateClass = 'eco'; } // Kühlt ab
        else { steamStateClass = 'heating'; } // Heizt auf Eco-Temp
    }
    else if (data.steamDelayActiveNow) { steamStateClass = ''; } // Wenn Verzögerung aktiv ist -> Standardfarbe (weiß)
    else if (data.tempD < data.setD - steamHeatThreshold) { steamStateClass = 'heating'; } // Normales Heizen
    else if (Math.abs(data.tempD - data.setD) < steamReadyThreshold) { steamStateClass = 'ready'; } // Normal Bereit
    setClass(tempDActual, 'temp-actual', steamStateClass);
    // --- Verfeinerte Farb-Logik ENDE ---


    // Status Text
    if (statusSpan) { statusSpan.textContent = data.statusText || 'Unbekannt'; statusSpan.classList.toggle('error', data.errorW || data.errorD || data.safetyW || data.safetyD); }

    // *** SHOT INFO & TIMER LOGIK mit 3s Verzögerung (v3.10) ***
    if (shotInfoDiv) {
        if (data.shotActive) { // Shot läuft laut Backend
            // Lösche einen eventuell laufenden Timeout zum Ausblenden
            if (hideShotInfoTimeoutId) {
                clearTimeout(hideShotInfoTimeoutId);
                hideShotInfoTimeoutId = null;
            }
            shotInfoDiv.style.display = 'block'; // Sicherstellen, dass sichtbar

            // Wenn der Timer im JS noch nicht gestartet wurde...
            if (jsShotStartTime === 0) {
                 console.log("Shot start detected by JS.");
                 jsShotStartTime = Date.now(); // ...merke dir die Browser-Startzeit
                 stopShotTimerInterval(); // Sicherheitshalber alten Timer löschen
                 updateShotTimerDisplay(); // Sofort anzeigen
                 shotUpdateIntervalId = setInterval(updateShotTimerDisplay, 100); // Starte Update-Intervall
            }
            // Gewichtsanzeige
            if (weightDisplay && data.weight !== undefined) { weightDisplay.textContent = `Gewicht: ${data.weight.toFixed(1)} / ${data.targetWeight.toFixed(1)} g`; }
            else if (weightDisplay) { weightDisplay.textContent = 'Gewicht: --.- g'; }

        } else { // Shot läuft NICHT laut Backend
            // Wenn der Timer im JS aber noch lief...
            if (jsShotStartTime !== 0) {
                 console.log("Shot stop detected by JS.");
                 stopShotTimerInterval(); // Stoppe das Update-Intervall

                 // Berechne und zeige finale Dauer AN
                 const finalDurationSeconds = (Date.now() - jsShotStartTime) / 1000;
                 if(timerDisplay) {
                     timerDisplay.textContent = finalDurationSeconds.toFixed(1) + 's';
                 }

                 jsShotStartTime = 0; // Setze Browser-Startzeit zurück

                 // Setze Timeout zum Ausblenden des Bereichs in 3 Sekunden
                 // (Lösche evtl. alten Timeout zuerst, falls Stop-Events sehr schnell aufeinander folgen)
                 if (hideShotInfoTimeoutId) { clearTimeout(hideShotInfoTimeoutId); }
                 hideShotInfoTimeoutId = setTimeout(() => {
                     if(shotInfoDiv) { shotInfoDiv.style.display = 'none'; }
                     hideShotInfoTimeoutId = null; // ID zurücksetzen nach Ausführung
                 }, 3000); // 3000ms = 3 Sekunden
            }
            // Verstecke den Bereich NICHT sofort, das macht der Timeout
        }
    }
    // *** ENDE SHOT INFO & TIMER LOGIK ***


    // Input Felder (nur wenn nicht fokussiert)
    if (setpointWInput && document.activeElement !== setpointWInput) setpointWInput.value = data.setW.toFixed(1);
    if (setpointDInput && document.activeElement !== setpointDInput) setpointDInput.value = data.setD.toFixed(1);

    // --- Update toggle states DIREKT ---
    const updateToggleState = (element, dataValue) => { if (element) { element.checked = dataValue === true || dataValue === 'true'; }};
    updateToggleState(maintToggle, data.maintenanceActive);
    updateToggleState(boostWToggle, data.boostWActive);
    updateToggleState(boostDToggle, data.boostDActive);
    updateToggleState(piToggle, data.piEnabled);
    updateToggleState(bbtToggle, data.bbtEnabled);
    updateToggleState(bbwToggle, data.bbwEnabled);
    //BBW Toggle basierend auf Waagenstatus aktivieren/deaktivieren ***
    if (bbwToggle) {
        // Deaktiviere, wenn 'scaleConnected' fehlt ODER explizit false ist
        bbwToggle.disabled = (data.scaleConnected === undefined || data.scaleConnected === false);
    }
    updateToggleState(scaleModeToggle, data.scaleModeActive);
    // Waage-Modus Toggle basierend auf Waagenstatus aktivieren/deaktivieren
    if (scaleModeToggle) {
        scaleModeToggle.disabled = (data.scaleConnected === undefined || data.scaleConnected === false);
    }

    // --- Ende Toggle Update ---

    // --- Update button states ---
    if (ecoStopButton) { ecoStopButton.disabled = !data.ecoActive; }
    if (steamNowButton) { steamNowButton.disabled = !data.steamDelayActiveNow; }
    if (tareButton) { tareButton.disabled = !data.scaleConnected; }
  }

  function fetchDashboardData() { 
    fetch('/dashboard-data').then(response => { 
      if (!response.ok) { throw new Error('Network response was not ok'); } return response.json(); })
      .then(data => { processDashboardData(data); })
      .catch(error => { console.error('Fehler beim Abrufen der Dashboard-Daten:', error); 
      const statusSpan = document.getElementById('machine_status'); 
      if(statusSpan) statusSpan.textContent = 'Fehler bei Datenabruf!'; }); }

  // --- Event Listener Setup ---
  document.addEventListener('DOMContentLoaded', () => { document.getElementById('setpointW_save').addEventListener('click', () => { sendDashboardAction('setTempW', document.getElementById('setpointW_input').value); }); document.getElementById('setpointD_save').addEventListener('click', () => { sendDashboardAction('setTempD', document.getElementById('setpointD_input').value); });
  const ecoStopBtn = document.getElementById('eco_stop_button'); 
  if (ecoStopBtn) { ecoStopBtn.addEventListener('click', () => { sendDashboardAction('stopEco'); }); } 
  const maintToggleListener = document.getElementById('maintenance_toggle');
  if (maintToggleListener) { maintToggleListener.addEventListener('change', (event) => { sendDashboardAction('toggleMaintenance', event.target.checked); }); } 
  const boostWToggleListener = document.getElementById('boostW_toggle'); 
  if (boostWToggleListener) { boostWToggleListener.addEventListener('change', (event) => { sendDashboardAction('toggleBoostW', event.target.checked); }); } 
  const boostDToggleListener = document.getElementById('boostD_toggle'); 
  if (boostDToggleListener) { boostDToggleListener.addEventListener('change', (event) => { sendDashboardAction('toggleBoostD', event.target.checked); }); } 
  const piToggle = document.getElementById('pi_toggle'); 
  if (piToggle) { piToggle.addEventListener('change', (event) => { sendDashboardAction('togglePI', event.target.checked); }); } 
  const bbtToggle = document.getElementById('bbt_toggle'); 
  if (bbtToggle) { bbtToggle.addEventListener('change', (event) => { sendDashboardAction('toggleBBT', event.target.checked); }); } 
  const bbwToggle = document.getElementById('bbw_toggle'); 
  if (bbwToggle) { bbwToggle.addEventListener('change', (event) => { sendDashboardAction('toggleBBW', event.target.checked); }); } document.getElementById('profile_load_button').addEventListener('click', () => { const select = document.getElementById('profile_select');
  const profileName = select.value; 
  if (profileName) { sendDashboardAction('loadProfile', profileName); } else { showDashboardMessage('Bitte zuerst ein Profil auswählen.', 'error'); } });
  const steamBtn = document.getElementById('steam_now_button'); 
  if (steamBtn) { steamBtn.addEventListener('click', () => { sendDashboardAction('overrideSteamDelay'); }); } 
  const tareBtn = document.getElementById('tare_button'); 
  if (tareBtn) { tareBtn.addEventListener('click', () => { sendDashboardAction('tareScale'); }); } fetchDashboardData(); dashboardIntervalId = setInterval(fetchDashboardData, updateInterval); });
  const scaleModeToggleListener = document.getElementById('scale_mode_toggle');
  if (scaleModeToggleListener) { scaleModeToggleListener.addEventListener('change', (event) => { sendDashboardAction('toggleScaleMode', event.target.checked); }); }
</script>
)rawliteral";

static const char dashboardBodyEnd[] PROGMEM = R"rawliteral(
</body></html>
)rawliteral";


// =============================================================================
// C++ Handler Funktionen für das Dashboard
// =============================================================================

// -----------------------------------------------------------------------------
// Handler für die Dashboard-Seite (GET /Dashboard) - v3.5
// -----------------------------------------------------------------------------
void handleDashboard(AsyncWebServerRequest *request) {
    AsyncResponseStream *response = request->beginResponseStream("text/html; charset=utf-8");

    response->print(FPSTR(dashboardHtmlHead));
    response->print(FPSTR(commonStyle)); // Stelle sicher, dass commonStyle existiert
    response->print(FPSTR(dashboardStyles)); // Enthält </head>
    response->print(FPSTR(dashboardBodyStart)); // <body>
    response->print(FPSTR(commonNav)); // Stelle sicher, dass commonNav existiert

    // Main Display bis Ende Shot Info
    response->print(FPSTR(dashboardContainerStart));
#ifdef ESP32
    // Optional: Gewichtsanzeige im Main Display
    response->print(FPSTR(dashboardShotWeight));
#endif

    // Control Panel HTML holen (enthält jetzt Eco Button statt Toggle)
    String controlPanelHtml = FPSTR(dashboardControlPanelHTML);

    // Profil-Optionen generieren und einfügen
    std::vector<String> profiles = listProfiles();
    String profileOptions = "";
    if (!profiles.empty()) {
        for (const String& name : profiles) {
            String nameHtmlEncoded = name;
            nameHtmlEncoded.replace("\"", "&quot;");
            profileOptions += "<option value=\"" + nameHtmlEncoded + "\">" + nameHtmlEncoded + "</option>";
            yield();
        }
    }
    controlPanelHtml.replace("{PROFILE_OPTIONS}", profileOptions);
    response->print(controlPanelHtml); // Sendet den Control Panel Teil BIS zum optionalen Brew Control

#ifdef ESP32
    // ESP32 spezifischen Brew Control Abschnitt senden
    response->print(FPSTR(dashboardBrewControlSection));
#endif

    // Restliche Container schließen
    response->print(FPSTR(dashboardEndContainer));

    // JavaScript einfügen
    response->print(FPSTR(dashboardJavaScript));

    // Body / HTML schließen
    response->print(FPSTR(dashboardBodyEnd));

    request->send(response);
}


// -----------------------------------------------------------------------------
// Handler für Aktionen vom Dashboard (POST /dashboard-action) - v3.5 Eco Button
// -----------------------------------------------------------------------------
void handleDashboardAction(AsyncWebServerRequest *request) {
    if (!request->hasArg("action")) {
        request->send(400, "application/json", "{\"success\":false, \"message\":\"Aktion fehlt!\"}");
        return;
    }
    String action = request->arg("action");
    String value = request->hasArg("value") ? request->arg("value") : "";
    bool success = false;
    String message = "";
    bool settingsChanged = false; // Flag für EEPROM Commit
    bool pidNeedsUpdate = false;  // Flag für PID Aktualisierung

    // Serial.printf("Dashboard Aktion: %s, Wert: %s\n", action.c_str(), value.c_str());

    // --- Temperatur-Änderungen ---
    if (action == "setTempW") {
        float newVal = value.toFloat();
        if (newVal >= 0 && newVal <= 135) {
            if (abs(newVal - SetpointWasser) > 0.01) {
                 SetpointWasser = newVal;
                 EEPROM.put(EEPROM_ADDR_SETPOINT_WASSER, SetpointWasser);
                 settingsChanged = true; pidNeedsUpdate = true;
                 message = "Wasser Sollwert auf " + String(newVal, 1) + " °C gesetzt.";
                 success = true;
            } else { message = "Wasser Sollwert bereits gesetzt."; success = true; }
        } else { message = "Ungültiger Wasser Sollwert."; }
    } else if (action == "setTempD") {
        float newVal = value.toFloat();
        if (newVal >= 0 && newVal <= 200) {
             if (abs(newVal - SetpointDampf) > 0.01) {
                SetpointDampf = newVal;
                EEPROM.put(EEPROM_ADDR_SETPOINT_DAMPF, SetpointDampf);
                settingsChanged = true; pidNeedsUpdate = true;
                message = "Dampf Sollwert auf " + String(newVal, 1) + " °C gesetzt.";
                success = true;
            } else { message = "Dampf Sollwert bereits gesetzt."; success = true; }
        } else { message = "Ungültiger Dampf Sollwert."; }

    // --- Eco Modus nur für diesen Zyklus beenden (Button) ---
    } else if (action == "stopEco") {
        //  Serial.println("[Action: stopEco] Request received.");
         // Nur ausführen, wenn Eco gerade aktiv kühlt
         if (ecoModeAktiv) {
             // Zeit für letzten Bezug auf jetzt setzen, um den Eco-Modus temporär zu deaktivieren
             lastShotTime = millis();
             // Normale Temperaturen wiederherstellen (aus EEPROM)
             EEPROM.get(EEPROM_ADDR_SETPOINT_WASSER, SetpointWasser);
             EEPROM.get(EEPROM_ADDR_SETPOINT_DAMPF, SetpointDampf);
             pidNeedsUpdate = true; // PID muss die neuen Setpoints bekommen
             // *** KEINE ÄNDERUNG MEHR AN ecoModeMinutes oder EEPROM hier! ***
             // settingsChanged = false; // Sicherstellen, dass kein unnötiger Commit erfolgt
             message = "Eco Modus für diesen Zyklus beendet."; // Angepasste Meldung
             success = true;
            //  Serial.println("[Action: stopEco] Deactivated running Eco cooling. Timer setting remains unchanged.");
         } else {
            message = "Eco Modus ist derzeit nicht aktiv."; // Angepasste Meldung
            success = true; // Keine Aktion nötig, aber kein Fehler
            // Serial.println("[Action: stopEco] Eco currently not active, no action taken.");
         }
    } // --- Ende stopEco ---

    // --- Wartungsmodus Toggle ---
    else if (action == "toggleMaintenance") {
        bool newState = (value == "true");
        if (newState != wartungsModusAktiv) {
             wartungsModusAktiv = newState;
             if (!wartungsModusAktiv) { // Deaktiviert -> Normale Temps laden
                  EEPROM.get(EEPROM_ADDR_SETPOINT_WASSER, SetpointWasser);
                  EEPROM.get(EEPROM_ADDR_SETPOINT_DAMPF, SetpointDampf);
             } else { // Aktiviert -> Wartungstemps setzen
                  SetpointWasser = wartungsModusTemp;
                  SetpointDampf = wartungsModusTemp;
             }
             pidNeedsUpdate = true; // Weil Setpoints geändert wurden
             message = String("Wartungsmodus ") + (wartungsModusAktiv ? "aktiviert." : "deaktiviert.");
             success = true;
             // Kein EEPROM Commit, da temporär
        } else { message="Wartungsmodus unverändert."; success=true;}

    // --- Heiz-Optionen (Persistent) ---
     } else if (action == "toggleBoostW") {
        bool newState = (value == "true");
        if (newState != boostWasserActive) {
            boostWasserActive = newState;
            EEPROM.put(EEPROM_ADDR_BOOST_WASSER_ACTIVE, boostWasserActive);
            settingsChanged = true;
            message = String("Boost Wasser ") + (boostWasserActive ? "aktiviert." : "deaktiviert.");
            success = true;
        } else { message="Boost Wasser unverändert."; success = true; }
    } else if (action == "toggleBoostD") {
        bool newState = (value == "true");
         if (newState != boostDampfActive) {
            boostDampfActive = newState;
            EEPROM.put(EEPROM_ADDR_BOOST_DAMPF_ACTIVE, boostDampfActive);
            settingsChanged = true;
            message = String("Boost Dampf ") + (boostDampfActive ? "aktiviert." : "deaktiviert.");
            success = true;
         } else { message="Boost Dampf unverändert."; success = true; }
    } else if (action == "overrideSteamDelay") {
        steamDelayOverridden = true; // Globale Variable setzen
        message = "Dampfverzögerung temporär übersprungen.";
        success = true;
        // Kein EEPROM Commit, da temporär

    // --- Profil Laden ---
     } else if (action == "loadProfile") {
        TemperatureProfile loadedData;
        String profileNameToLoad = value;
        profileNameToLoad.replace("&quot;", "\""); // Einfaches HTML Decoding

        if (loadProfile(profileNameToLoad, loadedData)) {
            applyProfileSettings(loadedData); // Wendet an und speichert EEPROM
            message = "Profil '" + String(loadedData.profileName) + "' geladen.";
            success = true;
            settingsChanged = false; // Bereits in applyProfileSettings erledigt
            pidNeedsUpdate = false;  // Bereits in applyProfileSettings erledigt
        } else {
             message = "Fehler beim Laden von Profil '" + profileNameToLoad + "'.";
        }

    // --- Brew Control Toggles (Persistent - ESP32 only) ---
#ifdef ESP32
    } else if (action == "togglePI") {
        bool newState = (value == "true");
        if (newState != preInfusionEnabled) {
            preInfusionEnabled = newState;
            EEPROM.put(EEPROM_ADDR_PREINF_ENABLED, preInfusionEnabled);
            settingsChanged = true;
            message = String("Pre-Infusion ") + (preInfusionEnabled ? "aktiviert." : "deaktiviert.");
            success = true;
        } else { message="Pre-Infusion unverändert."; success=true;}
    } else if (action == "toggleBBT") {
        bool newState = (value == "true");
         if (newState != brewByTimeEnabled) {
            brewByTimeEnabled = newState;
            EEPROM.put(EEPROM_ADDR_BREWBYTIME_ENABLED, brewByTimeEnabled);
            settingsChanged = true;
            message = String("Brew-By-Time ") + (brewByTimeEnabled ? "aktiviert." : "deaktiviert.");
            success = true;
         } else { message="Brew-By-Time unverändert."; success=true;}
    } else if (action == "toggleBBW") {
        bool newState = (value == "true");
        if (newState != brewByWeightEnabled) {
            brewByWeightEnabled = newState;
            EEPROM.put(EEPROM_ADDR_BREWBYWEIGHT_ENABLED, brewByWeightEnabled);
            settingsChanged = true;
            message = String("Brew-By-Weight ") + (brewByWeightEnabled ? "aktiviert." : "deaktiviert.");
            success = true;
        } else { message="Brew-By-Weight unverändert."; success=true;}

    // --- Waage Aktion (ESP32 only) ---
  } else if (action == "tareScale") {
    #if (SCALE_TYPE == SCALE_I2C)
      if (scaleConnected) {
          if (!tareScaleAfterDelay && !tareScaleSettling) {
              tareScaleAfterDelay = true;
              tareScaleDelayStartTime = millis();
          }
          message = "Tara eingeleitet.";
          success = true;
      } else { message = "I2C-Waage nicht verbunden."; }
    #elif (SCALE_TYPE == SCALE_ESPNOW)
      message = "Tarieren via UI nur für I2C-Waage.";
      success = false; // Aktion nicht erfolgreich, da nicht unterstützt
    #endif
  } else if (action == "toggleScaleMode") {
    bool newState = (value == "true");
    if (newState != scaleModeActive) {
        scaleModeActive = newState;
        // KEIN EEPROM Commit, da dies ein temporärer Modus ist
        // KEIN settingsChanged = true;
        message = String("Waage Modus ") + (scaleModeActive ? "aktiviert." : "deaktiviert.");
        success = true;
        // Serial.println(message); // Log-Ausgabe
    } else {
        message="Waage Modus unverändert.";
        success=true;
    }
#endif // ESP32

    // --- Unbekannte Aktion ---
    } else { message = "Unbekannte Aktion."; }

    // --- Abschlussaktionen ---
    if (settingsChanged) {
        EEPROM.commit();
    }
    if (pidNeedsUpdate) {
        pidWasser.SetTunings(KpWasser, KiWasser, KdWasser);
        pidDampf.SetTunings(KpDampf, KiDampf, KdDampf);
        // Serial.println("PID-Parameter nach Dashboard-Aktion aktualisiert.");
    }

    // JSON Antwort senden
    char responseBuf[200];
    message.replace("\"", "'");
    snprintf(responseBuf, sizeof(responseBuf), "{\"success\":%s, \"message\":\"%s\"}",
             success ? "true" : "false", message.c_str());
    request->send(200, "application/json", responseBuf);
}


// -----------------------------------------------------------------------------
// Handler für Dashboard-Daten (GET /dashboard-data) - v3.6 Korrigierte #ifdef
// -----------------------------------------------------------------------------
void handleDashboardData(AsyncWebServerRequest *request) {
    char buffer[650]; // Puffer ggf. anpassen
    char floatBuf[10];

    // --- Status bestimmen ---
    String statusText = "Bereit"; String statusKey = "ready";

    if (wasserSensorError || dampfSensorError || wasserSafetyShutdown || dampfSafetyShutdown) {
        statusText = "";
         if (wasserSensorError) statusText += "Fehler Wasser-Sensor! ";
         if (dampfSensorError) statusText += "Fehler Dampf-Sensor! ";
         if (wasserSafetyShutdown) statusText += "Sicherheitsabsch. Wasser! ";
         if (dampfSafetyShutdown) statusText += "Sicherheitsabsch. Dampf! ";
         statusKey = "error";
    } else if (wartungsModusAktiv) {
        statusText = "Wartungsmodus aktiv";
        statusKey = "maintenance";
    } else if (autoTuneWasserActive) {
        statusText = "PID Tuning Wasser...";
        statusKey = "tuning";
    } else if (autoTuneDampfActive) {
        statusText = "PID Tuning Dampf...";
        statusKey = "tuning";
    } else if (shotActive) {
        // *** KORREKTUR START: #ifdef auf eigenen Zeilen ***
#ifdef ESP32
        switch (currentPreInfusionState) {
            case PI_PRE_BREW: statusText = "Pre-Infusion..."; break;
            case PI_PAUSE:    statusText = "Pre-Infusion Pause..."; break;
            case PI_MAIN_BREW:statusText = "Bezug aktiv..."; break;
            default:          statusText = "Bezug aktiv..."; break;
        }
#else // Wenn nicht ESP32
        statusText = "Bezug aktiv...";
#endif
        statusKey = "brewing";
        // *** KORREKTUR ENDE ***
    } else if (ecoModeAktiv) {
         statusText = dynamicEcoActive ? "Eco+ Modus aktiv (Kühlen)" : "Eco Modus aktiv (Kühlen)";
         statusKey = "eco";
    } else if (fastHeatUpHeating) {
         statusText = "Fast Heat-Up aktiv..."; // Logik bleibt, auch wenn Toggle weg ist
         statusKey = "fastheatup";
    } else if (digitalRead(SSR_WASSER_PIN) == HIGH || digitalRead(SSR_DAMPF_PIN) == HIGH) {
         statusText = "Heizen...";
         statusKey = "heating";
    } else if (ecoModeMinutes > 0 && !ecoModeAktiv) { // Nur anzeigen wenn Timer gesetzt, aber nicht aktiv kühlt
         statusText = "Bereit (Eco Timer: " + String(ecoModeMinutes) + "min)";
         statusKey = "ready_eco_pending";
    }
    // Ansonsten bleibt statusText = "Bereit"

    // --- Zeit holen ---
    String currentTime = timeSynced ? timeClient.getFormattedTime() : "N/A";

    // --- Dampfverzögerung prüfen ---
    bool steamDelayIsCurrentlyActive = (dampfVerzoegerung > 0 && !steamDelayOverridden && (millis() - startupTime < (unsigned long)dampfVerzoegerung * 60000UL));

    // --- JSON zusammensetzen ---
    int offset = 0; // Wichtig: offset muss hier deklariert werden!
    offset += snprintf(buffer + offset, sizeof(buffer) - offset, "{");

    // Temperaturen
    dtostrf(InputWasser, 4, 1, floatBuf); offset += snprintf(buffer + offset, sizeof(buffer) - offset, "\"tempW\":%s,", floatBuf);
    dtostrf(SetpointWasser, 4, 1, floatBuf); offset += snprintf(buffer + offset, sizeof(buffer) - offset, "\"setW\":%s,", floatBuf);
    dtostrf(InputDampf, 5, 1, floatBuf); offset += snprintf(buffer + offset, sizeof(buffer) - offset, "\"tempD\":%s,", floatBuf);
    dtostrf(SetpointDampf, 5, 1, floatBuf); offset += snprintf(buffer + offset, sizeof(buffer) - offset, "\"setD\":%s,", floatBuf);

    // Status
    statusText.replace("\"", "\\\"");
    offset += snprintf(buffer + offset, sizeof(buffer) - offset, "\"statusText\":\"%s\",", statusText.c_str());
    offset += snprintf(buffer + offset, sizeof(buffer) - offset, "\"statusKey\":\"%s\",", statusKey.c_str());

    // Shot-Daten
    offset += snprintf(buffer + offset, sizeof(buffer) - offset, "\"shotActive\":%s,", shotActive ? "true" : "false");
    offset += snprintf(buffer + offset, sizeof(buffer) - offset, "\"shotStart\":%llu,", (unsigned long long)shotStartTime);

    // Modus-Flags
    offset += snprintf(buffer + offset, sizeof(buffer) - offset, "\"ecoActive\":%s,", ecoModeAktiv ? "true" : "false"); // Wird für Button :disabled benötigt
    offset += snprintf(buffer + offset, sizeof(buffer) - offset, "\"maintenanceActive\":%s,", wartungsModusAktiv ? "true" : "false");
    offset += snprintf(buffer + offset, sizeof(buffer) - offset, "\"boostWActive\":%s,", boostWasserActive ? "true" : "false");
    offset += snprintf(buffer + offset, sizeof(buffer) - offset, "\"boostDActive\":%s,", boostDampfActive ? "true" : "false");
    offset += snprintf(buffer + offset, sizeof(buffer) - offset, "\"steamDelayActiveNow\":%s,", steamDelayIsCurrentlyActive ? "true" : "false");

    // Fehler-Flags
    offset += snprintf(buffer + offset, sizeof(buffer) - offset, "\"errorW\":%s,", wasserSensorError ? "true" : "false");
    offset += snprintf(buffer + offset, sizeof(buffer) - offset, "\"errorD\":%s,", dampfSensorError ? "true" : "false");
    offset += snprintf(buffer + offset, sizeof(buffer) - offset, "\"safetyW\":%s,", wasserSafetyShutdown ? "true" : "false");
    offset += snprintf(buffer + offset, sizeof(buffer) - offset, "\"safetyD\":%s,", dampfSafetyShutdown ? "true" : "false");

    // ESP32 spezifische Daten
#ifdef ESP32
    dtostrf(currentWeightReading, 4, 1, floatBuf); offset += snprintf(buffer + offset, sizeof(buffer) - offset, "\"weight\":%s,", floatBuf);
    dtostrf(brewByWeightTargetGrams, 4, 1, floatBuf); offset += snprintf(buffer + offset, sizeof(buffer) - offset, "\"targetWeight\":%s,", floatBuf);
    offset += snprintf(buffer + offset, sizeof(buffer) - offset, "\"scaleConnected\":%s,", scaleConnected ? "true" : "false");
    offset += snprintf(buffer + offset, sizeof(buffer) - offset, "\"piEnabled\":%s,", preInfusionEnabled ? "true" : "false");
    offset += snprintf(buffer + offset, sizeof(buffer) - offset, "\"bbtEnabled\":%s,", brewByTimeEnabled ? "true" : "false");
    offset += snprintf(buffer + offset, sizeof(buffer) - offset, "\"bbwEnabled\":%s,", brewByWeightEnabled ? "true" : "false");
    offset += snprintf(buffer + offset, sizeof(buffer) - offset, "\"scaleModeActive\":%s,", scaleModeActive ? "true" : "false");
#else
    offset += snprintf(buffer + offset, sizeof(buffer) - offset, "\"weight\":0.0,"); offset += snprintf(buffer + offset, sizeof(buffer) - offset, "\"targetWeight\":0.0,"); offset += snprintf(buffer + offset, sizeof(buffer) - offset, "\"scaleConnected\":false,"); offset += snprintf(buffer + offset, sizeof(buffer) - offset, "\"piEnabled\":false,"); offset += snprintf(buffer + offset, sizeof(buffer) - offset, "\"bbtEnabled\":false,"); offset += snprintf(buffer + offset, sizeof(buffer) - offset, "\"bbwEnabled\":false,");
#endif

    // Letztes Element ohne Komma
    offset += snprintf(buffer + offset, sizeof(buffer) - offset, "\"currentTime\":\"%s\"", currentTime.c_str());

    offset += snprintf(buffer + offset, sizeof(buffer) - offset, "}");

    // JSON senden
    request->send(200, "application/json", buffer);
}

// =============================================================================
// ENDE: Dashboard Code Block
// =============================================================================


/************************************************************************************
 * Handler für Button der Waage
 ************************************************************************************/

#if (defined(ESP32) && (SCALE_TYPE == SCALE_I2C))
/**
 * Verarbeitet den Tastendruck des Waagen-Buttons.
 * Unterscheidet zwischen kurzem (Tarieren mit Verzögerung) und langem Druck (scaleModeActive umschalten).
 * Verwendet invertierte Button-Logik (0 = gedrückt).
 */
void handleScaleButtonPress(AsyncWebServerRequest *request) { // Umbenannt zur Klarheit, dass es um den Press-Event geht
    (void)request;
    if (!scaleConnected) {
        return;
    }

    uint8_t currentRawButtonState = scales.getBtnStatus();

    // --- Entprellung ---
    if (currentRawButtonState != lastScaleButtonRawState) {
        lastScaleButtonDebounceTime = millis();
    }
    lastScaleButtonRawState = currentRawButtonState;

    if ((millis() - lastScaleButtonDebounceTime) > debounceDelayScale) {
        if (currentRawButtonState != currentScaleButtonDebouncedState) {
            currentScaleButtonDebouncedState = currentRawButtonState;

            // Invertierte Logik: 0 = GEDRÜCKT, 1 (oder !=0) = NICHT GEDRÜCKT
            if (currentScaleButtonDebouncedState == 0 && lastScaleButtonDebouncedState != 0) {
                // Fallende Flanke: Button wurde gerade GEDRÜCKT
                scaleButtonPressStartTime = millis();
                scaleButtonIsCurrentlyPressed = true;
                // Serial.println(F("Waagen-Button gedrückt")); // Für Debugging
            } else if (currentScaleButtonDebouncedState != 0 && lastScaleButtonDebouncedState == 0) {
                // Steigende Flanke: Button wurde gerade LOSGELASSEN
                if (scaleButtonIsCurrentlyPressed) {
                    unsigned long pressDuration = millis() - scaleButtonPressStartTime;
                    // Serial.print(F("Waagen-Button losgelassen. Dauer: ")); Serial.println(pressDuration); // Für Debugging

                    if (pressDuration >= longPressThresholdScale) {
                        // Langer Druck
                        scaleModeActive = !scaleModeActive;
                        // Serial.print(F("Langer Druck: scaleModeActive = ")); Serial.println(scaleModeActive);
                        if (piezoEnabled) {
                            beepShort(scaleModeActive ? 3500 : 3200, 70); // Unterschiedliche Töne für an/aus
                        }
                    } else {
                        // Kurzer Druck: Tarieren nach Verzögerung anstoßen
                        if (scaleConnected && !tareScaleAfterDelay) { // Verhindere mehrfaches Auslösen der Verzögerung
                            // Serial.println(F("Kurzer Druck: Starte Verzögerung für Tara."));
                            tareScaleAfterDelay = true;
                            tareScaleDelayStartTime = millis();
                            // Der eigentliche Tara-Befehl erfolgt in loop() nach der Verzögerung
                        }
                    }
                }
                scaleButtonIsCurrentlyPressed = false;
            }
        }
    }
    lastScaleButtonDebouncedState = currentScaleButtonDebouncedState;
}

#endif