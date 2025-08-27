# Dual-PID Controller

**Präzision. Kontrolle. Perfekter Espresso.**

Open-Source-Firmware zur unabhängigen PID-Regelung von Wasser- und Dampftemperatur in Espressomaschinen.
Konfiguration und Überwachung erfolgen bequem über eine moderne Weboberfläche.

## To Do:
**Modularisierung des Codes**
- Code in strukturierte Module auslagern, für bessere Übersicht und Wartbarkeit
  
## Geplante Erweiterungen:
**Sensorik erweitern** 
- Drucksensor zur Brühdruckmessung fertig implementieren
- Ultraschallsensor zur Wasserstandskontrolle implementieren

## Highlights
- **Duale PID-Regelung** – präzise Temperatursteuerung für Wasser (Kessel/NTC) und Dampf (Thermoblock/MAX6675)
- **PID AutoTune** – automatische Ermittlung optimaler PID-Parameter per Knopfdruck
- **Intuitive Web-UI** – vollständige Kontrolle und Konfiguration via WLAN
- **Brew Control** – automatischer Bezugsstopp nach Zeit oder Gewicht
- **Flexible Pre-Infusion** – konfigurierbare Dauer und Pause
- **Live Temperatur-Charts** – grafische Echtzeit-Anzeige von Soll-/Ist-Temperaturen
- **Nutzerprofile** – speichern und laden Sie Profile für verschiedene Bohnen oder Zubereitungen
- **Backup & Restore** – sichern und wiederherstellen aller Einstellungen
- **OTA Firmware-Updates** – einfache Aktualisierung über das Webinterface

## Funktionen im Detail
### Kernfunktionen
- Dual-PID-Regelung für Wasser- und Dampftemperatur
- Zeitproportionale SSR-Ansteuerung
- WLAN-Konnektivität (Client- oder Access-Point-Modus)
- Umfangreiche, responsive Weboberfläche

### Komfort & Anpassung
- Brew-By-Time & Brew-By-Weight *(ab Platine V4 + ESP32, Waage über I2C Bus 1)*
- Pre-Infusion mit konfigurierbarer Pause
- Brühdruck- und Wassertanküberwachung *(geplant, ab Platine V4 + ESP32)*
- Systemtöne über Piezo-Summer *(ab Platine V4 + ESP32)*
- Nutzerprofile, Eco-Modus & ECO+
- Dampf-Startverzögerung, Fast-Heat-Up & Boost-Funktion
- Shot-Timer, Shot-Zähler und Betriebsstundenzähler
- Shot-/Bezugsverlauf und Nutzungsstatistik
- Live-Temperaturcharts und OLED-Display-Anzeige
- Konfigurierbare Geräteinfos

### Setup & Wartung
- PID AutoTune sowie manuelle PID-Konfiguration
- Einstellbare Sicherheitslimits
- Wartungs- und Reinigungserinnerung & -modus
- Persistente Speicherung aller Einstellungen im EEPROM
- OTA-Firmware-Updates und Backup/Restore der Einstellungen
- Einfache WiFi-Konfiguration (Hostname, SSID, Passwort, DHCP/Static IP)
- Sensorfehlererkennung mit Sicherheitsabschaltung

### Technische Basis
- **Plattform:** ESP8266 (Wemos D1 mini / Pro) oder ESP32 (D1 Mini NodeMCU, ab Firmware 2.9.5)
- **Sensoren:** NTC für Wasser, MAX6675 Thermoelement für Dampf
- **Display:** OLED (I2C), optimiert für 1,5" & 2,4"
- **Waage:** M5Stack-Miniwaageinheit (HX711) oder ESP-NOW-Waage (HX711)
- **Drucksensor:** 3,3V 15 Bar I2C Drucksensor (geplant)
- **Wasserstand:** NB1045 I2C Ranging Module (geplant)

## Hardware
- ESP8266 oder ESP32
- SSRs zur Heizungssteuerung
- Temperatursensoren (MAX6675 und NTC)
- Optional: OLED-Display (Adafruit_SH110X, Adafruit_GFX)
- Optional: Waage per ESP-NOW oder I2C

## Installation
1. Arduino IDE oder arduino-cli einrichten.
2. Benötigte Board-Pakete für ESP8266 oder ESP32 installieren.
3. Bibliotheken gemäß Sketch-Kommentaren hinzufügen.
   Für die Nutzung der I2C-Waage müssen zusätzlich die Skripte von M5Stack eingebunden werden:
   <https://github.com/m5stack/M5Unit-Miniscale/tree/main> (bei Verwendung der ESP-NOW-Waage nicht erforderlich).
4. `Dual_PID_FastHeatUp.ino` kompilieren und auf den Mikrocontroller flashen.

## Nutzung
Nach dem Flashen stellt der Controller eine WLAN-Verbindung her. 
Über die Weboberfläche können Temperaturen,
Profile und Wartungsfunktionen eingestellt werden.

## Links
- Projekt-Website: [https://pid.mueller.black](https://pid.mueller.black)
- Live-Demo: [https://pid-demo.mueller.black](https://pid-demo.mueller.black)
- Firmware-Download: [GitHub Releases](https://github.com/thomas-michael-mueller/dual-pid/releases)
- ESP-NOW-Waage: [ESP-NOW-Waage](https://github.com/thomas-michael-mueller/dual-pid-scale)

## Beiträge
Pull Requests und Issues sind willkommen.

## Lizenz
Dieses Projekt steht unter der [BSD-2-Klausel-Lizenz](LICENSE).
Die AutoTune-Bibliothek wurde von Brett Beauregard entwickelt: [Arduino-PID-AutoTune-Library](https://github.com/br3ttb/Arduino-PID-AutoTune-Library)
Dieses Projekt nutzt Chart.js (MIT-Lizenz) – siehe [Chart-Skript/LICENSE-Chart.js](Chart-Skript/LICENSE-Chart.js) oder den offiziellen Lizenztext unter [https://github.com/chartjs/Chart.js/blob/master/LICENSE.md](https://github.com/chartjs/Chart.js/blob/master/LICENSE.md).

