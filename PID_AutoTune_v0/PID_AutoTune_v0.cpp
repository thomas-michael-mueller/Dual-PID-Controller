/*
BSD 2-Clause License

Copyright (c) 2011, Brett Beauregard
Modifiziert von Thomas Müller, 2025
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <PID_AutoTune_v0.h>

// #define ATUNE_DEBUG  // Debug-Ausgaben aktivieren

PID_ATune::PID_ATune(double* Input, double* Output)
{
    input = Input;
    output = Output;
    controlType = 0; //default to PI
    noiseBand = 0.5;
    running = false;
    oStep = 30; // Default-Wert, wird normalerweise mit SetOutputStep überschrieben
    SetLookbackSec(10); // Default-Wert, wird normalerweise mit SetLookbackSec überschrieben
    lastTime = millis();
}

void PID_ATune::Cancel()
{
    running = false;
}

int PID_ATune::Runtime()
{
    justevaled = false;

    // Failsafe-Check (wie vorher)
    if (peakCount > 9 && running)
    {
        #ifdef ATUNE_DEBUG
        Serial.println("ATune: Failsafe peakCount > 9 triggered!");
        #endif
        running = false;
        FinishUp();
        return 1; // Fertig (Grund: Max Peaks)
    }

    unsigned long now = millis();

    // Prüfen, ob SampleTime abgelaufen ist
    if ((now - lastTime) < sampleTime) return 0; // Nicht return false, sondern 0 (wie im Original)
    lastTime = now;
    double refVal = *input;
    justevaled = true;

    // Initialisierung beim ersten Durchlauf
    if (!running)
    {
        peakType = 0;
        peakCount = 0;
        justchanged = false;
        absMax = refVal;
        absMin = refVal;
        setpoint = refVal;
        running = true;
        outputStart = *output;
        *output = outputStart + oStep;
        // Initialize lastInputs buffer
        for (int i = 0; i <= nLookBack; i++) { // Initialize entire buffer size
            lastInputs[i] = refVal;
        }
        #ifdef ATUNE_DEBUG
        Serial.println("ATune: Initializing and starting run.");
        #endif
    }
    else
    {
        // Update absMax und absMin (wie vorher)
        if (refVal > absMax) absMax = refVal;
        if (refVal < absMin) absMin = refVal;
    }

    // Output oszillieren lassen (wie vorher)
    if (refVal > setpoint + noiseBand) *output = outputStart - oStep;
    else if (refVal < setpoint - noiseBand) *output = outputStart + oStep;


    // Peak-Erkennung ============================================
    isMax = true; isMin = true;

    // --- CHANGE 1: Kurzeres Fenster für Peak-Erkennung definieren ---
    // Vergleiche nur mit den letzten N Samples, nicht mit allen 100
    // Ein Wert von 3 bis 7 ist hier oft sinnvoll. 5 ist ein Kompromiss.
    const int peakDetectLookback = 5;
    // ---------------------------------------------------------------

    // Prüfen, ob genug Samples für die *generelle* Pufferfüllung vorhanden sind
    // (Originalbedingung, lassen wir zur Sicherheit drin)
    if (nLookBack < 9) {
         // Schiebe den gesamten Puffer, aber führe noch keine Peak-Logik aus
         for (int i = nLookBack - 1; i >= 0; i--) {
             lastInputs[i + 1] = lastInputs[i];
         }
         lastInputs[0] = refVal;
         // Serial.println("ATune: Filling buffer..."); // DEBUG (kann viel Output erzeugen)
         return 0; // Noch nicht genug Historie für die Hauptlogik
     }


    // --- CHANGE 2: Peak-Erkennungsschleife angepasst ---
    // Führe die eigentliche Peak-Prüfung nur über das kurze Fenster durch
    int checkWindow = (nLookBack < peakDetectLookback) ? nLookBack : peakDetectLookback;
    for (int i = checkWindow - 1; i >= 0; i--) { // Schleife nur über 'checkWindow' Samples
        double val = lastInputs[i];
        if (isMax) isMax = refVal > val; // Ist aktueller Wert größer als die letzten 'checkWindow' Werte?
        if (isMin) isMin = refVal < val; // Ist aktueller Wert kleiner als die letzten 'checkWindow' Werte?
    }
    // ----------------------------------------------------

    // Schiebe den *gesamten* Puffer für die nächste Iteration weiter
    // (Behält die volle Historie für andere Zwecke oder zukünftige Logik bei)
    for (int i = nLookBack - 1; i >= 0; i--) {
        lastInputs[i + 1] = lastInputs[i];
    }
    lastInputs[0] = refVal;

    // Ende Peak-Erkennung =========================================


    // Peak-Logik & Zählung (weitgehend wie vorher) =================
    if (isMax)
    {
        if (peakType == 0) peakType = 1; // Erster Peak ist ein Max
        if (peakType == -1) // Übergang von Min zu Max
        {
            peakType = 1;
            justchanged = true;
            peak2 = peak1; // Zeit des vorigen Max merken für Pu-Berechnung
            #ifdef ATUNE_DEBUG
            Serial.printf("ATune: Max Peak detected (Value: %.2f, Time: %lu, PrevMaxTime: %lu)\n", refVal, now, peak2);
            #endif
        }
        peak1 = now; // Zeit des aktuellen Max speichern

        // --- CHANGE 3: peaks[] Zugriff abgesichert ---
        if (peakCount < 10) {
            peaks[peakCount] = refVal; // Max-Wert speichern (am Index des letzten Min!)
        } else {
            // Optional: Fehler loggen, falls dieser Fall eintritt
            #ifdef ATUNE_DEBUG
            Serial.println("ATune Warning: peakCount >= 10 in isMax!");
            #endif
        }
        // --------------------------------------------
    }
    else if (isMin)
    {
        if (peakType == 0) peakType = -1; // Erster Peak ist ein Min
        if (peakType == 1) // Übergang von Max zu Min
        {
            peakType = -1;
            peakCount++; // <<<< peakCount wird hier erhöht
            justchanged = true;
            #ifdef ATUNE_DEBUG
            Serial.printf("ATune: Min Peak detected (Value: %.2f). peakCount incremented to: %d\n", refVal, peakCount);
            #endif
        }

        if (peakCount < 10) {
            peaks[peakCount] = refVal; // Min-Wert speichern
        }
        // Keine Absicherung für peakCount >= 10 nötig, da peakCount > 9 oben abfängt
    }

	// --- CHANGE 4: Mindestzyklen für Stabilitätsprüfung erzwingen ---
	// Original war peakCount > 2 (Prüfung ab Zyklus 3)
	// Neu: peakCount >= 5 (Prüfung beginnt erst ab Zyklus 5)
	if(justchanged && peakCount >= 5)
	// -------------------------------------------------------------
	{
		// ... (Rest der Stabilitätsprüfung bleibt gleich) ...
		double amp1 = abs(peaks[peakCount - 1] - peaks[peakCount - 2]);
		double amp2 = abs(peaks[peakCount - 2] - peaks[peakCount - 3]);
		double avgSeparation = (amp1 + amp2) / 2.0;

		double threshold = 0.05 * (absMax - absMin);

                #ifdef ATUNE_DEBUG
                Serial.printf("ATune Stability Check - peakCount: %d, avgSeparation: %.4f, Threshold: %.4f (absMax: %.2f, absMin: %.2f)\n",
                                          peakCount, avgSeparation, threshold, absMax, absMin);
                #endif

		if (avgSeparation < threshold && threshold > 0)
		{
                        #ifdef ATUNE_DEBUG
                        Serial.println("ATune: Amplitude stable (and min cycles >= 5 met), finishing!");
                        #endif
			FinishUp();
			running = false;
			return 1; // Fertig (Grund: Stabilität nach min 5 Zyklen)
		}
	}
    justchanged = false; // Zurücksetzen für nächste Iteration

    // Wenn keine Bedingung erfüllt, weiterlaufen
    // Serial.println("ATune: Continuing..."); // DEBUG (Sehr viel Output!)
    return 0;
}

// Restliche Funktionen (FinishUp, GetKp, GetKi, GetKd, Set/Get-Methoden) bleiben wie im Original,
// es sei denn, du möchtest auch die Tuning-Regeln ändern (wie in Prio 2 zuvor besprochen).

void PID_ATune::FinishUp()
{
      *output = outputStart; // Setze Output zurück auf den Wert vor dem Tuning
      // Berechne Ku und Pu
      // Ku = Ultimate Gain. Formel basiert auf Relay-Methode.
      // Pu = Ultimate Period (Zeit zwischen zwei Maxima in Sekunden).
      if ((absMax - absMin) == 0) { // Division durch Null verhindern
          Ku = 0; // Oder einen anderen Fehlerwert setzen?
          Pu = 0;
          #ifdef ATUNE_DEBUG
          Serial.println("ATune ERROR: absMax - absMin is zero in FinishUp!");
          #endif
          return;
      }
      Ku = 4.0 * (2.0 * oStep) / ((absMax - absMin) * 3.14159);

      if (peak1 == peak2) { // Division durch Null bzw. ungültige Zeit verhindern
          Pu = 0;
          #ifdef ATUNE_DEBUG
          Serial.println("ATune ERROR: peak1 == peak2 in FinishUp!");
          #endif
      } else {
          Pu = (double)(peak1 - peak2) / 1000.0; // Zeitdifferenz in Sekunden
      }
      #ifdef ATUNE_DEBUG
      Serial.printf("ATune FinishUp - Ku: %.2f, Pu: %.2f\n", Ku, Pu);
      #endif
}

// --- GetKp, GetKi, GetKd verwenden die berechneten Ku, Pu ---
// Diese kannst du ändern, wenn du andere Tuning-Regeln möchtest

double PID_ATune::GetKp()
{
    // Klassisch ZN für PID Regler (controlType=1)
    if (controlType == 1) return 0.6 * Ku;
    // Klassisch ZN für PI Regler (controlType=0)
    else return 0.4 * Ku;
}

double PID_ATune::GetKi()
{
    // Klassisch ZN für PID Regler (controlType=1) -> Ki = Kp / Ti = (0.6*Ku) / (Pu/2.0) = 1.2*Ku / Pu
    if (controlType == 1) {
        if (Pu == 0) return 0; // Division durch Null verhindern
        return 1.2 * Ku / Pu;
    }
    // Klassisch ZN für PI Regler (controlType=0) -> Ki = Kp / Ti = (0.4*Ku) / (Pu/1.2) = 0.48*Ku/Pu
    else {
        if (Pu == 0) return 0; // Division durch Null verhindern
        return 0.48 * Ku / Pu;
    }
}

double PID_ATune::GetKd()
{
    // Klassisch ZN für PID Regler (controlType=1) -> Kd = Kp * Td = (0.6*Ku) * (Pu/8.0) = 0.075 * Ku * Pu
    if (controlType == 1) return 0.075 * Ku * Pu;
    // Kein Kd für PI Regler (controlType=0)
    else return 0;
}


// --- Setter und Getter bleiben unverändert ---

void PID_ATune::SetOutputStep(double Step)
{
    oStep = Step;
}

double PID_ATune::GetOutputStep()
{
    return oStep;
}

void PID_ATune::SetControlType(int Type) //0=PI, 1=PID
{
    controlType = Type;
}
int PID_ATune::GetControlType()
{
    return controlType;
}

void PID_ATune::SetNoiseBand(double Band)
{
    noiseBand = Band;
}

double PID_ATune::GetNoiseBand()
{
    return noiseBand;
}

void PID_ATune::SetLookbackSec(int value)
{
    if (value < 1) value = 1;

    if (value < 25)
    {
        nLookBack = value * 4;
        sampleTime = 250;
    }
    else
    {
        // Hier wird nLookBack immer auf 100 gesetzt für längere Zeiten!
        // Das Array lastInputs muss entsprechend groß sein (im .h File prüfen!)
        nLookBack = 100;
        sampleTime = value * 10; // sampleTime wird größer
    }
    // Ensure nLookBack isn't larger than array capacity (assuming array size is 101 based on original loop logic)
    if (nLookBack > 100) nLookBack = 100;
}

int PID_ATune::GetLookbackSec()
{
    // Berechne den tatsächlichen Lookback basierend auf nLookBack und sampleTime
    // Beachte: sampleTime kann variieren!
    return nLookBack * sampleTime / 1000;
}