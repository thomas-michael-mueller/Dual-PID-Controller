# PID AutoTune v0

Diese Bibliothek implementiert eine AutoTune-Routine für PID-Regler. Im Dual-PID-Projekt ermittelt sie automatisch passende PID-Parameter für Wasser- und Dampftemperatur.

## Herkunft

Die Grundlage bildet Brett Beauregards [Arduino-PID-AutoTune-Library](https://github.com/br3ttb/Arduino-PID-AutoTune-Library).

## Anpassungen im Dual-PID-Projekt

- Optimierte Peak-Erkennung mit verkürztem Analysefenster
- Failsafe-Abbruch nach überschrittenem Peak-Limit
- Zusätzliche Getter und Debug-Ausgaben
- Deutsche Kommentare und angepasste Standardwerte

## Lizenz (BSD-2-Klausel)

Copyright (c) 2012 Brett Beauregard
Copyright (c) 2025 Thomas Michael Müller

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
