# tiny-aiot-smart-hive

# Hummel Edge Impulse Projekt

## Projektübersicht
Dieses Projekt dient zur Erkennung und Zählung von Hummeln mithilfe eines Edge-Impulse-Modells auf einem ESP32-Kameramodul.  
Das Modell basiert auf **FOMO (Object Detection)** und wird direkt auf dem Gerät ausgeführt. Die Kamera liefert ein Livebild, das über einen integrierten Webserver angezeigt wird. Erkannte Hummeln werden als Bounding Boxen visualisiert und gezählt.

---

## Aktueller Stand
- **Aktuelle Modellversion:** `HummelModell_inferencing`
- **Aktueller Sketch:** `FinalSketch`
- **Edge-Impulse-Anpassung:**  
  Die Datei `ei_classifier_porting.cpp` im Modellordner wurde durch eine angepasste Version ersetzt, um die Inferenz korrekt auf dem ESP32 auszuführen.

---

## Funktionsweise (vereinfacht)
1. Die Kamera nimmt kontinuierlich Bilder auf.
2. Die Bilder werden auf die vom Modell erwartete Größe skaliert.
3. Das Edge-Impulse-Modell analysiert jedes Bild auf Hummeln.
4. Erkannte Hummeln werden:
   - im Livebild als Bounding Box dargestellt
   - intern getrackt
   - beim Durchqueren definierter Linien gezählt (rein / raus)
5. Das Livebild wird über einen Webserver im Browser angezeigt.

---

## Modell
- Typ: **Object Detection (FOMO)**
- Eingabegröße: **160 × 160 Pixel**
- Klassen: **Hummel**
- Training:  
  Basierend auf selbst aufgenommenen Bildern mit realem Hintergrund aus der späteren Einsatzumgebung.

---

## Wichtige Dateien
- `FinalSketch.ino`  
  Hauptsketch für Kamera, WLAN, Webserver, Inferenz und Zählung
- `HummelModell_inferencing/`  
  Edge-Impulse-Modellordner
- `ei_classifier_porting.cpp`  
  Angepasste Portierung für den ESP32 (ersetzt die Originaldatei aus dem Download)

---

## Aktuelle Probleme
- Hummeln werden im Livebild aktuell **nicht zuverlässig erkannt**
- Teilweise erscheinen Bounding Boxen auf zufälligen Bildbereichen
- Die genaue Ursache (Modell vs. Bildvorverarbeitung) ist noch in Analyse
- Ein stabiler Fix wurde bisher nicht gefunden

---


## Ziel
Zuverlässige Erkennung und Zählung von Hummeln am Nesteingang unter realen Bedingungen.
