# esp32-home-switch-strips-mqtt

ESP32-Steuerung fuer das LED-Lichtband im Wohnzimmer mit MQTT und Home Assistant.
Die Firmware befindet sich in [esp32-home-switch-strips-mqtt.ino](./esp32-home-switch-strips-mqtt.ino).

## Hardware und Firmware

Die im Quellcode konfigurierte Hardware:

| Einstellung | Wert |
| --- | --- |
| LED-Anzahl | 292 |
| LED-Treiber | FastLED, WS2811 |
| Datenpin | GPIO 26 |
| Farbkanal-Reihenfolge | BRG |
| Mikrofoneingang fuer Programm 1 | A0; FFT liest zusaetzlich `analogRead(0)` |
| MQTT-Port | 1883 |
| MQTT-Client-ID | `esp32-home-switch-strips-mqtt` |
| Arduino-OTA-Hostname | `esp32-home-switch-strips-mqtt` |

Es gibt **einen logischen Strip mit drei alternativen Programmen**, nicht drei
unabhaengige Strips. Alle 292 LEDs erhalten innerhalb eines Programms denselben
Farbwert. Die separaten Home-Assistant-Entitaeten `light.livingroom_light_1` bis
`light.livingroom_light_16` gehoeren nicht zu dieser hier dokumentierten Schnittstelle.

Die Firmware verwendet WiFi, PubSubClient, ArduinoOTA, FastLED, arduinoFFT und die
ArduinoJson-API mit `StaticJsonBuffer`. WLAN- und MQTT-Zugangsdaten werden ueber
`_authentification.h` eingebunden. Zugangsdaten nicht in die Dokumentation oder
oeffentliche Versionsverwaltung aufnehmen.

## MQTT-Schnittstelle

### Programme und Befehle

| Programm | Command-Topic | Funktion | Beispiel zum Einschalten |
| --- | --- | --- | --- |
| 1 | `wz/strip_1/program_1/set/on` | Mikrofon-/musikabhaengige Farbe und Helligkeit | `{"status":1}` |
| 2 | `wz/strip_1/program_2/set/on` | Feste Farbe und Helligkeit mit FastLED `CHSV` | `{"status":1,"hue":22,"saturation":85,"brightness":102}` |
| 3 | `wz/strip_1/program_3/set/on` | Feste RGB-Farbe | `{"status":1,"color":"FFD6AA"}` |

- `status: 1` waehlt das jeweilige Programm.
- `status: 0` schaltet den **gesamten Strip** aus, unabhaengig vom zuvor aktiven
  Programm. Beispiel: `{"status":0}` auf einem der drei Command-Topics.
- Programm 2 benoetigt beim Einschalten immer `hue`, `saturation` und
  `brightness`, jeweils im Bereich **0 bis 255**. Hue ist kein Winkel in Grad.
  `brightness: 102` entspricht 40 Prozent des numerischen Wertebereichs.
- Programm 3 erwartet sechs Hex-Zeichen **ohne fuehrendes `#`**. Es gibt keinen
  separaten Helligkeitsparameter fuer dieses Programm.
- Die Firmware validiert Parameterbereiche und JSON-Fehler nicht umfassend.
  Deshalb nur vollstaendige, gueltige Payloads senden.

Die aktuellen Warmweiss-Werte fuer Programm 2 sind `22 / 85 / 102`. `#FFD6AA`
war die gewuenschte RGB-Farbreferenz. FastLED verwendet fuer `CHSV` jedoch eine
eigene Farbabbildung: Diese Werte garantieren weder exakt diese RGB-Farbe noch
eine bestimmte Farbtemperatur in Kelvin. Den Farbeindruck am echten Strip
pruefen und bei Bedarf Hue/Saettigung anpassen.

### Statusrueckmeldung

Topic: `wz/strip_1/status/get/on`

```json
{"status":1}
```

`1` bedeutet, dass ein Programm aktiv ist; `0` bedeutet ausgeschaltet.
Die Rueckmeldung nennt **weder das aktive Programm noch Farbe oder Helligkeit**.
Sie ist keine Messung der tatsaechlichen Lichtabgabe und bestaetigt keinen
einzelnen Befehl. Die Firmware publiziert den Status ohne Retain.

## Bestehende Home-Assistant-Anbindung

In der aktuellen Installation existieren:

| Entitaet | Zweck |
| --- | --- |
| `switch.wz_strip_program_1` | Programm 1 einschalten / Strip ausschalten |
| `switch.wz_strip_program_2_rgb` | Programm 2 einschalten / Strip ausschalten |
| `switch.wz_strip_program_3_color` | Programm 3 einschalten / Strip ausschalten |
| `sensor.wz_strip_status` | Gesamtstatus `on` / `off` aus dem MQTT-Status-Topic |

Die drei MQTT-Schalter sind **optimistisch** konfiguriert. Ihre Zustaende sind
keine verlaessliche Rueckmeldung des ESP32 und werden durch direkte
`mqtt.publish`-Befehle nicht automatisch synchronisiert.

Die bestehenden Einschalt-Payloads der Schalter fuer Programm 2 und 3 enthalten
nur `{"status":1}`. Damit fehlen fuer Programm 2 die HSV-Werte und fuer
Programm 3 die Farbe. Die Praesenzautomation verwendet deshalb direkt
`mqtt.publish` mit vollstaendigen Parametern. Diese Dokumentation aendert die
bestehenden Schalter nicht.

### Manueller Befehl in Home Assistant

Unter Entwicklerwerkzeuge > Aktionen:

```yaml
action: mqtt.publish
data:
  topic: wz/strip_1/program_2/set/on
  payload: '{"status":1,"hue":22,"saturation":85,"brightness":102}'
  qos: 0
  retain: false
```

Zum Ausschalten denselben Aufruf mit `payload: '{"status":0}'` verwenden.

## Aqara FP2: Praesenzautomation im Wohnzimmer

Stand: 2026-10-09. Die Automation wurde in Home Assistant angelegt und aktiviert.

- Name: `Wohnzimmer FP2 - LED-Strip Praesenz Warmweiss`
- Automation-Entitaet: `automation.wohnzimmer_fp2_led_strip_praesenz_warmweiss`
- Gesamtpraesenz:
  `binary_sensor.wohnzimmer_presence_sensor_fp2_887b_presence_sensor_1`
- Die Zuordnung von Sensor 1 zur Gesamtpraesenz wurde vom Betreiber bestaetigt.
  Die anderen FP2-Sensoren sind nicht Teil dieser Automation.

### Verhalten

1. Bei `on`: Programm 2 sofort mit Warmweiss und 40 Prozent einschalten.
2. Bei `off`: fuenf Minuten warten, dann den Strip ausschalten, sofern der
   Sensor weiterhin `off` meldet.
3. Erneute Praesenz startet die Automation neu und bricht das wartende
   Ausschalten ab.
4. `unknown` oder `unavailable` brechen ebenfalls die Wartezeit ab; ein
   Sensorausfall wird nicht als Abwesenheit behandelt.
5. Beim Start von Home Assistant wird der aktuelle Sensorzustand ausgewertet.

### YAML der Steuerungslogik

Die Automation existiert bereits. Das Beispiel nicht zusaetzlich als zweite
Automation anlegen.

```yaml
alias: Wohnzimmer FP2 - LED-Strip Praesenz Warmweiss
triggers:
  - trigger: state
    entity_id: binary_sensor.wohnzimmer_presence_sensor_fp2_887b_presence_sensor_1
    to: "on"
  - trigger: state
    entity_id: binary_sensor.wohnzimmer_presence_sensor_fp2_887b_presence_sensor_1
    to: "off"
  - trigger: state
    entity_id: binary_sensor.wohnzimmer_presence_sensor_fp2_887b_presence_sensor_1
    to: "unknown"
  - trigger: state
    entity_id: binary_sensor.wohnzimmer_presence_sensor_fp2_887b_presence_sensor_1
    to: "unavailable"
  - trigger: homeassistant
    event: start
conditions: []
actions:
  - choose:
      - conditions:
          - condition: state
            entity_id: binary_sensor.wohnzimmer_presence_sensor_fp2_887b_presence_sensor_1
            state: "on"
        sequence:
          - action: mqtt.publish
            data:
              topic: wz/strip_1/program_2/set/on
              payload: '{"status":1,"hue":22,"saturation":85,"brightness":102}'
              qos: 0
              retain: false
      - conditions:
          - condition: state
            entity_id: binary_sensor.wohnzimmer_presence_sensor_fp2_887b_presence_sensor_1
            state: "off"
        sequence:
          - delay: "00:05:00"
          - condition: state
            entity_id: binary_sensor.wohnzimmer_presence_sensor_fp2_887b_presence_sensor_1
            state: "off"
          - action: mqtt.publish
            data:
              topic: wz/strip_1/program_2/set/on
              payload: '{"status":0}'
              qos: 0
              retain: false
mode: restart
```

### Grenzen und Betriebshinweise

- Die Wartezeit ist nicht persistent. Ein HA-Neustart verwirft sie und beginnt
  bei `off` eine neue volle Fuenf-Minuten-Wartezeit. Ein Automation-Reload
  verwirft sie ebenfalls, loest aber keinen HA-Start-Trigger aus; ohne weitere
  Sensorzustandsaenderung startet die Wartezeit dann nicht erneut.
- Das Anlegen/Aktivieren allein wertet den bereits vorhandenen Sensorzustand
  nicht aus. Die Steuerung startet bei einem passenden Trigger.
- MQTT-Befehle werden mit QoS 0 und ohne Retain gesendet. Bei Verbindungsfehlern
  gibt es keine garantierte Zustellung oder automatische Befehlwiederholung
  durch diese Automation. Ein ESP32-Neustart allein loest sie nicht aus.
- Es gibt keinen Helligkeitssensor-Schwellwert und keine Tag-/Nachtbedingung.
  Praesenz schaltet den Strip unabhaengig vom Umgebungslicht ein.
- Es gibt keinen manuellen Override. Bei Praesenz setzt ein Einschaltlauf
  wieder Programm 2; nach Abwesenheit wird auch ein manuell aktiviertes
  anderes Strip-Programm ausgeschaltet.
- MQTT-Port 1883 verwendet hier keine TLS-Verbindung; Arduino OTA ist im
  Quellcode ohne aktiviertes Passwort eingerichtet. Betrieb nur in einem
  vertrauenswuerdigen, passend abgeschirmten Netz.

## Verifikation und Funktionstest

Bereits geprueft: Home-Assistant-Konfiguration gueltig, gespeicherte Automation
zurueckgelesen, Automation aktiviert, MQTT-Topics/Payloads gegen die Firmware
geprueft. Ein realer Sensor-/Strip-End-to-End-Test ist noch nicht bestaetigt.

| Test | Erwartetes Ergebnis |
| --- | --- |
| Wohnzimmer betreten | Sensor 1 wird `on`; Strip startet sofort mit Programm 2 |
| Wohnzimmer verlassen | Strip bleibt bis zum Ablauf von fuenf Minuten an, dann aus |
| Innerhalb der fuenf Minuten zurueckkehren | Ausstehendes Ausschalten wird abgebrochen |
| Sensor waehrend der Wartezeit nicht verfuegbar | Kein Ausschalten aufgrund des Sensorausfalls |
| HA bei `off` neu starten | Neue volle Fuenf-Minuten-Wartezeit |
| Farbe am Strip ansehen | Warmweiss-Eindruck pruefen; bei Bedarf HSV anpassen |

Zur Diagnose die Automation-Traces, den FP2-Zustand und
`sensor.wz_strip_status` verwenden. Optimistische Programmschalter nicht als
Nachweis der tatsaechlichen Programmauswahl verwenden.
