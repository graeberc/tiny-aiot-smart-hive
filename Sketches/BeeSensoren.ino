#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <Wire.h>
#include <Adafruit_HDC1000.h>
#include <Adafruit_VEML6070.h> 


const char* ssid = "";
const char* password = "";

const char* SENSEBOX_ID = "697b3d91bd5b9f0007edb95b";

// IDs:
const char* SENSOR_ID_TEMP  = "697b3d91bd5b9f0007edb95c";
const char* SENSOR_ID_HUM   = "697b3d91bd5b9f0007edb95d";
const char* SENSOR_ID_UV    = "697b3d91bd5b9f0007edb95e"; 

const char* server = "ingress.opensensemap.org";

// Zertifikat für HTTPS (ISRG Root X1)
const char* root_ca = \
    "-----BEGIN CERTIFICATE-----\n" \
    "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\n" \
    "-----END CERTIFICATE-----\n";


WiFiClientSecure client;

// Sensoren Objekte
Adafruit_HDC1000 hdc;
Adafruit_VEML6070 uv = Adafruit_VEML6070();

const unsigned long intervalInterval = 60000;
unsigned long lastSend = 0;



void initWiFi() {
  Serial.println("Starte WLAN (Full Power)...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  int tryCount = 0;
  while (WiFi.status() != WL_CONNECTED && tryCount < 20) {
    delay(500);
    Serial.print(".");
    tryCount++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWLAN Verbunden!");
    Serial.print("IP: "); Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWLAN Fehler!");
  }
}

// Upload Funktion für Temp, Hum, UV
void submitValues(float t, float h, uint16_t uv_val) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Kein WLAN, versuche Reconnect...");
    WiFi.reconnect();
    return;
  }

  if (!client.connect(server, 443)) {
    Serial.println("SSL Verbindung fehlgeschlagen!");
    return;
  }

  // Datenpaket bauen (CSV Format)
  String body = "";
  // 1. Temp & Feuchte
  body += String(SENSOR_ID_TEMP) + "," + String(t) + "\n";
  body += String(SENSOR_ID_HUM)  + "," + String(h) + "\n";
  
  // 2. UV 
  body += String(SENSOR_ID_UV)   + "," + String(uv_val);

  // HTTPS Header senden
  client.print("POST /boxes/" + String(SENSEBOX_ID) + "/data HTTP/1.1\r\n");
  client.print("Host: " + String(server) + "\r\n");
  client.print("Content-Type: text/csv\r\n");
  client.print("Connection: close\r\n");
  client.print("Content-Length: " + String(body.length()) + "\r\n\r\n");
  
  client.print(body);


  unsigned long timeout = millis();
  while (client.connected() && millis() - timeout < 2000) {
    if (client.available()) {
      client.readStringUntil('\n'); 
      break; 
    }
  }
  client.stop();
}

void setup() {
  Serial.begin(115200);
  delay(1000); 
  Serial.println("\nBeeSense Lite (Temp/Hum + UV) startet...");

  // I2C Pins setzen
  Wire.begin(2, 1); 
  
  // 1. HDC1080 Starten
  if (!hdc.begin(0x40)) { 
    Serial.println("ACHTUNG: HDC Sensor nicht gefunden!");
  } else {
    Serial.println("Sensor OK: HDC10x bereit.");
  }

  // 2. UV Sensor Starten
  uv.begin(VEML6070_1_T); 
  Serial.println("Sensor OK: VEML6070 bereit.");

  initWiFi();
  client.setCACert(root_ca);
}


void loop() {
  if (millis() - lastSend > intervalInterval) {
    lastSend = millis();

    // 1. Messen
    float temp = hdc.readTemperature();
    float hum = hdc.readHumidity();
    uint16_t uv_val = uv.readUV();

    // 2. Anzeigen
    Serial.println("-----------------------------");
    Serial.print("Temperatur: "); Serial.print(temp); Serial.println(" °C");
    Serial.print("Feuchte:    "); Serial.print(hum); Serial.println(" %");
    Serial.print("UV Level:   "); Serial.println(uv_val);

    // 3. Senden 
    if (temp > -39.0 && hum > 0.0) {
        submitValues(temp, hum, uv_val);
    } else {
        Serial.println("FEHLER: Ungültige Messwerte HDC. Versuche Neustart...");
        hdc.begin(0x40); 
    }
  }
}