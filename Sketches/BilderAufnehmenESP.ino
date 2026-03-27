#include <Arduino.h>
#include "esp_camera.h"
#include <SPI.h>
#include <SD.h>
#include <WiFi.h>
#include <WebServer.h>

// Pins
#define PWDN_GPIO_NUM   -1
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM   15
#define SIOD_GPIO_NUM   4
#define SIOC_GPIO_NUM   5
#define Y9_GPIO_NUM     16
#define Y8_GPIO_NUM     17
#define Y7_GPIO_NUM     18
#define Y6_GPIO_NUM     12
#define Y5_GPIO_NUM     10
#define Y4_GPIO_NUM     8
#define Y3_GPIO_NUM     9
#define Y2_GPIO_NUM     11
#define VSYNC_GPIO_NUM  6
#define HREF_GPIO_NUM   7
#define PCLK_GPIO_NUM   13

#define VSPI_MISO 40
#define VSPI_MOSI 38
#define VSPI_SCLK 39
#define VSPI_SS   41

// Globals
SPIClass sdspi(SPI);
int img_counter = 0; 
WebServer server(80);

unsigned long last_capture_time = 0;
#define CAPTURE_INTERVAL 2000 // Alle 2 Sekunden ein Bild

// Wlan (ändern)
#define WIFI_SSID     "xxx"
#define WIFI_PASSWORD "xxx"

// Zähler und Speicher laden
#define COUNTER_FILE "/counter.txt"

void loadCounter() {
    if (SD.exists(COUNTER_FILE)) {
        File f = SD.open(COUNTER_FILE, FILE_READ);
        if (f) {
            String s = f.readString();
            img_counter = s.toInt(); 
            f.close();
            Serial.printf("Zähler geladen: Weiter bei Bild %d\n", img_counter);
        }
    } else {
        Serial.println("Kein Zähler gefunden, fange bei 0 an.");
        img_counter = 0;
    }
}

void saveCounter() {
    File f = SD.open(COUNTER_FILE, FILE_WRITE); 
    if (f) {
        f.seek(0); 
        f.print(img_counter); 
        f.close();
    }
}

// Webserver
void handleIndex() {
    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "text/html", "<html><head><title>Kamera</title></head><body><h2>Bilder</h2><ul>");

    File root = SD.open("/");
    if(!root){ server.sendContent("Fehler beim Lesen"); return; }

    File file = root.openNextFile();
    while(file){
        if(!file.isDirectory()){
            String fileName = file.name();
            if(fileName.endsWith(".jpg")){
                String line = "<li><a href='/img?name=" + fileName + "'>" + fileName + " (" + String(file.size()) + " B)</a></li>";
                server.sendContent(line);
            }
        }
        file = root.openNextFile();
    }
    server.sendContent("</ul></body></html>");
    server.client().stop();
}

void handleImage() {
    if (!server.hasArg("name")) { server.send(400, "text/plain", "Missing name"); return; }
    String filename = "/" + server.arg("name");
    if (SD.exists(filename)) {
        File img = SD.open(filename, FILE_READ);
        server.streamFile(img, "image/jpeg");
        img.close();
    } else {
        server.send(404, "text/plain", "Not found");
    }
}

// Setup
void setup() {
    Serial.begin(115200);

    // Kamera
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size = FRAMESIZE_240X240; 
    config.jpeg_quality = 12;
    config.fb_count = 1;

    if (esp_camera_init(&config) != ESP_OK) { Serial.println("Kamera Init Fehler"); while(1); }

    // SD-Karte
    sdspi.begin(VSPI_SCLK, VSPI_MISO, VSPI_MOSI, VSPI_SS);
    if (!SD.begin(VSPI_SS, sdspi, 4000000)) { Serial.println("SD Init Fehler"); while(1); }
    
    // Zähler laden
    loadCounter(); 

    // WIFI
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("WiFi verbunden");
    while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
    Serial.println("\nIP: " + WiFi.localIP().toString());

    server.on("/", handleIndex);
    server.on("/img", handleImage);
    server.begin();
}

// Loop
void loop() {
    server.handleClient();

    unsigned long now = millis();
    if (now - last_capture_time > CAPTURE_INTERVAL) {
        last_capture_time = now;

        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) { Serial.println("Capture Fehler"); return; }

        char filename[32];
        sprintf(filename, "/img_%05d.jpg", img_counter); 

        File file = SD.open(filename, FILE_WRITE);
        if (file) {
            file.write(fb->buf, fb->len);
            file.close();
            Serial.printf("Gespeichert: %s\n", filename);
            img_counter++; 
            saveCounter(); 
        } else {
            Serial.println("SD Schreibfehler");
        }
        esp_camera_fb_return(fb);
    }
}