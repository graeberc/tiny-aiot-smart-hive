#include <HummelProjekt_inferencing.h>
#include "esp_camera.h"
#include <WiFi.h>
[cite_start]#include <WiFiClientSecure.h>
#include "esp_http_server.h"
#include <vector>

// --- Bibliotheken für Sensoren ---
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_HDC1000.h>
#include <Adafruit_VEML6070.h>
#include <Adafruit_LTR329_LTR303.h>

// --- WLAN KONFIGURATION ---
const char* ssid = "***";
const char* password = "***";

// --- SENSEBOX KONFIGURATION ---
const char* SENSEBOX_ID = "697b3d91bd5b9f0007edb95b";
const char* SENSOR_ID_TEMP = "697b3d91bd5b9f0007edb95c";
const char* SENSOR_ID_HUM = "697b3d91bd5b9f0007edb95d";
const char* SENSOR_ID_UV = "697b3d91bd5b9f0007edb95e";
const char* SENSOR_ID_LIGHT = "697b3d91bd5b9f0007edb95e";
const char* SENSOR_ID_IN = "undefined";
const char* SENSOR_ID_OUT = "undefined";

// OpenSenseMap Server
const char* server = "ingress.opensensemap.org";

// --- CONFIG ---
#define CONFIDENCE_THRESHOLD 0.6
#define PERSISTENCE_REQUIRED 1
#define MODEL_W EI_CLASSIFIER_INPUT_WIDTH
#define MODEL_H EI_CLASSIFIER_INPUT_HEIGHT

// Zähler-Linien
#define LINE_TOP_POS 0.25 
#define LINE_BOT_POS 0.75
#define TRACKING_DIST 60 

int cnt_in = 0;
int cnt_out = 0;

// --- PINS & Objekte ---
#define PIN_QWIIC_SDA 2
#define PIN_QWIIC_SCL 1

// OLED
Adafruit_SSD1306 display(128, 64, &Wire, -1);

// Sensoren
Adafruit_HDC1000 hdc;
Adafruit_VEML6070 uv;
Adafruit_LTR329 ltr;

// Globale Variablen für Sensorwerte
float global_temp = 0.0;
float global_hum = 0.0;
uint16_t global_uv = 0;
uint16_t global_lux = 0;

// KAMERA PINS
#define PWDN_GPIO_NUM  46
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  15
#define SIOD_GPIO_NUM  4
#define SIOC_GPIO_NUM  5
#define Y9_GPIO_NUM    16
#define Y8_GPIO_NUM    17
#define Y7_GPIO_NUM    18
#define Y6_GPIO_NUM    12
#define Y5_GPIO_NUM    10
#define Y4_GPIO_NUM    8
#define Y3_GPIO_NUM    9
#define Y2_GPIO_NUM    11
#define VSYNC_GPIO_NUM 6
#define HREF_GPIO_NUM  7
#define PCLK_GPIO_NUM  13

httpd_handle_t stream_httpd = NULL;

// Buffer
uint16_t *ai_input_buf = NULL; 
uint16_t *display_buf = NULL;  

bool ai_is_running = false;
bool new_frame_reay = false;
SemaphoreHandle_t xMutex;

struct Box { int x, y, w, h; float prob; int persistence; };
Box confirmed_boxes[5];
int confirmed_count = 0;

// Tracking Vars
Box candidates[5];
int candidate_count = 0;

struct TrackedBee {
    int id;
    int cy; 
    bool touched_top;
    bool touched_bot;
    bool counted;
    int lost_frames;
};
std::vector<TrackedBee> trackers;
int next_bee_id = 1;

// --- Upload Funktion für OpenSenseMap ---
void uploadData() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi nicht verbunden, Upload übersprungen.");
        return;
    }

    WiFiClientSecure client;
    client.setInsecure(); // Zertifikat nicht prüfen (einfacher für ESP32)

    if (!client.connect(server, 443)) {
        Serial.println("Verbindung zu OpenSenseMap fehlgeschlagen!");
        return;
    }

    // CSV Format vorbereiten: sensorID,value\n
    String content = "";
    content += String(SENSOR_ID_TEMP) + "," + String(global_temp) + "\n";
    content += String(SENSOR_ID_HUM) + "," + String(global_hum) + "\n";
    content += String(SENSOR_ID_UV) + "," + String(global_uv) + "\n";
    content += String(SENSOR_ID_LIGHT) + "," + String(global_lux) + "\n";
    content += String(SENSOR_ID_IN) + "," + String(cnt_in) + "\n";
    content += String(SENSOR_ID_OUT) + "," + String(cnt_out) + "\n";

    // HTTP POST Request senden
    client.print("POST /boxes/" + String(SENSEBOX_ID) + "/data HTTP/1.1\r\n");
    client.print("Host: " + String(server) + "\r\n");
    client.print("Content-Type: text/csv\r\n");
    client.print("Content-Length: " + String(content.length()) + "\r\n");
    client.print("Connection: close\r\n\r\n");
    client.print(content);

    Serial.println("Daten an SenseBox gesendet!");
    
    // Antwort lesen (optional, zum Debuggen)
    while (client.connected()) {
        String line = client.readStringUntil('\n');
        if (line == "\r") break;
    }
    client.stop();
}

bool is_same_object(Box a, Box b) {
    int cx_a = a.x + a.w/2;
    int cy_a = a.y + a.h/2;
    int cx_b = b.x + b.w/2; int cy_b = b.y + b.h/2;
    int dist = sqrt(pow(cx_a - cx_b, 2) + pow(cy_a - cy_b, 2));
    return (dist < 40);
}

void update_counters() {
    for(auto &t : trackers) t.lost_frames++;

    for(int i=0; i<confirmed_count; i++) {
        int cy = confirmed_boxes[i].y + confirmed_boxes[i].h / 2;
        int best_idx = -1;
        float min_dist = TRACKING_DIST;

        for(int j=0; j<trackers.size(); j++) {
            float dist = abs(trackers[j].cy - cy);
            if(dist < min_dist) { min_dist = dist; best_idx = j; }
        }

        if(best_idx != -1) {
            trackers[best_idx].cy = cy;
            trackers[best_idx].lost_frames = 0;
        } else {
            TrackedBee t;
            t.id = next_bee_id++;
            t.cy = cy;
            t.touched_top = false; t.touched_bot = false; t.counted = false;
            t.lost_frames = 0;
            trackers.push_back(t);
        }
    }

    int line_top = MODEL_H * LINE_TOP_POS;
    int line_bot = MODEL_H * LINE_BOT_POS;

    for(auto &t : trackers) {
        if(t.lost_frames > 2) continue;
        if(t.cy < line_top) t.touched_top = true;
        if(t.cy > line_bot) t.touched_bot = true;

        if(t.touched_top && t.touched_bot && !t.counted) {
            if(t.cy > line_bot) {
                cnt_in++;
                Serial.printf(">>> ZÄHLER: Hummel REIN! (In: %d | Out: %d)\n", cnt_in, cnt_out);
            } else if (t.cy < line_top) {
                cnt_out++;
                Serial.printf("<<< ZÄHLER: Hummel RAUS! (In: %d | Out: %d)\n", cnt_in, cnt_out);
            }
            t.counted = true;
        }
    }
    for(int i=trackers.size()-1; i>=0; i--) {
        if(trackers[i].lost_frames > 10) trackers.erase(trackers.begin() + i);
    }
}

// --- Hilfsfunktion für Sensoren ---
void handleSensorsAndDisplay() {
    // Sensoren lesen und in globale Variablen speichern
    global_temp = hdc.readTemperature(); [cite_start]
    global_hum = hdc.readHumidity();     [cite_start]
    global_uv = uv.readUV();             [cite_start]
    uint16_t ch0, ch1;
    ltr.readBothChannels(ch0, ch1);
    global_lux = ch0;                    [cite_start]

    // OLED Update
    display.clearDisplay();
    display.setCursor(0, 0);
    
    // Luft
    display.print("Luft:   "); 
    if (!isnan(global_temp)) { display.print(global_temp, 1); display.println(" C"); }
    else { display.println("-.- C"); }

    // Feuchte
    display.print("Feucht: ");
    if (!isnan(global_hum)) { display.print(global_hum, 1); display.println(" %"); }
    else { display.println("-.- %"); }

    // Licht
    display.print("Licht:  "); display.println(global_lux);
    
    // Zählerstand anzeigen
    display.println("----------------");
    display.print("In: "); display.print(cnt_in);
    display.print(" Out: "); display.println(cnt_out);

    display.display();
}

void aiTask(void * parameter) {
    unsigned long lastSensorUpdate = 0;
    unsigned long lastUploadTime = 0; // Timer für Upload

    while(true) {
        unsigned long currentMillis = millis();

        [cite_start]// 1. Sensoren & Display (alle 2 Sekunden) [cite: 131]
        if(currentMillis - lastSensorUpdate > 2000) {
            handleSensorsAndDisplay();
            lastSensorUpdate = currentMillis;
        }

        // 2. Upload zur SenseBox (alle 60 Sekunden)
        if(currentMillis - lastUploadTime > 60000) {
            uploadData();
            lastUploadTime = currentMillis;
        }

        // 3. KI Logik (Frame Verarbeitung)
        if (new_frame_reay) {
            ai_is_running = true;
            new_frame_reay = false; 

            signal_t signal;
            signal.total_length = MODEL_W * MODEL_H;
            signal.get_data = [](size_t offset, size_t length, float *out_ptr) {
                for (size_t i = 0; i < length; i++) {
                    size_t p_idx = (offset + i) / 3;
                    size_t ch = (offset + i) % 3;
                    uint16_t pix = ai_input_buf[p_idx];
                    float val;
                    if (ch == 0) val = ((pix >> 11) & 0x1F) * (255.0f/31.0f);
                    else if (ch == 1) val = ((pix >> 5) & 0x3F) * (255.0f/63.0f);
                    else val = (pix & 0x1F) * (255.0f/31.0f);
                    out_ptr[i] = val;
                }
                return 0;
            };

            ei_impulse_result_t result = { 0 };
            run_classifier(&signal, &result, false);

            xSemaphoreTake(xMutex, portMAX_DELAY);
            Box next_candidates[5];
            int next_cand_count = 0;
            confirmed_count = 0;
            for (size_t ix = 0; ix < result.bounding_boxes_count; ix++) {
                auto bb = result.bounding_boxes[ix];
                if (bb.value > CONFIDENCE_THRESHOLD) {
                    Box newBox = {bb.x, bb.y, bb.width, bb.height, bb.value, 1};
                    for(int c=0; c<candidate_count; c++) {
                        if(is_same_object(candidates[c], newBox)) {
                            newBox.persistence = candidates[c].persistence + 1;
                            break;
                        }
                    }
                    if(next_cand_count < 5) {
                        next_candidates[next_cand_count] = newBox;
                        next_cand_count++;
                    }
                    if(newBox.persistence >= PERSISTENCE_REQUIRED) {
                        if(confirmed_count < 5) {
                            confirmed_boxes[confirmed_count] = newBox;
                            confirmed_count++;
                            Serial.printf("HUMMEL erkannt! (%.2f, %d mal)\n", bb.value, newBox.persistence);
                        }
                    }
                }
            }
            candidate_count = next_cand_count;
            for(int i=0; i<5; i++) candidates[i] = next_candidates[i];
            update_counters();
            xSemaphoreGive(xMutex);
            ai_is_running = false;
        }
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

void draw_face_box(uint16_t* buf, int w, int h, int bx, int by, int bw, int bh, uint16_t color) {
    if (bx < 0) bx = 0;
    if (by < 0) by = 0;
    for (int x = bx; x < bx + bw; x++) {
        if (x < w) {
            if (by < h) buf[by * w + x] = color;
            if ((by + bh - 1) < h) buf[(by + bh - 1) * w + x] = color;
        }
    }
    for (int y = by; y < by + bh; y++) {
        if (y < h) {
            if (bx < w) buf[y * w + bx] = color;
            if ((bx + bw - 1) < w) buf[y * w + (bx + bw - 1)] = color;
        }
    }
}

void draw_hud(uint16_t* buf) {
    int y1 = MODEL_H * LINE_TOP_POS;
    int y2 = MODEL_H * LINE_BOT_POS;
    
    for(int x = 0; x < MODEL_W; x+=2) { 
        buf[y1 * MODEL_W + x] = 0xF800;
        buf[y2 * MODEL_W + x] = 0x001F; 
    }
}

void resize_to_ai_buf(camera_fb_t *fb, uint16_t* dest_buf) {
    int src_h = fb->height;
    int src_w = fb->height; 
    int src_off_x = (fb->width - src_w) / 2; 

    uint16_t* src_buf = (uint16_t*)fb->buf;
    for (int y_dst = 0; y_dst < MODEL_H; y_dst++) {
        int y_src = (y_dst * src_h) / MODEL_H;
        for (int x_dst = 0; x_dst < MODEL_W; x_dst++) {
            int x_src = (x_dst * src_w) / MODEL_W + src_off_x;
            dest_buf[y_dst * MODEL_W + x_dst] = src_buf[y_src * fb->width + x_src];
        }
    }
}

static esp_err_t stream_handler(httpd_req_t *req) {
    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;
    char part_buf[64];
    httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=frame");
    const char* _STREAM_BOUNDARY = "\r\n--frame\r\n";
    const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";
    while (true) {
        fb = esp_camera_fb_get();
        if (!fb) { res = ESP_FAIL; break; }
        
        if (!ai_is_running && !new_frame_reay) {
            resize_to_ai_buf(fb, ai_input_buf);
            new_frame_reay = true;
        }
        
        resize_to_ai_buf(fb, display_buf);
        xSemaphoreTake(xMutex, portMAX_DELAY);
        draw_hud(display_buf);
        for (int i = 0; i < confirmed_count; i++) {
            draw_face_box(display_buf, MODEL_W, MODEL_H, 
                          confirmed_boxes[i].x, confirmed_boxes[i].y, 
                          confirmed_boxes[i].w, confirmed_boxes[i].h, 0x07E0);
        }
        xSemaphoreGive(xMutex);

        uint8_t *jpg_buf = NULL;
        size_t jpg_len = 0;
        fmt2jpg((uint8_t*)display_buf, MODEL_W * MODEL_H * 2, MODEL_W, MODEL_H, PIXFORMAT_RGB565, 60, &jpg_buf, &jpg_len);
        esp_camera_fb_return(fb);
        if (res == ESP_OK) res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        if (res == ESP_OK) {
            size_t hlen = snprintf(part_buf, 64, _STREAM_PART, jpg_len);
            res = httpd_resp_send_chunk(req, part_buf, hlen);
        }
        if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char *)jpg_buf, jpg_len);
        if(jpg_buf) free(jpg_buf);
        if (res != ESP_OK) break;
    }
    return res;
}

static esp_err_t index_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    const char* html = 
    "<body>"
    "<h2>Hummel Modell Cam</h2>"
    "<img src='/stream' style='width:500px; image-rendering: pixelated; border:4px solid #333;' />"
    "</body>";
    return httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
}

void setup() {
    Serial.begin(115200);
    esp_log_level_set("*", ESP_LOG_ERROR);
    Serial.println("BeeSense V27 (Lite) startet...");

    // --- Sensoren & Display starten ---
    Wire.begin(PIN_QWIIC_SDA, PIN_QWIIC_SCL); [cite_start]

    // Display
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3D)) {
        Serial.println("OLED Fehler!");
    } else {
        display.setRotation(2); 
        display.clearDisplay();
        display.setTextColor(WHITE);
        display.setTextSize(1);
        display.setCursor(0,0);
        display.println("Start...");
        display.display();
    }

    // Sensoren
    if (!hdc.begin()) Serial.println("HDC Fehler!");
    uv.begin(VEML6070_1_T);
    if (!ltr.begin()) Serial.println("LTR Fehler!");
    ltr.setGain(LTR3XX_GAIN_1);
    ltr.setIntegrationTime(LTR3XX_INTEGTIME_100);
    
    Serial.println("Verbinde mit WLAN...");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
    Serial.println("\nWLAN verbunden!");
    Serial.print("IP: "); Serial.println(WiFi.localIP());

    xMutex = xSemaphoreCreateMutex();
    ai_input_buf = (uint16_t*)malloc(MODEL_W * MODEL_H * 2);
    display_buf = (uint16_t*)malloc(MODEL_W * MODEL_H * 2);

    Serial.println("Starte Kamera...");
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
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 16000000;
    config.pixel_format = PIXFORMAT_RGB565;
    config.frame_size = FRAMESIZE_QVGA;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    config.fb_count = 2; 

    if (esp_camera_init(&config) != ESP_OK) { Serial.println("Kamera Fehler!"); return; }
    sensor_t *s = esp_camera_sensor_get();
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);

    xTaskCreatePinnedToCore(aiTask, "AI Task", 8192, NULL, 1, NULL, 0);
    httpd_config_t config_httpd = HTTPD_DEFAULT_CONFIG();
    config_httpd.server_port = 80;
    config_httpd.max_open_sockets = 13; 
    
    httpd_uri_t index_uri = { "/", HTTP_GET, index_handler, NULL };
    httpd_uri_t stream_uri = { "/stream", HTTP_GET, stream_handler, NULL };
    
    if (httpd_start(&stream_httpd, &config_httpd) == ESP_OK) {
        httpd_register_uri_handler(stream_httpd, &index_uri);
        httpd_register_uri_handler(stream_httpd, &stream_uri);
    }
    Serial.println("Webserver läuft!");
}

void loop() { vTaskDelete(NULL); }
