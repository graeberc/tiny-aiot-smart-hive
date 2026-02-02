#include <HummelEdgeImpulse_inferencing.h>
#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"
#include <vector>
#include <algorithm>
#include "edge-impulse-sdk/dsp/image/image.hpp"

// Wlan Anpassen!
const char* ssid = "xxx";
const char* password = "xxx";

// --- CONFIG ---
#define CONFIDENCE_THRESHOLD EI_CLASSIFIER_OBJECT_DETECTION_THRESHOLD
#define PERSISTENCE_REQUIRED 1
#define MIN_BOX_SIZE 4
#define MAX_BOXES 2

#define DEBUG_RESULTS 1

#define MODEL_W EI_CLASSIFIER_INPUT_WIDTH
#define MODEL_H EI_CLASSIFIER_INPUT_HEIGHT

// Zähler-Linien (Prozentual im Fenster)
#define LINE_TOP_POS 0.25 
#define LINE_BOT_POS 0.75
#define TRACKING_DIST 60 

int cnt_in = 0;
int cnt_out = 0;

// PINS
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

// Buffer Definitionen
uint16_t *ai_input_buf = NULL; 
uint16_t *display_buf = NULL;  
uint16_t *crop_buf = NULL;
size_t crop_buf_pixels = 0;

const int DISP_W = 320;
const int DISP_H = 240;

bool ai_is_running = false;
bool new_frame_reay = false;
SemaphoreHandle_t xMutex;

struct Box { int x, y, w, h; float prob; int persistence; };
Box confirmed_boxes[MAX_BOXES];
int confirmed_count = 0;

// Tracking Vars
Box candidates[MAX_BOXES];
int candidate_count = 0;

struct BoxMap {
    int start_x;
    int start_y;
    float scale_x;
    float scale_y;
};
BoxMap last_map = {0, 0, 1.0f, 1.0f};

struct TrackedBee {
    int id;
    int cx;
    int cy;
    int last_cx;
    int last_cy;
    bool from_top;
    bool from_bot;
    bool counted;
    int lost_frames;
};
std::vector<TrackedBee> trackers;
int next_bee_id = 1;

static inline int box_center_x(const Box &b) { return b.x + b.w / 2; }
static inline int box_center_y(const Box &b) { return b.y + b.h / 2; }

float box_iou(const Box &a, const Box &b) {
    int x1 = max(a.x, b.x);
    int y1 = max(a.y, b.y);
    int x2 = min(a.x + a.w, b.x + b.w);
    int y2 = min(a.y + a.h, b.y + b.h);
    int inter_w = max(0, x2 - x1);
    int inter_h = max(0, y2 - y1);
    int inter_area = inter_w * inter_h;
    int area_a = a.w * a.h;
    int area_b = b.w * b.h;
    int union_area = area_a + area_b - inter_area;
    if (union_area <= 0) return 0.0f;
    return (float)inter_area / (float)union_area;
}

bool is_same_object(const Box &a, const Box &b) {
    int dx = box_center_x(a) - box_center_x(b);
    int dy = box_center_y(a) - box_center_y(b);
    int dist = sqrt(dx * dx + dy * dy);
    return (dist < 40);
}

void update_counters() {
    for (auto &t : trackers) t.lost_frames++;

    std::vector<bool> matched(trackers.size(), false);

    for (int i = 0; i < confirmed_count; i++) {
        int cx = box_center_x(confirmed_boxes[i]);
        int cy = box_center_y(confirmed_boxes[i]);
        int best_idx = -1;
        float best_dist = TRACKING_DIST;

        for (int j = 0; j < (int)trackers.size(); j++) {
            if (matched[j]) continue;
            int dx = trackers[j].cx - cx;
            int dy = trackers[j].cy - cy;
            float dist = sqrt(dx * dx + dy * dy);
            if (dist < best_dist) { best_dist = dist; best_idx = j; }
        }

        if (best_idx != -1) {
            matched[best_idx] = true;
            trackers[best_idx].last_cx = trackers[best_idx].cx;
            trackers[best_idx].last_cy = trackers[best_idx].cy;
            trackers[best_idx].cx = cx;
            trackers[best_idx].cy = cy;
            trackers[best_idx].lost_frames = 0;
        } else {
            TrackedBee t;
            t.id = next_bee_id++;
            t.cx = cx;
            t.cy = cy;
            t.last_cx = cx;
            t.last_cy = cy;
            t.from_top = false;
            t.from_bot = false;
            t.counted = false;
            t.lost_frames = 0;
            trackers.push_back(t);
            matched.push_back(true);
        }
    }

    int line_top = MODEL_H * LINE_TOP_POS;
    int line_bot = MODEL_H * LINE_BOT_POS;

    for (auto &t : trackers) {
        if (t.lost_frames > 2) continue;

        if (t.last_cy < line_top && t.cy >= line_top) t.from_top = true;
        if (t.last_cy > line_bot && t.cy <= line_bot) t.from_bot = true;

        if (!t.counted) {
            if (t.from_top && t.last_cy <= line_bot && t.cy > line_bot) {
                cnt_in++;
                t.counted = true;
                Serial.printf(">>> ZÄHLER: Hummel REIN! (In: %d | Out: %d)\n", cnt_in, cnt_out);
            } else if (t.from_bot && t.last_cy >= line_top && t.cy < line_top) {
                cnt_out++;
                t.counted = true;
                Serial.printf("<<< ZÄHLER: Hummel RAUS! (In: %d | Out: %d)\n", cnt_in, cnt_out);
            }
        }
    }

    for (int i = trackers.size() - 1; i >= 0; i--) {
        if (trackers[i].lost_frames > 10) trackers.erase(trackers.begin() + i);
    }
}

void aiTask(void * parameter) {
    while(true) {
        if (new_frame_reay) {
            ai_is_running = true;
            new_frame_reay = false; 

            signal_t signal;
            signal.total_length = MODEL_W * MODEL_H;
            signal.get_data = [](size_t offset, size_t length, float *out_ptr) {
                for (size_t i = 0; i < length; i++) {
                    size_t p_idx = offset + i;
                    uint16_t pix = ai_input_buf[p_idx];

                    uint8_t r = ((pix >> 11) & 0x1F) * (255.0f / 31.0f);
                    uint8_t g = ((pix >> 5) & 0x3F) * (255.0f / 63.0f);
                    uint8_t b = (pix & 0x1F) * (255.0f / 31.0f);

                    // Modell ist grayscale: 1 Kanal (0..255)
                    out_ptr[i] = 0.299f * r + 0.587f * g + 0.114f * b;
                }
                return 0;
            };

            ei_impulse_result_t result = { 0 };
            run_classifier(&signal, &result, false);

#if DEBUG_RESULTS
            Serial.printf("FOMO boxes: %d\n", (int)result.bounding_boxes_count);
            for (size_t ix = 0; ix < result.bounding_boxes_count; ix++) {
                auto bb = result.bounding_boxes[ix];
                if (bb.value == 0) continue;
                Serial.printf("- %s (%.2f) x:%d y:%d w:%d h:%d\n",
                              bb.label, bb.value, bb.x, bb.y, bb.width, bb.height);
            }
#endif

            xSemaphoreTake(xMutex, portMAX_DELAY);
            Box next_candidates[MAX_BOXES] = {};
            int next_cand_count = 0;
            confirmed_count = 0;

            std::vector<Box> raw_boxes;
            raw_boxes.reserve(result.bounding_boxes_count);
            for (size_t ix = 0; ix < result.bounding_boxes_count; ix++) {
                auto bb = result.bounding_boxes[ix];
                if (bb.value > CONFIDENCE_THRESHOLD && bb.width >= MIN_BOX_SIZE && bb.height >= MIN_BOX_SIZE) {
                    raw_boxes.push_back({bb.x, bb.y, bb.width, bb.height, bb.value, 1});
                }
            }

            std::sort(raw_boxes.begin(), raw_boxes.end(), [](const Box &a, const Box &b) {
                return a.prob > b.prob;
            });

            std::vector<Box> nms_boxes;
            for (size_t i = 0; i < raw_boxes.size(); i++) {
                bool keep = true;
                for (size_t j = 0; j < nms_boxes.size(); j++) {
                    if (box_iou(raw_boxes[i], nms_boxes[j]) > 0.5f) { keep = false; break; }
                }
                if (keep) {
                    nms_boxes.push_back(raw_boxes[i]);
                    if ((int)nms_boxes.size() >= MAX_BOXES) break;
                }
            }

            for (size_t ix = 0; ix < nms_boxes.size(); ix++) {
                Box newBox = nms_boxes[ix];
                for (int c = 0; c < candidate_count; c++) {
                    if (is_same_object(candidates[c], newBox)) {
                        newBox.persistence = candidates[c].persistence + 1;
                        break;
                    }
                }
                if (next_cand_count < MAX_BOXES) {
                    next_candidates[next_cand_count] = newBox;
                    next_cand_count++;
                }
                if (newBox.persistence >= PERSISTENCE_REQUIRED) {
                    if (confirmed_count < MAX_BOXES) {
                        confirmed_boxes[confirmed_count] = newBox;
                        confirmed_count++;
                        Serial.printf("HUMMEL erkannt! (%.2f, %d mal)\n", newBox.prob, newBox.persistence);
                    }
                }
            }
            candidate_count = next_cand_count;
            for (int i = 0; i < MAX_BOXES; i++) candidates[i] = next_candidates[i];
            update_counters();
            xSemaphoreGive(xMutex);
            ai_is_running = false; 
        }
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

void draw_face_box(uint16_t* buf, int w, int h, int bx, int by, int bw, int bh, uint16_t color) {
    if (bx < 0) bx = 0; if (by < 0) by = 0;
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
    int y1 = DISP_H * LINE_TOP_POS;
    int y2 = DISP_H * LINE_BOT_POS;
    
    // Rote Linie Oben, Blaue Linie Unten
    for(int x = 0; x < DISP_W; x+=2) { 
        buf[y1 * DISP_W + x] = 0xF800; 
        buf[y2 * DISP_W + x] = 0x001F; 
    }
}

static bool ensure_crop_buf(size_t pixels) {
    if (crop_buf_pixels >= pixels && crop_buf != NULL) return true;
    if (crop_buf) { free(crop_buf); crop_buf = NULL; crop_buf_pixels = 0; }
    crop_buf = (uint16_t*)malloc(pixels * 2);
    if (!crop_buf) return false;
    crop_buf_pixels = pixels;
    return true;
}

// Resize/Crop using Edge Impulse mode to match model input
void resize_to_model_buf(camera_fb_t *fb, uint16_t* dest_buf) {
#if (EI_CLASSIFIER_RESIZE_MODE == EI_CLASSIFIER_RESIZE_SQUASH)
    last_map.start_x = 0;
    last_map.start_y = 0;
    last_map.scale_x = (float)fb->width / (float)MODEL_W;
    last_map.scale_y = (float)fb->height / (float)MODEL_H;

    ei::image::processing::resize_image(
        (const uint8_t*)fb->buf,
        fb->width,
        fb->height,
        (uint8_t*)dest_buf,
        MODEL_W,
        MODEL_H,
        2);
#else
    int crop_w, crop_h;
    ei::image::processing::calculate_crop_dims(
        fb->width,
        fb->height,
        MODEL_W,
        MODEL_H,
        crop_w,
        crop_h);

    if (!ensure_crop_buf((size_t)crop_w * (size_t)crop_h)) return;

    int start_x = (fb->width - crop_w) / 2;
    int start_y = (fb->height - crop_h) / 2;

    last_map.start_x = start_x;
    last_map.start_y = start_y;
    last_map.scale_x = (float)crop_w / (float)MODEL_W;
    last_map.scale_y = (float)crop_h / (float)MODEL_H;

    int res = ei::image::processing::cropImage(
        (const uint8_t*)fb->buf,
        fb->width,
        fb->height,
        start_x,
        start_y,
        (uint8_t*)crop_buf,
        crop_w,
        crop_h,
        16);

    if (res != 0) return;

    ei::image::processing::resize_image(
        (const uint8_t*)crop_buf,
        crop_w,
        crop_h,
        (uint8_t*)dest_buf,
        MODEL_W,
        MODEL_H,
        2);
#endif
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
            resize_to_model_buf(fb, ai_input_buf);
            new_frame_reay = true;
        }

        if (fb->width == DISP_W && fb->height == DISP_H) {
            memcpy(display_buf, fb->buf, DISP_W * DISP_H * 2);
        } else {
            ei::image::processing::resize_image(
                (const uint8_t*)fb->buf,
                fb->width,
                fb->height,
                (uint8_t*)display_buf,
                DISP_W,
                DISP_H,
                2);
        }
        
        xSemaphoreTake(xMutex, portMAX_DELAY);
        draw_hud(display_buf);
        for (int i = 0; i < confirmed_count; i++) {
            int bx = last_map.start_x + (int)(confirmed_boxes[i].x * last_map.scale_x);
            int by = last_map.start_y + (int)(confirmed_boxes[i].y * last_map.scale_y);
            int bw = (int)(confirmed_boxes[i].w * last_map.scale_x);
            int bh = (int)(confirmed_boxes[i].h * last_map.scale_y);
            draw_face_box(display_buf, DISP_W, DISP_H, bx, by, bw, bh, 0x07E0);
        }
        xSemaphoreGive(xMutex);

        uint8_t *jpg_buf = NULL;
        size_t jpg_len = 0;
        
        fmt2jpg((uint8_t*)display_buf, DISP_W * DISP_H * 2, DISP_W, DISP_H, PIXFORMAT_RGB565, 80, &jpg_buf, &jpg_len);
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

// Index Handler
static esp_err_t index_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    const char* html = 
    "<body>"
    "<h2>Hummel Modell Cam</h2>"
    "<img src='/stream' style='width:640px; border:4px solid #333;' />"
    "</body>";
    return httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
}

void setup() {
    Serial.begin(115200);
    esp_log_level_set("*", ESP_LOG_ERROR);
    Serial.println("BeeSense V27 (Lite) startet...");
    
    Serial.println("Verbinde mit WLAN...");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
    Serial.println("\nWLAN verbunden!");
    Serial.print("IP: "); Serial.println(WiFi.localIP());

    xMutex = xSemaphoreCreateMutex();
    ai_input_buf = (uint16_t*)malloc(MODEL_W * MODEL_H * 2);
    display_buf = (uint16_t*)malloc(DISP_W * DISP_H * 2);

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
