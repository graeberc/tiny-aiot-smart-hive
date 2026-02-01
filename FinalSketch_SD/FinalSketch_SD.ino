#include <Arduino.h>
#include <HummelProjekt_inferencing.h>
#include "esp_camera.h"
#include <SPI.h>
#include <SD.h>
#include <vector>
#include <algorithm>
#include <math.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"

// --- CONFIG ---
#define CONFIDENCE_THRESHOLD 0.85
#define PERSISTENCE_REQUIRED 2
#define MIN_BOX_SIZE 12
#define MAX_BOXES 2

#define MODEL_W EI_CLASSIFIER_INPUT_WIDTH
#define MODEL_H EI_CLASSIFIER_INPUT_HEIGHT

// Zähler-Linien (Prozentual)
#define LINE_TOP_POS 0.25
#define LINE_BOT_POS 0.75
#define TRACKING_DIST 60

// Kamera-Pins
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

// SD Pins (VSPI)
#define VSPI_MISO 40
#define VSPI_MOSI 38
#define VSPI_SCLK 39
#define VSPI_SS   41

// SD
SPIClass sdspi(SPI);
#define COUNTER_FILE "/counter.txt"
int img_counter = 0;

int cnt_in = 0;
int cnt_out = 0;

enum CountEvent { COUNT_NONE, COUNT_IN, COUNT_OUT };

// Buffers
uint16_t *ai_input_buf = NULL;
uint16_t *crop_buf = NULL;
size_t crop_buf_pixels = 0;

struct Box { int x, y, w, h; float prob; int persistence; };
Box confirmed_boxes[MAX_BOXES];
int confirmed_count = 0;
Box candidates[MAX_BOXES];
int candidate_count = 0;

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

static bool ensure_crop_buf(size_t pixels) {
    if (crop_buf_pixels >= pixels && crop_buf != NULL) return true;
    if (crop_buf) { free(crop_buf); crop_buf = NULL; crop_buf_pixels = 0; }
    crop_buf = (uint16_t*)malloc(pixels * 2);
    if (!crop_buf) return false;
    crop_buf_pixels = pixels;
    return true;
}

void resize_to_model_buf(camera_fb_t *fb, uint16_t* dest_buf) {
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
}

CountEvent update_counters() {
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

    CountEvent evt = COUNT_NONE;

    for (auto &t : trackers) {
        if (t.lost_frames > 2) continue;

        if (t.last_cy < line_top && t.cy >= line_top) t.from_top = true;
        if (t.last_cy > line_bot && t.cy <= line_bot) t.from_bot = true;

        if (!t.counted) {
            if (t.from_top && t.last_cy <= line_bot && t.cy > line_bot) {
                cnt_in++;
                t.counted = true;
                evt = COUNT_IN;
                Serial.printf(">>> ZÄHLER: Hummel REIN! (In: %d | Out: %d)\n", cnt_in, cnt_out);
            } else if (t.from_bot && t.last_cy >= line_top && t.cy < line_top) {
                cnt_out++;
                t.counted = true;
                evt = COUNT_OUT;
                Serial.printf("<<< ZÄHLER: Hummel RAUS! (In: %d | Out: %d)\n", cnt_in, cnt_out);
            }
        }
    }

    for (int i = trackers.size() - 1; i >= 0; i--) {
        if (trackers[i].lost_frames > 10) trackers.erase(trackers.begin() + i);
    }

    return evt;
}

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

void save_frame_to_sd(camera_fb_t *fb, CountEvent evt) {
    uint8_t *jpg_buf = NULL;
    size_t jpg_len = 0;

    if (!fmt2jpg((uint8_t*)fb->buf, fb->len, fb->width, fb->height, PIXFORMAT_RGB565, 80, &jpg_buf, &jpg_len)) {
        Serial.println("JPG Konvertierung fehlgeschlagen");
        return;
    }

    char filename[40];
    const char *tag = (evt == COUNT_IN) ? "in" : "out";
    sprintf(filename, "/%s_%05d.jpg", tag, img_counter);

    File file = SD.open(filename, FILE_WRITE);
    if (file) {
        file.write(jpg_buf, jpg_len);
        file.close();
        Serial.printf("Gespeichert: %s\n", filename);
        img_counter++;
        saveCounter();
    } else {
        Serial.println("SD Schreibfehler");
    }

    if (jpg_buf) free(jpg_buf);
}

void setup() {
    Serial.begin(115200);
    esp_log_level_set("*", ESP_LOG_ERROR);
    Serial.println("BeeSense SD Sketch startet...");

    ai_input_buf = (uint16_t*)malloc(MODEL_W * MODEL_H * 2);
    if (!ai_input_buf) { Serial.println("AI Buffer Fehler"); while(1); }

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

    if (esp_camera_init(&config) != ESP_OK) { Serial.println("Kamera Fehler!"); while(1); }
    sensor_t *s = esp_camera_sensor_get();
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);

    sdspi.begin(VSPI_SCLK, VSPI_MISO, VSPI_MOSI, VSPI_SS);
    if (!SD.begin(VSPI_SS, sdspi, 4000000)) { Serial.println("SD Init Fehler"); while(1); }
    loadCounter();
}

void loop() {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) { Serial.println("Capture Fehler"); return; }

    resize_to_model_buf(fb, ai_input_buf);

    signal_t signal;
    signal.total_length = MODEL_W * MODEL_H;
    signal.get_data = [](size_t offset, size_t length, float *out_ptr) {
        for (size_t i = 0; i < length; i++) {
            size_t p_idx = offset + i;
            uint16_t pix = ai_input_buf[p_idx];
            uint8_t r = ((pix >> 11) & 0x1F) * (255.0f / 31.0f);
            uint8_t g = ((pix >> 5) & 0x3F) * (255.0f / 63.0f);
            uint8_t b = (pix & 0x1F) * (255.0f / 31.0f);
            out_ptr[i] = (r << 16) | (g << 8) | b;
        }
        return 0;
    };

    ei_impulse_result_t result = { 0 };
    run_classifier(&signal, &result, false);

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
            }
        }
    }
    candidate_count = next_cand_count;
    for (int i = 0; i < MAX_BOXES; i++) candidates[i] = next_candidates[i];

    CountEvent evt = update_counters();

    if (evt != COUNT_NONE && confirmed_count > 0) {
        save_frame_to_sd(fb, evt);
    }

    esp_camera_fb_return(fb);
}
