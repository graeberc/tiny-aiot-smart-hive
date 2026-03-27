#include <HummelEdgeImpulse_inferencing.h>
#include "esp_camera.h"
#include <WiFi.h>
#include <Preferences.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include "esp_http_server.h"
#include <vector>
#include <algorithm>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "edge-impulse-sdk/classifier/postprocessing/ei_postprocessing_common.h"

// Typen für Erkennung/Tracking
struct Box {
    int x;
    int y;
    int w;
    int h;
    float prob;
    int persistence;
};

#if !((defined(EI_HAS_YOLOV5) && EI_HAS_YOLOV5) || (defined(EI_HAS_YOLOX) && EI_HAS_YOLOX) || (defined(EI_HAS_YOLOV7) && EI_HAS_YOLOV7) || (defined(EI_HAS_YOLOV2) && EI_HAS_YOLOV2) || (defined(EI_HAS_YOLOV11) && EI_HAS_YOLOV11) || (defined(EI_HAS_YOLO_PRO) && EI_HAS_YOLO_PRO) || (defined(EI_HAS_TAO_YOLO) && EI_HAS_TAO_YOLO) || (defined(EI_HAS_TAO_YOLOV3) && EI_HAS_TAO_YOLOV3) || (defined(EI_HAS_TAO_YOLOV4) && EI_HAS_TAO_YOLOV4))
#error "This sketch expects a YOLO Edge Impulse deployment (EI_HAS_YOLO* must be 1). Check that you're including the correct *_inferencing.h library."
#endif

// WLAN-Zugangsdaten (Anpassen!)
const char* ssid = "Perry";
const char* password = "mausbiber";

// OpenSenseMap Box- und Sensor-IDs
const char* OSM_BEE_BOX_ID = "69690d2fcbf9bc00079b077c";
const char* OSM_SENSOR_IN_ID = "696913c0cbf9bc0007a6706b";
const char* OSM_SENSOR_OUT_ID = "696913c0cbf9bc0007a6706d";
const char* OSM_SENSOR_TOTAL_ID = "696913c0cbf9bc0007a6706f";

// Mindest-Sicherheit für Erkennung (ggf. Anpassen!)
#define USER_CONFIDENCE_THRESHOLD 0.3f
#define CONFIDENCE_THRESHOLD USER_CONFIDENCE_THRESHOLD
// Bestätigte Frames bis Objekt als stabil gilt
#define PERSISTENCE_REQUIRED 1
// Minimale Boxgröße in Pixel
#define MIN_BOX_SIZE 3
// Maximale Anzahl verarbeiteter Boxen pro Frame
#define MAX_BOXES 10

// Setzt den Laufzeit-Schwellwert im EI-Postprocessing
static void set_postprocess_threshold(float threshold) {
    ei_impulse_handle_t* handle = &ei_default_impulse;
    if (!handle || !handle->impulse) return;

    for (size_t ix = 0; ix < handle->impulse->postprocessing_blocks_size; ix++) {
        const ei_postprocessing_block_t* block = &handle->impulse->postprocessing_blocks[ix];
        if (block->type != EI_CLASSIFIER_MODE_OBJECT_DETECTION || block->config == nullptr) {
            continue;
        }
        ei_fill_result_object_detection_i8_config_t* cfg =
            (ei_fill_result_object_detection_i8_config_t*)block->config;
        cfg->threshold = threshold;
    }
}

// Periodische Statusausgabe
#define PRINT_STATS 1
// Intervall der Statusausgabe in ms
#define STATS_PRINT_INTERVAL_MS 10000u

// Uploadintervall und Backoff-Verhalten
#define OSM_UPLOAD_INTERVAL_MS 10000u
#define OSM_FORCE_REFRESH_MS 300000u
#define OSM_POST_SPACING_MS 1500u
#define OSM_RETRY_429_DELAY_MS 5000u
#define OSM_BATCH_COOLDOWN_ON_429_MS 30000u
#define OSM_BATCH_COOLDOWN_ON_TRANSIENT_MS 15000u
// Zusätzliche Upload-Debuglogs
#define OSM_UPLOAD_DEBUG 0

// Zusammenfassung der Erkennungen im Log
#define HUMMEL_LOG_SUMMARY 1
#define HUMMEL_LOG_INTERVAL_MS 5000u

// Stream beim Start an/aus (0 = aus, 1 = an)
#define VIDEO_STREAM_DEFAULT_ENABLED 1

#define MODEL_W EI_CLASSIFIER_INPUT_WIDTH
#define MODEL_H EI_CLASSIFIER_INPUT_HEIGHT

// Obere/untere Zähllinie (0.0 bis 1.0)
#define LINE_UPPER_POS 0.35f
#define LINE_LOWER_POS 0.65f

// Tracking- und Zählverhalten
#define TRACKING_DIST 120
#define TRACKING_ACTIVE_LOST_FRAMES 8
#define TRACKING_REMOVE_LOST_FRAMES 12
#define TRACKING_REID_RESET_FRAMES 3
#define TRACKING_MAX_X_DRIFT 100
#define TRACKING_MAX_Y_DRIFT 80
#define TRACK_MIN_HITS_BEFORE_COUNT_IN 1
#define TRACK_MIN_HITS_BEFORE_COUNT_OUT 1
#define TRACK_COUNT_COOLDOWN_FRAMES 1
#define ENABLE_DIRECT_JUMP_COUNT 0
#define REMOVE_TRACK_AFTER_COUNT 1

// Startbestand an Hummeln im Stock 
#define START_STOCK_COUNT 100
// Verhindert negative Gesamtzahl
#define CLAMP_TOTAL_NON_NEGATIVE 1
// Speichern der Zählerwerte erst nach Ruhezeit
#define COUNTS_PERSIST_DEBOUNCE_MS 5000u

int cnt_in = 0;
int cnt_out = 0;
Preferences prefs;
volatile bool counts_dirty = false;
volatile uint32_t counts_last_change_ms = 0;

// Markiert, dass Zählerstände gespeichert werden müssen
static inline void mark_counts_dirty() {
    counts_last_change_ms = millis();
    counts_dirty = true;
}

// Lädt Zählerstände aus dem Flash (NVS)
static void load_counts_from_nvs() {
    cnt_in = prefs.getInt("in", 0);
    cnt_out = prefs.getInt("out", 0);
    Serial.printf("Zählerstand geladen: in=%d out=%d im_stock=%d\n", cnt_in, cnt_out, get_total_stock_count());
}

// Speichert Zählerstände in den Flash (NVS)
static void save_counts_to_nvs() {
    prefs.putInt("in", cnt_in);
    prefs.putInt("out", cnt_out);
}

// Hintergrundtask: speichert Zählerstände verzögert bei Änderungen
void persistCountsTask(void * parameter) {
    while (true) {
        if (counts_dirty) {
            uint32_t now = millis();
            if ((uint32_t)(now - counts_last_change_ms) >= COUNTS_PERSIST_DEBOUNCE_MS) {
                save_counts_to_nvs();
                counts_dirty = false;
            }
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

// Berechnet die aktuelle Stockanzahl
static inline int get_total_stock_count() {
    int total = START_STOCK_COUNT + cnt_in - cnt_out;
#if CLAMP_TOTAL_NON_NEGATIVE
    if (total < 0) total = 0;
#endif
    return total;
}

// Kamerapinbelegung
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
volatile int active_stream_sock = -1;

// Magic-Header für stabile Stream-Synchronisierung im Browser
static const uint8_t FRAME_MAGIC[4] = { 'H', 'B', 'E', 'E' };

// Allokiert bevorzugt PSRAM, sonst normalen RAM
static void* malloc_psram_fallback(size_t bytes) {
#ifdef ESP32
    void* p = ps_malloc(bytes);
    if (p) return p;
#endif
    return malloc(bytes);
}

// Arbeitsbuffer für Inferenz und Anzeige
uint8_t *ai_input_buf = NULL;
uint16_t *display_buf = NULL;
uint8_t *rgb888_frame_buf = NULL;
uint8_t *crop_buf = NULL;
size_t crop_buf_bytes = 0;

const int DISP_W = 320;
const int DISP_H = 240;

volatile bool ai_is_running = false;
volatile bool new_frame_reay = false;
volatile bool video_stream_enabled = (VIDEO_STREAM_DEFAULT_ENABLED != 0);
SemaphoreHandle_t xMutex;

Box confirmed_boxes[MAX_BOXES];
int confirmed_count = 0;

// Zustände für Kurzzeit-Tracking
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
    int hits;
    int count_cooldown;
    int lost_frames;
    uint8_t crossing_phase;
};
std::vector<TrackedBee> trackers;
int next_bee_id = 1;

// Sendet einen Messwert mit beliebigem Content-Type an openSenseMap
static int post_open_sense_map_value_once(const char* sensor_id, const String& body, const char* content_type) {
    if (WiFi.status() != WL_CONNECTED) return -1001;

    WiFiClientSecure client;
    client.setInsecure();

    HTTPClient http;
    String url = String("https://api.opensensemap.org/boxes/") + OSM_BEE_BOX_ID + "/" + sensor_id;
    if (!http.begin(client, url)) return -1000;

    http.setTimeout(8000);
    http.setReuse(false);

    http.addHeader("Content-Type", content_type);
    int code = http.POST(body);
    http.end();

    return code;
}

// Sendet einen Messwert im JSON-Format
static int post_open_sense_map_value(const char* sensor_id, int value) {
    if (sensor_id == NULL) return -1002;
    int code = post_open_sense_map_value_once(sensor_id, String("{\"value\":") + String(value) + "}", "application/json");
#if OSM_UPLOAD_DEBUG
    Serial.printf("oSEM try sensor=%s value=%d fmt=application/json code=%d\n", sensor_id, value, code);
    if (code >= 200 && code < 300) {
        Serial.printf("oSEM ok  sensor=%s fmt=application/json\n", sensor_id);
    }
#endif
    return code;
}

// Wiederholt Upload einmal bei HTTP 429
static int post_open_sense_map_value_with_retry(const char* sensor_id, int value) {
    int code = post_open_sense_map_value(sensor_id, value);
    if (code == 429) {
#if OSM_UPLOAD_DEBUG
    Serial.printf("oSEM retry sensor=%s after 429\n", sensor_id);
#endif
        vTaskDelay(OSM_RETRY_429_DELAY_MS / portTICK_PERIOD_MS);
        code = post_open_sense_map_value(sensor_id, value);
    }
    return code;
}

// Hintergrundtask: synchronisiert Zähler periodisch zu openSenseMap
void openSenseMapUploadTask(void * parameter) {
    uint32_t next_upload_ms = 0;
    uint32_t last_force_refresh_ms = 0;
    int last_sent_in = -1;
    int last_sent_out = -1;
    int last_sent_total = -1;

    while (true) {
        uint32_t now_ms = millis();
        if ((int32_t)(now_ms - next_upload_ms) >= 0) {
            next_upload_ms = now_ms + OSM_UPLOAD_INTERVAL_MS;

            if (WiFi.status() == WL_CONNECTED) {
                int in_now = cnt_in;
                int out_now = cnt_out;
                int total_now = get_total_stock_count();

                bool force_refresh = ((uint32_t)(now_ms - last_force_refresh_ms) >= OSM_FORCE_REFRESH_MS);
                bool send_in = force_refresh || (last_sent_in < 0) || (in_now != last_sent_in);
                bool send_out = force_refresh || (last_sent_out < 0) || (out_now != last_sent_out);
                bool send_total = force_refresh || (last_sent_total < 0) || (total_now != last_sent_total);

                if (force_refresh) {
                    last_force_refresh_ms = now_ms;
                }

                if (!send_in && !send_out && !send_total) {
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                    continue;
                }

                int code_in = 200;
                int code_out = 200;
                int code_total = 200;

                if (send_in) {
                    code_in = post_open_sense_map_value_with_retry(OSM_SENSOR_IN_ID, in_now);
                    if (code_in >= 200 && code_in < 300) last_sent_in = in_now;
                    vTaskDelay(OSM_POST_SPACING_MS / portTICK_PERIOD_MS);
                }

                if (send_out) {
                    code_out = post_open_sense_map_value_with_retry(OSM_SENSOR_OUT_ID, out_now);
                    if (code_out >= 200 && code_out < 300) last_sent_out = out_now;
                    vTaskDelay(OSM_POST_SPACING_MS / portTICK_PERIOD_MS);
                }

                if (send_total) {
                    code_total = post_open_sense_map_value_with_retry(OSM_SENSOR_TOTAL_ID, total_now);
                    if (code_total >= 200 && code_total < 300) last_sent_total = total_now;
                }

                if (code_in == 429 || code_out == 429 || code_total == 429) {
                    next_upload_ms = millis() + OSM_BATCH_COOLDOWN_ON_429_MS;
#if OSM_UPLOAD_DEBUG
                    Serial.printf("oSEM cooldown after 429: %lu ms\n", (unsigned long)OSM_BATCH_COOLDOWN_ON_429_MS);
#endif
                } else if (code_in <= 0 || code_out <= 0 || code_total <= 0 || code_in == 502 || code_out == 502 || code_total == 502) {
                    next_upload_ms = millis() + OSM_BATCH_COOLDOWN_ON_TRANSIENT_MS;
#if OSM_UPLOAD_DEBUG
                    Serial.printf("oSEM cooldown after transient error: %lu ms\n", (unsigned long)OSM_BATCH_COOLDOWN_ON_TRANSIENT_MS);
#endif
                }
            }
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

enum TrackZone : uint8_t {
    ZONE_ABOVE = 0,
    ZONE_BETWEEN = 1,
    ZONE_BELOW = 2,
};

// Ordnet y-Position in die drei Zonen ein
static inline uint8_t zone_from_y(int y, int upper_line, int lower_line) {
    if (y < upper_line) return (uint8_t)ZONE_ABOVE;
    if (y > lower_line) return (uint8_t)ZONE_BELOW;
    return (uint8_t)ZONE_BETWEEN;
}

// Berechnet die x-Mitte einer Box
static inline int box_center_x(const Box &b) { return b.x + b.w / 2; }
// Berechnet die y-Mitte einer Box
static inline int box_center_y(const Box &b) { return b.y + b.h / 2; }

// Berechnet IoU zweier Boxen
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

// Prüft, ob zwei Boxen zum gleichen Objekt gehören
bool is_same_object(const Box &a, const Box &b) {
    int dx = box_center_x(a) - box_center_x(b);
    int dy = box_center_y(a) - box_center_y(b);
    int dist = sqrt(dx * dx + dy * dy);
    return (dist < 40);
}

// Verfolgt Objekte über Frames und zählt REIN/RAUS
void update_counters(const Box* boxes, int box_count) {
    for (auto &t : trackers) t.lost_frames++;

    bool matched_local[64] = { false };
    bool reacquired_local[64] = { false };
    const int tracker_n = (int)trackers.size() > 64 ? 64 : (int)trackers.size();

    for (int i = 0; i < box_count; i++) {
        int cx = box_center_x(boxes[i]);
        int cy = box_center_y(boxes[i]);
        int best_idx = -1;
        float best_dist = TRACKING_DIST;

        for (int j = 0; j < tracker_n; j++) {
            if (matched_local[j]) continue;
            int pred_cx = trackers[j].cx + (trackers[j].cx - trackers[j].last_cx);
            int pred_cy = trackers[j].cy + (trackers[j].cy - trackers[j].last_cy);
            int dx = pred_cx - cx;
            int dy = pred_cy - cy;
            if (abs(dx) > TRACKING_MAX_X_DRIFT || abs(dy) > TRACKING_MAX_Y_DRIFT) continue;
            float dist = sqrt(dx * dx + dy * dy);
            if (dist < best_dist) { best_dist = dist; best_idx = j; }
        }

        if (best_idx != -1) {
            int prev_lost = trackers[best_idx].lost_frames;
            matched_local[best_idx] = true;
            if (prev_lost > TRACKING_REID_RESET_FRAMES) {
                trackers[best_idx].last_cx = cx;
                trackers[best_idx].last_cy = cy;
                trackers[best_idx].crossing_phase = 0;
                trackers[best_idx].count_cooldown = TRACK_COUNT_COOLDOWN_FRAMES;
                reacquired_local[best_idx] = true;
            } else {
                trackers[best_idx].last_cx = trackers[best_idx].cx;
                trackers[best_idx].last_cy = trackers[best_idx].cy;
            }
            trackers[best_idx].cx = cx;
            trackers[best_idx].cy = cy;
            trackers[best_idx].lost_frames = 0;
            trackers[best_idx].hits++;
        } else {
            TrackedBee t;
            t.id = next_bee_id++;
            t.cx = cx;
            t.cy = cy;
            t.last_cx = cx;
            t.last_cy = cy;
            t.hits = 1;
            t.count_cooldown = TRACK_COUNT_COOLDOWN_FRAMES;
            t.lost_frames = 0;
            t.crossing_phase = 0;
            trackers.push_back(t);
        }
    }

    int upper_line = (int)(MODEL_H * LINE_UPPER_POS);
    int lower_line = (int)(MODEL_H * LINE_LOWER_POS);

    auto remove_after_count = [](TrackedBee &tb) {
#if REMOVE_TRACK_AFTER_COUNT
        tb.lost_frames = TRACKING_REMOVE_LOST_FRAMES + 1;
        tb.crossing_phase = 0;
        tb.count_cooldown = 0;
#endif
    };

    for (int i = 0; i < (int)trackers.size(); i++) {
        auto &t = trackers[i];
        if (t.lost_frames > TRACKING_ACTIVE_LOST_FRAMES) continue;
        if (i < 64 && reacquired_local[i]) continue;
        if (t.count_cooldown > 0) t.count_cooldown--;

        uint8_t prev_zone = zone_from_y(t.last_cy, upper_line, lower_line);
        uint8_t curr_zone = zone_from_y(t.cy, upper_line, lower_line);

#if ENABLE_DIRECT_JUMP_COUNT
        if (t.count_cooldown == 0 && prev_zone == (uint8_t)ZONE_ABOVE && curr_zone == (uint8_t)ZONE_BELOW && t.hits >= TRACK_MIN_HITS_BEFORE_COUNT_IN) {
                cnt_in++;
            mark_counts_dirty();
                t.count_cooldown = TRACK_COUNT_COOLDOWN_FRAMES;
                t.crossing_phase = 0;
            remove_after_count(t);
                Serial.printf(">>> ZÄHLER: Hummel REIN! (In: %d | Out: %d)\n", cnt_in, cnt_out);
                continue;
        } else if (t.count_cooldown == 0 && prev_zone == (uint8_t)ZONE_BELOW && curr_zone == (uint8_t)ZONE_ABOVE && t.hits >= TRACK_MIN_HITS_BEFORE_COUNT_OUT) {
                cnt_out++;
            mark_counts_dirty();
                t.count_cooldown = TRACK_COUNT_COOLDOWN_FRAMES;
                t.crossing_phase = 0;
            remove_after_count(t);
                Serial.printf("<<< ZÄHLER: Hummel RAUS! (In: %d | Out: %d)\n", cnt_in, cnt_out);
                continue;
        }
#endif

        if (prev_zone == ZONE_ABOVE && curr_zone == ZONE_BETWEEN) {
            t.crossing_phase = 1;
        } else if (prev_zone == ZONE_BELOW && curr_zone == ZONE_BETWEEN) {
            t.crossing_phase = 2;
        }

        if (t.count_cooldown == 0 && t.crossing_phase == 1 && curr_zone == ZONE_BELOW && t.hits >= TRACK_MIN_HITS_BEFORE_COUNT_IN) {
                cnt_in++;
            mark_counts_dirty();
                t.count_cooldown = TRACK_COUNT_COOLDOWN_FRAMES;
                t.crossing_phase = 0;
            remove_after_count(t);
                Serial.printf(">>> ZÄHLER: Hummel REIN! (In: %d | Out: %d)\n", cnt_in, cnt_out);
        } else if (t.count_cooldown == 0 && t.crossing_phase == 2 && curr_zone == ZONE_ABOVE && t.hits >= TRACK_MIN_HITS_BEFORE_COUNT_OUT) {
                cnt_out++;
            mark_counts_dirty();
                t.count_cooldown = TRACK_COUNT_COOLDOWN_FRAMES;
                t.crossing_phase = 0;
            remove_after_count(t);
                Serial.printf("<<< ZÄHLER: Hummel RAUS! (In: %d | Out: %d)\n", cnt_in, cnt_out);
        }

        if (curr_zone == prev_zone && curr_zone != ZONE_BETWEEN) {
            t.crossing_phase = 0;
        }
    }

    for (int i = trackers.size() - 1; i >= 0; i--) {
        if (trackers[i].lost_frames > TRACKING_REMOVE_LOST_FRAMES) trackers.erase(trackers.begin() + i);
    }
}

// Haupttask: Inferenz, Filterung, Tracking-Update und Statistik
void aiTask(void * parameter) {
    uint32_t stats_last_ms = 0;
    uint32_t stats_frames = 0;
    uint32_t last_infer_ms = 0;
    uint32_t hummel_log_last_ms = 0;
    uint32_t hummel_log_events = 0;
    float hummel_log_best_prob = 0.0f;
    while(true) {
        if (new_frame_reay) {
            ai_is_running = true;
            new_frame_reay = false; 

            signal_t signal;
            const size_t pixel_count = (size_t)MODEL_W * (size_t)MODEL_H;
            signal.total_length = pixel_count;
            signal.get_data = [](size_t offset, size_t length, float *out_ptr) {
                const size_t pixel_count_cb = (size_t)MODEL_W * (size_t)MODEL_H;
                for (size_t i = 0; i < length; i++) {
                    size_t pixel_ix = offset + i;
                    if (pixel_ix >= pixel_count_cb) {
                        out_ptr[i] = 0.0f;
                        continue;
                    }
                    size_t base = pixel_ix * 3;
                    uint32_t r = ai_input_buf[base + 0];
                    uint32_t g = ai_input_buf[base + 1];
                    uint32_t b = ai_input_buf[base + 2];
                    out_ptr[i] = (float)((r << 16) | (g << 8) | b);
                }
                return 0;
            };

            ei_impulse_result_t result = { 0 };
            uint32_t t0 = millis();
            EI_IMPULSE_ERROR rc = run_classifier(&signal, &result, false);
            uint32_t t1 = millis();
            last_infer_ms = (t1 - t0);
            if (rc != EI_IMPULSE_OK) {
                Serial.printf("run_classifier failed: %d\n", (int)rc);
                ai_is_running = false;
                vTaskDelay(50 / portTICK_PERIOD_MS);
                continue;
            }

            stats_frames++;

            Box next_candidates[MAX_BOXES] = {};
            int next_cand_count = 0;
            Box next_confirmed[MAX_BOXES] = {};
            int next_confirmed_count = 0;

            constexpr int kMaxDet =
#if defined(EI_CLASSIFIER_MAX_OBJECT_DETECTION_COUNT)
                EI_CLASSIFIER_MAX_OBJECT_DETECTION_COUNT;
#else
                10;
#endif

            Box raw_boxes[kMaxDet] = {};
            int raw_count = 0;
            for (size_t ix = 0; ix < result.bounding_boxes_count; ix++) {
                auto bb = result.bounding_boxes[ix];
                if (bb.value > CONFIDENCE_THRESHOLD && bb.width >= MIN_BOX_SIZE && bb.height >= MIN_BOX_SIZE) {
                    if (raw_count < kMaxDet) {
                        raw_boxes[raw_count++] = { bb.x, bb.y, bb.width, bb.height, bb.value, 1 };
                    }
                }
            }

            std::sort(raw_boxes, raw_boxes + raw_count, [](const Box &a, const Box &b) {
                return a.prob > b.prob;
            });

            Box nms_boxes[MAX_BOXES] = {};
            int nms_count = 0;
            for (int i = 0; i < raw_count; i++) {
                bool keep = true;
                for (int j = 0; j < nms_count; j++) {
                    if (box_iou(raw_boxes[i], nms_boxes[j]) > 0.5f) { keep = false; break; }
                }
                if (keep) {
                    nms_boxes[nms_count++] = raw_boxes[i];
                    if (nms_count >= MAX_BOXES) break;
                }
            }

#if PRINT_STATS
            if (stats_last_ms == 0) {
                stats_last_ms = t1;
                stats_frames = 0;
            } else if ((t1 - stats_last_ms) >= STATS_PRINT_INTERVAL_MS) {
                const uint32_t elapsed_ms = (uint32_t)(t1 - stats_last_ms);
                float fps = (elapsed_ms > 0) ? ((float)stats_frames * 1000.0f / (float)elapsed_ms) : 0.0f;

                Serial.printf("infer=%lums fps=%.2f in=%d out=%d im_stock=%d\n",
                              (unsigned long)last_infer_ms,
                              fps,
                              cnt_in,
                              cnt_out,
                              get_total_stock_count());

                stats_last_ms = t1;
                stats_frames = 0;
            }
#endif

            for (int ix = 0; ix < nms_count; ix++) {
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
                    if (next_confirmed_count < MAX_BOXES) {
                        next_confirmed[next_confirmed_count++] = newBox;
#if HUMMEL_LOG_SUMMARY
                        hummel_log_events++;
                        if (newBox.prob > hummel_log_best_prob) {
                            hummel_log_best_prob = newBox.prob;
                        }
#endif
                    }
                }
            }

#if HUMMEL_LOG_SUMMARY
            if ((t1 - hummel_log_last_ms) >= HUMMEL_LOG_INTERVAL_MS) {
                if (hummel_log_events > 0) {
                    Serial.printf("HUMMEL erkannt: %lux (max p=%.2f / %lus)\n",
                                  (unsigned long)hummel_log_events,
                                  (double)hummel_log_best_prob,
                                  (unsigned long)(HUMMEL_LOG_INTERVAL_MS / 1000u));
                }
                hummel_log_last_ms = t1;
                hummel_log_events = 0;
                hummel_log_best_prob = 0.0f;
            }
#endif

            if (xSemaphoreTake(xMutex, 10 / portTICK_PERIOD_MS) == pdTRUE) {
                candidate_count = next_cand_count;
                for (int i = 0; i < MAX_BOXES; i++) candidates[i] = next_candidates[i];
                confirmed_count = next_confirmed_count;
                for (int i = 0; i < MAX_BOXES; i++) confirmed_boxes[i] = next_confirmed[i];
                xSemaphoreGive(xMutex);
            }

            update_counters(next_confirmed, next_confirmed_count);
            ai_is_running = false; 
        }
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

// Hintergrundtask: liefert Frames für Inferenz im headless Betrieb
void frameFeedTask(void * parameter) {
    while (true) {
        bool stream_client_connected = (active_stream_sock >= 0);
        if ((!video_stream_enabled || !stream_client_connected) && !ai_is_running && !new_frame_reay) {
            camera_fb_t *fb = esp_camera_fb_get();
            if (fb) {
                if (rgb888_frame_buf && ai_input_buf) {
                    resize_to_model_buf(fb, ai_input_buf);
                    new_frame_reay = true;
                }
                esp_camera_fb_return(fb);
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

// Zeichnet eine Rechteckbox in den Ausgabepuffer
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

// Zeichnet die beiden Zähllinien
void draw_hud(uint16_t* buf) {
    int y_upper = (int)(DISP_H * LINE_UPPER_POS);
    int y_lower = (int)(DISP_H * LINE_LOWER_POS);

    for (int x = 0; x < DISP_W; x += 2) {
        if (y_upper >= 0 && y_upper < DISP_H) {
            buf[y_upper * DISP_W + x] = 0xFFFF;
        }
        if (y_lower >= 0 && y_lower < DISP_H) {
            buf[y_lower * DISP_W + x] = 0xFFE0;
        }
    }
}

// Stellt sicher, dass der Crop-Buffer groß genug ist
static bool ensure_crop_buf_bytes(size_t bytes) {
    if (crop_buf_bytes >= bytes && crop_buf != NULL) return true;
    if (crop_buf) { free(crop_buf); crop_buf = NULL; crop_buf_bytes = 0; }
    crop_buf = (uint8_t*)malloc_psram_fallback(bytes);
    if (!crop_buf) return false;
    crop_buf_bytes = bytes;
    return true;
}

// Konvertiert RGB565 nach RGB888
static inline void rgb565_to_rgb888(const uint8_t *src, uint8_t *dst, size_t pixels) {
    for (size_t i = 0; i < pixels; i++) {
        uint16_t pix = ((uint16_t)src[i * 2] << 8) | (uint16_t)src[i * 2 + 1];
        uint8_t r = (uint8_t)(((pix >> 11) & 0x1F) << 3);
        uint8_t g = (uint8_t)(((pix >> 5) & 0x3F) << 2);
        uint8_t b = (uint8_t)((pix & 0x1F) << 3);
        dst[i * 3 + 0] = r;
        dst[i * 3 + 1] = g;
        dst[i * 3 + 2] = b;
    }
}

// Skaliert/Kropt Kameraframe auf Eingangsgröße des Modells
void resize_to_model_buf(camera_fb_t *fb, uint8_t* dest_buf) {
    const size_t src_pixels = (size_t)fb->width * (size_t)fb->height;
    if (!rgb888_frame_buf) return;
    rgb565_to_rgb888((const uint8_t*)fb->buf, rgb888_frame_buf, src_pixels);

#if (EI_CLASSIFIER_RESIZE_MODE == EI_CLASSIFIER_RESIZE_SQUASH)
    last_map.start_x = 0;
    last_map.start_y = 0;
    last_map.scale_x = (float)fb->width / (float)MODEL_W;
    last_map.scale_y = (float)fb->height / (float)MODEL_H;

    ei::image::processing::resize_image(
        (const uint8_t*)rgb888_frame_buf,
        fb->width,
        fb->height,
        (uint8_t*)dest_buf,
        MODEL_W,
        MODEL_H,
        3);
#else
    int crop_w, crop_h;
    ei::image::processing::calculate_crop_dims(
        fb->width,
        fb->height,
        MODEL_W,
        MODEL_H,
        crop_w,
        crop_h);

    if (!ensure_crop_buf_bytes((size_t)crop_w * (size_t)crop_h * 3u)) return;

    int start_x = (fb->width - crop_w) / 2;
    int start_y = (fb->height - crop_h) / 2;

    last_map.start_x = start_x;
    last_map.start_y = start_y;
    last_map.scale_x = (float)crop_w / (float)MODEL_W;
    last_map.scale_y = (float)crop_h / (float)MODEL_H;

    int res = ei::image::processing::crop_image_rgb888_packed(
        (const uint8_t*)rgb888_frame_buf,
        fb->width,
        fb->height,
        start_x,
        start_y,
        (uint8_t*)crop_buf,
        crop_w,
        crop_h);
    if (res != 0) return;

    ei::image::processing::resize_image(
        (const uint8_t*)crop_buf,
        crop_w,
        crop_h,
        (uint8_t*)dest_buf,
        MODEL_W,
        MODEL_H,
        3);
#endif
}

// Stream-Endpunkt: liefert Rohframes und triggert nebenbei Inferenz
static esp_err_t stream_handler(httpd_req_t *req) {
    if (!video_stream_enabled) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_set_type(req, "text/plain");
        return httpd_resp_send(req, "Stream deaktiviert", HTTPD_RESP_USE_STRLEN);
    }

    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;
    int req_sock = httpd_req_to_sockfd(req);
    active_stream_sock = req_sock;
    httpd_resp_set_type(req, "application/octet-stream");

    while (true) {
        if (!video_stream_enabled) {
            res = httpd_resp_send_chunk(req, NULL, 0);
            break;
        }

        fb = esp_camera_fb_get();
        if (!fb) { res = ESP_FAIL; break; }
        
        if (!ai_is_running && !new_frame_reay) {
            if (rgb888_frame_buf && ai_input_buf) {
                resize_to_model_buf(fb, ai_input_buf);
                new_frame_reay = true;
            }
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
        
        draw_hud(display_buf);
        if (xSemaphoreTake(xMutex, 0) == pdTRUE) {
            for (int i = 0; i < confirmed_count; i++) {
                int bx = last_map.start_x + (int)(confirmed_boxes[i].x * last_map.scale_x);
                int by = last_map.start_y + (int)(confirmed_boxes[i].y * last_map.scale_y);
                int bw = (int)(confirmed_boxes[i].w * last_map.scale_x);
                int bh = (int)(confirmed_boxes[i].h * last_map.scale_y);
                draw_face_box(display_buf, DISP_W, DISP_H, bx, by, bw, bh, 0x07E0);
            }
            xSemaphoreGive(xMutex);
        }

        esp_camera_fb_return(fb);

        if (res == ESP_OK) {
            const uint32_t payload_size = (uint32_t)((size_t)DISP_W * (size_t)DISP_H * 2u);
            uint8_t header[20];
            int in_now = cnt_in;
            int out_now = cnt_out;
            int total_now = get_total_stock_count();
            header[0] = FRAME_MAGIC[0];
            header[1] = FRAME_MAGIC[1];
            header[2] = FRAME_MAGIC[2];
            header[3] = FRAME_MAGIC[3];
            header[4] = (uint8_t)(payload_size & 0xFF);
            header[5] = (uint8_t)((payload_size >> 8) & 0xFF);
            header[6] = (uint8_t)((payload_size >> 16) & 0xFF);
            header[7] = (uint8_t)((payload_size >> 24) & 0xFF);
            header[8] = (uint8_t)(in_now & 0xFF);
            header[9] = (uint8_t)((in_now >> 8) & 0xFF);
            header[10] = (uint8_t)((in_now >> 16) & 0xFF);
            header[11] = (uint8_t)((in_now >> 24) & 0xFF);
            header[12] = (uint8_t)(out_now & 0xFF);
            header[13] = (uint8_t)((out_now >> 8) & 0xFF);
            header[14] = (uint8_t)((out_now >> 16) & 0xFF);
            header[15] = (uint8_t)((out_now >> 24) & 0xFF);
            header[16] = (uint8_t)(total_now & 0xFF);
            header[17] = (uint8_t)((total_now >> 8) & 0xFF);
            header[18] = (uint8_t)((total_now >> 16) & 0xFF);
            header[19] = (uint8_t)((total_now >> 24) & 0xFF);

            res = httpd_resp_send_chunk(req, (const char *)header, sizeof(header));
            if (res == ESP_OK) {
                res = httpd_resp_send_chunk(req, (const char *)display_buf, (size_t)payload_size);
            }
        }
        if (res != ESP_OK) break;
    }
    if (active_stream_sock == req_sock) {
        active_stream_sock = -1;
    }
    return res;
}

// Schaltet Videostream ein
static esp_err_t stream_on_handler(httpd_req_t *req) {
    video_stream_enabled = true;
    Serial.println("[stream] ON");
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_send(req, "stream=on", HTTPD_RESP_USE_STRLEN);
}

// Schaltet Videostream aus und trennt aktive Session
static esp_err_t stream_off_handler(httpd_req_t *req) {
    video_stream_enabled = false;
    int sock = active_stream_sock;
    if (sock >= 0 && stream_httpd != NULL) {
        httpd_sess_trigger_close(stream_httpd, sock);
        active_stream_sock = -1;
    }
    Serial.println("[stream] OFF");
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_send(req, "stream=off", HTTPD_RESP_USE_STRLEN);
}

// Liefert aktuellen Stream-Status als JSON
static esp_err_t stream_status_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/json");
    const char* v = video_stream_enabled ? "{\"stream\":true}" : "{\"stream\":false}";
    return httpd_resp_send(req, v, HTTPD_RESP_USE_STRLEN);
}

// Liefert Zählerstände als JSON
static esp_err_t counts_handler(httpd_req_t *req) {
    char json[96];
    int in_now = cnt_in;
    int out_now = cnt_out;
    int total_now = get_total_stock_count();
    snprintf(json, sizeof(json), "{\"in\":%d,\"out\":%d,\"total\":%d}", in_now, out_now, total_now);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
}

// Liefert die Weboberfläche mit Livebild und Zähleranzeige
static esp_err_t index_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    const char* html = 
    "<body style='font-family:Arial, sans-serif;'>"
    "<h2>Hummel Livecam</h2>"
    "<div style='margin-bottom:10px;'>"
    "<button id='btnOn'>Stream AN</button> "
    "<button id='btnOff'>Stream AUS </button> "
    "<span id='st'></span>"
    "</div>"
    "<div id='counts' style='margin-bottom:8px;font-size:18px;font-weight:bold;'>In: - | Out: - | Im Stock: -</div>"
    "<canvas id='view' width='320' height='240' style='width:640px; border:4px solid #333; image-rendering:pixelated;'></canvas>"
    "<script>"
    "const w=320,h=240,frameSize=w*h*2;"
    "const HEADER_SIZE=20;"
    "const canvas=document.getElementById('view');"
    "const ctx=canvas.getContext('2d');"
    "const img=ctx.createImageData(w,h);"
    "const st=document.getElementById('st');"
    "const countsEl=document.getElementById('counts');"
    "const btnOn=document.getElementById('btnOn');"
    "const btnOff=document.getElementById('btnOff');"
    "const MAGIC=[72,66,69,69];"
    "let streamEnabled=true;"
    "let streamRunning=false;"
    "let streamToken=0;"
    "let activeReader=null;"
    "let activeAbort=null;"
    "let buf=new Uint8Array(0);"

    "async function refreshStatus(){"
    "  try{"
    "    const r=await fetch('/stream/status',{cache:'no-store'});"
    "    const j=await r.json();"
    "    streamEnabled=!!j.stream;"
    "    st.textContent='Status: '+(streamEnabled?'AN':'AUS');"
    "  }catch(e){}"
    "}"

    "async function refreshCounts(){"
    "  try{"
    "    const r=await fetch('/counts',{cache:'no-store'});"
    "    const j=await r.json();"
    "    countsEl.textContent='In: '+j.in+' | Out: '+j.out+' | Im Stock: '+j.total;"
    "  }catch(e){}"
    "}"

    "function drawFrame(frame){"
    "  for(let i=0,j=0;i<frameSize;i+=2,j+=4){"
    "    const pix=(frame[i]<<8)|frame[i+1];"
    "    const r=((pix>>11)&0x1f)<<3;"
    "    const g=((pix>>5)&0x3f)<<2;"
    "    const b=(pix&0x1f)<<3;"
    "    img.data[j]=r; img.data[j+1]=g; img.data[j+2]=b; img.data[j+3]=255;"
    "  }"
    "  ctx.putImageData(img,0,0);"
    "}"

    "function findMagic(b,start){"
    "  for(let i=start;i<=b.length-4;i++){"
    "    if(b[i]===MAGIC[0]&&b[i+1]===MAGIC[1]&&b[i+2]===MAGIC[2]&&b[i+3]===MAGIC[3]) return i;"
    "  }"
    "  return -1;"
    "}"

    "function processBuffer(){"
    "  while(true){"
    "    const p=findMagic(buf,0);"
    "    if(p<0){"
    "      if(buf.length>3) buf=buf.slice(buf.length-3);"
    "      return;"
    "    }"
    "    if(p>0) buf=buf.slice(p);"
    "    if(buf.length<HEADER_SIZE) return;"
    "    const size=(buf[4])|(buf[5]<<8)|(buf[6]<<16)|(buf[7]<<24);"
    "    if(size!==frameSize){ buf=buf.slice(1); continue; }"
    "    const inCount=(buf[8])|(buf[9]<<8)|(buf[10]<<16)|(buf[11]<<24);"
    "    const outCount=(buf[12])|(buf[13]<<8)|(buf[14]<<16)|(buf[15]<<24);"
    "    const totalCount=(buf[16])|(buf[17]<<8)|(buf[18]<<16)|(buf[19]<<24);"
    "    countsEl.textContent='In: '+inCount+' | Out: '+outCount+' | Im Stock: '+totalCount;"
    "    if(buf.length<HEADER_SIZE+size) return;"
    "    const frame=buf.slice(HEADER_SIZE,HEADER_SIZE+size);"
    "    buf=buf.slice(HEADER_SIZE+size);"
    "    drawFrame(frame);"
    "  }"
    "}"

    "async function startStreamLoop(){"
    "  if(streamRunning || !streamEnabled) return;"
    "  const myToken=++streamToken;"
    "  buf=new Uint8Array(0);"
    "  streamRunning=true;"
    "  while(streamEnabled && myToken===streamToken){"
    "    try{"
    "      activeAbort=new AbortController();"
    "      const r=await fetch('/stream',{cache:'no-store',signal:activeAbort.signal});"
    "      if(!r.ok || !r.body) { await new Promise(s=>setTimeout(s,300)); continue; }"
    "      activeReader=r.body.getReader();"
    "      while(streamEnabled && myToken===streamToken){"
    "        const out=await activeReader.read();"
    "        if(out.done) break;"
    "        const value=out.value;"
    "        if(value){"
    "          const tmp=new Uint8Array(buf.length+value.length);"
    "          tmp.set(buf,0); tmp.set(value,buf.length); buf=tmp;"
    "          processBuffer();"
    "        }"
    "      }"
    "      try{ if(activeReader) await activeReader.cancel(); }catch(e){}"
    "      activeReader=null;"
    "      activeAbort=null;"
    "    }catch(e){"
    "      activeReader=null;"
    "      activeAbort=null;"
    "      await new Promise(s=>setTimeout(s,500));"
    "    }"
    "  }"
    "  streamRunning=false;"
    "}"

    "btnOn.onclick=async()=>{"
    "  await fetch('/stream/on',{method:'POST'});"
    "  streamEnabled=true;"
    "  await refreshStatus();"
    "  startStreamLoop();"
    "};"
    "btnOff.onclick=async()=>{"
    "  await fetch('/stream/off',{method:'POST'});"
    "  streamEnabled=false;"
    "  streamToken++;"
    "  try{ if(activeReader) await activeReader.cancel(); }catch(e){}"
    "  try{ if(activeAbort) activeAbort.abort(); }catch(e){}"
    "  activeReader=null;"
    "  activeAbort=null;"
    "  buf=new Uint8Array(0);"
    "  await refreshStatus();"
    "};"

    "function stopLocalStreamNow(){"
    "  streamEnabled=false;"
    "  streamToken++;"
    "  try{ if(activeReader) activeReader.cancel(); }catch(e){}"
    "  try{ if(activeAbort) activeAbort.abort(); }catch(e){}"
    "  activeReader=null;"
    "  activeAbort=null;"
    "  buf=new Uint8Array(0);"
    "}"

    "window.addEventListener('pagehide',()=>{"
    "  stopLocalStreamNow();"
    "  try{ navigator.sendBeacon('/stream/off','off'); }catch(e){}"
    "});"

    "window.addEventListener('beforeunload',()=>{"
    "  stopLocalStreamNow();"
    "  try{ navigator.sendBeacon('/stream/off','off'); }catch(e){}"
    "});"

    "setInterval(refreshStatus,1000);"
    "setInterval(refreshCounts,5000);"
    "refreshStatus().then(()=>{ if(streamEnabled) startStreamLoop(); });"
    "refreshCounts();"
    "</script>"
    "</body>";
    return httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
}

// Initialisiert WLAN, Kamera, Tasks und Webserver
void setup() {
    Serial.begin(115200);
    esp_log_level_set("*", ESP_LOG_ERROR);
    Serial.println("BEEhavior startet...");

    set_postprocess_threshold(USER_CONFIDENCE_THRESHOLD);

    prefs.begin("bee-counts", false);
    load_counts_from_nvs();
    
    Serial.println("Verbinde mit WLAN...");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
    Serial.println("\nWLAN verbunden!");
    Serial.print("IP: "); Serial.println(WiFi.localIP());

    xMutex = xSemaphoreCreateMutex();
    ai_input_buf = (uint8_t*)malloc_psram_fallback((size_t)MODEL_W * (size_t)MODEL_H * 3u);
    rgb888_frame_buf = (uint8_t*)malloc_psram_fallback((size_t)DISP_W * (size_t)DISP_H * 3u);
    display_buf = (uint16_t*)malloc_psram_fallback((size_t)DISP_W * (size_t)DISP_H * 2u);
    if (!ai_input_buf || !rgb888_frame_buf || !display_buf) {
        Serial.printf("ERR: buffer alloc failed (ai:%p rgb:%p disp:%p)\n", ai_input_buf, rgb888_frame_buf, display_buf);
        while (true) { delay(1000); }
    }

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

    xTaskCreatePinnedToCore(aiTask, "AI Task", 32768, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(frameFeedTask, "Frame Feed", 8192, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(openSenseMapUploadTask, "oSEM Upload", 8192, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(persistCountsTask, "Persist Counts", 4096, NULL, 1, NULL, 0);

    httpd_config_t config_httpd = HTTPD_DEFAULT_CONFIG();
    config_httpd.server_port = 80;
    config_httpd.max_open_sockets = 13; 
    config_httpd.stack_size = 8192;
    
    httpd_uri_t index_uri = { "/", HTTP_GET, index_handler, NULL };
    httpd_uri_t stream_uri = { "/stream", HTTP_GET, stream_handler, NULL };
    httpd_uri_t stream_on_uri = { "/stream/on", HTTP_POST, stream_on_handler, NULL };
    httpd_uri_t stream_off_uri = { "/stream/off", HTTP_POST, stream_off_handler, NULL };
    httpd_uri_t stream_status_uri = { "/stream/status", HTTP_GET, stream_status_handler, NULL };
    httpd_uri_t counts_uri = { "/counts", HTTP_GET, counts_handler, NULL };
    
    if (httpd_start(&stream_httpd, &config_httpd) == ESP_OK) {
        httpd_register_uri_handler(stream_httpd, &index_uri);
        httpd_register_uri_handler(stream_httpd, &stream_uri);
        httpd_register_uri_handler(stream_httpd, &stream_on_uri);
        httpd_register_uri_handler(stream_httpd, &stream_off_uri);
        httpd_register_uri_handler(stream_httpd, &stream_status_uri);
        httpd_register_uri_handler(stream_httpd, &counts_uri);
    }
    Serial.println("Webserver läuft!");
}

// Nicht benutzt, da alle Arbeiten in Tasks laufen
void loop() { vTaskDelete(NULL); }
