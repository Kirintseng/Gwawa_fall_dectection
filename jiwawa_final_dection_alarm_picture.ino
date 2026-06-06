#include <gwawa3.0_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "esp_camera.h"

// ========== Wi-Fi 與 HTTP 函式庫 ==========
#include <WiFi.h>
#include <HTTPClient.h>
// ===============================================

// Select camera model
#define CAMERA_MODEL_ESP_EYE // Has PSRAM

#if defined(CAMERA_MODEL_ESP_EYE)
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     10
#define SIOD_GPIO_NUM     40
#define SIOC_GPIO_NUM     39

#define Y9_GPIO_NUM       48
#define Y8_GPIO_NUM       11
#define Y7_GPIO_NUM       12
#define Y6_GPIO_NUM       14
#define Y5_GPIO_NUM       16
#define Y4_GPIO_NUM       18
#define Y3_GPIO_NUM       17
#define Y2_GPIO_NUM       15
#define VSYNC_GPIO_NUM    38
#define HREF_GPIO_NUM     47
#define PCLK_GPIO_NUM     13
#else
#error "Camera model not selected"
#endif

/* Constant defines -------------------------------------------------------- */
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS           320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS           240
#define EI_CAMERA_FRAME_BYTE_SIZE                 3

/* Private variables ------------------------------------------------------- */
static bool debug_nn = false; 
static bool is_initialised = false;
uint8_t *snapshot_buf; 

static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,

    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,

    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG, 
    .frame_size = FRAMESIZE_QVGA,    

    .jpeg_quality = 12, 
    .fb_count = 1,       
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

/* Function definitions ------------------------------------------------------- */
bool ei_camera_init(void);
void ei_camera_deinit(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) ;

// ========== 網路與 Discord Webhook 設定變數 ==========
const char* ssid = "PELAB";       
const char* password = "pelab8018";   
const char* discordWebhookUrl = "https://discord.com/api/webhooks/1511005649365110976/wg3pW-APCIVsKgQH9bNizecq9nqIzrl0QOliVckBMHMC8UoTLWj_nytah4SuUYvA16nB"; 

unsigned long lastNotificationTime = 0;
const unsigned long notificationCooldown = 10000; // 觸發冷卻時間：10秒

// ========== 【全新：時間濾波 (防彈跳) 控制變數】 ==========
int consecutive_fall_count = 0;         // 目前連續偵測到跌倒的影格數
const int REQUIRED_FALL_FRAMES = 3;     // 門檻：必須「連續 3 幀」判定跌倒才算數 (可根據靈敏度微調，例如改為 2 或 4)
const float FALL_CONFIDENCE_THRESHOLD = 0.80; // 門檻：單幀信心度大於 80% 才採信
// ========================================================

// 發送 Discord 現場照片
void sendDiscordPhoto(String message) {
    if (WiFi.status() == WL_CONNECTED) {
        
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            Serial.println("相機擷取失敗，無法發送照片！");
            return;
        }

        HTTPClient http;
        http.begin(discordWebhookUrl);
        
        String boundary = "----Esp32Boundary" + String(millis());
        http.addHeader("Content-Type", "multipart/form-data; boundary=" + boundary);

        String head = "--" + boundary + "\r\n";
        head += "Content-Disposition: form-data; name=\"payload_json\"\r\n\r\n";
        head += "{\"content\":\"" + message + "\"}\r\n";
        head += "--" + boundary + "\r\n";
        head += "Content-Disposition: form-data; name=\"file\"; filename=\"fall_scene.jpg\"\r\n";
        head += "Content-Type: image/jpeg\r\n\r\n";

        String tail = "\r\n--" + boundary + "--\r\n";

        size_t totalLen = head.length() + fb->len + tail.length();
        uint8_t *body = (uint8_t *)ps_malloc(totalLen);
        if (body == NULL) {
            body = (uint8_t *)malloc(totalLen); 
        }

        if (body != NULL) {
            memcpy(body, head.c_str(), head.length());
            memcpy(body + head.length(), fb->buf, fb->len);
            memcpy(body + head.length() + fb->len, tail.c_str(), tail.length());

            int httpResponseCode = http.POST(body, totalLen);
            
            if (httpResponseCode == 200 || httpResponseCode == 204) {
                Serial.println("======> 📸 Discord 現場照片推播成功！");
            } else {
                Serial.printf("Discord 推播失敗，錯誤碼: %d\n", httpResponseCode);
            }
            free(body); 
        } else {
            Serial.println("記憶體不足，無法組合照片封包！");
        }
        
        http.end();
        esp_camera_fb_return(fb); 
    } else {
        Serial.println("Wi-Fi 未連線，無法發送 Discord 通知！");
    }
}

/**
* @brief      Arduino setup function
*/
void setup()
{
    Serial.begin(115200);
    while (!Serial);
    Serial.println("Edge Impulse Inferencing Demo with Discord Photo & Temporal Filtering");

    WiFi.begin(ssid, password);
    Serial.print("正在連線至 Wi-Fi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWi-Fi 連線成功！");
    Serial.print("IP 位址: ");
    Serial.println(WiFi.localIP());

    if (ei_camera_init() == false) {
        ei_printf("Failed to initialize Camera!\r\n");
    }
    else {
        ei_printf("Camera initialized\r\n");
    }

    ei_printf("\nStarting continious inference in 2 seconds...\n");
    ei_sleep(2000);
}

/**
* @brief      Get data and run inferencing
*/
void loop()
{
    if (ei_sleep(5) != EI_IMPULSE_OK) {
        return;
    }

    snapshot_buf = (uint8_t*)malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE);

    if(snapshot_buf == nullptr) {
        ei_printf("ERR: Failed to allocate snapshot buffer!\n");
        return;
    }

    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_get_data;

    if (ei_camera_capture((size_t)EI_CLASSIFIER_INPUT_WIDTH, (size_t)EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf) == false) {
        ei_printf("Failed to capture image\r\n");
        free(snapshot_buf);
        return;
    }

    ei_impulse_result_t result = { 0 };

    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", err);
        free(snapshot_buf); 
        return;
    }

    ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
                result.timing.dsp, result.timing.classification, result.timing.anomaly);

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
    ei_printf("Object detection bounding boxes:\r\n");
    
    // 預設這一個 Frame 沒有偵測到跌倒
    bool fall_detected_this_frame = false;
    float max_fall_confidence = 0.0;

    for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
        ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
        if (bb.value == 0) {
            continue;
        }
        ei_printf("  %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\r\n",
                bb.label, bb.value, bb.x, bb.y, bb.width, bb.height);
                
        // 檢查是否有信心度高於門檻的 fall 標籤
        if (strcmp(bb.label, "fall") == 0 && bb.value >= FALL_CONFIDENCE_THRESHOLD) { 
            fall_detected_this_frame = true;
            if (bb.value > max_fall_confidence) {
                max_fall_confidence = bb.value; // 記錄最高信心度，方便推播顯示
            }
        }
    }

    // ========== 核心：時間濾波 (防彈跳) 邏輯 ==========
    if (fall_detected_this_frame) {
        consecutive_fall_count++; // 連續計數 +1
        Serial.printf("!!! 偵測到跌倒特徵 !!! (目前連續幀數: %d / 門檻: %d)\n", consecutive_fall_count, REQUIRED_FALL_FRAMES);

        // 只有連續達到標準，且過了冷卻時間才發送 Discord
        if (consecutive_fall_count >= REQUIRED_FALL_FRAMES) {
            if (millis() - lastNotificationTime > notificationCooldown) {
                sendDiscordPhoto("🚨 **【緊急警告】** 系統確認目標已跌倒！請立即查看案發現場照片！(最高信心度: " + String(max_fall_confidence, 2) + ")");
                lastNotificationTime = millis();
                // 發送後將計數器歸零，避免在冷卻時間過後瞬間又觸發
                consecutive_fall_count = 0; 
            }
        }
    } else {
        // 如果這個 Frame 完全沒有抓到跌倒，計數器立刻無情歸零！
        if (consecutive_fall_count > 0) {
            Serial.println("--- 跌倒特徵中斷，計數器歸零 ---");
            consecutive_fall_count = 0;
        }
    }
    // ===============================================

#else
    // (如果您的模型是純影像分類，這裡目前暫不加入防彈跳，因為您使用的是物件偵測)
    ei_printf("Predictions:\r\n");
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        ei_printf("  %s: ", ei_classifier_inferencing_categories[i]);
        ei_printf("%.5f\r\n", result.classification[i].value);
    }
#endif

    free(snapshot_buf);
}

/**
 * @brief   Setup image sensor & start streaming
 */
bool ei_camera_init(void) {
    if (is_initialised) return true;

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
      Serial.printf("Camera init failed with error 0x%x\n", err);
      return false;
    }

    sensor_t * s = esp_camera_sensor_get();
    if (s->id.PID == OV3660_PID) {
      s->set_vflip(s, 1); 
      s->set_brightness(s, 1); 
      s->set_saturation(s, 0); 
    }

#if defined(CAMERA_MODEL_M5STACK_WIDE)
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
#elif defined(CAMERA_MODEL_ESP_EYE)
    // 保持修正狀態：關閉上下顛倒與左右鏡像，照片恢復正常視角
    s->set_vflip(s, 0);
    s->set_hmirror(s, 0);
    s->set_awb_gain(s, 1);
#endif

    is_initialised = true;
    return true;
}

void ei_camera_deinit(void) {
    esp_err_t err = esp_camera_deinit();
    if (err != ESP_OK)
    {
        ei_printf("Camera deinit failed\n");
        return;
    }
    is_initialised = false;
    return;
}

bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
    bool do_resize = false;
    if (!is_initialised) {
        ei_printf("ERR: Camera is not initialized\r\n");
        return false;
    }

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        ei_printf("Camera capture failed\n");
        return false;
    }

   bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);
   esp_camera_fb_return(fb);

   if(!converted){
       ei_printf("Conversion failed\n");
       return false;
   }

    if ((img_width != EI_CAMERA_RAW_FRAME_BUFFER_COLS)
        || (img_height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS)) {
        do_resize = true;
    }

    if (do_resize) {
        ei::image::processing::crop_and_interpolate_rgb888(
        out_buf,
        EI_CAMERA_RAW_FRAME_BUFFER_COLS,
        EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
        out_buf,
        img_width,
        img_height);
    }
    return true;
}

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr)
{
    size_t pixel_ix = offset * 3;
    size_t pixels_left = length;
    size_t out_ptr_ix = 0;
    while (pixels_left != 0) {
        out_ptr[out_ptr_ix] = (snapshot_buf[pixel_ix + 2] << 16) + (snapshot_buf[pixel_ix + 1] << 8) + snapshot_buf[pixel_ix];
        out_ptr_ix++;
        pixel_ix+=3;
        pixels_left--;
    }
    return 0;
}
