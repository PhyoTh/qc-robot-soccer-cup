// ============================================================
// HiwonderCamStream.ino
// MJPEG streaming firmware for Hiwonder ESP32-S3 Vision Module
//
// Arduino IDE board settings:
//   Board:              ESP32S3 Dev Module
//   PSRAM:              OPI PSRAM
//   Flash Size:         8MB (64Mb)
//   Partition Scheme:   Huge APP (3MB No OTA/1MB SPIFFS)
//   USB CDC On Boot:    Disabled
//   Flash Mode:         DIO
//   Upload Speed:       921600
// ============================================================

#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"
#include "img_converters.h"   // for frame2jpg()

// ------------------------------------------------------------
// *** CHANGE THIS FOR EACH ROBOT ***
// ------------------------------------------------------------
#define CAMERA_SSID  "miniAuto_CAM_01"
#define CAMERA_PASS  "hiwonder123"
// ------------------------------------------------------------

#define AP_IP        "192.168.5.1"
#define AP_GATEWAY   "192.168.5.1"
#define AP_SUBNET    "255.255.255.0"
#define STREAM_PORT  81

// ------------------------------------------------------------
// Camera pins — Hiwonder ESP32S3-CAM V1.0 (CAMERA_MODEL_ESP32S3_EYE)
// ------------------------------------------------------------
#define PWDN_GPIO_NUM   -1
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM   15
#define SIOD_GPIO_NUM    4
#define SIOC_GPIO_NUM    5

#define Y2_GPIO_NUM     11
#define Y3_GPIO_NUM      9
#define Y4_GPIO_NUM      8
#define Y5_GPIO_NUM     10
#define Y6_GPIO_NUM     12
#define Y7_GPIO_NUM     18
#define Y8_GPIO_NUM     17
#define Y9_GPIO_NUM     16

#define VSYNC_GPIO_NUM   6
#define HREF_GPIO_NUM    7
#define PCLK_GPIO_NUM   13

#define XCLK_FREQ_HZ 15000000  // Hiwonder's value

// ------------------------------------------------------------
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* STREAM_CONTENT_TYPE =
    "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* STREAM_PART =
    "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t stream_httpd = NULL;
bool camera_ok = false;

static esp_err_t stream_handler(httpd_req_t *req) {
    if (!camera_ok) {
        const char* msg = "Camera init failed";
        httpd_resp_send(req, msg, strlen(msg));
        return ESP_OK;
    }

    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;
    char part_buf[64];

    res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
    if (res != ESP_OK) return res;
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    while (true) {
        fb = esp_camera_fb_get();
        if (!fb) { res = ESP_FAIL; break; }

        // GC2145 outputs RGB565 — convert to JPEG for streaming
        uint8_t *jpg_buf = NULL;
        size_t   jpg_len = 0;
        bool converted = frame2jpg(fb, 80, &jpg_buf, &jpg_len);
        esp_camera_fb_return(fb);
        fb = NULL;

        if (!converted) { res = ESP_FAIL; break; }

        // Send boundary
        res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
        if (res != ESP_OK) { free(jpg_buf); break; }

        // Send part header
        size_t hlen = snprintf(part_buf, sizeof(part_buf), STREAM_PART, jpg_len);
        res = httpd_resp_send_chunk(req, part_buf, hlen);
        if (res != ESP_OK) { free(jpg_buf); break; }

        // Send JPEG data
        res = httpd_resp_send_chunk(req, (const char *)jpg_buf, jpg_len);
        free(jpg_buf);
        if (res != ESP_OK) break;
    }
    return res;
}

static esp_err_t index_handler(httpd_req_t *req) {
    char html[1024];
    snprintf(html, sizeof(html),
        "<html><head><title>%s</title>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<style>body{background:#111;color:#eee;font-family:sans-serif;"
        "display:flex;flex-direction:column;align-items:center;padding:20px;}"
        "img{max-width:100%%;border:2px solid #444;}"
        "button{margin:12px;padding:10px 24px;font-size:16px;cursor:pointer;"
        "background:#2a7;color:#fff;border:none;border-radius:6px;}"
        ".err{color:#f66;font-size:14px;}"
        "</style></head><body>"
        "<h2>&#127909; %s</h2>"
        "%s"
        "<img id='s' src='' /><br>"
        "<button onclick=\"document.getElementById('s').src="
        "'http://192.168.5.1:81/stream'\">&#9654; Start Stream</button>"
        "<button onclick=\"document.getElementById('s').src=''\">&#9646;&#9646; Stop</button>"
        "</body></html>",
        CAMERA_SSID, CAMERA_SSID,
        camera_ok ? "" : "<p class='err'>&#9888; Camera init failed</p>"
    );
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, html, strlen(html));
}

void startStreamServer() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = STREAM_PORT;
    config.ctrl_port   = STREAM_PORT + 1000;
    httpd_uri_t index_uri  = { .uri="/",       .method=HTTP_GET, .handler=index_handler,  .user_ctx=NULL };
    httpd_uri_t stream_uri = { .uri="/stream", .method=HTTP_GET, .handler=stream_handler, .user_ctx=NULL };
    if (httpd_start(&stream_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(stream_httpd, &index_uri);
        httpd_register_uri_handler(stream_httpd, &stream_uri);
    }
}

void setup() {
    // Start WiFi AP first
    IPAddress local_ip, gateway, subnet;
    local_ip.fromString(AP_IP);
    gateway.fromString(AP_GATEWAY);
    subnet.fromString(AP_SUBNET);
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(local_ip, gateway, subnet);
    WiFi.softAP(CAMERA_SSID, CAMERA_PASS);
    startStreamServer();

    // Init camera — GC2145 uses RGB565, not JPEG
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer   = LEDC_TIMER_0;
    config.pin_d0       = Y2_GPIO_NUM;
    config.pin_d1       = Y3_GPIO_NUM;
    config.pin_d2       = Y4_GPIO_NUM;
    config.pin_d3       = Y5_GPIO_NUM;
    config.pin_d4       = Y6_GPIO_NUM;
    config.pin_d5       = Y7_GPIO_NUM;
    config.pin_d6       = Y8_GPIO_NUM;
    config.pin_d7       = Y9_GPIO_NUM;
    config.pin_xclk     = XCLK_GPIO_NUM;
    config.pin_pclk     = PCLK_GPIO_NUM;
    config.pin_vsync    = VSYNC_GPIO_NUM;
    config.pin_href     = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn     = PWDN_GPIO_NUM;
    config.pin_reset    = RESET_GPIO_NUM;
    config.xclk_freq_hz = XCLK_FREQ_HZ;
    config.pixel_format = PIXFORMAT_RGB565;  // GC2145 native format
    config.frame_size   = FRAMESIZE_QVGA;    // 320x240
    config.fb_count     = 2;
    config.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
    config.fb_location  = CAMERA_FB_IN_PSRAM;

    camera_ok = (esp_camera_init(&config) == ESP_OK);

    if (camera_ok) {
        sensor_t *s = esp_camera_sensor_get();
        if (s) {
            s->set_vflip(s, 0);
            s->set_hmirror(s, 0);
        }
    }
}

void loop() {
    delay(10000);
}
