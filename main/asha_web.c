#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "esp_event.h"
#include "esp_check.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_ip_addr.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

#include "asha_web.h"
#include "wifi_credentials.h"

#define LOG_COUNT 64
#define LOG_LEN 112
#define WEB_AUTH_HEADER_MAX 160

static const char *TAG = "ASHA_WEB";
static esp_netif_t *sta_netif;
static portMUX_TYPE state_mux = portMUX_INITIALIZER_UNLOCKED;
static char current_stage[32] = "Başlatılıyor";
static char current_detail[96] = "Firmware açılıyor";
static char current_ble_scan[160] = "Henüz BLE reklamı alınmadı";
static char event_log[LOG_COUNT][LOG_LEN];
static unsigned event_next;
static unsigned event_total;
static bool usb_audio_received;
static asha_web_retry_cb_t retry_callback;

static const char basic_auth_alphabet[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

static esp_err_t encode_basic_credentials(char *output, size_t output_capacity,
                                          size_t *output_len)
{
    const size_t username_len = strlen(ASHA_WEB_USERNAME);
    const size_t password_len = strlen(ASHA_WEB_PASSWORD);
    const size_t input_len = username_len + 1 + password_len;
    const size_t encoded_len = 4 * ((input_len + 2) / 3);
    if (username_len == 0 || password_len < 12 ||
        username_len > 32 || password_len > 64 ||
        encoded_len + 1 > output_capacity) {
        return ESP_ERR_INVALID_SIZE;
    }
    for (size_t i = 0; i < username_len; ++i) {
        unsigned char ch = (unsigned char)ASHA_WEB_USERNAME[i];
        if (ch <= 0x20 || ch >= 0x7f || ch == ':') return ESP_ERR_INVALID_ARG;
    }
    for (size_t i = 0; i < password_len; ++i) {
        unsigned char ch = (unsigned char)ASHA_WEB_PASSWORD[i];
        if (ch < 0x20 || ch >= 0x7f) return ESP_ERR_INVALID_ARG;
    }

    size_t input_offset = 0;
    size_t output_offset = 0;
    while (input_offset < input_len) {
        uint8_t bytes[3] = {0};
        size_t remaining = input_len - input_offset;
        size_t count = remaining < sizeof(bytes) ? remaining : sizeof(bytes);
        for (size_t i = 0; i < count; ++i, ++input_offset) {
            if (input_offset < username_len) {
                bytes[i] = (uint8_t)ASHA_WEB_USERNAME[input_offset];
            } else if (input_offset == username_len) {
                bytes[i] = ':';
            } else {
                bytes[i] = (uint8_t)ASHA_WEB_PASSWORD[input_offset - username_len - 1];
            }
        }
        output[output_offset++] = basic_auth_alphabet[bytes[0] >> 2];
        output[output_offset++] = basic_auth_alphabet[((bytes[0] & 0x03) << 4) |
                                                       (bytes[1] >> 4)];
        output[output_offset++] = count > 1 ?
            basic_auth_alphabet[((bytes[1] & 0x0f) << 2) | (bytes[2] >> 6)] : '=';
        output[output_offset++] = count > 2 ? basic_auth_alphabet[bytes[2] & 0x3f] : '=';
    }
    output[output_offset] = '\0';
    *output_len = output_offset;
    return ESP_OK;
}

static bool authenticate_request(httpd_req_t *req)
{
    char expected[WEB_AUTH_HEADER_MAX];
    size_t expected_len = 0;
    if (encode_basic_credentials(expected, sizeof(expected), &expected_len) != ESP_OK) {
        ESP_LOGE(TAG, "Dashboard authentication credentials are empty or too long");
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_set_hdr(req, "Cache-Control", "no-store");
        httpd_resp_sendstr(req, "Dashboard authentication is not configured");
        return false;
    }

    size_t header_len = httpd_req_get_hdr_value_len(req, "Authorization");
    if (header_len < sizeof("Basic ") - 1 ||
        header_len >= WEB_AUTH_HEADER_MAX ||
        header_len != sizeof("Basic ") - 1 + expected_len) {
        goto unauthorized;
    }

    char authorization[WEB_AUTH_HEADER_MAX];
    if (httpd_req_get_hdr_value_str(req, "Authorization", authorization,
                                   sizeof(authorization)) != ESP_OK ||
        strncasecmp(authorization, "Basic ", sizeof("Basic ") - 1) != 0) {
        goto unauthorized;
    }

    uint8_t difference = 0;
    for (size_t i = 0; i < expected_len; ++i) {
        difference |= (uint8_t)authorization[sizeof("Basic ") - 1 + i] ^
                      (uint8_t)expected[i];
    }
    if (difference == 0) return true;

unauthorized:
    httpd_resp_set_status(req, "401 Unauthorized");
    httpd_resp_set_hdr(req, "WWW-Authenticate", "Basic realm=\"ASHA S3\", charset=\"UTF-8\"");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_sendstr(req, "Authentication required");
    return false;
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        const wifi_event_sta_disconnected_t *event = event_data;
        ESP_LOGW(TAG, "Wi-Fi disconnected (reason=%d); reconnecting", event->reason);
        asha_web_set_status("Wi-Fi bağlantısı kesildi", "Yeniden bağlanılıyor");
        esp_err_t err = esp_wifi_connect();
        if (err != ESP_OK) ESP_LOGE(TAG, "Wi-Fi reconnect request failed: %s", esp_err_to_name(err));
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *event = event_data;
        ESP_LOGI(TAG, "Wi-Fi connected; log page: http://" IPSTR, IP2STR(&event->ip_info.ip));
        char detail[sizeof(current_detail)];
        snprintf(detail, sizeof(detail), "Aynı Wi-Fi ağındaki cihazdan http://" IPSTR " adresini açın",
                 IP2STR(&event->ip_info.ip));
        asha_web_set_status("Wi-Fi bağlandı", detail);
    }
}

static void append_html_escaped(char *out, size_t capacity, size_t *used, const char *input)
{
    if (!out || !used || !input || *used >= capacity) return;
    for (const char *p = input; *p && *used < capacity - 1; ++p) {
        const char *replacement = NULL;
        switch (*p) {
        case '&': replacement = "&amp;"; break;
        case '<': replacement = "&lt;"; break;
        case '>': replacement = "&gt;"; break;
        case '\"': replacement = "&quot;"; break;
        case '\'': replacement = "&#39;"; break;
        default: break;
        }
        if (replacement) {
            size_t n = strlen(replacement);
            if (*used + n >= capacity) break;
            memcpy(out + *used, replacement, n);
            *used += n;
        } else {
            out[(*used)++] = *p;
        }
    }
    out[*used] = '\0';
}

static void record_event_locked(const char *text)
{
    snprintf(event_log[event_next], LOG_LEN, "%u s  %s",
             (unsigned)(esp_log_timestamp() / 1000), text);
    event_next = (event_next + 1) % LOG_COUNT;
    if (event_total < LOG_COUNT) event_total++;
}

void asha_web_set_status(const char *stage, const char *detail)
{
    portENTER_CRITICAL(&state_mux);
    snprintf(current_stage, sizeof(current_stage), "%s", stage ? stage : "?");
    snprintf(current_detail, sizeof(current_detail), "%s", detail ? detail : "");
    char event[LOG_LEN];
    snprintf(event, sizeof(event), "%.31s: %.75s", current_stage, current_detail);
    record_event_locked(event);
    portEXIT_CRITICAL(&state_mux);
    ESP_LOGI(TAG, "%s — %s", stage ? stage : "?", detail ? detail : "");
}

void asha_web_add_event(const char *event)
{
    portENTER_CRITICAL(&state_mux);
    record_event_locked(event ? event : "Olay");
    portEXIT_CRITICAL(&state_mux);
}

void asha_web_set_ble_scan_report(const char *report)
{
    portENTER_CRITICAL(&state_mux);
    snprintf(current_ble_scan, sizeof(current_ble_scan), "%s",
             report ? report : "Henüz BLE reklamı alınmadı");
    portEXIT_CRITICAL(&state_mux);
}

void asha_web_set_usb_audio_received(bool received)
{
    portENTER_CRITICAL(&state_mux);
    usb_audio_received = received;
    portEXIT_CRITICAL(&state_mux);
}

static void json_escape(char *out, size_t capacity, const char *input)
{
    size_t used = 0;
    for (const unsigned char *p = (const unsigned char *)input; *p && used + 1 < capacity; ++p) {
        if (*p == '"' || *p == '\\') {
            if (used + 2 >= capacity) break;
            out[used++] = '\\';
            out[used++] = (char)*p;
        } else if (*p < 0x20) {
            if (used + 6 >= capacity) break;
            int n = snprintf(out + used, capacity - used, "\\u%04x", *p);
            if (n < 0) break;
            used += (size_t)n;
        } else {
            out[used++] = (char)*p;
        }
    }
    out[used] = '\0';
}

typedef struct {
    char stage[sizeof(current_stage)];
    char detail[sizeof(current_detail)];
    char ble_scan[sizeof(current_ble_scan)];
    char (*events)[LOG_LEN];
    unsigned count;
    bool usb_received;
} web_snapshot_t;

static bool take_snapshot(web_snapshot_t *snapshot)
{
    snapshot->events = malloc(LOG_COUNT * LOG_LEN);
    if (!snapshot->events) return false;

    portENTER_CRITICAL(&state_mux);
    snprintf(snapshot->stage, sizeof(snapshot->stage), "%s", current_stage);
    snprintf(snapshot->detail, sizeof(snapshot->detail), "%s", current_detail);
    snprintf(snapshot->ble_scan, sizeof(snapshot->ble_scan), "%s", current_ble_scan);
    snapshot->count = event_total;
    snapshot->usb_received = usb_audio_received;
    unsigned start = (event_next + LOG_COUNT - snapshot->count) % LOG_COUNT;
    for (unsigned i = 0; i < snapshot->count; ++i) {
        snprintf(snapshot->events[i], LOG_LEN, "%s", event_log[(start + i) % LOG_COUNT]);
    }
    portEXIT_CRITICAL(&state_mux);
    return true;
}

static esp_err_t send_html_text(httpd_req_t *req, const char *text)
{
    char escaped[LOG_LEN * 6 + 1];
    size_t used = 0;
    append_html_escaped(escaped, sizeof(escaped), &used, text);
    return httpd_resp_send_chunk(req, escaped, used);
}

static esp_err_t root_handler(httpd_req_t *req)
{
    if (!authenticate_request(req)) return ESP_OK;
    web_snapshot_t snapshot;
    if (!take_snapshot(&snapshot)) return ESP_ERR_NO_MEM;
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t err = httpd_resp_send_chunk(req,
        "<!doctype html><html lang='tr'><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<meta http-equiv='refresh' content='2'><title>ASHA S3 logları</title>"
        "<style>body{font:16px system-ui;max-width:760px;margin:32px auto;padding:0 16px;background:#10151d;color:#e8eef7}"
        ".card{background:#1b2532;border-radius:12px;padding:18px;margin:14px 0}h1{font-size:24px}pre{white-space:pre-wrap;overflow-wrap:anywhere;color:#b7d4ff}small{color:#9eafc4}</style></head><body>"
        "<h1>ASHA S3 cihaz logları</h1><small>Son 64 olay; sayfa 2 saniyede bir yenilenir.</small><div class='card'><b>Durum: ",
        HTTPD_RESP_USE_STRLEN);
    if (err == ESP_OK) err = send_html_text(req, snapshot.stage);
    if (err == ESP_OK) err = httpd_resp_send_chunk(req, "</b><p>", HTTPD_RESP_USE_STRLEN);
    if (err == ESP_OK) err = send_html_text(req, snapshot.detail);
    if (err == ESP_OK) err = httpd_resp_send_chunk(req,
        snapshot.usb_received ? "</p><p>USB ses: PCM paketi alındı</p></div><div class='card'><b>Son BLE reklamı</b><p>"
                              : "</p><p>USB ses: PCM paketi bekleniyor</p></div><div class='card'><b>Son BLE reklamı</b><p>",
        HTTPD_RESP_USE_STRLEN);
    if (err == ESP_OK) err = send_html_text(req, snapshot.ble_scan);
    if (err == ESP_OK) err = httpd_resp_send_chunk(req, "</p></div><div class='card'><b>Olaylar</b><pre>", HTTPD_RESP_USE_STRLEN);
    for (unsigned i = 0; err == ESP_OK && i < snapshot.count; ++i) {
        if (i) err = httpd_resp_send_chunk(req, "\n", 1);
        if (err == ESP_OK) err = send_html_text(req, snapshot.events[i]);
    }
    if (err == ESP_OK) err = httpd_resp_send_chunk(req, "</pre></div></body></html>", HTTPD_RESP_USE_STRLEN);
    if (err == ESP_OK) err = httpd_resp_send_chunk(req, NULL, 0);
    free(snapshot.events);
    return err;
}

static esp_err_t status_handler(httpd_req_t *req)
{
    if (!authenticate_request(req)) return ESP_OK;
    web_snapshot_t snapshot;
    if (!take_snapshot(&snapshot)) return ESP_ERR_NO_MEM;
    httpd_resp_set_type(req, "application/json; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t err = httpd_resp_send_chunk(req, "{\"stage\":\"", HTTPD_RESP_USE_STRLEN);
    char escaped[LOG_LEN * 6 + 1];
    if (err == ESP_OK) {
        json_escape(escaped, sizeof(escaped), snapshot.stage);
        err = httpd_resp_send_chunk(req, escaped, HTTPD_RESP_USE_STRLEN);
    }
    if (err == ESP_OK) err = httpd_resp_send_chunk(req, "\",\"detail\":\"", HTTPD_RESP_USE_STRLEN);
    if (err == ESP_OK) {
        json_escape(escaped, sizeof(escaped), snapshot.detail);
        err = httpd_resp_send_chunk(req, escaped, HTTPD_RESP_USE_STRLEN);
    }
    if (err == ESP_OK) err = httpd_resp_send_chunk(req, "\",\"ble_scan\":\"", HTTPD_RESP_USE_STRLEN);
    if (err == ESP_OK) {
        json_escape(escaped, sizeof(escaped), snapshot.ble_scan);
        err = httpd_resp_send_chunk(req, escaped, HTTPD_RESP_USE_STRLEN);
    }
    if (err == ESP_OK) err = httpd_resp_send_chunk(req,
        snapshot.usb_received ? "\",\"usb_audio_received\":true,\"events\":[" : "\",\"usb_audio_received\":false,\"events\":[",
        HTTPD_RESP_USE_STRLEN);
    for (unsigned i = 0; err == ESP_OK && i < snapshot.count; ++i) {
        if (i) err = httpd_resp_send_chunk(req, ",", 1);
        if (err == ESP_OK) err = httpd_resp_send_chunk(req, "\"", 1);
        if (err == ESP_OK) {
            json_escape(escaped, sizeof(escaped), snapshot.events[i]);
            err = httpd_resp_send_chunk(req, escaped, HTTPD_RESP_USE_STRLEN);
        }
        if (err == ESP_OK) err = httpd_resp_send_chunk(req, "\"", 1);
    }
    if (err == ESP_OK) err = httpd_resp_send_chunk(req, "]}", 2);
    if (err == ESP_OK) err = httpd_resp_send_chunk(req, NULL, 0);
    free(snapshot.events);
    return err;
}

static esp_err_t retry_handler(httpd_req_t *req)
{
    if (!authenticate_request(req)) return ESP_OK;
    int rc = retry_callback ? retry_callback() : ESP_ERR_INVALID_STATE;
    char body[128];
    snprintf(body, sizeof(body), "{\"ok\":%s,\"message\":\"%s\"}", rc == 0 ? "true" : "false",
             rc == 0 ? "Yeniden deneme istendi." : "Bağlantı yenileme isteği uygulanamadı.");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
}

esp_err_t asha_web_start(asha_web_retry_cb_t retry_cb)
{
    retry_callback = retry_cb;
    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "netif init failed");
    esp_err_t err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;
    sta_netif = esp_netif_create_default_wifi_sta();
    if (!sta_netif) return ESP_FAIL;

    const size_t ssid_len = strlen(ASHA_WIFI_SSID);
    const size_t password_len = strlen(ASHA_WIFI_PASSWORD);
    if (ssid_len == 0 || ssid_len > 32 || password_len > 64 ||
        strcmp(ASHA_WIFI_SSID, "YOUR_WIFI_NAME") == 0 ||
        strcmp(ASHA_WIFI_PASSWORD, "YOUR_WIFI_PASSWORD") == 0) {
        ESP_LOGE(TAG, "Configure valid Wi-Fi credentials in the ignored main/wifi_credentials.h");
        return ESP_ERR_INVALID_ARG;
    }
    char encoded_credentials[WEB_AUTH_HEADER_MAX];
    size_t encoded_credentials_len = 0;
    if (strcmp(ASHA_WEB_PASSWORD, "CHANGE_ME_TO_A_UNIQUE_PASSWORD") == 0 ||
        encode_basic_credentials(encoded_credentials, sizeof(encoded_credentials),
                                 &encoded_credentials_len) != ESP_OK) {
        ESP_LOGE(TAG, "Configure a dashboard username and a 12-64 character password in the ignored main/wifi_credentials.h");
        return ESP_ERR_INVALID_ARG;
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&cfg), TAG, "Wi-Fi init failed");
    ESP_RETURN_ON_ERROR(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                   wifi_event_handler, NULL), TAG, "Wi-Fi event handler registration failed");
    ESP_RETURN_ON_ERROR(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                   wifi_event_handler, NULL), TAG, "IP event handler registration failed");
    wifi_config_t sta = {0};
    memcpy(sta.sta.ssid, ASHA_WIFI_SSID, ssid_len);
    memcpy(sta.sta.password, ASHA_WIFI_PASSWORD, password_len);
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "set STA mode failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &sta), TAG, "set STA config failed");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "Wi-Fi start failed");
    asha_web_set_status("Wi-Fi ağı aranıyor", "Bağlantı kurulunca seri logda IP adresi görünecek");
    ESP_RETURN_ON_ERROR(esp_wifi_connect(), TAG, "Wi-Fi connection request failed");

    httpd_config_t http_cfg = HTTPD_DEFAULT_CONFIG();
    http_cfg.max_uri_handlers = 4;
    httpd_handle_t server = NULL;
    ESP_RETURN_ON_ERROR(httpd_start(&server, &http_cfg), TAG, "HTTP server start failed");
    const httpd_uri_t root = {.uri = "/", .method = HTTP_GET, .handler = root_handler};
    const httpd_uri_t status = {.uri = "/api/status", .method = HTTP_GET, .handler = status_handler};
    const httpd_uri_t retry = {.uri = "/api/retry", .method = HTTP_POST, .handler = retry_handler};
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &root), TAG, "root handler failed");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &status), TAG, "status handler failed");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &retry), TAG, "retry handler failed");
    asha_web_set_status("Log sunucusu hazır", "Wi-Fi bağlantısı ve IP adresi bekleniyor");
    return ESP_OK;
}
