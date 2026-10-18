#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_log_buffer.h"
#include "nvs_flash.h"
#include "nimble/ble.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_esp_gap.h"
#include "host/ble_sm.h"
#include "host/ble_hs_adv.h"
#include "host/ble_gatt.h"
#include "host/ble_store.h"
#include "host/ble_l2cap.h"
#include "host/util/util.h"
#include "os/os_mbuf.h"
#include "os/os_mempool.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "services/gap/ble_svc_gap.h"
#include "usb_device_uac.h"
#include "asha_web.h"
#include "asha_protocol.h"

static const char *tag = "ASHA_S3_USB";
#define USB_PCM_SAMPLES_PER_FRAME 320
#define USB_PCM_QUEUE_FRAMES 3
#define USB_RESAMPLER_TAPS 63

static QueueHandle_t usb_pcm_queue;
static bool usb_audio_first_packet_logged;
static int16_t usb_resampler_history[USB_RESAMPLER_TAPS];
static unsigned usb_resampler_head;
static unsigned usb_downsample_phase;
static int16_t usb_pcm_frame[USB_PCM_SAMPLES_PER_FRAME];
static size_t usb_pcm_frame_samples;
static unsigned usb_pcm_dropped_frames;
static unsigned usb_pcm_underflow_frames;
static int16_t usb_pcm_discard_frame[USB_PCM_SAMPLES_PER_FRAME];

static const int16_t usb_resampler_coeffs[USB_RESAMPLER_TAPS] = {
    -4, 20, 32, 19, -17, -53, -54, 0, 80, 118, 56, -88, -207,
    -174, 38, 291, 361, 117, -320, -607, -428, 211, 878, 966,
    173, -1126, -1946, -1253, 1301, 4983, 8244, 9546, 8244, 4983,
    1301, -1253, -1946, -1126, 173, 966, 878, 211, -428, -607,
    -320, 117, 361, 291, 38, -174, -207, -88, 56, 118, 80, 0,
    -54, -53, -17, 19, 32, 20, -4
};

static bool resample_usb_mono_sample(int16_t sample, int16_t *output)
{
    usb_resampler_history[usb_resampler_head] = sample;
    if (++usb_resampler_head == USB_RESAMPLER_TAPS) usb_resampler_head = 0;
    if (++usb_downsample_phase != 3) return false;
    usb_downsample_phase = 0;

    int64_t accumulator = 0;
    unsigned history_index = usb_resampler_head;
    for (unsigned tap = 0; tap < USB_RESAMPLER_TAPS; ++tap) {
        history_index = history_index == 0 ? USB_RESAMPLER_TAPS - 1 : history_index - 1;
        accumulator += (int32_t)usb_resampler_history[history_index] *
                       usb_resampler_coeffs[tap];
    }
    accumulator += accumulator >= 0 ? 16384 : -16384;
    int64_t filtered = accumulator / 32768;
    if (filtered > INT16_MAX) filtered = INT16_MAX;
    if (filtered < INT16_MIN) filtered = INT16_MIN;
    *output = (int16_t)filtered;
    return true;
}

static void enqueue_usb_pcm_frame(const int16_t *frame)
{
    if (xQueueSend(usb_pcm_queue, frame, 0) == pdTRUE) return;

    if (xQueueReceive(usb_pcm_queue, usb_pcm_discard_frame, 0) == pdTRUE) {
        ++usb_pcm_dropped_frames;
    }
    if (xQueueSend(usb_pcm_queue, frame, 0) != pdTRUE) {
        ++usb_pcm_dropped_frames;
    }
    if ((usb_pcm_dropped_frames % 50) == 1) {
        ESP_LOGW(tag, "USB PCM queue overrun; discarded stale frames to limit latency (%u total)",
                 usb_pcm_dropped_frames);
    }
}

static esp_err_t usb_audio_output_cb(uint8_t *buf, size_t len, void *cb_ctx)
{
    (void)cb_ctx;
    if (!buf || !usb_pcm_queue) return ESP_ERR_INVALID_STATE;
    if (len && !usb_audio_first_packet_logged) {
        usb_audio_first_packet_logged = true;
        ESP_LOGI(tag, "USB UAC speaker PCM received: %u bytes; filtering/downsampling 48 kHz stereo to 16 kHz mono",
                 (unsigned)len);
        asha_web_set_usb_audio_received(true);
        asha_web_set_status("USB ses alınıyor", "Bilgisayardan ilk USB PCM paketi geldi");
    }

    /* UAC format: 48 kHz, stereo, signed 16-bit little-endian PCM. */
    for (size_t i = 0; i < len / 4; ++i) {
        int16_t left, right;
        memcpy(&left, buf + i * 4, sizeof(left));
        memcpy(&right, buf + i * 4 + 2, sizeof(right));
        int16_t mono = (int16_t)(((int32_t)left + right) / 2);
        int16_t resampled;
        if (resample_usb_mono_sample(mono, &resampled)) {
            usb_pcm_frame[usb_pcm_frame_samples++] = resampled;
            if (usb_pcm_frame_samples == USB_PCM_SAMPLES_PER_FRAME) {
                enqueue_usb_pcm_frame(usb_pcm_frame);
                usb_pcm_frame_samples = 0;
            }
        }
    }
    return ESP_OK;
}

static int blecent_gap_event(struct ble_gap_event *event, void *arg);
static void start_pairing(uint16_t conn_handle);
static void start_characteristics_after_security(uint16_t conn_handle);
static ble_uuid_any_t mfi_uuid;
static ble_uuid_any_t asha_uuid;
static ble_uuid_any_t psm_uuid;
static ble_uuid_any_t readonly_props_uuid;
static ble_uuid_any_t audio_control_uuid;
static ble_uuid_any_t audio_status_uuid;
static ble_uuid_any_t volume_uuid;
static bool connect_started;
static ble_addr_t bonded_peer_addrs[CONFIG_BT_NIMBLE_MAX_BONDS];
static unsigned bonded_peer_count;
static bool gatt_discovery_started;
static uint16_t asha_start_handle;
static uint16_t asha_end_handle;
static uint16_t psm_value_handle;
static uint16_t readonly_props_value_handle;
static uint16_t audio_control_value_handle;
static uint16_t audio_status_value_handle;
static uint16_t audio_status_cccd_handle;
static uint16_t volume_value_handle;
static bool audio_status_notify_supported;
static uint8_t psm_properties;
static uint8_t readonly_props_properties;
static uint8_t audio_control_properties;
static struct ble_l2cap_chan *asha_coc_chan;
static TaskHandle_t asha_audio_task_handle;
static volatile uint16_t asha_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static bool asha_conn_update_started;
static volatile bool asha_watchdog_enabled;
static volatile bool asha_failure_pending;
static volatile TickType_t asha_progress_tick;
static volatile enum { ASHA_CMD_IDLE, ASHA_CMD_WAIT_START_STATUS, ASHA_CMD_STREAMING,
                       ASHA_CMD_WAIT_STOP_STATUS } asha_cmd_state;

#define ASHA_AUDIO_FRAME_BYTES 160
#define ASHA_AUDIO_SDU_BYTES (ASHA_AUDIO_FRAME_BYTES + 1)
#define ASHA_PCM_SAMPLES_PER_FRAME 320
#define ASHA_AUDIO_FRAME_PERIOD_MS 20

void asha_test_encoder_reset(void);
int asha_test_encode_frame(uint8_t sequence, uint8_t packet[ASHA_AUDIO_SDU_BYTES]);
int asha_encode_pcm_frame(uint8_t sequence, const int16_t pcm[ASHA_PCM_SAMPLES_PER_FRAME],
                          uint8_t packet[ASHA_AUDIO_SDU_BYTES]);

#define ASHA_COC_MTU 512
#define ASHA_COC_BUF_COUNT 3
static os_membuf_t asha_coc_mem[OS_MEMPOOL_SIZE(ASHA_COC_BUF_COUNT, ASHA_COC_MTU * 2)];
static struct os_mempool asha_coc_mempool;
static struct os_mbuf_pool asha_coc_mbuf_pool;
static bool asha_coc_connect_started;
static int readonly_props_read_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                                struct ble_gatt_attr *attr, void *arg);
static int asha_coc_event_cb(struct ble_l2cap_event *event, void *arg);
static int start_asha_coc(uint16_t conn_handle, uint16_t psm);
static int asha_status_dsc_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                              uint16_t chr_val_handle, const struct ble_gatt_dsc *dsc,
                              void *arg);
static int asha_status_subscribe_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                                  struct ble_gatt_attr *attr, void *arg);
static int start_psm_read(uint16_t conn_handle);
static int psm_read_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                       struct ble_gatt_attr *attr, void *arg);
static int web_retry_asha(void);
static void reset_asha_connection_state(void);
static void fail_asha_connection(const char *stage, const char *detail);
static void mark_asha_progress(void);
static bool start_asha_watchdog(uint16_t conn_handle);
static void blecent_scan(void);
void ble_store_config_init(void);

#define ASHA_SETUP_TIMEOUT_MS 30000
#define ASHA_TERMINATE_REASON 0x13

static const char *ble_status_description(int status)
{
    switch (status) {
    case 0:
        return "success";
    case BLE_HS_HCI_ERR(BLE_ERR_AUTH_FAIL):
        return "HCI authentication failure";
    case BLE_HS_HCI_ERR(BLE_ERR_PINKEY_MISSING):
        return "HCI PIN or encryption key missing";
    case BLE_HS_HCI_ERR(BLE_ERR_CONN_SPVN_TMO):
        return "HCI connection supervision timeout";
    case BLE_HS_HCI_ERR(BLE_ERR_REM_USER_CONN_TERM):
        return "peer terminated the connection";
    default:
        return "other BLE/HCI error";
    }
}

static void mark_asha_progress(void)
{
    asha_progress_tick = xTaskGetTickCount();
}

static void reset_asha_connection_state(void)
{
    gatt_discovery_started = false;
    asha_start_handle = 0;
    asha_end_handle = 0;
    psm_value_handle = 0;
    psm_properties = 0;
    readonly_props_value_handle = 0;
    readonly_props_properties = 0;
    audio_control_value_handle = 0;
    audio_control_properties = 0;
    audio_status_value_handle = 0;
    audio_status_cccd_handle = 0;
    volume_value_handle = 0;
    audio_status_notify_supported = false;
    asha_coc_connect_started = false;
    asha_coc_chan = NULL;
    asha_audio_task_handle = NULL;
    asha_conn_update_started = false;
    asha_failure_pending = false;
    asha_cmd_state = ASHA_CMD_IDLE;
    mark_asha_progress();
}

static void fail_asha_connection(const char *stage, const char *detail)
{
    if (asha_failure_pending) return;
    asha_failure_pending = true;
    asha_watchdog_enabled = false;
    asha_cmd_state = ASHA_CMD_IDLE;
    asha_audio_task_handle = NULL;
    ESP_LOGE(tag, "%s: %s", stage, detail);
    asha_web_set_status(stage, detail);
    uint16_t conn_handle = asha_conn_handle;
    if (conn_handle == BLE_HS_CONN_HANDLE_NONE) return;

    int rc = ble_gap_terminate(conn_handle, ASHA_TERMINATE_REASON);
    if (rc != 0) {
        struct ble_gap_conn_desc desc;
        if (ble_gap_conn_find(conn_handle, &desc) != 0) {
            ESP_LOGW(tag, "Failed connection already gone; resuming BLE scan");
            connect_started = false;
            asha_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            asha_watchdog_enabled = false;
            reset_asha_connection_state();
            blecent_scan();
        } else {
            ESP_LOGE(tag, "Could not terminate failed ASHA connection: rc=%d", rc);
        }
    }
}

static void asha_setup_watchdog_task(void *arg)
{
    uint16_t conn_handle = (uint16_t)(uintptr_t)arg;
    const TickType_t timeout = pdMS_TO_TICKS(ASHA_SETUP_TIMEOUT_MS);
    while (asha_conn_handle == conn_handle && asha_watchdog_enabled) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (asha_conn_handle != conn_handle || !asha_watchdog_enabled) break;
        if ((TickType_t)(xTaskGetTickCount() - asha_progress_tick) >= timeout) {
            fail_asha_connection("ASHA zaman aşımı",
                                 "BLE kurulum/handshake adımı 30 saniyede ilerlemedi; yeniden bağlanılıyor");
            break;
        }
    }
    vTaskDelete(NULL);
}

static bool start_asha_watchdog(uint16_t conn_handle)
{
    asha_watchdog_enabled = true;
    mark_asha_progress();
    if (xTaskCreate(asha_setup_watchdog_task, "asha_watchdog", 3072,
                    (void *)(uintptr_t)conn_handle, 4, NULL) == pdPASS) {
        return true;
    }
    asha_watchdog_enabled = false;
    return false;
}

static bool is_mfi_candidate(const struct ble_hs_adv_fields *fields)
{
    for (int i = 0; i < fields->num_uuids128; i++) {
        if (ble_uuid_cmp(&fields->uuids128[i].u, &mfi_uuid.u) == 0) return true;
    }
    return false;
}

static uint8_t identity_address_type(uint8_t address_type)
{
    switch (address_type) {
    case BLE_ADDR_PUBLIC_ID:
        return BLE_ADDR_PUBLIC;
    case BLE_ADDR_RANDOM_ID:
        return BLE_ADDR_RANDOM;
    default:
        return address_type;
    }
}

static bool is_bonded_peer_address(const ble_addr_t *addr, bool *resolved_rpa)
{
    *resolved_rpa = false;
    for (unsigned i = 0; i < bonded_peer_count; ++i) {
        if (identity_address_type(addr->type) ==
                identity_address_type(bonded_peer_addrs[i].type) &&
            memcmp(addr->val, bonded_peer_addrs[i].val, BLE_DEV_ADDR_LEN) == 0) {
            return true;
        }
    }

    if (BLE_ADDR_IS_RPA(addr)) {
        uint8_t rpa[BLE_DEV_ADDR_LEN];
        uint8_t identity[BLE_DEV_ADDR_LEN];
        uint8_t identity_type = BLE_ADDR_PUBLIC;
        memcpy(rpa, addr->val, sizeof(rpa));
        if (ble_gap_rpa_resolve(rpa, identity, &identity_type)) {
            for (unsigned i = 0; i < bonded_peer_count; ++i) {
                if (identity_type == bonded_peer_addrs[i].type &&
                    memcmp(identity, bonded_peer_addrs[i].val, BLE_DEV_ADDR_LEN) == 0) {
                    *resolved_rpa = true;
                    return true;
                }
            }
        }
    }
    return false;
}

static void remember_bonded_peer(const ble_addr_t *peer_id_addr)
{
    for (unsigned i = 0; i < bonded_peer_count; ++i) {
        if (bonded_peer_addrs[i].type == peer_id_addr->type &&
            memcmp(bonded_peer_addrs[i].val, peer_id_addr->val, BLE_DEV_ADDR_LEN) == 0) {
            return;
        }
    }
    if (bonded_peer_count < CONFIG_BT_NIMBLE_MAX_BONDS) {
        bonded_peer_addrs[bonded_peer_count++] = *peer_id_addr;
    } else {
        ESP_LOGW(tag, "Bond cache is full; peer identity will be reloaded after restart");
    }
}

static void print_discovery_report(const ble_addr_t *addr, int8_t rssi,
                                   const struct ble_hs_adv_fields *fields)
{
    char uuid[BLE_UUID_STR_LEN];
    char scan_report[160];
    bool resolved_rpa = false;
    bool bonded_peer = is_bonded_peer_address(addr, &resolved_rpa);
    bool rpa_address = BLE_ADDR_IS_RPA(addr);
    snprintf(scan_report, sizeof(scan_report),
             "%s addr=%02x:%02x:%02x:%02x:%02x:%02x type=%u RSSI=%d bond_match=%u rpa_addr=%u rpa_resolved=%u name=%.*s",
             is_mfi_candidate(fields) ? "MFi" : "BLE",
             addr->val[5], addr->val[4], addr->val[3], addr->val[2], addr->val[1], addr->val[0],
             addr->type, rssi, bonded_peer, rpa_address, resolved_rpa, fields->name_len,
             fields->name ? (const char *)fields->name : "");
    asha_web_set_ble_scan_report(scan_report);
    ESP_LOGI(tag, "ADV addr=%02x:%02x:%02x:%02x:%02x:%02x type=%u RSSI=%d bond_match=%u rpa_addr=%u rpa_resolved=%u name=\"%.*s\"",
             addr->val[5], addr->val[4], addr->val[3], addr->val[2], addr->val[1], addr->val[0],
             addr->type, rssi, bonded_peer, rpa_address, resolved_rpa, fields->name_len,
             fields->name ? (const char *)fields->name : "");
    if (is_mfi_candidate(fields)) {
        char event[112];
        snprintf(event, sizeof(event), "MFi ADV %02x:%02x:%02x:%02x:%02x:%02x type=%u RSSI=%d bond=%u RPA=%u/%u name=%.*s",
                 addr->val[5], addr->val[4], addr->val[3], addr->val[2], addr->val[1], addr->val[0],
                 addr->type, rssi, bonded_peer, rpa_address, resolved_rpa, fields->name_len,
                 fields->name ? (const char *)fields->name : "(adsız)");
        asha_web_add_event(event);
    }
    for (int i = 0; i < fields->num_uuids16; i++)
        ESP_LOGI(tag, "  service UUID16: %s", ble_uuid_to_str(&fields->uuids16[i].u, uuid));
    for (int i = 0; i < fields->num_uuids32; i++)
        ESP_LOGI(tag, "  service UUID32: %s", ble_uuid_to_str(&fields->uuids32[i].u, uuid));
    for (int i = 0; i < fields->num_uuids128; i++)
        ESP_LOGI(tag, "  service UUID128: %s", ble_uuid_to_str(&fields->uuids128[i].u, uuid));
    if (fields->svc_data_uuid16 && fields->svc_data_uuid16_len) {
        ESP_LOGI(tag, "  service data UUID16 (%u bytes)", fields->svc_data_uuid16_len);
        ESP_LOG_BUFFER_HEX_LEVEL(tag, fields->svc_data_uuid16, fields->svc_data_uuid16_len, ESP_LOG_INFO);
    }
    if (fields->svc_data_uuid128 && fields->svc_data_uuid128_len) {
        ESP_LOGI(tag, "  service data UUID128 (%u bytes)", fields->svc_data_uuid128_len);
        ESP_LOG_BUFFER_HEX_LEVEL(tag, fields->svc_data_uuid128, fields->svc_data_uuid128_len, ESP_LOG_INFO);
    }
}

static int asha_status_dsc_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                              uint16_t chr_val_handle, const struct ble_gatt_dsc *dsc,
                              void *arg);
static int asha_status_subscribe_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                                  struct ble_gatt_attr *attr, void *arg);
static int start_psm_read(uint16_t conn_handle);
static int psm_read_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                       struct ble_gatt_attr *attr, void *arg)
{
    (void)arg;
    if (error->status != 0) {
        char detail[96];
        snprintf(detail, sizeof(detail), "LE_PSM_OUT okunamadı (status=%d)", error->status);
        fail_asha_connection("ASHA PSM okuma hatası", detail);
        return 0;
    }
    if (!attr || !attr->om) {
        fail_asha_connection("ASHA PSM okuma hatası",
                             "LE_PSM_OUT okuması boş GATT yanıtı döndürdü");
        return 0;
    }
    uint16_t len = OS_MBUF_PKTLEN(attr->om);
    uint8_t value[2];
    if (len != sizeof(value) || os_mbuf_copydata(attr->om, 0, sizeof(value), value) != 0) {
        char detail[96];
        snprintf(detail, sizeof(detail), "LE_PSM_OUT uzunluğu geçersiz (%u bayt)", len);
        fail_asha_connection("ASHA PSM değeri geçersiz", detail);
        return 0;
    }
    uint16_t psm = (uint16_t)value[0] | ((uint16_t)value[1] << 8);
    ESP_LOGI(tag, "LE_PSM_OUT=0x%04x; opening ASHA L2CAP CoC", psm);
    mark_asha_progress();
    asha_web_set_status("LE_PSM_OUT bulundu", "ASHA L2CAP ses kanalı açılıyor");
    if (start_asha_coc(conn_handle, psm) != 0) {
        fail_asha_connection("ASHA CoC başlatılamadı", "L2CAP CoC bağlantı isteği gönderilemedi");
    }
    return 0;
}

static int asha_stop_write_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                              struct ble_gatt_attr *attr, void *arg)
{
    (void)conn_handle;
    (void)attr;
    (void)arg;
    if (error->status != 0) {
        char detail[96];
        snprintf(detail, sizeof(detail), "AudioControlPoint Stop yazımı başarısız (status=%d)",
                 error->status);
        fail_asha_connection("ASHA Stop hatası", detail);
        asha_cmd_state = ASHA_CMD_IDLE;
    } else {
        mark_asha_progress();
        ESP_LOGI(tag, "AudioControlPoint Stop write acknowledged; awaiting status notification");
    }
    return 0;
}

static int asha_start_write_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                               struct ble_gatt_attr *attr, void *arg)
{
    (void)conn_handle;
    (void)attr;
    (void)arg;
    if (error->status != 0) {
        char detail[96];
        snprintf(detail, sizeof(detail), "AudioControlPoint Start yazımı başarısız (status=%d)",
                 error->status);
        fail_asha_connection("ASHA Start hatası", detail);
        asha_cmd_state = ASHA_CMD_IDLE;
    } else {
        mark_asha_progress();
        ESP_LOGI(tag, "AudioControlPoint Start write acknowledged; awaiting status notification");
    }
    return 0;
}

static void asha_send_stop(uint16_t conn_handle)
{
    const uint8_t stop_cmd = 2;
    asha_cmd_state = ASHA_CMD_WAIT_STOP_STATUS;
    if (!start_asha_watchdog(conn_handle)) {
        fail_asha_connection("ASHA watchdog başlatılamadı",
                             "Stop handshake zaman aşımı görevi oluşturulamadı");
        return;
    }
    ESP_LOGI(tag, "Audio stream ending; sending ASHA Stop");
    int rc = ble_gattc_write_flat(conn_handle, audio_control_value_handle,
                                  &stop_cmd, sizeof(stop_cmd), asha_stop_write_cb, NULL);
    if (rc != 0) {
        asha_cmd_state = ASHA_CMD_IDLE;
        ESP_LOGE(tag, "Starting AudioControlPoint Stop write failed: rc=%d", rc);
        fail_asha_connection("ASHA Stop başlatılamadı", "AudioControlPoint yazma isteği kabul edilmedi");
    }
}

static void asha_test_audio_task(void *arg)
{
    uint16_t conn_handle = (uint16_t)(uintptr_t)arg;
    TaskHandle_t self = xTaskGetCurrentTaskHandle();
    uint8_t packet[ASHA_AUDIO_SDU_BYTES];
    int16_t pcm[ASHA_PCM_SAMPLES_PER_FRAME];
    int16_t stale_pcm[ASHA_PCM_SAMPLES_PER_FRAME];
    uint8_t sequence = 0;
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t frame_period = pdMS_TO_TICKS(ASHA_AUDIO_FRAME_PERIOD_MS);
    unsigned pacing_overruns = 0;
    unsigned late_dropped_frames = 0;
    bool stream_ok = true;

    asha_test_encoder_reset();
    ESP_LOGI(tag, "Streaming USB PCM to ASHA as G.722/16 kHz; silence is sent when USB audio is idle");
    asha_web_set_status("ASHA ses akışı aktif", "G.722 / 16 kHz; USB sessizse sessizlik gönderiliyor");
    while (true) {
        if (asha_audio_task_handle != self || asha_conn_handle != conn_handle ||
            asha_cmd_state != ASHA_CMD_STREAMING || asha_coc_chan == NULL) {
            stream_ok = false;
            break;
        }
        if (xQueueReceive(usb_pcm_queue, pcm, 0) != pdTRUE) {
            memset(pcm, 0, sizeof(pcm));
            if ((++usb_pcm_underflow_frames % 1500) == 1) {
                ESP_LOGW(tag, "USB PCM underrun; sending silence (%u missing frames total)",
                         usb_pcm_underflow_frames);
            }
        }
        mark_asha_progress();
        int encoded_len = asha_encode_pcm_frame(sequence, pcm, packet);
        if (encoded_len != ASHA_AUDIO_FRAME_BYTES) {
            ESP_LOGE(tag, "G.722 encoder returned %d bytes, expected %u",
                     encoded_len, ASHA_AUDIO_FRAME_BYTES);
            stream_ok = false;
            break;
        }

        struct os_mbuf *sdu = os_mbuf_get_pkthdr(&asha_coc_mbuf_pool, 0);
        if (sdu == NULL || os_mbuf_append(sdu, packet, sizeof(packet)) != 0) {
            if (sdu) os_mbuf_free_chain(sdu);
            ESP_LOGE(tag, "Could not allocate/append ASHA audio SDU");
            stream_ok = false;
            break;
        }

        int rc = ble_l2cap_send(asha_coc_chan, sdu);
        while (rc == BLE_HS_ESTALLED && asha_cmd_state == ASHA_CMD_STREAMING) {
            if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(500)) == 0) {
                ESP_LOGE(tag, "Timed out waiting for ASHA CoC transmit credits");
                rc = BLE_HS_ETIMEOUT;
                break;
            }
            rc = ble_l2cap_send(asha_coc_chan, sdu);
        }
        if (rc != 0) {
            os_mbuf_free_chain(sdu);
            ESP_LOGE(tag, "Sending ASHA audio SDU failed: rc=%d sequence=%u", rc, sequence);
            stream_ok = false;
            break;
        }
        ++sequence;
        if ((sequence % 100) == 0) {
            ESP_LOGI(tag, "ASHA USB audio SDUs sent: %u", sequence);
        }

        TickType_t now = xTaskGetTickCount();
        TickType_t elapsed = (TickType_t)(now - last_wake);
        if (elapsed >= frame_period) {
            ++pacing_overruns;
            unsigned dropped_now = 0;
            while (uxQueueMessagesWaiting(usb_pcm_queue) > 1 &&
                   xQueueReceive(usb_pcm_queue, stale_pcm, 0) == pdTRUE) {
                ++dropped_now;
            }
            late_dropped_frames += dropped_now;
            if ((pacing_overruns % 50) == 1) {
                ESP_LOGW(tag, "ASHA pacing overrun (%u total); discarded %u stale PCM frames (%u total)",
                         pacing_overruns, dropped_now, late_dropped_frames);
            }
            /* Rebase the periodic schedule instead of bursting to catch up. */
            last_wake = now;
        }
        vTaskDelayUntil(&last_wake, frame_period);
    }

    if (asha_cmd_state == ASHA_CMD_STREAMING) {
        if (!stream_ok) ESP_LOGW(tag, "USB audio stream ended; stopping ASHA");
        asha_send_stop(conn_handle);
    }
    if (asha_audio_task_handle == self) asha_audio_task_handle = NULL;
    vTaskDelete(NULL);
}

static void asha_send_start(uint16_t conn_handle)
{
    /* Start, G.722/16 kHz, unknown type, configurable signed dB attenuation, no other ear. */
    const uint8_t start_cmd[5] = {
        1, 1, 0, (uint8_t)(int8_t)CONFIG_ASHA_START_VOLUME_DB, 0
    };
    if (!audio_control_value_handle || !audio_status_notify_supported) {
        fail_asha_connection("ASHA Start başlatılamadı",
                             "AudioControlPoint veya AudioStatusPoint bildirimi kullanılamıyor");
        return;
    }
    asha_cmd_state = ASHA_CMD_WAIT_START_STATUS;
    mark_asha_progress();
    ESP_LOGI(tag, "Connection update complete; sending G.722/16 kHz Start command (volume=%d dB)",
             CONFIG_ASHA_START_VOLUME_DB);
    int rc = ble_gattc_write_flat(conn_handle, audio_control_value_handle,
                                  start_cmd, sizeof(start_cmd), asha_start_write_cb, NULL);
    if (rc != 0) {
        asha_cmd_state = ASHA_CMD_IDLE;
        ESP_LOGE(tag, "Starting AudioControlPoint Start write failed: rc=%d", rc);
        fail_asha_connection("ASHA Start başlatılamadı", "AudioControlPoint yazma isteği kabul edilmedi");
    }
}

static void asha_request_audio_conn_update(uint16_t conn_handle)
{
    if (asha_conn_update_started) return;
    asha_conn_handle = conn_handle;
    asha_conn_update_started = true;
    /* 20 ms interval; 5 ms connection event on LE 1M; 5 s supervision timeout. */
    const struct ble_gap_upd_params params = {
        .itvl_min = 16,
        .itvl_max = 16,
        .latency = 10,
        .supervision_timeout = 500,
        .min_ce_len = 8,
        .max_ce_len = 8,
    };
    ESP_LOGI(tag, "Requesting ASHA G.722 link parameters: interval=20 ms latency=10 CE=5 ms");
    int rc = ble_gap_update_params(conn_handle, &params);
    if (rc != 0) {
        asha_conn_update_started = false;
        ESP_LOGE(tag, "Connection parameter update request failed: rc=%d", rc);
        fail_asha_connection("BLE bağlantı parametresi hatası",
                             "ASHA ses aktarımı için bağlantı aralığı güncellenemedi");
    } else {
        mark_asha_progress();
    }
}

static int readonly_props_read_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                                struct ble_gatt_attr *attr, void *arg)
{
    (void)arg;
    if (error->status != 0) {
        char detail[96];
        snprintf(detail, sizeof(detail), "ReadOnlyProperties okunamadı (status=%d)",
                 error->status);
        fail_asha_connection("ASHA özellik okuma hatası", detail);
        return 0;
    }
    if (!attr || !attr->om) {
        fail_asha_connection("ASHA özellik okuma hatası",
                             "ReadOnlyProperties okuması boş GATT yanıtı döndürdü");
        return 0;
    }
    uint16_t len = OS_MBUF_PKTLEN(attr->om);
    uint16_t actual_len = len;
    uint8_t value[32];
    if (len > sizeof(value)) len = sizeof(value);
    if (os_mbuf_copydata(attr->om, 0, len, value) != 0) {
        fail_asha_connection("ASHA özellik okuma hatası",
                             "ReadOnlyProperties içeriği BLE paketinden kopyalanamadı");
        return 0;
    }
    ESP_LOGI(tag, "ReadOnlyProperties: %u byte(s), handle=%u", len, attr->handle);
    ESP_LOG_BUFFER_HEX_LEVEL(tag, value, len, ESP_LOG_INFO);
    char properties_event[96];
    int event_used = snprintf(properties_event, sizeof(properties_event),
                              "ASHA props len=%u data=", actual_len);
    for (uint16_t i = 0; i < len && event_used > 0 &&
         (size_t)event_used < sizeof(properties_event); ++i) {
        int written = snprintf(properties_event + event_used,
                               sizeof(properties_event) - (size_t)event_used,
                               "%02x", value[i]);
        if (written < 0 || (size_t)written >=
            sizeof(properties_event) - (size_t)event_used) break;
        event_used += written;
    }
    asha_web_add_event(properties_event);
    asha_read_only_properties_t properties;
    if (!asha_parse_read_only_properties(value, actual_len, &properties)) {
        char detail[96];
        if (actual_len != ASHA_READ_ONLY_PROPERTIES_SIZE) {
            snprintf(detail, sizeof(detail),
                     "ReadOnlyProperties length=%u, firmware expects %u",
                     actual_len, ASHA_READ_ONLY_PROPERTIES_SIZE);
        } else {
            snprintf(detail, sizeof(detail),
                     "ReadOnlyProperties version=%u (0x%02x), supported versions are %u/%u",
                     value[ASHA_PROP_OFFSET_VERSION],
                     value[ASHA_PROP_OFFSET_VERSION],
                     ASHA_PROP_VERSION_0, ASHA_PROP_VERSION_1);
        }
        asha_web_add_event(detail);
        fail_asha_connection("ASHA özellikleri geçersiz", detail);
        return 0;
    }
    ESP_LOGI(tag, "ASHA properties: version=%u device_caps=0x%02x side=%s binaural=%u feature_map=0x%02x coc_audio=%u render_delay_ms=%u codec_mask=0x%04x g722_16k=%u",
             properties.version, properties.device_capabilities,
             (properties.device_capabilities & 0x01) ? "right" : "left",
             (properties.device_capabilities >> 1) & 0x01,
             properties.feature_map,
             properties.feature_map & ASHA_PROP_FEATURE_LE_COC,
             properties.render_delay_ms, properties.codec_mask,
             (properties.codec_mask & ASHA_PROP_CODEC_G722_16KHZ) != 0);
    if (!asha_read_only_properties_support_g722(&properties)) {
        fail_asha_connection("ASHA ses biçimi desteklenmiyor",
                             "Cihaz CoC sesini veya G.722/16 kHz codec'ini desteklemiyor");
        return 0;
    }
    mark_asha_progress();
    asha_request_audio_conn_update(conn_handle);
    return 0;
}
static int asha_coc_event_cb(struct ble_l2cap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {
    case BLE_L2CAP_EVENT_COC_CONNECTED:
        if (event->connect.status != 0) {
            char detail[96];
            snprintf(detail, sizeof(detail), "L2CAP CoC bağlantısı başarısız (status=%d)",
                     event->connect.status);
            fail_asha_connection("ASHA CoC bağlantı hatası", detail);
            return 0;
        }
        struct ble_l2cap_chan_info info;
        int rc = ble_l2cap_get_chan_info(event->connect.chan, &info);
        if (rc != 0) {
            ESP_LOGE(tag, "CoC connected, but channel info lookup failed: rc=%d", rc);
            fail_asha_connection("ASHA CoC hatası", "Bağlanan L2CAP kanal bilgisi alınamadı");
            return 0;
        }
        ESP_LOGI(tag, "ASHA L2CAP CoC CONNECTED: conn=%u psm=0x%04x scid=0x%04x dcid=0x%04x local_mtu=%u peer_mtu=%u",
                 event->connect.conn_handle, info.psm, info.scid, info.dcid,
                 info.our_coc_mtu, info.peer_coc_mtu);
        asha_web_set_status("ASHA CoC bağlı", "L2CAP ses kanalı açıldı; ASHA özellikleri okunuyor");
        mark_asha_progress();
        asha_coc_chan = event->connect.chan;
        if (readonly_props_value_handle != 0) {
            rc = ble_gattc_read(event->connect.conn_handle, readonly_props_value_handle,
                                readonly_props_read_cb, NULL);
            if (rc != 0) {
                ESP_LOGE(tag, "Starting ReadOnlyProperties read failed: rc=%d", rc);
                fail_asha_connection("ASHA özellik okuması başlatılamadı",
                                     "ReadOnlyProperties GATT okuma isteği kabul edilmedi");
            }
        } else {
            ESP_LOGW(tag, "ReadOnlyProperties characteristic was not discovered");
            fail_asha_connection("ASHA özellikleri eksik",
                                 "ReadOnlyProperties karakteristiği bulunamadı");
        }
        return 0;
    case BLE_L2CAP_EVENT_COC_DISCONNECTED:
        asha_coc_chan = NULL;
        ESP_LOGW(tag, "ASHA L2CAP CoC disconnected: conn=%u",
                 event->disconnect.conn_handle);
        if (event->disconnect.conn_handle == asha_conn_handle &&
            asha_cmd_state != ASHA_CMD_IDLE) {
            fail_asha_connection("ASHA CoC bağlantısı kesildi",
                                 "Ses kanalı beklenmedik biçimde kapandı; BLE yeniden bağlanıyor");
        }
        return 0;
    case BLE_L2CAP_EVENT_COC_TX_UNSTALLED:
        if (asha_audio_task_handle != NULL) {
            xTaskNotifyGive(asha_audio_task_handle);
        }
        ESP_LOGD(tag, "ASHA CoC transmit resumed: conn=%u status=%d",
                 event->tx_unstalled.conn_handle, event->tx_unstalled.status);
        return 0;
    case BLE_L2CAP_EVENT_COC_DATA_RECEIVED:
        ESP_LOGI(tag, "CoC received unexpected test SDU: %u bytes",
                 OS_MBUF_PKTLEN(event->receive.sdu_rx));
        os_mbuf_free_chain(event->receive.sdu_rx);
        return 0;
    default:
        ESP_LOGI(tag, "L2CAP CoC event type=%u", event->type);
        return 0;
    }
}

static int start_asha_coc(uint16_t conn_handle, uint16_t psm)
{
    struct os_mbuf *sdu_rx;
    if (psm == 0) {
        ESP_LOGE(tag, "Peer returned invalid LE_PSM_OUT=0");
        return BLE_HS_EINVAL;
    }
    if (asha_coc_connect_started) {
        ESP_LOGW(tag, "CoC connection request already started; ignoring duplicate PSM read");
        return BLE_HS_EALREADY;
    }
    sdu_rx = os_mbuf_get_pkthdr(&asha_coc_mbuf_pool, 0);
    if (sdu_rx == NULL) {
        ESP_LOGE(tag, "Could not allocate CoC receive SDU buffer");
        return BLE_HS_ENOMEM;
    }
    asha_coc_connect_started = true;
    ESP_LOGI(tag, "Initiating L2CAP CoC: conn=%u psm=0x%04x mtu=%u",
             conn_handle, psm, ASHA_COC_MTU);
    int rc = ble_l2cap_connect(conn_handle, psm, ASHA_COC_MTU, sdu_rx,
                               asha_coc_event_cb, NULL);
    if (rc != 0) {
        asha_coc_connect_started = false;
        os_mbuf_free_chain(sdu_rx);
        ESP_LOGE(tag, "Starting ASHA L2CAP CoC failed immediately: rc=%d", rc);
    }
    return rc;
}
static int start_psm_read(uint16_t conn_handle)
{
    if (!psm_value_handle) {
        fail_asha_connection("ASHA PSM karakteristiği eksik",
                             "LE_PSM_OUT handle'ı bulunmadığı için CoC açılamıyor");
        return BLE_HS_EINVAL;
    }
    int rc = ble_gattc_read(conn_handle, psm_value_handle, psm_read_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(tag, "Starting LE_PSM_OUT read failed: rc=%d", rc);
        fail_asha_connection("ASHA PSM okuması başlatılamadı",
                             "LE_PSM_OUT GATT okuma isteği kabul edilmedi");
    } else {
        mark_asha_progress();
    }
    return rc;
}

static int asha_status_subscribe_cb(uint16_t conn_handle,
                                    const struct ble_gatt_error *error,
                                    struct ble_gatt_attr *attr, void *arg)
{
    (void)arg;
    if (error->status != 0) {
        char detail[96];
        snprintf(detail, sizeof(detail), "AudioStatusPoint CCCD yazımı başarısız (status=%d)",
                 error->status);
        fail_asha_connection("ASHA bildirim aboneliği hatası", detail);
        return 0;
    }
    mark_asha_progress();
    ESP_LOGI(tag, "Subscribed to AudioStatusPoint notifications (CCCD handle=%u)",
             attr ? attr->handle : audio_status_cccd_handle);
    start_psm_read(conn_handle);
    return 0;
}

static int asha_status_dsc_cb(uint16_t conn_handle,
                              const struct ble_gatt_error *error,
                              uint16_t chr_val_handle,
                              const struct ble_gatt_dsc *dsc, void *arg)
{
    (void)arg;
    if (error->status == BLE_HS_EDONE) {
        if (!audio_status_cccd_handle) {
            fail_asha_connection("ASHA bildirim desteği eksik",
                                 "AudioStatusPoint CCCD bulunamadı; Start/Stop handshake yapılamaz");
            return 0;
        }
        const uint8_t cccd_value[2] = {0x01, 0x00};
        ESP_LOGI(tag, "Enabling AudioStatusPoint notifications");
        int rc = ble_gattc_write_flat(conn_handle, audio_status_cccd_handle,
                                      cccd_value, sizeof(cccd_value),
                                      asha_status_subscribe_cb, NULL);
        if (rc != 0) {
            ESP_LOGE(tag, "Starting AudioStatusPoint subscription failed: rc=%d", rc);
            fail_asha_connection("ASHA bildirim aboneliği başlatılamadı",
                                 "CCCD yazma isteği kabul edilmedi");
        } else {
            mark_asha_progress();
        }
        return 0;
    }
    if (error->status != 0) {
        ESP_LOGE(tag, "AudioStatusPoint descriptor discovery failed: status=%d",
                 error->status);
        fail_asha_connection("ASHA descriptor keşif hatası",
                             "AudioStatusPoint CCCD keşfi başarısız oldu");
        return 0;
    }
    if (chr_val_handle == audio_status_value_handle && dsc &&
        ble_uuid_cmp(&dsc->uuid.u, BLE_UUID16_DECLARE(BLE_GATT_DSC_CLT_CFG_UUID16)) == 0) {
        audio_status_cccd_handle = dsc->handle;
        ESP_LOGI(tag, "Found AudioStatusPoint CCCD: handle=%u", dsc->handle);
    }
    return 0;
}
static int psm_chr_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                      const struct ble_gatt_chr *chr, void *arg)
{
    (void)arg;
    if (error->status == BLE_HS_EDONE) {
        if (!psm_value_handle) {
            fail_asha_connection("ASHA karakteristiği eksik",
                                 "LE_PSM_OUT karakteristiği bulunamadı");
            return 0;
        }
        if (!(psm_properties & BLE_GATT_CHR_PROP_READ)) {
            fail_asha_connection("ASHA karakteristiği geçersiz",
                                 "LE_PSM_OUT okuma özelliğine sahip değil");
            return 0;
        }
        if (!readonly_props_value_handle || !audio_control_value_handle ||
            !audio_status_value_handle || !audio_status_notify_supported) {
            fail_asha_connection("ASHA karakteristikleri eksik",
                                 "ReadOnlyProperties, AudioControlPoint veya AudioStatusPoint notify eksik");
            return 0;
        }
        if (!(readonly_props_properties & BLE_GATT_CHR_PROP_READ) ||
            !(audio_control_properties & BLE_GATT_CHR_PROP_WRITE)) {
            fail_asha_connection("ASHA karakteristik özellikleri geçersiz",
                                 "ReadOnlyProperties okunamıyor veya AudioControlPoint yazılamıyor");
            return 0;
        }
        int rc = ble_gattc_disc_all_dscs(conn_handle, audio_status_value_handle,
                                         asha_end_handle, asha_status_dsc_cb, NULL);
        if (rc != 0) {
            ESP_LOGE(tag, "Starting AudioStatusPoint descriptor discovery failed: rc=%d", rc);
            fail_asha_connection("ASHA descriptor keşfi başlatılamadı",
                                 "AudioStatusPoint CCCD keşif isteği kabul edilmedi");
        } else {
            mark_asha_progress();
        }
        return 0;
    }
    if (error->status != 0) {
        ESP_LOGE(tag, "Characteristic discovery failed: status=%d", error->status);
        fail_asha_connection("ASHA karakteristik keşif hatası",
                             "ASHA servis karakteristikleri keşfedilemedi");
        return 0;
    }
    if (chr && ble_uuid_cmp(&chr->uuid.u, &psm_uuid.u) == 0) {
        psm_value_handle = chr->val_handle;
        psm_properties = chr->properties;
        ESP_LOGI(tag, "Found LE_PSM_OUT: value_handle=%u properties=0x%02x",
                 chr->val_handle, chr->properties);
    } else if (chr && ble_uuid_cmp(&chr->uuid.u, &readonly_props_uuid.u) == 0) {
        readonly_props_value_handle = chr->val_handle;
        readonly_props_properties = chr->properties;
        ESP_LOGI(tag, "Found ReadOnlyProperties: value_handle=%u properties=0x%02x",
                 chr->val_handle, chr->properties);
    } else if (chr && ble_uuid_cmp(&chr->uuid.u, &audio_control_uuid.u) == 0) {
        audio_control_value_handle = chr->val_handle;
        audio_control_properties = chr->properties;
        ESP_LOGI(tag, "Found AudioControlPoint: value_handle=%u properties=0x%02x",
                 chr->val_handle, chr->properties);
    } else if (chr && ble_uuid_cmp(&chr->uuid.u, &audio_status_uuid.u) == 0) {
        audio_status_value_handle = chr->val_handle;
        audio_status_notify_supported = (chr->properties & BLE_GATT_CHR_PROP_NOTIFY) != 0;
        ESP_LOGI(tag, "Found AudioStatusPoint: value_handle=%u properties=0x%02x notify=%u",
                 chr->val_handle, chr->properties, audio_status_notify_supported);
    } else if (chr && ble_uuid_cmp(&chr->uuid.u, &volume_uuid.u) == 0) {
        volume_value_handle = chr->val_handle;
        ESP_LOGI(tag, "Found Volume: value_handle=%u properties=0x%02x",
                 chr->val_handle, chr->properties);
    }
    return 0;
}
static void start_asha_characteristic_discovery(uint16_t conn_handle)
{
    ESP_LOGI(tag, "ASHA service handles: %u..%u; discovering characteristics",
             asha_start_handle, asha_end_handle);
    psm_value_handle = 0;
    psm_properties = 0;
    readonly_props_value_handle = 0;
    readonly_props_properties = 0;
    audio_control_value_handle = 0;
    audio_control_properties = 0;
    audio_status_value_handle = 0;
    audio_status_cccd_handle = 0;
    volume_value_handle = 0;
    audio_status_notify_supported = false;
    int rc = ble_gattc_disc_all_chrs(conn_handle, asha_start_handle,
                                     asha_end_handle, psm_chr_cb, NULL);
    if (rc) {
        ESP_LOGE(tag, "Starting ASHA characteristic discovery failed: rc=%d", rc);
        fail_asha_connection("ASHA karakteristik keşfi başlatılamadı",
                             "GATT karakteristik keşif isteği kabul edilmedi");
    } else {
        mark_asha_progress();
    }
}

static int asha_svc_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                       const struct ble_gatt_svc *service, void *arg)
{
    (void)arg;
    if (error->status == BLE_HS_EDONE) {
        if (!asha_start_handle) {
            fail_asha_connection("ASHA servisi bulunamadı",
                                 "Bağlanılan cihaz ASHA FDF0 servisini sunmuyor");
            return 0;
        }
        mark_asha_progress();
        ESP_LOGI(tag, "ASHA FDF0 found; requesting stored-bond encryption or pairing");
        asha_web_set_status("ASHA servisi bulundu", "Güvenli bağlantı / eşleşme başlatılıyor");
        start_pairing(conn_handle);
        return 0;
    }
    if (error->status != 0) {
        ESP_LOGE(tag, "Service discovery failed: status=%d", error->status);
        fail_asha_connection("ASHA servis keşif hatası",
                             "GATT servis keşfi başarısız oldu");
        return 0;
    }
    if (service && ble_uuid_cmp(&service->uuid.u, &asha_uuid.u) == 0) {
        asha_start_handle = service->start_handle;
        asha_end_handle = service->end_handle;
        mark_asha_progress();
        ESP_LOGI(tag, "Found ASHA FDF0 service handles: %u..%u",
                 service->start_handle, service->end_handle);
    }
    return 0;
}
static void start_asha_service_discovery(uint16_t conn_handle)
{
    reset_asha_connection_state();
    int rc = ble_gattc_disc_all_svcs(conn_handle, asha_svc_cb, NULL);
    if (rc) {
        ESP_LOGE(tag, "Starting GATT service discovery failed: rc=%d", rc);
        fail_asha_connection("ASHA servis keşfi başlatılamadı",
                             "GATT servis keşif isteği kabul edilmedi");
    }
}

static void start_pairing(uint16_t conn_handle)
{
    int rc = ble_gap_security_initiate(conn_handle);
    if (rc) {
        ESP_LOGE(tag, "Starting ASHA security failed: rc=%d", rc);
        fail_asha_connection("ASHA güvenlik hatası",
                             "BLE şifreleme/eşleşme işlemi başlatılamadı");
        return;
    }
    mark_asha_progress();
    ESP_LOGI(tag, "Security procedure started; stored bond will encrypt link, otherwise pairing will run");
}

static void start_characteristics_after_security(uint16_t conn_handle)
{
    if (gatt_discovery_started) return;
    struct ble_gap_conn_desc desc;
    int rc = ble_gap_conn_find(conn_handle, &desc);
    if (rc != 0) {
        fail_asha_connection("ASHA güvenlik hatası", "BLE bağlantı durumu okunamadı");
        return;
    }
    if (!desc.sec_state.encrypted) {
        ESP_LOGW(tag, "Deferring ASHA characteristic discovery until link encryption is active");
        return;
    }
    ESP_LOGI(tag, "Encrypted ASHA link ready; discovering characteristics");
    start_asha_characteristic_discovery(conn_handle);
    if (asha_conn_handle == conn_handle) gatt_discovery_started = true;
}

static void blecent_scan(void)
{
    uint8_t own_addr_type;
    struct ble_gap_disc_params params = {0};
    int rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc) { ESP_LOGE(tag, "Address type selection failed: rc=%d", rc); return; }
    params.filter_duplicates = 1;
    params.passive = 1;
    rc = ble_gap_disc(own_addr_type, BLE_HS_FOREVER, &params, blecent_gap_event, NULL);
    if (rc) {
        ESP_LOGE(tag, "Starting scan failed: rc=%d", rc);
        asha_web_set_status("BLE taraması başlatılamadı", "NimBLE tarama API'si hata döndürdü; seri log gerekli");
    } else {
        asha_web_set_status("Kanso aranıyor", "MFi reklamları taranıyor; Kanso eşleşme modunda ve takılı olsun");
    }
}

static int web_retry_asha(void)
{
    if (asha_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        asha_web_set_status("Bağlantı yenileniyor", "Mevcut BLE bağlantısı kapatılıyor; otomatik tarama yeniden başlayacak");
        return ble_gap_terminate(asha_conn_handle, ASHA_TERMINATE_REASON);
    }
    if (connect_started) {
        asha_web_set_status("Bağlantı deneniyor", "Mevcut Kanso bağlantı denemesi sürüyor");
        return 0;
    }
    asha_web_set_status("Kanso aranıyor", "NimBLE taraması zaten otomatik olarak çalışıyor");
    return 0;
}

static void maybe_connect_candidate(const ble_addr_t *addr, const struct ble_hs_adv_fields *fields)
{
    bool mfi_candidate = is_mfi_candidate(fields);
    bool resolved_rpa = false;
    bool bonded_peer = is_bonded_peer_address(addr, &resolved_rpa);
    if ((!mfi_candidate && !bonded_peer) || connect_started) return;
    connect_started = true;
    if (bonded_peer && !mfi_candidate) {
        if (resolved_rpa) {
            ESP_LOGI(tag, "Resolved private advertisement address to stored bonded peer; connecting");
        } else {
            ESP_LOGI(tag, "Stored bonded peer advertisement found; connecting without MFi UUID");
        }
        asha_web_set_status("Kayıtlı Kanso bulundu",
                            resolved_rpa ?
                            "Dönen BLE özel adresi bond anahtarıyla çözüldü; bağlantı deneniyor" :
                            "NVS’deki bond adresi reklamda eşleşti; bağlantı deneniyor");
    } else {
        ESP_LOGI(tag, "MFi UUID candidate found; cancelling scan before ASHA GATT connection");
        asha_web_set_status("MFi adayı bulundu", "MFi UUID reklamı görüldü; bağlantı deneniyor");
    }

    int cancel_rc = ble_gap_disc_cancel();
    if (cancel_rc != 0 && cancel_rc != BLE_HS_EALREADY) {
        ESP_LOGE(tag, "Could not cancel scan: rc=%d", cancel_rc);
        connect_started = false;
        return;
    }

    uint8_t own_addr_type;
    int rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc == 0) rc = ble_gap_connect(own_addr_type, addr, 30000, NULL, blecent_gap_event, NULL);
    if (rc) {
        ESP_LOGE(tag, "Candidate connection start failed: rc=%d; scanning again", rc);
        connect_started = false;
        blecent_scan();
    }
}

static int blecent_gap_event(struct ble_gap_event *event, void *arg)
{
    struct ble_hs_adv_fields fields;
    int rc;
    switch (event->type) {
    case BLE_GAP_EVENT_DISC:
        rc = ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data);
        if (rc) return 0;
        print_discovery_report(&event->disc.addr, event->disc.rssi, &fields);
        maybe_connect_candidate(&event->disc.addr, &fields);
        return 0;
#if CONFIG_EXAMPLE_EXTENDED_ADV
    case BLE_GAP_EVENT_EXT_DISC:
        rc = ble_hs_adv_parse_fields(&fields, event->ext_disc.data, event->ext_disc.length_data);
        if (rc) { ESP_LOGW(tag, "Extended advertisement parse failed: rc=%d", rc); return 0; }
        print_discovery_report(&event->ext_disc.addr, event->ext_disc.rssi, &fields);
        ESP_LOGI(tag, "  extended data_status=%u SID=%u PHY=%u/%u", event->ext_disc.data_status,
                 event->ext_disc.sid, event->ext_disc.prim_phy, event->ext_disc.sec_phy);
        maybe_connect_candidate(&event->ext_disc.addr, &fields);
        return 0;
#endif
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status) {
            ESP_LOGE(tag, "Connection failed: status=%d", event->connect.status);
            connect_started = false;
            asha_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            asha_watchdog_enabled = false;
            reset_asha_connection_state();
            blecent_scan();
        } else {
            ESP_LOGI(tag, "Connected; discovering GATT services before pairing");
            asha_conn_handle = event->connect.conn_handle;
            asha_watchdog_enabled = false;
            reset_asha_connection_state();
            if (!start_asha_watchdog(event->connect.conn_handle)) {
                fail_asha_connection("ASHA watchdog başlatılamadı",
                                     "BLE kurulum zaman aşımı görevi oluşturulamadı");
                return 0;
            }
            asha_web_set_status("BLE bağlı", "ASHA FDF0 servisi keşfediliyor");
            start_asha_service_discovery(event->connect.conn_handle);
        }
        return 0;
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(tag, "Disconnected: reason=%d", event->disconnect.reason);
        {
            char disconnect_detail[96];
            snprintf(disconnect_detail, sizeof(disconnect_detail),
                     "BLE disconnect reason=%d (0x%x): %s; yeniden taranıyor",
                     event->disconnect.reason,
                     (unsigned)event->disconnect.reason,
                     ble_status_description(event->disconnect.reason));
            asha_web_set_status("BLE bağlantısı kesildi", disconnect_detail);
        }
        connect_started = false;
        asha_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        asha_watchdog_enabled = false;
        reset_asha_connection_state();
        blecent_scan();
        return 0;
    case BLE_GAP_EVENT_NOTIFY_RX: {
        if (event->notify_rx.conn_handle == asha_conn_handle &&
            event->notify_rx.attr_handle == audio_status_value_handle) {
            uint8_t status = 0xff;
            uint16_t len = event->notify_rx.om ? OS_MBUF_PKTLEN(event->notify_rx.om) : 0;
            bool status_valid = event->notify_rx.om && len == 1 &&
                                os_mbuf_copydata(event->notify_rx.om, 0, 1, &status) == 0;
            if (status_valid) {
                ESP_LOGI(tag, "AudioStatusPoint notification: status=0x%02x conn=%u",
                         status, event->notify_rx.conn_handle);
                if (event->notify_rx.conn_handle == asha_conn_handle && status == 0) {
                    if (asha_cmd_state == ASHA_CMD_WAIT_START_STATUS) {
                        asha_cmd_state = ASHA_CMD_STREAMING;
                        asha_watchdog_enabled = false;
                        mark_asha_progress();
                        if (xTaskCreate(asha_test_audio_task, "asha_test_audio", 4096,
                                        (void *)(uintptr_t)event->notify_rx.conn_handle,
                                        5, &asha_audio_task_handle) != pdPASS) {
                            ESP_LOGE(tag, "Could not create ASHA test audio task");
                            fail_asha_connection("ASHA ses görevi başlatılamadı",
                                                 "USB PCM aktarım görevi oluşturulamadı");
                        }
                    } else if (asha_cmd_state == ASHA_CMD_WAIT_STOP_STATUS) {
                        asha_watchdog_enabled = false;
                        asha_cmd_state = ASHA_CMD_IDLE;
                        mark_asha_progress();
                        ESP_LOGI(tag, "ASHA audio stream stopped successfully");
                    }
                } else if (status_valid && event->notify_rx.conn_handle == asha_conn_handle &&
                           (asha_cmd_state == ASHA_CMD_WAIT_START_STATUS ||
                            asha_cmd_state == ASHA_CMD_WAIT_STOP_STATUS)) {
                    ESP_LOGE(tag, "ASHA command rejected with AudioStatusPoint=0x%02x; handshake stopped", status);
                    fail_asha_connection("ASHA handshake reddedildi",
                                         "İşitme cihazı Start/Stop komutunu kabul etmedi");
                }
            } else {
                fail_asha_connection("ASHA status bildirimi geçersiz",
                                     "AudioStatusPoint bildirimi tam olarak bir status baytı içermeli");
            }
        }
        return 0;
    }
    case BLE_GAP_EVENT_CONN_UPDATE: {
        uint16_t conn_handle = event->conn_update.conn_handle;
        ESP_LOGI(tag, "Connection update complete: status=%d conn=%u",
                 event->conn_update.status, conn_handle);
        if (asha_conn_update_started && conn_handle == asha_conn_handle &&
            event->conn_update.status == 0) {
            asha_conn_update_started = false;
            mark_asha_progress();
            struct ble_gap_conn_desc desc;
            rc = ble_gap_conn_find(conn_handle, &desc);
            if (rc == 0) {
                ESP_LOGI(tag, "Negotiated link: interval=%u units (%.2f ms) latency=%u timeout=%u units",
                         desc.conn_itvl, desc.conn_itvl * 1.25, desc.conn_latency,
                         desc.supervision_timeout);
                asha_send_start(conn_handle);
            } else {
                ESP_LOGE(tag, "Connection update succeeded but link lookup failed: rc=%d", rc);
                fail_asha_connection("BLE bağlantı parametresi hatası",
                                     "Güncellenen bağlantı parametreleri okunamadı");
            }
        } else if (asha_conn_update_started && conn_handle == asha_conn_handle) {
            asha_conn_update_started = false;
            char detail[96];
            snprintf(detail, sizeof(detail), "Bağlantı parametresi güncellemesi reddedildi (status=%d)",
                     event->conn_update.status);
            fail_asha_connection("BLE bağlantı parametresi hatası", detail);
        }
        return 0;
    }
    case BLE_GAP_EVENT_DISC_COMPLETE:
        ESP_LOGI(tag, "Scan complete: reason=%d", event->disc_complete.reason);
        return 0;
    case BLE_GAP_EVENT_ENC_CHANGE: {
        if (event->enc_change.status != 0) {
            ESP_LOGE(tag, "Link encryption failed: status=%d conn_handle=%u",
                     event->enc_change.status, event->enc_change.conn_handle);
            char security_error[96];
            snprintf(security_error, sizeof(security_error),
                     "BLE encryption failed: status=%d (0x%x), %s",
                     event->enc_change.status,
                     (unsigned)event->enc_change.status,
                     ble_status_description(event->enc_change.status));
            asha_web_add_event(security_error);
            if (event->enc_change.status == BLE_HS_HCI_ERR(BLE_ERR_PINKEY_MISSING)) {
                struct ble_gap_conn_desc desc;
                if (ble_gap_conn_find(event->enc_change.conn_handle, &desc) == 0) {
                    int delete_rc = ble_store_util_delete_peer(&desc.peer_id_addr);
                    if (delete_rc == 0) {
                        ESP_LOGW(tag, "Removed stale bond after peer reported PIN or key missing; will pair again");
                        asha_web_add_event("Eski BLE bond anahtarı geçersiz; kayıt silinip yeniden eşleşilecek");
                    } else {
                        ESP_LOGE(tag, "Could not remove stale bond after key-missing error: rc=%d",
                                 delete_rc);
                    }
                } else {
                    ESP_LOGE(tag,                     "Could not inspect connection after key-missing error");
                }
            }
            fail_asha_connection("ASHA şifreleme hatası", security_error);
            return 0;
        }
        struct ble_gap_conn_desc desc;
        rc = ble_gap_conn_find(event->enc_change.conn_handle, &desc);
        if (rc == 0) {
            ESP_LOGI(tag, "Security state conn=%u encrypted=%u authenticated=%u bonded=%u key_size=%u",
                     event->enc_change.conn_handle, desc.sec_state.encrypted,
                     desc.sec_state.authenticated, desc.sec_state.bonded,
                     desc.sec_state.key_size);
            char bond_event[80];
            snprintf(bond_event, sizeof(bond_event), "BLE şifreleme: bonded=%u, anahtar=%u bit",
                     desc.sec_state.bonded, desc.sec_state.key_size);
            asha_web_add_event(bond_event);
            if (desc.sec_state.encrypted && !gatt_discovery_started) {
                mark_asha_progress();
                start_characteristics_after_security(event->enc_change.conn_handle);
            }
        } else {
            ESP_LOGI(tag, "Link encryption enabled on conn_handle=%u; conn_find rc=%d",
                     event->enc_change.conn_handle, rc);
            if (event->enc_change.conn_handle == asha_conn_handle) {
                fail_asha_connection("ASHA güvenlik hatası",
                                     "Şifrelenen BLE bağlantısının durumu okunamadı");
            }
        }
        return 0;
    }
    case BLE_GAP_EVENT_PASSKEY_ACTION:
        ESP_LOGW(tag, "Pairing requested interactive passkey action=%u conn_handle=%u",
                 event->passkey.params.action, event->passkey.conn_handle);
        return 0;
    case BLE_GAP_EVENT_REPEAT_PAIRING: {
        struct ble_gap_conn_desc desc;
        rc = ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc);
        if (rc) {
            ESP_LOGE(tag, "Repeat-pairing connection lookup failed: rc=%d", rc);
            return BLE_GAP_REPEAT_PAIRING_IGNORE;
        }
        rc = ble_store_util_delete_peer(&desc.peer_id_addr);
        if (rc != 0) {
            ESP_LOGE(tag, "Could not remove stale bond requested for re-pairing: rc=%d", rc);
            asha_web_add_event("Eski bond silinemedi; güvenli yeniden eşleşme reddedildi");
            return BLE_GAP_REPEAT_PAIRING_IGNORE;
        }
        ESP_LOGW(tag, "Removed stale bond after peer requested re-pairing; retrying pairing");
        asha_web_add_event("Eş cihaz yeniden eşleşme istedi; eski bond silinip eşleşme yineleniyor");
        return BLE_GAP_REPEAT_PAIRING_RETRY;
    }
    case BLE_GAP_EVENT_PARING_COMPLETE: {
        uint16_t conn_handle = event->pairing_complete.conn_handle;
        ESP_LOGI(tag, "Pairing procedure callback: SM error=%d conn_handle=%u",
                 event->pairing_complete.status, conn_handle);
        char pairing_result[96];
        snprintf(pairing_result, sizeof(pairing_result),
                 "SMP callback SM error=%d (0x%x)",
                 event->pairing_complete.status,
                 (unsigned)event->pairing_complete.status);
        if (event->pairing_complete.status != 0) {
            ESP_LOGE(tag, "Pairing procedure reported an SMP error");
            asha_web_add_event(pairing_result);
            fail_asha_connection("ASHA eşleşme hatası",
                                 pairing_result);
            return 0;
        }
        struct ble_gap_conn_desc desc;
        rc = ble_gap_conn_find(conn_handle, &desc);
        if (rc) {
            ESP_LOGE(tag, "Could not inspect paired link: rc=%d", rc);
            fail_asha_connection("ASHA eşleşme hatası",
                                 "Eşleşme sonrası BLE güvenlik durumu okunamadı");
            return 0;
        }
        ESP_LOGI(tag, "Pairing link state: encrypted=%u authenticated=%u bonded=%u key_size=%u",
                 desc.sec_state.encrypted, desc.sec_state.authenticated,
                 desc.sec_state.bonded, desc.sec_state.key_size);
        char security_state[96];
        snprintf(security_state, sizeof(security_state),
                 "BLE security callback snapshot: enc=%u auth=%u bond=%u key=%u; encryption result follows",
                 desc.sec_state.encrypted, desc.sec_state.authenticated,
                 desc.sec_state.bonded, desc.sec_state.key_size);
        asha_web_add_event(security_state);
        if (desc.sec_state.bonded) remember_bonded_peer(&desc.peer_id_addr);
        mark_asha_progress();
        start_characteristics_after_security(conn_handle);
        return 0;
    }
    default:
        return 0;
    }
}

static void blecent_on_reset(int reason) { ESP_LOGE(tag, "NimBLE reset: reason=%d", reason); }
static void blecent_on_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    if (rc) { ESP_LOGE(tag, "Identity address setup failed: rc=%d", rc); return; }
    ble_addr_t bonded_peers[CONFIG_BT_NIMBLE_MAX_BONDS];
    int bonded_count = 0;
    rc = ble_store_util_bonded_peers(bonded_peers, &bonded_count,
                                      CONFIG_BT_NIMBLE_MAX_BONDS);
    if (rc == 0) {
        bonded_peer_count = bonded_count > CONFIG_BT_NIMBLE_MAX_BONDS ?
                            CONFIG_BT_NIMBLE_MAX_BONDS : (unsigned)bonded_count;
        memcpy(bonded_peer_addrs, bonded_peers, bonded_peer_count * sizeof(ble_addr_t));
        ESP_LOGI(tag, "Persistent BLE bonds loaded from NVS: %d/%d",
                 bonded_count, CONFIG_BT_NIMBLE_MAX_BONDS);
        char bond_event[96];
        snprintf(bond_event, sizeof(bond_event), "Başlangıçta NVS’den yüklenen BLE bond kaydı: %d",
                 bonded_count);
        asha_web_add_event(bond_event);
        for (unsigned i = 0; i < bonded_peer_count; ++i) {
            char bond_address[96];
            snprintf(bond_address, sizeof(bond_address),
                     "Stored bond[%u] identity=%02x:%02x:%02x:%02x:%02x:%02x type=%u",
                     i, bonded_peer_addrs[i].val[5], bonded_peer_addrs[i].val[4],
                     bonded_peer_addrs[i].val[3], bonded_peer_addrs[i].val[2],
                     bonded_peer_addrs[i].val[1], bonded_peer_addrs[i].val[0],
                     bonded_peer_addrs[i].type);
            asha_web_add_event(bond_address);
            ESP_LOGI(tag, "Stored bond peer[%u] identity addr=%02x:%02x:%02x:%02x:%02x:%02x type=%u",
                     i, bonded_peer_addrs[i].val[5], bonded_peer_addrs[i].val[4],
                     bonded_peer_addrs[i].val[3], bonded_peer_addrs[i].val[2],
                     bonded_peer_addrs[i].val[1], bonded_peer_addrs[i].val[0],
                     bonded_peer_addrs[i].type);
        }
    } else {
        ESP_LOGE(tag, "Reading persistent BLE bonds failed: rc=%d", rc);
        asha_web_add_event("NVS’den BLE bond kayıtları okunamadı; seri logu kontrol edin");
    }
    blecent_scan();
}
static void blecent_host_task(void *param)
{
    ESP_LOGI(tag, "NimBLE host task started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_ERROR_CHECK(asha_web_start(web_retry_asha));
    usb_pcm_queue = xQueueCreate(USB_PCM_QUEUE_FRAMES, sizeof(usb_pcm_frame));
    ESP_ERROR_CHECK(usb_pcm_queue ? ESP_OK : ESP_ERR_NO_MEM);
    uac_device_config_t uac_config = {
        .output_cb = usb_audio_output_cb,
    };
    ESP_ERROR_CHECK(uac_device_init(&uac_config));
    ESP_LOGI(tag, "USB Audio Class speaker ready: 48 kHz stereo, 16-bit; USB PHY switched to OTG");
    ESP_ERROR_CHECK(nimble_port_init());
    assert(ble_uuid_from_str(&mfi_uuid, "7d74f4bd-c74a-4431-862c-cce884371592") == 0);
    assert(ble_uuid_from_str(&asha_uuid, "0000fdf0-0000-1000-8000-00805f9b34fb") == 0);
    assert(ble_uuid_from_str(&psm_uuid, "2d410339-82b6-42aa-b34e-e2e01df8cc1a") == 0);
    assert(ble_uuid_from_str(&readonly_props_uuid, "6333651e-c481-4a3e-9169-7c902aad37bb") == 0);
    assert(ble_uuid_from_str(&audio_control_uuid, "f0d4de7e-4a88-476c-9d9f-1937b0996cc0") == 0);
    assert(ble_uuid_from_str(&audio_status_uuid, "38663f1a-e711-4cac-b641-326b56404837") == 0);
    assert(ble_uuid_from_str(&volume_uuid, "00e4ca9e-ab14-41e4-8823-f9e70c7e91df") == 0);
    ble_hs_cfg.reset_cb = blecent_on_reset;
    ble_hs_cfg.sync_cb = blecent_on_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
    ble_hs_cfg.sm_bonding = 1;
    /* NO_IO cannot complete MITM authentication; retain encrypted SC bonding. */
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_sc = 1;
    ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_NO_IO;
    /* NimBLE's IDF defaults leave both key-distribution masks at zero.
     * Bonding requires key distribution; for LE Secure Connections, enable
     * identity key distribution in both directions. */
    ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ESP_LOGI(tag, "SMP config: bonding=%u SC=%u MITM=%u IO=%u key_dist=ID/(ENC|ID); NO_IO uses Just Works",
             ble_hs_cfg.sm_bonding, ble_hs_cfg.sm_sc, ble_hs_cfg.sm_mitm,
             ble_hs_cfg.sm_io_cap);
#if CONFIG_BT_NIMBLE_GAP_SERVICE
    ESP_ERROR_CHECK(ble_svc_gap_device_name_set("asha-s3-usb"));
#endif
    int coc_pool_rc = os_mempool_init(&asha_coc_mempool, ASHA_COC_BUF_COUNT, ASHA_COC_MTU, asha_coc_mem, "asha_coc_pool");
    ESP_ERROR_CHECK(coc_pool_rc);
    coc_pool_rc = os_mbuf_pool_init(&asha_coc_mbuf_pool, &asha_coc_mempool, ASHA_COC_MTU, ASHA_COC_BUF_COUNT);
    ESP_ERROR_CHECK(coc_pool_rc);
    ble_store_config_init();
    nimble_port_freertos_init(blecent_host_task);
}
