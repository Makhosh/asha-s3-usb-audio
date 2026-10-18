#pragma once

#include "esp_err.h"
#include <stdbool.h>

typedef int (*asha_web_retry_cb_t)(void);

esp_err_t asha_web_start(asha_web_retry_cb_t retry_cb);
void asha_web_set_status(const char *stage, const char *detail);
void asha_web_add_event(const char *event);
void asha_web_set_ble_scan_report(const char *report);
void asha_web_set_usb_audio_received(bool received);

