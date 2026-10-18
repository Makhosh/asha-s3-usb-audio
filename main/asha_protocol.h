#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define ASHA_READ_ONLY_PROPERTIES_SIZE 17

enum {
    ASHA_PROP_OFFSET_VERSION = 0,
    ASHA_PROP_OFFSET_DEVICE_CAPABILITIES = 1,
    ASHA_PROP_OFFSET_FEATURE_MAP = 10,
    ASHA_PROP_OFFSET_RENDER_DELAY = 11,
    ASHA_PROP_OFFSET_CODEC_MASK = 15,
    ASHA_PROP_VERSION_0 = 0,
    ASHA_PROP_VERSION_1 = 1,
    ASHA_PROP_FEATURE_LE_COC = 0x01,
    ASHA_PROP_CODEC_G722_16KHZ = 0x0002
};

typedef struct {
    uint8_t version;
    uint8_t device_capabilities;
    uint8_t feature_map;
    uint16_t render_delay_ms;
    uint16_t codec_mask;
} asha_read_only_properties_t;

bool asha_parse_read_only_properties(const uint8_t *data, size_t len,
                                     asha_read_only_properties_t *properties);
bool asha_read_only_properties_support_g722(const asha_read_only_properties_t *properties);
