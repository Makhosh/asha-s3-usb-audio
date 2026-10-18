#include "asha_protocol.h"

bool asha_parse_read_only_properties(const uint8_t *data, size_t len,
                                     asha_read_only_properties_t *properties)
{
    if (!data || !properties || len != ASHA_READ_ONLY_PROPERTIES_SIZE) {
        return false;
    }
    uint8_t version = data[ASHA_PROP_OFFSET_VERSION];
    if (version != ASHA_PROP_VERSION_0 && version != ASHA_PROP_VERSION_1) {
        return false;
    }

    properties->version = data[ASHA_PROP_OFFSET_VERSION];
    properties->device_capabilities = data[ASHA_PROP_OFFSET_DEVICE_CAPABILITIES];
    properties->feature_map = data[ASHA_PROP_OFFSET_FEATURE_MAP];
    properties->render_delay_ms =
        (uint16_t)data[ASHA_PROP_OFFSET_RENDER_DELAY] |
        ((uint16_t)data[ASHA_PROP_OFFSET_RENDER_DELAY + 1] << 8);
    properties->codec_mask =
        (uint16_t)data[ASHA_PROP_OFFSET_CODEC_MASK] |
        ((uint16_t)data[ASHA_PROP_OFFSET_CODEC_MASK + 1] << 8);
    return true;
}

bool asha_read_only_properties_support_g722(const asha_read_only_properties_t *properties)
{
    return properties &&
           (properties->feature_map & ASHA_PROP_FEATURE_LE_COC) != 0 &&
           (properties->codec_mask & ASHA_PROP_CODEC_G722_16KHZ) != 0;
}
