#include <assert.h>
#include <stddef.h>
#include <stdint.h>

#include "asha_protocol.h"

int main(void)
{
    const uint8_t kanso_properties[ASHA_READ_ONLY_PROPERTIES_SIZE] = {
        0x00, 0x00, 0x89, 0x00, 0x00, 0x00, 0x16, 0xd5, 0x59,
        0x69, 0x01, 0x64, 0x00, 0x00, 0x00, 0x02, 0x00
    };
    const uint8_t valid_properties[ASHA_READ_ONLY_PROPERTIES_SIZE] = {
        0x01, 0x03, 0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde,
        0xf0, 0x01, 0x34, 0x12, 0x56, 0x78, 0x02, 0x00
    };
    asha_read_only_properties_t parsed = {0};

    assert(asha_parse_read_only_properties(kanso_properties,
                                           sizeof(kanso_properties), &parsed));
    assert(parsed.version == ASHA_PROP_VERSION_0);
    assert(parsed.device_capabilities == 0x00);
    assert(parsed.feature_map == ASHA_PROP_FEATURE_LE_COC);
    assert(parsed.render_delay_ms == 100);
    assert(parsed.codec_mask == ASHA_PROP_CODEC_G722_16KHZ);
    assert(asha_read_only_properties_support_g722(&parsed));

    assert(asha_parse_read_only_properties(valid_properties,
                                           sizeof(valid_properties), &parsed));
    assert(parsed.version == ASHA_PROP_VERSION_1);
    assert(parsed.device_capabilities == 0x03);
    assert(parsed.feature_map == ASHA_PROP_FEATURE_LE_COC);
    assert(parsed.render_delay_ms == 0x1234);
    assert(parsed.codec_mask == ASHA_PROP_CODEC_G722_16KHZ);
    assert((parsed.feature_map & ASHA_PROP_FEATURE_LE_COC) != 0);
    assert((parsed.codec_mask & ASHA_PROP_CODEC_G722_16KHZ) != 0);
    assert(asha_read_only_properties_support_g722(&parsed));

    asha_read_only_properties_t unsupported = parsed;
    unsupported.feature_map = 0;
    assert(!asha_read_only_properties_support_g722(&unsupported));
    unsupported = parsed;
    unsupported.codec_mask = 0;
    assert(!asha_read_only_properties_support_g722(&unsupported));
    assert(!asha_read_only_properties_support_g722(NULL));

    assert(!asha_parse_read_only_properties(valid_properties,
                                            sizeof(valid_properties) - 1, &parsed));
    assert(!asha_parse_read_only_properties(valid_properties,
                                            sizeof(valid_properties) + 1, &parsed));

    uint8_t unsupported_version[ASHA_READ_ONLY_PROPERTIES_SIZE];
    for (size_t i = 0; i < sizeof(unsupported_version); ++i) {
        unsupported_version[i] = valid_properties[i];
    }
    unsupported_version[ASHA_PROP_OFFSET_VERSION] = 0x02;
    assert(!asha_parse_read_only_properties(unsupported_version,
                                            sizeof(unsupported_version), &parsed));

    assert(!asha_parse_read_only_properties(NULL, sizeof(valid_properties), &parsed));
    assert(!asha_parse_read_only_properties(valid_properties,
                                            sizeof(valid_properties), NULL));
    return 0;
}
