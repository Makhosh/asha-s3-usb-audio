/*
 * G.722 encoder adapter for the ASHA S3 USB audio path.
 * See the accompanying G.722 and WebRTC license and patent notices.
 */
#include <stdint.h>

#include "g722/g722_enc_dec.h"

namespace {
g722_encode_state_t encoder_state;
constexpr int16_t kTone[20] = {
    0, 1236, 2351, 3236, 3804, 4000, 3804, 3236, 2351, 1236,
    0, -1236, -2351, -3236, -3804, -4000, -3804, -3236, -2351, -1236,
};
constexpr unsigned kSamplesPerFrame = 320;
unsigned tone_phase;
}

extern "C" void asha_test_encoder_reset(void)
{
    g722_encode_init(&encoder_state, 64000, G722_PACKED);
    tone_phase = 0;
}

extern "C" int asha_test_encode_frame(uint8_t sequence, uint8_t packet[161])
{
    int16_t pcm[kSamplesPerFrame];
    packet[0] = sequence;
    for (unsigned i = 0; i < kSamplesPerFrame; ++i) {
        pcm[i] = kTone[tone_phase];
        tone_phase = (tone_phase + 1) % 20;
    }
    return g722_encode(&encoder_state, packet + 1, pcm, kSamplesPerFrame);
}

extern "C" int asha_encode_pcm_frame(uint8_t sequence, const int16_t pcm[kSamplesPerFrame],
                                      uint8_t packet[161])
{
    packet[0] = sequence;
    return g722_encode(&encoder_state, packet + 1, pcm, kSamplesPerFrame);
}
