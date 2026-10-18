# USB-to-ASHA Audio Walkthrough

This document describes this project's USB audio transmitter. It is not the
generic ESP-IDF BLE CoC test example: the firmware receives USB Audio Class
speaker PCM, negotiates the ASHA service with a hearing device, and sends
encoded audio over an ASHA L2CAP CoC.

## Hardware and host setup

- Use an ESP32-S3 board whose native USB OTG port supports USB device mode.
  Connect that data port to a USB host (for example, a computer); the board's
  USB-to-UART connector is not the audio interface.
- The host sees a USB speaker endpoint configured for 48 kHz, stereo,
  signed 16-bit PCM. Select it as the playback output and play audio.
- The peer must be an ASHA-compatible hearing device supporting the 17-byte
  ASHA properties layout, G.722 at 16 kHz, and LE CoC. No external GPIO wiring
  is used for the audio or BLE link.
- Wi-Fi connectivity is only needed for the authenticated diagnostics page, but
  valid Wi-Fi and dashboard credentials must be present for firmware startup.

For clean setup, use ESP-IDF 6.1.0, create `main/wifi_credentials.h` from
`main/wifi_credentials.h.example`, configure the Wi-Fi and dashboard values,
then run `idf.py set-target esp32s3`, `idf.py reconfigure`, and `idf.py build`.
The component manager resolves `espressif/usb_device_uac` from the manifest and
lock file. BLE uses ESP-IDF's `bt` component; no local ESP-IDF example path is
needed.

## Audio path

`app_main()` initializes NVS, the optional Wi-Fi diagnostics service, a
three-frame PCM queue, the USB Audio device, and NimBLE. The USB UAC output
callback in `main/main.c`:

1. Reads signed 16-bit little-endian stereo samples and averages left/right to
   mono.
2. Runs the FIR downsampler from 48 kHz to 16 kHz.
3. Collects 320 samples (20 ms) per frame. The queue is bounded and replaces
   stale audio on overrun to limit latency.

After the ASHA Start status is accepted, the audio task consumes one frame per
20 ms, substitutes silence when the queue is empty, encodes each frame as
160-byte G.722, and adds the ASHA sequence byte to form a 161-byte SDU. It sends
the SDU on the negotiated LE CoC and rebases its schedule after delays instead
of sending catch-up bursts. A Stop command is sent when streaming ends.

## BLE discovery, bonding, and ASHA setup

The NimBLE host scans passively for the ASHA/MFi advertisement. It can also
recognize a previously bonded peer from the NVS bond list, resolving the peer's
rotating BLE private address (RPA) with its stored IRK. On a candidate:

1. The firmware connects and discovers the ASHA FDF0 service.
2. It initiates link security. Bonding and LE Secure Connections are enabled;
   MITM is disabled because the ESP32 has no input/output for passkey
   authentication. First-time pairing therefore uses Just Works; there is no
   PIN prompt on the board. Keep the hearing device powered and in pairing mode
   when creating the initial bond.
3. Once the link is encrypted, it discovers the ASHA characteristics, reads
   and validates the 17-byte ReadOnlyProperties value, checks G.722/16 kHz and
   LE CoC support, and enables AudioStatusPoint notifications.
4. It reads the peer's LE_PSM_OUT and 17-byte ReadOnlyProperties, accepts
   protocol version fields 0 and 1, validates G.722/16 kHz and CoC support,
   establishes the L2CAP CoC, sends the ASHA Start command with configured
   volume attenuation, and waits for the status notification before streaming.

Setup has a 30-second watchdog. Failed procedures terminate the connection and
resume scanning. Bonds persist across restarts in NVS. The repeated-pairing
handler removes a conflicting stale bond only when NimBLE or the peer explicitly
reports a key mismatch / repeat-pairing request, then retries bonding. A normal
successful reconnect, including one using a rotating RPA, reuses the stored bond
without pairing again.

## Diagnostics

`asha_web.c` serves `/`, `/api/status`, and `/api/retry` over HTTP Basic
Authentication. Status includes the current stage, USB audio reception, BLE
scan information, and a bounded event log. The IP address is printed in the
serial log after Wi-Fi connects. See the project `README.md` for credential
setup and security limitations: HTTP does not encrypt either credentials or
diagnostic data. Pairing diagnostics include the raw SMP result code, whether
the link became encrypted/bonded, and the BLE encryption failure code. The
security callback snapshot is not proof of completed bonding: the BLE encryption
event reports whether encryption actually succeeded. A `0x208` encryption status
is an HCI connection supervision timeout. The web log also reports each stored
bond identity and whether an advertisement matched it or resolved as an RPA.
The ReadOnlyProperties value and exact length are included in the web event log
to diagnose differences between hearing-device firmware versions.

## Main implementation files

- `main/main.c`: USB PCM reception, resampling, NimBLE scan/security, ASHA GATT
  and CoC setup, G.722 streaming, and reconnect behavior.
- `main/asha_protocol.c` / `main/asha_protocol.h`: ASHA properties parsing and
  capability validation.
- `main/asha_web.c`: Wi-Fi diagnostics page and authenticated status/retry APIs.
- `main/Kconfig.projbuild`: configurable ASHA Start volume attenuation.
- `main/idf_component.yml`: managed USB Audio component dependency.
