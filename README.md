# USB Audio to ASHA Hearing Device (ESP32-S3)

This project enumerates as a USB Audio Class speaker, receives stereo PCM from
a USB host, and streams audio to an ASHA-compatible hearing device over BLE
L2CAP Connection-Oriented Channels (CoC). The current audio path is:

```text
USB host: 48 kHz, stereo, signed 16-bit PCM
  -> mono mix and FIR downsampling to 16 kHz
  -> 320-sample / 20 ms frames
  -> G.722 encoding (160 bytes per frame)
  -> ASHA SDU (sequence byte + encoded frame) over BLE CoC
```

The firmware is intended for an ESP32-S3 and currently validates the ASHA
capabilities it needs (G.722 at 16 kHz and LE CoC). It is an independent project
and is not affiliated with or endorsed by hearing-device manufacturers.

## Requirements

- ESP-IDF **6.1.0** and its supported Windows or Linux development environment.
- An **ESP32-S3** board with its native USB OTG connector available for USB
  device operation. A USB-to-UART programming connector is not the audio port.
- A USB host that can send audio to a USB speaker, such as a computer.
- An ASHA-compatible hearing device. The current discovery/status text refers
  to the Cochlear Kanso; other devices must advertise and implement the ASHA
  characteristics and capabilities this firmware validates.
- A 2.4 GHz Wi-Fi network is used only for diagnostics. Valid Wi-Fi and
  dashboard credentials must still be configured for firmware startup; the
  board can continue operating without joining the network.

No external GPIO wiring is required. Connect the board's native USB OTG data
port to the USB host. Power/program the board using the connector and method
specified by its manufacturer; board connector and power arrangements vary.

## Fresh setup and build

Run these commands from an ESP-IDF 6.1.0 terminal in the project directory.
On the first build, ESP-IDF Component Manager downloads the declared managed
dependencies; the project does not depend on an ESP-IDF examples directory on
the local machine.

1. Create the local credentials file from the template, then edit it:

   ```powershell
   Copy-Item .\main\wifi_credentials.h.example .\main\wifi_credentials.h
   ```

   Set `ASHA_WIFI_SSID`, `ASHA_WIFI_PASSWORD`, and a unique
   `ASHA_WEB_PASSWORD`. The username must be visible ASCII, 1-32 characters
   without spaces or `:`; the password must be printable ASCII, 12-64 characters.
   The local header is ignored by Git. Do not commit it.

2. Configure and build for the ESP32-S3:

   ```powershell
   idf.py set-target esp32s3
   idf.py reconfigure
   idf.py build
   ```

3. Flash and open the serial monitor, replacing `COMx` with the board's port:

   ```powershell
   idf.py -p COMx flash
   ```

The managed USB Audio component is declared in `main/idf_component.yml` and
locked in `dependencies.lock`. NimBLE is provided by ESP-IDF's built-in `bt`
component; no example-only NimBLE helper is required.

## Project files

- `main/main.c`: USB audio capture, BLE discovery/bonding, ASHA setup, and
  streaming.
- `main/asha_protocol.c`: ASHA properties parsing and capability checks.
- `main/asha_web.c`: Wi-Fi diagnostics page and authenticated status/retry APIs.
- `main/g722/`: G.722 encoder used by the audio path.
- `main/wifi_credentials.h.example`: safe template for local credentials.
- `tutorial/coc_blecent_walkthrough.md`: deeper explanation of the audio and
  BLE implementation.

## Run the audio stream

1. Connect the board's native USB OTG port to the computer. Select the new USB
   speaker/output device on the computer and play audio.
   The USB interface is configured for 48 kHz, stereo, 16-bit PCM.
2. Put the hearing device on, powered, and in its Bluetooth pairing mode for
   first-time bonding. Keep it close to the ESP32-S3 during setup.
3. The firmware scans for the MFi advertisement used by the ASHA device or
   resolves its rotating BLE private address (RPA) against a stored bond IRK.
   It connects, discovers the ASHA service, and starts encrypted LE Secure
   Connections bonding when needed. It uses Just Works
   because the ESP32 has no pairing input/output; there is no PIN to enter on
   the board. Once the ASHA handshake succeeds, USB audio is sent; if no USB
   audio is arriving, the stream sends silence.
4. Check serial logs for USB PCM reception, ASHA connection/handshake, and
   `ASHA USB audio SDUs sent` messages. The stream volume attenuation defaults
   to -80 dB; adjust `ASHA_START_VOLUME_DB` under the project configuration if
   required.

The BLE bond is stored in NVS and reused on later starts. The device retries
scanning after a disconnect or failed setup. If the peer reports that its
stored encryption key is missing, the firmware removes the conflicting bond
and retries pairing instead of repeatedly failing with a stale key. Rotating
private advertising addresses are resolved against stored peer IRKs so they
still match the bond after a restart.

## Diagnostics and security

When configured with Wi-Fi credentials, the serial log reports the board's IP.
Open `http://<device-ip>/` on the same trusted Wi-Fi and sign in with
`ASHA_WEB_USERNAME` and `ASHA_WEB_PASSWORD`. The status page exposes USB/BLE
state and logs; `/api/status` and `/api/retry` are also authenticated.

The page uses plain HTTP. Basic Authentication does not encrypt the credentials
or diagnostic data in transit. Use it only on a trusted local network; do not
port-forward the device or expose it to the internet. Credentials are embedded
in firmware and are not protected against firmware extraction.

## Troubleshooting

- **No USB audio:** verify the native OTG data connector is connected to the
  host, select the board as the host's audio output, and start playback. A
  charge-only cable or USB-to-UART connector will not carry UAC audio.
- **No ASHA connection:** ensure the hearing device is on and in pairing mode
  for initial bonding. Check serial scan reports and keep it near the board. If
  the device was reset or re-paired separately, watch the serial log for a stale
  bond being removed and allow the new pairing attempt to complete.
- **BLE encryption timeout (`0x208`):** this is a connection supervision
  timeout, not a pairing-rejected code. Check the web event log for the stored
  bond identity and the MFi advertisement's `bond`, `RPA`, and address type
  fields to see whether the advertising peer matches the stored bond.
- **No dashboard response:** confirm Wi-Fi credentials, use the current IP from
  the serial log, and sign in with the configured dashboard credentials.
- **Firmware stops during startup:** replace the template dashboard password
  with a valid 12-64 character printable ASCII password before building.
