# ESP32_WS90_Decoder_MQTT (ESP-IDF)

ESP32 packet-mode WS90 decoder with MQTT publishing for Home Assistant.

## Status

- Packet-mode RFM69 receive path ported.
- WS90 decode and JSON formatting ported.
- Wi-Fi and MQTT client integration added.
- Home Assistant MQTT discovery expanded for key WS90 metrics.
- Raw PIO fallback is not ported yet.

## Build

1. Install ESP-IDF (v5.x recommended).
2. In this folder, run:
   - `idf.py set-target esp32`
   - `idf.py menuconfig` (set Wi-Fi/MQTT values)
   - `idf.py build flash monitor`

## Configuration (`idf.py menuconfig`)

Under `WS90 MQTT Configuration`:

- `WS90_WIFI_SSID`
- `WS90_WIFI_PASS`
- `WS90_MQTT_BROKER_URI`
- `WS90_MQTT_USERNAME` (optional)
- `WS90_MQTT_PASSWORD` (optional)
- `WS90_MQTT_CLIENT_ID` (recommended when broker ACLs require fixed client IDs)
- `WS90_MQTT_TLS_SKIP_VERIFY` — skip TLS certificate verification (enable for cloud/subscription brokers via `mqtts://` when no CA certificate is embedded; **insecure**, use only on trusted networks or for development)
- `WS90_MQTT_STATE_TOPIC`
- `WS90_PIN_MISO`
- `WS90_PIN_MOSI`
- `WS90_PIN_SCK`
- `WS90_PIN_CS`
- `WS90_PIN_RST`
- `WS90_DISPLAY_ENABLE`
- `WS90_DISPLAY_I2C_PORT`
- `WS90_DISPLAY_SDA`
- `WS90_DISPLAY_SCL`
- `WS90_DISPLAY_ADDR`

## Connecting to a Cloud / Subscription MQTT Broker

Cloud and subscription-based MQTT brokers (e.g. HiveMQ Cloud, CloudMQTT, or a managed
Eclipse Mosquitto service) typically require:

1. **TLS transport** — set `WS90_MQTT_BROKER_URI` to `mqtts://your-broker-host:8883`
2. **Username and password** — set `WS90_MQTT_USERNAME` and `WS90_MQTT_PASSWORD`
3. **Fixed client ID** — set `WS90_MQTT_CLIENT_ID` (avoids session conflicts on reconnect)

If the broker's TLS certificate hostname does not exactly match the URI (common with
cloud brokers that serve multiple hostnames), enable **`WS90_MQTT_TLS_SKIP_VERIFY`** in
`idf.py menuconfig`. This skips the TLS certificate *common-name* (hostname) check while
still validating the certificate chain against the CA store bundled into the firmware.
For most well-known cloud brokers (e.g. those using Let's Encrypt or DigiCert), the
ESP-IDF default certificate bundle already trusts the issuing CA, so no additional CA
certificate needs to be embedded.

> **Note:** `WS90_MQTT_TLS_SKIP_VERIFY` only bypasses the hostname check.  For brokers
> with self-signed or otherwise untrusted certificates, you must also provide the broker's
> CA certificate via `broker.verification.certificate` in the code, or enable the ESP-IDF
> certificate bundle (`CONFIG_MBEDTLS_CERTIFICATE_BUNDLE`) in SDK configuration.
> Only enable TLS skip verify on networks you trust; for production deployments embed the
> broker's CA certificate instead.
