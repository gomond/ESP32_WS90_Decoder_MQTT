#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_event.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "mqtt_client.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_wifi.h"

#define WIFI_SSID       CONFIG_WS90_WIFI_SSID
#define WIFI_PASS       CONFIG_WS90_WIFI_PASS
#define MQTT_BROKER_URI CONFIG_WS90_MQTT_BROKER_URI
#define MQTT_USER       CONFIG_WS90_MQTT_USERNAME
#define MQTT_PASS       CONFIG_WS90_MQTT_PASSWORD
#define MQTT_CLIENT_ID  CONFIG_WS90_MQTT_CLIENT_ID
#define MQTT_STATE_TOPIC CONFIG_WS90_MQTT_STATE_TOPIC
#define WS90_REQUIRE_EXPECTED_ID CONFIG_WS90_REQUIRE_EXPECTED_ID
#define WS90_RADIO_DIAGNOSTICS   CONFIG_WS90_RADIO_DIAGNOSTICS
#define WS90_DIAG_INTERVAL_MS    CONFIG_WS90_DIAG_INTERVAL_MS

#ifndef CONFIG_WS90_PIN_MISO
#define CONFIG_WS90_PIN_MISO 19
#endif
#ifndef CONFIG_WS90_PIN_MOSI
#define CONFIG_WS90_PIN_MOSI 27
#endif
#ifndef CONFIG_WS90_PIN_SCK
#define CONFIG_WS90_PIN_SCK 5
#endif
#ifndef CONFIG_WS90_PIN_CS
#define CONFIG_WS90_PIN_CS 18
#endif
#ifndef CONFIG_WS90_PIN_RST
#define CONFIG_WS90_PIN_RST 14
#endif

#ifndef CONFIG_WS90_DISPLAY_ENABLE
#define CONFIG_WS90_DISPLAY_ENABLE 1
#endif
#ifndef CONFIG_WS90_DISPLAY_I2C_PORT
#define CONFIG_WS90_DISPLAY_I2C_PORT 0
#endif
#ifndef CONFIG_WS90_DISPLAY_SDA
#define CONFIG_WS90_DISPLAY_SDA 21
#endif
#ifndef CONFIG_WS90_DISPLAY_SCL
#define CONFIG_WS90_DISPLAY_SCL 22
#endif
#ifndef CONFIG_WS90_DISPLAY_ADDR
#define CONFIG_WS90_DISPLAY_ADDR 0x3C
#endif

#define PIN_MISO                CONFIG_WS90_PIN_MISO
#define PIN_MOSI                CONFIG_WS90_PIN_MOSI
#define PIN_SCK                 CONFIG_WS90_PIN_SCK
#define PIN_CS                  CONFIG_WS90_PIN_CS
#define PIN_RST                 CONFIG_WS90_PIN_RST

#define DISPLAY_I2C_PORT        CONFIG_WS90_DISPLAY_I2C_PORT
#define DISPLAY_SDA             CONFIG_WS90_DISPLAY_SDA
#define DISPLAY_SCL             CONFIG_WS90_DISPLAY_SCL
#define DISPLAY_ADDR            CONFIG_WS90_DISPLAY_ADDR

#ifndef CONFIG_WS90_MQTT_CLIENT_ID
#undef MQTT_CLIENT_ID
#define MQTT_CLIENT_ID ""
#endif

#define RFM69_REG_FIFO          0x00
#define RFM69_REG_OPMODE        0x01
#define RFM69_REG_DATAMODUL     0x02
#define RFM69_REG_BITRATEMSB    0x03
#define RFM69_REG_BITRATELSB    0x04
#define RFM69_REG_FDEVMSB       0x05
#define RFM69_REG_FDEVLSB       0x06
#define RFM69_REG_FRFMSB        0x07
#define RFM69_REG_FRFMID        0x08
#define RFM69_REG_FRFLSB        0x09
#define RFM69_REG_RXBW          0x19
#define RFM69_REG_AFCBW         0x1A
#define RFM69_REG_RSSIVALUE     0x24
#define RFM69_REG_DIOMAPPING1   0x25
#define RFM69_REG_IRQFLAGS1     0x27
#define RFM69_REG_IRQFLAGS2     0x28
#define RFM69_REG_RSSITHRESH    0x29
#define RFM69_REG_SYNCCONFIG    0x2E
#define RFM69_REG_SYNCVALUE1    0x2F
#define RFM69_REG_SYNCVALUE2    0x30
#define RFM69_REG_SYNCVALUE3    0x31
#define RFM69_REG_SYNCVALUE4    0x32
#define RFM69_REG_PACKETCONFIG1 0x37
#define RFM69_REG_PAYLOADLENGTH 0x38
#define RFM69_REG_FIFOTHRESH    0x3C
#define RFM69_REG_PACKETCONFIG2 0x3D

#define RFM69_MODE_STDBY       0x04
#define RFM69_MODE_RX          0x10
#define RFM69_OPMODE_MASK      0x1C

#define RADIO_BITRATE_BPS      17241u
#define RADIO_FDEV_HZ          33500u
#define RADIO_CENTER_HZ        433920000u
#define RFM_SPI_CLOCK_HZ       4000000u

#define WS90_FRAME_BYTES       32u
#define RFM_CAPTURE_BYTES      32u
#define WS90_EXPECTED_ID       0x00C0E4u
#define HEARTBEAT_IDLE_MS      10000u

#define OLED_WIDTH             128u
#define OLED_HEIGHT            64u
#define OLED_PAGES             (OLED_HEIGHT / 8u)
#define OLED_FB_SIZE           (OLED_WIDTH * OLED_PAGES)
#define OLED_GLYPH_W           3u
#define OLED_GLYPH_H           5u
#define OLED_FONT_SCALE        2u

static const char *TAG = "WS90_MQTT";
static EventGroupHandle_t wifi_event_group;
static const int WIFI_CONNECTED_BIT = BIT0;

static spi_device_handle_t rfm_spi;
static esp_mqtt_client_handle_t mqtt_client;
static bool mqtt_connected = false;

#if CONFIG_WS90_DISPLAY_ENABLE
static bool oled_ready = false;
static uint8_t oled_fb[OLED_FB_SIZE];
#endif

static const char *mqtt_connack_code_to_str(int code) {
    switch (code) {
        case 0:
            return "Connection Accepted";
        case 1:
            return "Unacceptable Protocol Version";
        case 2:
            return "Identifier Rejected";
        case 3:
            return "Server Unavailable";
        case 4:
            return "Bad Username or Password";
        case 5:
            return "Not Authorized";
        default:
            return "Unknown";
    }
}

static void mqtt_clear_discovery_topic(const char *topic) {
    if (!mqtt_connected || mqtt_client == NULL) {
        return;
    }
    esp_mqtt_client_publish(mqtt_client, topic, "", 0, 1, 1);
}

static void mqtt_publish_discovery_sensor(const char *topic,
                                          const char *name,
                                          const char *unique_id,
                                          const char *unit,
                                          const char *device_class,
                                          const char *state_class,
                                          const char *value_template) {
    if (!mqtt_connected || mqtt_client == NULL) {
        return;
    }

    char payload[768];
    snprintf(payload,
             sizeof(payload),
             "{"
             "\"name\":\"%s\","
             "\"uniq_id\":\"%s\","
             "\"stat_t\":\"%s\","
             "\"unit_of_meas\":\"%s\","
             "\"dev_cla\":\"%s\","
             "\"state_class\":\"%s\","
             "\"val_tpl\":\"%s\","
             "\"dev\":{"
             "\"ids\":[\"ws90_weather\"],"
             "\"name\":\"WS90\","
             "\"mdl\":\"WS90_Weather_MQTT\","
             "\"mf\":\"Custom\""
             "}"
             "}",
             name,
             unique_id,
             MQTT_STATE_TOPIC,
             unit,
             device_class,
             state_class,
             value_template);

    esp_mqtt_client_publish(mqtt_client, topic, payload, 0, 1, 1);
}

static void mqtt_publish_discovery_text(const char *topic,
                                        const char *name,
                                        const char *unique_id,
                                        const char *value_template) {
    if (!mqtt_connected || mqtt_client == NULL) {
        return;
    }

    char payload[768];
    snprintf(payload,
             sizeof(payload),
             "{"
             "\"name\":\"%s\"," 
             "\"uniq_id\":\"%s\"," 
             "\"stat_t\":\"%s\"," 
             "\"val_tpl\":\"%s\"," 
             "\"dev\":{"
             "\"ids\":[\"ws90_weather\"],"
             "\"name\":\"WS90\"," 
             "\"mdl\":\"WS90_Weather_MQTT\"," 
             "\"mf\":\"Custom\""
             "}"
             "}",
             name,
             unique_id,
             MQTT_STATE_TOPIC,
             value_template);

    esp_mqtt_client_publish(mqtt_client, topic, payload, 0, 1, 1);
}

static void mqtt_publish_discovery_sensor_simple(const char *topic,
                                                 const char *name,
                                                 const char *unique_id,
                                                 const char *unit,
                                                 const char *value_template) {
    if (!mqtt_connected || mqtt_client == NULL) {
        return;
    }

    char payload[768];
    snprintf(payload,
             sizeof(payload),
             "{"
             "\"name\":\"%s\","
             "\"uniq_id\":\"%s\","
             "\"stat_t\":\"%s\","
             "\"unit_of_meas\":\"%s\","
             "\"val_tpl\":\"%s\","
             "\"dev\":{"
             "\"ids\":[\"ws90_weather\"],"
             "\"name\":\"WS90\","
             "\"mdl\":\"WS90_Weather_MQTT\","
             "\"mf\":\"Custom\""
             "}"
             "}",
             name,
             unique_id,
             MQTT_STATE_TOPIC,
             unit,
             value_template);

    esp_mqtt_client_publish(mqtt_client, topic, payload, 0, 1, 1);
}

static inline esp_err_t rfm_write(uint8_t addr, uint8_t value) {
    uint8_t tx[2] = { (uint8_t)(addr | 0x80u), value };
    spi_transaction_t t = {0};
    t.length = 16;
    t.tx_buffer = tx;
    return spi_device_transmit(rfm_spi, &t);
}

static inline uint8_t rfm_read(uint8_t addr) {
    uint8_t tx[2] = { (uint8_t)(addr & 0x7Fu), 0x00 };
    uint8_t rx[2] = { 0, 0 };
    spi_transaction_t t = {0};
    t.length = 16;
    t.tx_buffer = tx;
    t.rx_buffer = rx;
    if (spi_device_transmit(rfm_spi, &t) != ESP_OK) {
        return 0;
    }
    return rx[1];
}

static void rfm_read_fifo(uint8_t *buffer, size_t len) {
    for (size_t i = 0; i < len; i++) {
        buffer[i] = rfm_read(RFM69_REG_FIFO);
    }
}

static void rfm_set_mode(uint8_t mode) {
    uint8_t op = rfm_read(RFM69_REG_OPMODE);
    op = (uint8_t)((op & (uint8_t)~RFM69_OPMODE_MASK) | (mode & RFM69_OPMODE_MASK));
    rfm_write(RFM69_REG_OPMODE, op);

    for (uint32_t i = 0; i < 20; i++) {
        uint8_t irq1 = rfm_read(RFM69_REG_IRQFLAGS1);
        if (irq1 & 0x80u) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

static void rfm_write_bitrate(uint32_t bitrate_bps) {
    uint32_t bitrate_reg = (32000000u + (bitrate_bps / 2u)) / bitrate_bps;
    if (bitrate_reg > 0xFFFFu) {
        bitrate_reg = 0xFFFFu;
    }
    rfm_write(RFM69_REG_BITRATEMSB, (uint8_t)((bitrate_reg >> 8) & 0xFFu));
    rfm_write(RFM69_REG_BITRATELSB, (uint8_t)(bitrate_reg & 0xFFu));
}

static void rfm_write_frequency_hz(uint32_t freq_hz) {
    uint64_t frf = (((uint64_t)freq_hz) << 19) / 32000000ull;
    rfm_write(RFM69_REG_FRFMSB, (uint8_t)((frf >> 16) & 0xFFu));
    rfm_write(RFM69_REG_FRFMID, (uint8_t)((frf >> 8) & 0xFFu));
    rfm_write(RFM69_REG_FRFLSB, (uint8_t)(frf & 0xFFu));
}

static void rfm_set_sync_profile(uint8_t profile) {
    switch (profile) {
        case 0:
            rfm_write(RFM69_REG_SYNCCONFIG, 0x99);
            rfm_write(RFM69_REG_SYNCVALUE1, 0xAA);
            rfm_write(RFM69_REG_SYNCVALUE2, 0xAA);
            rfm_write(RFM69_REG_SYNCVALUE3, 0x2D);
            rfm_write(RFM69_REG_SYNCVALUE4, 0xD4);
            break;
        case 1:
            rfm_write(RFM69_REG_SYNCCONFIG, 0x91);
            rfm_write(RFM69_REG_SYNCVALUE1, 0xAA);
            rfm_write(RFM69_REG_SYNCVALUE2, 0x2D);
            rfm_write(RFM69_REG_SYNCVALUE3, 0xD4);
            break;
        default:
            rfm_write(RFM69_REG_SYNCCONFIG, 0x88);
            rfm_write(RFM69_REG_SYNCVALUE1, 0x2D);
            rfm_write(RFM69_REG_SYNCVALUE2, 0xD4);
            break;
    }
}

static void rfm_reset(void) {
    gpio_set_level(PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
}

static bool rfm_spi_sanity_check(void) {
    uint8_t reg0 = rfm_read(RFM69_REG_OPMODE);
    uint8_t reg1 = rfm_read(RFM69_REG_DATAMODUL);
    uint8_t reg2 = rfm_read(RFM69_REG_SYNCCONFIG);

    bool all_zero = (reg0 == 0x00u) && (reg1 == 0x00u) && (reg2 == 0x00u);
    bool all_ff = (reg0 == 0xFFu) && (reg1 == 0xFFu) && (reg2 == 0xFFu);

    uint8_t original_sync1 = rfm_read(RFM69_REG_SYNCVALUE1);
    const uint8_t pattern_a = 0x5Au;
    const uint8_t pattern_b = 0xA5u;

    rfm_write(RFM69_REG_SYNCVALUE1, pattern_a);
    uint8_t read_a = rfm_read(RFM69_REG_SYNCVALUE1);
    rfm_write(RFM69_REG_SYNCVALUE1, pattern_b);
    uint8_t read_b = rfm_read(RFM69_REG_SYNCVALUE1);
    rfm_write(RFM69_REG_SYNCVALUE1, original_sync1);

    bool write_ok = (read_a == pattern_a) && (read_b == pattern_b);
    bool sanity_ok = write_ok && !all_zero && !all_ff;

    ESP_LOGI(TAG,
             "rfm69 spi probe: op=0x%02X datamodul=0x%02X sync=0x%02X sync1_orig=0x%02X wrA=0x%02X wrB=0x%02X result=%s",
             reg0,
             reg1,
             reg2,
             original_sync1,
             read_a,
             read_b,
             sanity_ok ? "PASS" : "FAIL");

    if (!sanity_ok) {
        if (all_zero) {
            ESP_LOGE(TAG, "rfm69 spi probe fail: reads are all 0x00 (likely no SPI response or held low)");
        }
        if (all_ff) {
            ESP_LOGE(TAG, "rfm69 spi probe fail: reads are all 0xFF (likely no SPI response or floating MISO)");
        }
        if (!write_ok) {
            ESP_LOGE(TAG, "rfm69 spi probe fail: write/readback mismatch (wrA=0x%02X wrB=0x%02X)", read_a, read_b);
        }
    }

    return sanity_ok;
}

static void rfm_init(void) {
    rfm_set_mode(RFM69_MODE_STDBY);
    vTaskDelay(pdMS_TO_TICKS(5));

    rfm_write(RFM69_REG_DATAMODUL, 0x00);
    rfm_write_bitrate(RADIO_BITRATE_BPS);

    uint32_t fdev_reg = (RADIO_FDEV_HZ + 30u) / 61u;
    if (fdev_reg > 0x3FFFu) {
        fdev_reg = 0x3FFFu;
    }
    rfm_write(RFM69_REG_FDEVMSB, (uint8_t)((fdev_reg >> 8) & 0x3Fu));
    rfm_write(RFM69_REG_FDEVLSB, (uint8_t)(fdev_reg & 0xFFu));

    rfm_write_frequency_hz(RADIO_CENTER_HZ);
    rfm_write(RFM69_REG_RXBW, 0b10000010);
    rfm_write(RFM69_REG_AFCBW, 0b10000010);

    rfm_set_sync_profile(0);

    rfm_write(RFM69_REG_PACKETCONFIG1, 0x00);
    rfm_write(RFM69_REG_PAYLOADLENGTH, RFM_CAPTURE_BYTES);
    rfm_write(RFM69_REG_FIFOTHRESH, 0x8F);
    rfm_write(RFM69_REG_PACKETCONFIG2, 0x02);

    rfm_write(RFM69_REG_RSSITHRESH, 0xB4);
    rfm_write(RFM69_REG_DIOMAPPING1, 0x00);
    rfm_set_mode(RFM69_MODE_RX);
}

static void rfm_log_diagnostics(uint32_t packet_count, uint32_t decode_count) {
    uint8_t opmode = rfm_read(RFM69_REG_OPMODE);
    uint8_t datamodul = rfm_read(RFM69_REG_DATAMODUL);
    uint8_t syncconfig = rfm_read(RFM69_REG_SYNCCONFIG);
    uint8_t irq1 = rfm_read(RFM69_REG_IRQFLAGS1);
    uint8_t irq2 = rfm_read(RFM69_REG_IRQFLAGS2);
    uint8_t packet_cfg1 = rfm_read(RFM69_REG_PACKETCONFIG1);
    uint8_t payload_len = rfm_read(RFM69_REG_PAYLOADLENGTH);
    uint8_t rssi_raw = rfm_read(RFM69_REG_RSSIVALUE);
    int rssi_dbm = -((int)rssi_raw / 2);

    ESP_LOGI(TAG,
             "diag: op=0x%02X data=0x%02X sync=0x%02X irq1=0x%02X irq2=0x%02X pkt=0x%02X len=%u rssi=%d dBm pkt=%u ok=%u",
             opmode,
             datamodul,
             syncconfig,
             irq1,
             irq2,
             packet_cfg1,
             payload_len,
             rssi_dbm,
             packet_count,
             decode_count);
}

static uint8_t ws90_crc8(const uint8_t *data, size_t len, uint8_t poly, uint8_t init) {
    uint8_t crc = init;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80u) {
                crc = (uint8_t)((crc << 1) ^ poly);
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

static uint8_t ws90_add_bytes(const uint8_t *data, size_t len) {
    uint16_t sum = 0;
    for (size_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return (uint8_t)sum;
}

static uint8_t ws90_reverse8(uint8_t x) {
    x = (uint8_t)(((x & 0xF0u) >> 4) | ((x & 0x0Fu) << 4));
    x = (uint8_t)(((x & 0xCCu) >> 2) | ((x & 0x33u) << 2));
    x = (uint8_t)(((x & 0xAAu) >> 1) | ((x & 0x55u) << 1));
    return x;
}

static const char *wind_dir_to_compass_16(int wind_dir_deg) {
    static const char *dirs[16] = {
        "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
        "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"
    };

    int deg = wind_dir_deg % 360;
    if (deg < 0) {
        deg += 360;
    }
    int idx = (deg + 11) / 22;
    idx %= 16;
    return dirs[idx];
}

#if CONFIG_WS90_DISPLAY_ENABLE
static esp_err_t oled_send(uint8_t control, const uint8_t *data, size_t len) {
    uint8_t packet[17];
    while (len > 0) {
        size_t chunk = (len > 16u) ? 16u : len;
        packet[0] = control;
        memcpy(&packet[1], data, chunk);
        esp_err_t err = i2c_master_write_to_device(DISPLAY_I2C_PORT,
                                                   DISPLAY_ADDR,
                                                   packet,
                                                   chunk + 1u,
                                                   pdMS_TO_TICKS(100));
        if (err != ESP_OK) {
            return err;
        }
        data += chunk;
        len -= chunk;
    }
    return ESP_OK;
}

static esp_err_t oled_cmd(uint8_t cmd) {
    return oled_send(0x00u, &cmd, 1u);
}

static void oled_clear(void) {
    memset(oled_fb, 0, sizeof(oled_fb));
}

static void oled_draw_pixel(uint8_t x, uint8_t y) {
    if (x >= OLED_WIDTH || y >= OLED_HEIGHT) {
        return;
    }
    oled_fb[x + ((y / 8u) * OLED_WIDTH)] |= (uint8_t)(1u << (y % 8u));
}

static bool glyph_3x5(char c, uint8_t rows[5]) {
    switch (c) {
        case '0': rows[0]=0x7; rows[1]=0x5; rows[2]=0x5; rows[3]=0x5; rows[4]=0x7; return true;
        case '1': rows[0]=0x2; rows[1]=0x6; rows[2]=0x2; rows[3]=0x2; rows[4]=0x7; return true;
        case '2': rows[0]=0x7; rows[1]=0x1; rows[2]=0x7; rows[3]=0x4; rows[4]=0x7; return true;
        case '3': rows[0]=0x7; rows[1]=0x1; rows[2]=0x7; rows[3]=0x1; rows[4]=0x7; return true;
        case '4': rows[0]=0x5; rows[1]=0x5; rows[2]=0x7; rows[3]=0x1; rows[4]=0x1; return true;
        case '5': rows[0]=0x7; rows[1]=0x4; rows[2]=0x7; rows[3]=0x1; rows[4]=0x7; return true;
        case '6': rows[0]=0x7; rows[1]=0x4; rows[2]=0x7; rows[3]=0x5; rows[4]=0x7; return true;
        case '7': rows[0]=0x7; rows[1]=0x1; rows[2]=0x1; rows[3]=0x1; rows[4]=0x1; return true;
        case '8': rows[0]=0x7; rows[1]=0x5; rows[2]=0x7; rows[3]=0x5; rows[4]=0x7; return true;
        case '9': rows[0]=0x7; rows[1]=0x5; rows[2]=0x7; rows[3]=0x1; rows[4]=0x7; return true;
        case 'C': rows[0]=0x7; rows[1]=0x4; rows[2]=0x4; rows[3]=0x4; rows[4]=0x7; return true;
        case 'D': rows[0]=0x6; rows[1]=0x5; rows[2]=0x5; rows[3]=0x5; rows[4]=0x6; return true;
        case 'E': rows[0]=0x7; rows[1]=0x4; rows[2]=0x7; rows[3]=0x4; rows[4]=0x7; return true;
        case 'H': rows[0]=0x5; rows[1]=0x5; rows[2]=0x7; rows[3]=0x5; rows[4]=0x5; return true;
        case 'M': rows[0]=0x5; rows[1]=0x7; rows[2]=0x7; rows[3]=0x5; rows[4]=0x5; return true;
        case 'N': rows[0]=0x5; rows[1]=0x7; rows[2]=0x7; rows[3]=0x7; rows[4]=0x5; return true;
        case 'S': rows[0]=0x7; rows[1]=0x4; rows[2]=0x7; rows[3]=0x1; rows[4]=0x7; return true;
        case 'T': rows[0]=0x7; rows[1]=0x2; rows[2]=0x2; rows[3]=0x2; rows[4]=0x2; return true;
        case 'W': rows[0]=0x5; rows[1]=0x5; rows[2]=0x7; rows[3]=0x7; rows[4]=0x5; return true;
        case '.': rows[0]=0x0; rows[1]=0x0; rows[2]=0x0; rows[3]=0x0; rows[4]=0x2; return true;
        case ':': rows[0]=0x0; rows[1]=0x2; rows[2]=0x0; rows[3]=0x2; rows[4]=0x0; return true;
        case '%': rows[0]=0x5; rows[1]=0x1; rows[2]=0x2; rows[3]=0x4; rows[4]=0x5; return true;
        case '/': rows[0]=0x1; rows[1]=0x1; rows[2]=0x2; rows[3]=0x4; rows[4]=0x4; return true;
        case '-': rows[0]=0x0; rows[1]=0x0; rows[2]=0x7; rows[3]=0x0; rows[4]=0x0; return true;
        case ' ': rows[0]=0x0; rows[1]=0x0; rows[2]=0x0; rows[3]=0x0; rows[4]=0x0; return true;
        default: return false;
    }
}

static void oled_draw_char3x5_scaled(uint8_t x, uint8_t y, char c, uint8_t scale) {
    uint8_t rows[5];
    if (!glyph_3x5(c, rows)) {
        c = ' ';
        (void)glyph_3x5(c, rows);
    }

    for (uint8_t row = 0; row < OLED_GLYPH_H; row++) {
        for (uint8_t col = 0; col < OLED_GLYPH_W; col++) {
            if (rows[row] & (uint8_t)(1u << (2u - col))) {
                for (uint8_t sy = 0; sy < scale; sy++) {
                    for (uint8_t sx = 0; sx < scale; sx++) {
                        oled_draw_pixel((uint8_t)(x + (col * scale) + sx),
                                        (uint8_t)(y + (row * scale) + sy));
                    }
                }
            }
        }
    }
}

static void oled_draw_text3x5_scaled(uint8_t x, uint8_t y, const char *text, uint8_t scale) {
    uint8_t char_w = (uint8_t)(OLED_GLYPH_W * scale);
    uint8_t step = (uint8_t)(char_w + scale);
    while (*text != '\0' && x <= (uint8_t)(OLED_WIDTH - char_w)) {
        oled_draw_char3x5_scaled(x, y, *text, scale);
        x = (uint8_t)(x + step);
        text++;
    }
}

static esp_err_t oled_flush(void) {
    for (uint8_t page = 0; page < OLED_PAGES; page++) {
        ESP_RETURN_ON_ERROR(oled_cmd((uint8_t)(0xB0u | page)), TAG, "oled page set failed");
        ESP_RETURN_ON_ERROR(oled_cmd(0x00u), TAG, "oled col low failed");
        ESP_RETURN_ON_ERROR(oled_cmd(0x10u), TAG, "oled col high failed");
        ESP_RETURN_ON_ERROR(oled_send(0x40u, &oled_fb[page * OLED_WIDTH], OLED_WIDTH), TAG, "oled data failed");
    }
    return ESP_OK;
}

static esp_err_t ws90_display_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = DISPLAY_SDA,
        .scl_io_num = DISPLAY_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
    };

    ESP_RETURN_ON_ERROR(i2c_param_config(DISPLAY_I2C_PORT, &conf), TAG, "i2c param config failed");

    esp_err_t install_err = i2c_driver_install(DISPLAY_I2C_PORT, conf.mode, 0, 0, 0);
    if (install_err != ESP_OK && install_err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "i2c driver install failed: %s", esp_err_to_name(install_err));
        return install_err;
    }

    const uint8_t init_seq[] = {
        0xAE, 0x20, 0x00, 0xB0, 0xC8, 0x00, 0x10, 0x40,
        0x81, 0x7F, 0xA1, 0xA6, 0xA8, 0x3F, 0xD3, 0x00,
        0xD5, 0x80, 0xD9, 0xF1, 0xDA, 0x12, 0xDB, 0x40,
        0x8D, 0x14, 0xAF
    };

    for (size_t i = 0; i < sizeof(init_seq); i++) {
        ESP_RETURN_ON_ERROR(oled_cmd(init_seq[i]), TAG, "oled init cmd failed");
    }

    oled_clear();
    ESP_RETURN_ON_ERROR(oled_flush(), TAG, "oled initial flush failed");
    oled_ready = true;
    ESP_LOGI(TAG,
             "SSD1306 ready on I2C port %d, SDA=%d, SCL=%d, addr=0x%02X",
             DISPLAY_I2C_PORT,
             DISPLAY_SDA,
             DISPLAY_SCL,
             DISPLAY_ADDR);
    return ESP_OK;
}

static void ws90_display_update(float temp_c,
                                int humidity,
                                int wind_dir_deg,
                                const char *wind_dir_compass,
                                float wind_speed_m_s) {
    if (!oled_ready) {
        return;
    }

    char line1[22];
    char line2[22];
    char line3[22];
    char line4[22];

    snprintf(line1, sizeof(line1), "T%.1fC", (double)temp_c);
    snprintf(line2, sizeof(line2), "H%d%%", humidity);
    snprintf(line3, sizeof(line3), "WD%d%s", wind_dir_deg, wind_dir_compass);
    snprintf(line4, sizeof(line4), "WS%.1f", (double)wind_speed_m_s);

    oled_clear();
    oled_draw_text3x5_scaled(0, 0, line1, OLED_FONT_SCALE);
    oled_draw_text3x5_scaled(0, 16, line2, OLED_FONT_SCALE);
    oled_draw_text3x5_scaled(0, 32, line3, OLED_FONT_SCALE);
    oled_draw_text3x5_scaled(0, 48, line4, OLED_FONT_SCALE);

    esp_err_t err = oled_flush();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "oled flush failed: %s", esp_err_to_name(err));
    }
}
#else
static esp_err_t ws90_display_init(void) {
    return ESP_OK;
}

static void ws90_display_update(float temp_c,
                                int humidity,
                                int wind_dir_deg,
                                const char *wind_dir_compass,
                                float wind_speed_m_s) {
    (void)temp_c;
    (void)humidity;
    (void)wind_dir_deg;
    (void)wind_dir_compass;
    (void)wind_speed_m_s;
}
#endif

static void shift_left_bits_len(const uint8_t *in, uint8_t *out, uint32_t len, uint8_t bits) {
    if (bits == 0u) {
        memcpy(out, in, len);
        return;
    }
    for (uint32_t i = 0; i < len; i++) {
        uint8_t next = (i + 1u < len) ? in[i + 1u] : 0u;
        out[i] = (uint8_t)((in[i] << bits) | (next >> (8u - bits)));
    }
}

static void shift_right_bits_len(const uint8_t *in, uint8_t *out, uint32_t len, uint8_t bits) {
    if (bits == 0u) {
        memcpy(out, in, len);
        return;
    }
    for (int i = (int)len - 1; i >= 0; i--) {
        uint8_t prev = (i > 0) ? in[i - 1] : 0u;
        out[i] = (uint8_t)((in[i] >> bits) | (prev << (8u - bits)));
    }
}

static void mqtt_publish_discovery(void) {
    if (!mqtt_connected || mqtt_client == NULL) {
        return;
    }

    mqtt_publish_discovery_sensor("homeassistant/sensor/temperature/config",
                                  "Temperature",
                                  "ws90_weather_temp",
                                  "°C",
                                  "temperature",
                                  "measurement",
                                  "{{ value_json.temperature_c }}");

    mqtt_publish_discovery_sensor("homeassistant/sensor/humidity/config",
                                  "Humidity",
                                  "ws90_weather_humidity",
                                  "%",
                                  "humidity",
                                  "measurement",
                                  "{{ value_json.humidity }}");

    mqtt_publish_discovery_sensor("homeassistant/sensor/wind_avg/config",
                                  "Wind Average",
                                  "ws90_weather_wind_avg",
                                  "m/s",
                                  "wind_speed",
                                  "measurement",
                                  "{{ value_json.wind_avg_m_s }}");

    mqtt_publish_discovery_sensor("homeassistant/sensor/wind_max/config",
                                  "Wind Gust",
                                  "ws90_weather_wind_max",
                                  "m/s",
                                  "wind_speed",
                                  "measurement",
                                  "{{ value_json.wind_max_m_s }}");

    mqtt_publish_discovery_sensor_simple("homeassistant/sensor/wind_dir/config",
                                         "Wind Direction",
                                         "ws90_weather_wind_dir",
                                         "°",
                                         "{{ value_json.wind_dir_deg }}");

    mqtt_publish_discovery_sensor_simple("homeassistant/sensor/wind_dir_compass/config",
                                         "Wind Direction Compass",
                                         "ws90_weather_wind_dir_compass",
                                         "",
                                         "{{ value_json.wind_dir_compass }}");

    mqtt_publish_discovery_sensor("homeassistant/sensor/rain/config",
                                  "Rain",
                                  "ws90_weather_rain",
                                  "mm",
                                  "precipitation",
                                  "measurement",
                                  "{{ value_json.rain_mm }}");

    mqtt_publish_discovery_sensor_simple("homeassistant/sensor/uv/config",
                                         "UV Index",
                                         "ws90_weather_uv",
                                         "",
                                         "{{ value_json.uv_index }}");

    mqtt_publish_discovery_sensor("homeassistant/sensor/light/config",
                                  "Illuminance",
                                  "ws90_weather_light",
                                  "lx",
                                  "illuminance",
                                  "measurement",
                                  "{{ value_json.light_lux }}");

    mqtt_publish_discovery_sensor("homeassistant/sensor/battery/config",
                                  "Battery",
                                  "ws90_weather_battery",
                                  "%",
                                  "battery",
                                  "measurement",
                                  "{{ (value_json.battery_level * 100) | round(0) }}");

    mqtt_publish_discovery_sensor("homeassistant/sensor/battery_mv/config",
                                  "Battery Voltage",
                                  "ws90_weather_battery_mv",
                                  "mV",
                                  "voltage",
                                  "measurement",
                                  "{{ value_json.battery_mv }}");

    mqtt_publish_discovery_sensor("homeassistant/sensor/supercap_v/config",
                                  "Supercap Voltage",
                                  "ws90_weather_supercap_v",
                                  "V",
                                  "voltage",
                                  "measurement",
                                  "{{ value_json.supercap_v }}");

    mqtt_publish_discovery_sensor("homeassistant/sensor/rain_start/config",
                                  "Rain Start",
                                  "ws90_weather_rain_start",
                                  "",
                                  "",
                                  "measurement",
                                  "{{ value_json.rain_start }}");

    mqtt_publish_discovery_sensor("homeassistant/sensor/firmware/config",
                                  "Firmware",
                                  "ws90_weather_firmware",
                                  "",
                                  "",
                                  "measurement",
                                  "{{ value_json.firmware }}");

    mqtt_publish_discovery_sensor("homeassistant/sensor/crc8/config",
                                  "CRC8",
                                  "ws90_weather_crc8",
                                  "",
                                  "",
                                  "measurement",
                                  "{{ value_json.crc8 }}");

    mqtt_publish_discovery_sensor("homeassistant/sensor/chk_sum/config",
                                  "Checksum Sum",
                                  "ws90_weather_chk_sum",
                                  "",
                                  "",
                                  "measurement",
                                  "{{ value_json.chk_sum }}");

    mqtt_publish_discovery_sensor("homeassistant/sensor/chk_byte/config",
                                  "Checksum Byte",
                                  "ws90_weather_chk_byte",
                                  "",
                                  "",
                                  "measurement",
                                  "{{ value_json.chk_byte }}");

    mqtt_publish_discovery_text("homeassistant/text/flags/config",
                                "Flags",
                                "ws90_weather_flags",
                                "{{ value_json.flags }}");

    mqtt_publish_discovery_text("homeassistant/text/raw_hex/config",
                                "Raw Frame",
                                "ws90_weather_raw_hex",
                                "{{ value_json.raw_hex }}");
}

static void mqtt_cleanup_legacy_discovery(void) {
    if (!mqtt_connected || mqtt_client == NULL) {
        return;
    }

    mqtt_clear_discovery_topic("homeassistant/sensor/ws90_wind_dir_compass/config");
    mqtt_clear_discovery_topic("homeassistant/sensor/ws90_flags/config");
    mqtt_clear_discovery_topic("homeassistant/sensor/ws90_raw_hex/config");
    mqtt_clear_discovery_topic("homeassistant/text/ws90_wind_dir_compass/config");
}

static void mqtt_publish_state(const char *json) {
    if (mqtt_client == NULL) {
        return;
    }
    int msg_id = esp_mqtt_client_publish(mqtt_client, MQTT_STATE_TOPIC, json, 0, 1, 0);
    if (msg_id < 0) {
        ESP_LOGW(TAG, "MQTT publish enqueue failed for state topic");
    }
}

static bool ws90_decode_and_publish(const uint8_t *b) {
    if (b[0] != 0x90u) {
        return false;
    }

    int id = (b[1] << 16) | (b[2] << 8) | b[3];
#if CONFIG_WS90_REQUIRE_EXPECTED_ID
    if (id != (int)WS90_EXPECTED_ID) {
        return false;
    }
#endif

    uint8_t crc = ws90_crc8(b, 31, 0x31, 0x00);
    uint8_t chk = ws90_add_bytes(b, 31);
    if ((crc != 0u) || (chk != b[31])) {
        return false;
    }

    int light_raw = (b[4] << 8) | b[5];
    float light_lux = light_raw * 10.0f;
    int battery_mv = b[6] * 20;
    int battery_lvl = (battery_mv < 1400) ? 0 : ((battery_mv - 1400) / 16);
    if (battery_lvl > 100) {
        battery_lvl = 100;
    }
    int flags = b[7];
    int temp_raw = ((b[7] & 0x03) << 8) | b[8];
    float temp_c = (temp_raw - 400) * 0.1f;
    int humidity = b[9];
    int wind_avg = ((b[7] & 0x10) << 4) | b[10];
    int wind_dir = ((b[7] & 0x20) << 3) | b[11];
    int wind_max = ((b[7] & 0x40) << 2) | b[12];
    int uv_index = b[13];
    int rain_raw = (b[19] << 8) | b[20];
    int rain_start = (b[16] & 0x10) >> 4;
    int supercap_v = (b[21] & 0x3f);
    int firmware = b[29];
    const char *wind_dir_compass = wind_dir_to_compass_16(wind_dir);

    ws90_display_update(temp_c,
                        humidity,
                        wind_dir,
                        wind_dir_compass,
                        (float)(wind_avg * 0.1f));

    char raw_hex[(WS90_FRAME_BYTES * 2u) + 1u];
    for (size_t i = 0; i < WS90_FRAME_BYTES; i++) {
        snprintf(&raw_hex[i * 2u], 3u, "%02X", b[i]);
    }

    char extra[31];
    snprintf(extra, sizeof(extra), "%02X%02X%02X%02X%02X------%02X%02X%02X%02X%02X%02X%02X",
             b[14], b[15], b[16], b[17], b[18], b[22], b[23], b[24], b[25], b[26], b[27], b[28]);

    char json[768];
    snprintf(json, sizeof(json),
             "{\"model\":\"Ecowitt WS90\",\"id\":\"%06X\",\"battery_level\":%.3f,\"battery_mv\":%d,\"temperature_c\":%.1f,\"humidity\":%d,\"wind_dir_deg\":%d,\"wind_dir_compass\":\"%s\",\"wind_avg_m_s\":%.1f,\"wind_max_m_s\":%.1f,\"uv_index\":%.1f,\"light_lux\":%.1f,\"flags\":\"%02X\",\"rain_mm\":%.1f,\"rain_start\":%d,\"supercap_v\":%.1f,\"firmware\":%d,\"extra\":\"%s\",\"raw_hex\":\"%s\",\"crc8\":%u,\"chk_sum\":%u,\"chk_byte\":%u,\"mic\":\"CRC\"}",
             id,
             (double)(battery_lvl * 0.01f),
             battery_mv,
             (double)temp_c,
             humidity,
             wind_dir,
             wind_dir_compass,
             (double)(wind_avg * 0.1f),
             (double)(wind_max * 0.1f),
             (double)(uv_index * 0.1f),
             (double)light_lux,
             flags,
             (double)(rain_raw * 0.1f),
             rain_start,
             (double)(supercap_v * 0.1f),
             firmware,
             extra,
             raw_hex,
             crc,
             chk,
             b[31]);

    printf("%s\n", json);
    mqtt_publish_state(json);
    return true;
}

static bool ws90_decode_with_alignment(const uint8_t *raw) {
    uint8_t base[WS90_FRAME_BYTES];
    uint8_t shifted[WS90_FRAME_BYTES];
    uint8_t rotated[WS90_FRAME_BYTES];

    if (ws90_decode_and_publish(raw)) {
        return true;
    }

    for (uint32_t mode = 0; mode < 4u; mode++) {
        for (uint32_t i = 0; i < WS90_FRAME_BYTES; i++) {
            uint8_t v = raw[i];
            if (mode == 1u) {
                v = (uint8_t)~v;
            } else if (mode == 2u) {
                v = ws90_reverse8(v);
            } else if (mode == 3u) {
                v = (uint8_t)~ws90_reverse8(v);
            }
            base[i] = v;
        }

        if (ws90_decode_and_publish(base)) {
            return true;
        }

        for (uint8_t off = 1u; off < WS90_FRAME_BYTES; off++) {
            for (uint8_t i = 0u; i < WS90_FRAME_BYTES; i++) {
                rotated[i] = base[(uint8_t)((off + i) % WS90_FRAME_BYTES)];
            }
            if (ws90_decode_and_publish(rotated)) {
                return true;
            }
        }

        for (uint8_t shift = 1u; shift <= 7u; shift++) {
            shift_left_bits_len(base, shifted, WS90_FRAME_BYTES, shift);
            if (ws90_decode_and_publish(shifted)) {
                return true;
            }

            shift_right_bits_len(base, shifted, WS90_FRAME_BYTES, shift);
            if (ws90_decode_and_publish(shifted)) {
                return true;
            }
        }
    }

    return false;
}

static bool handle_capture_bytes(const uint8_t *capture, uint32_t capture_len) {
    uint8_t transformed[RFM_CAPTURE_BYTES];
    uint8_t shifted[RFM_CAPTURE_BYTES];

    if (capture_len < WS90_FRAME_BYTES) {
        return false;
    }

    for (uint32_t mode = 0; mode < 4u; mode++) {
        for (uint32_t i = 0; i < capture_len; i++) {
            uint8_t v = capture[i];
            if (mode == 1u) {
                v = (uint8_t)~v;
            } else if (mode == 2u) {
                v = ws90_reverse8(v);
            } else if (mode == 3u) {
                v = (uint8_t)~ws90_reverse8(v);
            }
            transformed[i] = v;
        }

        for (uint8_t bit_shift = 0u; bit_shift <= 7u; bit_shift++) {
            shift_left_bits_len(transformed, shifted, capture_len, bit_shift);
            if (ws90_decode_with_alignment(shifted)) {
                return true;
            }

            if (bit_shift != 0u) {
                shift_right_bits_len(transformed, shifted, capture_len, bit_shift);
                if (ws90_decode_with_alignment(shifted)) {
                    return true;
                }
            }
        }
    }

    return ws90_decode_with_alignment(capture);
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    (void)arg;
    (void)event_data;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
        esp_wifi_connect();
        ESP_LOGW(TAG, "Wi-Fi disconnected, retrying");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
        ESP_LOGI(TAG, "Wi-Fi connected");
    }
}

static void wifi_init_sta(void) {
    wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {0};
    snprintf((char *)wifi_config.sta.ssid, sizeof(wifi_config.sta.ssid), "%s", WIFI_SSID);
    snprintf((char *)wifi_config.sta.password, sizeof(wifi_config.sta.password), "%s", WIFI_PASS);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    (void)handler_args;
    (void)base;

    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            mqtt_connected = true;
            ESP_LOGI(TAG, "MQTT connected");
            mqtt_cleanup_legacy_discovery();
            mqtt_publish_discovery();
            break;
        case MQTT_EVENT_DISCONNECTED:
            mqtt_connected = false;
            ESP_LOGW(TAG, "MQTT disconnected");
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT error");
            if (event != NULL && event->error_handle != NULL) {
                esp_mqtt_error_codes_t *err = event->error_handle;
                ESP_LOGE(TAG,
                         "MQTT error details: type=%d transport_sock_errno=%d connect_return_code=%d (%s)",
                         err->error_type,
                         err->esp_transport_sock_errno,
                         err->connect_return_code,
                         mqtt_connack_code_to_str(err->connect_return_code));
            }
            break;
        default:
            break;
    }
}

static void mqtt_start(void) {
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
    };

    if (strlen(MQTT_USER) > 0) {
        mqtt_cfg.credentials.username = MQTT_USER;
        mqtt_cfg.credentials.authentication.password = MQTT_PASS;
    }

    if (strlen(MQTT_CLIENT_ID) > 0) {
        mqtt_cfg.credentials.client_id = MQTT_CLIENT_ID;
    }

    ESP_LOGI(TAG,
             "MQTT init: uri=%s user_set=%s pass_set=%s client_id=%s",
             MQTT_BROKER_URI,
             strlen(MQTT_USER) > 0 ? "yes" : "no",
             strlen(MQTT_PASS) > 0 ? "yes" : "no",
             strlen(MQTT_CLIENT_ID) > 0 ? MQTT_CLIENT_ID : "(auto)");

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    ESP_ERROR_CHECK(esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL));
    ESP_ERROR_CHECK(esp_mqtt_client_start(mqtt_client));
}

static void rfm_spi_init(void) {
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = PIN_MISO,
        .sclk_io_num = PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64,
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = RFM_SPI_CLOCK_HZ,
        .mode = 0,
        .spics_io_num = PIN_CS,
        .queue_size = 1,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &rfm_spi));

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_RST),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
}

static void ws90_radio_task(void *arg) {
    (void)arg;

    rfm_spi_init();
    rfm_reset();

    bool spi_ok = rfm_spi_sanity_check();
    if (!spi_ok) {
        ESP_LOGE(TAG, "RFM69 SPI check failed; continuing for diagnostics");
    }

    rfm_init();

    uint8_t opmode = rfm_read(RFM69_REG_OPMODE);
    uint8_t datamodul = rfm_read(RFM69_REG_DATAMODUL);
    uint8_t syncconfig = rfm_read(RFM69_REG_SYNCCONFIG);

    ESP_LOGI(TAG,
             "WS90 RX packet-mode start: bitrate=%u fdev=%u opmode=0x%02X datamodul=0x%02X sync=0x%02X",
             RADIO_BITRATE_BPS,
             RADIO_FDEV_HZ,
             opmode,
             datamodul,
             syncconfig);

    uint8_t frame[RFM_CAPTURE_BYTES];
    uint32_t heartbeat_ms = (uint32_t)esp_log_timestamp();
    uint32_t last_activity_ms = heartbeat_ms;
    uint32_t last_diag_ms = heartbeat_ms;
    uint32_t packet_count = 0;
    uint32_t decode_count = 0;

    while (true) {
        uint8_t opm = rfm_read(RFM69_REG_OPMODE);
        if ((opm & 0x1Cu) != RFM69_MODE_RX) {
            rfm_set_mode(RFM69_MODE_RX);
        }

        uint8_t irq2 = rfm_read(RFM69_REG_IRQFLAGS2);
        if (irq2 & 0x04u) {
            packet_count++;
            rfm_read_fifo(frame, RFM_CAPTURE_BYTES);
            bool decoded = handle_capture_bytes(frame, RFM_CAPTURE_BYTES);
            if (decoded) {
                decode_count++;
                last_activity_ms = (uint32_t)esp_log_timestamp();
            }
        } else {
            uint32_t now_ms = (uint32_t)esp_log_timestamp();
            if (((now_ms - heartbeat_ms) >= 1000u) && ((now_ms - last_activity_ms) >= HEARTBEAT_IDLE_MS)) {
                ESP_LOGI(TAG, "alive: waiting packets");
                heartbeat_ms = now_ms;
            }

#if CONFIG_WS90_RADIO_DIAGNOSTICS
            if ((now_ms - last_diag_ms) >= (uint32_t)WS90_DIAG_INTERVAL_MS) {
                rfm_log_diagnostics(packet_count, decode_count);
                last_diag_ms = now_ms;
            }
#endif
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void app_main(void) {
    bool wifi_ready = (strlen(WIFI_SSID) > 0) && (strlen(WIFI_PASS) > 0);
    bool mqtt_ready = strlen(MQTT_BROKER_URI) > 0;

    if (!wifi_ready) {
        ESP_LOGW(TAG, "Wi-Fi credentials missing; running radio-only mode. Set WS90_WIFI_SSID and WS90_WIFI_PASS in menuconfig.");
    }

    if (!mqtt_ready) {
        ESP_LOGW(TAG, "MQTT broker URI missing; MQTT publish disabled. Set WS90_MQTT_BROKER_URI in menuconfig.");
    }

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    esp_err_t display_err = ws90_display_init();
    if (display_err != ESP_OK) {
        ESP_LOGW(TAG, "Display init failed, continuing without OLED updates");
    }

    if (wifi_ready) {
        wifi_init_sta();
        xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
        if (mqtt_ready) {
            mqtt_start();
        }
    }

    xTaskCreatePinnedToCore(ws90_radio_task, "ws90_radio", 8192, NULL, 5, NULL, 1);
}
