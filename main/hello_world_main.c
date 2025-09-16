/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_chip_info.h"
#include "esp_rom_efuse.h"
#include "freertos/task.h"
#include "hal/spi_types.h"
#include "soc/gpio_num.h"
#include "soc/soc_caps.h"
#include "tinyusb.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define PIN GPIO_NUM_48
#define LEDS 74
#define BRIGHTNESS 3
#define PERIOD 3000

void generate_byte(uint8_t target, uint8_t data[3]) {
  // 0b110 = 1, 0b100 = 0
  // also trans to MSB
  memset(data, 0, 3);
  data[0] |= (target & 0b10000000) ? 0b11000000 : 0b10000000;
  data[0] |= (target & 0b01000000) ? 0b00011000 : 0b00010000;
  data[0] |= (target & 0b00100000) ? 0b00000011 : 0b00000010;
  data[1] |= (target & 0b00010000) ? 0b01100000 : 0b01000000;
  data[1] |= (target & 0b00001000) ? 0b00001101 : 0b00001001;
  data[2] |= (target & 0b00000100) ? 0b10000000 : 0b00000000;
  data[2] |= (target & 0b00000010) ? 0b00110000 : 0b00100000;
  data[2] |= (target & 0b00000001) ? 0b00000110 : 0b00000100;
}

void generate_color(uint8_t r, uint8_t g, uint8_t b, uint8_t data[9]) {
  generate_byte(g, data);
  generate_byte(r, data + 3);
  generate_byte(b, data + 6);
}

void generate_all(uint8_t r, uint8_t g, uint8_t b, uint8_t *data) {
  for (int i = 0; i < LEDS; i++) {
    generate_color(r, g, b, data + i * 9);
  }
}

// Interface counter
enum interface_count {
#if CFG_TUD_MIDI
  ITF_NUM_MIDI = 0,
  ITF_NUM_MIDI_STREAMING,
#endif
  ITF_COUNT
};

// USB Endpoint numbers
enum usb_endpoints {
  // Available USB Endpoints: 5 IN/OUT EPs and 1 IN EP
  EP_EMPTY = 0,
#if CFG_TUD_MIDI
  EPNUM_MIDI,
#endif
};

#define TUSB_DESCRIPTOR_TOTAL_LEN                                              \
  (TUD_CONFIG_DESC_LEN + CFG_TUD_MIDI * TUD_MIDI_DESC_LEN)

/**
 * @brief String descriptor
 */
static const char *s_str_desc[5] = {
    // array of pointer to string descriptors
    (char[]){0x09, 0x04}, // 0: is supported language is English (0x0409)
    "Piano Prompter",     // 1: Manufacturer
    "Piano Prompter",     // 2: Product
    "123456",             // 3: Serials, should use chip ID
    "MIDI device",        // 4: MIDI
};

/**
 * @brief Configuration descriptor
 *
 * This is a simple configuration descriptor that defines 1 configuration and a
 * MIDI interface
 */
static const uint8_t s_midi_cfg_desc[] = {
    // Configuration number, interface count, string index, total length,
    // attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, ITF_COUNT, 0, TUSB_DESCRIPTOR_TOTAL_LEN, 0, 500),

    // Interface number, string index, EP Out & EP In address, EP size
    TUD_MIDI_DESCRIPTOR(ITF_NUM_MIDI, 4, EPNUM_MIDI, (0x80 | EPNUM_MIDI), 64),
};

#if (TUD_OPT_HIGH_SPEED)
/**
 * @brief High Speed configuration descriptor
 *
 * This is a simple configuration descriptor that defines 1 configuration and a
 * MIDI interface
 */
static const uint8_t s_midi_hs_cfg_desc[] = {
    // Configuration number, interface count, string index, total length,
    // attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, ITF_COUNT, 0, TUSB_DESCRIPTOR_TOTAL_LEN, 0, 100),

    // Interface number, string index, EP Out & EP In address, EP size
    TUD_MIDI_DESCRIPTOR(ITF_NUM_MIDI, 4, EPNUM_MIDI, (0x80 | EPNUM_MIDI), 512),
};
#endif // TUD_OPT_HIGH_SPEED

// LEDs
static uint8_t led_data[LEDS * 9 + 1];
static uint8_t led_state[LEDS];
static spi_device_handle_t led_spi;
static spi_transaction_t led_transcation = {
    .length = sizeof(led_data) * 8 + 8,
    .tx_buffer = led_data,
    .rx_buffer = NULL,
    .user = (void *)0,
};
esp_err_t led_set(uint8_t r, uint8_t g, uint8_t b, uint8_t index) {
  printf("LED[%d]: r(%d) g(%d) b(%d)\n", index, r, g, b);
  led_state[index] =
      ((r ? 1 : 0) << 0) | ((g ? 1 : 0) << 1) | ((b ? 1 : 0) << 2);
  generate_color(r, g, b, led_data + index * 9);
  return spi_device_polling_transmit(led_spi, &led_transcation);
}

esp_err_t led_all_off(void) {
  printf("LED: off\n");
  memset(led_state, 0, sizeof(led_state));
  generate_all(0, 0, 0, led_data);
  return spi_device_polling_transmit(led_spi, &led_transcation);
}

void app_main(void) {
  tinyusb_config_t const tusb_cfg = {
    .device_descriptor = NULL, // If device_descriptor is NULL,
                               // tinyusb_driver_install() will use Kconfig
    .string_descriptor = s_str_desc,
    .string_descriptor_count = sizeof(s_str_desc) / sizeof(s_str_desc[0]),
    .external_phy = false,
#if (TUD_OPT_HIGH_SPEED)
    .fs_configuration_descriptor =
        s_midi_cfg_desc, // HID configuration descriptor for full-speed and
                         // high-speed are the same
    .hs_configuration_descriptor = s_midi_hs_cfg_desc,
    .qualifier_descriptor = NULL,
#else
    .configuration_descriptor = s_midi_cfg_desc,
#endif // TUD_OPT_HIGH_SPEED
  };
  ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

  ESP_ERROR_CHECK(gpio_set_direction(GPIO_NUM_11, GPIO_MODE_OUTPUT));
  ESP_ERROR_CHECK(gpio_set_direction(PIN, GPIO_MODE_OUTPUT));

  ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_11, 1));
  esp_err_t ret;
  spi_bus_config_t buscfg = {
      .miso_io_num = -1,
      .mosi_io_num = PIN,
      .sclk_io_num = -1,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = SOC_SPI_MAXIMUM_BUFFER_SIZE,
  };
  spi_device_interface_config_t devcfg = {
      .clock_speed_hz =
          2400 *
          1000,  // 2.4 MHz, 3 master bits per slave bit, 3 bytes per color
      .mode = 0, // SPI mode 0
      .spics_io_num = -1, // CS pin
      .queue_size = 100, // We want to be able to queue 7 transactions at a time
  };

  ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
  ESP_ERROR_CHECK(ret);

  ret = spi_bus_add_device(SPI2_HOST, &devcfg, &led_spi);
  ESP_ERROR_CHECK(ret);

  gpio_dump_io_configuration(stdout, (1ULL << 11) | (1ULL << PIN));

  generate_all(0, 0, 0, led_data);
  ret = spi_device_polling_transmit(led_spi, &led_transcation);
  ESP_ERROR_CHECK(ret);

  // while (1) {
  //   for (uint8_t i = 1; i <= LEDS; i++) {
  //     led_set(BRIGHT, 0, 0, i - 1);
  //     led_set(0, 0, 0, i);
  //     vTaskDelay(100 / portTICK_PERIOD_MS);
  //   }
  // }

  uint8_t packet[4];
  uint8_t brightness = BRIGHTNESS;
  while (1) {
    if (tud_midi_available()) {
      if (tud_midi_packet_read(packet)) {
        printf("MIDI: %02x %02x %02x %02x\n", packet[0], packet[1], packet[2],
               packet[3]);
        uint8_t header = packet[0];
        uint8_t cin = header & 0x0f;
        if (cin == 0x8 || cin == 0x9) {
          uint8_t channel = packet[1] & 0x0f;
          uint8_t note = packet[2] - 21;
          uint8_t led = 88 - note;

          if (channel == 3) {
            // turn off the LED
            if (note == 88) {
              // all LEDs
              led_all_off();
            } else {
              led_set(0, 0, 0, led);
            }
            continue;
          }

          uint8_t vel = packet[3];
          if ((packet[1] & 0xf0) == 0x80 || (vel == 0)) {
            // note off
            led_set(0, 0, 0, led);
          } else if ((packet[1] & 0xf0) == 0x90) {
            // note on
            // channel 0 for red, channel 1 for green, channel 2 for blue,
            // channel 3 for off

            if (channel == 0) {
              led_set(brightness, 0, 0, led);
            } else if (channel == 1) {
              led_set(0, brightness, 0, led);
            } else if (channel == 2) {
              led_set(0, 0, brightness, led);
            }
          }
        } else if (cin == 0xb) {
          // control change
          if (packet[2] == 7) {
            // volume as brightness
            brightness = 255 * pow(packet[3] / 127., 2);
            printf("LED: brightness %d\n", brightness);
            for (uint8_t i = 0; i < LEDS; i++) {
              if (led_state[i]) {
                generate_color(brightness * ((led_state[i] & 0b001) >> 0),
                               brightness * ((led_state[i] & 0b010) >> 1),
                               brightness * ((led_state[i] & 0b100) >> 2),
                               led_data + i * 9);
              }
            }
            spi_device_polling_transmit(led_spi, &led_transcation);
          }
        }
      }
    }
  }
  vTaskDelay(1);
}
