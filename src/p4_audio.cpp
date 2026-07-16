// p4_audio.cpp — see p4_audio.h. ES8311 (I2C ctrl + I2S STD data) via the
// esp_codec_dev component; the I2S channel uses the NEW driver so
// esp_codec_dev_read() handles format plumbing. The XIAO keeps its legacy-i2s
// PDM path untouched.
#if defined(CAMERA_MODEL_ESP32P4)

#include <Arduino.h>
#include "p4_audio.h"
#include "camera_p4.h"          // p4I2cBus(): shared I2C bus (camera SCCB + codec)
#include "driver/i2s_std.h"
#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"
#include "es8311_codec.h"

// NANO wiring per the SCHEMATIC netlist (the wiki pin table is wrong):
// U10 ES8311  MCLK(2)=GPIO13  SCLK(6)=GPIO12  LRCK(8)=GPIO10
//             ASDOUT(7)=GPIO11 (mic -> P4)   DSDIN(9)=GPIO9 (P4 -> codec)
// NS4150B speaker-amp enable = GPIO53 (via R71), for future playback.
#define P4_I2S_MCLK  GPIO_NUM_13
#define P4_I2S_SCLK  GPIO_NUM_12
#define P4_I2S_LRCK  GPIO_NUM_10
#define P4_I2S_DSDIN GPIO_NUM_9    // P4 -> codec (playback, unused for now)
#define P4_I2S_ASDOUT GPIO_NUM_11  // codec -> P4 (mic capture)
#define P4_AUDIO_RATE 16000        // fleet audio format: 16 kHz 16-bit mono

static i2s_chan_handle_t s_rx = NULL;
static i2s_chan_handle_t s_tx = NULL;
static esp_codec_dev_handle_t s_dev = NULL;

bool p4AudioInit() {
  if (s_dev) return true;
  if (!p4I2cBus()) {
    Serial.println("[audio] no I2C bus (camera init failed?) - no codec");
    return false;
  }

  // I2S STD channel pair (RX for the mic; TX reserved for the NS4150B speaker).
  i2s_chan_config_t ch = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  if (i2s_new_channel(&ch, &s_tx, &s_rx) != ESP_OK) {
    Serial.println("[audio] i2s_new_channel failed");
    return false;
  }
  i2s_std_config_t std_cfg = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(P4_AUDIO_RATE),
      .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                      I2S_SLOT_MODE_MONO),
      .gpio_cfg = {
          .mclk = P4_I2S_MCLK,
          .bclk = P4_I2S_SCLK,
          .ws   = P4_I2S_LRCK,
          .dout = P4_I2S_DSDIN,
          .din  = P4_I2S_ASDOUT,
          .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
      },
  };
  std_cfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256; // 4.096 MHz MCLK
  if (i2s_channel_init_std_mode(s_tx, &std_cfg) != ESP_OK ||
      i2s_channel_init_std_mode(s_rx, &std_cfg) != ESP_OK) {
    Serial.println("[audio] i2s std init failed");
    return false;
  }

  // Codec control over the shared I2C bus.
  audio_codec_i2c_cfg_t i2c_cfg = {};
  i2c_cfg.addr = ES8311_CODEC_DEFAULT_ADDR;
  i2c_cfg.bus_handle = p4I2cBus();
  const audio_codec_ctrl_if_t *ctrl = audio_codec_new_i2c_ctrl(&i2c_cfg);
  if (!ctrl) { Serial.println("[audio] codec i2c ctrl failed (no ES8311 on bus?)"); return false; }

  audio_codec_i2s_cfg_t i2s_cfg = {};
  i2s_cfg.rx_handle = s_rx;
  i2s_cfg.tx_handle = s_tx;
  const audio_codec_data_if_t *data = audio_codec_new_i2s_data(&i2s_cfg);
  const audio_codec_gpio_if_t *gpio = audio_codec_new_gpio();

  es8311_codec_cfg_t es_cfg = {};
  es_cfg.ctrl_if = ctrl;
  es_cfg.gpio_if = gpio;
  es_cfg.codec_mode = ESP_CODEC_DEV_WORK_MODE_ADC; // capture only for now
  es_cfg.pa_pin = -1;          // NS4150B enable unused until playback lands
  es_cfg.master_mode = false;  // P4 is the I2S master, codec is slave
  es_cfg.use_mclk = true;      // MCLK is wired (GPIO13 -> U10 pin 2, schematic)
  const audio_codec_if_t *codec = es8311_codec_new(&es_cfg);
  if (!codec) { Serial.println("[audio] es8311_codec_new failed"); return false; }

  esp_codec_dev_cfg_t dev_cfg = {};
  dev_cfg.dev_type = ESP_CODEC_DEV_TYPE_IN;
  dev_cfg.codec_if = codec;
  dev_cfg.data_if = data;
  s_dev = esp_codec_dev_new(&dev_cfg);
  if (!s_dev) { Serial.println("[audio] esp_codec_dev_new failed"); return false; }

  esp_codec_dev_sample_info_t fs = {};
  fs.bits_per_sample = 16;
  fs.channel = 1;
  fs.sample_rate = P4_AUDIO_RATE;
  if (esp_codec_dev_open(s_dev, &fs) != ESP_CODEC_DEV_OK) {
    Serial.println("[audio] codec open failed");
    s_dev = NULL;
    return false;
  }
  esp_codec_dev_set_in_gain(s_dev, 30.0f); // sane analog default; slider adjusts

  // Boot self-test: discard the codec's startup ramp, then log RMS + peak for
  // three consecutive 100ms windows. A live mic in a room shows nonzero RMS
  // and clear peaks on any noise; flat zeros = the capture path is dead.
  {
    static int16_t probe[1600];
    for (int w = 0; w < 6; w++) {
      if (esp_codec_dev_read(s_dev, probe, sizeof(probe)) != ESP_CODEC_DEV_OK) {
        Serial.println("[audio] probe read failed");
        break;
      }
      if (w < 3) continue; // first 300ms: codec ramp/mute settle
      uint64_t acc = 0;
      int16_t peak = 0;
      for (size_t i = 0; i < 1600; i++) {
        acc += (int32_t)probe[i] * probe[i];
        int16_t a = probe[i] < 0 ? -probe[i] : probe[i];
        if (a > peak) peak = a;
      }
      Serial.printf("[audio] probe win%d: RMS=%u peak=%d\n",
                    w - 2, (unsigned)sqrtf((float)(acc / 1600)), peak);
    }
  }
  return true;
}

bool p4AudioRead(void *dst, size_t len, size_t *bytesRead) {
  if (!s_dev) { if (bytesRead) *bytesRead = 0; return false; }
  // esp_codec_dev_read blocks until the full buffer is filled (DMA-paced),
  // matching the XIAO's i2s_read(portMAX_DELAY) contract.
  bool ok = esp_codec_dev_read(s_dev, dst, (int)len) == ESP_CODEC_DEV_OK;
  if (bytesRead) *bytesRead = ok ? len : 0;
  return ok;
}

void p4AudioSetGainDb(float db) {
  if (s_dev) esp_codec_dev_set_in_gain(s_dev, db);
}

#endif // CAMERA_MODEL_ESP32P4
