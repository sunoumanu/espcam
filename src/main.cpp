#include <Arduino.h>
#include <WiFi.h>
#include <esp_camera.h>
#include <WebServer.h>
#include <SD_MMC.h>
#include <FS.h>
#include <driver/i2s.h>

// ===================== WiFi credentials =====================
const char *WIFI_SSID = "PUT YOUR WIFI SSID HERE";
const char *WIFI_PASS = "PUT YOUR WIFI PASSWORD HERE";

// ============== XIAO ESP32S3 Sense camera pins ==============
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  10
#define SIOD_GPIO_NUM  40
#define SIOC_GPIO_NUM  39

#define Y9_GPIO_NUM    48
#define Y8_GPIO_NUM    11
#define Y7_GPIO_NUM    12
#define Y6_GPIO_NUM    14
#define Y5_GPIO_NUM    16
#define Y4_GPIO_NUM    18
#define Y3_GPIO_NUM    17
#define Y2_GPIO_NUM    15
#define VSYNC_GPIO_NUM 38
#define HREF_GPIO_NUM  47
#define PCLK_GPIO_NUM  13

// ======== XIAO ESP32S3 Sense SD card (1-bit SD_MMC) ========
#define SD_MMC_CLK  7
#define SD_MMC_CMD  9
#define SD_MMC_D0   8

// ========= XIAO ESP32S3 Sense PDM microphone ===============
#define PDM_CLK_PIN   42
#define PDM_DATA_PIN  41
#define I2S_PORT      I2S_NUM_0

// ================== Audio settings ==========================
#define AUDIO_SAMPLE_RATE   16000
#define AUDIO_CHANNELS      1
#define AUDIO_BITS          16
#define AUDIO_BLOCK_ALIGN   (AUDIO_CHANNELS * (AUDIO_BITS / 8))  // 2
#define AUDIO_BYTES_PER_SEC (AUDIO_SAMPLE_RATE * AUDIO_BLOCK_ALIGN)  // 32000
#define AUDIO_BUF_SIZE      4096  // bytes per I2S read

// ================== Recording settings ======================
#define MAX_FILE_DURATION     (5 * 60 * 1000UL)          // 5 min per file
#define MAX_FILE_SIZE         (100UL * 1024UL * 1024UL)  // 100 MB per file
#define MAX_INDEX_ENTRIES     72000                       // video + audio entries
#define HEADER_PATCH_INTERVAL 30                          // patch every N video frames

// =============================================================

WebServer server(80);

// ===================== AVI recording ========================

struct AviIndexEntry {
  uint32_t offset;   // offset from movi list start
  uint32_t size;     // chunk data size
  uint8_t  isAudio;  // 0 = video "00dc", 1 = audio "01wb"
};

static File          aviFile;
static bool          sdCardOK       = false;
static bool          micOK          = false;
static bool          isRecording    = false;
static bool          hasAudioStream = false;  // set when header is written
static uint32_t      aviVideoFrames = 0;
static uint32_t      aviAudioBytes  = 0;
static uint32_t      aviIdxCount    = 0;      // total index entries (video + audio)
static uint32_t      aviMoviSize    = 0;      // bytes written inside movi LIST
static uint32_t      aviMaxVideoFrame = 0;
static uint32_t      aviMaxAudioChunk = 0;
static uint32_t      fileStartTime  = 0;
static uint32_t      fileIndex      = 0;
static uint32_t      aviActualFps   = 15;
static uint16_t      aviWidth       = 800;
static uint16_t      aviHeight      = 600;
static AviIndexEntry *aviIdx        = nullptr;
static TaskHandle_t  recTaskHandle  = nullptr;

// Header patch offsets — captured dynamically during aviWriteHeader()
// so we never hardcode byte positions again
static uint32_t OFF_RIFF_SIZE;
static uint32_t OFF_HDRL_SIZE;
static uint32_t OFF_AVIH_USPERFRAME;
static uint32_t OFF_AVIH_MAXBYTES;
static uint32_t OFF_AVIH_TOTALFRAMES;
static uint32_t OFF_AVIH_SUGBUF;
static uint32_t OFF_VSTRH_RATE;
static uint32_t OFF_VSTRH_LENGTH;
static uint32_t OFF_VSTRH_SUGBUF;
static uint32_t OFF_ASTRH_LENGTH;   // audio stream (only valid if hasAudioStream)
static uint32_t OFF_ASTRH_SUGBUF;
static uint32_t OFF_MOVI_SIZE;

// Little-endian helpers
static void writeDword(File &f, uint32_t v) {
  uint8_t b[4] = {(uint8_t)(v), (uint8_t)(v >> 8), (uint8_t)(v >> 16), (uint8_t)(v >> 24)};
  f.write(b, 4);
}

static void writeWord(File &f, uint16_t v) {
  uint8_t b[2] = {(uint8_t)(v), (uint8_t)(v >> 8)};
  f.write(b, 2);
}

static void writeFourCC(File &f, const char *cc) {
  f.write((const uint8_t *)cc, 4);
}

static void patchDword(File &f, uint32_t offset, uint32_t value) {
  uint32_t pos = f.position();
  f.seek(offset);
  writeDword(f, value);
  f.seek(pos);
}

// Write the AVI header with video stream + optional audio stream.
// All patchable field offsets are captured via f.position() — no hardcoding.
static void aviWriteHeader(File &f) {
  hasAudioStream = micOK;
  uint32_t numStreams = hasAudioStream ? 2 : 1;

  // --- RIFF header ---
  writeFourCC(f, "RIFF");
  OFF_RIFF_SIZE = f.position(); writeDword(f, 0);  // placeholder
  writeFourCC(f, "AVI ");

  // --- LIST hdrl ---
  writeFourCC(f, "LIST");
  OFF_HDRL_SIZE = f.position(); writeDword(f, 0);  // placeholder, patch at end of hdrl
  writeFourCC(f, "hdrl");
  uint32_t hdrlContentStart = f.position();

  // --- avih (Main AVI Header) — 56 bytes data ---
  writeFourCC(f, "avih");
  writeDword(f, 56);
  OFF_AVIH_USPERFRAME = f.position(); writeDword(f, 0);  // dwMicroSecPerFrame
  OFF_AVIH_MAXBYTES   = f.position(); writeDword(f, 0);  // dwMaxBytesPerSec
  writeDword(f, 0);                                        // dwPaddingGranularity
  writeDword(f, 0x10);                                     // dwFlags = AVIF_HASINDEX
  OFF_AVIH_TOTALFRAMES = f.position(); writeDword(f, 0);  // dwTotalFrames
  writeDword(f, 0);                                        // dwInitialFrames
  writeDword(f, numStreams);                               // dwStreams
  OFF_AVIH_SUGBUF = f.position(); writeDword(f, 0);       // dwSuggestedBufferSize
  writeDword(f, aviWidth);
  writeDword(f, aviHeight);
  writeDword(f, 0); writeDword(f, 0); writeDword(f, 0); writeDword(f, 0);  // reserved[4]

  // --- LIST strl (video) ---
  writeFourCC(f, "LIST");
  uint32_t vStrlSizePos = f.position(); writeDword(f, 0);  // placeholder
  writeFourCC(f, "strl");
  uint32_t vStrlContentStart = f.position();

  // strh (video stream header) — 56 bytes data
  writeFourCC(f, "strh");
  writeDword(f, 56);
  writeFourCC(f, "vids");
  writeFourCC(f, "MJPG");
  writeDword(f, 0);               // dwFlags
  writeWord(f, 0); writeWord(f, 0);  // wPriority, wLanguage
  writeDword(f, 0);               // dwInitialFrames
  writeDword(f, 1);               // dwScale
  OFF_VSTRH_RATE   = f.position(); writeDword(f, 0);  // dwRate (actual FPS)
  writeDword(f, 0);               // dwStart
  OFF_VSTRH_LENGTH = f.position(); writeDword(f, 0);  // dwLength
  OFF_VSTRH_SUGBUF = f.position(); writeDword(f, 0);  // dwSuggestedBufferSize
  writeDword(f, (uint32_t)-1);    // dwQuality
  writeDword(f, 0);               // dwSampleSize (0 for video)
  writeWord(f, 0); writeWord(f, 0);                   // rcFrame left, top
  writeWord(f, aviWidth); writeWord(f, aviHeight);     // rcFrame right, bottom

  // strf (video format — BITMAPINFOHEADER) — 40 bytes data
  writeFourCC(f, "strf");
  writeDword(f, 40);
  writeDword(f, 40);              // biSize
  writeDword(f, aviWidth);
  writeDword(f, aviHeight);
  writeWord(f, 1);                // biPlanes
  writeWord(f, 24);               // biBitCount
  writeFourCC(f, "MJPG");         // biCompression
  writeDword(f, aviWidth * aviHeight * 3);  // biSizeImage
  writeDword(f, 0); writeDword(f, 0);      // pels per meter
  writeDword(f, 0); writeDword(f, 0);      // clr used, important

  // Patch video strl LIST size
  uint32_t vStrlEnd = f.position();
  patchDword(f, vStrlSizePos, vStrlEnd - vStrlSizePos - 4);

  // --- LIST strl (audio) — only if microphone available ---
  if (hasAudioStream) {
    writeFourCC(f, "LIST");
    uint32_t aStrlSizePos = f.position(); writeDword(f, 0);
    writeFourCC(f, "strl");

    // strh (audio stream header) — 56 bytes data
    writeFourCC(f, "strh");
    writeDword(f, 56);
    writeFourCC(f, "auds");         // fccType
    writeDword(f, 0);               // fccHandler (unused for PCM)
    writeDword(f, 0);               // dwFlags
    writeWord(f, 0); writeWord(f, 0);  // wPriority, wLanguage
    writeDword(f, 0);               // dwInitialFrames
    writeDword(f, 1);               // dwScale
    writeDword(f, AUDIO_SAMPLE_RATE);  // dwRate
    writeDword(f, 0);               // dwStart
    OFF_ASTRH_LENGTH = f.position(); writeDword(f, 0);  // dwLength (total sample frames)
    OFF_ASTRH_SUGBUF = f.position(); writeDword(f, 0);  // dwSuggestedBufferSize
    writeDword(f, (uint32_t)-1);    // dwQuality
    writeDword(f, AUDIO_BLOCK_ALIGN);  // dwSampleSize
    writeWord(f, 0); writeWord(f, 0);  // rcFrame (unused for audio)
    writeWord(f, 0); writeWord(f, 0);

    // strf (audio format — PCMWAVEFORMAT) — 16 bytes data
    writeFourCC(f, "strf");
    writeDword(f, 16);
    writeWord(f, 1);               // wFormatTag = PCM
    writeWord(f, AUDIO_CHANNELS);
    writeDword(f, AUDIO_SAMPLE_RATE);
    writeDword(f, AUDIO_BYTES_PER_SEC);
    writeWord(f, AUDIO_BLOCK_ALIGN);
    writeWord(f, AUDIO_BITS);

    // Patch audio strl LIST size
    uint32_t aStrlEnd = f.position();
    patchDword(f, aStrlSizePos, aStrlEnd - aStrlSizePos - 4);
  }

  // Patch hdrl LIST size
  uint32_t hdrlEnd = f.position();
  patchDword(f, OFF_HDRL_SIZE, hdrlEnd - OFF_HDRL_SIZE - 4);

  // --- LIST movi ---
  writeFourCC(f, "LIST");
  OFF_MOVI_SIZE = f.position(); writeDword(f, 0);  // placeholder
  writeFourCC(f, "movi");

  f.flush();
}

// Patch all AVI header fields in-place so the file is always valid on disk.
static void aviPatchHeaders(File &f) {
  uint32_t elapsed = millis() - fileStartTime;
  if (elapsed > 0 && aviVideoFrames > 0) {
    aviActualFps = (aviVideoFrames * 1000UL) / elapsed;
    if (aviActualFps < 1) aviActualFps = 1;
  }
  uint32_t usPerFrame = (aviActualFps > 0) ? (1000000 / aviActualFps) : 66666;

  uint32_t fileSize = f.position();
  patchDword(f, OFF_RIFF_SIZE,       fileSize - 8);
  patchDword(f, OFF_AVIH_USPERFRAME, usPerFrame);
  patchDword(f, OFF_AVIH_MAXBYTES,   aviMaxVideoFrame * aviActualFps);
  patchDword(f, OFF_AVIH_TOTALFRAMES, aviVideoFrames);
  patchDword(f, OFF_AVIH_SUGBUF,     aviMaxVideoFrame);
  patchDword(f, OFF_VSTRH_RATE,      aviActualFps);
  patchDword(f, OFF_VSTRH_LENGTH,    aviVideoFrames);
  patchDword(f, OFF_VSTRH_SUGBUF,    aviMaxVideoFrame);

  if (hasAudioStream) {
    patchDword(f, OFF_ASTRH_LENGTH, aviAudioBytes / AUDIO_BLOCK_ALIGN);
    patchDword(f, OFF_ASTRH_SUGBUF, aviMaxAudioChunk);
  }

  patchDword(f, OFF_MOVI_SIZE, aviMoviSize + 4);  // +4 for "movi" fourcc
  f.flush();
}

// Write one JPEG video frame
static bool aviWriteVideoFrame(File &f, const uint8_t *data, uint32_t len) {
  if (aviIdxCount >= MAX_INDEX_ENTRIES) return false;

  uint32_t padLen = (len & 1) ? len + 1 : len;

  aviIdx[aviIdxCount].offset  = aviMoviSize + 4;  // +4 to skip "movi" fourcc
  aviIdx[aviIdxCount].size    = len;
  aviIdx[aviIdxCount].isAudio = 0;

  writeFourCC(f, "00dc");
  writeDword(f, len);
  f.write(data, len);
  if (len & 1) { uint8_t pad = 0; f.write(&pad, 1); }

  aviMoviSize += 8 + padLen;
  aviIdxCount++;
  aviVideoFrames++;
  if (len > aviMaxVideoFrame) aviMaxVideoFrame = len;

  // Patch headers periodically based on video frame count
  if ((aviVideoFrames % HEADER_PATCH_INTERVAL) == 0) {
    aviPatchHeaders(f);
  }

  return true;
}

// Write one audio chunk (PCM samples)
static bool aviWriteAudioChunk(File &f, const uint8_t *data, uint32_t len) {
  if (aviIdxCount >= MAX_INDEX_ENTRIES || len == 0) return false;

  uint32_t padLen = (len & 1) ? len + 1 : len;

  aviIdx[aviIdxCount].offset  = aviMoviSize + 4;
  aviIdx[aviIdxCount].size    = len;
  aviIdx[aviIdxCount].isAudio = 1;

  writeFourCC(f, "01wb");
  writeDword(f, len);
  f.write(data, len);
  if (len & 1) { uint8_t pad = 0; f.write(&pad, 1); }

  aviMoviSize += 8 + padLen;
  aviIdxCount++;
  aviAudioBytes += len;
  if (len > aviMaxAudioChunk) aviMaxAudioChunk = len;

  return true;
}

// Write idx1 index, patch headers, close file
static void aviFinalize(File &f) {
  if (aviIdxCount == 0) {
    f.close();
    return;
  }

  // Write idx1 chunk
  writeFourCC(f, "idx1");
  writeDword(f, aviIdxCount * 16);
  for (uint32_t i = 0; i < aviIdxCount; i++) {
    writeFourCC(f, aviIdx[i].isAudio ? "01wb" : "00dc");
    writeDword(f, 0x10);  // AVIIF_KEYFRAME
    writeDword(f, aviIdx[i].offset);
    writeDword(f, aviIdx[i].size);
  }

  aviPatchHeaders(f);
  uint32_t finalSize = f.position();
  f.close();

  Serial.printf("AVI saved: %u video + %u audio chunks, %.1f MB\n",
    aviVideoFrames, aviIdxCount - aviVideoFrames, finalSize / 1048576.0);
}

// Open a new AVI file
static bool aviOpenNewFile() {
  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    camera_fb_t *probe = esp_camera_fb_get();
    if (probe) {
      aviWidth  = probe->width;
      aviHeight = probe->height;
      esp_camera_fb_return(probe);
    }
  }

  char path[32];
  snprintf(path, sizeof(path), "/vid_%04u.avi", fileIndex++);

  aviFile = SD_MMC.open(path, FILE_WRITE);
  if (!aviFile) {
    Serial.printf("Failed to create %s\n", path);
    return false;
  }

  aviVideoFrames   = 0;
  aviAudioBytes    = 0;
  aviIdxCount      = 0;
  aviMoviSize      = 0;
  aviMaxVideoFrame = 0;
  aviMaxAudioChunk = 0;
  fileStartTime    = millis();

  aviWriteHeader(aviFile);
  Serial.printf("Recording to %s (%ux%u%s)\n", path, aviWidth, aviHeight,
    hasAudioStream ? " + audio" : "");
  return true;
}

// FreeRTOS task: record video + audio to SD card
static void recordingTask(void *param) {
  Serial.println("Recording task started on core " + String(xPortGetCoreID()));

  // Allocate index buffer
  if (psramFound()) {
    aviIdx = (AviIndexEntry *)ps_malloc(MAX_INDEX_ENTRIES * sizeof(AviIndexEntry));
  } else {
    aviIdx = (AviIndexEntry *)malloc(MAX_INDEX_ENTRIES * sizeof(AviIndexEntry));
  }
  if (!aviIdx) {
    Serial.println("ERROR: Failed to allocate AVI index buffer!");
    vTaskDelete(nullptr);
    return;
  }

  // Allocate audio read buffer
  uint8_t *audioBuf = nullptr;
  if (micOK) {
    audioBuf = (uint8_t *)malloc(AUDIO_BUF_SIZE);
    if (!audioBuf) {
      Serial.println("WARNING: Failed to allocate audio buffer, recording video only");
      micOK = false;
    }
  }

  if (!aviOpenNewFile()) {
    free(aviIdx);
    if (audioBuf) free(audioBuf);
    vTaskDelete(nullptr);
    return;
  }
  isRecording = true;

  while (true) {
    // Capture and write video frame
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) {
      if (fb->format == PIXFORMAT_JPEG && fb->len > 0) {
        aviWriteVideoFrame(aviFile, fb->buf, fb->len);
      }
      esp_camera_fb_return(fb);

      // Read and write available audio after each video frame
      if (micOK && audioBuf && hasAudioStream) {
        size_t bytesRead = 0;
        esp_err_t err = i2s_read(I2S_PORT, audioBuf, AUDIO_BUF_SIZE,
                                 &bytesRead, pdMS_TO_TICKS(5));
        if (err == ESP_OK && bytesRead > 0) {
          aviWriteAudioChunk(aviFile, audioBuf, bytesRead);
        }
      }
    } else {
      vTaskDelay(1);
    }

    // Split file check
    bool splitByTime = (millis() - fileStartTime) >= MAX_FILE_DURATION;
    bool splitBySize = aviFile.position() >= MAX_FILE_SIZE;
    bool splitByIdx  = aviIdxCount >= MAX_INDEX_ENTRIES;

    if (splitByTime || splitBySize || splitByIdx) {
      aviFinalize(aviFile);
      if (!aviOpenNewFile()) {
        Serial.println("Failed to open next AVI file, stopping recording");
        isRecording = false;
        break;
      }
    }
  }

  if (audioBuf) free(audioBuf);
  free(aviIdx);
  aviIdx = nullptr;
  vTaskDelete(nullptr);
}

// ===================== SD card init =========================

bool initSD() {
  SD_MMC.setPins(SD_MMC_CLK, SD_MMC_CMD, SD_MMC_D0);

  if (!SD_MMC.begin("/sdcard", true)) {
    Serial.println("SD card mount FAILED");
    return false;
  }

  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD card inserted");
    return false;
  }

  const char *types[] = {"UNKNOWN", "MMC", "SD", "SDHC"};
  Serial.printf("SD card: %s, %.1f MB free\n",
    (cardType <= CARD_SDHC) ? types[cardType] : "UNKNOWN",
    (SD_MMC.totalBytes() - SD_MMC.usedBytes()) / 1048576.0);

  return true;
}

// =================== Microphone init ========================

bool initMicrophone() {
  i2s_config_t i2s_config = {};
  i2s_config.mode            = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM);
  i2s_config.sample_rate     = AUDIO_SAMPLE_RATE;
  i2s_config.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  i2s_config.channel_format  = I2S_CHANNEL_FMT_ONLY_LEFT;
  i2s_config.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  i2s_config.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  i2s_config.dma_buf_count   = 8;
  i2s_config.dma_buf_len     = 1024;
  i2s_config.use_apll        = false;

  esp_err_t err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  if (err != ESP_OK) {
    Serial.printf("I2S driver install failed: 0x%x\n", err);
    return false;
  }

  i2s_pin_config_t pin_config = {};
  pin_config.bck_io_num   = I2S_PIN_NO_CHANGE;
  pin_config.ws_io_num    = PDM_CLK_PIN;     // PDM clock output → GPIO 42
  pin_config.data_out_num = I2S_PIN_NO_CHANGE;
  pin_config.data_in_num  = PDM_DATA_PIN;    // PDM data input  → GPIO 41

  err = i2s_set_pin(I2S_PORT, &pin_config);
  if (err != ESP_OK) {
    Serial.printf("I2S set pin failed: 0x%x\n", err);
    i2s_driver_uninstall(I2S_PORT);
    return false;
  }

  Serial.printf("PDM microphone initialized: %d Hz, %d-bit, %d ch\n",
    AUDIO_SAMPLE_RATE, AUDIO_BITS, AUDIO_CHANNELS);
  return true;
}

// ===================== Camera init ==========================

void initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode    = CAMERA_GRAB_LATEST;

  if (psramFound()) {
    config.frame_size   = FRAMESIZE_SVGA;
    config.jpeg_quality = 4;
    config.fb_count     = 2;
    config.fb_location  = CAMERA_FB_IN_PSRAM;
    Serial.println("PSRAM found - using double buffer");
  } else {
    config.frame_size   = FRAMESIZE_SVGA;
    config.jpeg_quality = 10;
    config.fb_count     = 1;
    config.fb_location  = CAMERA_FB_IN_DRAM;
    Serial.println("No PSRAM - using single buffer");
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init FAILED: 0x%x\n", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);
    s->set_brightness(s, 1);
    s->set_saturation(s, -2);
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    s->set_wb_mode(s, 0);
    Serial.println("OV3660 detected - applied tuning + AWB");
  }

  delay(100);
  sensor_t *af = esp_camera_sensor_get();
  af->set_reg(af, 0x3022, 0xFF, 0x04);
  Serial.println("Auto-focus enabled");
  Serial.println("Camera initialized OK");
}

// ==================== Web server ============================

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>ESP32-S3 Camera Stream</title>
  <style>
    * { margin: 0; padding: 0; box-sizing: border-box; }
    body {
      font-family: Arial, sans-serif;
      background: #1a1a2e;
      color: #eee;
      display: flex;
      flex-direction: column;
      align-items: center;
      min-height: 100vh;
      padding: 20px;
    }
    h1 { margin-bottom: 10px; color: #e94560; }
    .info { margin-bottom: 15px; color: #aaa; font-size: 14px; }
    #stream {
      max-width: 100%;
      border: 2px solid #e94560;
      border-radius: 8px;
      background: #000;
    }
    .controls {
      margin-top: 15px;
      display: flex;
      gap: 10px;
      flex-wrap: wrap;
      justify-content: center;
    }
    select, button {
      padding: 8px 16px;
      border: 1px solid #e94560;
      border-radius: 4px;
      background: #16213e;
      color: #eee;
      font-size: 14px;
      cursor: pointer;
    }
    button:hover { background: #e94560; }
    .rec-indicator {
      margin-top: 10px;
      padding: 6px 14px;
      border-radius: 4px;
      font-size: 13px;
      font-weight: bold;
    }
    .rec-on  { background: #c0392b; color: #fff; }
    .rec-off { background: #555; color: #aaa; }
  </style>
</head>
<body>
  <h1>ESP32-S3 Camera</h1>
  <div class="info" id="resolution">Loading...</div>
  <img id="stream" src="/stream" alt="Camera Stream">
  <div class="controls">
    <label>Resolution:
      <select id="res" onchange="setResolution(this.value)">
        <option value="15">QXGA (2048x1536)</option>
        <option value="13">UXGA (1600x1200)</option>
        <option value="12">SXGA (1280x1024)</option>
        <option value="10">XGA (1024x768)</option>
        <option value="9" selected>SVGA (800x600)</option>
        <option value="8">VGA (640x480)</option>
        <option value="5">CIF (400x296)</option>
        <option value="0">QQVGA (160x120)</option>
      </select>
    </label>
    <label>Quality:
      <select id="quality" onchange="setQuality(this.value)">
        <option value="4" selected>Best</option>
        <option value="10">High</option>
        <option value="15">Medium</option>
        <option value="25">Low</option>
      </select>
    </label>
    <button onclick="location.reload()">Reconnect</button>
  </div>
  <div id="recStatus" class="rec-indicator rec-off">SD: checking...</div>
  <script>
    function setResolution(val) { fetch('/control?var=framesize&val=' + val); }
    function setQuality(val)    { fetch('/control?var=quality&val=' + val); }
    function updateStatus() {
      fetch('/status').then(r => r.json()).then(d => {
        var el = document.getElementById('recStatus');
        if (d.recording) {
          var txt = 'REC ' + d.file + ' | ' + d.frames + ' frames | ' + d.sizeMB + ' MB';
          if (d.mic) txt += ' | MIC';
          el.className = 'rec-indicator rec-on';
          el.textContent = txt;
        } else if (d.sdcard) {
          el.className = 'rec-indicator rec-off';
          el.textContent = 'SD ready (not recording)';
        } else {
          el.className = 'rec-indicator rec-off';
          el.textContent = 'No SD card';
        }
      }).catch(() => {});
    }
    updateStatus();
    setInterval(updateStatus, 2000);
  </script>
</body>
</html>
)rawliteral";

void handleStream() {
  WiFiClient client = server.client();

  String response = "HTTP/1.1 200 OK\r\n"
                    "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
                    "Access-Control-Allow-Origin: *\r\n"
                    "\r\n";
  client.print(response);

  while (client.connected()) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Frame capture failed");
      continue;
    }

    String header = "--frame\r\n"
                    "Content-Type: image/jpeg\r\n"
                    "Content-Length: " + String(fb->len) + "\r\n"
                    "\r\n";
    client.print(header);
    client.write(fb->buf, fb->len);
    client.print("\r\n");

    esp_camera_fb_return(fb);
    if (!client.connected()) break;
  }
}

void handleCapture() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    server.send(500, "text/plain", "Camera capture failed");
    return;
  }
  server.sendHeader("Content-Disposition", "inline; filename=capture.jpg");
  server.send_P(200, "image/jpeg", (const char *)fb->buf, fb->len);
  esp_camera_fb_return(fb);
}

void handleControl() {
  String var = server.arg("var");
  int val = server.arg("val").toInt();

  sensor_t *s = esp_camera_sensor_get();
  int res = -1;

  if (var == "framesize") {
    if (val >= 0 && val <= 15) {
      res = s->set_framesize(s, (framesize_t)val);
    }
  } else if (var == "quality") {
    res = s->set_quality(s, val);
  }

  server.send(res == 0 ? 200 : 400, "text/plain", res == 0 ? "OK" : "Failed");
}

void handleStatus() {
  char json[300];
  snprintf(json, sizeof(json),
    "{\"sdcard\":%s,\"recording\":%s,\"mic\":%s,"
    "\"file\":\"vid_%04u.avi\",\"frames\":%u,\"sizeMB\":\"%.1f\"}",
    sdCardOK ? "true" : "false",
    isRecording ? "true" : "false",
    (micOK && hasAudioStream) ? "true" : "false",
    fileIndex > 0 ? fileIndex - 1 : 0,
    aviVideoFrames,
    aviFile ? aviFile.position() / 1048576.0 : 0.0);
  server.send(200, "application/json", json);
}

void handleIndex() {
  server.send_P(200, "text/html", INDEX_HTML);
}

// ======================== Setup =============================

void setup() {
  Serial.begin(115200);

  delay(1000);
  Serial.println("\n=== ESP32-S3 Camera + Audio Streaming ===");

  initCamera();

  micOK = initMicrophone();
  if (!micOK) {
    Serial.println("Microphone not available - recording video only");
  }

  sdCardOK = initSD();

  if (sdCardOK) {
    xTaskCreatePinnedToCore(
      recordingTask,
      "AVI_Rec",
      8192,
      nullptr,
      1,
      &recTaskHandle,
      0
    );
  } else {
    Serial.println("SD card not available - recording disabled");
  }

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("Connecting to %s", WIFI_SSID);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(" Connected!");
    Serial.printf("Stream URL: http://%s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\nWiFi connection failed! Starting AP mode...");
    WiFi.softAP("ESP32CAM", "12345678");
    Serial.printf("AP started. Connect to 'ESP32CAM' and go to http://%s\n",
                  WiFi.softAPIP().toString().c_str());
  }

  server.on("/", handleIndex);
  server.on("/stream", handleStream);
  server.on("/capture", handleCapture);
  server.on("/control", handleControl);
  server.on("/status", handleStatus);
  server.begin();
  Serial.println("HTTP server started");

  // Enable LEDs after boot
  pinMode(21, OUTPUT);
  digitalWrite(21, LOW);
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
}

void loop() {
  server.handleClient();
}
