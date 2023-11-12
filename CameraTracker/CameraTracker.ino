#include "esp_camera.h"

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h> // only for esp_wifi_set_channel()

#define CHANNEL 1

//
// WARNING!!! PSRAM IC required for UXGA resolution and high JPEG quality
//            Ensure ESP32 Wrover Module or other board with PSRAM is selected
//            Partial images will be transmitted if image exceeds buffer size
//
//            You must select partition scheme from the board menu that has at least 3MB APP space.
//            Face Recognition is DISABLED for ESP32 and ESP32-S2, because it takes up from 15 
//            seconds to process single frame. Face Detection is ENABLED if PSRAM is enabled as well


#define CAMERA_MODEL_AI_THINKER // Has PSRAM
#include "camera_pins.h"

#define MAX_PAYLOAD_SIZE 200 // The whole packet can not be larget than 250Bytes.
typedef struct
{
  uint32_t lat;
  uint32_t lon;
  uint32_t alt;
  uint32_t temp;
} telemetry_t;

typedef struct
{
  uint32_t packet_num;
  uint32_t batch_id;
  uint32_t packets_in_batch;
  uint32_t payload_size;
  union
  {
    telemetry_t telemetry; // Only the first packet includes telemetry
    uint8_t jpeg[MAX_PAYLOAD_SIZE];
  } payload;
} packet_t;

esp_now_peer_info_t slave = {
        .peer_addr = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, // Broadcast mode
        .channel = CHANNEL
};

void startCameraServer();

// Init ESP Now with fallback
void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  }
  else {
    Serial.println("ESPNow Init Failed");
    ESP.restart();
  }
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
//  Serial.print("Last Packet Send Status: ");
//  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);
  // TOOD: uncomment when needed
  // WiFi.setTxPower(WIFI_POWER_19_5dBm);
  Serial.println("Start!");
  InitESPNow();
  esp_now_register_send_cb(OnDataSent);
  
  esp_err_t addStatus = esp_now_add_peer(&slave);
  if (addStatus != ESP_OK)
  {
    Serial.println("Add peer Failed");
  }

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_QQVGA;
  config.pixel_format = PIXFORMAT_JPEG; // for streaming
  //config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;
  
  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if(config.pixel_format == PIXFORMAT_JPEG){
    if(psramFound()){
      config.jpeg_quality = 10;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      // Limit the frame size when PSRAM is not available
      config.frame_size = FRAMESIZE_QQVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    // Best option for face detection/recognition
    config.frame_size = FRAMESIZE_QQVGA;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1); // flip it back
    s->set_brightness(s, 1); // up the brightness just a bit
    s->set_saturation(s, -2); // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  if(config.pixel_format == PIXFORMAT_JPEG){
    s->set_framesize(s, FRAMESIZE_QQVGA);//FRAMESIZE_QQVGA
  }

  Serial.printf("Camera sensor: 0x%x", s->id.PID);
}

void send_batch(telemetry_t *telemetry, camera_fb_t *pic)
{
  static uint32_t batch_id = 0;
  packet_t packet;
  static const uint8_t header_size = sizeof(packet.packet_num) +
                                     sizeof(packet.batch_id) +
                                     sizeof(packet.packets_in_batch) +
                                     sizeof(packet.payload_size);
  static const uint8_t jpeg_frame_size = sizeof(packet.payload.jpeg);

  // Prepare and send header
  packet.packet_num = 0;
  packet.batch_id = batch_id++;
  packet.packets_in_batch = (pic->len + (jpeg_frame_size)) / jpeg_frame_size + 1; // Rounding up plus first packet.
  packet.payload_size = sizeof(packet.payload.telemetry);
  packet.payload.telemetry = *telemetry;
  esp_now_send(slave.peer_addr, (const uint8_t*)&packet, header_size + sizeof(packet.payload.telemetry)); // Short first packet with telemetry.

  // Image payload
  for (uint32_t offset = 0; offset < pic->len; offset += jpeg_frame_size)
  {
    packet.payload_size = jpeg_frame_size;

    if ((pic->len - offset) < jpeg_frame_size)
        packet.payload_size = pic->len - offset; // Last packet.

    packet.packet_num++;
    memcpy(&packet.payload.jpeg, pic->buf + offset, packet.payload_size);

    Serial.printf("Sending packet %d from %d, jpeg size %d\n", packet.packet_num, offset, packet.payload_size);
    esp_now_send(slave.peer_addr, (const uint8_t*)&packet, packet.payload_size + header_size);
    delay(10);
  }

  Serial.printf("Batch sent, jpeg_size %d\n", pic->len);
}

void loop() {
  static telemetry_t telemetry = {.lat = 531325, .lon = 231688, .alt = 101, .temp = 23};

  telemetry.alt++;

  Serial.printf("Taking picture...\n");
  camera_fb_t *pic = esp_camera_fb_get();

  Serial.printf("Picture taken! Its size was: %zu bytes, format: 0x%x\n", pic->len, pic->format);

  send_batch(&telemetry, pic);

  esp_camera_fb_return(pic);

  delay(300);
}
