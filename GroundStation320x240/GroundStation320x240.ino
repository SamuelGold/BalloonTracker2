// Uncomment #include <User_Setups/Setup22_TTGO_T4_v1.3.h> in ./libraries/TFT_eSPI/User_Setup_Select.h to make display working.
#include <TFT_eSPI.h>
#include <SPI.h>
#include <esp_now.h>
#include "WiFi.h"
#include <Wire.h>
#include <JPEGDecoder.h>
#include "driver/rtc_io.h"

#define minimum(a,b)     (((a) < (b)) ? (a) : (b))

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

// TFT Pins has been set in the TFT_eSPI library in the User Setup file TTGO_T_Display.h
// #define TFT_MOSI            19
// #define TFT_SCLK            18
// #define TFT_CS              5
// #define TFT_DC              16
// #define TFT_RST             23
// #define TFT_BL              4   // Display backlight control pin

#define CHANNEL 1

TFT_eSPI tft = TFT_eSPI(240, 320); // Invoke custom library

char buff[512];
int vref = 1100;
int btnCick = false;

//#define ENABLE_SPI_SDCARD

//Uncomment will use SDCard, this is just a demonstration,
//how to use the second SPI
#ifdef ENABLE_SPI_SDCARD

#include "FS.h"
#include "SD.h"

SPIClass SDSPI(HSPI);

#define MY_CS       33
#define MY_SCLK     25
#define MY_MISO     27
#define MY_MOSI     26

void setupSDCard()
{
    SDSPI.begin(MY_SCLK, MY_MISO, MY_MOSI, MY_CS);
    //Assuming use of SPI SD card
    if (!SD.begin(MY_CS, SDSPI)) {
        Serial.println("Card Mount Failed");
        tft.setTextColor(TFT_RED);
        tft.drawString("SDCard Mount FAIL", tft.width() / 2, tft.height() / 2 - 32);
        tft.setTextColor(TFT_GREEN);
    } else {
        tft.setTextColor(TFT_GREEN);
        Serial.println("SDCard Mount PASS");
        tft.drawString("SDCard Mount PASS", tft.width() / 2, tft.height() / 2 - 48);
        String size = String((uint32_t)(SD.cardSize() / 1024 / 1024)) + "MB";
        tft.drawString(size, tft.width() / 2, tft.height() / 2 - 32);
    }
}
#else
#define setupSDCard()
#endif


//! Long time delay, it is recommended to use shallow sleep, which can effectively reduce the current consumption
void espDelay(int ms)
{
    esp_sleep_enable_timer_wakeup(ms * 1000);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
    esp_light_sleep_start();
}

// Init ESP Now with fallback
void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  }
  else {
    Serial.println("ESPNow Init Failed");
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
  }
}

void jpegRender(int xpos, int ypos) {
  // retrieve infomration about the image
  uint16_t  *pImg;
  uint16_t mcu_w = JpegDec.MCUWidth;
  uint16_t mcu_h = JpegDec.MCUHeight;
  uint32_t max_x = JpegDec.width;
  uint32_t max_y = JpegDec.height;

  // Jpeg images are draw as a set of image block (tiles) called Minimum Coding Units (MCUs)
  // Typically these MCUs are 16x16 pixel blocks
  // Determine the width and height of the right and bottom edge image blocks
  uint32_t min_w = minimum(mcu_w, max_x % mcu_w);
  uint32_t min_h = minimum(mcu_h, max_y % mcu_h);

  // save the current image block size
  uint32_t win_w = mcu_w;
  uint32_t win_h = mcu_h;

  // record the current time so we can measure how long it takes to draw an image
  uint32_t drawTime = millis();

  // save the coordinate of the right and bottom edges to assist image cropping
  // to the screen size
  max_x += xpos;
  max_y += ypos;

  // read each MCU block until there are no more
  while ( JpegDec.read()) {

    // save a pointer to the image block
    pImg = JpegDec.pImage;

    // calculate where the image block should be drawn on the screen
    int mcu_x = JpegDec.MCUx * mcu_w + xpos;
    int mcu_y = JpegDec.MCUy * mcu_h + ypos;

    //Serial.printf("\n JpegDec.MCUx: %d JpegDec.MCUy: %d mcu_x: %d mcu_y: %d\n", JpegDec.MCUx, JpegDec.MCUy, mcu_x, mcu_y);

    // check if the image block size needs to be changed for the right and bottom edges
    if (mcu_x + mcu_w <= max_x) win_w = mcu_w;
    else win_w = min_w;
    if (mcu_y + mcu_h <= max_y) win_h = mcu_h;
    else win_h = min_h;

    // calculate how many pixels must be drawn
    uint16_t mcu_pixels = win_w * win_h;

    // draw image MCU block only if it will fit on the screen
    if ( ( mcu_x + win_w) <= tft.width() && ( mcu_y + win_h) <= tft.height() - 16)
    {
      // Now set a MCU bounding window on the TFT to push pixels into (x, y, x + width - 1, y + height - 1)
      //Serial.printf("\n Window: %dx%d - %dx%d\n", mcu_x, mcu_y, mcu_x + win_w - 1, mcu_y + win_h - 1);
      tft.setAddrWindow(mcu_x, mcu_y, win_w, win_h);
      //tft.setWindow(mcu_x, mcu_y, mcu_x + win_w - 1, mcu_y + win_h - 1);

      // Write all MCU pixels to the TFT window
      //while (mcu_pixels--) tft.pushColor(*pImg++); // Send MCU buffer to TFT 16 bits at a time
      tft.pushColors(pImg, mcu_pixels, 1);
    }
    /*else
    {
      while (mcu_pixels--) *pImg++;  
    }*/

    // Stop drawing blocks if the bottom of the screen has been reached,
    // the abort function will close the file
    else if ( ( mcu_y + win_h) >= tft.height()) JpegDec.abort();

  }

  // calculate how long it took to draw the image
  drawTime = millis() - drawTime;

  // print the results to the serial port
  //Serial.print  ("Total render time was    : "); Serial.print(drawTime); Serial.println(" ms");
  //Serial.println("=====================================");

}

void infoRender(char *info, uint32_t value, uint32_t x, uint32_t y, uint32_t x_size, uint32_t y_size)
{
  tft.setAddrWindow(x, y + 11, x_size, y_size);
  for (uint32_t i = 0; i < x_size * y_size; i++)
  {
    tft.pushColor(TFT_RED);
  }

  tft.setTextSize(1);
  tft.setTextColor(TFT_GREEN);
  tft.drawString(info, x, y);
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE);
  tft.drawString(String(value), x, y + 12);
}

void telemetryRender(telemetry_t tele)
{
    uint32_t x_offset = 0;
    uint32_t x_step = 85;
    uint32_t y_offset = 210;
    uint32_t y_size = 17;
    
    infoRender("Lat:", tele.lat, x_step * x_offset++, y_offset, x_step, y_size);
    infoRender("Lon:", tele.lon, x_step * x_offset++, y_offset, x_step, y_size);
    infoRender("Alt:", tele.alt, x_step * x_offset++, y_offset, x_step, y_size);
    infoRender("Temp:", tele.temp, x_step * x_offset++, y_offset, x_step, y_size);
}

void packetsRender(uint32_t last_packet_time)
{
  uint32_t max_line_width = 320;
  uint32_t line_height = 3;

  tft.setAddrWindow(0, 0, max_line_width, line_height);
  for (uint32_t i = 0; i < max_line_width * line_height; i++)
  {
    tft.pushColor(TFT_BLACK);
  }

  if (last_packet_time > 0)
  {
    if (last_packet_time > max_line_width)
      last_packet_time = max_line_width;

    tft.setAddrWindow(0, 0, last_packet_time, line_height);

    for (uint32_t i = 0; i < last_packet_time * line_height; i++)
    {
      tft.pushColor(TFT_RED);
    }
  }

  if (last_packet_time < 2)
  {

  }
}

static bool jpeg_received = false;
static bool telemetry_received = false;
static telemetry_t telemetry;
static uint32_t jpeg_size;
static uint8_t jpeg[16000];

// callback when data is recv from Master
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  packet_t *packet = (packet_t *)data;
  static uint32_t current_batch = 0;
  static const uint8_t jpeg_frame_size = sizeof(packet->payload.jpeg);

  //Serial.printf("\n Got packet %d, packets in batch: %d \n", packet->packet_num, packet->packets_in_batch);

  if (packet->packet_num == 0) // First packet
  {
    telemetry = packet->payload.telemetry;
    telemetry_received = true;
    //Serial.printf("\nBatch start! Lat: %d Lon: %d Alt: %d Temp: %d\n",
    //              telemetry.lat, telemetry.lon, telemetry.alt, telemetry.temp);
    jpeg_size = 0;
    current_batch = packet->batch_id;
  }
  else if (current_batch == packet->batch_id) // Continue processing the batch.
  {
    uint32_t jpeg_data_len = packet->payload_size;
    uint32_t jpeg_offset = (packet->packet_num - 1) * jpeg_frame_size;

    if ((jpeg_offset + jpeg_data_len) > sizeof(jpeg))
    {
      Serial.printf("\n JPEG overflow! %d of %d\n", jpeg_offset + jpeg_data_len, sizeof(jpeg));    
      return;
    }

    memcpy(jpeg + jpeg_size, &packet->payload.jpeg, jpeg_data_len);
    jpeg_size += jpeg_data_len;

    if (packet->packets_in_batch == packet->packet_num + 1) // Last packet
    {
      jpeg_received = true;
    }
  }
}

void setup()
{
    Serial.begin(115200);
    Serial.println("Start");

    tft.init();
    tft.setRotation(3);
    tft.fillScreen(TFT_BLACK);

    setupSDCard();

    //Set device in AP mode to begin with
    WiFi.mode(WIFI_AP);
    // Init ESPNow with a fallback logic
    InitESPNow();
    // Once ESPNow is successfully Init, we will register for recv CB to
    // get recv packer info.
    esp_now_register_recv_cb(OnDataRecv);
}

void loop()
{
  static uint32_t packet_counter = 0;
  const static uint32_t loop_delay = 10;
  const static uint32_t packet_period = 1000; // Telemetry packet period, ~1/sec

  packet_counter++;

  if (telemetry_received)
  {
    packet_counter = 0;
    telemetryRender(telemetry);
    telemetry_received = false;
  }

  if (jpeg_received)  
  {
    int status = JpegDec.decodeArray(jpeg, jpeg_size);

    if (status == 1) // Successfull
    {
      jpegRender(0, 3);
      telemetryRender(telemetry);
    }
    else
    {
      Serial.printf("\n JPEG decoding failed: 0x%x\n", status);
    }

    jpeg_received = false;
  }
  
  packetsRender((packet_counter * loop_delay) / packet_period); // How long we got the last packet.
  delay(loop_delay);
}