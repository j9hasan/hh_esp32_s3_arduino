#include <Arduino.h>
#include "lvgl.h"
#include "demos/lv_demos.h"
#include <Arduino_GFX_Library.h>
#include "wifi_comm.h"
#include "bt_comm.h"
#include <SD.h>

/*-------------------------------------------------INITIALIZE GRAPHICS SETTINGS----------------------------------------*/
#define GFX_BL DF_GFX_BL
#if defined(ESP32) && (CONFIG_IDF_TARGET_ESP32)

Arduino_DataBus *bus = create_default_Arduino_DataBus();
Arduino_GFX *gfx = new Arduino_ILI9341(bus, DF_GFX_RST, 3 /* rotation */, false /* IPS */);

#endif

#if defined(ESP32) && (CONFIG_IDF_TARGET_ESP32S3)

Arduino_ESP32RGBPanel *bus = new Arduino_ESP32RGBPanel(
    // GFX_NOT_DEFINED /* CS */, GFX_NOT_DEFINED /* SCK */, GFX_NOT_DEFINED /* SDA */,
    41 /* DE */, 40 /* VSYNC */, 39 /* HSYNC */, 42 /* PCLK */,
    14 /* R0 */, 21 /* R1 */, 47 /* R2 */, 48 /* R3 */, 45 /* R4 */,
    9 /* G0 */, 46 /* G1 */, 3 /* G2 */, 8 /* G3 */, 16 /* G4 */, 1 /* G5 */,
    15 /* B0 */, 7 /* B1 */, 6 /* B2 */, 5 /* B3 */, 4 /* B4 */,
    0 /* hsync_polarity */, 180 /* hsync_front_porch */, 30 /* hsync_pulse_width */, 16 /* hsync_back_porch */,
    0 /* vsync_polarity */, 12 /* vsync_front_porch */, 13 /* vsync_pulse_width */, 10 /* vsync_back_porch */ // ,1 /* pclk_active_neg */, 8000000 /* prefer_speed */, GFX_NOT_DEFINED /* auto_flush */

);
Arduino_RGB_Display *gfx = new Arduino_RGB_Display(
    800, 480, bus, 0, true, NULL);

#endif
/*-------------------------------------------------GRAPHICS SETTING DONE----------------------------------------*/

#include "touch.h"
#include "BmpClass.h"

static BmpClass bmpClass;

// defines
#define BMP_FILENAME "/bmg1.bmp"

#define SD_SCK 12
#define SD_MISO 13
#define SD_MOSI 11
#define SD_CS 10

// pixel drawing callback
static void bmpDrawCallback(int16_t x, int16_t y, uint16_t *bitmap, int16_t w, int16_t h)
{
  gfx->draw16bitRGBBitmap(x, y, bitmap, w, h);
}

// static protootypes
void printMemoryInfo();
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p);
void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data);
void sd_init();
void print_bmp();

/* Change to your screen resolution */
static uint32_t screenWidth;
static uint32_t screenHeight;
static uint32_t bufSize;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t *disp_draw_buf;
static lv_color_t *disp_draw_buf2;
static lv_disp_drv_t disp_drv;

void setup()
{
  Serial.begin(115200);
  // Serial.setDebugOutput(true);
  // while(!Serial);
  Serial.println("Arduino_GFX LVGL Widgets example");

#ifdef GFX_EXTRA_PRE_INIT
  GFX_EXTRA_PRE_INIT();
#endif

  // Init Display
  if (!gfx->begin())
  {
    Serial.println("gfx->begin() failed!");
  }
  gfx->fillScreen(BLACK);

#ifdef GFX_BL
  pinMode(GFX_BL, OUTPUT);
  digitalWrite(GFX_BL, HIGH);
#endif

  // Init touch device
  touch_init();

  lv_init();

  sd_init();

  screenWidth = gfx->width();
  screenHeight = gfx->height();

  // #define DIRECT_MODE

#ifdef DIRECT_MODE
  bufSize = screenWidth * screenHeight;
#else
  bufSize = screenWidth * 40;
#endif

#ifdef ESP32
  // use MALLOC_CAP_INTERNAL for better perf, but consumes more heap
  disp_draw_buf = (lv_color_t *)heap_caps_malloc(sizeof(lv_color_t) * bufSize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  // disp_draw_buf2 = (lv_color_t *)heap_caps_malloc(sizeof(lv_color_t) * bufSize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!disp_draw_buf)
  {
    // remove MALLOC_CAP_SPIRAM flag try again
    disp_draw_buf = (lv_color_t *)heap_caps_malloc(sizeof(lv_color_t) * bufSize, MALLOC_CAP_8BIT);
  }
#else
  disp_draw_buf = (lv_color_t *)malloc(sizeof(lv_color_t) * bufSize);
#endif
  if (!disp_draw_buf)
  {
    Serial.println("LVGL disp_draw_buf allocate failed!");
  }
  else
  {
    lv_disp_draw_buf_init(&draw_buf, disp_draw_buf, NULL, bufSize);

    /* Initialize the display */
    lv_disp_drv_init(&disp_drv);
    /* Change the following line to your display resolution */
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
#ifdef DIRECT_MODE
    disp_drv.direct_mode = true;
#endif
    lv_disp_drv_register(&disp_drv);

    /* Initialize the (dummy) input device driver */
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register(&indev_drv);

    lv_demo_widgets();

    // call other func
    connectToWiFi();
    // initBLE();

    Serial.println("Setup done");
  }
  printMemoryInfo();
}
int time_;
bool printed = false;
void loop()
{
  time_ = millis() / 1000;
  lv_timer_handler(); /* let the GUI do its work */

#ifdef DIRECT_MODE
#if (LV_COLOR_16_SWAP != 0)
  gfx->draw16bitBeRGBBitmap(0, 0, (uint16_t *)disp_draw_buf, screenWidth, screenHeight);
#else
  gfx->draw16bitRGBBitmap(0, 0, (uint16_t *)disp_draw_buf, screenWidth, screenHeight);
#endif
#endif // #ifdef DIRECT_MODE

#ifdef CANVAS
  gfx->flush();
#endif
  if (time_ > 5 && !printed)
  {
    Serial.println("Printing");
    print_bmp();
    printed = true;
  }
  delay(5);
}

void printMemoryInfo()
{
  Serial.print("Free heap: ");
  Serial.print(ESP.getFreeHeap() / 1024);
  Serial.println(" KB");

  Serial.print("Free PSRAM: ");
  Serial.print(ESP.getFreePsram() / 1024);
  Serial.println(" KB");
}
/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
#ifndef DIRECT_MODE
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

#if (LV_COLOR_16_SWAP != 0)
  gfx->draw16bitBeRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#else
  gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#endif
#endif // #ifndef DIRECT_MODE

  lv_disp_flush_ready(disp);
}

void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data)
{
  if (touch_has_signal())
  {
    if (touch_touched())
    {
      data->state = LV_INDEV_STATE_PR;

      /*Set the coordinates*/
      data->point.x = touch_last_x;
      data->point.y = touch_last_y;
    }
    else if (touch_released())
    {
      data->state = LV_INDEV_STATE_REL;
    }
  }
  else
  {
    data->state = LV_INDEV_STATE_REL;
  }
}
void sd_init()
{
  // SD(SPI)
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI);
  SPI.setFrequency(1000000);
  if (!SD.begin(SD_CS, SPI))
  {
    Serial.println("Card Mount Failed");
    while (1)
      ;
  }
  else
  {
    Serial.println("SD OK");
  }
}
void print_bmp()
{
  printMemoryInfo();
  File bmpFile = SD.open(BMP_FILENAME, "r");

  // read BMP file header
  bmpClass.draw(&bmpFile, bmpDrawCallback, false /* useBigEndian */,
                10 /* x */, 10 /* y */, gfx->width() /* widthLimit */, gfx->height() /* heightLimit */);

  bmpFile.close();
  printMemoryInfo();
}