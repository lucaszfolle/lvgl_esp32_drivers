/**
 * @file ili9488.c
 */

/*********************
 *      INCLUDES
 *********************/
#include "ili9488.h"
#include "disp_spi.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_heap_caps.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* So I can access the display struct internals */
#include "src/display/lv_display_private.h"
/*********************
 *      DEFINES
 *********************/
#define TAG "lcd_driver"

/**********************
 *      TYPEDEFS
 **********************/

/*The LCD needs a bunch of command/argument values to be initialized. They are stored in this struct. */
typedef struct
{
  uint8_t cmd;
  uint8_t data[16];
  uint8_t databytes; // No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void ili9488_set_orientation(uint8_t orientation);

static void ili9488_send_cmd(uint8_t cmd);
static void ili9488_send_data(void *data, uint16_t length);
static void ili9488_send_color(void *data, uint16_t length);

/**********************
 *  STATIC VARIABLES
 **********************/

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
// From github.com/jeremyjh/ESP32_TFT_library
// From github.com/mvturnho/ILI9488-lvgl-ESP32-WROVER-B
// From https://github.com/lvgl/lvgl_esp32_drivers/issues/133 -> correction about displaying collors
void ili9488_init(void)
{
  lcd_init_cmd_t ili_init_cmds[] = {
      {ILI9488_CMD_SLEEP_OUT, {0x00}, 0x80},
      {ILI9488_CMD_POSITIVE_GAMMA_CORRECTION, {0x00, 0x03, 0x09, 0x08, 0x16, 0x0A, 0x3F, 0x78, 0x4C, 0x09, 0x0A, 0x08, 0x16, 0x1A, 0x0F}, 15},
      {ILI9488_CMD_NEGATIVE_GAMMA_CORRECTION, {0x00, 0x16, 0x19, 0x03, 0x0F, 0x05, 0x32, 0x45, 0x46, 0x04, 0x0E, 0x0D, 0x35, 0x37, 0x0F}, 15},
      {ILI9488_CMD_POWER_CONTROL_1, {0x17, 0x15}, 2},
      {ILI9488_CMD_POWER_CONTROL_2, {0x41}, 1},
      {ILI9488_CMD_VCOM_CONTROL_1, {0x00, 0x12, 0x80}, 3},
      {ILI9488_CMD_MEMORY_ACCESS_CONTROL, {(0x20 | 0x08)}, 1},
      {ILI9488_CMD_COLMOD_PIXEL_FORMAT_SET, {0x66}, 1},
      {ILI9488_CMD_INTERFACE_MODE_CONTROL, {0x00}, 1},
      {ILI9488_CMD_FRAME_RATE_CONTROL_NORMAL, {0xA0}, 1},
      {ILI9488_CMD_DISPLAY_INVERSION_CONTROL, {0x02}, 1},
      {ILI9488_CMD_DISPLAY_FUNCTION_CONTROL, {0x02, 0x02, 0x3B}, 3},
      {ILI9488_CMD_ENTRY_MODE_SET, {0xC6}, 1},
      {ILI9488_CMD_SET_IMAGE_FUNCTION, {0x00}, 1},
      {ILI9488_CMD_DISP_INVERSION_OFF, {0}, 0},
      {ILI9488_CMD_WRITE_CTRL_DISPLAY, {0x28}, 1},
      {ILI9488_CMD_WRITE_DISPLAY_BRIGHTNESS, {0x7F}, 1},
      {ILI9488_CMD_ADJUST_CONTROL_3, {0xA9, 0x51, 0x2C, 0x82}, 4},
      {ILI9488_CMD_DISPLAY_ON, {0x00}, 0x80},
      {0, {0}, 0xff},
  };

// Initialize non-SPI GPIOs
#if ESP_IDF_VERSION_MAJOR >= 5
  esp_rom_gpio_pad_select_gpio(ILI9488_DC);
#else
  gpio_pad_select_gpio(ILI9488_DC);
#endif
  gpio_set_direction(ILI9488_DC, GPIO_MODE_OUTPUT);

#if ILI9488_USE_RST
#if ESP_IDF_VERSION_MAJOR >= 5
  esp_rom_gpio_pad_select_gpio(ILI9488_RST);
#else
  gpio_pad_select_gpio(ILI9488_RST);
#endif
  gpio_set_direction(ILI9488_RST, GPIO_MODE_OUTPUT);

  gpio_set_drive_capability(ILI9488_RST, GPIO_DRIVE_CAP_1);
  gpio_set_drive_capability(ILI9488_DC, GPIO_DRIVE_CAP_1);

  // Reset the display
  gpio_set_level(ILI9488_RST, 0);
  vTaskDelay(100 / portTICK_DELAY_MS);
  gpio_set_level(ILI9488_RST, 1);
  vTaskDelay(100 / portTICK_DELAY_MS);
#endif

  ESP_LOGI(TAG, "ILI9488 initialization.");

  // Exit sleep
  ili9488_send_cmd(0x01); /* Software reset */
  vTaskDelay(100 / portTICK_DELAY_MS);

  // Send all the commands
  uint16_t cmd = 0;
  while (ili_init_cmds[cmd].databytes != 0xff)
  {
    ili9488_send_cmd(ili_init_cmds[cmd].cmd);
    ili9488_send_data(ili_init_cmds[cmd].data, ili_init_cmds[cmd].databytes & 0x1F);
    if (ili_init_cmds[cmd].databytes & 0x80)
    {
      vTaskDelay(100 / portTICK_DELAY_MS);
    }
    cmd++;
  }

  ili9488_set_orientation(CONFIG_LV_DISPLAY_ORIENTATION);
}

// Flush function based on mvturnho repo
// https://github.com/lvgl/lvgl_esp32_drivers/issues/116 -> heap +3 to avoid display artifacts
void ili9488_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
  const uint32_t size = lv_area_get_width(area) * lv_area_get_height(area);
  const uint16_t *p_buf16 = (uint16_t *)px_map;
  const uint8_t *p_flush_start = NULL;
  const struct _lv_display_t *p_display = disp;

  /* Currently sending the buffer 1 */
  if (px_map == p_display->buf_1->data)
  {
    /* Starts from the last pixel, storing the result from the half-buffer's end */
    uint8_t *p_dstbuf = (px_map + ((p_display->buf_1->data_size * 3) / 2));
    p_buf16 += size;
		for (uint32_t ii = 0u; ii < size; ++ii)
    {
    		const uint32_t color = *--p_buf16;
			*--p_dstbuf = (uint8_t) (((color & 0x001F) << 3) | ((color & 0x0010) >> 2));
			*--p_dstbuf = (uint8_t) ((color & 0x07E0) >> 3);
			*--p_dstbuf = (uint8_t) (((color & 0xF800) >> 8) | ((color & 0x8000) >> 13));
    }
    p_flush_start = p_dstbuf;
  }
  /* Currently sending the buffer 2 */
  else if (px_map == p_display->buf_2->data)
  {
    /* Starts from the first pixel, storing the result from the beginning of the half-buffer */
    uint8_t *p_dstbuf = (px_map - (p_display->buf_2->data_size / 2));
    p_flush_start = p_dstbuf;
		for (uint32_t ii = 0u; ii < size; ++ii)
    {
			const uint32_t color = *p_buf16++;
			*p_dstbuf++ = (uint8_t) (((color & 0xF800) >> 8) | ((color & 0x8000) >> 13));
			*p_dstbuf++ = (uint8_t) ((color & 0x07E0) >> 3);
			*p_dstbuf++ = (uint8_t) (((color & 0x001F) << 3) | ((color & 0x0010) >> 2));
    }
  }
  else
  {
    ESP_LOGE(TAG, "px_map is not equal to any buffer start");
    return;
  }

  /* Column addresses  */
  uint8_t xb[] = {
      (uint8_t)(area->x1 >> 8) & 0xFF,
      (uint8_t)(area->x1) & 0xFF,
      (uint8_t)(area->x2 >> 8) & 0xFF,
      (uint8_t)(area->x2) & 0xFF};

  /* Page addresses  */
  uint8_t yb[] = {
      (uint8_t)(area->y1 >> 8) & 0xFF,
      (uint8_t)(area->y1) & 0xFF,
      (uint8_t)(area->y2 >> 8) & 0xFF,
      (uint8_t)(area->y2) & 0xFF};

  /*Column addresses*/
  ili9488_send_cmd(ILI9488_CMD_COLUMN_ADDRESS_SET);
  ili9488_send_data(xb, 4);

  /*Page addresses*/
  ili9488_send_cmd(ILI9488_CMD_PAGE_ADDRESS_SET);
  ili9488_send_data(yb, 4);

  /*Memory write*/
  ili9488_send_cmd(ILI9488_CMD_MEMORY_WRITE);
  ili9488_send_color((void *)p_flush_start, size * 3);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

static void ili9488_send_cmd(uint8_t cmd)
{
  disp_wait_for_pending_transactions();
  gpio_set_level(ILI9488_DC, 0); /*Command mode*/
  disp_spi_send_data(&cmd, 1);
}

static void ili9488_send_data(void *data, uint16_t length)
{
  disp_wait_for_pending_transactions();
  gpio_set_level(ILI9488_DC, 1); /*Data mode*/
  disp_spi_send_data(data, length);
}

static void ili9488_send_color(void *data, uint16_t length)
{
  disp_wait_for_pending_transactions();
  gpio_set_level(ILI9488_DC, 1); /*Data mode*/
  disp_spi_send_colors(data, length);
}

static void ili9488_set_orientation(uint8_t orientation)
{
  // ESP_ASSERT(orientation < 4);

  const char *orientation_str[] = {
      "PORTRAIT", "PORTRAIT_INVERTED", "LANDSCAPE", "LANDSCAPE_INVERTED"};

  ESP_LOGI(TAG, "Display orientation: %s", orientation_str[orientation]);

#if defined(CONFIG_LV_PREDEFINED_DISPLAY_NONE)
  uint8_t data[] = {0x48, 0x88, 0x28, 0xE8};
#endif

  ESP_LOGI(TAG, "0x36 command value: 0x%02X", data[orientation]);

  ili9488_send_cmd(0x36);
  ili9488_send_data((void *)&data[orientation], 1);
}
