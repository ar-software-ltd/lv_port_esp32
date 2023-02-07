/**
 * @file st7789.c
 *
 * Mostly taken from lbthomsen/esp-idf-littlevgl github.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "esp_log.h"

#include "st7789.h"

#include "disp_spi.h"
#include "driver/gpio.h"

/*********************
 *      DEFINES
 *********************/
#define TAG "st7789"

#if defined CONFIG_LV_DISPLAY_USE_SPI_CS
    #define USE_SOFTWARE_CS   0
#else
    #define USE_SOFTWARE_CS   1
#endif

#if USE_SOFTWARE_CS == 1
    /// @note Do not control CS. CS permanently grounded
    #define CS_SET(val)     // gpio_set_level(ST7789_CS, val)
#else
    #define CS_SET(val)
#endif

/// SPI frequency for LCD reading according the documentation
#define SPI_TFT_SLOW_CLOCK_SPEED_HZ     (5 * 1000 * 1000)

/**********************
 *      TYPEDEFS
 **********************/

/*The LCD needs a bunch of command/argument values to be initialized. They are stored in this struct. */
typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;

/**********************
 *  STATIC PROTOTYPES
 **********************/

static void st7789_cs_config(void);
static void st7789_dc_config(void);
static void st7789_rst_config(void);
static void st7789_bckl_config(void);

static void st7789_send_cmd(uint8_t cmd);
static void st7789_send_data(void *data, uint16_t length);
static void st7789_send_color(void *data, uint16_t length);

static void st7789_clear();
static void st7789_set_orientation(uint8_t orientation);

/**********************
 *  STATIC VARIABLES
 **********************/

const st7789_id_t default_display_id = {
    .u32_value = 0x00528585
};
static bool send_color_use_spi_queued = true;

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
void st7789_init(void)
{
    lcd_init_cmd_t st7789_init_cmds[] = {
        {0xCF, {0x00, 0x83, 0X30}, 3},
        {0xED, {0x64, 0x03, 0X12, 0X81}, 4},
        {ST7789_PWCTRL2, {0x85, 0x01, 0x79}, 3},
        {0xCB, {0x39, 0x2C, 0x00, 0x34, 0x02}, 5},
        {0xF7, {0x20}, 1},
        {0xEA, {0x00, 0x00}, 2},
        {ST7789_LCMCTRL, {0x2c}, 1},
        {ST7789_VCMOFSET, {0x35, 0x3E}, 2},
        {ST7789_CABCCTRL, {0xBE}, 1},
        {ST7789_MADCTL, {0x00}, 1}, // Set to 0x28 if your display is flipped
        {ST7789_COLMOD, {0x05}, 1},
        {0xb2, {0x0c, 0x0c, 0x00, 0x33, 0x33}, 5},
        {0xbb, {0x39}, 1},
        {0xc2, {0x01}, 1},
        {0xc3, {0x10}, 1},
        {0xc4, {0x20}, 1},
        {0xc6, {0x0f}, 1},
        {0xd0, {0xa4, 0xa1}, 2},
        {0xd6, {0xa1}, 1},
	{ST7789_INVON, {0}, 0},
        {ST7789_RGBCTRL, {0x00, 0x1B}, 2},
        {0xF2, {0x08}, 1},
        {ST7789_GAMSET, {0x01}, 1},
        {ST7789_PVGAMCTRL, {0x0d, 0x0f, 0x11, 0x07, 0x05, 0x02, 0x28, 0x33, 0x3f, 0x26, 0x14, 0x15, 0x24, 0x28}, 14},
        {ST7789_NVGAMCTRL, {0x0d, 0x0e, 0x11, 0x07, 0x05, 0x02, 0x28, 0x22, 0x3f, 0x2a, 0x18, 0x19, 0x26, 0x28}, 14},
        {ST7789_CASET, {0x00, 0x00, 0x00, 0xEF}, 4},
        {ST7789_RASET, {0x00, 0x00, 0x01, 0x3f}, 4},
        {ST7789_RAMWR, {0}, 0},
        {ST7789_GCTRL, {0x35}, 1},
        {0xB6, {0x0A, 0x82, 0x27, 0x00}, 4},
        {ST7789_SLPOUT, {0}, 0x80},
        {0, {0}, 0xff},
    };

    //Initialize non-SPI GPIOs

    st7789_cs_config();
    st7789_dc_config();
    st7789_rst_config();
    st7789_bckl_config();

    //Reset the display
    st7789_hw_reset();

    //Send all the commands
    uint16_t cmd = 0;
    while (st7789_init_cmds[cmd].databytes!=0xff) {
        st7789_send_cmd(st7789_init_cmds[cmd].cmd);
        st7789_send_data(st7789_init_cmds[cmd].data, st7789_init_cmds[cmd].databytes&0x1F);
        if (st7789_init_cmds[cmd].databytes & 0x80) {
                vTaskDelay(100 / portTICK_RATE_MS);
        }
        cmd++;
    }

    st7789_clear();

    st7789_disp_on();
    vTaskDelay(100 / portTICK_RATE_MS);

    st7789_enable_backlight(true);

    st7789_set_orientation(CONFIG_LV_DISPLAY_ORIENTATION);
}

void st7789_enable_backlight(bool backlight)
{
#if ST7789_ENABLE_BACKLIGHT_CONTROL
    printf("%s backlight.\n", backlight ? "Enabling" : "Disabling");
    uint32_t tmp = 0;

#if (ST7789_BCKL_ACTIVE_LVL==1)
    tmp = backlight ? 1 : 0;
#else
    tmp = backlight ? 0 : 1;
#endif

    gpio_set_level(ST7789_BCKL, tmp);
#endif
}

/* The ST7789 display controller can drive 320*240 displays, when using a 240*240
 * display there's a gap of 80px, we need to edit the coordinates to take into
 * account that gap, this is not necessary in all orientations. */
void st7789_flush(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_map)
{
    uint8_t data[4] = {0};

    uint16_t offsetx1 = area->x1;
    uint16_t offsetx2 = area->x2;
    uint16_t offsety1 = area->y1;
    uint16_t offsety2 = area->y2;

#if (CONFIG_LV_TFT_DISPLAY_OFFSETS)
    offsetx1 += CONFIG_LV_TFT_DISPLAY_X_OFFSET;
    offsetx2 += CONFIG_LV_TFT_DISPLAY_X_OFFSET;
    offsety1 += CONFIG_LV_TFT_DISPLAY_Y_OFFSET;
    offsety2 += CONFIG_LV_TFT_DISPLAY_Y_OFFSET;

#elif (LV_HOR_RES_MAX == 240) && (LV_VER_RES_MAX == 240)
#if (CONFIG_LV_DISPLAY_ORIENTATION_PORTRAIT)
    offsetx1 += 80;
    offsetx2 += 80;
#elif (CONFIG_LV_DISPLAY_ORIENTATION_LANDSCAPE_INVERTED)
    offsety1 += 80;
    offsety2 += 80;
#endif
#endif

    /*Column addresses*/
    st7789_send_cmd(ST7789_CASET);
    data[0] = (offsetx1 >> 8) & 0xFF;
    data[1] = offsetx1 & 0xFF;
    data[2] = (offsetx2 >> 8) & 0xFF;
    data[3] = offsetx2 & 0xFF;
    st7789_send_data(data, 4);

    /*Page addresses*/
    st7789_send_cmd(ST7789_RASET);
    data[0] = (offsety1 >> 8) & 0xFF;
    data[1] = offsety1 & 0xFF;
    data[2] = (offsety2 >> 8) & 0xFF;
    data[3] = offsety2 & 0xFF;
    st7789_send_data(data, 4);

    /*Memory write*/
    st7789_send_cmd(ST7789_RAMWR);

    uint32_t size = lv_area_get_width(area) * lv_area_get_height(area);

    st7789_send_color((void*)color_map, size * 2);

}

void st7789_hw_reset(void) {
    gpio_set_level(ST7789_RST, 0);
    vTaskDelay(60 / portTICK_RATE_MS);
    gpio_set_level(ST7789_RST, 1);
    vTaskDelay(140 / portTICK_RATE_MS);
}

void st7789_nop(void)
{
    st7789_send_cmd(ST7789_NOP);
}

void st7789_sw_reset(void)
{
    st7789_send_cmd(ST7789_SWRESET);
}

st7789_id_t st7789_get_id(void)
{
    // According to IDF docs SPI rx buffer must be multiple of 4 bytes and aligned
    uint8_t buff_rx[4] __aligned(4) = {0};
    uint8_t cmd = ST7789_RDDID;

    disp_wait_for_pending_transactions();

    // Set SPI slow clock
    disp_spi_change_device_speed(SPI_TFT_SLOW_CLOCK_SPEED_HZ);

    CS_SET(0);

    gpio_set_level(ST7789_DC, 0);
    disp_spi_send_data(&cmd, 1);

    disp_wait_for_pending_transactions();
    gpio_set_level(ST7789_DC, 1);

    // Receive transaction is devided by 2 transactions
    // The first transaction transmits 1 dummy clock and then receives first byte
    disp_spi_transaction(NULL, 1, (DISP_SPI_RECEIVE | DISP_SPI_SEND_POLLING | DISP_SPI_VARIABLE_DUMMY), &buff_rx[0], 0, 1);
    // The second transaction receives other bytes
    disp_spi_transaction(NULL, 2, (DISP_SPI_RECEIVE | DISP_SPI_SEND_POLLING), &buff_rx[1], 0, 0);

    CS_SET(1);

    // Set SPI default clock
    disp_spi_change_device_speed(SPI_TFT_CLOCK_SPEED_HZ);

    st7789_id_t id = {
        .u32_value = *(uint32_t *)buff_rx
    };

    return id;
}

void st7789_disp_on(void)
{
    st7789_send_cmd(ST7789_DISPON);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/
static void st7789_send_cmd(uint8_t cmd)
{
    disp_wait_for_pending_transactions();

    CS_SET(0);
    gpio_set_level(ST7789_DC, 0);
    disp_spi_send_data(&cmd, 1);
    CS_SET(1);
}

static void st7789_send_data(void * data, uint16_t length)
{
    disp_wait_for_pending_transactions();
    
    CS_SET(0);
    gpio_set_level(ST7789_DC, 1);
    disp_spi_send_data(data, length);
    CS_SET(1);
}

static void st7789_send_color(void * data, uint16_t length)
{
    disp_wait_for_pending_transactions();

    CS_SET(0);
    gpio_set_level(ST7789_DC, 1);

    if (send_color_use_spi_queued) {
        disp_spi_send_colors(data, length);
    }
    else {
        disp_spi_send_colors_polling(data, length);
    }
    
    CS_SET(1);
}

static void st7789_set_orientation(uint8_t orientation)
{
    // ESP_ASSERT(orientation < 4);

    const char *orientation_str[] = {
        "PORTRAIT", "PORTRAIT_INVERTED", "LANDSCAPE", "LANDSCAPE_INVERTED"
    };

    ESP_LOGI(TAG, "Display orientation: %s", orientation_str[orientation]);

    uint8_t data[] = 
    {
#if CONFIG_LV_PREDEFINED_DISPLAY_TTGO
	0x60, 0xA0, 0x00, 0xC0
#else
	0xC0, 0x00, 0x60, 0xA0
#endif
    };
    
    ESP_LOGI(TAG, "0x36 command value: 0x%02X", data[orientation]);

    st7789_send_cmd(ST7789_MADCTL);
    st7789_send_data((void *) &data[orientation], 1);
}

static void st7789_cs_config(void) {
#if USE_SOFTWARE_CS == 1
    gpio_pad_select_gpio(ST7789_CS);
    gpio_set_direction(ST7789_CS, GPIO_MODE_OUTPUT);
    /// @note CS permanently grounded
    gpio_set_level(ST7789_CS, 0);
#endif
}

static void st7789_dc_config(void) {
    gpio_pad_select_gpio(ST7789_DC);
    gpio_set_direction(ST7789_DC, GPIO_MODE_OUTPUT);
}

static void st7789_rst_config(void) {
    gpio_pad_select_gpio(ST7789_RST);
    gpio_set_direction(ST7789_RST, GPIO_MODE_OUTPUT);
}

static void st7789_bckl_config(void) {
#if ST7789_ENABLE_BACKLIGHT_CONTROL
    gpio_pad_select_gpio(ST7789_BCKL);
    gpio_set_direction(ST7789_BCKL, GPIO_MODE_OUTPUT);
#endif
}

static void st7789_clear() {
    lv_color_t color[LV_HOR_RES_MAX] = {0};
    lv_area_t area = {
        .x1 = 0,
        .x2 = LV_HOR_RES_MAX - 1
    };

    // Need use polling SPI method
    send_color_use_spi_queued = false;

    for (lv_coord_t y = 0; y < LV_VER_RES_MAX; y++) {
        area.y1 = y;
        area.y2 = y;

        st7789_flush(NULL, &area, color);
    }

    send_color_use_spi_queued = true;
}
