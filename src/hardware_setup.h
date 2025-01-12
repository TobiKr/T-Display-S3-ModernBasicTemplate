#pragma once

// https://github.com/Xinyuan-LilyGO/T-Display-S3/blob/main/example/factory/pin_config.h

/*ESP32S3*/
// Backlight
#define PIN_LCD_BL                   38

// Data pins
#define PIN_LCD_D0                   39
#define PIN_LCD_D1                   40
#define PIN_LCD_D2                   41
#define PIN_LCD_D3                   42
#define PIN_LCD_D4                   45
#define PIN_LCD_D5                   46
#define PIN_LCD_D6                   47
#define PIN_LCD_D7                   48

// Display power
#define PIN_LCD_POWER                15

// Display control pins
#define PIN_LCD_RES                  5 // Reset
#define PIN_LCD_CS                   6 // Chip select
#define PIN_LCD_DC                   7 // Data/Command
#define PIN_LCD_WR                   8 // Write
#define PIN_LCD_RD                   9 // Read

#define PIN_BUTTON_1                 0  // "Bot" (boot) button
#define PIN_BUTTON_2                 14 // "Key" button
#define PIN_BAT_VOLT                 4 // Battery voltage measurement

// I2C communication
#define PIN_IIC_SCL                  17 // SCL
#define PIN_IIC_SDA                  18 // SDA

// Touch screen (not available)
#define PIN_TOUCH_INT                16 // Touch interrupt 
#define PIN_TOUCH_RES                21 // Touch reset

/* LCD CONFIG */
#define EXAMPLE_LCD_PIXEL_CLOCK_HZ   (6528000) //(10 * 1000 * 1000)
// The pixel number in horizontal and vertical
#define LCD_HOR_RES                  320
#define LCD_VER_RES                  170
#define LVGL_LCD_BUF_SIZE            (LCD_HOR_RES * LCD_VER_RES)
#define EXAMPLE_PSRAM_DATA_ALIGNMENT 64
#define LVGL_TICK_PERIOD_MS          2