#pragma once
// Code inspired by: https://github.com/KamranAghlami/T-Display-S3/blob/main/src/hardware/display.h

#include <memory>

#include <esp_lcd_panel_io.h>

#include "hardware_setup.h"

// https://github.com/Xinyuan-LilyGO/T-Display-S3/issues/74
class Backlight {
    uint8_t pin;
    uint8_t level = 0;

public:
    /// @brief Constructs a backlight by configuring the corresponding pin as an output
    /// @param pin The physical pin responsible for the backlight 
    Backlight(uint8_t pin) : pin(pin) {
        pinMode(pin, OUTPUT);
    };

    /// @brief Deconstructs the backlight by configuring its pin as an input
    ~Backlight() {
        pinMode(pin, INPUT);
    };

    /// @brief Sets the brightness of the backlight 
    /// @param level A brightness level in the range [0; 255]. The perceived brightness is not a linear function of this value, however. A significant jump in visibility happens around a value of 45.
    void set_brightness(uint8_t level);
};

struct lcd_message
{
    uint8_t command;
    std::vector<uint8_t> parameters;
    size_t length;
};

// Display implemented as a Meyers' Singleton: https://www.modernescpp.com/index.php/creational-patterns-singleton/
class Display
{
    lv_display_t * display = nullptr;
    std::pair<void*, void*> draw_buffers = {nullptr, nullptr};

    esp_lcd_i80_bus_handle_t i80_bus = nullptr;
    esp_lcd_panel_io_handle_t io_handle = nullptr;
    esp_lcd_panel_handle_t panel_handle = nullptr;

    Backlight backlight = Backlight(PIN_LCD_BL);

    uint16_t width = LCD_VER_RES;
    uint16_t height = LCD_HOR_RES;

    /// @brief Initialize display
    Display();

    /// @brief Initializes LVGL, sets up logging and a tick timer. Also creates the lv_display_t* display member
    void initialize_lvgl();

    /// @brief Sets up communication with the display and configures it
    void initialize_hardware();

    /// @brief Get display instance
    static Display &get();

    /// @brief Deinitialize display
    ~Display();

public:
    Display(const Display&) = delete;              // No copying
    Display(Display&&) = delete;                   // No moving
    Display& operator = (const Display&) = delete; // No copy assignment
    Display& operator = (Display&&) = delete;      // No move assignment

    /// @brief Initialize display and LVGL
    static void init();

    /// @return Width of the display
    uint16_t get_width();

    /// @return Height of the display
    uint16_t get_height();

    
    /// @brief Sets the brightness of the backlight
    /// @param level A brightness level in the range [0; 255]. The perceived brightness is not a linear function of this value, however. A significant jump in visibility happens around a value of 45.
    static void set_backlight(uint8_t level);

    /// @brief Sends image data to the display. The given coordinates are inclusive. I.e., an x-range [x_start; x_end] = [0; 5] is 6 pixels wide (0, 1, 2, 3, 4, 5)
    /// @param x_start Left target coordinate
    /// @param x_end Right target coordinate
    /// @param y_start Top target coordinate
    /// @param y_end Bottom target coordinate
    /// @param data The image data
    static void draw_image(uint16_t x_start, uint16_t x_end, uint16_t y_start, uint16_t y_end, uint16_t* data);

    /// @brief Sends a command to the display
    /// @param message The command to send and its parameters
    void send_command(const lcd_message& message);
};
