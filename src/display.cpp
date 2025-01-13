// Code inspired by: https://github.com/KamranAghlami/T-Display-S3/blob/main/src/hardware/display.cpp

#include <Arduino.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h> // Create LCD panel for model ST7789
#include <lvgl.h>

#include "display.h"
#include "hardware_setup.h"

Display::Display()
{
    // Initialize lvgl here to create display (required to initialize hardware)
    initialize_lvgl();

    // Power on and initialize hardware 
    pinMode(PIN_LCD_POWER, OUTPUT);
    digitalWrite(PIN_LCD_POWER, HIGH);

    pinMode(PIN_LCD_RD, OUTPUT);
    digitalWrite(PIN_LCD_RD, HIGH);

    initialize_hardware();

    backlight.set_brightness(50);
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
}

void Display::initialize_lvgl()
{
    if (!lv_is_initialized())
    {
        lv_init();
    }

    // Set up logging
    #ifdef LV_USE_LOG
        lv_log_register_print_cb([](lv_log_level_t level, const char *message)
        {
            LV_UNUSED(level);
            Serial.println(message);
            // Serial.flush();
        });
    #endif

    // Create timer
    // https://github.com/espressif/esp-idf/blob/b3f7e2c8a4d354df8ef8558ea7caddc07283a57b/examples/peripherals/lcd/i80_controller/main/i80_controller_example_main.c#L473
    // LVGL is not thread-safe. It is fine to call lv_tick_inc in a timer interrupt, but lv_timer_handler may not be called simultaneously with other lvgl functions. 
    // Creating thread-safety by use of e.g., a lock, is not feasible, since everything in LVGL is done with pointers. I.e., you can't protect the screen from being accessed, since a new pointer to the screen can simply be spawned out of nowhere using lv_screen_active().
    // See:
    // - https://docs.lvgl.io/master/porting/tick.html
    // - https://docs.lvgl.io/master/porting/timer_handler.html
    // - https://docs.lvgl.io/master/porting/os.html
    // - https://www.reddit.com/r/esp32/comments/1az731x/how_do_timer_interrupts_and_their_priority_work/kskwgcc/
    // - https://forum.lvgl.io/t/lv-inv-area-asserted-at-expression-disp-rendering-in-progress-invalidate-area-is-not-allowed-during-rendering-lv-refr-c-257/14856/6
    // - https://forum.lvgl.io/t/can-i-safely-call-lv-timer-handler-less-often/14900/3
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = [](void *arg)
        {
            lv_tick_inc(LVGL_TICK_PERIOD_MS);
        },
        .name = "lvgl_tick"};
    esp_timer_handle_t lvgl_tick_timer = nullptr;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000));

    display = lv_display_create(height, width);

    // Create bufferydoos
    // It's recommended to choose the size of the draw buffer(s) to be at least 1/10 screen sized
    // https://github.com/lvgl/lvgl/blob/5ce363464fc81dec5378fc02a341b35786f0946b/docs/integration/driver/display/lcd_stm32_guide.rst
    // https://github.com/espressif/esp-idf/blob/b3f7e2c8a4d354df8ef8558ea7caddc07283a57b/examples/peripherals/lcd/i80_controller/main/i80_controller_example_main.c#L453
    // https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/system/mm_sync.html#memory-allocation-helper
    
    const auto buffer_size = height * width * lv_color_format_get_size(lv_display_get_color_format(display));

    draw_buffers.first = heap_caps_malloc(buffer_size, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!draw_buffers.first)
    {
        LV_LOG_ERROR("display draw buffer malloc failed");
        return;
    }

    draw_buffers.second = heap_caps_malloc(buffer_size, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!draw_buffers.second)
    {
        LV_LOG_ERROR("display buffer malloc failed");
        lv_free(draw_buffers.first);
        return;
    }

    lv_display_set_buffers(display, draw_buffers.first, draw_buffers.second, buffer_size, LV_DISPLAY_RENDER_MODE_PARTIAL);

    const auto flush_callback = [](lv_display_t *disp, const lv_area_t *area, uint8_t *pix_map)
    {
        uint16_t* data = reinterpret_cast<uint16_t*>(pix_map);
        lv_draw_sw_rgb565_swap(data, (area->x2 - area->x1 + 1) * (area->y2 - area->y1 + 1));
        Display::draw_image(area->x1, area->x2, area->y1, area->y2, data);
    };

    lv_display_set_flush_cb(display, flush_callback);
}

void Display::initialize_hardware()
{
    // Create bus for I80 LCD (Intel 8080 parallel LCD) interfaced display
    // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/lcd.html#i80-lcd-panel
    // https://github.com/Xinyuan-LilyGO/T-Display-S3/blob/main/example/lv_demos/lv_demos.ino
    // 1. Set up Intel 8080 parallel bus
    const esp_lcd_i80_bus_config_t bus_config = {
        .dc_gpio_num = PIN_LCD_DC,      // GPIO number of the data/command select pin (aka RS)
        .wr_gpio_num = PIN_LCD_WR,      // GPIO number of the pixel clock (aka WR)
        .clk_src = LCD_CLK_SRC_PLL160M, // Clock source of the bus
        .data_gpio_nums = {
            // Array of GPIO numbers of the data bus. Number of elements equal to bus_width
            PIN_LCD_D0,
            PIN_LCD_D1,
            PIN_LCD_D2,
            PIN_LCD_D3,
            PIN_LCD_D4,
            PIN_LCD_D5,
            PIN_LCD_D6,
            PIN_LCD_D7,
        },
        .bus_width = 8,                                             // Width of the data bus
        .max_transfer_bytes = LVGL_LCD_BUF_SIZE * sizeof(uint16_t), // Transfer full buffer of pixels (assume pixel is RGB565) at most in one transaction
    };
    ESP_ERROR_CHECK(esp_lcd_new_i80_bus(&bus_config, &i80_bus));

    // 2. Allocate LCD IO device handle from I80 bus
    const auto pixel_transfer_callback = [](esp_lcd_panel_io_handle_t _panel_io, esp_lcd_panel_io_event_data_t* _event_data, void *user_ctx)
    {
        auto display = static_cast<lv_display_t*>(user_ctx);
        lv_display_flush_ready(display);
        return false;
    };

    const esp_lcd_panel_io_i80_config_t io_config = {
        .cs_gpio_num = PIN_LCD_CS, // GPIO number of chip select pin
        .pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ, // Pixel clock frequency in Hz. Higher pixel clock frequency results in higher refresh rate, but may cause flickering if the DMA bandwidth is not sufficient or the LCD controller chip does not support high pixel clock frequency.
        .trans_queue_depth = 20,// Maximum number of transactions that can be queued in the LCD IO device. A bigger value means more transactions can be queued up, but it also consumes more memory.
        .on_color_trans_done = pixel_transfer_callback,
        .user_ctx = display,
        // Bit width of the command and parameters that are recognized by the LCD controller chip. This is chip specific.
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .dc_levels = {
            .dc_idle_level = 0,
            .dc_cmd_level = 0,
            .dc_dummy_level = 0,
            .dc_data_level = 1,
        },
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i80(i80_bus, &io_config, &io_handle));

    // 3. Install the LCD controller driver. It is responsible for sending the commands and parameters to the LCD controller chip
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PIN_LCD_RES, // GPIO number of the reset pin. Set to -1, if the LCD controller chip does not have a reset pin
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16, // Bit width of the pixel color data. Used for computing the number of bytes to send to the LCD controller chip
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));

    // 4. Adding manufacturer specific initialization 
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false));
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(panel_handle, 0, 35));

    // Set extra configurations e.g., gamma control with the underlying IO handle
    // Please consult your manufacturer for special commands and corresponding values
    // WARNING: Command descriptions given by ChatGPT, since the original code had no documentation for these magic constants
    // Set extra configurations e.g., gamma control with the underlying IO handle
    // Please consult your manufacturer for special commands and corresponding values
    // WARNING: Command descriptions given by ChatGPT, since the original code had no documentation for these magic constants
    
    const lcd_message init_messages[] = {
        {0x11, {0}, 0 | 0x80}, // Turn off display controller sleep mode. 0x80 indicates data command (not control command)
        {0x3A, {0X05}, 1}, // Specify 16-bit color pixel format (RGB565)
        {0xB2, {0X0B, 0X0B, 0X00, 0X33, 0X33}, 5}, // Adjust frame rate 
        {0xB7, {0X75}, 1}, // Configure gate driver timing 
        {0xBB, {0X28}, 1}, // Set VCOM voltage level
        {0xC0, {0X2C}, 1}, // Configure power supply parameters 
        {0xC2, {0X01}, 1}, // More power control settings
        {0xC3, {0X1F}, 1}, // More VCOM control settings
        {0xC6, {0X13}, 1}, // Set display interface parameters 
        {0xD0, {0XA7}, 1}, // More power control settings
        {0xD0, {0XA4, 0XA1}, 2}, // More power control settings
        {0xD6, {0XA1}, 1}, // Set display function control 
        {0xE0, {0XF0, 0X05, 0X0A, 0X06, 0X06, 0X03, 0X2B, 0X32, 0X43, 0X36, 0X11, 0X10, 0X2B, 0X32}, 14}, // Set gamma correction coefficients for positive gamma
        {0xE1, {0XF0, 0X08, 0X0C, 0X0B, 0X09, 0X24, 0X2B, 0X22, 0X43, 0X38, 0X15, 0X16, 0X2F, 0X37}, 14} // Set gamma correction coefficients for negative gamma 
    };

    for (size_t i = 0; i < sizeof(init_messages) / sizeof(init_messages[0]); i++) {
        const auto& message = init_messages[i];
        ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(io_handle, message.command, 
            message.parameters.data(), message.length & 0x7f));
        if (message.length & 0x80) {
            delay(120);
        }
    } 
}

Display::~Display()
{
    ESP_ERROR_CHECK(esp_lcd_panel_del(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_io_del(io_handle));
    ESP_ERROR_CHECK(esp_lcd_del_i80_bus(i80_bus));

    pinMode(PIN_LCD_RD, INPUT);
    pinMode(PIN_LCD_POWER, INPUT);
}

uint16_t Display::get_width()
{
    return width;
}

uint16_t Display::get_height()
{
    return height;
}

void Display::set_backlight(uint8_t level)
{
    get().backlight.set_brightness(level);
}

void Display::draw_image(uint16_t x_start, uint16_t x_end, uint16_t y_start, uint16_t y_end, uint16_t* data)
{
    auto& display = Display::get();
    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(display.panel_handle, x_start, y_start, x_end + 1, y_end + 1, data));
}

void Display::send_command(const lcd_message& message) {
    esp_lcd_panel_io_tx_param(io_handle, message.command, message.parameters.data(), message.length);
}

Display &Display::get()
{
    static Display instance;
    return instance;
}

void Display::init()
{
    Display::get();
}

void Backlight::set_brightness(uint8_t level) {
    analogWrite(pin, level);
}