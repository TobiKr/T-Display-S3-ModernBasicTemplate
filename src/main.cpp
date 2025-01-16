#include <Arduino.h>
#include <lvgl.h>

#include "display.h"
#include "hardware_setup.h"

uint64_t count = 0;
lv_obj_t* display_text;

void ui_init()
{
    const auto screen = lv_screen_active();
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x003a57), LV_PART_MAIN);

    display_text = lv_label_create(screen);
    lv_label_set_text(display_text, "Hello, world!");
    lv_obj_set_style_text_font(display_text, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(screen, lv_color_hex(0xffffff), LV_PART_MAIN);
    lv_obj_align(display_text, LV_ALIGN_CENTER, 0, 0);
}

void setup() {
    Serial.setTxTimeoutMs(0); // Fix slow-down without serial connection. See: https://github.com/espressif/arduino-esp32/issues/6983#issuecomment-1346941805
    Serial.begin(115200);
    sleep(1); // Wait a bit for Serial to become ready. Printing immediately doesn't work
    Serial.println("Starting...");
    
    Display::init();
    ui_init();
}

void loop() {
    lv_timer_periodic_handler(); // Need to call this here, since LVGL is not thread-safe. See Display::initialize_lvgl
    String psram_size = "PSRAM Size: " + String(ESP.getPsramSize() / 1024) + "KB";
    String psram_free = "PSRAM free: " + String(ESP.getFreePsram() / 1024) + "KB";

    Serial.println(psram_size);
    Serial.println(psram_free);
    
    Serial.print(count);
    Serial.print(": ");
    Serial.println("Oi!");
    ++count;

    std::array messages{psram_size.c_str(), psram_free.c_str(), "Oi!", "Bonjour!", "Hanloha!", "Ahoy!", "Aloha!", "Howdy!", "Hi-diddly-ho!"};
    lv_label_set_text(display_text, messages[count % messages.size()]);

    delay(1000);
}