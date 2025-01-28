#ifndef DISPLAY_HPP
#define DISPLAY_HPP

#include <WiFi.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET    -1
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
extern Adafruit_SSD1306 display; 


struct DisplayStatus {
    int num_motors;
    String ip_address;
    bool soft_error;
};

extern DisplayStatus status;


void setup_display();
void print_msg(String msg);
void display_status_bar(String msg);


#endif // DISPLAY_HPP