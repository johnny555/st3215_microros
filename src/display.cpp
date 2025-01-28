
#include "display.hpp"
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

extern DisplayStatus status = {0, "Not Connected", false};


void print_msg(String msg)
{
    if (status.num_motors > 0 || status.ip_address != "Not Connected") {
        display_status_bar(msg);
    } else {
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0,0);
        display.print(msg);
        display.display();
    }
}

void display_status_bar(String msg) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print("M:");
    display.print(status.num_motors);
    display.print(" ");
    display.print(status.ip_address);
    
    if (status.soft_error) {
        display.print(" Err");
    }
    display.print("\n");
    display.print(msg);
    display.display();
}


void setup_display() {
    display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
    display.display();
    display.clearDisplay();
}