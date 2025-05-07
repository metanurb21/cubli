#ifndef LEDCONTROL_H
#define LEDCONTROL_H

#include <FastLED.h>

#define NUM_LEDS 1 // 9
#define LED_DATA_PIN 19
#define COLOR_ORDER GRB
#define CHIPSET WS2812B
#define BRIGHTNESS 255
#define VOLTS 3.3
#define MAX_AMPS 500 // milliamps

namespace LEDControl
{
	void initLEDs();
	void setLEDColor(int index, CRGB color);
	void setAllLEDs(CRGB color, bool delay, uint32_t duration);
	void clearLEDs();
	void fadeLEDs(CRGB color);
}
#endif // LEDCONTROL_H
