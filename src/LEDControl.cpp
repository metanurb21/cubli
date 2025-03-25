#include "LEDControl.h"
#include "setup.h"

void initLEDs()
{
	FastLED.addLeds<CHIPSET, LED_DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS);
	FastLED.setMaxPowerInVoltsAndMilliamps(VOLTS, MAX_AMPS);
	FastLED.setBrightness(BRIGHTNESS);
	FastLED.clear();
	FastLED.show();
}

void setLEDColor(int index, CRGB color)
{
	if (index >= 0 && index < NUM_LEDS)
	{
		leds[index] = color;
		FastLED.show();
	}
}

void clearLEDs()
{
	FastLED.clear();
	FastLED.show();
}
