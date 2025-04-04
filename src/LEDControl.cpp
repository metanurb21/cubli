#include "LEDControl.h"
#include "setup.h"
#include <FastLED.h>

namespace LEDControl
{
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

	void setAllLEDs(CRGB color, bool delay, uint32_t duration)
	{
		for (int i = 0; i < NUM_LEDS; i++)
		{
			leds[i] = color;
			FastLED.delay(80);
		}

		FastLED.show();
		if (delay)
		{
			FastLED.delay(duration);
			clearLEDs();
		}
	}

	void clearLEDs()
	{
		FastLED.clear();
		FastLED.show();
	}
}