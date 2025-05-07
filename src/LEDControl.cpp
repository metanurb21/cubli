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
		// for (int i = 0; i < NUM_LEDS; i++)
		// {
		leds[0] = color;
		FastLED.show();
		if (delay)
		{
			FastLED.delay(80);
		}
		// }

		// if (delay)
		// {
		// 	FastLED.delay(duration);
		// 	clearLEDs();
		// }
	}

	void fadeLEDs(CRGB color)
	{
		if (fade_led_toggle)
		{
			if (millis() - lastIncrementLEDTime >= 5)
			{

				leds[0] = color;
				// leds[0].maximizeBrightness(FastLED_fade_counter);
				leds[0].fadeToBlackBy(255 - FastLED_fade_counter);

				FastLED.show();
				FastLED_fade_counter++;
				lastIncrementLEDTime = millis();
				if (FastLED_fade_counter >= 255)
				{
					fade_led_toggle = false;
				}
			}
		}
		else
		{
			if (millis() - lastIncrementLEDTime >= 5)
			{

				leds[0] = color;
				/// leds[0].maximizeBrightness(FastLED_fade_counter);
				leds[0].fadeLightBy(255 - FastLED_fade_counter);

				FastLED.show();
				FastLED_fade_counter--;
				lastIncrementLEDTime = millis();
				if (FastLED_fade_counter <= 0)
				{
					fade_led_toggle = true;
				}
			}
		}
	}

	void clearLEDs()
	{
		FastLED.clear();
		FastLED.show();
	}
}