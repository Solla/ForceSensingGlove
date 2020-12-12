#include "CD74HC4067.h"

CD74HC4067::CD74HC4067(int pin0, int pin1, int pin2, int pin3)
{
	pinMode((_pins[0] = pin0), OUTPUT);
	pinMode((_pins[1] = pin1), OUTPUT);
	pinMode((_pins[2] = pin2), OUTPUT);
	pinMode((_pins[3] = pin3), OUTPUT);
}

void CD74HC4067::channel(int channel)
{
	for (int i = 0; i < 4; ++i)
	{
		digitalWrite(_pins[i], channel & 1);
		channel >> 1;
	}
}
