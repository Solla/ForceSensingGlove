#ifndef CD74HC4067_h
#define CD74HC4067_h
#include "Arduino.h"
class CD74HC4067
{
	public:
		CD74HC4067(int pin0, int pin1, int pin2, int pin3);
		void channel(int channel);
	private:
		int _pins[4];
};
#endif
