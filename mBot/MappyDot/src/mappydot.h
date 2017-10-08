#ifndef mappydot_h
#define mappydot_h

#include <Arduino.h>
///@brief Class for MappyDot
class MappyDot
{
	public:
		MappyDot(uint8_t mappydot_address);
		uint16_t getDistance();
	private:
	    uint8_t address;
		
};

#endif