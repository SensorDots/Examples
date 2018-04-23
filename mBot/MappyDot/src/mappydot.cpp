/**
   MappyDot mDot Extension

   Copyright (C) 2017  Blair Wyatt

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#include "mappydot.h"
#include "Wire.h"

void mm_to_bytes(uint8_t *bytes, uint16_t mm)
{
    bytes[0] = (mm >> 8 & 0xFF);
    bytes[1] = (mm & 0xFF);
}

MappyDot::MappyDot(uint8_t mappydot_address){
	delay(2000);
	Wire.begin();
	address = mappydot_address;
}

uint16_t MappyDot::getDistance()
{
	Wire.requestFrom(address, 2); 
	uint16_t distance = Wire.read() << 8; distance |= Wire.read(); 
	return distance;
}