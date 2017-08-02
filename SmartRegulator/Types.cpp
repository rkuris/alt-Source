//      Types.cpp
//
//      Copyright (c) 2016, 2017 by William A. Thomason.      http://arduinoalternatorregulator.blogspot.com/
//
//
//
//              This program is free software: you can redistribute it and/or modify
//              it under the terms of the GNU General Public License as published by
//              the Free Software Foundation, either version 3 of the License, or
//              (at your option) any later version.
//
//              This program is distributed in the hope that it will be useful,
//              but WITHOUT ANY WARRANTY; without even the implied warranty of
//              MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//              GNU General Public License for more details.
//
//              You should have received a copy of the GNU General Public License
//              along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
//

#include <Arduino.h>
#include "Types.h"

// floatString converts a float to a string
// We return one of a set of static buffers
char *floatString(float v, unsigned char decimals) {
    const int OUTPUT_BUFS = 6;  // maximum number of floats in a single sprintf
    const int MAX_OUTPUT = 13;  // largest possible output string
    static char outputBuffers[OUTPUT_BUFS][MAX_OUTPUT+1];
    int callCount;

    char *pos = outputBuffers[++callCount % OUTPUT_BUFS];
    char *opos = pos;
    if (v < 0) {
	    *(opos++) = '-';
	    v = -v;
    }
    int mult = 1;
    int multleft = decimals;
    while(multleft--) {
	    mult *= 10;
    }
    char format[9];
    snprintf_P(format, 9, PSTR("%%d.%%0%dd"), decimals);
    snprintf_P(opos, MAX_OUTPUT, format, (int)v, (int)((v - (int)v) * mult));
    return pos;
}
