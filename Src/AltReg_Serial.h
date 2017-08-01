
//      AltReg_Serial.h
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


#ifndef _ALTREG_SERIAL_H_
#define _ALTREG_SERIAL_H_

#include <Arduino.h>
#include "Config.h"
#include "CPE.h"    





                                //----- External communications, Baud rate, buffer sizes, timeouts, etc..
                                //
#define UPDATE_STATUS_RATE         1000UL               // Send an update of the Status (via Bluetooth / Serial port) every 1 seconds.
#define UPDATE_MAJOR_SENSITIVITY     60                 // Every 60 Update cycles, send the not-so-critical information.... (Max 126)
#define INBOUND_BUFF_SIZE            60                 // Size of input command buffer (CAUTION:  250 -- 8-bit indexes are used)
#define OUTBOUND_BUFF_SIZE          200                 // Large Outbound buffer, make sure we do not overrun (Primarily CPE;)(CAUTION:  250 -- 8-bit indexes are used)
#define IB_BUFF_FILL_TIMEOUT     60000UL                // If a complete 'command' string is not received within 60 seconds, abort it.  Set = 0 to disable this feature.
#define SDM_SENSITIVITY              10                 // Send Debug Material:  only update the Serial Port with debug info every 10 times through. 
                                                        // (Set = 1 for max updates, max value 126)





void check_inbound(void);
void send_outbound(bool pushAll);
int  frac2int (float frac, int limit);
 
void prep_AST(char *buffer);
void prep_CPE(char *buffer, CPS  *cpsPtr, int Index);
void prep_NPC(char *buffer);
void prep_CST(char *buffer);
void prep_SST(char *buffer);
void prep_SCV(char *buffer);



#endif  // _ALTREG_SERIAL_H_




