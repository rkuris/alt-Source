
//      Flash.h
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


#ifndef _FLASH_H_
#define _FLASH_H_

#include <Arduino.h>
#include "Config.h"                                               // Pick up the specific structures and their sizes for this program.
#include "Sensors.h"
#include "CPE.h"   
#include "Alternator.h"


void transfer_default_CPS(uint8_t index, CPS  *cpsPtr); 
void write_CPS_EEPROM(uint8_t index, CPS *cpsPtr); 
void write_SCS_EEPROM(SCS *scsPtr); 
void write_CAL_EEPROM(CAL *calPtr); 
bool read_CPS_EEPROM(uint8_t index, CPS *cpsPtr); 
bool read_SCS_EEPROM(SCS *scsPtr);
bool read_CAL_EEPROM(CAL *calPtr);
void restore_all(void);
void commit_EEPROM(void);

#ifdef SYSTEMCAN  
 void write_CCS_EEPROM(CCS *ccsPtr); 
 bool read_CCS_EEPROM(CCS *ccsPtr);
 #endif





                //----- The following are used during the management of the EEPROM, to validate saved data
                //      If a structure format is changed, change these keys to help invalidate prior saved data in the CPU.

#define SCS_ID1_K  0xFC3A                                       // Key value that should be contained in the EEPROM EKEY structure to indicate a valid sytemConfig structure has been saved
#define SCS_ID2_K  0x69D3
#define CPS_ID1_K  0xB66C                                       // Key value that should be continued in the EEPROM EKEY structure to indicate a valid CPE structure has been saved
#define CPS_ID2_K  0x0A47                                       // (Changed in 0.2.0 - as CPS was expanded)
#define CAL_ID1_K  0x1FAC                                       // Calibration Structure
#define CAL_ID2_K  0x0A97
#define CCS_ID1_K  0x813A                                       // CAN structure                
#define CCS_ID2_K  0xC03A




                //-----  EEPROM is laid out in this way:  (I was not able to get #defines to work, as the preprocessor seems to not be able to handle sizeof() )
                //          EEPROM_EKEY @ 0                                  
                //          EEPROM_SCS  @ sizeof(EKEY)
                //          EEPROM_CPS  @ sizeof(EKEY)  + sizeof(SCS)
                //          EEPROM_CAL  @ sizeof(EKEY)  + sizeof(SCS)  + sizeof(CPS)*MAX_CPES
                //          EEPROM_CCS  @ sizeof(EKEY)  + sizeof(SCS)  + sizeof(CPS)*MAX_CPES + sizeof(CAL)
                
                

#define  EKEY_FLASH_LOCAITON  0
#define  SCS_FLASH_LOCAITON  (sizeof(EKEY))
#define  CPS_FLASH_LOCAITON  (sizeof(EKEY) + sizeof(SCS) + (sizeof(CPS)*index))
#define  CAL_FLASH_LOCAITON  (sizeof(EKEY) + sizeof(SCS) + (sizeof(CPS)*MAX_CPES)) 
#define  CCS_FLASH_LOCAITON  (sizeof(EKEY) + sizeof(SCS) + (sizeof(CPS)*MAX_CPES)  + sizeof(CAL))



                                                            
typedef struct EKEY {                                           // EEPROM Key Structure - used to validate storage of CPS and SCV structures in the EEPROM

   unsigned     SCS_ID1;                                        // Unique bit pattern that must match to indicate an systemConfig structure has been placed
   unsigned     SCS_ID2;                                        // into the EEPROM at one time.
   unsigned     CPS_ID1[MAX_CPES];                              // Unique bit pattern that must match to indicate an ChargeParms structure has been placed
   unsigned     CPS_ID2[MAX_CPES];                              // into the EEPROM at one time.
   unsigned     CAL_ID1;                                        // Calibration structure
   unsigned     CAL_ID2;                                      
   
    #ifdef SYSTEMCAN  
   unsigned     CCS_ID1; 
   unsigned     CCS_ID2; 
   
   unsigned long CCS_CRC32;                                     //  CRC-32 of last stored CANConfig structure
   #endif
   unsigned long SCS_CRC32;                                     //  CRC-32 of last stored SystemConfig structure
   unsigned long CPS_CRC32[MAX_CPES];                           //  CRC-32 of last stored ChargeParms structure
   unsigned long CAL_CRC32;                                     //  CRC-32 of last stored ADCCal structure
   
   } EKEY;
                                                        






#endif  //_FLASH_H_





