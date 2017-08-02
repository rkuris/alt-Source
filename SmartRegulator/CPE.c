//      CPE.c
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





#include "CPE.h"

     
     

                                                                //  Default parameters.  The selected row (as indicted by the DIP switches) is copied from the FLASH into the working "chargeParms" variable
                                                                //      during Startup().   Also, the EEPROM is checked to see if there is a replacement entry for the default values and if so, those are
                                                                //      copied. 
                                                                //
                                                                //      CAUTION:   ALL ENTRIES IN THIS TABLE ASSUMES A NOMINAL 12V / 500Ah - SMALL SIZED BATTERY SYSTEM.  
                                                                //                 THEY WILL BE AUTOMATICALLY ADJUSTED FOR HIGHER VOLTAGE / CAPACITY BATTERIES VIA THE AUTO-SENSING CAPABILITY.
                                                                //                 When populating this table, if you have a higher voltage battery (ala 48v), then make sure to scale
                                                                //                 these appropriately back to a '12v' value (ala, divide volts / 4, mult Amps by 4)
                                                                //
                                                                //      The CPEs below have been defined assuming the Amp shunt is INSTALLED AT THE BATTERY vs. at the Alternator.  If the regulator is configured
                                                                //      with the Amp shunt on the Alternator, exit_amp values need to be adjusted to accommodate house loads.  A suggestion is to add 5A to each 
                                                                //       'threshold' value, with assumption that ship running loads will be around 5A for a small boat.
                                                                //      These will scale with the Battery Size scaling switches, so that a larger boat with say a 1500AH battery, that 5A will turn into 15A.
                                                                //

const CPS PROGMEM defaultCPS[MAX_CPES] = {
        //      Bulk/Accpt                  Overcharge                               Float                          Post Float              Equalization                    Temp Comp                   
        {14.1, 6.0*3600000UL, 15,        0,  0.0,   0*3600000UL,        13.4,  -1, 0*3600000UL, -10,  0, 12.8,    0*3600000UL, 0.0, 0,     0.0,  0,   0*60000UL, 0,     0.004*6, -9, -45, 52},  // #1 Default (safe) profile & AGM #1 (Low VOltage AGM).
        {14.8, 3.0*3600000UL,  5,        0,  0.0,   0*3600000UL,        13.5,  -1, 0*3600000UL, -10,  0, 12.8,    0*3600000UL, 0.0, 0,     0.0,  0,   0*60000UL, 0,     0.005*6, -9, -45, 52},  // #2 Standard FLA (e.g. Starter Battery, small storage)
        {14.6, 4.5*3600000UL,  5,        0,  0.0,   0*3600000UL,        13.4,  -1, 0*3600000UL, -10,  0, 12.8,    0*3600000UL, 0.0, 0,    15.3,  0, 3.0*60000UL, 0,     0.005*6, -9, -45, 52},  // #3 HD FLA (GC, L16, larger)
        {14.7, 4.5*3600000UL,  3,        0,  0.0,   0*3600000UL,        13.4,  -1, 0*3600000UL, -10,  0, 12.8,    0*3600000UL, 0.0, 0,     0.0,  0,   0*60000UL, 0,     0.004*6, -9, -45, 52},  // #4 AGM #2 (Higher Voltage AGM)
        {14.1, 6.0*3600000UL,  5,        0,  0.0,   0*3600000UL,        13.5,  -1, 0*3600000UL, -10,  0, 12.8,    0*3600000UL, 0.0, 0,     0.0,  0,   0*60000UL, 0,     0.005*6, -9, -45, 52},  // #5 GEL
        {14.0, 1.0*3600000UL, 15,        0,  0.0,   0*3600000UL,        13.1,  -1, 0*3600000UL,   0,  0,  0.0,    0*3600000UL, 0.0, 0,     0.0,  0,   0*60000UL, 0,     0.000*6, -9, -45, 52},  // #6 RESERVED (place saver of very safe values)
        {14.4, 6.0*3600000UL, 15,       15, 15.3, 3.0*3600000UL,        13.1,  -1, 0*3600000UL, -10,  0, 12.8,    0*3600000UL, 0.0, 0,    15.3,  0, 3.0*60000UL, 0,     0.005*6, -9, -45, 52},  // #7 4-stage HD LFA (+ Custom #1 changeable profile)  
        {13.9, 1.0*3600000UL, 15,        0,  0.0,   0*3600000UL,        13.36,  0, 0*3600000UL,   0, 50, 13.3,    0*3600000UL, 0.0, 0,     0.0,  0,   0*60000UL, 0,     0.000*6,  0,   0, 45}   // #8 LiFeP04        (+ Custom #2 changeable profile)
        };

                                                                // Side note:  The Arduino programming environment will place the above populated table into EPROM during compile time.
                                                                //              Upon power on, the Startup code will copy the selected CPE entry (table row) into RAM for use. 
                                                                //
                                                                //              These values are normalized for a 500Ah/12v battery, and when used are adjusted as follows: 
                                                                //                      - Volts:   Increased (multiplied) by battery voltage scaling factor  - "systemVoltMult"
                                                                //                                 As determined by auto-sensing the battery voltage at startup (or forced override via ASCII command).
                                                                //                                 e.g.:  at 48v system would multiply all voltages above by 4
                                                                //              
                                                                //                      - Amps:    Increased (multiplied) by battery capacity scaling factor  - "systemAmpMult". 
                                                                //                                 As selected by the System Size DIP switches (or forced override via ASCII command).
                                                                //                                 e.g:  A 1000 Ah battery was selected -- a capacity of 2x --  and Amp values will be increased by 2x before being used.
                                                                //
                                                                //                      
                                                                //                      
                                                                //                      - Temps:   BAT_TEMP_1C_COMP is increased (multiplied) by battery scaling factor. 
                                                                //                                 As determined by auto-sensing the battery voltage via the RAW Atmel A/D port.
                                                                //                                 e.g.:  at 48v system would multiply BAT_TEMP_1C_COMP above by 4
                                                                //                                
                                                                //              Entries that are NOT modified are:
                                                                //                      - Temp Limits:  MIN_TEMP_COMP_LIMIT, BAT_MIN_CHARGE_TEMP  and  BAT_MAX_CHARGE_TEMP are not scaled.
                                                                //                      
                                                                //                      - Time values:  No time value is modified
                                                                //                                        Note that the temp limits (ala min and Max temps) are NOT changed.
                                                                //              
                                                                





