//      Sensors.h
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


#ifndef _SENSORS_H_
#define _SENSORS_H_

typedef struct CAL {                                        // Calibration Structure - holds board specific values, set at time of manufacturing.
      bool          Locked;                                 // Calibration already done at Factory, and hence should NEVER be overridden (even master reset)
      float         VBatGainError;                          // Error of VBat ADC + resister dividers 
      float         AmpGainError;
      int16_t       VBatOffset;
      int16_t       AmpOffset;                              // Offset error of shunt circuit measured @ 0A 
      } CAL;

      

extern bool    updatingVAs;
extern bool    shuntAmpsMeasured; 

extern float   measuredAltVolts;
extern float   measuredAltAmps;
extern int     measuredAltWatts;
extern float   measuredBatVolts;
extern float   measuredBatAmps;

extern int     measuredFETTemp;
extern int     measuredFieldAmps;
extern int     measuredAltTemp;
extern int     measuredAlt2Temp; 
extern int     measuredBatTemp;   

extern unsigned long   accumulatedLrAH;
extern unsigned long   accumulatedLrWH;
extern unsigned long   generatorLrRunTime;

extern CAL  ADCCal;



bool initialize_sensors(void);
bool read_sensors(void);
bool sample_ALT_VoltAmps(void);
bool read_ALT_VoltAmps(void);
void resolve_BAT_VoltAmpTemp(void);
void update_run_summary(void);
void reset_run_summary(void);



#endif  /*  _SENSORS_H_ */



