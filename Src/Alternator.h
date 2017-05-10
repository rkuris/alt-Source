//      Alternator.h
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


#ifndef _ALTERNATOR_H_
#define _ALTERNATOR_H_

#include <Arduino.h>
#include "Device_Unique.h"


typedef enum  {unknown, disabled, FAULTED, FAULTED_REDUCED_LOAD,                                                                   // Used by most or all  (0..3)
               pending_R, ramping, determine_ALT_cap, bulk_charge, acceptance_charge, overcharge_charge, float_charge, forced_float_charge, post_float, equalize, RBM_CVCC} tModes; 
                                                                                                                                // Alternator specific (4..14)
                                                                                                                                // Take care not to change the order of these, some tests use
                                                                                                                                // things like <= pending_R






                                //----- This structure is used to hold many global 'system' configurations values.  By placing them into a Structure (as opposed to
                                //      using #define statements) there is the ability to modify them during run-time, and even save them into FLASH.

#define MAX_NAME_LEN  18                                         // Up to 18 characters in the NAME, also 18 for the PASSWORD used in the Bluetooth and CAN.
#define MAX_PIN_LEN   18                                         //  (#define needs to be here as Arduino has poor pre-processing and cannot look ahead)
                                                                //  !!!! Something very odd here...  The size of SCV struct seems to be at a crossover point.  By making ONE of these
                                                                //   18 instead of 20, I save over 100 bytes of code size...  No idea why, but with both at 18 I save 110 bytes of
                                                                //   complied code, and 110 bytes if huge!!   So, 18 it is...

typedef struct  {                                               // System Configuration Structure

   bool         FAVOR_32V_redact;                               // It is impossible to auto-define a 32v system from a 24v system just from the battery voltage, as their
                                                                // expected ranges overlap. (there is also a small overlap with 48v batteries).
                                                                // This flag will FORCE clarification by favoring a 32v system over a 24v (and 48v) when there is Ambiguity.  See startup()

   bool         REVERSED_SHUNT;                                 // Is the AMP Shunt wired up backwards?  If so, this Flag will invert the readings via Software.                
                                                                //------  Default values for Alternator Goals (Amps, temp, etc..)
   uint8_t      ALT_TEMP_SETPOINT;                              // Temp range we want the Alternator to run at (max). Anything above it, start backing off Alternator Field drive.
   float        ALT_AMP_DERATE_NORMAL;                          // In normal operation, run the Alternator this % of it  demonstrated max capability
   float        ALT_AMP_DERATE_SMALL_MODE;                      // But if user has selected Small Alternator Mode, then scale back to a lower %
   float        ALT_AMP_DERATE_HALF_POWER;                      // If user shorts out the Alt Temp probe, we will then use this value to derate.
   int          ALT_PULLBACK_FACTOR;                            // Exponential factor in RPM based pull-back factor; to protect Alts by reducing max field PWM at low RPMs
                                                                // How many RPMs are needed to increase idle-pull-back 1% above 1/4 power at idle.
                                                                // Range 0..10, set = 0 to disable pullback feature.  See set_VAWL() in manage_ALT();
                                                                // Set = -1 to trigger fixed 70% cap when stator signal is not seen.
   int          ALT_IDLE_RPM;                                   // Used in conjunction with PBF to manage Field Drive at lower RPMs.  Establishes the 'starting point' for 1/4 field drive.
                                                                // Range 0..1500, set = 0 to enable auto determination of Idle RPMs. 
   
   int          ALT_AMPS_LIMIT;                                 // The regulator may OPTIONALLY be configured to limit the maximum number of AMPs being drawn from the alternator.
                                                                // For example if you have small wire sizes in the alternators wiring harness. Place the size of the Alternator here. 
                                                                // If this is set = -1, the regulator will enable a auto-size-deterring feature and during Bulk phase it will for
                                                                // short periods of time drive the alternator very hard to measure its Amps viability.  (this is triggered any time an
                                                                // increase in noted Amps exceeds what was noted before, OR if an increase in RPMS is noted..)
                                                                // CAUTION:  During Auto-size phase Full-Drive will occur EVEN if the half-power mode has need selected (by shorting the Alt NTC sender wires)
                                                                // Set this = 0 to disable AMPS based management of the alternator, the regulator will simply drive the Alternator as hard as it can.
                                                                // CAUTION:  If Amps based management is disabled, then features such as Small Alternator, and Half Power will no longer function.
                                                                //          You also run the risk of driving the FIELD at Full Field, with no increase in Alternator output - causes excessive heating in the field.



   int          ALT_WATTS_LIMIT;                                // The regulator also has the ability to limit the max number of Watts load being placed on the engine.  For example if you
                                                                // are using a large alternator on a small engine.  To enable this feature, place the max # of WATTS you want the alternator 
                                                                // to deliver here and the regulator will cap total engine load to this value.  Note this is loosely related to ALT_AMPS_LIMIT,
                                                                // but Watts is a true measurement when talking about loads being placed on engines, as Amps do not take into account the 
                                                                // current battery voltage..
                                                                // Set this to 0 to disable this feature let the regulator drive to the max capability of the Alternator (or the defined 
                                                                // ALT_AMPS_LIMIT above)
                                                                // Set = -1 to auto-calc the Watts limit based on the determined (or defined) size of the Alternator and the current target charge voltage.

                                                                // Note that ALT_WATTS_LIMIT and ALT_AMPS_LIMIT values are NOT adjusted by the sensing of the battery nominal voltage, If you enter 200A and
                                                                // 5000W, the regulator will cap at 200A or 5000W independent of the battery voltage...



                                                                // ------ Parameters used to calibrate RPMs as measured from Alternator Stator Pulses
                                                                //        IF you wish to read 'engine RPMs' from the regulator, you will need to adjust these.  But w/o any adjustment the 
                                                                //        regulator will function correctly, for its purpose.
   uint8_t      ALTERNATOR_POLES;                               // # of poles on alternator 
   float        ENGINE_ALT_DRIVE_RATIO;                         // engine pulley diameter / alternator diameter



                                                                //----   Controller Hardware configuration parameters   Here hardware options are noted.  See Blog for more details
   int          AMP_SHUNT_RATIO;                                // Spec of amp shunt, Amp/mV * % Calibration error (e.g:   250A / 75mV shunt * 100% -->  250 / 0.075 * 1.00 = 3,333) 
                                                                //  Note if Shunt voltage will exceed 80mV, then the INA-226 calibration will need to be adjusted.

   int          FIELD_TACH_PWM;                                 // If the user has selected Alternator Tach mode via the DIP switch, then this value will be used for the minimum Field PWM
                                                                // (As opposed to 0).  If this is set = -1, then during initial RAMP the field value to use for Tach mode will be auto-determined
                                                                // and set to a value just before Amps are produced by the alternator.  Set = 0 to disable Tach mode, independent of the DIP switch.





   bool         USE_BT;                                         // User configurable, Use or Disable Bluetooth for this module?  
   char         REG_NAME[MAX_NAME_LEN+1];                       // What should THIS regulator's name be, in a string.  (+1 to hold the string NULL terminator)                
   char         REG_PSWD[MAX_PIN_LEN+1];                        // Password to be programmed into the Bluetooth. 

   bool         BT_CONFIG_CHANGED;                              // Has the BT Configuration been changed from the Defaults?  (This is used to disable the ability to update charge parameters 
                                                                // or the system config until the Bluetooth is a bit more secure then the as-compiled defaults)

   uint8_t      CP_INDEX_OVERRIDE;                              // User has used issues command to override the DIP switches for the cpIndex.      -1 = use DIP switches.
   float        BC_MULT_OVERRIDE;                               // User has used issues command to override the DIP switches for the bcMultiplier.  0 = use DIP switches.
   float        SV_OVERRIDE;                                    // User forced Voltage Multiplier (1..4) associated with 12v..48v.  Use 0 to enable auto-detect feature.
   uint8_t      CONFIG_LOCKOUT;                                 // 0=no lockout, 1=no config change, 2=no change, no input (ala, External Amps Offset).  Can ONLY be changed by HARDWARE restore-to-factory clearing.
   } SCS;






extern volatile unsigned int    interuptCounter;
extern volatile bool            statorIRQflag;
extern unsigned long            lastPWMChanged;
extern unsigned long            altModeChanged; 
extern unsigned long            EORLastReceived; 
extern tModes   alternatorState;
extern int      altCapAmps;
extern int      altCapRPMs;

extern int      targetAltWatts;
extern float    targetBatVolts;
extern float    targetAltAmps;
extern int      measuredRPMs;
extern bool     smallAltMode;
extern bool     tachMode;
extern int      fieldPWMvalue; 
extern int      thresholdPWMvalue;
extern bool     usingEXTAmps; 
extern float    persistentBatAmps;
extern float    persistentBatVolts;

extern CPS     workingParms;
extern SCS     systemConfig;

extern float   systemVoltMult;
extern float   systemAmpMult;
extern uint8_t  cpIndex;



void stator_IRQ(void);                          
void calculate_RPMs(void);
void calculate_ALT_targets(void);
void set_ALT_mode(tModes settingMode);
void set_ALT_PWM(int  PWM);
void manage_ALT(void);
bool initialize_alternator(void);

#ifdef SYSTEMCAN 
 uint8_t ALT_Per_Util(void);
 #endif


#endif  // _ALTERNATOR_H_


