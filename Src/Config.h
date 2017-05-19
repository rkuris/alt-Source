//
//      Config.h
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


#ifndef _Config_H_
#define _Config_H_

#include <Arduino.h>            // Pick up Arduino specifics, ala PROGMEM


                                                       
/****************************************************************************************
 ****************************************************************************************
 *                                                                                      *
 *                              CUSTOMIZING PARAMETERS                                  *
 *                                                                                      *
 *                                                                                      *
 *      Change the following parameters to meet the exact needs of your installation.   *
 *                                                                                      *
 *                                                                                      *
 ****************************************************************************************
 ****************************************************************************************/




#define  REG_FIRMWARE_VERSION  "AREG1.0.2"                      // Sent out with SST; status string as well as the CAN product ID 
                                                                // Use format of xxxxyyyyyy  where xxxx= 4 chars of device type, and yyy= varying chars of version number.


//#define SIMULATION                                              // Enables forcing code to allow for bench testing on stand-alone  card..
//#define DEBUG                                                   // Debug code, Extra information sent via the Serial port for use during mocking-up and/or Debugging
                                                                //  The Debug switch also allows the RN41 to accept commands externally, and bypasses the need to change 
                                                                //  the Bluetooth name and PIN before accepting any ASCII change commands.


#define FEATURE_OUT_LAMP                                        // Enable FEATURE_OUT port as a LAMP driver, to show ruff status as well as blink out the FAULT code
//#define FEATURE_OUT_ENGINE_STOP                               // Enable FEATURE_OUT port to go active when we enter FLOAT mode.  Useful to auto-stop a DC generator when charging has finished.
//#define FEATURE_OUT_COMBINER                                  // Enable FEATURE_OUT port to go active when alternator is in Accept or Float phase.  That is when the main battery had been 'bulked up'.
                                                                //   Useful for connecting external relay to join 2nd battery, but you need to make sure to set the Capacity DIP switches to reflect the 
                                                                //   total capacity of the TWO (or more) batteries when combined.
                                                                // CAUTION:  Make sure to only pick ONE of these Feature_out port usages, else who knows what you will get!

#define FEATURE_IN_EQUALIZE                                     // Enable FEATURE_IN port to select EQUALIZE mode
#define FEATURE_IN_RESTORE                                      // Enable FEATURE_IN port to be used during Startup to restore the Regulator to as-shipped (compiled) state.  
                                                                //      NOTE:  "restore" may be combined with other Feature_in defines, as it impacts only startup operation, not running.


#define FEATURE_IN_DISABLE_CHARGE                               // Enable FEATURE_IN port to prevent entering active charge modes (Bulk, Acceptance, Overcharge) and only allow Float or Post_Float.
                                                                //   This capability will ONLY be active if CPE #8 is selected, and it will also prevent other feature_in options from being usable
                                                                //   with CPE #8.



                                                                
                                                                
                                                                
              
                                                
#define OPTIBOOT                                                // Compile this code for use with Optiboot bootloader?  If so, fully enable the Watchdog timer.
                                                                // If Optiboot is not used, and the 'default' Arduino IDE bootloader is used, there are some workarounds
                                                                // needed to make reboot() function, and if the watchdog is ever triggered the regulator will hang vs.
                                                                // restart.  This is a fault / bug in the basic Arduino bootloader for 3.3v + atMega328.  See blog for more details.





















                                // --------  BOARD SPECIFIC SELECTIONS
                                //          This source supports two version of the Arduino Regulator:
                                //              - Original version utilizing the ATmega328 CPU (Through hole w/optional Bluetooth module)
                                //              - 2nd gen version using the ATmegaxxM1 uC and supporting a CAN bus (but no Bluetooth)
                                //
                                //          Here we will detect which CPU is being compiled, and make the appropriate port selections
                                //          for I/O ports and additional include files as needed.
                                //
                                //          Note that ALL I/O PORTS are to be defined here, in one common location.  
                                //          This will help avoid issues with duplicate errors...
                                
                                
                                
#if defined (__AVR_ATmega328P__) || \
    defined (__AVR_ATmega328__)                             // Original ATmega328 based alternator regulator.
       
    #define STANDALONE                                      // Build flag to select different code for this Stand-along regulator.
    #define CPU_AVR                                         // Utilize the Atmel AVR I2C hardware (sensors.cpp)
    #include <avr/wdt.h>     
                                              
                                                
                                                
      
      //----   Origional 'Uno' based regulator.
    #define FEATURE_OUT_PORT                 3              // Drives Feature-out open-collector port.
    #define LED_GREEN_PORT                   8              // Connected to status LED on the PCB.
    #define LED_RED_PORT                     8              // The original PCB only has a green LED, so use the same port# for both.
    #define FIELD_PWM_PORT                   9              // Field PWM is connected to Atmel pin-15 (port 9 on Arduino)
    #define CHARGE_PUMP_PORT                 6              // PWM port that drives the FET boost-voltage Charge Pump
    #define BT_STATE_PORT                   10              // Bluetooth STATE status sensing port.
    #define FEATURE_IN_PORT                 11
    #define DIP_BIT1                         7              // DIP Switch bits directly wired to digital port
    #define DIP_BIT2                        A1              // Notice there is no DIP0, that is used to control power to the Bluetooth module.
    #define DIP_BIT3                        A0
    #define DIP_BIT4                         4
    #define DIP_BIT5                         5
    #define DIP_BIT6                        12
    #define DIP_BIT7                        13
    #define DIP_MASK                        0xFE            // There are 7-bits usable in this version, lsb is not used.                                        


    #define NTC_A_PORT                      A2              // Alternator NTC port
    #define NTC_B_PORT                      A3              // Battery NTC port
 
    #define STATOR_IRQ_NUMBER                0              // Stator IRQ is attached to pin-4 on the Atmel CPU (INT-0 /port 2 on the Arduino)

    #define SYSTEM_BAUD                   9600UL            // Nice and slow Baud, to assure reliable comms with local RN-41 Bluetooth module + future expansion. 
    #define DEFAULT_BT_CONFIG_CHANGED      false            // On the 2nd generation regulator, with support for a Bluetooth module, we need to lockout any changes
                                                            // via ASCII commands into the user has revised the name and password.  See users guide for details.
    
    #define FIELD_PWM_MAX                  0xFF             // Maximum alternator PWM value.  The original Alt Regulator had a separate Charge Pump,
                                                            // and could run true Full Field all day long.

    #define AALT_SCALER           ((10000+220) / 220) / 50  // Ratio of R22/R24 combined with U6 (INA282) 50x gain followed by resistor dividers.
    #define VALT_SCALER                 (10+9.881)/9.881    // Scaling values of Voltage Divider resisters on the INA-226 VBat+ sensing line. (R4/R10)
                                                            // (Note that I add in the 'typical' Zin  of 830K of the INA-226 into the 'R10' calculation)
                                                            // (Note also this ratio is 'upside down' from typical - I did this so I could use a * instead of a / in the calc..)
 
    
    #define  set_PWM_frequency() TCCR1B = (TCCR1B & 0b11111000) | 0x04;                     // Set Timer 1 (Pin 9/10 PWM) to 122Hz (from default 488hz).  This more matches
                                                                                            // optimal Alternator Field requirements, as frequencies above 400Hz seem to send

    
    
    
#elif defined(__AVR_ATmega32C1__) || \
      defined(__AVR_ATmega64C1__) || \
      defined(__AVR_ATmega32M1__) || \
      defined(__AVR_ATmega64M1__)                           //----  New CAN Enabled regulator hardware.

    #define  SYSTEMCAN                                      // Build flag to select different code for the CAN enabled / 'systems' version.
    #define  CPU_AVRCAN                                     // Need to use software simulation for I2C ports  (sensors.cpp)
    #include <avr/wdt.h>  
    #include "AltReg_CAN.h" 
                                                         
    #define REG_HARDWARE_VERSION   "1.3.1 (2017-01-30)"     // Min PCB Hardware version usable with this version of firmware                                                         
    #define REG_PRODUCT_CODE               100              //  100 is the product code for ATmega64M1 based Smart Alternator Regulator
    
    
    #define FEATURE_OUT_PORT                 8              // Drives Feature-out open-collector port.
    #define LED_GREEN_PORT                  14              // Connected to status LED on the PCB.
    #define LED_RED_PORT                    13 
    #define FIELD_PWM_PORT                   2              // Field PWM 
    #define FEATURE_IN_PORT                  9
    #define DIP_BIT0                        A6              // DIP Switch bits directly wired to ports
    #define DIP_BIT1                        A8 
    #define DIP_BIT2                        11
    #define DIP_BIT3                        A4
    #define DIP_BIT4                        A3  
    #define DIP_BIT5                        A5 
    #define DIP_BIT6                        A2 
    #define DIP_BIT7                         7
    #define DIP_MASK                        0xFF            // There are 8-bits usable in this version                                        


    #define NTC_A_PORT                      A0              // A port, primarily Alternator temperature sensor
    #define NTC_B_PORT                      A1              // B Port, primary Battery temperature sensor - reverts of 2nd Alternator sensor if battery temperature is provided by external source.
    #define NTC_FET_PORT                    A7              // Onboard FET temperature sensor (Definition is also used to enable FET NTC code in sensors.cpp)
    #define STATOR_IRQ_NUMBER                3              // Stator IRQ is attached to pin-30 on the Atmel CPU (INT-3 /port 12 on the Arduino)

    
                                                            
                                                            
                                                            
        //-----  I2C ports and config values  
        //       The soft I2C lib being used is an ATMEL lib, not Arduino.   So, the port definitions are different...
        //       It is Assembler based and very well done - however, it is not fully 'Arduino' compatible.
        //       Two differences are:  It uses Atmel PORT names/number vs. Arduino 'port' number, and
        //                              those values need to be defined BEFORE including the software lib.   So:
    #define SDA_PORT PORTB                                  // This is Arduino port D5 on the ATmegaxxM1
    #define SDA_PIN 0
    #define SCL_PORT PORTB                                  // This is Arduino port D6 on the ATmegaxxM1
    #define SCL_PIN 1 
    
    //#define I2C_FASTMODE 0                                // Set = 1 to run in fast mode (400 kHz)
    //#define I2C_SLOWMODE 1                                // Set = 1 to run in slow mode (25Khz)   -- Default is normal mode (100Khz)
    #define I2C_TIMEOUT 20                                  // timeout after 20 msec -- do not wait for clock stretching longer than this time
    //#define I2C_NOINTERRUPT 0                             // Set = 1 to disable interrupts during time between START() and STOP() 
    //#define I2C_CPUFREQ (F_CPU/8)                         // slow down CPU frequency - Useful if you plan on doing any clock switching

    #define SYSTEM_BAUD                   115200UL          // With no Bluetooth module, run the serial port faster to min time blocked from full buffer.
    #define DEFAULT_BT_CONFIG_CHANGED      true             // On the regulators w/o a Bluetooth module, we do not need to take the extra security caution of locking out any changes
                                                            // via ASCII commands into the user has revised the name and password.  See users guide for details.

                                                            
    #define FIELD_PWM_MAX                  0xFE             // Maximum alternator PWM value - As the newer version of the regulator does not have a charge-pump,
                                                            // we need to restrict Full Field to some PWM, to allow refreshing of the boost-cap in the FET driver chip


    #define AALT_SCALER        (((2*4990)+221) / 221) / 50  // Ratio of R19..R22 combined with U9 (INA282) 50x gain followed by resistor dividers.
    #define VALT_SCALER             (30+19.529)/19.529      // Scaling values of Voltage Divider resisters on the INA-226 VBat+ sensing line. (R1/R6)
                                                            // (Note that I included the 'typical' Zin of the INA-226 (830K) into the 'R6' calculation)
                                                            // (Note also this ratio is 'upside down' from typical - I did this so I could use a * instead of a / in the calc..)
                         
                                      
    
    #define  set_PWM_frequency() TCCR1B = (TCCR1B & 0b11111000) | 0x04;                     // Set Timer 1 (Pin 9/10 PWM) to 122Hz (from default 488hz).  This more matches
                                                                                            // optimal Alternator Field requirements, as frequencies above 400Hz seem to send


                                                                         
///@  HERE IS THE CORE DIFFERENCE SECTION!!!
#elif defined(STM32F072xB) || \
      defined(STM32F078xx) || \
      defined(__SAMD21G18A__)
                         // Note:  the SAMDxxx is a placeholder to allow test compiles with the Arduino ZERO (also a M0 ARM chip) 
                         // If you are reading this, all this code stuff is for investigation of a 4th gen regulator, one based on the STM32F07x CPU -- lower cost design, plus 2x the FLASH space!             
                                                                         

      
    #define STANDALONE                                      // Build flag to select different code for this Stand-along regulator.
    #define CPU_STM32                                       // Utilize the STM I2C hardware (sensors.cpp)
    #define EEPROM_SIM                                      // EEPROM needs to be simulated   
 
    #define REG_PRODUCT_CODE               200              //  200 is the product code for STM32F072 based Smart Alternator Regulator
 
///@  -- Some temp defines and stuffer, just to get past compiler...
                                                
                                                
      
      //----   Origional 'Uno' based regulator.
    #define FEATURE_OUT_PORT                 3              // Drives Feature-out open-collector port.
    #define LED_GREEN_PORT                   8              // Connected to status LED on the PCB.
    #define LED_RED_PORT                     8              // The original PCB only has a green LED, so use the same port# for both.
    #define FIELD_PWM_PORT                   9              // Field PWM is connected to Atmel pin-15 (port 9 on Arduino)
    #define CHARGE_PUMP_PORT                 6              // PWM port that drives the FET boost-voltage Charge Pump
    #define BT_STATE_PORT                   10              // Bluetooth STATE status sensing port.
    #define FEATURE_IN_PORT                 11
    #define DIP_BIT0                         7              // DIP Switch bits directly wired to digital port
    #define DIP_BIT1                         7
    #define DIP_BIT2                        A1
    #define DIP_BIT3                        A0
    #define DIP_BIT4                         4
    #define DIP_BIT5                         5
    #define DIP_BIT6                        12
    #define DIP_BIT7                        13
    #define DIP_MASK                        0xFF            // There are 8-bits usable in this version                                        


    #define NTC_A_PORT                      A2              // Alternator NTC port
    #define NTC_B_PORT                      A3              // Battery NTC port
 
    #define STATOR_IRQ_NUMBER                0              // Stator IRQ is attached to pin-4 on the Atmel CPU (INT-0 /port 2 on the Arduino)

    #define SYSTEM_BAUD                   115200UL          // With no Bluetooth module, run the serial port faster to min time blocked from full buffer.
    #define DEFAULT_BT_CONFIG_CHANGED      true             // On the regulators w/o a Bluetooth module, we do not need to take the extra security caution of locking out any changes
                                                            // via ASCII commands into the user has revised the name and password.  See users guide for details.


    
    #define FIELD_PWM_MAX                  0xFF             // Maximum alternator PWM value.  The original Alt Regulator had a separate Charge Pump,
                                                            // and could run true Full Field all day long.

    #define AALT_SCALER            ((10000+220) / 220) / 50 // Ratio of R22/R24 combined with U6 (INA282) 50x gain followed by resistor dividers.
    #define VALT_SCALER                 (10+9.881)/9.881    // Scaling values of Voltage Divider resisters on the INA-226 VBat+ sensing line. (R4/R10)
                                                            // (Note that I add in the 'typical' Zin  of 830K of the INA-226 into the 'R10' calculation)
                                                            // (Note also this ratio is 'upside down' from typical - I did this so I could use a * instead of a / in the calc..)
 



                        
///@ -- from ME!!!
#define snprintf_P(s, f, ...) snprintf((s), (f), __VA_ARGS__)

#define wdt_reset()           //!  NEED SOME DIFFERENT CODE HERE FOR THESE BYPASSED CAPABILITIES
#define wdt_enable(WDT_PER)
#define wdt_disable()
#define set_PWM_frequency()          // TCCR1B = (TCCR1B & 0b11111000) | 0x04;                     // Set Timer 1 (Pin 9/10 PWM) to 122Hz (from default 488hz).  This more matches
                                                                                            // optimal Alternator Field requirements, as frequencies above 400Hz seem to send



///@ -- From the Maple STM32 porting file arduino.h

#include <inttypes.h>

#define PROGMEM
#define PGM_P  const char *
#define PSTR(str) (str)

#define _SFR_BYTE(n) (n)

typedef void prog_void;
typedef char prog_char;
typedef unsigned char prog_uchar;
typedef int8_t prog_int8_t;
typedef uint8_t prog_uint8_t;
typedef int16_t prog_int16_t;
typedef uint16_t prog_uint16_t;
typedef int32_t prog_int32_t;
typedef uint32_t prog_uint32_t;

#define memcpy_P(dest, src, num) memcpy((dest), (src), (num))
#define strcpy_P(dest, src) strcpy((dest), (src))
#define strcat_P(dest, src) strcat((dest), (src))
#define strcmp_P(a, b) strcmp((a), (b))
#define strstr_P(a, b) strstr((a), (b))
#define strlen_P(a) strlen((a))
#define sprintf_P(s, f, ...) sprintf((s), (f), __VA_ARGS__)

#define F(a) ((a))
#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#define pgm_read_word(addr) (*(const unsigned short *)(addr))
#define pgm_read_dword(addr) (*(const unsigned long *)(addr))
#define pgm_read_float(addr) (*(const float *)(addr))

#define pgm_read_byte_near(addr) pgm_read_byte(addr)
#define pgm_read_word_near(addr) pgm_read_word(addr)
#define pgm_read_dword_near(addr) pgm_read_dword(addr)
#define pgm_read_float_near(addr) pgm_read_float(addr)
#define pgm_read_byte_far(addr) pgm_read_byte(addr)
#define pgm_read_word_far(addr) pgm_read_word(addr)
#define pgm_read_dword_far(addr) pgm_read_dword(addr)
#define pgm_read_float_far(addr) pgm_read_float(addr)


#else
        #error Unsupported Smart Regulator CPU 
#endif 


























                                //----   COMBINER VALUES
                                //       Used to define how the feature-out combiner will work.
                                //       note:  all voltages are in 'nominal' 12v form, and will be adjusted at runtime by the systemVoltMult, but not battery temperature...
#ifdef FEATURE_OUT_COMBINER

  #define       COMBINE_CUTIN_VOLTS     13.2                            // Enable to combiner once OUR battery voltage reaches this level.
  #define       COMBINE_HOLD_VOLTS      13.0                            // Once enabled remain so on even if voltage sags to this value - a hysteresis to reduce relay chatter/cycling.
  #define       COMBINE_DROPOUT_VOLTS   14.2                            // But then disable the combiner once OUR battery voltage reaches this level.           
                                                                        // This is for cases where we are looking to get a boost from the other battery during
                                                                        // bulk phase, but do not want to run the risk of 'back charging' the other battery
                                                                        // once voltage raises a bit.   --HOWEVER--
  #define       COMBINE_ACCEPT_CARRYOVER   0.75*3600000UL               // If we indeed want to continue on and back charge the other battery (say, in the case the other
                                                                        // battery has no other charging source, ala a bow thruster battery), then this is a brute-force
                                                                        // carry over into the Accept phase, keep the combiner on for this many hours. (0.75, or 45 minutes)
                                                                        // If you want this to actually WORK, then you will need to raise the dropout voltage above to a higher
                                                                        // threshold, perhaps 15v or so??
                                                                        // See documentation for more details around these parameters and how to configure them..
  #endif




  
  
 



                                                                
                                                                
//
//
//------------------------------------------------------------------------------------
//
//      The Following are internal parameters and typically will not need to be changed.
//
//




                                // -----   Critical fault values: exceeding these causes system fault and fault handler.
                                //         Note that these values are set for a 'normalized - 12v / 500Ah battery', and will be automatically scaled during runtime
                                //         by the regulator after sampling the battery voltage and setting the variable "systemVoltMult"
                                //
#define FAULT_BAT_VOLTS_CHARGE          16.5                    // This is where we will fault.  Make sure you have sufficient headroom for Over Charge voltage (if being used)
#define FAULT_BAT_VOLTS_EQUALIZE        18.0                    // When doing Equalization, allow a higher limit.
#define FAULT_BAT_VOLTS_LOW              8.0                    // Anything below this means battery is damaged, or sensing wire is missing / failing.
#define FAULT_BAT_TEMP                   140                    //  At or above this, fault out. in degrees-F
#define FAULT_ALT_TEMP                   1.1                    //  Fault if Alt Temp exceeds desired set point by 10%   
#define FAULT_FET_TEMP                   200                    // If Field driver FETs are over 200f, something is wrong with the FETs - fault.

#define ADPT_ACPT_TIME_FACTOR              5                    // If the regulators is operating in Adaptive Acceptance Duration mode (either because EXIT_ACPT_AMPS was set = -1, or
                                                                // if we are unable to measure any amps), the amount of time we spend in Bulk phase will be multiplied by this factor, and
                                                                // we will remain in Acceptance phase no longer then this, or EXIT_ACPT_DURATION - whichever is less.  This is in reality a backup
                                                                // to Amp based decisions, in case the shunt is not installed, or fails.  Or in the case where the install never uses the Amp shunt, and
                                                                // charging is started with a full battery - do not want to boil off the battery.
        


#define ENGINE_WARMUP_DURATION          30000UL                 // Allow engine 30 seconds to start and 'warm up' before placing a load on it.  


                                // ------ Parameters used to calibrate RPMs as measured from Alternator Stator Pulses
#define RPM_IRQ_AVERAGING_FACTOR           100                  // # sector pulses we count between RPM calculations, to smooth things out.  100 should give 3-6 updates / second as speed.
#define IRQ_uS_TIMEOUT                   10000                  // If we do not see pulses every 10mS on average, figure things have stopped.
#define IDLE_SETTLE_PERIOD               10000                  // While looking for a potential new low for idleRPMs, the engine must maintain this new 'idle' period for at least 10 seconds.





    

                                //----  PWM Field Control values
#define FIELD_PWM_MIN              0x00                 // Minimum alternator PWM value - It is unlikely you will need to change this.
// #define FIELD_PWM_MAX           -------              // Maximum alternator PWM value - (This is now defined in the CPU specific section of device_Unique.h)
#define MAX_TACH_PWM                 75                 // Do not allow MIN PWM (held in 'thresholdPWMvalue') to go above this value when Tach Mode is enabled.  Safety, esp in case Auto tech mode is enabled.
                                                        //  This same value is used to limit the highest value the user is allowed to enter using the $SCT ASCII command.
#define PWM_CHANGE_CAP                2                 // Limits how much UP we will change the Field PWM in each time 'adjusting' it.

#define PWM_CHANGE_RATE            100UL                // Time (in mS) between the 'adjustments' of the PWM.  Allows a settling period before making another move.
#define PWM_RAMP_RATE              400UL                // When ramping, slow the rate of change down  to update only this often (in mS)  
                                                        //    This combined with PWM_CHANGE_CAP will define the ramping time.
                                                        //    (for PWM to reach the FIELD_PWM_MAX value and exit Ramp).

   
                                                        // These are used to count down how many PWM_CHANGE_RATE cycles must pass before we look at the 
                                                        // these slow changing temperature values.  (TA = Alt)
                                                        // If any of these are over value, we will adjust DOWN the PWM once every xx times through adjusting PWM. 
#define TAM_SENSITIVITY             100                 // For Alternator Temperature, check every 100x PWM_CHANGE_RATE (or every 10 seconds)  (Max 126)



 
                                //---- The following scaling values are used in the PID calculations used in manage_ALT()
                                //     Factors are adjusted in manage_alt() by systemMult as needed.
                                //     Values are represented in 'Gain' format, and MUST be defined as floating values (e.g.  30.0  vs. 30) in order to keep each component of the 
                                //      PID engine working with fractions (each small fractional value is significant when summing up the PID components)
#define KpPWM_V                     20.0
#define KiPWM_V                     10.0
#define KdPWM_V                     50.0

#define KpPWM_A                      0.6              
#define KiPWM_A                      0.3 
#define KdPWM_A                      0.7

#define KpPWM_W                      0.05     
#define KiPWM_W                      0.0                
#define KdPWM_W                      0.02


#define KpPWM_TA                     0.500              // Adjust PWM 1 step for each 2 degree error in Alt temp.   
#define KdPWM_TA                     0.750

#define PID_I_WINDUP_CAP             0.9                // Capping value for the 'I' factor in the PID engines.  I is not allowed to influence the PWM any more then this limit 
                                                        // to prevent 'integrator Runaway' 



                                                          
                                //---- Load Dump / Raw Overvoltage detection thresholds and actions.
                                //     (These action occur asynchronous to the PID engine -- handled in real time linked to the ADCs sampling rate.)
  
                                                       
#define LD1_THRESHOLD              0.040                // During a Load Dump situation, VBat can start to rise VERY QUICKLY.   Too quick for the PID engine
#define LD2_THRESHOLD              0.080                // in the case of large (200A+) alternators.  If measured voltage exceeds target voltage by these thresholds 
#define LD3_THRESHOLD              0.100                // the PWM is pulled back.   Once these brute force changes are made, the normal PID engine can start 
                                                        // adjusting things back to the new situation.  PROTECT THE BATTERY IS #1 CRITERIA!!!!
        
#define LD1_PULLBACK               0.95                 // On 1st sign of overvoltage, pull back the field a little.
#define LD2_PULLBACK               0.85                 // On 2nd sign of overvoltage, pull back harder 
                                                        // LD 3 overvoltage holds the Field at 0 until the overvoltage situation clears.

                            

  
                                                        


#define OT_PULLBACK_THRESHOLD           -1              // If Alt Temp produce a PWM error of this or more in manage_alt(), then it seems we have a condition
                                                        // of a fight between that temp value and the current watts target - creating an osculation situation.
#define OT_PULLBACK_FACTOR              0.97            // When triggered, we will pull down the amps target this ratio to try and self correct.

#define USE_AMPS_THRESHOLD              5               // If at some time we do not measure AT LEAST this amount of Amps, then we will ASSUME the Amp Shunt is not
                                                        // connected - this will cause Manage_ALT() to ignore any Amps based thresholds when deciding if it is time to
                                                        // transition out of a given charging phase.  It is a way for the regulator to act in Voltage Only mode - just
                                                        // do not connect up the shunt!  (Actually, would be better to put a wire jumper across the Shunt terminals)
                                                        
                                                        
                                                        
#define MAX_SUPPORTED_AMPS              1000            // If no limits are set by the user, use these MAXes.  Note these values will be shown in the ASCII status strings as goals..
#define MAX_SUPPORTED_WATTS            15000

                                                        // When deciding to change Alternator charge states, and adjust the throttle, we use persistent Watts and Amps.
                                                        // These are averaged values over X periods.  These are used to smooth changes in 
                                                        // Alternator State modes - to allow for short term bumps and dips.
#define AMPS_PERSISTENCE_FACTOR         2048            // Amps will be averaged over this number of samples at "PWM_CHANGE_RATE". (2048 = ~3.5 mins look-back)
                                                        // Set = 1L to disable  (Used to exit Acceptance and Float modes)
#define VOLTS_PERSISTENCE_FACTOR        300             // Volts will be averaged over this number of samples at "PWM_CHANGE_RATE". (300 = ~1/2 min look-back)
                                                        // Set = 1 to disable  (Used to exit post_float mode)

                                                        // Desensitizing parameters for deciding when to initiate a new Alternator Capacity Measurement cycle, 
#define SAMPLE_ALT_CAP_RPM_THRESH      50               // Initiate a new Capacity Sample if we have seen in increase in RPMs / Amps from the prior reading.  
#define SAMPLE_ALT_CAP_AMPS_THRESH      2               // To keep from being too twitchy ..  





                                // --------  INA226 Registers & configuration
#define INA226_I2C_ADDRESS         0x40                 // I2C addresses of INA226 device. Measures Amp shunt, and Vbat+                                
#define CONFIG_REG                 0x00                 // Register Pointers in the INA-226
#define SHUNT_V_REG                0x01
#define VOLTAGE_REG                0x02 
//#define POWER_REG                0x03                 // Because I am doing raw voltage reads of the Shunt, no need to access any of the INA-226 calculated Amps/power
//#define SHUNT_A_REG              0x04
//#define CAL_REG                  0x05                 // Nor the Cal reg 
#define STATUS_REG                 0x06
    
#define INA226_CONFIG            0x4523                 // Configuration: Average 16 samples of 1.1mS A/Ds (17mS conversion time), mode=shunt&volt:triggered





                                // ------ Values use to read the Feature-in port to handle debouncing.
#define DEBOUNCE_COUNT               5                  // We will do 5 samples of the Feature_in port to handling any de-bouncing.
#define DEBOUNCE_TIME                1                  // And if asked for, we will block the system for 1mS between each of those samples in order to complete a
                                                        // debounce cycle when feature_IN() is called.  Do not make this too large, as it is a raw delay.
                                                        // And remember, there is an external R/C to help filter any bouncing before we even see it.



                                // ------ Values use for the NTC temp sensors.
#define   NTC_RO                10000                   // Nominal resistance of NTC sensors at 25c
#define   NTC_BETA               3950                   // Most common Beta for probes on EBay..
#define   NTC_BETA_FETs          3380                   // Beta of NTC chip used for FET sensing.
#define   NTC_RF                10000                   // Value of 'Feed' resister
#define   NTC_RG                  100                   // Value of 'Ground Isolation' resister - used only on the external NTC probes.
#define   NTC_AVERAGING            15                   // There are up to 3 NTC sensors, and we will average 5 A/D samples each before converting to a temperature, to smooth things over. (Max 254)
                                                        // Note that this, combined with SENSOR_SAMPLE_RATE will define how often the temperatures get updated...




                        
                                // ----- Mainloop timing values, how often do we update the PWM, check for key pressed, etc.
                                //         All times are in mS
#define SENSOR_SAMPLE_RATE           50UL               // If we are not able to synchronize with the stator, force a sample of Volts, Amps, Temperatures, every 50mS min. 
#define ACCUMULATE_SAMPLING_RATE   1000UL               // Update the accumulated AHs and WHs  every 1 second.  
#define SAMPLE_ALT_CAP_DURATION   10000UL               // When we have decided it is time to sample the Alternators capability, run it hard for 10 seconds.  
#define SAMPLE_ALT_CAP_REST       30000UL               // and give a 30 second minute rest period between Sampling Cycles. 
#define OA_HOLD_DURATION          60000UL               // Hold on to external offset amps (received via $EOA command) for only 60 seconds MAX.




                                // --------  Global Timouts
#define WDT_PER                 WDTO_8S                 // Set the Watchdog Timer for 8 seconds (Lock step this with CHECK_WDT_mS below)
#define CHECK_WDT_mS               8000                 // Used by validation in pre-compile checks for errors in critical timing.  Make this the # mS the WDT is set for.
#define I2C_TIMEOUT                  20                 // Bail on faulted I2C functions after 20mS

 


                                //-----  Bluetooth values
#define BT_TIMEOUT                1000UL                // Wait up to 1 second for the RN41 module to acknowledge a command.

#define CLEAR_BT_LOCK_AMPS            5                 // In order for systemConfig.BT_CONFIG_CHANGED to be changed to TRUE, we must have received a valid '$SCN:' while the
                                                        // alternator is not actually charging.  This defines the Amps threshold we need to be under to decide we are 'not charging'.
                                                        // Note that you can PERMANENTLY disable the ability to change Bluetooth configuration by setting this to -1











                //----- LED blinking controls and conversion tables
                //



#define LED_RATE_NORMAL         300                             // Normally flash the LED patterns changing the LED every 300mS
#define LED_RATE_SLOW           600                             // And slow down to once per second for dramatic effect
#define LED_RATE_FAST           100                             //  Even more drama!

#define LED_FAULTED             0xAA04                          // FAULTED LED pattern  - Use FAST and repeat 2x times.
#define LED_BULK                0xAAAA                          // Blink out Normal while in Ramp, or Bulk mode
#define LED_ACCEPT              0xA0A0                          // Blink out Normal while in Acceptance mode
#define LED_OC                  0xAAAA                          // Blink out Slow while in Overcharge
#define LED_FLOAT               0xFF00                          // Blink out Normal while in Float / post-float mode
#define LED_EQUALIZE            0xA000                          // Blink out Fast while in Equalize mode
#define LED_IDLE                0x0100                          // Blink out Normal while idle.
#define LED_RESETTING           0xAAAA                          // Show that we are about to reset, blink out FAST





        
                //----    Error codes. If there is a FAULTED status, the variable errorCode will contain one of these...
                //              Note at this time, only one error code is retained.  Multi-faults will only show the last one in the checking tree.
                //              Errors with + 0x8000 on them will cause the regulator to re-start, others will freeze the regulator.
                //              (Note combinations like 10, and 11 are not used.  Because one cannot flash out 0's, and kind of hard to 
                //               tell if 11 is a 1+1, or a real slow 2+0)



#define FC_LOOP_BAT_TEMP                12              // Battery temp exceeded limit
#define FC_LOOP_BAT_VOLTS               13              // Battery Volts exceeded upper limit (measured via INA226)
#define FC_LOOP_BAT_LOWV                14  + 0x8000U   // Battery Volts exceeded lower limit, either damaged or sensing wire missing. (or engine not started!)

#define FC_LOOP_ALT_TEMP                21              // Alternator temp exceeded limit
#define FC_LOOP_ALT_RPMs                22              // Alternator seems to be spinning way to fast!
#define FC_LOOP_ALT2_TEMP               23              // Alternator #2 temp exceeded limit


#define FC_LOG_ALT_STATE                31              // Global Variable AlternatorState has some unsupported value in check_for_faults() 
#define FC_LOG_ALT_STATE1               32              // Global Variable AlternatorState has some unsupported value in manage_ALT() 
#define FC_LOG_CPI_STATE                33              // Global Variable cpIndex has some unsupported value in caculate_ALT_Targets() 
#define FC_LOG_CPINDEX                  34              // Global Variable cpIndex has some unsupported value in check_for_faults() 
#define FC_LOG_SYSAMPMULT               35              // Global Variable systemAmpMult has some unsupported value in check_for_faults() 
        
        
#define FC_SYS_FET_TEMP                 40              // Internal Field FET temperature exceed limit.


#define FC_CAN_BATTERY_DISCONNECTED     50              // We have received a CAN message that the battery charging bus has been disconnected.;
#define FC_CAN_BATTERY_HVL_DISCONNECTED 51              // We have noted that a command has been sent asking for the battery bus to be disconnected!
#define FC_LOG_BATTINST                 52              // Battery Instance number is out of range (needs to be from 1..100)

#define FC_INA226_READ_ERROR            100 + 0x8000U   // Returned I2C error code is added to this, see I2C lib for error codes.







void      reboot(void); 
uint8_t   readDipSwitch(void);
bool      feature_in(bool waitForDebounce);

extern bool     sendDebugString;
extern int8_t   LEDRepeat;
extern int8_t   SDMCounter;

extern unsigned        faultCode; 

extern char const firmwareVersion[];


#endif  // _Config_H_



