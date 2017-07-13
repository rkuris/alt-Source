//------------------------------------------------------------------------------------------------------
//
//
//
// Alternator Regulator based on the Arduino IDE
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
//
//
//
//    This software is a 3/4-stage alternator regulator.  A fundamental difference between this and currently available
//      regulators is the ability to monitor Amps being delivered to the battery as well as Voltage.  By doing so more intelligent 
//      charging transitions can be made, resulting in shorter charging times and longer battery life.
//
//      This work is derived from the Smart Engine Control and Alternator Regulator project, and takes the same approach
//      for alternator regulation, but removes all the engine control and integration. http://smartdcgenerator.blogspot.com/
//
//
//    Default special (not via DIP switch, or Configuration) user features are selected by:
//        - Shorting NTC sensor the ALTERNATOR will place regulator in 1/2 Alternator amps mode.
//        - Tying Feature-IN pin HIGH at time of Power On will restore the regulator to as-compiled config (erasing any information in FLASH)
//              (Charge Profile #0..6 only)
//        - During operation, tying feature-IN input HIGH will enable Equalize mode, providing the regulator is in float/post-float mode.
//              (Except if Charge Profile #7 LiFePO4 is selected, in that case a HIGH feature-in will cause regulator to stop charging (force to float))
//
//
//
//
//
//
//
//
//      A note on this source:  In 2016 the 3rd generation of the Arduino Alternator Regulator was made available, one that was designed to operate as a 'system' 
//        utilizing a CAN bus to communicate.  Though the core functionality is the same there are several hardware changes.  This source code is designed to support both
//        gen 2&3 through compile-time flags.  It uses the target CPU ID (ATmega328 for generation 2, ATmegaxxM1 for generation 3) to select changes in the source.
//        Look for the key defines:  STANDALONG and SYSTEMCAN throughout the code.  Other decisions are based on a presence test of a feature, example:  the Battery NTC
//        code is selected based on the presence of BAT_NTC_PORT, as opposed to STANDALONE.  It is hoped this will be somewhat more readable.   I will also comment, not 
//        every small detail is bracketed with #ifdefs, example some small variables I just leave - rather than clutter up the source code with too many #ifdefs.
//
//        Please also note that with the new modularized approach to the source (as opposed to one very very long file), Arduino IDE 1.6.8 or greater must be used.  Else the preprocessor
//        will not be able to correctly arrange the include files and you will get compiler errors!!!
//
//
//
//
//
//
//
//
//      07/13/2017  v1.1.0      Removed 'Favor-32v' flag and 32v autoselect, hold in preramp state if we are receiving ASCII config commands.
//                              Changed from Deg F to Deg C, added Tach-mode flag to $SCT:, added PID_VOLTAGE_SENS (margin, better support of small alternators + large bats)
//      05/15/2017  v1.0.2      Remember assigned CAN node ID, corrected Amp-shunt cal
//      01/28/2017  v1.0.1      Revised SST; to place version number at beginning of string.
//      01/25/2017  v1.0.0      Add support for CAN enabled version of regulator, corrected "$SCO:" command not recognized. Improved auto idle detection and idle field pull-back formula,
//                              Logic to manage A&B Temp ports as well FET NTC sensor.
//                              Changes to API:  $AST; ==> Added VAlt, Fld%, Alt2Temp & FETTemp,  $SCV; ==> Added Idle RPMs value,  $SCA ==> Manage idle RPMs configuration.
//                                               $CCN; ==> New CAN configuration command,   $SCT: accepts % value vs. raw PWM.
//                              Modularized files - REQUIRES USE OF ARDUINO IDE1.6.8 or above.
//                              Corrected bug: 0 amps in CPE for EQUAL_BAT_A_SETPOINT --> Disabled Amp limits during Equalization, but allows equalization.
//                              Check for overvoltage during RAMPING
//                              Improved PID tuning (Voltage), and load-dump handling.
//                              Favor 36v changed to favor32v
//                              Corrected several ASCII command bugs
//
//            
//
//      xx/xx/2016  v0.1.9      Increased idle PWM pull-back gain, corrected Alt Gain ratio
//      03/27/2016  v0.1.8      corrected "$SCO:" command not recognized. 
//      11/10/2015  v0.1.7      Corrected bug in Alternator Temperature regulation code, Refined Alt Temp PID values to be more aggressive. 
//      07/07/2015  V0.1.6      Corrected fatal coding error in manage_alt();, CPE FORCE_TO_FLOAT ASCII voltage reporting, FORCED_FLOAT and TACH mode improvements, 
//                              Changed SST; to show version number,  Warmup duration shorted to 30 seconds, improved NTC sensor range, 
//      03/04/2015  v0.1.5           <WITHDRAWN, FATAL ERROR IN MANAGE_ALT()>   
//                                   Refined PID tuning for Volts/Amps/Watts. Corrected Acceptance time-based exit w/Amp shunt working, verified Arduino IDE 1.6.0
//      02/13/2015  v0.1.4      PD to PID engine, correct Amp-offset, Vout rounding error.
//      10/12/2014  v0.1.3      Improved "Combiner" code, Feature-in 'force-to-float' option, added AmpHour exit criteria to Float/pst-float, 
//                              Allow Floating numbers for Volts and Amps multipliers - greater flexibility, reverse Amp Shunt polarity ASCII flag, 
//      06/20/2014  v0.1.2      Correct INA226 Amp Reg, improved Bluetooth code. 
//      05/17/2014  v0.1.1      Adaptive Acpt Mode time based exit, new ASCII commands, Stall detect and PWM cap, Field drive capping when stalled.
//      04/08/2014  v0.1.0      Code updated to match V0.1.2 of the PCB design - new FET driver, simpler DIP switches, dropped Dual&raw VBat. 
//                              Upgraded to INA-226, revised Bluetooth module.  Added Load Dump pullback logic.  User input to override autovolt detection,
//                              external charge source Amp offset, config lockdown (Security).  Updated default Charge Profiles.  Enabled Voltage-only
//                              capability (shunt not connected).
//      08/24/2013  v0.0.3      1st usable code release -  core regulation and auto-sizing works at basic level.
//      07/23/2013  v0.0.2      Initial debug with regulator hardware - still some issues, but posting to allow external GUI app development
//      06/11/2013  v0.0.1      1st edition posted, has only been tested in UNO card.
//      05/28/2013  v0.0.0      1st edits, code cut from v0.1.0 of the Smart Engine Control and Alternator Regulator source.
//
//
//
//
//
//------------------------------------------------------------------------------------------------------
//
//

                                               
                                                
#include "Config.h"
#include "Flash.h"
#include "CPE.h"
#include "AltReg_Serial.h"
#include "Alternator.h"
#include "Sensors.h"
#include "AltReg_CAN.h"




//
//------------------------------------------------------------------------------------------------------
//
//      Global Variables
//
//              These control major functions within the program
//
//
//

                                                                     
unsigned        faultCode;                                              // If alternatorState is set to FAULTED, this holds a # that indicates what faulted.




unsigned        LEDPattern;                                             // Holds the requested bit pattern to send to the LED.  Send LSB 1st.
unsigned        LEDBitMask;                                             // Used to select one bit at a time to 'send to the LED'
int8_t          LEDRepeat;                                              // How many times were we requested to show this pattern?
unsigned        LEDTiming;                                              // How quickly should we send out the Bits?
unsigned        LEDBitCntr;                                             // How many bits to we have left in the LED bit pattern?
unsigned long   LEDUpdated;                                             // When was the LED last changes in accordance to the bit-pattern?
bool            LEDFOMirror;                                            // When Blinking Out to the LED, should we also blink out to the Feature-Out port?  (requires #include FEATURE_OUT_LAMP to be defined)

int8_t          SDMCounter              = SDM_SENSITIVITY;              // Use this to moderate the number of times the Serial port is updated with debug info.
bool            sendDebugString         = false;                        // By default, lets assume we are NOT sending out the debug string.  Unless overridden by $EDB:, or
                                                                        // if #define DEBUG is present (look in startup() )


char const firmwareVersion[] = REG_FIRMWARE_VERSION;                    // Sent out with SST; status string and CAN initialization for product ID.

                                                                        
#ifdef FEATURE_OUT_COMBINER
  bool          combinerEnabled = false;                                // Is the combiner currently enabled?  Used in part to allow a hysteresis check before disabling on low voltage.
  #endif



  
        
        
        

 /***************************************************************************************
 ****************************************************************************************
 *                                                                                      *
 *                              STARTUP FUNCTIONS                                       *
 *                                                                                      *
 *                                                                                      *
 *      This is called one time at controller startup.                                  *
 *                                                                                      *
 *                                                                                      *
 ****************************************************************************************
 ****************************************************************************************/


 
    
    

//
//------------------------------------------------------------------------------------------------------
// Start-up routine.  
//
//      For now both cold start and Watchdog restart will come through here
//
//
//
//------------------------------------------------------------------------------------------------------

void setup() {

  wdt_enable(WDT_PER);                                                                  // Because we do SOOO much during setup use the Watchdog during startup...

  Serial.begin(SYSTEM_BAUD);                                                            // Start the serial port.

    #ifdef DEBUG
        sendDebugString = true;                                                         // Send out debug string if in debug mode :-)
        Serial.print("Size of EEPROM being used (out of 2048 bytes) = ");
        Serial.println(sizeof(CCS)  + CCS_FLASH_LOCAITON);
        #endif
        

   #ifdef SIMULATION
        randomSeed(analogRead(0));                                                      // Prime the random number generator (used during VBat simulation)
        #endif


        //---  Assign Atmel pins and their functions.  
        //         Am using Arduino calls as opposed to Atmel - to make things easier to read.
        //         Make sure to also see the Initialze_xxx(); functions, as they will also assign port values.
  pinMode(LED_RED_PORT,         OUTPUT);
  pinMode(LED_GREEN_PORT,       OUTPUT);                                                // Note that on the non-system board (the original one), the GREEN LED is
                                                                                        // the same port as the RED one.  Doing it this way saves some #ifdef's...
  pinMode(FEATURE_OUT_PORT,     OUTPUT);
  pinMode(FEATURE_IN_PORT,      INPUT);                                 

  
        //----  DIP switch ports.
  
#ifdef DIP_BIT0  
  pinMode(DIP_BIT0,             INPUT_PULLUP);                                          // Some boards do not use DIP-0 as an input.
  #endif                                                            
  pinMode(DIP_BIT1,             INPUT_PULLUP);                                          // We will use the weak internal Atmel Pull-ups for the DIP switch.
  pinMode(DIP_BIT2,             INPUT_PULLUP);
  pinMode(DIP_BIT3,             INPUT_PULLUP);
  pinMode(DIP_BIT4,             INPUT_PULLUP);
  pinMode(DIP_BIT5,             INPUT_PULLUP);                                          // BTW, this kind of shows a key delta between Arduino and Atmel programming.
  pinMode(DIP_BIT6,             INPUT_PULLUP);                                          // We could accomplish all these settings with just a couple of register writes,
  pinMode(DIP_BIT7,             INPUT_PULLUP);                                          // as opposed to the dozen or so 'pinMode' calls used in Arduino.





  
  wdt_reset();                                                                          // Pat-pat..

  


        //--------      Adjust System Configuration
        //              Now that things are initialized, we need to adjust the system configuration based on the Users settings of the DIP switches.
        //              We also need to fetch the EEPROM to see if there was a saved 'custom' configuration the user had entered via the GIU.
        //              
        //              Note the DIP switches (and hence any selections / adjustments) are sampled ONLY at Power up,
        //
        //




                                        //------  Read the DIP Switches, and make adjustments to system based on them
    uint8_t dipSwitch;                                                                   
    dipSwitch = readDipSwitch();                                                        // Read the Dip Switch value                          
                                        
                                                                                        // For Stand-along (2nd gen) regulator, DIP Switch mapping is:
                                                                                        //   Bit 0 = Bluetooth Power Enable (not readable via the Atmel CPU)
                                                                                        //   Bit 1 = Charge Profile 0 - Used to select which entry in chargeParms[] array we should be using.
                                                                                        //   Bit 2 = Charge Profile 1
                                                                                        //   Bit 3 = Charge Profile 2 
                                                                                        //   Bit 4 = Battery Capacity 0 - Used to indicate 'size' of battery, to adjust exit amp values
                                                                                        //   Bit 5 = Battery Capacity 1
                                                                                        //   Bit 6 = Small Alt Mode     - Is the Alternator a small capability, and hence the amps should be limited to prevent overheating?
                                                                                        //   Bit 7 = Tachometer Mode    - Turn on to make sure some small level of field PWM is always sent, even during Float and Post-Float modes..
   #ifdef STANDALONE
     cpIndex       =  (dipSwitch >> 1) & 0x07;                                          //  Mask off the bits indicating the Battery Type selection
     systemAmpMult = ((dipSwitch >> 4) & 0x03) + 1;                                     //  Mask off the bits indicating the Battery Capacity selection and adjust to 1..4x
     smallAltMode  =  (dipSwitch >> 6) & 0x01;                                          //  Mask off the bit indicating Small Alternator Capability
     tachMode      =  (dipSwitch >> 7) & 0x01;                                          //  Mask off the bit indicating Tachometer mode
     #endif
   


                                                                                        // On CAN enabled (3rd gen) regulator, DIP Switch mapping is:
                                                                                        //   Bit 0..1 = System / Battery Number to associate with - to look for an RBM.
                                                                                        //   Bit 2..4 = Charge Profile  (If not under control of RBM - Remote Battery Master)
                                                                                        //   Bit 5..6 = Battery Capacity 0 - Used to indicate 'size' of battery (If not under control of RBM)
                                                                                        //   Bit 7 = Small Alt Mode     - Is the Alternator a small capability, and hence the amps should be limited to prevent overheating?
   #ifdef SYSTEMCAN                                                                                     
   
     batteryInstance     = ((dipSwitch)      & 0x03) + 1;                               //  Mask off the bits indicating the Battery ID being associated with (1..8)
     cpIndex             =  (dipSwitch >> 2) & 0x07;                                    //  Mask off the bits indicating the Battery Type (CPE) selection
     systemAmpMult       = ((dipSwitch >> 5) & 0x03) + 1;                               //  Mask off the bits indicating the Battery Capacity selection and adjust to 1..4x
     smallAltMode        =  (dipSwitch >> 7) & 0x01;                                    //  Mask off the bit indicating Small Alternator Capability
     tachMode            =  false;                                                      //  No DIP switch for this, so assume do-not-enable TACH mode (User is able to override via ASCII command, we pick that up later)
     #endif

   
   

              
                                        //------  Fetch Configuration files from EEPROM.  The structures already contain their heir 'default' values compiled FLASH.  But we check to see if there are 
                                        //        validated user-saved overrides in EEPROM memory.

                                                                                        
   read_SCS_EEPROM(&systemConfig);                                                      // See if there are valid structures that have been saved in the EEPROM to overwrite the default (as-compiled) values
   read_CAL_EEPROM(&ADCCal);                                                            // See if there is an existing Calibration structure contained in the EEPROM.

   #ifdef SYSTEMCAN                                                                                     
    read_CCS_EEPROM(&canConfig);     
    #endif
                                                                                    

                                                                                        
   thresholdPWMvalue = systemConfig.FIELD_TACH_PWM;                                     // Transfer over the users desire into the working variable.  If -1, we will do Auto determine.  If anything else we will just use that
                                                                                        // value as the MIN PWM drive.  Note if user sets this = 0, they have in effect disabled Tack mode independent DIP switch.
   if (systemConfig.FORCED_TM == true)
      tachMode = true;                                                                  // If user has set a specific value, or asked for auto-size, then we must assume they want Tach-mode enabled, independent of the DIP


   


                                        //---  Initialize library functions and hardware modules
                                        //
    
   initialize_sensors();
   initialize_alternator();
  
  
  #ifdef STANDALONE                                                                     // Startup the stand-alone regulators Vbat and Amps sensor.
    config_BT(systemConfig.USE_BT);                                                     // Configure the Bluetooth, and if user has disabled it via software - turn it off as well...
    #endif
    
           
  #ifdef  SYSTEMCAN 
    initialize_CAN();
    #endif
         
                                                                                        
                                        //-----  Sample the System Voltage and adjust system to accommodate different VBats
                                        //
                                        
   delay (100);                                                                         // It should have only take 17mS for the INA226 to complete a sample, but let's add a bit of padding..                                   
   read_ALT_VoltAmps();                                                                 // Sample the voltage the alternator is connected to 
     
   if       (measuredAltVolts < 17.0)   systemVoltMult = 1;                             //  Likely 12v 'system'
   else  if (measuredAltVolts >= 40.0)  systemVoltMult = 4;                             //  Must be 48v 'system'
   else                                 systemVoltMult = 2;                             //  Anything in-between we will treat as a 24v 'system' 
                                                                                        //    Note that beginning with v1.0.3, '32v' auto-select has been removed, too risky - user should fix voltMult using $SCO: command




                                        //------ Now that we have done all the above work, let's see if the user has chosen to override any of the features!

   if (systemConfig.CP_INDEX_OVERRIDE != 0)    cpIndex        = systemConfig.CP_INDEX_OVERRIDE - 1;
   if (systemConfig.BC_MULT_OVERRIDE  != 0.0)  systemAmpMult  = systemConfig.BC_MULT_OVERRIDE;
   if (systemConfig.SV_OVERRIDE       != 0.0)  systemVoltMult = systemConfig.SV_OVERRIDE;
   if (systemConfig.ALT_AMPS_LIMIT    !=  -1)  altCapAmps     = systemConfig.ALT_AMPS_LIMIT;            // User has declared alternator size - make sure it is recognized before getting into things.
                                                                                                        // (Needed here in case user has forced regulator to float via feature_in out the door
                                                                                                        //  bypassing RAMP: mode where this variable would normally get checked)
 #ifdef  SYSTEMCAN 
    if (canConfig.BI_OVERRIDE != 0)            batteryInstance = canConfig.BI_OVERRIDE;
    #endif
    
    
         

        
   
   if (read_CPS_EEPROM(cpIndex, &workingParms) != true)                                 // See if there is a valid user modified CPE in EEPROM we should be using.
         transfer_default_CPS(cpIndex, &workingParms);                                  // No, so prime the working Charge profile tables with default values from the FLASH (PROGMEM) store.


  
 
        //---- And after all that, is the user requesting a Master Reset?

  #ifdef FEATURE_IN_RESTORE
    #ifdef FEATURE_IN_DISABLE_CHARGE                                                    // If this feature is enabled, skip master reset with CPE #8 (LI profile), as Feature-in can be used to disable charger by a BMS...
      if (cpIndex != 7)                                                                 // So in a very sneaky way, we add on an additional 'if' before continuing on..
      #endif

     if (feature_in(true) == true) {                                                    // Check feature_in port, and do all the 'debounce' when doing so.
                                                                                        // feature_in is active, so do a restore all now. 
        systemConfig.CONFIG_LOCKOUT = 0;                                                // As this is a HARDWARE restore-all, override any lockout.
        restore_all();                                                                  // We will not come back from this, as it will reboot after restoring all.
        }
     #endif



}                                                                                       // End of the Setup() function.



















/****************************************************************************************
 ****************************************************************************************
 *                                                                                      *
 *                              SUPPORTIVE FUNCTIONS                                    *
 *                                                                                      *
 *                                                                                      *
 *      This routines are called from within the Mainloop functions or Startup.         *
 *      These are mostly low-level functions to read sensors, keyboards, refresh        *
 *         displays, etc...                                                             *
 *                                                                                      *
 *                                                                                      *
 ****************************************************************************************
 ****************************************************************************************/



#ifdef STANDALONE                                                                         // All of this  Bluetooth stuff is ONLY on the stand-alone version..

//------------------------------------------------------------------------------------------------------
// Configure Bluetooth
//
//      This function is called from Startup.  It will look for the BT module and verify its name, password, mode of operation, etc.
//      and make configuration changes if needed.
//
//      If FALSE is passed in, after making configuration changes, this function will place the Bluetooth module into a powered-down state.
//
//      Note that this function CAN take a long time to complete (Several seconds), depending on the condition of the module...
//      Note also that we ASSUME to BT module is fixed at 9600 baud via hardware selection (or pre-configuration)
//
//------------------------------------------------------------------------------------------------------

void config_BT(bool  enableBT) {

                                                            
   char    buffer[15 + MAX_NAME_LEN + MAX_PIN_LEN];                                     // Used to assemble commands to the Bluetooth
   bool    BTreboot_needed;


        BTreboot_needed = false;                                                        // Assume we do NOT need to reconfigure the Bluetooth module.



        //---   IS there a BT module out there?
        //

                                                                                        // Place RN-41 module into Command mode. 
        send_command("$$$");                                                            // (send_command() makes sure the Serial Buffers are flushed before starting a new sequence.)
        if (!wait_string ("CMD"))  return;                                              //  If no Bluetooth is found, just skip the rest!


        wdt_reset();                                                                    // Some of these BT ops can take a while, make sure the Dog is kept happy :-)



        //----  Seems so, Is it configured as we want it?
        //

        send_command("GN\r");                                                           // Ask for the current configured Name.
        if (!wait_string(systemConfig.REG_NAME)) {                                      // Name returned is something other then what we want.

           strcpy(buffer,"SN,");                                                        // Set Device NAME to what we want.
           strcat(buffer,systemConfig.REG_NAME);
           strcat(buffer,"\r");

           send_command(buffer);
           wait_string ("AOK");                                                         // Let it take.

           BTreboot_needed = true;                                                      // And set flag indicated we need to 'reboot' the BT when finished.
           }

        wdt_reset();


        
        send_command("GP\r");                                                           // Ask for the current configured Password
        if (!wait_string(systemConfig.REG_PSWD)) {                                       // Password returned is something other then what we want.

           strcpy(buffer,"SP,");                                                        // Set Device PASSWORD.
           strcat(buffer,systemConfig.REG_PSWD);
           strcat(buffer,"\r");

           send_command(buffer);
           wait_string ("AOK");

           BTreboot_needed = true;                                                      // And set flag indicated we need to 'reboot' the BT when finished.
           }


        wdt_reset();



        //----  Does the user want to turned off (via Software)?
        //

        if (enableBT == false) {                                                        // BT has been disabled by a user in software config.
                
           send_command("Q,1\r");                                                       // Place module into powered down mode
           wait_string ("AOK");                                                         // Let it take.
           send_command("Z\r");                                                         // and low power mode
           wait_string ("AOK");

           BTreboot_needed = true;                                                      // And set flag indicated we need to 'reboot' the BT when finished.
           }


        wdt_reset();



        //----  Finally, let's look at some other config details, make sure they are how we want them.
        //

        send_command("GA\r");                                                           // Current Mode = Legacy Password?
        if (!wait_string("4")) {                                                        // 4 = legacy password.  (Note dbl quote " vs. single ' is used.
                                                                                        //   We need to mass a NULL terminated string, not a single character)
           send_command("SA,4\r");                                                      
           wait_string("AOK");  

           BTreboot_needed = true;                                                      // And set flag indicated we need to 'reboot' the BT when finished.
           }

        wdt_reset();




        #ifdef DEBUG                                                                    // Set ability to place RN41 into command mode.
        send_command("GT\r");                                                           // in DEBUG, allow both local as well as remote to send 'CMD' and config BT device.
        if (!wait_string("255")) {

           send_command("ST,255\r");                                                    
           wait_string("AOK");  

           BTreboot_needed = true;                                                      
           }
        #else
        send_command("GT\r");                                                           // However in Normal mode, only allow the local Atmel CPU to enter command mode.
        if (!wait_string("253")) {                                                      // Blocking any outside attempt to hi-jack the RN41 Bluetooth!

           send_command("ST,253\r");                                                    
           wait_string("AOK");  

           BTreboot_needed = true;
           }
        #endif

        wdt_reset();






        //----  OK, all done.  Do we need to 'reboot' the module?
        //

        if (BTreboot_needed == true) {                                                  // We made some changes, and need to ask the RN-41 to reboot.
            send_command("R,1\r");                                                      // All done, reset RN41 module with new configuration.
            wait_string ("Reboot");
            delay(500);                                                                 // Specs says it takes 500mS after reboot to respond.
            }
        else
           send_command("---\r");                                                       // Else just drop out of Command Mode.


        wdt_reset();                                                                    // One last time, before we say good-by to config_BT()
        
   
}










//----  Helper function for Bluetooth.
//

void send_command(const char *buffer) {                         // Transmits out the passed string to the serial buffer, after making sure the buffers are flushed.
                                                                // This helps avoid out-of-step issues with the RN-41 device...

   Serial.flush();                                              // Make sure the output and input buffers are clear before sending out a new command.
   while (Serial.available()>0)
          Serial.read();                                        // Clear the input buffer, just dump anything that comes in..

   Serial.write(buffer);                                        // NOW we can send the actual command.
   Serial.flush();                                              // And wait for the output buffer to clear
   delay(10);                                                   // This, for some reason, helped a lot.  let the RN-41 get things in order before moving on I guess, or something 
                                                                // was changed in the Arduino Flush lib function. . . .
   }



bool wait_string(const char match[]) {                          // Read the serial port until it finds a match for the passed string, or BT timeout has been exceeded.
   unsigned long  startedLooking;                               // At what time did we start looking to for this string?  Used for time-out
   int            index = 0;
 
   startedLooking = millis();

   do {                                                         // Timeout loop . . .
     while (Serial.available()>0){                              //  . . . wrapping around character available / match loop.
        if (Serial.read() == match[index])
                index++;                                        // Looking promising for a match!
            else
                return(false);                                  // No Go.  (Am rather harsh here, response must match EXACTLY as expected to return True.)

        if (match[index] == '\0')                               // Did we make it to the NULL terminator in the passed character string array?
            return(true);                                       // Yes, return success!
        }
    } while ((millis() - startedLooking) < BT_TIMEOUT);

   return(false);                                               // Looks like we timed out...
   }

#endif    // STANDALONE




//------------------------------------------------------------------------------------------------------
// Read DIP Switch
//      This function is called from Startup and will read the DIP switch.  It will return all 7 usable bits 
//      assembled into one returned value.  It is also called during the $SCN: command to make sure the DIP switch 
//      is set to all-on before allowing initial updating of the Name and Password.
//
//
//       For stand-alone regulator, DIP Switch mapping is:
//              Bit 0 = Bluetooth Power Enable (not readable via the Atmel CPU --  0 is always returned.)
//              Bit 1 = Charge Profile bit-0    - Used to select which entry in chargeParms[] array we should be using.
//              Bit 2 = Charge Profile bit-1
//              Bit 3 = Charge Profile bit-2 
//              Bit 4 = Battery Capacity bit-0  - Used to indicate 'size' of battery, to adjust exit amp values
//              Bit 5 = Battery Capacity bit-1
//              Bit 6 = Small Alt Mode           = The Alternator has a small capability, and hence the amps should be limited to prevent overheating
//              Bit 7 = Alt Tach Mode            = Will cause the Regulator to always have some small value of PWM present to allow for external Alt drive Tachs to work
//                                                 Caution with this mode, as it MIGHT cause some overcharging to the battery. . . .
// 
//
//      CAN enabled (systems) regulator, DIP switch mapping is:
//              Bit 0..2 = Which 'battery' / CAN systemID should we look to be connected to? 
//              Bit 3    = Small Alt Mode         = The Alternator has a small capability, and hence the amps should be limited to prevent overheating
//
//------------------------------------------------------------------------------------------------------


uint8_t readDipSwitch() {

uint8_t dipSwitch;


#ifdef SIMULATION
  #ifdef STANALONE
//   return(B00111100);                                                                         // Testing, CPE=7th, Bat Size = 10, Small Alt Mode = off, Tach mode = 0ff, BIG HD FLA battery (Industrial, etc) w/OC mode enabled
     return(B00110100);                                                                         // Testing, CPE=3rd, Bat Size = 10, Small Alt Mode = off, Tach mode = 0ff, BIG FLA battery (T-105, etc)
//   return(B00000010);                                                                         // Testing, CPE=2nd, Bat Size = 10, Small Alt Mode = off, Tach mode = 0ff
//   return(B00001110);                                                                         // Testing, CPE=8th, Bat Size = 10, Small Alt Mode = off, Tach mode = 0ff  LiFePO4 testing
//   return(B01000010);                                                                         // Testing, CPE=2nd, Bat Size = 10, Small Alt Mode = ON,  Tach mode = 0ff
     #endif

  #ifdef  SYSTEMCAN
     return(B01101010);                                                                        // Testing, CPE=3rd - BIG FLA battery (T-105, etc), Bat Size = 11, Small Alt Mode = off, BatID = 1 
     #endif
     
#endif
     

     
     
  #ifdef DIP_BIT0   
    dipSwitch  =  digitalRead(DIP_BIT0);                                                        // Start with the LSB switch,
  #else
    dipSwitch  = 0;                                                                             // Stand-alone regulator uses the LSB for Bluetooth Power
  #endif

  dipSwitch |= (digitalRead(DIP_BIT1) << 1);                                                    // Then read the DIP switches one switch at a time, ORing them at their appropriate bit location,
  dipSwitch |= (digitalRead(DIP_BIT2) << 2); 
  dipSwitch |= (digitalRead(DIP_BIT3) << 3);
  dipSwitch |= (digitalRead(DIP_BIT4) << 4);
  dipSwitch |= (digitalRead(DIP_BIT5) << 5);
  dipSwitch |= (digitalRead(DIP_BIT6) << 6);
  dipSwitch |= (digitalRead(DIP_BIT7) << 7);
     
  return(~dipSwitch);                                                                           // Invert the returned value, as the DIP switches are really pull-downs.  So, a HIGH means switch is NOT active.

}








//------------------------------------------------------------------------------------------------------
// Feature in
//      This function will sample the Feature In port and return TRUE if it is currently being held High.
//      Passed flag will tell us if we should just check the Feature_in port, doing all the de-bounce 
//      checking in this one call.  Or if we should just check the debounce and return the last known state
//      if we have not completed the required number of samples (to be checked again at a later time).
//      (e.g:  Should this function behave as a Blocking or non-blocking function??)
//
//      Returns True only if the feature_in pin has been held active during the entire debouncing time.
//
//
//------------------------------------------------------------------------------------------------------

bool feature_in(bool waitForDebounce) {
  bool static    lastKnownState  = false;                                               // Used to retain the featureIn statue during 'next' debounce cycle.
  bool static    proposedState   = false;                                               // Used to see if we have a constant state during entire debounce time period.
  bool           readState;
  int8_t  static debounceCounter = DEBOUNCE_COUNT;                                      // Used to count-down sampling for de-bouncing. 
  int8_t         stuckCounter    = 2*DEBOUNCE_COUNT;                                    // OK, if there is a LOT of noise on the feature-in pin, only stay in this routing for so long.
                                                                                        //  (

  while(--stuckCounter > 0) {                                                           // A bit of safety, this will prevent is from being stuck here forever if there is a noisy feature_in...


     readState = (digitalRead(FEATURE_IN_PORT) == HIGH);                                // So this 'extra' step to make sure there is no issues between typing of TRUE and HIGH.


     if (readState == proposedState) {                                                  // feature-in looks to be stable.
        if (--debounceCounter <= 0) {                                                   // And it appears it has been for some time as well!
           lastKnownState  = proposedState;                                             //   Recognize the new 'state'
           debounceCounter = DEBOUNCE_COUNT;                                            //   Reset the debounce counter to take a new go at the feature_in port
           break;                                                                       //   And return the new state!
           }
        }
     else {
        proposedState   =  readState;                                                   // Nope, not stable.  Looks like we have a new candidate for the feature_in() port.
        debounceCounter =  DEBOUNCE_COUNT;                                              // Reset the counter and let’s keep looking to see if it remains this way for a while.
        }
     

    if (waitForDebounce == true)
         delay(DEBOUNCE_TIME);                                                          // They want us to do all the debouncing, etc..
    else
        break;                                                                          // Don't want to stick around, so break out of this While() loop.

  }



 return(lastKnownState); 

}






//------------------------------------------------------------------------------------------------------
// Manage System State 
//      This function look at the system and decide what changes should occur.
//
//      For the stand-alone alternator, it is much simpler - if the Alternator is not running, and not Faulted, start it up!
//
// 
//------------------------------------------------------------------------------------------------------


void manage_system_state() {



   
                        // --------  Do we need to start the Alternator up?

 if ((alternatorState == unknown)  || (alternatorState == disabled)) {  
                                                                                                        

        set_ALT_PWM(FIELD_PWM_MIN);                                                             // Make sure Field is at starting point. 
        set_ALT_mode(pending_R);                                                                // Prepare for Alternator, let manage_ALT() change state into Ramping
        reset_run_summary();                                                                    // Zero out the accumulated AHs counters as we start this new 'charge cycle'
        }


 }














//------------------------------------------------------------------------------------------------------
// Reboot
//
//      This function will disable the Alternator and then forces the Atmel CPU to reboot via a watchdog timeout.
//      It is typically called making a change to the EEPROM information to allow the controller to re-initialize
//      
// 
//
//------------------------------------------------------------------------------------------------------


        
void reboot() {

     set_ALT_PWM(0);                                                            // 1st, turn OFF the alternator, All the way OFF as we will no longer be managing it...
     #ifdef CHARGE_PUMP_PORT
        analogWrite(CHARGE_PUMP_PORT,0);                                        // and the Charge Pump as well.
        #endif

     commit_EEPROM();                                                           // Make sure any clean-up in the EEPROM is done (Specificly for those which use EEPROM emulation via flash)
     
     wdt_enable(WDT_PER);                                                       // JUST IN CASE:  Make sure the Watchdog is enabled! 
                                                                                //  (And in any case, let it be the shorter reboot watchdog timeout.

     blink_LED (LED_RESETTING,LED_RATE_FAST, -1, true);                         // Show that something different is going on...

     Serial.write("RST;\r\n");
     Serial.flush();                                                            // And then make sure the output buffer clears before proceeding with the actual reset.

   #ifdef OPTIBOOT
     while (true)                                                               // Optiboot!
        refresh_LED();                                                          // Sit around, blinking the LED, until the Watchdog barks.
        

     #else                                                                      // The standard Arduino 3.3v / atMega328 has a bug in it that will not allow the watchdog to 
                                                                                // work - instead the system hangs.  Optiboot fixes this.  (Note the standard bootload for the 
                                                                                // Arduino UNO card is actually an Optiboot bootloader..)
                                                                                // If we are not being placed into an Optiboot target, we need to do something kludge to cause the
                                                                                // system to reboot, as opposed to leverage the watchdog timer.  So:

        void(* resetFunc) (void) = 0;                                           // Kludge - just jump the CPU to address 0000 in the EEPROM.
                                                                                // declare reset function @ address 0.  Work around for non-support in watchdog on Arduino 3.3v
        resetFunc();                                                            // Then jump to it to 'restart' things.
        #endif




}





/****************************************************************************************
 ****************************************************************************************
 *                                                                                      *
 *                              COMMUNICATION FUNCTIONS                                 *
 *                                                                                      *
 *                                                                                      *
 *      These routines are used to communicate to and from the Regulator, either via    *
 *      Bluetooth or the Serial Port, as well as the CAN port (if present)              *
 *                                                                                      *
 *                                                                                      *
 ****************************************************************************************
 ****************************************************************************************/



//------------------------------------------------------------------------------------------------------
//
// Update LED
//
//      This routing is called from Mainloop and will set the LED to the blinking pattern appropriate to the
//      current Alternator Status and flash it.  A trick to force this utility to refresh the blinking pattern is to 
//      set the global variable "LEDRepeat" = 0;   This will terminate the current blinking pattern smoothly and 
//      let this routine reset the pattern to the new one.  (ala, use when changing states in manage_alternator(); )
//
//      If #include FEATURE_OUT_LAMP is defined, this routine will also update the Feature-Out port to FULL ON
//      when the alternator is in a no-charge state.
//
//
//
//
//------------------------------------------------------------------------------------------------------



void update_LED() {

   if (refresh_LED() == true)                                           // If there is already a pattern blinking out, let it finish before overwriting it.
        return;
 


  switch (alternatorState) {
        case ramping:
        case determine_ALT_cap:
        case bulk_charge: 
        case RBM_CVCC:                  blink_LED (LED_BULK,       LED_RATE_NORMAL, -1, false);
                                        break;

        case acceptance_charge:         blink_LED (LED_ACCEPT,     LED_RATE_NORMAL,   -1, false);
                                        break;

        case overcharge_charge:         blink_LED (LED_OC,         LED_RATE_SLOW, -1, false);
                                        break;

        case float_charge:
        case forced_float_charge:
        case post_float:                blink_LED (LED_FLOAT,      LED_RATE_NORMAL, -1, false);
                                        break;

        case equalize:                  blink_LED (LED_EQUALIZE,   LED_RATE_FAST,   -1, true);                  // Include the dash LAMP while equalizing
                                        break;


        default:                        blink_LED (LED_IDLE,       LED_RATE_NORMAL, -1, false);
                                        break;
        }


}





//------------------------------------------------------------------------------------------------------
//
// Blink LED
//
//      This routing will initiate a new 'LED' status update blinking cycle.  It will shift out a pattern based
//      on the pattern given to it.   Note that if a LED flashing pattern is already in effect, calling this 
//      will abort that existing one and replace it with this new request.
//      
//      There is a dependency on calling refresh_LED() every once and a while in order to actually update the LED.
//
//              pattern --> Pattern to send to LED, will be shifted out MSB 1st to the LED every timing period
//              timing  --> Number of mS that should pass between each bit-pattern update to the LED
//              repeat  --> Once the pattern has finished, repeat it this many times.  -1 = repeat until stopped.
//              mirror  --> Should we mirror the blinking on the FEATURE_OUT port (requires #include FEATURE_OUT_LAMP to be defined) 
//
//
//
//------------------------------------------------------------------------------------------------------



void blink_LED (unsigned pattern, unsigned led_time, int8_t led_repeat,  bool mirror) {

        LEDPattern   = pattern;
        LEDTiming    = led_time;
        LEDRepeat    = led_repeat;
        LEDFOMirror  = mirror;


        digitalWrite(LED_RED_PORT,   LOW);                              // As we are starting a new pattern, make sure the LED is turned off
        digitalWrite(LED_GREEN_PORT, LOW);                              // in preparation for the new bit pattern to be shifted out.

        LEDBitMask = 0;                                                 // We will start with the MSB in the pattern
        LEDUpdated = millis();
        
}









//------------------------------------------------------------------------------------------------------
//
// Refresh LED
//
//      This routine will look to see if sufficient time has passed and change the LED to the next bit in the bit pattern.
//      It will repeat the pattern x times as determined by blink_LED() call and return TRUE.
//      If the pattern (and repeats) has been exhausted, the LED will be turned off and FALSE will be returned.
//
//
//
//------------------------------------------------------------------------------------------------------


bool refresh_LED() {


   if ((LEDBitMask != 0) && ((millis() - LEDUpdated) <= LEDTiming))     // Time to change a bit?
        return (true);                                                  //  Nope, wait some more..  (Unless we are starting a new pattern, then let's get that 1st bit out!)


   if ((LEDBitMask == 0) && (LEDRepeat == 0)) {
        digitalWrite(LED_RED_PORT,   LOW);                               //  We are all done, make sure the LED is off . . .
        digitalWrite(LED_GREEN_PORT, LOW);    

        #ifdef FEATURE_OUT_LAMP
          if (LEDFOMirror == true)
                digitalWrite(FEATURE_OUT_PORT, LOW);                    //  And adjust the LAMP if so configured.
        #endif

        return(false);                                                  //  . . . and return that there is no more
        }


   if (LEDBitMask == 0) {                                               // Reset the LED mask to start with MSB?
        LEDBitMask = 0x8000;                                            // Yes
        LEDRepeat--;                                                    // and take the Counter down one
        }
     else
        LEDBitMask = LEDBitMask >> 1;                                   // Adjust the bit mask and counter, and let the next time back into
                                                                        // refresh_LED() decide if we are finished.  (This will assure if the MSB is
                                                                        // LED-on, there it will be light for timing mS..)

   if ((LEDPattern & LEDBitMask) == 0) {
        digitalWrite(LED_RED_PORT,   LOW);                              // LED should be off.
        digitalWrite(LED_GREEN_PORT, LOW); 
        }
    else {                                                              // LED should be on, but which color?
        if (alternatorState == FAULTED) 
            digitalWrite(LED_RED_PORT,   HIGH);                         // FAULTED status gets RED LED blinking.
        else {
            digitalWrite(LED_GREEN_PORT, HIGH);                         // Normal status gets the GREEN one
#ifdef SYSTEMCAN
            if ((CAN_RBM_sourceID != 0) && (!ignoringRBM))
                digitalWrite(LED_RED_PORT, HIGH);                       // But if we are taking orders from a CAN MASTER. change the status LED to yellow.
#endif    
            }
        }



#ifdef FEATURE_OUT_LAMP
    if (LEDFOMirror == true) {                                          //  And adjust the LAMP if so configured.
      if ((LEDPattern & LEDBitMask) == 0)       
        digitalWrite(FEATURE_OUT_PORT, LOW);
      else
        digitalWrite(FEATURE_OUT_PORT, HIGH);
      }
   else {                                                               //  Lamp not being used for active blinking, lets see if we should just have it on :-) 
      if ((alternatorState == unknown)  || (alternatorState == disabled) || (alternatorState == pending_R))     
        digitalWrite(FEATURE_OUT_PORT, HIGH);                           //    . .  set it to ON if we are not charging, else to OFF if all is OK.
      else 
        digitalWrite(FEATURE_OUT_PORT, LOW);
     }

#endif

        

   LEDUpdated = millis();                                               //  Reset timer, and    
   return (true);                                                       //    say there is More to Come!
   
}
















/****************************************************************************************
 ****************************************************************************************
 *                                                                                      *
 *                              MAINLOOP SUPPORT FUNCTIONS                              *
 *                                                                                      *
 *                                                                                      *
 *      These routines are called from the Mainloop (as opposed to being inline)        *
 *      They are placed here to help readability of the mainloop.                       *
 *                                                                                      *
 *                                                                                      *
 ****************************************************************************************
 ****************************************************************************************/





//------------------------------------------------------------------------------------------------------
// Handle FEATURE_IN port
//
//      This function will check the Feature-In port is also checked in this function, 
//      to enable whatever it is configured for by the #defines
// 
//------------------------------------------------------------------------------------------------------

void handle_feature_in(void) { 
                //----- Check the Feature-in port
                //      Doing this 1st as the code below uses a lot of 'returns' to exit this function when it has finished processing input command strings...
 

  #ifdef FEATURE_IN_EQUALIZE
    #ifdef FEATURE_IN_DISABLE_CHARGE                                    // If this feature is enabled, skip equalize with CPE #8
      if (cpIndex != 7)                                                 // So in a very sneaky way, we add on an additional 'if' before continuing on..
    #endif
      if (feature_in(false) == true) {                                  // Check the feature-in port, but do NOT wait around for debouncing.. (We will catch it next time)
         if (((alternatorState  == acceptance_charge) || (alternatorState   == float_charge))      &&   // True, user is asking for Equalize.  Can they have it?
             ((workingParms.EXIT_EQUAL_AMPS == 0)     || (persistentBatAmps <= (workingParms.EXIT_EQUAL_AMPS * systemAmpMult))))    
                set_ALT_mode(equalize);                                                                 // OK, they want it - it seems like the battery is ready for it - so:  let them have it. 
         } else {
            if (alternatorState == equalize)                                                            // We are equalizing, and user dropped Feature-in line.
                set_ALT_mode(float_charge);                                                             // So stop equalizing, and go into Float...
         }  

  #endif



   #ifdef FEATURE_IN_DISABLE_CHARGE                                     
      if ((cpIndex == 7) && (alternatorState > pending_R)) {                                            // Check to see if we are being asked to force the mode into float via the feature-in port.
          if (feature_in(false) == true) {                                                              // An external device is telling us to keep out of bulk/accept/OC.  (e.g, a LiFoPO4 BMC)
              if (alternatorState != forced_float_charge)                                               // If not already in forced_float, change the state.
                set_ALT_mode(forced_float_charge);                                                  
              }
            else {                                                                                      // Fearure_in() is no longer being held active.
             if (alternatorState == forced_float_charge)                                                // All right, Feature-in port is NOT asking use to force float mode.   By chance, did it?
                 set_ALT_mode(ramping);                                                                 // Yes it had, and now it is no longer doing so.  So am external BMS must be asking us to restart charging.
            }
          }
   #endif
}





//------------------------------------------------------------------------------------------------------
//
// Update FEATURE_OUT port
//
//      This routine will manage the Feature Out port (other than Blinking) depending on how #defined are 
//      at the beginning of the program.  Note that 'communications' type usage of the Feature_out port
//      (Specifically, blinking of the light to mirror the LED in error conditions) is not handled here, but
//      instead as part of the BLINK_XXX() functions above.
//
//
//
//
//------------------------------------------------------------------------------------------------------


void update_feature_out(void) { 

   #ifdef FEATURE_OUT_LAMP                                                                      // This stub is here to prevent the user from enabling more than one feature at a time...
                                                                                                // Nothing here: LED mirror is handled above in all the BLINK() type functions.

   #elif defined FEATURE_OUT_COMBINER                                                           // Enable FEATURE_OUT port to go active when VBat exceeds ????, useful for connecting external relay to join 2nd battery
        switch (alternatorState) {
          case acceptance_charge:                                                               // During the Acceptance Phase, see if we are configured for a time-out...
                
                if ((millis() - altModeChanged) > COMBINE_ACCEPT_CARRYOVER)
                           combinerEnabled = false;                                             //  ...disable combiner.

                break;




          case bulk_charge:
                                                                                                // Go through a nested tree of decisions to see if we should enable the combiner
                if (measuredBatVolts >= (COMBINE_CUTIN_VOLTS   * systemVoltMult))   combinerEnabled = true;     // 1st check:  is VBat above the cut-in voltage?   Yes, enable the combiner.
                                                                                                                // Continue the remaining nested checks outside of the SWITCH, in that way
                                                                                                                // the boundary tests are always preformed.  (e.g., during carryover in acceptance mode)
                break;


        
          default:
                 combinerEnabled = false;                                                                       // All other modes, disable combiner

          }


        if (measuredBatVolts >= (COMBINE_DROPOUT_VOLTS * systemVoltMult))   combinerEnabled = false;            // 2nd check:  if VBat is too high, disable combiner
        if (measuredBatVolts <  (COMBINE_HOLD_VOLTS    * systemVoltMult))   combinerEnabled = false;            // 3rd check:  if VBat is below the cut-out, disable combiner.
                                                                                                                // All other cases, just leave it in the state it already is in.  This will provide for
                                                                                                                // a level of hysteresis between the cutin volts and the hold-volts levels




        if (combinerEnabled == true)                                                            // Now that is known which state we want, set the feature_out port.
              digitalWrite(FEATURE_OUT_PORT, HIGH);     
           else 
              digitalWrite(FEATURE_OUT_PORT, LOW);      
                





   #elif defined FEATURE_OUT_ENGINE_STOP                                                        // Enable FEATURE_OUT port to go active when we enter FLOAT mode.  Useful to auto-stop a DC generator when charging has finished.
        switch (alternatorState) {
          case float_charge:
          case forced_float_charge:
          case post_float:
                digitalWrite(FEATURE_OUT_PORT, HIGH);                                           // If we are in Float, or Post_float:  Enable Feature-out to indicate battery is all charged.
                break;

          default:
                digitalWrite(FEATURE_OUT_PORT, LOW);                                            // All other modes, Disable Feature-out port
          }

   #endif
   }






//------------------------------------------------------------------------------------------------------
// Check for Fault conditions 
//
//      This function will check to see if something has faulted on the engine.  
//
//      If a fault condition is found, the appropriate state will be set of FAULTED, the global variable 
//      faultCode will be set, and this function will return TRUE indicating a fault.
//
//      If no fault is found, this function will return FALSE.
// 
//------------------------------------------------------------------------------------------------------

bool  check_for_faults() {

        unsigned u;
        

        
        //----  Alternator doing OK?

        u = 0;                                                                                  // Assume there is no fault present.
        switch (alternatorState)  {
                case unknown:
                case disabled:
                case pending_R:
                case post_float:
                        break;

        

        
                case bulk_charge:                                                               //  Do some more checks if we are running.
                case determine_ALT_cap:
                case ramping:
                case acceptance_charge:
                case overcharge_charge:
                case RBM_CVCC:
                        if (measuredBatVolts            > (FAULT_BAT_VOLTS_CHARGE  * systemVoltMult))           u = FC_LOOP_BAT_VOLTS;
                                                                                                //  Slightly lower limit when not equalizing.
                                                
                                                                                                //  Then fall through and do the rest of the active checks              
                case equalize:
                case float_charge:
                case forced_float_charge:
                        if  (measuredBatVolts           > (FAULT_BAT_VOLTS_EQUALIZE * systemVoltMult))          u = FC_LOOP_BAT_VOLTS;  
                                                                                                // We check for Float overvolt using the higher Equalize level, because when we 
                                                                                                // leave Equalize we will go into Float mode.  This prevents a false-fault, though it
                                                                                                // does leave us a bit less protected while in Float mode...


                                        
                        if  (measuredAltTemp            > (systemConfig.ALT_TEMP_SETPOINT   * FAULT_ALT_TEMP))  u = FC_LOOP_ALT_TEMP;
                        if  (measuredAlt2Temp           > (systemConfig.ALT_TEMP_SETPOINT   * FAULT_ALT_TEMP))  u = FC_LOOP_ALT2_TEMP;
                        if  (measuredBatTemp            >  FAULT_BAT_TEMP)                                      u = FC_LOOP_BAT_TEMP;
                        if ((measuredBatVolts           < (FAULT_BAT_VOLTS_LOW * systemVoltMult)) && (fieldPWMvalue > (FIELD_PWM_MAX - FIELD_PWM_MIN)/3))
                                                                                                                u = FC_LOOP_BAT_LOWV;
                                                                                                // Check for low battery voltage, but hold off until we have applied at least 1/3 field drive.
                                                                                                // In this way, we CAN start charging a very low battery, but will not go wild driving the alternator
                                                                                                // if nothing is happening.  (ala, the ignition is turned on, but the engine not started.)

                
                        break;                                                                  // If we make it to here, all is well with the Alternator!



                default:                                                                        
                        u = FC_LOG_ALT_STATE;                                                   //  Some odd, unsupported mode.   Logic error!

                }


           if (u !=0) {
                alternatorState = FAULTED;
                faultCode  = u;
                }







        //----  System doing OK?

        u = 0;                                                                                  //  Again, use 'u' to receive fault-code if a fault is found.  For not assume no fault.
        if (cpIndex           != (cpIndex & 0x07))                     u = FC_LOG_CPINDEX;      // cpIndex pointer out of range?
        if (systemAmpMult     != constrain(systemAmpMult, 0.0, 10.0))  u = FC_LOG_SYSAMPMULT;   // systemAmpMult out of range?
        if  (measuredFETTemp   > FAULT_FET_TEMP)                       u = FC_SYS_FET_TEMP;     // Driver FETs overheating?

#ifdef SYSTEMCAN        
         if ((batteryInstance <= 0) || (batteryInstance > 100))        u = FC_LOG_BATTINST;      // Battery instance not between 1..100?
        #endif
        
        
        
                                                                        
        if (u !=0) {
          alternatorState = FAULTED;                                                            //   Just use the AlternatorState to indicate these two system faults,
          faultCode  = u;
          }
                        





        if (alternatorState     == FAULTED)     return (true);                                  //  Did something fault?
            else                                return (false);

}









//------------------------------------------------------------------------------------------------------
//
//      Handle FAULT condition
//
//
//      This function is called when the system is in a FAULTED condition.  It will make sure things are
//      shut down, and blink out the appropriate error code on the LED.
//
//      If the Fault Code has the 'restart flag' set (by containing 0x8000), after blinking out the fault code
//      the regulator will be restarted.  Else the regulator will continue to blink out the fault code.
//
//
//------------------------------------------------------------------------------------------------------

void handle_fault_condition() {

        unsigned j;
        char buffer[80];

        //-----  Make sure the alternator, etc. is stopped.
       set_ALT_PWM(0);                                                                           // Field PWM Drive to Low (Off) 
        #ifdef CHARGE_PUMP_PORT  
            analogWrite(CHARGE_PUMP_PORT,0);                                                     // and the Charge Pump as well.
            #endif
                
 

        j = faultCode & 0x7FFFU;                                                                // Masking off the restart bit.
        if (sendDebugString == true)
             snprintf_P(buffer,sizeof(buffer)-1, PSTR("FLT;,%d, ,%d,%d\r\n"),                   // Send out the Fault Code number, and some other info.... 
                        j,                      
                        (int) alternatorState,
                        fieldPWMvalue); 
        else 
             snprintf_P(buffer,sizeof(buffer)-1, PSTR("FLT;%d\r\n"), j);                        // Just send out the Fault Code number.

        Serial.write(buffer);
        send_outbound(true);                                                                    // And follow it with all the rest of the status information.



        blink_LED (LED_FAULTED, LED_RATE_FAST, 2, true);                                        // Blink out 'Faulted' pattern to both LED and LAMP
        while (refresh_LED() == true);                                                          // Send the pattern out to the LED.

        delay (1000);                                                                           // 1 second pause

        

        if (j >= 100)
           blink_out (((j/100) % 10));                                                          // Blink out Fault number 100's 1st

        blink_out (((j/ 10) % 10));                                                             // Blink out Fault number 10's 2nd
        blink_out ((j       % 10));                                                             // Blink out Fault number units last
        
        delay (5000);                                                                           // Pause 5 seconds
                                        
}




void blink_out(int digd) {

        static const PROGMEM  unsigned num2blinkTBL[9] = {                                      // Table used convert a number into a blink-pattern
        0x0000,                                                                                 // Blink digit '0'  (Note, Arduino pre-processor cannot process binary decs larger than 8 bits..   So, 0x____)
        0x0002,                                                                                 // Blink digit '1'
        0x000A,                                                                                 // Blink digit '2'
        0x002A,                                                                                 // Blink digit '3'
        0x00AA,                                                                                 // Blink digit '4'
        0x02AA,                                                                                 // Blink digit '5'
        0x0AAA,                                                                                 // Blink digit '6'
        0x2AAA,                                                                                 // Blink digit '7'
        0xAAAA                                                                                  // Blink digit '8'      For 9, you will need to blink out "1"+8 :-)
        };                                                                                      //   (BTW: This 1+8 is why the other digits appear to be 'shifted left' one bit, to make a smooth LED blink when doing a 9



     if (digd == 9) {
           blink_LED (0x0001, LED_RATE_NORMAL, 1, true);                                        // 9 has to be sent as an "1"+8
           while (refresh_LED() == true);       
           digd--;
           }

      blink_LED (pgm_read_dword_near(num2blinkTBL+digd), LED_RATE_NORMAL, 1, true);                             
      while (refresh_LED() == true);                                                            // Send the pattern out to the LED.
      delay (750);
      }













/****************************************************************************************
 ****************************************************************************************
 *                                                                                      *
 *                              MAIN LOOP                                               *
 *                                                                                      *
 *                                                                                      *
 *      This gets called repeatedly by the Arduino environment                          *
 *                                                                                      *
 *                                                                                      *
 *                                                                                      *
 ****************************************************************************************
 ****************************************************************************************/


void loop()  {

   if (alternatorState == FAULTED)  {
                wdt_disable();                                                                          // Turn off the Watch Dog so we do not 'restart' things.
                handle_fault_condition();                                                               // Take steps to protect system

                if (faultCode & 0x8000U)                                                                // If the Restart flag is set in the fault code,
                   reboot();                                                                            //    then do a forced software restart of the regulator

                return;                                                                                 // Else bail out, we will need a full hardware reset from the user.
                }







  //
  //
  //-------   OK, we are NOT in a fault condition.  Let's get to business, read Sensors and calculate how the machine should be behaving
  //
  //

        if (read_sensors()== false) return;                                                             // If there was an error in reading a critical sensor we have FAULTED, restart the loop.
                                                                                                        // Treat volts and amps sensed directly by the regulator as the ALTERNATOR values. .
        resolve_BAT_VoltAmpTemp();                                                                      // . .   but then look to see if they should also be considered the BATTERY values.
                                                                                                        //       (Will be yes, unless we receive battery volts / amps externally via an ASCII command or the CAN)
        calculate_RPMs();                                                                               // What speed is the engine spinning?
        calculate_ALT_targets();                                                                        // With all that known, update the target charging Volts, Amps, Watts, RPMs...  global variables.
 





  //
  //
  //-------     Now lets adjust the system.  But first, lets make sure we have not exceeded some threshold and hence faulted.
  //
  //

        if (check_for_faults() == true)  return;                                                        // Check for FAULT conditions 
                                                                                                        // If we found one, bail out now and enter holding pattern when we re-enter main loop.


        manage_ALT();                                                                                   // OK we are not faulted, we have made all our calculations. . . let's set the Alternator Field.
        manage_system_state();                                                                          // See if the overall System State needs changing.



  //
  //
  //-------     Tell the world what we are doing, and see if they want us to do something else!
  //
  //
        handle_feature_in();
        check_inbound();                                                                                // See if any communication is coming in via the Bluetooth (or DEBUG terminal), or Feature-in port.

        update_run_summary();                                                                           // Update the Run Summary variables
        send_outbound(false);                                                                           // And send the status via serial port - pacing the strings out.
        update_LED();                                                                                   // Set the blinking pattern and refresh it. (Will also blink the FEATURE_OUT if so configured via #defines
        update_feature_out();                                                                           // Handle any other FEATURE_OUT mode (as defined by #defines) other then Blinking.
 
     
        #ifdef SYSTEMCAN
          send_CAN();                                                                                   // Send out CAN status messages.
          check_CAN();                                                                                  // See if we have any incoming messages.
          decide_if_CAN_RBM();                                                                          // Decide who the Can RemoteBatteryMaster will be.  Including if it should be us.
          #endif 



        wdt_reset();                                                                                    // Pet the Dog so he does not bit us!

}                                                                                                       // All done this time around.  Back to Adriano and it will call this 
 
 
 
 
 
 
 

