//      AltReg_SERIAL.cpp
//
//      Copyright (c) 2016, 2017 by William A. Thomason.      http://arduinoalternatorregulator.blogspot.com/
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


#include "Device_Unique.h"
#include "AltReg_Serial.h"
#include "Alternator.h"
#include "Sensors.h"
#include "Flash.h"
 





char    ibBuf[INBOUND_BUFF_SIZE+1];                                     // Static buffer used to assemble inbound data in background. (+1 to allow for NULL terminator)
bool    ibBufFilling = false;                                           // Static flag used to manage filling of ibBuf.  If this is set = true, we have IDed the start of 
                                                                        // a new command 'string' are working to get the rest of it.
                                                                        // During this time period, all normal 'status updates' outputs from the regulator will be suspended.
                                                                        // This is to make a more direct linkage between a command that asks for a response and the actual response.
uint8_t  UMCounter         = 0;                                         // Update Monitor - Use this to moderate the number of times we send the non-critical information via Serial




//---- Local only helper function prototypes for Check_inbound() & Send_outbound()
bool getInt(char *buffer, int *dest, int LLim, int HLim);
bool getByte(char *buffer, uint8_t *dest, uint8_t LLim, uint8_t HLim);
bool getDuration(char *buffer, unsigned long *dest,  int HLim);
bool getFloat(char *buffer, float *dest, float LLim, float HLim);
bool getBool(char *buffer, bool *dest);
int frac2int (float frac, int limit);
void  append_string(char *dest, const char *src, int n);







//------------------------------------------------------------------------------------------------------
// Fill Inbound Buffer
//
//      This helper function will look to the Serial Port (TTL and Bluetooth) and assemble a complete command string 
//      starting after the $ and up to the terminating CR/LF, or '@' char (the latter used to work around bug in Arduino IDE)
//      It is non-blocking and will return FALSE if the string entire string is not ready, or TRUE
//      once the string has been assembled.   
//
//      If this is a CAN enabled target, the CAN terminal buffer will also be looked at. 
//
//      
// 
//------------------------------------------------------------------------------------------------------

bool fill_ib_buffer(){

 //  static unsigned int   ibIndex = 0;                                 // This pointer retains its value between function calls.
   static uint8_t   ibIndex = 0;                                        // This pointer retains its value between function calls.
   static unsigned long  ibBufFillStarted;                              // Will contain the time the last 'start' of a string happened
   char c;


   #ifdef SYSTEMCAN
     while((Serial.available() > 0) || (CAN_ASCII_recieved_count > 0)){
        if     (CAN_ASCII_recieved_count > 0)  c = CAN_ASCII_recieved_characters[--CAN_ASCII_recieved_count];
        else if(Serial.available()       > 0)  c = Serial.read();
   #else
      while(Serial.available() > 0){
        c = Serial.read();        
   #endif

        if(c == '$') {                                                  // "$" (start of command) character?
            ibIndex          = 0;                                       //   Yes!  Initialize ibIndex variable
            ibBufFilling     = true;                                    //   And start storing the remaining characters into the buffer. 
            ibBufFillStarted = millis();
            break;                                                      //   (The '$' character is not saved in the buffer...)
            }

        if(ibBufFilling == true){                                       // We are actively filling the inbound buffer, as we found the start '$' character.                     
            if(ibIndex  >= (INBOUND_BUFF_SIZE-1))  {                    // Are we about to overflow the buffer array?  (Need to leave room for null terminator)
                ibBufFilling  = false;                                  //  Yes, abort and start looking again for a shorter one - something must have gone wrong....
                break;
                }

            if((c == '\r') || (c == '@')){                              // Did we find the termination character? ('@' for BUG in Arduino IDE, will not send CR\LF, so 'fake it' using @)
                if (ibIndex < 3) {                                      // If terminator is found before the command string is AT LEAST 4 characters in length, something is wrong.  
                    ibBufFilling = false;                              // ALL valid commands are in form of xxx:, so must be at least 4 characters long.  So, abandon this and start over.
                    break;
                    }

                ibBufFilling = false;                                   // All done storing strings his time around - start looking again for a $
                return (true);                                          // Return that we have a valid command string!
                }
            else{
                ibBuf[ibIndex++] = c;                                   // Just a plane character, put it into the buffer
                ibBuf[ibIndex]   = 0;                                   // And add a null terminate just-in-case
                }
            }
         }



   if ((ibBufFilling == true) && (IB_BUFF_FILL_TIMEOUT != 0UL) &&       //  Have we timed-out waiting for a complete command string to be send to us?
       ((millis() - ibBufFillStarted) > IB_BUFF_FILL_TIMEOUT)) {
        ibBufFilling = false;                                           //  Yes, abort this and start looking for a new command initiator character ('$')
#ifdef SYSTEMCAN
        CAN_ASCII_source = 0;                                           // And also reset the CAN terminal ID to Idle mode - something has gotten out of sync and they need to start over..
        CAN_ASCII_recieved_count = 0;
#endif
       }

   return (false);                                                      // We have read all there is in the Serial queue, so go back and try to get the rest later.

}









//------------------------------------------------------------------------------------------------------
// Check Inbound
//
//      This function will check the serial terminal for incoming communications, either via the Bluetooth or
//      an attached Debug terminal.  It is used not only during debug, but also during operation to allow for
//      simple user configuration of the device (ala adjusting custom charging profiles - to be stored in FLASH)
//
// 
//------------------------------------------------------------------------------------------------------

void check_inbound()  {

    union{
        SCS SC;                                                         // Working buffer to hold the read values from the EEPROM
        CPS CP;                                                         // Note how I am using a 'union' to overlap working buffers and save of Stack RAM space...
          #ifdef SYSTEMCAN  
        CCS CC;
          #endif
        } buff;

    char     charBuffer[OUTBOUND_BUFF_SIZE+1];                          // Work buffer to assemble strings before sending to the serial port. (for commands that send back responses)
    char*    cp;
    int8_t   index;
    int      j;






                //----- Serial Port (TTL or Bluetooth) receiving code.  Read the 'packet', verify it, and make changes as needed.
                //      Store those changes in FLASH if needed.


   if(fill_ib_buffer() == true){                                        // Assemble a string, and see if it is "completed" yet..

        if (ibBuf[3] != ':')    return;                                 // ALL valid commands have a ":" in the 4th position.




        if (ibBuf[0] == 'R') {                                          // They are 'Requesting' something to be sent back to them...

          if ((ibBuf[1] == 'B') && (ibBuf[2] == 'T')) {                 // $RBT:  ReBooT?  Yea, they are 'requesting' a reboot!
               if (systemConfig.CONFIG_LOCKOUT != 0)   return;          // If system is locked-out, do not allow
               reboot();                                                //  Else just reboot (will not return from there...)
               }


          charBuffer[0] = NULL;                                         // Looks like they really might want some status sent back.  
                                                                        // Prime buffer ass empty and see if it gets filled.

          if ((ibBuf[1] == 'C') && (ibBuf[2] == 'P')) {                 //   They want a copy of one of the Charge Profile Entries

            index = ibBuf[4] - '1';                                     //   Which one?  There SHOULD be a number following, go get it.
                                                                        //     adjusting for "0-origin" of Arrays.
            if (index == -1)       
                index = cpIndex;                                        //   If user sent '0' (i will contain -1), this means they want the 'current' one.

            if ((index >= MAX_CPES) || (index < 0))
                return;                                                 //   And make sure it is in bounds of the array (This will also trap-out any non-numeric character following the ':')
                


            if (read_CPS_EEPROM(index, &buff.CP) != true)               //   See if there is a valid user modified CPE in EEPROM we should be using.
                transfer_default_CPS(index, &buff.CP);                  //   No, so get the correct entry from the values in the FLASH (PROGMEM) store.

            prep_CPE(charBuffer, &buff.CP, index);                      //   And Finally,  assemble the string to send out requested information. 
            }


          if ((ibBuf[1] == 'S') && (ibBuf[2] == 'S'))                   //   They want a copy of the System Status
            prep_SST(charBuffer); 
          
            
          if ((ibBuf[1] == 'S') && (ibBuf[2] == 'C'))                   //   They want a copy of the System Config
            prep_SCV(charBuffer); 
          

          if ((ibBuf[1] == 'C') && (ibBuf[2] == 'S'))                   //   They want a copy of the CAN Status
            prep_SST(charBuffer);  
 

          if ((ibBuf[1] == 'N') && (ibBuf[2] == 'P'))                   //   They want a copy of the Name/Password (aka Bluetooth) Config
            prep_NPC(charBuffer); 


          #ifdef SYSTEMCAN
          if (CAN_ASCII_source != 0) {
             strcpy (CANCharBuffer, charBuffer);                        //   If a remote CAN node requested this string, send it to them as well.
             CANCharBuffIndex = 0;                                      // Set pointer to beginning of CAN string to send back.
             }
            #endif            
            
          Serial.write(charBuffer);                                     // Send out the 'Requested' string our via Serial port
          return;                                                       //   And we are all done with this command
        }


                //------  Something about changing a Charge Profile
                //        Note that I only allow changes to CPE #7 or 8 (index = 6 or 7), if you want to change any other ones you
                //        will need to modify the source code.  This is to help prevent accidents.

        if ((ibBuf[0] == 'C') && (ibBuf[1] == 'P')) {                   // Something to do with the Charge Profiles . . .

            index =  ibBuf[4] - '1';                                    //   Which one?  There SHOULD be a number following, go get it.
            if ((index < (MAX_CPES - CUSTOM_CPES)) ||                   //   Make sure it is within range (7 or 8), abort if not..
                (index >= MAX_CPES))                                    //   (Only the last two Charge Profiles are allowed to be modified)     
                return;
        
            if (read_CPS_EEPROM(index, &buff.CP) != true)               // Go get requested index entry, and check to see if it is valid.
                transfer_default_CPS(index, &buff.CP);                  //   Nope, not a good one in EEPROM, so fetch the default entry from FLASH


           if (systemConfig.CONFIG_LOCKOUT != 0)        return;         // If system is locked-out, do not allow any changes...



                                                                        //  Now, let's move on and see what they wish to change...

            switch (ibBuf[2]) {
                case 'A':                                                       //   Change  ACCEPT parameters in CPE user entry n 
                                                                                //   $CPA:n  <VBat Set Point>, <Exit Duration>, <Exit Amps>
                        if (!getFloat   ((ibBuf + 5),   &buff.CP.ACPT_BAT_V_SETPOINT,  0.0, 20.0)) return;      //  20 volts MAX  - If any problems, just abort this command.
                        if (!getDuration(NULL,          &buff.CP.EXIT_ACPT_DURATION,     (60*10))) return;      //  10 Hours MAX, converted into mS for use
                        if (!getInt     (NULL,          &buff.CP.EXIT_ACPT_AMPS ,        -1, 200)) return;      // 200 Amps MAX
                        if (!getInt     (NULL,          &j,                               0,   0)) return;      // Place holder for future dV/dt 
                        break;                                                                                  // Got it all, drop down and finish storing it into EEPROM




                case 'O':                                                       // Change OVERCHARGE  parameters in CPE user entry n 
                                                                                //   $CPO:n    <Exit Amps>, <Exit Duration>, <Exit  VBat>       

                        if (!getInt     ((ibBuf + 5),   &buff.CP.LIMIT_OC_AMPS,           0, 50)) return;       // 50 Amps MAX
                        if (!getDuration(NULL,          &buff.CP.EXIT_OC_DURATION,      (60*10))) return;       // 10 Hours MAX, converted into mS for use
                        if (!getFloat   (NULL,          &buff.CP.EXIT_OC_VOLTS,       0.0, 20.0)) return;       // 20 volts MAX
                        if (!getInt     (NULL,          &j,                               0,  0)) return;       // Place holder for future dV/dt 
                        break;                                                  




                case 'F':                                                       // Change FLOAT  parameters in CPE user entry n 
                                                                                //   $CPF:n    <VBat Set Point>, <Exit Duration>, <Revert Amps> , <Revert Volts>        
                                                                        
                        if (!getFloat   ((ibBuf + 5),   &buff.CP.FLOAT_BAT_V_SETPOINT, 0.0, 20.0)) return;      //  20 volts MAX
                        if (!getInt     ( NULL,         &buff.CP.LIMIT_FLOAT_AMPS,        -1, 50)) return;
                        if (!getDuration( NULL,         &buff.CP.EXIT_FLOAT_DURATION,   (60*500))) return;      // 500 Hours MAX, converted into mS for use
                        if (!getInt     ( NULL,         &buff.CP.FLOAT_TO_BULK_AMPS,     -300, 0)) return;      // 300 Amps MAX
                        if (!getInt     ( NULL,         &buff.CP.FLOAT_TO_BULK_AHS,      -250, 0)) return;
                        if (!getFloat   ( NULL,         &buff.CP.FLOAT_TO_BULK_VOLTS,  0.0, 20.0)) return;      //  20 volts MAX
                        break;                                                  





                case 'P':                                                       // Change POST-FLOAT parameters in CPE user entry n
                                                                                //   $CPP:n     <Exit Duration>, <Revert VBat>  

                        if (!getDuration((ibBuf + 5),   &buff.CP.EXIT_PF_DURATION,      (60*500))) return;      // 500 Hours MAX, converted into mS for use
                        if (!getFloat   ( NULL,         &buff.CP.PF_TO_BULK_VOLTS,     0.0, 20.0)) return;      //  20 volts MAX to trigger moving back to Bulk mode (via Ramp)
                        if (!getInt     ( NULL,         &buff.CP.PF_TO_BULK_AHS,         -250, 0)) return;
                        break;                                                  





                case 'E':                                                       // Change EQUALIZE  parameters in CPE user entry n 
                                                                                //   $CPE:n     <VBat Set Point>, < Max Amps >, <Exit Duration>, <Exit Amps>

                        if (!getFloat   ((ibBuf + 5),   &buff.CP.EQUAL_BAT_V_SETPOINT, 0.0, 25.0)) return;      //   25 volts MAX for Equalization.
                        if (!getInt     ( NULL,         &buff.CP.LIMIT_EQUAL_AMPS,         0, 50)) return;      //  50A MAX while equalizing...
                        if (!getDuration( NULL,         &buff.CP.EXIT_EQUAL_DURATION,        240)) return;      //  240 MINUTES MAX, converted into mS for use  
                        if (!getInt     ( NULL,         &buff.CP.EXIT_EQUAL_AMPS,          0, 50)) return;      //   50 Amps MAX for Equalization
                        break;






                case 'B':                                                       // Change BATTERY parameters in CPE user entry n 
                                                                                //   $CPB:n     <VBat Comp per 1f>, < Min Comp Temp >, <Max Charge Temp>

                        if (!getFloat((ibBuf + 5),      &buff.CP.BAT_TEMP_1F_COMP,     0.0, 0.1)) return;       //   0.1v / deg f MAX.  (Note, this is for a normalized 12v battery)
                        if (!getInt  ( NULL,            &buff.CP.MIN_TEMP_COMP_LIMIT,  -20, 100)) return;       //  -20f to 100f range should be good???
                        if (!getInt  ( NULL,            &buff.CP.BAT_MIN_CHARGE_TEMP,  -50,  20)) return;
                        if (!getInt  ( NULL,            &buff.CP.BAT_MAX_CHARGE_TEMP,   70, 200)) return;       //  Cap at 200 to protect NTC sensor? (Esp Epoxy filling??)
                        break;  



                        
                case 'R':                                                       // $CPR:n  RESTORES Charge Profile table entry 'n' to default 
                        write_CPS_EEPROM(index, NULL);                          // Erase selected saved systemConfig structure in the EEPROM (if present)
                        Serial.write("AOK;\r\n");                               // Let user know we understand.
                        return;                                                 // All done.
                        


                default:        return;
                }

                write_CPS_EEPROM(index, &buff.CP);                              // Save back the new entry
                Serial.write("AOK;\r\n");                                       // Let user know we understand.
                return;                                                         // And return to see if more commands are being sent.  User must issue $RBT command for changes
                                                                                //   to be loaded into regulators working memory.
             }






        if ((ibBuf[0] == 'S') && (ibBuf[1] == 'C')) {                           // Something to do with the System Configuration . . .
            if (systemConfig.CONFIG_LOCKOUT != 0)       return;                 // If system is locked-out, do not allow any changes...


            if (read_SCS_EEPROM(&buff.SC) != true)                              // Prime the work buffer with anything already in FLASH
                buff.SC = systemConfig;                                         //   (or the default values if FLASH is empty / invalid)

            switch (ibBuf[2]) {
                case 'A':                                                       // Changes ALTERNATOR parameters in System Configuration table
                                                                                // $SCA: <32v?>, < Alt Target Temp >, <Alt Derate (norm) >,
                                                                                //       <Alt Derate (small) >,<Alt Derate (half) >,<PBF>, 
                                                                                //       <Alt Amp Cap.>, <System Watt Cap. >, <Amp Shunt Ratio>

                        if (!getBool ((ibBuf + 4), &buff.SC.FAVOR_32V                     )) return;
                        if (!getByte (NULL, &buff.SC.ALT_TEMP_SETPOINT,            60, 240)) return;                                    
                        if (!getFloat(NULL, &buff.SC.ALT_AMP_DERATE_NORMAL,       0.1, 1.0)) return;
                        if (!getFloat(NULL, &buff.SC.ALT_AMP_DERATE_SMALL_MODE,   0.1, 1.0)) return;
                        if (!getFloat(NULL, &buff.SC.ALT_AMP_DERATE_HALF_POWER,   0.1, 1.0)) return;
                        if (!getInt  (NULL, &buff.SC.ALT_PULLBACK_FACTOR,        -1,    10)) return;
                        if (!getInt  (NULL, &buff.SC.ALT_AMPS_LIMIT,             -1,   500)) return;
                        if (!getInt  (NULL, &buff.SC.ALT_WATTS_LIMIT,            -1, 20000)) return;
                        if (!getInt  (NULL, &buff.SC.AMP_SHUNT_RATIO,           500, 20000)) return;
                        if (!getBool (NULL, &buff.SC.REVERSED_SHUNT                       )) return;
                        if (!getInt  (NULL, &buff.SC.ALT_IDLE_RPM,                0,  1500)) buff.SC.ALT_IDLE_RPM = 0;
                        break;                                                  

                        
                        

                case 'T':                                                             // Changes TACHOMETER parameters in System Configuration table
                                                                                      // $SCT: <Alt Polls>, < Eng/Alt drive ratio >, <Field Tach Min>

                        if (!getByte((ibBuf + 4), &buff.SC.ALTERNATOR_POLES ,        2,            25)) return;
                        if (!getFloat(NULL,       &buff.SC.ENGINE_ALT_DRIVE_RATIO, 0.5,          20.0)) return;
                        if (!getInt  (NULL,       &buff.SC.FIELD_TACH_PWM,          -1,            30)) return;
                        
                        if (buff.SC.FIELD_TACH_PWM > 0)
                                buff.SC.FIELD_TACH_PWM = min(((buff.SC.FIELD_TACH_PWM * FIELD_PWM_MAX)/100)   , MAX_TACH_PWM);
                                                                                      // Convert any entered % of max field into a raw PWM number.
                        break;  

        



                
                case 'N':                                                               // Changes 'Name' (and Password) parameters in System Configuration table.  Used by Bluetooth and CAN to ID this regulator 
                                                                                        //   $SCN: <Enable BT?>, <Name>, <Password>                     
                        
                        if (!getBool  ((ibBuf + 4),       &buff.SC.USE_BT)) return;     //   Enable the Bluetooth (via Software)?  1 = Yes, anything else = No.

                        cp = strtok(NULL, ",");                                         //   Get the regulators name / ID
                        if ((cp == NULL) || (strlen(cp)> MAX_NAME_LEN)) return;  
                        strcpy(buff.SC.REG_NAME, cp);

                        cp = strtok(NULL, ",");                                         //   Get the password
                        if ((cp == NULL) || (strlen(cp)> MAX_PIN_LEN)) return;   
                        strcpy(buff.SC.REG_PSWD, cp);


                        if ((measuredAltAmps < CLEAR_BT_LOCK_AMPS) && (readDipSwitch()== DIP_MASK)) {
                                systemConfig.BT_CONFIG_CHANGED = true;                  // Clear the lock-flag, but only if we are not 'charging' and the DIPs are all on...
                                buff.SC.BT_CONFIG_CHANGED      = true;                  // Note it also in the soon-to-be-written buffer as well, so that this will be preserved after rebooting.
                                }                                             

                        break;                                                  





                case 'O':                                                             // OVERRIDE DIP Switch settings for CP_Index and BC_Index.
                                                                                      // $SCO:  <CP_Index>, <BC_Index >, <SV_Override>, <Lockout>
 
                        if (!getByte    ((ibBuf + 4),   &buff.SC.CP_INDEX_OVERRIDE,   0,  8  )) return;
                        if (!getFloat   ( NULL,         &buff.SC.BC_MULT_OVERRIDE,    0, 10.0)) return;
                        if (!getFloat   ( NULL,         &buff.SC.SV_OVERRIDE,         0,  4.0)) return;
                        if (!getByte    ( NULL,         &buff.SC.CONFIG_LOCKOUT,      0,  2  )) return;
                

                        break;  

        


                case 'R':                                                       // RESTORES System Configuration table to default
                        write_SCS_EEPROM(NULL);                                 // Erase any saved systemConfig structure in the EEPROM
                        Serial.write("AOK;\r\n");                               // Let user know we understand.
                        return;                                                 // All done.

                default:        return;
                }


            write_SCS_EEPROM(&buff.SC);                                         // Save back the new Charge profile 
            Serial.write("AOK;\r\n");                                           // Let user know we understand.
            return;                                                             // And return to see if more commands are being sent.  User must issue $RBT command for changes
                                                                                //   to be loaded into regulators working memory.
            }
        





        #ifdef SYSTEMCAN 
        if ((ibBuf[0] == 'C') && (ibBuf[1] == 'C')) {                           // Something to do with the CAN Configuration . . .
            if (systemConfig.CONFIG_LOCKOUT != 0)       return;                 // If system is locked-out, do not allow any changes...


            if (read_CCS_EEPROM(&buff.CC) != true)                              // Prime the work buffer with anything already in FLASH
                 buff.CC = canConfig;                                           //   (or the default values if FLASH is empty / invalid)


            switch (ibBuf[2]) {   


                case 'R':                                                       // RESTORES CAN Configuration table to default
                        write_CCS_EEPROM(NULL);                                 // Erase any saved CANConfig structure in the EEPROM
                        Serial.write("AOK;\r\n");                               // Let user know we understand.
                        return;                                                 // All done.
    


               case 'N':                                                       // Can CoNfiguration       
                                                                               //$CCN:    <Battery Instance Override>, <Device Instance >, <Device Priority>,
                                                                               //         <AllowRMB?>, <ShuntAtBat?>,  <Enable-OSE?>, <Enable-NMEA2000?>, <Enable_NMEA2000_RAT?>
                        if (!getByte    ((ibBuf + 4),   &buff.CC.BI_OVERRIDE,         0,  100  )) return;
                        if (!getByte    ( NULL,         &buff.CC.DEVICE_INSTANCE,     1,   13  )) return;
                        if (!getByte    ( NULL,         &buff.CC.DEVICE_PRIORITY,     1,  250  )) return;
                        if (!getBool    ( NULL,         &buff.CC.CONSIDER_MASTER               )) return;
                        if (!getBool    ( NULL,         &buff.CC.SHUNT_AT_BAT                  )) return;
                        if (!getBool    ( NULL,         &buff.CC.ENABLE_OSE                    )) return;
                        if (!getBool    ( NULL,         &buff.CC.ENABLE_NMEA2000               )) return;
                        if (!getBool    ( NULL,         &buff.CC.ENABLE_NMEA2000_RAT           )) return;
                        
                        break;
                        
                        
                default:        return;
                }


            write_CCS_EEPROM(&buff.CC);                                         // Save back the new CAN Configuration structure
            Serial.write("AOK;\r\n");                                           // Let user know we understand.
            return;                                                             // And return to see if more commands are being sent.  User must issue $RBT command for changes
                                                                                //   to be loaded into regulators working memory.
            }
            #endif  // SYSTEMCAN







                        //---   Finally, check for some of the other commands, ones that do not fit a good pattern to allow a CASE statement to be used.
                        //
                        //


        if ((ibBuf[0] == 'E') && (ibBuf[1] == 'B') && (ibBuf[2] == 'A')) {      // $EBA:  External Battery Amps?
                if (systemConfig.CONFIG_LOCKOUT > 1)    return;                 // If system is locked-out, do not allow Override!

                float proposedBatAmps;
                
                if (getFloat ((ibBuf + 4), &proposedBatAmps, -500.0,  500.0)) { // Get the externally supplied Battery Amps.
                        EORLastReceived = millis();                             // Looks like we got a good one, note the time received.
                        usingEXTAmps    = true;
                        measuredBatAmps = proposedBatAmps;                      // Only change if a valid number is sent to us, else just leave the existing value alone.
                        Serial.write("AOK;\r\n");                               // Let user know we understand.
                        }                
                return;
                }



        if ((ibBuf[0] == 'E') && (ibBuf[1] == 'D') && (ibBuf[2] == 'B')) {      // $EDB:  Enable DeBug ASCII string??
                Serial.write("AOK;\r\n");                                       // Let user know we understand.
  
                SDMCounter      = SDM_SENSITIVITY;                              // Yes, set up the counters and turn on the switch!
                sendDebugString = true;
                }



        if ((ibBuf[0] == 'M') && (ibBuf[1] == 'S') && (ibBuf[2] == 'R')) {      // $MSR:  Master System Restore?
                if (systemConfig.CONFIG_LOCKOUT != 0)   return;                 // If system is locked-out, do not allow restore.

                Serial.write("AOK;\r\n");                                       // Let user know we understand.
  
                restore_all();                                                  // Yes.  Erase all the EEPROM and reset. (we will not come back from here)
                }                                                               //  btw: doing individual character compares as done here uses about 25 bytes less
                                                                                // code then using  -- strstr(ibBuf,"MSR") != NULL -- 



        if ((ibBuf[0] == 'F') && (ibBuf[1] == 'R') && (ibBuf[2] == 'M')) {      // $FRM:  Force Regulator Mode ASCII string??

                switch (ibBuf[4]) {                                             // SHOULD be an ASCII character following the ':', if note, will hold NULL
                        case 'B':       set_ALT_mode(bulk_charge);              //  B   = Force into BULK mode.
                                        break;

                        case 'A':       set_ALT_mode(acceptance_charge);        //  A   = Force into ACCEPTANCE mode.
                                        break;

                        case 'O':       set_ALT_mode(overcharge_charge);        //  O   = Force into OVER-CHARGE mode.
                                        break;

                        case 'F':       set_ALT_mode(float_charge);             //  F   = Force into FLOAT mode.
                                        break;

                        case 'P':       set_ALT_mode(post_float);               //  P   = Force into POST-FLOAT mode.
                                        break;

                        case 'E':       set_ALT_mode(equalize);                 //  E   = Force into EQUALIZE mode.
                                        break;

                        default:        return;                                 // Not something we understand..
                        }


                Serial.write("AOK;\r\n");                                       // One got through!  Let user know we understand.
                
                return;  
                
                }

                

        }

}








//---- Helper functions for Check_inbound()
bool getInt(char *buffer, int *dest, int LLim, int HLim) {
    char* cp;
        cp = strtok(buffer, ",");
        if (cp == NULL)  return(false); 
        *dest = constrain(atoi(cp), LLim, HLim);        
        return(true);
   }

bool getByte(char *buffer, uint8_t *dest, uint8_t LLim, uint8_t HLim) {
    char* cp;
        cp = strtok(buffer, ",");
        if (cp == NULL)  return(false); 
        *dest = (uint8_t) constrain(atoi(cp), LLim, HLim);        
        return(true);
   }

bool getDuration(char *buffer, unsigned long *dest,  int HLim) {
     char* cp;
        cp = strtok(buffer, ",");
        if (cp == NULL)  return(false); 
        *dest = (unsigned long) (constrain(atoi(cp), 0, HLim)) * 60000UL;               // Noticed, EVERY duration has this 60000UL multiplier and 0 LLim, so no need to pass as separate parameter...
        return(true);
   }


bool getFloat(char *buffer, float *dest, float LLim, float HLim) {
    char* cp;
        cp = strtok(buffer, ",");
        if (cp == NULL)  return(false); 
        *dest = constrain(atof(cp), LLim, HLim);        
        return(true);
   }


bool getBool(char *buffer, bool *dest) {
    char* cp;
        cp = strtok(buffer, ",");
        if (cp == NULL)  return(false); 
        *dest = (atoi(cp) == 1);        
        return(true);
   }   





//------------------------------------------------------------------------------------------------------
// Send Outbound
//
//      This function will send to the Serial Terminal the current system status.  It is used to send
//      information primarily via the Bluetooth to an external HUI program.  
//
//      In FAULTED conditions no check will be made in timing or counters to pace the rate of data
//      being sent, data will simple be sent.
//
//      
// 
//------------------------------------------------------------------------------------------------------


void  send_outbound() {

char    charBuffer[OUTBOUND_BUFF_SIZE+1];                                                       // Large working buffer to assemble strings before sending to the serial port.
uint8_t i, j;
unsigned long static lastStatusSent = 0U;                                                        // When was the Status last sent?



 if (alternatorState != FAULTED)  {                                                             // Only do these pacing / restart checks if the system is not Faulted.
                                                                                                // If we are in a FAULT state, then send out the data no matter what...
    if ((millis() - lastStatusSent) < UPDATE_STATUS_RATE)     return;                           // Time to send Update packet?  No, not just yet.
    if( ibBufFilling == true)                                 return;                           // Suspend the sending of status updates while a new command is being assembled.
    }                                                                                           // This way there is no confusion over data received from the regulator as to if it
                                                                                                // is a response to a request-for command, or just the 'normal' status data being pushed out.



                                //----- Send status strings via the serial port. 
                                //      Make sure to 'space them out', sending every once and a while - not all in one block...

  j = UMCounter % UPDATE_MAJOR_SENSITIVITY;
   
  for (i=0; i <= 5; i++) {                                                                  // Loop through all strings, seeing which ones we should send this time.
      charBuffer[0] = '\0';  
      
      if (i == 0)                                                                                  prep_AST(charBuffer);                          // Alternator STatus goes each cycle through                                              
      if((i == 1)  &&  ((j == ((1*UPDATE_MAJOR_SENSITIVITY/5)-1))||(alternatorState == FAULTED)))  prep_SST(charBuffer);                          // Send the System Status
      if((i == 2)  &&  ((j == ((2*UPDATE_MAJOR_SENSITIVITY/5)-1))||(alternatorState == FAULTED)))  prep_CST(charBuffer);                          // Send the CAN Control Variables  (n/a on 1st gen regulator)
      if((i == 3)  &&  ((j == ((3*UPDATE_MAJOR_SENSITIVITY/5)-1))||(alternatorState == FAULTED)))  prep_CPE(charBuffer, &workingParms, cpIndex);  // Send the currently active charge profile
      if((i == 4)  &&  ((j == ((4*UPDATE_MAJOR_SENSITIVITY/5)-1))||(alternatorState == FAULTED)))  prep_SCV(charBuffer);                          // Send the System Control Variables (using this buffer as a work space)
      if((i == 5)  &&  ((j == ((5*UPDATE_MAJOR_SENSITIVITY/5)-1)) |(alternatorState == FAULTED)))  prep_NPC(charBuffer);                          // Send the Name/Password (was Bluetooth) Config. 
      
      if (charBuffer[0] != '\0')                                                           // Was a string prepared for us to send?
          Serial.write(charBuffer);                      
   }

  lastStatusSent = millis();
  UMCounter++;
}








//------------------------------------------------------------------------------------------------------
// Preg Outbound strings 
//
//      These functions will assemble the outbound strings into the passed buffer.
//      Note that the passed buffer MUST BE AT LEAST 'OUTBOUND_BUFF_SIZE' in size.
//      
// 
//------------------------------------------------------------------------------------------------------



void prep_AST(char *buffer) {                                                           // Assembled the Alternator Status ASCII string.  Pass in working buffer of BUFF_SIZE
                                                                                        //  Alternator STatus - sent very often
    snprintf_P(buffer,OUTBOUND_BUFF_SIZE, PSTR("AST;,%d.%02d, ,%d.%02d,%d.%01d,%d.%01d,%d, ,%d.%02d,%d,%d,%d, ,%d,%d, ,%d, ,%d.%02d,%d,%d,%d\r\n"),
        (int)  (generatorLrRunTime / (3600UL * 1000UL)),                                // Runtime Hours
        (int) ((generatorLrRunTime / (3600UL * 10UL  )) % 100),                         // Runtime 1/100th

        (int)    measuredBatVolts,                                                      // Battery Voltage to 1/100th of a volt
        frac2int(measuredBatVolts,100),
        (int)    measuredAltAmps,                                                       // Alternator Amps to 1/10th of an amp
        frac2int(measuredAltAmps,10), 
        (int)    measuredBatAmps,                                                       // User supplied Offset Amps to 1/10th of an amp
        frac2int(measuredBatAmps,10),  
        measuredAltWatts, 

        
        (int)    targetBatVolts,
        frac2int(targetBatVolts,100),
        (int)    targetAltAmps,
        targetAltWatts,
        alternatorState,

        measuredBatTemp,                                                                // In deg F
        max(measuredAltTemp,measuredAlt2Temp),


        measuredRPMs,
        

        (int)     measuredAltVolts,                                                     // Alternator Voltage to 1/100th of a volt
        frac2int (measuredAltVolts,100),
        measuredFETTemp,
        measuredFieldAmps,
        ((100*fieldPWMvalue) / FIELD_PWM_MAX) );
        }




void prep_CPE(char *buffer, CPS  *cpsPtr, int index) {                     
   snprintf_P(buffer,OUTBOUND_BUFF_SIZE, PSTR("CPE;,%d,%d.%02d,%d,%d,%d, ,%d,%d,%d.%02d,%d, ,%d.%02d,%d,%d,%d,%d,%d.%02d, ,%d,%d.%02d,%d, ,%d.%02d,%d,%d,%d, ,%d.%03d,%d,%d,%d\r\n"),
                        (index + 1),                                                                        // Convert from "0-origin" of array indexes for display
        (int)            cpsPtr->ACPT_BAT_V_SETPOINT,
         frac2int       (cpsPtr->ACPT_BAT_V_SETPOINT,100),
        (unsigned int)  (cpsPtr->EXIT_ACPT_DURATION/60000UL),                                               // Show time running in Minutes, as opposed to mS
                         cpsPtr->EXIT_ACPT_AMPS,
        (int)             0,                                                                                // Place holder for future dV/dT exit parameter, hard coded = 0 for now.

                         cpsPtr->LIMIT_OC_AMPS,
        (unsigned int)  (cpsPtr->EXIT_OC_DURATION/60000UL),
        (int)            cpsPtr->EXIT_OC_VOLTS,
        frac2int        (cpsPtr->EXIT_OC_VOLTS,100),
        (int)            0,                                                                                 // Place holder for future dV/dT exit parameter, hard coded = 0 for now.


        (int)            cpsPtr->FLOAT_BAT_V_SETPOINT,
        frac2int        (cpsPtr->FLOAT_BAT_V_SETPOINT, 100),
                         cpsPtr->LIMIT_FLOAT_AMPS,
        (unsigned int)  (cpsPtr->EXIT_FLOAT_DURATION/60000UL),
                         cpsPtr->FLOAT_TO_BULK_AMPS,
                         cpsPtr->FLOAT_TO_BULK_AHS,
        (int)            cpsPtr->FLOAT_TO_BULK_VOLTS,
        frac2int        (cpsPtr->FLOAT_TO_BULK_VOLTS, 100),
        
        (unsigned int)   (cpsPtr->EXIT_PF_DURATION/60000UL),                                                //  Show in Minutes, as opposed to mS,
        (int)             cpsPtr->PF_TO_BULK_VOLTS,
        frac2int         (cpsPtr->PF_TO_BULK_VOLTS, 100),
                          cpsPtr->PF_TO_BULK_AHS,

        (int)             cpsPtr->EQUAL_BAT_V_SETPOINT,
        frac2int         (cpsPtr->EQUAL_BAT_V_SETPOINT, 100),
                          cpsPtr->LIMIT_EQUAL_AMPS,
        (unsigned int)   (cpsPtr->EXIT_EQUAL_DURATION/60000UL),                                             //  Show in Minutes, as opposed to mS,
                          cpsPtr->EXIT_EQUAL_AMPS ,

        (int)             cpsPtr->BAT_TEMP_1F_COMP,
        frac2int         (cpsPtr->BAT_TEMP_1F_COMP,1000),
                          cpsPtr->MIN_TEMP_COMP_LIMIT,
                          cpsPtr->BAT_MIN_CHARGE_TEMP,
                          cpsPtr->BAT_MAX_CHARGE_TEMP);
        }



void prep_SCV(char *buffer) {                                                                               // Prep the System Control Variables.  Pass in working buffer of OUTBOUND_BUFF_SIZE
        snprintf_P(buffer, OUTBOUND_BUFF_SIZE, PSTR("SCV;,%1u,%1u,%1u,%d.%02d,%d.%02d,%1u, ,%d,%d.%02d,%d.%02d,%d.%02d,%d, ,%d,%d, ,%d,%d.%02d,%d, ,%d,%d\r\n"),
                  systemConfig.CONFIG_LOCKOUT,
                  systemConfig.FAVOR_32V,
                  systemConfig.REVERSED_SHUNT,
        (int)     systemConfig.SV_OVERRIDE,
        frac2int (systemConfig.SV_OVERRIDE, 100),
        (int)     systemConfig.BC_MULT_OVERRIDE,
        frac2int (systemConfig.BC_MULT_OVERRIDE, 100),
                  systemConfig.CP_INDEX_OVERRIDE,


                  systemConfig.ALT_TEMP_SETPOINT,
        (int)     systemConfig.ALT_AMP_DERATE_NORMAL,
        frac2int (systemConfig.ALT_AMP_DERATE_NORMAL, 100),
        (int)     systemConfig.ALT_AMP_DERATE_SMALL_MODE,
        frac2int (systemConfig.ALT_AMP_DERATE_SMALL_MODE, 100),
        (int)     systemConfig.ALT_AMP_DERATE_HALF_POWER,
        frac2int (systemConfig.ALT_AMP_DERATE_HALF_POWER, 100),
                  systemConfig.ALT_PULLBACK_FACTOR,

                  systemConfig.ALT_AMPS_LIMIT,
                  systemConfig.ALT_WATTS_LIMIT,

                  systemConfig.ALTERNATOR_POLES,
        (int)     systemConfig.ENGINE_ALT_DRIVE_RATIO,
        frac2int (systemConfig.ENGINE_ALT_DRIVE_RATIO , 1000),
                  systemConfig.AMP_SHUNT_RATIO,
                  
                  systemConfig.ALT_IDLE_RPM,
                  ((systemConfig.FIELD_TACH_PWM>0) ? ((100*systemConfig.FIELD_TACH_PWM)/FIELD_PWM_MAX) : systemConfig.FIELD_TACH_PWM));
        }




void prep_NPC(char *buffer) {                                                                               // Prep the System Control Variable string.  Pass in working buffer of OUTBOUND_BUFF_SIZE
        
        snprintf_P(buffer, OUTBOUND_BUFF_SIZE-3, PSTR("NPC;,%1u,%s,%s\r\n"),                                // Prime things with the initial ASCII string..
        systemConfig.USE_BT,
        systemConfig.REG_NAME,
        systemConfig.REG_PSWD);

        }






void prep_SST(char *buffer) { 
        snprintf_P(buffer,OUTBOUND_BUFF_SIZE-3, PSTR("SST;,%s, ,%1u,%1u, ,%d,%d.%02d,%d.%02d, ,%d,%d, ,%d,%d\r\n"), //  System Status 
                firmwareVersion,

                smallAltMode,
                tachMode,

                (int)   (cpIndex + 1),
                (int)    systemAmpMult,
                frac2int(systemAmpMult,100),

                (int)    systemVoltMult,
                frac2int(systemVoltMult,100),

                altCapAmps,                                             
                altCapRPMs,

                (int)((accumulatedLrAH / 3600UL)  * (ACCUMULATE_SAMPLING_RATE / 1000UL)),               // Convert into actual AHs
                (int)((accumulatedLrWH / 3600UL)  * (ACCUMULATE_SAMPLING_RATE / 1000UL))                // Convert into actual WHs                                              
                );

        }



void prep_CST(char *buffer) { 
     
        #ifdef SYSTEMCAN                                                                                // Prep  the CAN Control Variable string. (Only on CAN enabled regulator)
        snprintf_P(buffer, OUTBOUND_BUFF_SIZE, PSTR("CST;,%d,%d,%d,%d,  ,%1u,%1u,  ,%1u,%1u,%1u, ,%d,%1u,%1u,  ,%d\r\n"),
                  batteryInstance,
                  canConfig.BI_OVERRIDE,
                  canConfig.DEVICE_INSTANCE,
                  canConfig.DEVICE_PRIORITY,

                  canConfig.ENABLE_NMEA2000,
                  canConfig.ENABLE_OSE,
                  
                  canConfig.CONSIDER_MASTER,
                  CAN_weAreRBM,
                  canConfig.SHUNT_AT_BAT,
    
                  CAN_RBM_sourceID,
                  ignoringRBM,
                  canConfig.ENABLE_NMEA2000_RAT,
                                    
                  fetch_CAN_localID()); 
        #endif
        }




//----  Helper fucntion for Send_outbound();
int frac2int (float frac, int limit) {                                                  // Returned rounded fractional portion to 'limit' digits
    float f;
    int   i;
    
    f = abs(frac) * limit;                                                              // Remove any sign, and scale it up.
//  f = f + 0.5;                                                                        // Round it -- w/o the ability to report out a carry, this caused 13.997v to show as 13.00 after rounding!
    i = (int) f % limit;                                                                // Trim off the digits above the '.' point (those above the scaling factor), 
                                                                                        // and make what's left an Int.
    return (i);
    }

    
    
    

void  append_string(char *dest, const char *src, int n) {
    int i; 
    int offset;
    int limit;
    
    offset = strlen(dest);                                                        // Where should we start appending?
    limit  = n - offset;                                                          // Figure out how much room is left in the destination string
        
    for(i = 0; (i < limit) && *(src + i); i++) {                                  // As long as we have not exceeded the remaining room (leaving room for a final \o), and have not reached the end of the 'src' string
        *(dest + i + offset) = *(src + i);                                        // copy over the character from the Source to the Destination.
        }
        
    *(dest + i + offset) = '\0';                                                  // NULL terminator goes at the end.
  }













