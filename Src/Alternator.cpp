//      Alternator.C
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


#include "Device_Unique.h"
#include "Alternator.h"
#include "AltReg_Serial.h"
#include "Sensors.h"








//---  The core function of the regulator is to manage the alternators field in response to several criteria, e.g.:  battery voltage, system load, temperature, engine RPMs., etc..
//      (Simple regulators typically only look at voltage).  For this there are three key variables:  The current field drive value, as well as an upper limit, or cap which can be set
//      to limit alternator output for some reason (e.g, we are at idle RPMs and need to lessen the load to the engine).  There is also a 'floor' value typically used to assure the field
//      is being driven hard anough to allow for stator pulses to be produced and recognized by the stator IRQ ckt.  Here are the critical variables to accomplish all this:
int     fieldPWMvalue     = FIELD_PWM_MIN;                              // How hard are we driving the Alternator Field?  Start not driving it.
int     fieldPWMLimit     = FIELD_PWM_MAX;                              // The upper limit of PWM we should use, after adjusting for reduced power modes..  Note that during auto-sizing, this will be
                                                                        // reset to FIELD_PWM_MAX, as opposed to constrained values from user selected reduced power modes.
int     thresholdPWMvalue = FIELD_PWM_MIN;                              // This contains the PWM level needed to provide for a Stator IRQ sufficient to produce a stable RPM measurement.
                                                                        // It may be pre-defined by the user (systemConfig.FIELD_TACH_PWM).  Or if the user has asked for auto-determination - 
                                                                        // while operating (typically during RAMP phase) once there is sufficient stability in the Stator IRQ signal to allow 
                                                                        // calculation the then current PWM value is noted. If TACH mode is enabled, this becomes the floor for the lower limit of 
                                                                        // PWM drive.  Hopefully keeping the tach alive at all times.
                                                                        // Will contain -1 if we have not seen a stator IRQ signal (and the user wants to look for one.)




 //---  Timing variables and smoothed voltages and current values - used to pace and control state changes
unsigned long lastPWMChanged;                                           // Used to give a settling time when the PWM is altered.
unsigned long altModeChanged;                                           // When was the charging mode was last changed.

unsigned long adptExitAcceptDuration  = 4*3600000UL;                    // If we will be doing 'adaptive time' based exiting of Acceptance mode, this holds the amount of time we should remain in
                                                                        // acceptance mode.  It is calculated in manage_alt() and is set at ADPT_ACPT_TIME_FACTOR * the duration we were in BULK phase.
                                                                        // Initializing it for 4 hrs in case we have some condition where we never were in Bulk mode and went directly into Accept
                                                                        //  (Example via an ASCII command, this is a safety fall-back)

float         persistentBatAmps  =   0;                                 // Used to smooth alternator mode changes.  Usually the same as measuredAltAmps, unless user has overridden this via $EOR command.
float         persistentBatVolts = 0.0;                                 // Used in post-float to decide if we need to restart a charging cycle.
unsigned long enteredFloatLrAH   =   0;                                 // Snapshot of value in accumulatedLrAH when we 1st entered Float/Post Float mode.  Used to cacl AHs withdrawn from the battery to check
                                                                        // against AHs exits in the CPEs.







//---   Tachometer veriables.  Driven from the Stator IRQ ckt.
unsigned long priorInterupt_uS = 0L;                                    // Used for time calculations in interrupt handler.  Note this contains Microseconds for better resolution!
                                                                        // Initialized = 0 to indicate no sampling has happened yet.
volatile unsigned int    interuptCounter  = 0;                          // Used for smoothing function in interrupt handler when calculating RPMs.
volatile bool            statorIRQflag    = false;                      // Used by read_sensors() and IQR_vector() to lock-step INA226 sampling with stator pulses Stator
int                      measuredRPMs     = 0;                          // Current measured RPM of Engine (via the alternator, after converting for belt diameter).  Contains 0 = if the RPMs cannot be measured.
                                                                        //    Note these are incremented in the IRQ handler, hence Volatile directive.
bool                     tachMode         = false;                      // Has the user indicated (via the DIP Switch) that they are driving a Tachometer via the Alternator, and hence
                                                                        // we should always give some small level of Field PWM??





//---   Targets which are 'regulated' towards:
int             altCapAmps        = 0;                                  // This will contain the capacity of the Alternator, either determined by auto-sizing or as declared to use by the user.
int             altCapRPMs        = 0;                                  // If we did an auto-sizing cycle, this will be the high-water mark RPMs  (= 0 indicates we have not yet measured the capacity)
                                                                        // Default deployment is assumed to be Battery Centric; hence these two limits are disabled.  Note that reduced power modes will
                                                                        // directly adjust the PWM duty-cycle, to approx things like Half-Power mode, etc.
int             targetAltWatts  = 0;                                    // Where do we want to be.  Will me adjusted for charger mode and battery temp.
float           targetBatVolts  = 0;
float           targetAltAmps   = 0;





//---   States and Modes -  defines or modifies various functions behaviors
tModes          alternatorState   = unknown;                            // What is the current state of the alternator regulators?  (Ramping, bulk, float, faulted, etc...)
bool            smallAltMode = false;                                   // Has the user indicated (vita the DIP Switch) they want us to run in the Small Alternator mode?  If so, we will assume the
                                                                        //   alternator has limited heat dispersion capability and will as such reduce its output to a max of ALT_AMP_DERATE_SMALL_MODE
                                                                        //   of its capability (as auto-measured, or defined by the user).
bool            usingEXTAmps      = false;                              // This flag is used to note that we received Battery Amps for the $EBA: ASCII command or a remote CAN master to
unsigned long   EORLastReceived;                                        //   And when it was received.  (Used to age externally supplied information)






                                //---   Default System Config variable is defined here.
    
SCS systemConfig = {
        false,                          // .FAVOR_32V                   --> Do NOT favor 32v systems over 24/48v during auto-detection.
        false,                          // .REVERSED_SHUNT              --> Assume shunt is not reversed.
        200,                            // .ALT_TEMP_SETPOINT           --> Default Alternator temp - 200f
        1.00,                           // .ALT_AMP_DERATE_NORMAL       --> Normal cap Alternator at 100% of demonstrated max Amp capability, 
        0.75,                           // .ALT_AMP_DERATE_SMALL_MODE   --> Unless user has selected Small Alt Mode via DIP switch, then do 75% of its capability
        0.50,                           // .ALT_AMP_DERATE_HALF_POWER   --> User has shorted out the Alternator Temp NTC probe, indicating they want to do 1/2 power mode.
        -1,                             // .ALT_PULLBACK_FACTOR         --> Used to pull-back Field Drive as we move towards Idle.  
        0,                              // .ALT_IDLE_RPM                --> Used to pull-back Field Drive as we move towards idle.  Set = 0 causes RPMs to be determined automatically during operation.
        0,                              // .ALT_AMPS_LIMIT              --> The regulator may OPTIONALLY be configured to limit the size of the alternator output 
                                        //                                  Set = 0 to disable Amps capping.  Set = -1 to auto-size Alternator during Ramp. (required Shunt on Alt, not Bat)
        0,                              // .ALT_WATTS_LIMIT             --> The regulator may OPTIONALLY be configured to limit the load placed on the engine via the Alternator.
                                        //                                  Set = 0 to disable, -1 to use auto-calc based on Alternator size. (Required Shunt on Alt, not Bat)
        12,                             // .ALTERNATOR_POLES            --> # of polls on alternator (Leece Neville 4800/4900 series are 12 pole alts)
       ((6.7 / 2.8) * 1.00),            // .ENGINE_ALT_DRIVE_RATIO      --> Engine pulley diameter / alternator diameter &  fine tuning calibration ratio       
 (int) ((500/0.050)  * 1.00),           // .AMP_SHUNT_RATIO             --> Spec of amp shunt,  500A / 50mV shunt (Link10 default) and % calibrating error
                                        //                                   CAUTION:  Do NOT exceed 80mV on the AMP Shunt input
        -1,                             // .FIELD_TACH_PWM              --> If user has selected Tach Mode, use this for MIN Field PWM.  
                                        //                                    Set = -1 to 'auto determine' the this value during RAMP phase
                                        //                                    Set =  0 to in effect 'disable' tach mode, independent of the DIP switch.

        true,                           // .USE_BT                      --> Should we try to use the Bluetooth?
        "ALTREG",                       // .REG_NAME                    --> Name of Regulator (Used for NMEA2000 and Bluetooth) module.  MAX 18 CHARS LONG!  (see MAX_NAME_LEN)
        "1234",                         // .REG_PSWD                    --> Password to use for Bluetooth module.  MAX 18 CHARS LONG!  (see MAX_PIN_LEN)         
        DEFAULT_BT_CONFIG_CHANGED,      // .BT_CONFIG_CHANGED           --> BT name and password are still the default.  Updates to configuration data is prevented until the name & password is changed.

         0,                             // .CP_INDEX_OVERRIDE           --> Use the DIP switch selected indexes
         0.0,                           // .BC_MULT_OVERRIDE            --> Use the DIP switch selected multiplier
         0.0,                           // .SV_OVERRIDE                 --> Enable Auto System voltage detection
         0 };                           // .CONFIG_LOCKOUT;             --> No lockouts at this time.



CPS             workingParms;                                           // Charge Parameters we are currently working with, appropriate entry is copied from EEPROM or FLASH during startup();
uint8_t         cpIndex       = 0;                                      // Which entry in the chargeParms structure should we be using for battery setpoints?  (Default = 1st one)


float           systemAmpMult = 1;                                      // Multiplier used to adjust Amp targets for the charging setpoints?
                                                                        //      i.e.:   systemBatMult = 1 for <500Ah          battery
                                                                        //      i.e.:   systemBatMult = 2 for  500   - 1000Ah battery
                                                                        //      i.e.:   systemBatMult = 3 for 1000Ah - 1500Ah battery
                                                                        //      i.e.:   systemBatMult = 4 for        > 1500Ah battery
                                                                        //
                                                                        //  using Floating point allows fine tuning of battery size via ASCII commands.
                                                                        

float           systemVoltMult = 1;                                     // The actually system battery is this mutable of a "12v battery".   It is set during startup by reading the
                                                                        //  at-rest value of the system battery via a raw A/D on the Atmel CPU and calculating the multiplier value. And is used                                                                                                //  to adjust the target battery voltages contained within the chargeParm table.
                                                                        //      i.e.:   systemBatMult = 1 for 12v battery
                                                                        //      i.e.:   systemBatMult = 2 for 24v battery
                                                                        //      i.e.:   systemBatMult = 3 for 36v battery
                                                                        //      i.e.:   systemBatMult = 4 for 48v battery
                                                                        //
                                                                        //  Using a Float allows better support for other voltages, ala 2.666 --> 32v batteries. 3.5 --> 42v ones.
 










// Internal function prototypes.
void set_VAWL(float passedV);









//------------------------------------------------------------------------------------------------------
// Initialize Alternator
//      This function is called once during Startup to establish the ports and global variables needed
//      to manage the Alternators PWM field.
//
//------------------------------------------------------------------------------------------------------


bool initialize_alternator(void){

    //----  Setup the I/O ports
    pinMode(FIELD_PWM_PORT,       OUTPUT);                                                  // Set up the FET Driver pins
    set_ALT_PWM(0);                                                                         // When just starting up, make sure to turn off the Field.


  #ifdef STANDALONE
        pinMode(CHARGE_PUMP_PORT,   OUTPUT);
        analogWrite (CHARGE_PUMP_PORT,128);                                                 // Get the Charge Pump a going, build up that FET boost voltage!
        #endif



        //---   The Arduino environment is made to be simple, and in many ways it is.  But also that simplicity causes problems in the details.
        //      In writing the code I have tried to use mostly Arduino style functions, but there are cases where I need to use the rather more
        //      'complex and complete' AVR type coding style, directly accessing the ATmega328 hardware.  Here is one example:

    set_PWM_frequency();                                                                    // Set Timer 1 (Pin 9/10 PWM) to 122Hz (from default 488hz).  This more matches
                                                                                            // optimal Alternator Field requirements, as frequencies above 400Hz seem to send
                                                                                            //  (See device_unique.h for details)

    attachInterrupt (STATOR_IRQ_NUMBER, stator_IRQ, RISING);                                // Setup the Interrupt from the Stator.

    set_ALT_mode(unknown);                                                                  // We are just starting out...

    return(true);
}










//------------------------------------------------------------------------------------------------------
// Stator IRQ Handler
//      This function is called at on every spike from the Stator sensor.  It is used to estimate RPMs of
//      the engine, and well as to synchronize the current and voltage sampling of the alternator
//      via the INA-226 chip.
//
//------------------------------------------------------------------------------------------------------
void stator_IRQ()
{

   interuptCounter++;
   statorIRQflag = true;                                                                                // Signal to the main loop that an the stator voltage has started to rise.


}







//------------------------------------------------------------------------------------------------------
// Calculate RPMs
//      This function will calculate the RPMs based in the current interrupt counter and time between last calculation
//
//------------------------------------------------------------------------------------------------------

void calculate_RPMs() {


   // Calculate RPMs

   long workingTime;
   int  workRPMs;
   int  workingCounter;

   workingTime    = micros();
   workingCounter = interuptCounter;                                                                    // Utilizing a local snap-shot of the interrupt counter variable so that we can have a stable value and
                                                                                                        // do out calculations w/o disabling IRQs.


   if (workingTime - (long)(priorInterupt_uS + (long)((long)IRQ_uS_TIMEOUT * (long)RPM_IRQ_AVERAGING_FACTOR)) >= 0) {
                                                                                                        // If we have waited too long for the needed number of IRQs, we are either just
                                                                                                        //     starting, (priorInterupt_uS == 0), or we are not getting any stator IRQs at the moment.
                                                                                                        //  Or if the Field is turned off - and hence IQRs are unstable . . .
        priorInterupt_uS  = workingTime;                                                                        // Reset all the counters.
        interuptCounter   = 0;
        measuredRPMs      = 0;                                                                          // Let the world know we have no idea what the RPMs are...

        return;
        }



   if (workingCounter >= RPM_IRQ_AVERAGING_FACTOR) {                                                    // Wait for several interrupts before doing anything, a smoothing function.

      workRPMs  = (int) ((60 * 1000000 /(workingTime - priorInterupt_uS) * workingCounter )
                       / ((systemConfig.ALTERNATOR_POLES * systemConfig.ENGINE_ALT_DRIVE_RATIO)/2));
                                                                                                        // Calculate RPMs based on time, adjusting for # of interrupts we have received,
                                                                                                        // 60 seconds in a minute, number of polls on the alternator,
                                                                                                        // and the engine/alternator belt drive ratio.

      if (workRPMs > 0) {                                                                               // Do we have a valid RPMs measurement?
         measuredRPMs   = workRPMs;                                                                     // Yes, take note of it.  If we had calculated a negative number (due to micros() wrapping),
                                                                                                        // just ignore it this time around and update the value next cycle.

         //     Now - letÃ¢â‚¬â„¢s see if we need to be looking for any tests or levels we need to save.

         if ((systemConfig.FIELD_TACH_PWM == -1)  &&                                                    // Did user set this to Auto Determine Field PWM min for tech mode dive?
             ((fieldPWMvalue < thresholdPWMvalue)  || (thresholdPWMvalue == -1)))                       //  And is this either a new PWM drive 'low', or have we never even see a low value before ( == -1)
                thresholdPWMvalue = fieldPWMvalue;                                                      //  Yes, Yes, and/or Yes:  So, lets take note of this PWM value.


         if (thresholdPWMvalue > MAX_TACH_PWM)                                                          // Range check:  Do not allow the 'floor' PWM value to exceed this limit, a safety in case something goes wrong with auto-detect code...
                thresholdPWMvalue = MAX_TACH_PWM;



         priorInterupt_uS = workingTime;                                                                // Reset the averaging / smoothing counters and wait for the next group of IRQs.
         interuptCounter  = 0;
         }
     }

}








//------------------------------------------------------------------------------------------------------
//
//      Calculate Alternator Targets
//
//
//      This function calculates target or goals values for Alternator Volts, Amps and Watts limits.
//      After calculating, it will update appropriate target Global Variables.
//
//------------------------------------------------------------------------------------------------------

void  calculate_ALT_targets(void) {

int     i;


 if (cpIndex >= MAX_CPES) {                                                                             // As this is a rather critical function, do a range check on the current index
        alternatorState = FAULTED;
        faultCode       = FC_LOG_CPI_STATE;                                                             // cpIndex is out of array bound....
        }                                                                                               // Fall through and the case(alternatorState) = default will set values = 0
                                                                                                        // in effect turning off the regulator.


 if ((measuredBatTemp != -99) && (alternatorState != RBM_CVCC) &&                                       // If we are able to measure battery temperature (and remote Master is not in TOTAL control of things...)
     ((measuredBatTemp >= workingParms.BAT_MAX_CHARGE_TEMP) || (measuredBatTemp <= workingParms.BAT_MIN_CHARGE_TEMP)))
        set_ALT_mode(float_charge);                                                                     // If we are too warm or too cold - force charger to Float Charge safety voltage.




    switch (alternatorState) {
        case pending_R:
        case ramping:
        case bulk_charge:
        case determine_ALT_cap:
        case acceptance_charge:
        case RBM_CVCC:
                set_VAWL(workingParms.ACPT_BAT_V_SETPOINT);                                                     // Set the Volts/Amps/Watts limits (See helper function just below)
                break;


        case overcharge_charge:
                set_VAWL(workingParms.EXIT_OC_VOLTS);                                                           // Set the Volts taking into account comp factors (system voltage, bat temp..)
                targetAltAmps  = min(targetAltAmps, (workingParms.LIMIT_OC_AMPS * systemAmpMult));              // Need to override the default Amps / Watts calc, as we do things a bit different in OC mode.
                targetAltWatts = min(targetAltWatts,(workingParms.EXIT_OC_VOLTS * systemVoltMult * targetAltAmps));
                break;



        case float_charge:
        case forced_float_charge:
                set_VAWL(workingParms.FLOAT_BAT_V_SETPOINT);                                                    // Set the Volts/Amps/Watts limits (See helper function just below)

                if (workingParms.LIMIT_FLOAT_AMPS  != -1) {
                    targetAltAmps  = min(targetAltAmps, (workingParms.LIMIT_FLOAT_AMPS     * systemAmpMult));   // If user wants to 'regulate' Amps during Float, need to override the default Amps / Watts calcs.
                    targetAltWatts = min(targetAltWatts,(workingParms.FLOAT_BAT_V_SETPOINT * systemVoltMult * targetAltAmps));
                    }

                break;



        case equalize:
                set_VAWL(workingParms.EQUAL_BAT_V_SETPOINT);                                            // Set the Volts/Amps/Watts limits (See helper function just below)

                if (workingParms.LIMIT_EQUAL_AMPS != 0) {
                    targetAltAmps  = min(targetAltAmps, (workingParms.LIMIT_EQUAL_AMPS      * systemAmpMult));
                    targetAltWatts = min(targetAltWatts,(workingParms.EQUAL_BAT_V_SETPOINT  * systemVoltMult * targetAltAmps));
                }
                break;                                                                                  // In equalization mode need to re-calc the Watts limits, as one of the
                                                                                                        // ChargeMode config parameters is a limit of Amps - independent of the Alternator
                                                                                                        // capability.


        default:
                targetBatVolts = 0.0;                                                                   // We are shut down, faulted, or in an undefined state.
                targetAltWatts = 0;
                targetAltAmps  = 0;
        }





    //----  CAN stuff - when the regulator is participating in a 'system', it needs to check with its neighbors and the current acting Remote Battery Master.
    //      Primarily thing to consider are pull-backs because too much energy is being produced and delivered to the battery.  But we also need to work out the
    //      'prioritization' of charging sources, where if the battery (and any associated house loads) are less than the total available charging source capacity,
    //      we want the lower 'priority' charging sources to back off.  Example:  Let the high priority Solar devices work Flat-out, while the generator (alternator) sources back off.
    //      It is here that we look at the current situation and adjust the goals for THIS device relative to the rest of the world.

#ifdef SYSTEMCAN
   if ((CAN_RBM_sourceID != 0) && (ignoringRBM == false) &&
       (CAN_RBM_desiredChargeState != RVCDCbcm_Undefined)  && (CAN_RBM_desiredChargeState != RVCDCbcm_Unknown)) {

        set_VAWL(CAN_RBM_desiredVolts / systemVoltMult);                                                // RBM is controlling us, we need to make some other checks and adjustments.
                                                                                                        // 1st we will set the voltage goal to what the RBM is asking for.
                                                                                                        // Calling set_VAWL() again will also reset any CPE based Amp/Watts overrides above - as
                                                                                                        // we want the RBM to make those decisions, not the local regulator CPE entries.
 
        //----   1st back-off check:  Are we the lowest-priority charging source?  Do we need to do some prioritization?
        //         e.g.: Is too much current is being delivered to the battery?  
        if ((millis() - CAN_LPCS_lastReceived) > REMOTE_CAN_LPCS_TIMEOUT) {                             // Are we the LPCS (Lower Priority Charging Source) who should be taking prioritization action??
        
            if (CAN_RBM_amps > CAN_RBM_desiredAmps) {                                                   // Is the 'system' over-delivering Amps to the battery
                                                                                                        // Yes, we need to to something.
                if (shuntAmpsMeasured) {                                                                // If we it seems we are able to measure alternator current output, use AMPS to set the 'high limit'                                
                    if ((millis() - CAN_EPCS_lastReceived) > REMOTE_CAN_LPCS_TIMEOUT)
                         targetAltAmps = measuredAltAmps - (CAN_RBM_amps - CAN_RBM_desiredAmps)/2;      // If there someone of equal priority let's share the burden
                    else targetAltAmps = measuredAltAmps - (CAN_RBM_amps - CAN_RBM_desiredAmps);        // Not sharing, looks like we are truly the LPCS, try to do all the backup ourselves.
                                                                                                        // (Note:   Shared burden:  At present, the back off is just a rough 50/50.  Need to see how this works in the
                                                                                                        //          case where there are more than 2x 'same priority' charging sources - think several small solar MPPT
                                                                                                        //          controllers.   It may be that we need to be smarter - maintain an actual count of 'same priority' charging
                                                                                                        //          sources.  Or perhaps only back off 10% each time, and let the world slowly settle...)
                }
                else                                                                                    // Well, we are not configured to measure AMPs (or the shunt broke).  Need to do something else..
                    fieldPWMLimit = min(fieldPWMLimit, fieldPWMvalue) * ((CAN_RBM_amps - CAN_RBM_desiredAmps) / CAN_RBM_amps);
            }                                                                                           // Rather crude..  But if are x% over limit, pull back the field x% from its current value...
                                                                                                        //!! HEY !! THIS MAY NEED TO BE IMPROVED, AS IT MIGHT CAUSE A SLOW OSCULATION DUE TO OVER CORRECTION...
                                                                                                        
                                                                                                            
            if (CAN_RBM_amps < CAN_RBM_desiredAmps) {                                                   // Is the 'system' under-delivering Amps to the battery?
               if (shuntAmpsMeasured)                                                                   // If we it seems we are able to measure alternator current output, use AMPS to set the 'high limit'                                
                         targetAltAmps = measuredAltAmps + (CAN_RBM_desiredAmps - CAN_RBM_amps);        // Bump up the AMPs we regulate to by the gap.
                                                                                                        // Note that if Amps are not being measured, we will not do anything in this case - just let the PID
                                                                                                        // engine raise the PWM the change-cap rate each cycle.  This WILL result in a little overshoot due to 
                                                                                                        // lag in receiving revised Amperage measurement from the RBM.
            }
        }



        //---   2nd check:  Is there a higher priority charging soruce which is under-utilized?
        if  ((millis() - CAN_HPUUCS_lastReceived) <= REMOTE_CAN_HPUUCS_TIMEOUT) {                       // Did the CAN code recently notice someone being under-utilized?
                fieldPWMLimit = min(fieldPWMLimit, (fieldPWMvalue - PWM_CHANGE_CAP));                   // Yes, Start capping the PWM Field Drive a little below where manage_alt() currently has it.
                                                                                                        // Once manage_alt() has actually lowered the PWM value, we can check again to see if the system still
         }


        
        
        
        
        //---   3nd check:  Is there an equal priority charging source in the system?  If so, we should try and equally distribute the load across all of them.
        //      Examples would be:  Dual engines with alternator  on each, or perhaps a series of solar MPPT controllers..
        if (((millis() - CAN_EPCS_lastReceived) > REMOTE_CAN_LPCS_TIMEOUT) && (average_EPC_utilization != 0)) { 
                                                                                                        // Is there someone of equal priority, who we should be sharing things will?
            int perUtil = ALT_Per_Util();                                                               // Looks like it.  Take note what OUR utilization is.. 
            if (perUtil > (average_EPC_utilization * 1.2))                                              // Are we producing more then the average output of our peers by a bunch?
                fieldPWMLimit = min(fieldPWMLimit, fieldPWMvalue) * ((perUtil - average_EPC_utilization) / perUtil);
                                                                                                        // Yes, pull back some.
        } else
            average_EPC_utilization = 0;                                                                // No one is talking to us anymore. (Been too long)  Reset the average value...

        
        
        //----  Cleanup, do range checks to make sure we have not ended up with some odd values.
        if (fieldPWMLimit < FIELD_PWM_MIN)   fieldPWMLimit = FIELD_PWM_MIN;                             // Negative value check.
        if (tachMode)                        fieldPWMLimit = max(fieldPWMLimit, thresholdPWMvalue);     // And do not pull down too much so that we lose the Tach sync.
        if (targetAltAmps < 0.0)             targetAltAmps = 0.0;

        
        

   } /* end of 'are we under RBM control?'  (Ala, participating in a 'system' and need to act in a coordinated way with the rest of the charging sources) */
   else 
#endif      /* CAUTION:  Note rather 'tricky' use of the #endif..   It allows the ELSE to fall directly into the following IF statement.  Take care that changes after the IF do not break this 'tricky' usage.. */
   if ((targetBatVolts != 0.0) && (measuredBatTemp != -99))  {                                          // Charge mode is not under RBM control.  (Though, they may still be sending remote Vbat, Amps, temp info..)
                                                                                                        // If we can read the Bat Temp probe, do the Battery Temp Comp Calcs.
        if (measuredBatTemp < workingParms.MIN_TEMP_COMP_LIMIT)                                         // 1st check to see if it is really cold out, if so only compensate up to the
              i = workingParms.MIN_TEMP_COMP_LIMIT;                                                     // 'limit' - else we risk overvolting the battery in very cold climates.
        else  i = measuredBatTemp;

        targetBatVolts += (BAT_TEMP_NOMINAL - i) * workingParms.BAT_TEMP_1F_COMP * systemVoltMult ;
        }


}                                                                                                       //  End of  calculate_ALT_targets();








//-------       'helper' process called by a LOT of places in calculate_ALT_targets();
//              Pass in the targetBatVolts you want to use and this will set the global variable and may use it to calc the target system watts.
//              This function will also 'adjust' the passed in set voltage parameter based in the current systemVoltMult

void set_VAWL(float passedV) {

        targetBatVolts = passedV * systemVoltMult;                                                      // Set global regulate-to voltage.

                                // Set the 'high water limits' (Alt Amps, Watts, and max PWM to use) applying various de-rating values.
                                //   Note that we ALWAYS apply the de-rating values, even if the user has told us the Alternator size.
                                //   Note also that in addition to de-rating via measured Amps, we will also 'de-rate' based in the max
                                //   PWM value to be sent out.
                                //   Some of these 'global' limits will be over-written after calling set_VAWL(), example, when determining alternators
                                //   capacity, in some of the float modes and at times when we are in-sync with a remote battery master via the CAN bus.


                               
                                                                                                        // Start with the biggie, the overall de-rating % based on user's selection
        if (measuredAltTemp == -100){                                                                   // User has indicated they want 'half power' mode by shorting out the Alt Temp NTC sender.
            targetAltAmps = systemConfig.ALT_AMP_DERATE_HALF_POWER * altCapAmps;
            fieldPWMLimit = systemConfig.ALT_AMP_DERATE_HALF_POWER * FIELD_PWM_MAX;                     // Prime the 'max' allowed PWM using the appropriate de-rating factor.  (Will adjust later for idle)
            }
        else if (smallAltMode  == true){                                                                //  User selected Small Alternator mode, lets treat it gently.
            targetAltAmps = systemConfig.ALT_AMP_DERATE_SMALL_MODE * altCapAmps;
            fieldPWMLimit = systemConfig.ALT_AMP_DERATE_SMALL_MODE * FIELD_PWM_MAX;
            }
        else {                                                                                          //  No de-ratings, Full Power (Well, maybe just a little back from that if user configured DERATE_NORMAL ..)
           targetAltAmps = systemConfig.ALT_AMP_DERATE_NORMAL * altCapAmps;
           fieldPWMLimit = systemConfig.ALT_AMP_DERATE_NORMAL * FIELD_PWM_MAX;
           }




                                // Now we will see if we need to further adjust down the max PWM allowed based in the Alternator RPMs.
                                //   (Trying to prevent frying the alternator with Full Field if the engine is turning slowly - or perhaps stopped)

        if ((thresholdPWMvalue > 0) && (measuredRPMs == 0) && (systemConfig.ALT_PULLBACK_FACTOR == -1)) // At one time have we seen RPMS?  But not now?  And has user selected 70% cap for this situation?
            fieldPWMLimit = min(fieldPWMLimit, FIELD_PWM_MAX * 0.70);                                   // Yes, cap PWM to 70% max field -- reduce stress on field.


        if ((measuredRPMs != 0) && (systemConfig.ALT_PULLBACK_FACTOR > 0)){                             // Well, we still can measure RPMs - Are we configured to do a PWM pull-back based on the current RPMs?
                fieldPWMLimit = constrain((60 + (3*(measuredRPMs - systemConfig.ALT_IDLE_RPM)/systemConfig.ALT_PULLBACK_FACTOR)), FIELD_PWM_MIN, fieldPWMLimit);
                                                                                                        // This actually comes to around 0.8% additional PWM for every APBF RPMs, but letÃ¢â‚¬â„¢s call it close enough..
                                                                                                        // User configurable ALT_PULLBACK_FACTOR  (via PBF in $SCA command) determine how quickly this pull-back is phased out.
                fieldPWMLimit = max(fieldPWMLimit, thresholdPWMvalue);                                  // However, do not pull down too much so that we lose the Tach sync.
                }


                                // Finally, do we need to adjust things out for some other 'special cases'

        if ((targetAltAmps == 0) || (alternatorState == determine_ALT_cap))
                targetAltAmps = 1000;                                                                   // If we need to do a new auto-size cycle (or user has told us to disable all
                                                                                                        // AMP capacity limits for the Alternator), set Amps to a LARGE number
                                                                                                        // (Ok you, yes YOU!  If you are here to change this 1000A value it must mean you have a serious system.
                                                                                                        //  Which is why I put in the FIXED 1000 value - to get you HERE to consciously make a change, showing you
                                                                                                        //  understand what you are doing!!!  Send me an Email, would like to hear how this all works for you...)
        if (alternatorState == determine_ALT_cap)
                fieldPWMLimit = FIELD_PWM_MAX;                                                          // And if we are indeed determining the Alt Capacity, we need to be able to drive the Field PWM Full bore!


        if (systemConfig.ALT_WATTS_LIMIT  == -1)
                targetAltWatts = targetBatVolts * targetAltAmps ;                                       // User has selected Auto-calculation for Watts limit
           else targetAltWatts = systemConfig.ALT_WATTS_LIMIT;                                          //  Or they have specified a fixed value (or disabled watts capping by specifying 0).

        if ((targetAltWatts == 0) || (alternatorState == determine_ALT_cap))
                 targetAltWatts = 15000;                                                                // If we have NOT measured the capacity (or user has told us to disable watts capacity limits
                                                                                                        // for the System Wattage), set Watts to a LARGE number  (Ahem, see 1000A comment above  :-)

}








//------------------------------------------------------------------------------------------------------
//
//  Set Alternator Mode
//              This function is used to change the Alternator Mode.
//              If the mode is changed, it will also reset global counters and timers as appropriate.
//              Note that is the alternator is already in the requested mode, no global variables will be changes -
//              nor will the timers be reset.
//
//
//------------------------------------------------------------------------------------------------------

void set_ALT_mode(tModes settingMode)  {

    if (alternatorState != settingMode) {
        alternatorState     = settingMode;

        altModeChanged      = millis();
        LEDRepeat           = 0;                                                // Force a resetting of the LED blinking pattern
        shuntAmpsMeasured   = false;                                            // Reset the amp shunt flag, in case it has failed during operation.  (Not a total fail-safe, but this adds a little more reliability)
        enteredFloatLrAH    = accumulatedLrAH;                                  // Noting AHs at this point in case we have been asked to enter a Float, or post-float mode (one of the exits is AH based)
    }
}







//------------------------------------------------------------------------------------------------------
//
//  Set Alternator PWM
//              This function is used to change the Alternator PWM field.
//
//
//------------------------------------------------------------------------------------------------------

void  set_ALT_PWM(int PWM) {

   lastPWMChanged = millis();
   fieldPWMvalue  = PWM;


   #ifndef SIMULATION                                                              // If simulating, do NOT drive the field PWM port at all!
     analogWrite(FIELD_PWM_PORT,PWM);
     #endif

}





//------------------------------------------------------------------------------------------------------
//
//   Alternator % Utilization
//      This function returns the current % of utilization.
//      Calc is based on Amps or field %, depending on how regulator is configured.
//
//------------------------------------------------------------------------------------------------------

#ifdef SYSTEMCAN
uint8_t ALT_Per_Util(void) {

    uint8_t utilization;
    int     floor_PWM, cap_PWM;
    
    
    if ((altCapAmps > 0)  && (shuntAmpsMeasured))                                                       // If we do know the Amps capacity of the alternator, and are measuring Amps..
        utilization = (uint8_t) ((measuredAltAmps * 100.0) / altCapAmps);                               //   .. use that info to calc the % output being delivered.               
    
    else {                                                                                              // Else we need to just use raw PWM drive level...
                                                                                                        // Being a little smart - looking at the min and max PWM values allowed...
        if (thresholdPWMvalue >= 0)                                                                     // For MIN, set it to just where the stator IRQs start working.
            floor_PWM = thresholdPWMvalue;
        else
            floor_PWM = FIELD_PWM_MIN;                                                                  // And if they do not work, all we can do is set it for the MIN value.
        
                                                                                                        // For the max, see if user has de-rated the alternator any. 
        if (measuredAltTemp == -100)                                                                    // User has indicated they want 'half power' mode by shorting out the Alt Temp NTC sender.
            cap_PWM = systemConfig.ALT_AMP_DERATE_HALF_POWER * FIELD_PWM_MAX;  
        else if (smallAltMode  == true)                                                                 //  User selected Small Alternator mode, lets treat it gently.
            cap_PWM = systemConfig.ALT_AMP_DERATE_SMALL_MODE * FIELD_PWM_MAX;
        else                                                                                            //  No de-ratings, Full Power (Well, maybe just a little back from that if user configured DERATE_NORMAL ..)
           cap_PWM = systemConfig.ALT_AMP_DERATE_NORMAL * FIELD_PWM_MAX;
 

        utilization = (uint8_t) (100*(fieldPWMvalue - floor_PWM) / cap_PWM);                           // Best we can do - try to bracket the current PWM value between allowable floor and ceiling.
    }

    return(constrain(utilization, 0, 100));                                                           // Just in case something is out of bounds        
}
#endif





//------------------------------------------------------------------------------------------------------
//
//  Manage the Alternator.
//              This function will check and change the Alternator state as well as make adjustments to the PWM Field Value
//              If the system is so configured, it will also manage the auto-sampling of the Alternators AMPs capacity.
//
//              -- Consider Ramping
//              -- Consider how far from target we are, further away = hit change it harder.
//              --  And if overvoltage, drop it down even faster!
//
//
//
//
//      Note:  Make sure to also look at the ASCII command $FRM:, as it ALSO can change alternator modes...
//
//
//------------------------------------------------------------------------------------------------------

void manage_ALT()  {

        //----   Working variable used each time through, to hold calcs for the PID engine.
  float  errorV;                                        // Calc the real-time delta error (P value of PID) Measured - target:  Note the order, over target will result in positive number!
  float  errorA;
  float  errorW;


  float VdErr;                                          //  Calculate 1st order derivative of VBat error  (Rate of Change, D value of PID)
  float AdErr;                                          //  Calculate 1st order derivative of Alt Amps error
  float WdErr;                                          //  Calculate 1st order derivative of Alt Watts error
  int   ATdErr;                                         //  Calculate 1st order derivative of Alt Temp error (convert to Float in calc, leave INT here for smaller code size)
        //----  Once the PID values are calculated, we then use the PID formula to calculate the PWM adjustments.
        //      Note that many things are 'regulated', Battery Voltage, but also current, alternator watts (engine load), and alternator temperature.
  int   PWMErrorW;                                      // Watts delta (Engine  limit)                          // Calculated PWM correction factors
  int   PWMErrorV;                                      // Volts delta (Battery limited)                        //   +  --> Drive the PWM harder
  int   PWMErrorA;                                      // Alt Amps delta (Alternator limited)
  int   PWMErrorTA;                                     // Alt Temp delta (Alternator limited)

  int   PWMError;                                                                                       // Holds final PWM modification value.


        //-----  Working variables that must RETAIN their values between calls for mange_alt().  Some are for the PID, others for load-dumps management and temperature pull-backs.
  float static ViErr  = 0;                              // Accumulated integral error of VBat errors - retained between calls to manage_alt();
  float static AiErr  = 0;                              // Accumulated integral error of Alt Amps errors  (I values of PID)
  float static WiErr  = 0;                              // Accumulated integral error of Alt Watts errors

  float static  priorBatVolts     = 0;                  // These is used in manage_ALT() to implement PID type logic, it holds the battery voltage of the prior
  float static  priorAltAmps      = 0;                  // to be used to derived the derivative of VBat.
  int   static  priorAltWatts     = 0;
  int   static  priorAT           = -99;                // Start at -99 to match the 'alt temp probe not working' values in measuredAltTemp

  int8_t static TAMCounter        = TAM_SENSITIVITY;    // We will make ADJUSTMENTS based on Temp Errors only every x cycles through adjusting PWM.



  bool    static otCycleTriggered  = false;             // Used to take down target watts based on over temp situations (Eng, Alt, or EGT).
                                                        // This flag will make sure we do this takedown only once per overtemp cycle.
  float   static otWattsPullbackFactor = 1.0;           // And IF we have detected an Over Temp condition, we want to reduce the load on the engine and
                                                        // try and help cool things off.  THIS variable is used to accomplish this
                                                        // by adjusting the targetAltWatts before calculating the watts error in manage_alt.
                                                        // Initialize with no pull-back factor (= 1.0).


  bool    static LD1Triggered    = false;                      // Has one of the Load Dump thresholds been triggered?  (This keeps us from over correcting)
  bool    static LD2Triggered    = false;

  unsigned long enteredMills;                           // Time in millis() managed_alt() was entered.  Used throughout function and saves 300 bytes of code vs. repeated millis() calls
  char          charBuffer[OUTBOUND_BUFF_SIZE+1];       // Used to assemble Debug ASCII String (if needed)



        //------ NOW we can start the code!!
        //

        if (updatingVAs) return;                                                                // If Volts/Amps measurements are being refreshed just skip checking things this time around until they are ready.


        errorV = measuredBatVolts -  targetBatVolts;                                            // Calc the error values, as they are used a lot down the road.
        errorA = measuredAltAmps  -  targetAltAmps;                                             // + = over target, - = under target.
        errorW = measuredAltWatts - (targetAltWatts * otWattsPullbackFactor);                   // (Adjust down Target Alt Watts for any overtemp condition...)

        enteredMills = millis();                                                                // Remember this as we are going to use it a lot..
                                                                                                // Using the working variable saves code size, and also assures we have consistency with all
                                                                                                // the time-stamping that will happen inside of manage_alt()












        //----  1st, seeing as we have a new valid voltage reading, letÃ¢â‚¬â„¢s do the quick over-voltage check as well check to see if there is if there seems to be load-dump situation...
        //

        if (errorV > (LD1_THRESHOLD * systemVoltMult)) {                                        // Yes, we are AT LEAST over the 1st line...

           if (!LD1Triggered) {                                                                 // OK, this is the 1st time we have seen this level
                fieldPWMvalue *= LD1_PULLBACK;                                                  // Cut the Field PWM drive a some this 1st overvoltage step.
                LD1Triggered   = true;                                                          // but note that we have already done the LD1 reduction, to prevent overcorrection.
                }

           if ((errorV > (LD2_THRESHOLD * systemVoltMult)) && (!LD2Triggered)) {                // Over the 2nd trip level?  (and 1st time we have seen this slightly higher voltage?)
                fieldPWMvalue *= (LD2_PULLBACK / LD1_PULLBACK);                                 // Yes, cut it again - harder this time...
                LD2Triggered   = true;                                                          // Note that % value is cumulative with prior LD pullback..  Using  /LDx_.. to back-out the cumulative effect
                }

 

           if (errorV  > (LD3_THRESHOLD * systemVoltMult)) {                                    // And finally, if we are way over just shut things down
            #ifndef SIMULATION                                                              
              analogWrite(FIELD_PWM_PORT,FIELD_PWM_MIN);                                        // OK, this is (hopefully) a short-term spike.  To help things along, just spike the field down
              #endif                                                                            // while overvolt.  Once things have settled down, let the 'regulator' handle things normally.  
                                                                                                // (or said another way:  pull the field down, but do not adjust the running PWM value...)
            sample_ALT_VoltAmps();                                                              // Start a new local VA sample cycle, and see how things settle out when we resume our normal PID regulation cycle.
            LD1Triggered = false;                                                               // Resetting flags, if when we come back in we are STILL overvoltage 
            LD2Triggered = false;
            return;                                                                             // (I expect this to give a 10-20mS 'shot' of 0 field drive)



           }



           if (tachMode)
                fieldPWMvalue = max(fieldPWMvalue, thresholdPWMvalue);                          // But if Tach mode, do not let PWM drop too low - else tach will stop working.

           set_ALT_PWM(fieldPWMvalue);                                                          // Send out the new PWM value (do it here to get quick response, and also in case the time-loop
                                                                                                // PWM_CHANGE_RATE has not occurred, as that will cause manage_alt() to exit this time through).
           }
        else {
           LD1Triggered = false;                                                                // Does NOT look like a load-dump (not exceeding the LD1 threshold) so reset the triggers
           LD2Triggered = false;
           }





        //---   OK then, let's get on with the rest of things.  But 1st, is it even time for us to adjust the PWM yet?
        //       Aside from the Load Dump checks we did above (which happen every time we get a new Vbat reading) we need to take some time to let the alternator and system
        //       settle in to changes.   Alts seem to take anywhere from 100-300mS to 'respond' to a change in PWM up, a bit less for down.  By controlling how often we
        //       try to adjust the PWM, we give the system time to respond to a prior change.
        //       A 2nd benefit is that by using a fixed time between adjustments  the PID calculations are somewhat simplified, specifically in the Derivative and Integral factors.

        if ((enteredMills - lastPWMChanged) <= PWM_CHANGE_RATE)                                         //   Is it too soon to make a change?
                return;                                                                                 //      Yes - skip doing anything this time around.







        //--- Calculate the values for the Integral (I) and 1st order Derivative (D) of the PID engine.
        //

        VdErr =  measuredBatVolts - priorBatVolts;                                                      // 'D's 1st ! Note we are using the D of the 'input' to the PID engine, this avoids the
        priorBatVolts = measuredBatVolts;                                                               //  issue knows as the 'Derivative Kick'

        AdErr =  measuredAltAmps - priorAltAmps;
        priorAltAmps = measuredAltAmps;

        WdErr =  measuredAltWatts - priorAltWatts;
        priorAltWatts = measuredAltWatts;


        ATdErr = 0;                                                                                     // Assume we will NOT be doing Alt Temp calcs this time around.
        if (--TAMCounter  <= 0){                                                                        // We will make ADJUSTMENTS based on Temp Error only every x times through.
            if (priorAT > 0)                                                                            // Are we even measuring the Alt temp?
                ATdErr  = max(measuredAltTemp,measuredAlt2Temp) - priorAT;                              // But only if we have done this once before, and hence set the 'prior' alt temperature.
                                                                                                        // Take note, we only do any alt pull-back calc if the temps are above 0 degrees.  If the alt is
                                                                                                        // really that cold, letÃ¢â‚¬â„¢s just assume we can drive it harder.  (Plus, code would need to be larger
                                                                                                        // to adjust for neg temps.  Ala, going from -20 to -15 is really a positive 5 change, not -5.
                                                                                                        // by just checking for >0 we avoided needing to use additional code space for an ABS() function.
            TAMCounter  = TAM_SENSITIVITY;
            priorAT = max(measuredAltTemp,measuredAlt2Temp);
            }



        ViErr  += (errorV  * KiPWM_V  / systemVoltMult);                                                // Calc the I values.
        AiErr  += (errorA  * KiPWM_A);                                                                  // Note also that the scaling factors are figured in here, as opposed to during the PID formula below.
        WiErr  += (errorW  * KiPWM_W  / systemVoltMult);                                                // Doing so helps avoid issues down the road if we ever implement an auto-tuning capability, and change the I

        ViErr  = constrain( ViErr, 0, PID_I_WINDUP_CAP);                                               // Keep the accumulated errors from getting out of hand, their impact is meant to be a soft refinement, not a
        AiErr  = constrain( AiErr, 0, PID_I_WINDUP_CAP);                                               // sledge hammer!
        WiErr  = constrain( WiErr, 0, PID_I_WINDUP_CAP);                                               // Also - ONLY use 'I' to pull-back the PWM, never to allow it to be driven stronger.

        PWMErrorV   = (int) ((errorV * -KpPWM_V / systemVoltMult)  -  ViErr  - (VdErr * KdPWM_V / systemVoltMult));
        PWMErrorA   = (int) ((errorA * -KpPWM_A)                   -  AiErr  - (AdErr * KdPWM_A));
        PWMErrorW   = (int) ((errorW * -KpPWM_W / systemVoltMult)  -  WiErr  - (WdErr * KdPWM_W / systemVoltMult));

        if (max(measuredAltTemp,measuredAlt2Temp) > 0)
                PWMErrorTA = ((float)(systemConfig.ALT_TEMP_SETPOINT - max(measuredAltTemp,measuredAlt2Temp))*KpPWM_TA) - ((float)ATdErr * KdPWM_TA);
          else  PWMErrorTA = PWM_CHANGE_CAP;                                                            // Only do Temp Adjustments if we are able to read Temps (and it is not very very cold..)!
                                                                                                        //  Else allow just a little raise, until we hit some other limit.







                        //---   OK, now that we have the individual PWM errors calculated, we want to do some special checks.
                        //      The one is to help reduce a tug-of-war between the system overheating (Ta, Te, Tx) which will cause
                        //      PWMs to be reduced - and once things have cooled off some having the Watts target bring back the SAME load
                        //      that caused the over-temp situation in the 1st place.  Thus creating an osculation.
                        //      So, to reduce that we will take down the Watts Target (targetAltWatts) each time we get into a large overtemp
                        //      situation.


        if (PWMErrorTA   <= OT_PULLBACK_THRESHOLD) {                                                    // Is the Alternator over temp by a noticeable amount?
             if (otCycleTriggered != true)  {                                                           // Yes, have we seen this before?

                  otWattsPullbackFactor  *= OT_PULLBACK_FACTOR;                                         // No, this is the 1st time.  So Pull back the target watts some % of its current value
                  otCycleTriggered        = true;                                                       // And set the flag so that we will not do another pull down this cycle
                 }

             }  else
                otCycleTriggered = false;                                                               // None of the temp based values are noticeable over the limit, so letÃ¢â‚¬â„¢s reset the cycle flag
                                                                                                        // and if we again get into an overtemp situation we will again pull down the target watts.




                        //---   Now lets calculate how much total error we have accumulated
                        //      We will let the 'littlest kid' win.  Meaning, we will adjust the PWM up only as much as the smallest one calculated for above will call for.
                        //      while also letting the largest pull-down control things when needed.

        PWMError = min(PWMErrorV, PWMErrorA);                                                           // If there is ANYONE who thinks PWM should be pulled down (or left as-is), let them have the 1st say.
        PWMError = min(PWMError,  PWMErrorW);
        PWMError = min(PWMError,  PWM_CHANGE_CAP);                                                      // While we are at it, make sure we do not ramp up too fast!  (Note that ramping down is not capped)
                                                                                                        // We only do temperature based adjustments based on the slower cycle time (e.g. TAM_SENSITIVITY)
        if (TAMCounter == TAM_SENSITIVITY)                                                              // Over Alternator, adjust down due to overtemp.
                PWMError = min(PWMError, PWMErrorTA);

        if (PWMErrorTA  <= 0)                                                                           // But even if the otCycleCounteris holding things off, if devices are over-temp do not drive things harder..
                 PWMError = min(PWMError, 0);                                                           // Do however make sure to carry forward anyone else who needs to take the PWM down (e.g., VBat , etc)






        //----  Check to see if it is time to transition charging phases
        //
        //        Ramp   --> Bulk:    Total time allocated for Ramping exceeded, or we are driving alternator full bore.
        //        Bulk   --> Accept:  VBatt reached max
        //        Accept --> Float:   Been in Accept phase for EXIT_ACPT_DURATION
        //        Float  --> ????
        //


        if (measuredBatAmps >= persistentBatAmps)                                                       // Adjust the smoothing Amps variable now.
           persistentBatAmps = measuredBatAmps;                                                         // We want to track increases quickly,
        else if (measuredBatAmps > 0.0)                                                                 // but decreases slowly...  This will prevent us from changing state too soon. (And don't count discharging)
           persistentBatAmps = (((persistentBatAmps * (float)(AMPS_PERSISTENCE_FACTOR-1L)) + measuredBatAmps) / AMPS_PERSISTENCE_FACTOR);


        if (measuredBatVolts >= persistentBatVolts)                                                     // Adjust the smoothing Volts variable now.
           persistentBatVolts = measuredBatVolts;                                                       // We want to track increases quickly,
        else                                                                                            // but slowly decreases...  This will prevent us from changing state too soon.
           persistentBatVolts = (((persistentBatVolts * (float)(VOLTS_PERSISTENCE_FACTOR-1)) + measuredBatVolts) / VOLTS_PERSISTENCE_FACTOR);



        switch (alternatorState) {



          case pending_R:
                if ((tachMode) && (systemConfig.FIELD_TACH_PWM > 0))                                    // If user has configured system to have a min PWM value for Tach mode
                        fieldPWMvalue = systemConfig.FIELD_TACH_PWM;                                    // send that out, even during engine warm-up period.
                else    fieldPWMvalue = FIELD_PWM_MIN;                                                  //  All other cases, Alternator should still be turned off.

                PWMError  = 0;                                                                          // In we will not be making ANY adjustments to the PWM for now.

                if (((enteredMills- altModeChanged) > ENGINE_WARMUP_DURATION) &&                        // Have we been in WarmUp period long enough to start ramping?
                     ~((tachMode) && (systemConfig.FIELD_TACH_PWM > 0) && (measuredRPMs == 0)))         //  (But 1st - if we expect to be able to measure RPMs, don't leave pending until we do  (Engine might be stopped))
                        set_ALT_mode(ramping);                                                          // It is time to start Ramping!

                break;





          case ramping:
                persistentBatAmps  = measuredBatAmps;                                                   //  While ramping, just track the actually measured amps and Watts.
                persistentBatVolts = measuredBatVolts;                                                  //  Overwriting the persistence calculation above.



                if (systemConfig.ALT_AMPS_LIMIT  != -1)                                                 // Starting a new 'charge cycle' (1st time or restart from float).  Re-sample Alt limits
                     altCapAmps = systemConfig.ALT_AMPS_LIMIT;                                          // User is telling us the capacity of the alternator (Or disabled Amps by setting this = 0)
                else altCapAmps = 0;                                                                    // User has selected Auto-determine mode for Alternator size, so reset the high water
                                                                                                        // mark for the next auto-sizing cycle.
                altCapRPMs = 0;




                if ((fieldPWMvalue >= fieldPWMLimit) ||                                                 // Driving alternator full bore?
                    (errorV >= 0)                    ||                                                 // Reached terminal voltage?
                    (errorA >= 0)                    ||                                                 // Reached terminal Amps?
                    (errorW >= 0)                    ||                                                 // Reached terminal Watts?  (Reaching ANY of these limits should cause exit of RAMP mode)
                    ((enteredMills - altModeChanged) >=  PWM_RAMP_RATE*FIELD_PWM_MAX/PWM_CHANGE_CAP)) {
                                                                                                        // Or, have we been ramping long enough?

                        if (systemConfig.ALT_AMPS_LIMIT == -1)  set_ALT_mode(determine_ALT_cap);        //      Yes or Yes!  Time to go into Bulk Phase
                             else                               set_ALT_mode(bulk_charge);              //      But 1st see if we need to measure the Alternators Capacity...
                } else {
                    if ((enteredMills - lastPWMChanged) <= PWM_RAMP_RATE)                               // Still under limits, while ramping wait longer between changes..
                        return;
                }

                break;






          case determine_ALT_cap:
                if ((systemConfig.ALT_AMPS_LIMIT != -1) ||                                              // If we are NOT configured to auto-determining the Alt Capacity,   --OR--
                    (fieldPWMvalue == FIELD_PWM_MAX)    ||                                              // we have Maxed Out the Field (PWM capping should have been removed during alt_cap mode) --OR--
                    (errorV >= 0)                       ||                                              // Reached terminal voltage?  --OR-- (meaning, we really can not finish determine the Alt cap as the battery is kind of full.....)
                    ((enteredMills - altModeChanged) >= SAMPLE_ALT_CAP_DURATION)){                      // And have been doing an Alt Cap Sampling Cycle long enough..

                        set_ALT_mode(bulk_charge);                                                      // Stop this cycle - go back to Bulk Charge mode.
                        }


                if (measuredAltAmps > altCapAmps) {                                                     // Still pushing the alternator hard.
                        altCapAmps = measuredAltAmps;                                                   // Take note if we have a new High Amp Value . . .
                        altCapRPMs = measuredRPMs;
                        }

                break;






                //---  BULK CHARGE MODE
                //      The purpose of bulk mode is to drive as much energy into the battery as fast as it will take it.
                //      Bulk is easy, we simply drive the alternator hard until the battery voltage reaches the terminal voltage as defined by .ACPT_BAT_V_SETPOINT in the CPE.
                //      While in Bulk Mode, we will also see if there is reason for us to re-sample the alternator capacity (if configured to do so), ala the RPM have increased
                //      indicating the engine has sped up.


          case bulk_charge:
                if (errorV >= 0) {                                                                      // Have we reached terminal voltage during Bulk?
                   set_ALT_mode(acceptance_charge);                                                     //      Yes, Bulk is easy - got the volts so go into Acceptance Phase!
                   adptExitAcceptDuration   = (enteredMills - altModeChanged) * ADPT_ACPT_TIME_FACTOR;  // Calculate a time-only based acceptance duration based on how long we had been in Bulk mode,
                                                                                                        //  In case we cannot see Amps.
                   }


                                                                                                        // Not yet.  See if we need to start a auto-capacity sample cycle on the Alternator

                if (systemConfig.ALT_AMPS_LIMIT != -1)                                                  // Are we configured to auto-capacity sample the alternator?
                   break;                                                                               //    Nope - they told us how big it is - so done for now.

                if ((enteredMills - altModeChanged) <= SAMPLE_ALT_CAP_REST)                                     // Did we just do a capacity sample cycle, and if so have we 'rested' the alternator long enough?
                    break;                                                                              // Yes we did a cycle, and no we have not rested sufficient.

                                                                                                        // OK, we are configured to do auto Alt Sampling, we have not just done one, so . . .
                if ((measuredRPMs     > (altCapRPMs + SAMPLE_ALT_CAP_RPM_THRESH))  ||                   // IF we are spinning the alternator faster, -OR-
                    (measuredAltAmps  > (altCapAmps + SAMPLE_ALT_CAP_AMPS_THRESH))) {                   //    we have seen a new High Amp Value .

                        set_ALT_mode(determine_ALT_cap);                                                // Start a new 'capacity determining' cycle (If we are not already on one)
                                                                                                        // This last step is kind of the key.  During a new cycle we will not artificially reduce the output
                                                                                                        // of the alternator, but instead run it as hard as we can to see if we get a new High Water Mark.
                                                                                                        // After a short period of time we will then re-enable the reduced current modes (ala smallAltMode)
                        }

                break;










                //---  ACCEPTANCE CHARGE MODE
                //      In acceptance phase we are packing in extra energy into the battery until it is fully charged.  We cap the voltage to .ACPT_BAT_V_SETPOINT to keep from overheating
                //      the battery, and the battery itself controls how many Amps it will 'accept'.  The handling of accept phase is one of the key benefits of using this regulator, as
                //      by monitoring the accepted amps we can MEASURE the batteries state of charge.  Once the acceptance amps fall below about 1-2% of the batteries Ah capacity, we know
                //      the battery is fully charged.  The Amp trigger is defined by .EXIT_ACPT_AMPS, once we fall below this level we move on.  We can also move on by staying too long in
                //      acceptance phase.  CPE entry .EXIT_ACPT_DURATION allows for a time limit to be defined, and if we exceed this time limit we move on.  This can be used when the AMP
                //      shunt is not connected, but also provides a level of protection increase something goes wrong - protection from boiling the battery dry.
                //      Finally, the regulator can be configured to disable Amp based determination of Exit-Acceptance and instead to a time-based exit criteria based on some factor
                //      of the amount of timer we spent in Bulk.  This will happen if the user has entered -1 in the  EXIT_ACPT_AMPS, OR we do not seem to be able to measure any Amps
                //      (ala, the user has not connected up the shunt).  Note that even with Adaptive Acceptance duration, we will never exceed the configured EXIT_ACPT_DURATION value.




          case acceptance_charge:
                if (((enteredMills - altModeChanged) >= workingParms.EXIT_ACPT_DURATION)        ||      // 3 ways to exit.  Have we have been in Acceptance Phase long enough?  --OR--

                   (((workingParms.EXIT_ACPT_AMPS  == -1)   ||                                          // Have we been configured to do Adaptive Acceptance?
                     ((shuntAmpsMeasured == false) && (usingEXTAmps == false)))   &&                    //   (Or if we are unable to measure Amps, force Adaptive Acceptance, to protect the battery)
                    ((enteredMills - altModeChanged)      >=   adptExitAcceptDuration))         ||      // ..  Yes, early exit Acceptance Phase if we have exceeded the amount of time in Bulk by x-factor.
                                                                                                        //                                                                      --OR--

                   (( workingParms.EXIT_ACPT_AMPS         >  0)                   &&                    // Is exiting by Amps enabled, and we have reached that threshold?
                    ((shuntAmpsMeasured == true) || (usingEXTAmps == true))       &&                    //  ... and does it look like we are even measuring Amps?
                    ( errorV                              >= 0)                   &&                    //  ... Also, make sure the low amps are not because the engine is idling, or perhaps a large external load
                    ( persistentBatAmps                   <= (workingParms.EXIT_ACPT_AMPS * systemAmpMult)))  ) {       //  has been applied.  We need to see low amps at the appropriate full voltage!




                   if (workingParms.LIMIT_OC_AMPS == 0)    set_ALT_mode(float_charge);                  //      Yes -- time to float  -- OR --. . . .
                        else                               set_ALT_mode(overcharge_charge);             //         . . .into OC mode, if it is configured (Limit Amps != 0)

                }
                break;








                //---  OVERCHARGE MODE
                //      Overcharge is used by some batteries to pack just-a-little-more in after completing the acceptance phase.  (ala, some AMG batteries like an Overcharge).
                //      Overcharge holds the charge current at a low level while allowing the voltage to rise.  Once the voltage reaches a defined point, the battery is considered
                //      fully charged.  Note that to make full use of this capability, the Amp Shunt should be installed on the BATTERY, not the ALTERNATOR.
                //      The regulator will allow for overcharge to be optionally configured, compete with its own target voltage .EXIT_OC_BAT_VOLTS, Amp limit via .LIMIT_OC_AMPS, and
                //      a time limit .EXIT_OC_DURATION.    To disable OC mode, set LIMIT_OC_AMPS = 0.  (for safety, Time or Volts = 0 will also disable OC mode)


        case overcharge_charge:
                if (((enteredMills - altModeChanged) >= workingParms.EXIT_OC_DURATION) ||               // Have we have been in Overcharge Phase long enough?  --OR--
                    ( workingParms.LIMIT_OC_AMPS           == 0) ||                                     // Are we even configured to do OC mode? --OR--
                    ( workingParms.EXIT_OC_VOLTS           == 0) ||
                    ( errorV                               >= 0)) {                                     // Did we reach the terminal voltage for Overcharge mode?

                   set_ALT_mode(float_charge);                                                          // Yes to one of the conditions.  NOW we can go into Acceptance Phase
                   }

                break;






                //---  FLOAT MODE
                //      Float is not really a Charge mode, it is more intended to just hold station.  To keep the battery at a point where it will neither continue to charge,
                //      nor discharge.  You can think of it as putting in just enough energy to make up for any self-discharge of the battery.  Exiting Float will happen in several
                //      ways.  A normal exit will be by time; .EXIT_FLOAT_DURATION in the CPE will tell us how long to stay in float.  Setting this to '0' will cause
                //      us to never move out of float on to the next charge state (Post Float).  In normal operations, just hanging around in Float once a battery is fully charged is
                //      the right choice: supplying sufficient energy to keep the battery happy, and also providing any additional amps as needed to drive house loads - so those loads
                //      do not try and take energy from the battery.
                //
                //      But what happens if that house loads gets really large, too large for the Alternator to keep up?  Energy will start to be sapped out of the battery and as some time
                //      we will have to recognize the battery is no longer fully charged.  The 1st (and preferred) way is if we start to see negative Amps on the shunt.  Configuring the
                //      Exit Amps value for a neg number will let the regulator watch the battery and then go out of float when it starts to see too much of a discharge.
                //
                //      If the Amp Shunt is attached to the Alternator, one COULD also look to see if a large number of Amps is being asked for, perhaps near the full capacity
                //      of the Alternator.
                //
                //      An indirect way to recognize the battery is being drawn upon is if the battery voltage drops below .FLOAT_TO_BULK_VOLTS, the alternator will revert to Bulk mode.
                //      This is an indirect way of telling of the battery is losing energy.
                //
                //      Starting with revision 0.1.3, two new capabilities were added:
                //         1) Reevaluation of Amps while in Float (See set_alt_targets(), allowed 'managing' battery amps that flow into battery, even down to 0A
                //         2) Exit criteria based in number of Ahs that have been withdrawn from the battery after 1st entering Float mode.





          case float_charge:
                if ((((long)workingParms.EXIT_FLOAT_DURATION) != 0)  &&                                 // Has max time for Float been configured?
                    ((enteredMills - altModeChanged) >= workingParms.EXIT_FLOAT_DURATION)) {            // And have we been in Float long enough?

                        set_ALT_mode(post_float);                                                       //  Yes, go into Post Float mode for now
                        fieldPWMvalue    = FIELD_PWM_MIN;                                               //  Turn off alternator
                        PWMError         = 0;
                        }


                if (((workingParms.FLOAT_TO_BULK_AMPS  != 0) && (persistentBatAmps   < (workingParms.FLOAT_TO_BULK_AMPS  * systemAmpMult )))  ||
                    ((workingParms.FLOAT_TO_BULK_VOLTS != 0) && (persistentBatVolts  < (workingParms.FLOAT_TO_BULK_VOLTS * systemVoltMult)))  ||
                    ((workingParms.FLOAT_TO_BULK_AHS   != 0) &&
                                ((int) (((accumulatedLrAH - enteredFloatLrAH) / 3600UL) * (ACCUMULATE_SAMPLING_RATE / 1000UL))  > (workingParms.FLOAT_TO_BULK_AHS  * systemAmpMult )))) {

                                                                                                        //  Do we need to go back into Bulk mode?
                        set_ALT_mode(ramping);                                                          //      Yes! (Via ramping, so as to soften shock to fan belts)
                        break;
                        }

          case forced_float_charge:                                                                     //  If we have been 'forced' into Float mode via the Feature-in, we just stay there.
                                                                                                        //  (check_inbound() will note when the feature-in signal is removed and take us out of Float)
                break;









          case  post_float:                                                                             //  During Post-Float the alternator is turned off, but voltage is monitors to see if a large load is placed
                                                                                                        //  on the system and we need to restart charging.
                if ((((long)workingParms.EXIT_PF_DURATION) != 0)  &&                                    // Has max time for Post-Float been configured?
                    ((enteredMills - altModeChanged) >= workingParms.EXIT_PF_DURATION)) {               // And have we been in Post-Float long enough?

                        set_ALT_mode(float_charge);                                                     //  Yes, switch back to Float for a while.

                        break;
                        }


                if (((workingParms.PF_TO_BULK_VOLTS != 0.0)  && (persistentBatVolts < workingParms.PF_TO_BULK_VOLTS * systemVoltMult)) ||
                    ((workingParms.PF_TO_BULK_AHS    != 0)   &&
                                ((int) (((accumulatedLrAH - enteredFloatLrAH) / 3600UL) * (ACCUMULATE_SAMPLING_RATE / 1000UL))  > (workingParms.PF_TO_BULK_AHS  * systemAmpMult )))) {
                                                                                                        //  Do we need to go back into Bulk mode?
                        set_ALT_mode(ramping);                                                          //      Yes or Yes!  Time to go into Bulk Phase  (Via ramping, so as to soften shock to fan belts)
                                                                                                        //      In this case I go directly back into recharging, as opposed to float.  This is because the battery
                                                                                                        //      has shown some sign of discharging, so no need to flip to float to only then flip to bulk.
                        break;
                        }


                fieldPWMvalue = FIELD_PWM_MIN;                                                          //  If still in post Float mode, make sure to turn off alternator.
                PWMError      = 0;

                break;





          case equalize:
                if (((enteredMills - altModeChanged) >= workingParms.EXIT_EQUAL_DURATION)     ||
                    ((workingParms.EXIT_EQUAL_AMPS         != 0)                &&
                     ((shuntAmpsMeasured == true) || (usingEXTAmps == true))    &&                      //  ... and does it look like we are even measuring Amps?
                     (persistentBatAmps                    <= (workingParms.EXIT_EQUAL_AMPS * systemAmpMult)))  ) {
                                                                                                        // Have we have been in Equalize mode long enough?  --OR--
                                                                                                        //   Is exiting by Amps enabled, and we have reached that threshold?
                                                                                                        //   Note that we do NOT also check PWMErrorV here (as with acceptance and Overcharge)
                                                                                                        //   It is the operators responsibility to keep RPMS high and monitor Equalization charge phase.

                   set_ALT_mode(float_charge);                                                          //      Yes -- time to float  - let the main loop take care of adjusting the PWM.
                   }

                break;


          case RBM_CVCC:                                                                                // Running in true slave mode, doing what the Remote Battery Master is telling us to do.
                break;


          case disabled:
          case unknown:
                PWMError      = 0;
                LEDRepeat     = 0;                                                                      // And force a resetting of the LED blinking pattern

                fieldPWMvalue = FIELD_PWM_MIN;                                                          //  Turn off alternator.
                break;


          case FAULTED:                                                                                 // If we got called here while in FAULT condition, do not change anything.
          case FAULTED_REDUCED_LOAD:
                return;


          default:
                alternatorState = FAULTED;                                                              // We should never have gotten here.   Something is wrong. . .
                faultCode       = FC_LOG_ALT_STATE1;
                return;

          }                                                                                             //  End of Switch/case








     //-----   Put out the PWM value to the Field control, making sure the adjusted value is within bounds.
     //        (And if the Tach mode is enabled, make sure we have SOME PWM)
     //
     fieldPWMvalue += PWMError;                                                                         // Adjust the field value based on above calculations.
     if (tachMode)
        fieldPWMvalue = max(fieldPWMvalue, thresholdPWMvalue);                                          // But if Tach mode, do not let PWM drop too low - else tach will stop working.


     fieldPWMvalue = constrain(fieldPWMvalue, FIELD_PWM_MIN, fieldPWMLimit);                            // And in any case, always make sure we have not fallen out of bounds.

     set_ALT_PWM(fieldPWMvalue);                                                                        // Ok, after all that DO IT!  Update the PWM


     //-----    Before we leave, does the user want to see detailed Debug information of what just happened?
     //

     if ((sendDebugString == true) && (--SDMCounter  <= 0)) {

#ifdef SYSTEMCAN
             snprintf_P(charBuffer,OUTBOUND_BUFF_SIZE, PSTR("DBG;,%d.%03d, ,%d,%d, ,%d, ,%d,%d,%d,%d,%d, ,%d,%d, %c,%d.%03d,%d.%01d, ,%d,%d,%d,  ,%d,%d,%d,%d.%03d\r\n"),
#else
             snprintf_P(charBuffer,OUTBOUND_BUFF_SIZE, PSTR("DBG;,%d.%03d, ,%d,%d, ,%d, ,%d,%d,%d,%d,%d, ,%d,%d, %c,%d.%03d,%d.%01d, ,%d,%d,%d\r\n"),
#endif
             (int) (enteredMills / 1000UL),                                                             // Timestamp - Seconds
             (int) (enteredMills % 1000),                                                               // time-stamp - 1000th of seconds

             (int) alternatorState,
                   fieldPWMvalue,

                   digitalRead(FEATURE_IN_PORT),


                   PWMErrorV,
                   PWMErrorA,
                   PWMErrorW,
                   PWMErrorTA,
                   PWMError,

                   (int) (ViErr*1000.0),
                   (int) (VdErr*1000.0),



                  //    --- Pick one  ----
  /*                 'A',
                  (int)    measuredAltVolts,                                                            // Alternator Voltage to 1/1000th of a volt
                  frac2int(measuredAltVolts, 1000),
                  (int)    measuredAltAmps,                                                             // Alternator Amps to 1/10th of an amp
                  frac2int(measuredAltAmps, 10),
*/
                  'B',
                  (int)    measuredBatVolts,                                                            // Battery Voltage to 1/1000th of a volt
                  frac2int(measuredBatVolts, 1000),
                  (int)    measuredBatAmps,                                                             // Battery Amps to 1/10th of an amp
                  frac2int(measuredBatAmps, 10),

/*                 'P',
                  (int)    persistentBatVolts,                                                          // Battery Voltage to 1/1000th of a volt
                  frac2int(persistentBatVolts, 1000),
                  (int)    persistentBatAmps,                                                           // Battery Amps to 1/10th of an amp
                  frac2int(persistentBatAmps, 10),
*/


                  usingEXTAmps,
                  thresholdPWMvalue,
                  fieldPWMLimit

#ifdef SYSTEMCAN
                  , fetch_CAN_localID(),
                  CAN_RBM_sourceID,
                  ALT_Per_Util(),
                  (int)    CAN_RBM_voltsOffset,                                                             // Remote Battery Voltage adjustment value to 1/1000th of a volt
                  frac2int(CAN_RBM_voltsOffset, 1000)
#endif

                   );


        Serial.write(charBuffer);
        SDMCounter  = SDM_SENSITIVITY;
        }





}





