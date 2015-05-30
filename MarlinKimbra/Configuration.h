#ifndef CONFIGURATION_H
#define CONFIGURATION_H

// This configuration file contains basic settings. Select your:
//  - board type
//  - Mechanism type (cartesian-corexy-delta-scara)
//  - temperature sensor type
//
// Mechanisms-settings can be found in configuration_xxxxxx.h
// Advanced settings can be found in Configuration_adv.h


#include "boards.h"
// Choose your board type.
// Either an numeric ID or name defined in boards.h is valid.
// See: https://github.com/MagoKimbra/MarlinKimbra/blob/master/Documentation/Hardware.md

#define MOTHERBOARD BOARD_RAMPS_13_EFB

// User-specified version info of this build to display in [Pronterface, etc] terminal window during
// startup. Implementation of an idea by Prof Braino to inform user that any changes made to this
// build by the user have been successfully uploaded into firmware.
#define STRING_VERSION "4.1.2"
#define STRING_URL "reprap.org"
#define STRING_VERSION_CONFIG_H __DATE__ " " __TIME__     // build date and time
#define STRING_CONFIG_H_AUTHOR "(none, default config)"   // Who made the changes.
#define STRING_SPLASH_LINE1 "v" STRING_VERSION            // will be shown during bootup in line 1
#define STRING_SPLASH_LINE2 STRING_VERSION_CONFIG_H       // will be shown during bootup in line 2

// SERIAL_PORT selects which serial port should be used for communication with the host.
// This allows the connection of wireless adapters (for instance) to non-default port pins.
// Serial port 0 is still used by the Arduino bootloader regardless of this setting.
#define SERIAL_PORT 0

// This determines the communication speed of the printer
// 115200 - 250000
#define BAUDRATE 115200

// This enables the serial port associated to the Bluetooth interface on AT90USB devices
//#define BTENABLED

// Define this to set a unique identifier for this printer, (Used by some programs to differentiate between machines)
// You can use an online service to generate a random UUID. (eg http://www.uuidgenerator.net/version4)
//#define MACHINE_UUID "00000000-0000-0000-0000-000000000000"

// If you want test the firmware uncomment below. Use Serial arduino monitor...
//#define FIRMWARE_TEST // ONLY BAUDRATE 115200

/***********************************************************************\
 **************************** Define type printer **********************
 ***********************************************************************/
#define CARTESIAN
//#define COREXY
//#define DELTA
//#define SCARA
/***********************************************************************\

/***********************************************************************\
 ********************** Do not touch this section **********************
 ***********************************************************************/
#if defined(CARTESIAN)
  #include "Configuration_Cartesian.h"
#elif defined(COREXY)
  #include "Configuration_Corexy.h"
#elif defined(DELTA)
  #include "Configuration_Delta.h"
#elif defined(SCARA)
  #include "Configuration_Scara.h"
#endif
/***********************************************************************/

// This defines the number of extruder real or virtual
#define EXTRUDERS 1

// This defines the number of Driver extruder you have and use
#define DRIVER_EXTRUDERS 1

// This is used for single nozzle and multiple extrusion configuration
// Uncomment below to enable (One Hotend)
//#define SINGLENOZZLE

/***********************************************************************
 *********************** Multiextruder MKR4  ***************************
 ***********************************************************************
 *                                                                     *
 * Setting for more extruder width relay system                        *
 * See pins.h for pin command relay                                    *
 *                                                                     *
 ***********************************************************************/
//#define MKR4
//**********************************************************************


/***********************************************************************
 *********************** Multiextruder NPr2  ***************************
 ***********************************************************************
 *                                                                     *
 * Setting fot color meccanism NPr2 by NicolaP (www.3dmakerlab.it)     *
 * Find angle setting by g-Code "M997 Cxxx"                            *
 *                                                                     *
 ***********************************************************************/
//#define NPR2

#define COLOR_STEP {120,25,-65,-155} // CARTER ANGLE
#define COLOR_SLOWRATE 170           // MICROSECOND delay for carter motor routine (Carter Motor Feedrate: upper value-slow feedrate)  
#define COLOR_HOMERATE 4             // FEEDRATE for carter home
#define MOTOR_ANGLE 1.8              // Nema angle for single step 
#define DRIVER_MICROSTEP 4           // Microstep moltiplicator driver (set jumper MS1-2-3) off-on-off 1/4 microstepping.
#define CARTER_MOLTIPLICATOR 14.22   // CARTER MOLTIPLICATOR (gear ratio 13/31-10/31)
//**********************************************************************


// The following define selects which power supply you have. Please choose the one that matches your setup
// 0 = Normal power
// 1 = ATX
// 2 = X-Box 360 203 Watts (the blue wire connected to PS_ON and the red wire to VCC)
#define POWER_SUPPLY 0

// Define this to have the electronics keep the power supply off on startup. If you don't know what this is leave it.
//#define PS_DEFAULT_OFF


//===========================================================================
//============================= Thermal Settings ============================
//===========================================================================

//================================ Thermistor ===============================
//--NORMAL IS 4.7kohm PULLUP!-- 1kohm pullup can be used on hotend sensor, using correct resistor and table
//
//// Temperature sensor settings:
// -2 is thermocouple with MAX6675 (only for sensor 0)
// -1 is thermocouple with AD595
// 0 is not used
// 1 is 100k thermistor - best choice for EPCOS 100k (4.7k pullup)
// 2 is 200k thermistor - ATC Semitec 204GT-2 (4.7k pullup)
// 3 is Mendel-parts thermistor (4.7k pullup)
// 4 is 10k thermistor !! do not use it for a hotend. It gives bad resolution at high temp. !!
// 5 is 100K thermistor - ATC Semitec 104GT-2 (Used in ParCan & J-Head) (4.7k pullup)
// 6 is 100k EPCOS - Not as accurate as table 1 (created using a fluke thermocouple) (4.7k pullup)
// 7 is 100k Honeywell thermistor 135-104LAG-J01 (4.7k pullup)
// 71 is 100k Honeywell thermistor 135-104LAF-J01 (4.7k pullup)
// 8 is 100k 0603 SMD Vishay NTCS0603E3104FXT (4.7k pullup)
// 9 is 100k GE Sensing AL03006-58.2K-97-G1 (4.7k pullup)
// 10 is 100k RS thermistor 198-961 (4.7k pullup)
// 11 is 100k beta 3950 1% thermistor (4.7k pullup)
// 12 is 100k 0603 SMD Vishay NTCS0603E3104FXT (4.7k pullup) (calibrated for Makibox hot bed)
// 13 is 100k Hisens 3950  1% up to 300°C for hotend "Simple ONE " & "Hotend "All In ONE"
// 20 is the PT100 circuit found in the Ultimainboard V2.x
// 60 is 100k Maker's Tool Works Kapton Bed Thermistor beta=3950
//
//    1k ohm pullup tables - This is not normal, you would have to have changed out your 4.7k for 1k
//                          (but gives greater accuracy and more stable PID)
// 51 is 100k thermistor - EPCOS (1k pullup)
// 52 is 200k thermistor - ATC Semitec 204GT-2 (1k pullup)
// 55 is 100k thermistor - ATC Semitec 104GT-2 (Used in ParCan & J-Head) (1k pullup)
//
// 1047 is Pt1000 with 4k7 pullup
// 1010 is Pt1000 with 1k pullup (non standard)
// 147 is Pt100 with 4k7 pullup
// 110 is Pt100 with 1k pullup (non standard)
// 998 and 999 are Dummy Tables. They will ALWAYS read 25°C or the temperature defined below. 
//     Use it for Testing or Development purposes. NEVER for production machine.
//     #define DUMMY_THERMISTOR_998_VALUE 25
//     #define DUMMY_THERMISTOR_999_VALUE 100

#define TEMP_SENSOR_0 1
#define TEMP_SENSOR_1 0
#define TEMP_SENSOR_2 0
#define TEMP_SENSOR_3 0
#define TEMP_SENSOR_BED 1

// This makes temp sensor 1 a redundant sensor for sensor 0. If the temperatures difference between these sensors is to high the print will be aborted.
//#define TEMP_SENSOR_1_AS_REDUNDANT
#define MAX_REDUNDANT_TEMP_SENSOR_DIFF 10 // (degC)

// Actual temperature must be close to target for this long before M109 returns success
#define TEMP_RESIDENCY_TIME 10  // (seconds)
#define TEMP_HYSTERESIS 3       // (degC) range of +/- temperatures considered "close" to the target one
#define TEMP_WINDOW     1       // (degC) Window around target to start the residency timer x degC early.

// The minimal temperature defines the temperature below which the heater will not be enabled It is used
// to check that the wiring to the thermistor is not broken.
// Otherwise this would lead to the heater being powered on all the time.
#define HEATER_0_MINTEMP 5      // (degC)
#define HEATER_1_MINTEMP 5      // (degC)
#define HEATER_2_MINTEMP 5      // (degC)
#define HEATER_3_MINTEMP 5      // (degC)
#define BED_MINTEMP 5      // (degC)

// When temperature exceeds max temp, your heater will be switched off.
// This feature exists to protect your hotend from overheating accidentally, but *NOT* from thermistor short/failure!
// You should use MINTEMP for thermistor short/failure protection.
#define HEATER_0_MAXTEMP 275      // (degC)
#define HEATER_1_MAXTEMP 275      // (degC)
#define HEATER_2_MAXTEMP 275      // (degC)
#define HEATER_3_MAXTEMP 275      // (degC)
#define BED_MAXTEMP 150      // (degC)

// If your bed has low resistance e.g. .6 ohm and throws the fuse you can duty cycle it to reduce the
// average current. The value should be an integer and the heat bed will be turned on for 1 interval of
// HEATER_BED_DUTY_CYCLE_DIVIDER intervals.
//#define HEATER_BED_DUTY_CYCLE_DIVIDER 4

// If you want the M105 heater power reported in watts, define the BED_WATTS, and (shared for all hotend) HOTEND_WATTS
//#define HOTEND_WATTS (12.0*12.0/6.7)  //  P=I^2/R
//#define BED_WATTS (12.0*12.0/1.1)     // P=I^2/R

//===========================================================================
//============================= PID Settings ================================
//===========================================================================
// PID Tuning Guide here: http://reprap.org/wiki/PID_Tuning
// Comment the following line to disable PID and enable bang-bang.
#define PIDTEMP

#define BANG_MAX 255       // limits current to nozzle while in bang-bang mode; 255=full current
#define PID_MAX BANG_MAX   // limits current to nozzle while PID is active (see PID_FUNCTIONAL_RANGE below); 255=full current
//#define PID_DEBUG        // Sends debug data to the serial port.
//#define PID_OPENLOOP 1   // Puts PID in open loop. M104/M140 sets the output power from 0 to PID_MAX
//#define SLOW_PWM_HEATERS // PWM with very low frequency (roughly 0.125Hz=8s) and minimum state time of approximately 1s useful for heaters driven by a relay
// If the temperature difference between the target temperature and the actual temperature
// is more then PID_FUNCTIONAL_RANGE then the PID will be shut off and the heater will be set to min/max.
#define PID_FUNCTIONAL_RANGE 10 // degC
#define PID_INTEGRAL_DRIVE_MAX PID_MAX // Limit for the integral term
#define K1 0.95 // Smoothing factor within the PID

//             HotEnd{HE0,HE1,HE2,HE3}
#define DEFAULT_Kp {14.75,41.51,41.51,41.51}      // Kp for E0, E1, E2, E3
#define DEFAULT_Ki {0.95,7.28,7.28,7.28}      // Ki for E0, E1, E2, E3
#define DEFAULT_Kd {57.27,59.17,59.17,59.17}      // Kd for E0, E1, E2, E3
//===========================================================================


//===========================================================================
//============================= PID > Bed Temperature Control ===============
//===========================================================================
// Select PID or bang-bang with PIDTEMPBED. If bang-bang, BED_LIMIT_SWITCHING will enable hysteresis
//
// Uncomment this to enable PID on the bed. It uses the same frequency PWM as the extruder.
// If your PID_dT is the default, and correct for your hardware/configuration, that means 7.689Hz,
// which is fine for driving a square wave into a resistive load and does not significantly impact you FET heating.
// This also works fine on a Fotek SSR-10DA Solid State Relay into a 250W heater.
// If your configuration is significantly different than this and you don't understand the issues involved, you probably
// shouldn't use bed PID until someone else verifies your hardware works.
// If this is enabled, find your own PID constants below.
//#define PIDTEMPBED
//
//#define BED_LIMIT_SWITCHING

// This sets the max power delivered to the bed, and replaces the HEATER_BED_DUTY_CYCLE_DIVIDER option.
// all forms of bed control obey this (PID, bang-bang, bang-bang with hysteresis)
// setting this to anything other than 255 enables a form of PWM to the bed just like HEATER_BED_DUTY_CYCLE_DIVIDER did,
// so you shouldn't use it unless you are OK with PWM on your bed.  (see the comment on enabling PIDTEMPBED)
#define MAX_BED_POWER 255 // limits duty cycle to bed; 255=full current

//#define PID_BED_DEBUG // Sends debug data to the serial port.
//120v 250W silicone heater into 4mm borosilicate (MendelMax 1.5+)
//from FOPDT model - kp=.39 Tp=405 Tdead=66, Tc set to 79.2, aggressive factor of .15 (vs .1, 1, 10)
#define DEFAULT_bedKp 10
#define DEFAULT_bedKi 1
#define DEFAULT_bedKd 305

//120v 250W silicone heater into 4mm borosilicate (MendelMax 1.5+)
//from pidautotune
//    #define  DEFAULT_bedKp 97.1
//    #define  DEFAULT_bedKi 1.41
//    #define  DEFAULT_bedKd 1675.16

// FIND YOUR OWN: "M303 E-1 C8 S90" to run autotune on the bed at 90 degreesC for 8 cycles.
//===========================================================================


//this prevents dangerous Extruder moves, i.e. if the temperature is under the limit
//can be software-disabled for whatever purposes by
#define PREVENT_DANGEROUS_EXTRUDE
//if PREVENT DANGEROUS EXTRUDE is on, you can still disable (uncomment) very long bits of extrusion separately.
#define PREVENT_LENGTHY_EXTRUDE

#define EXTRUDE_MINTEMP 170 // degC
#define EXTRUDE_MAXLENGTH (X_MAX_LENGTH+Y_MAX_LENGTH) //prevent extrusion of very large distances.


//===========================================================================
//======================== Thermal Runaway Protection =======================
//===========================================================================

/**
 * Thermal Runaway Protection protects your printer from damage and fire if a
 * thermistor falls out or temperature sensors fail in any way.
 *
 * The issue: If a thermistor falls out or a temperature sensor fails,
 * Marlin can no longer sense the actual temperature. Since a disconnected
 * thermistor reads as a low temperature, the firmware will keep the heater on.
 *
 * The solution: Once the temperature reaches the target, start observing.
 * If the temperature stays too far below the target (hysteresis) for too long,
 * the firmware will halt as a safety precaution.
 *
 * Note that because the countdown starts only AFTER the temperature reaches
 * the target, this will not catch a thermistor that is already disconnected
 * when the print starts!
 *
 * To enable for all extruder heaters, uncomment the two defines below:
 */

// Parameters for all extruder heaters
#define THERMAL_RUNAWAY_PROTECTION_PERIOD 40    // in seconds
#define THERMAL_RUNAWAY_PROTECTION_HYSTERESIS 4 // in degree Celsius

// To enable for the bed heater, uncomment the two defines below:

// Parameters for the bed heater
#define THERMAL_RUNAWAY_PROTECTION_BED_PERIOD 20    // in seconds
#define THERMAL_RUNAWAY_PROTECTION_BED_HYSTERESIS 2 // in degree Celsius


//===========================================================================
//============================ User Interfaces ==============================
//===========================================================================

//==============================LCD and SD support=============================

// Choose ONE of these 3 charsets. This has to match your hardware. Ignored for full graphic display.
// To find out what type you have - compile with (test) - upload - click to get the menu. You'll see two typical lines from the upper half of the charset.
// See also documentation/LCDLanguageFont.md
  #define DISPLAY_CHARSET_HD44780_JAPAN        // this is the most common hardware
  //#define DISPLAY_CHARSET_HD44780_WESTERN
  //#define DISPLAY_CHARSET_HD44780_CYRILLIC

//#define ULTRA_LCD  //general LCD support, also 16x2
//#define DOGLCD  // Support for SPI LCD 128x64 (Controller ST7565R graphic Display Family)
#define SDSUPPORT // Enable SD Card Support in Hardware Console
//#define SDSLOW // Use slower SD transfer mode (not normally needed - uncomment if you're getting volume init error)
//#define SD_CHECK_AND_RETRY // Use CRC checks and retries on the SD communication
#define ENCODER_PULSES_PER_STEP 1 // Increase if you have a high resolution encoder
//#define ENCODER_STEPS_PER_MENU_ITEM 5 // Set according to ENCODER_PULSES_PER_STEP or your liking
//#define ULTIMAKERCONTROLLER //as available from the Ultimaker online store.
//#define ULTIPANEL  //the UltiPanel as on Thingiverse
//#define LCD_FEEDBACK_FREQUENCY_DURATION_MS 100 // the duration the buzzer plays the UI feedback sound. ie Screen Click
//#define LCD_FEEDBACK_FREQUENCY_HZ 1000         // this is the tone frequency the buzzer plays when on UI feedback. ie Screen Click
                                                 // 0 to disable buzzer feedback. Test with M300 S<frequency Hz> P<duration ms>
// PanelOne from T3P3 (via RAMPS 1.4 AUX2/AUX3)
// http://reprap.org/wiki/PanelOne
//#define PANEL_ONE

// The MaKr3d Makr-Panel with graphic controller and SD support
// http://reprap.org/wiki/MaKr3d_MaKrPanel
//#define MAKRPANEL

// The Panucatt Devices Viki 2.0 and mini Viki with Graphic LCD
// http://panucatt.com
// ==> REMEMBER TO INSTALL U8glib to your ARDUINO library folder: http://code.google.com/p/u8glib/wiki/u8glib
//#define VIKI2
//#define miniVIKI

// The RepRapDiscount Smart Controller (white PCB)
// http://reprap.org/wiki/RepRapDiscount_Smart_Controller
//#define REPRAP_DISCOUNT_SMART_CONTROLLER

// The GADGETS3D G3D LCD/SD Controller (blue PCB)
// http://reprap.org/wiki/RAMPS_1.3/1.4_GADGETS3D_Shield_with_Panel
//#define G3D_PANEL

// The RepRapDiscount FULL GRAPHIC Smart Controller (quadratic white PCB)
// http://reprap.org/wiki/RepRapDiscount_Full_Graphic_Smart_Controller
//
// ==> REMEMBER TO INSTALL U8glib to your ARDUINO library folder: http://code.google.com/p/u8glib/wiki/u8glib
#define REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER

// The RepRapWorld REPRAPWORLD_KEYPAD v1.1
// http://reprapworld.com/?products_details&products_id=202&cPath=1591_1626
//#define REPRAPWORLD_KEYPAD
//#define REPRAPWORLD_KEYPAD_MOVE_STEP 10.0 // how much should be moved when a key is pressed, eg 10.0 means 10mm per click

// The Elefu RA Board Control Panel
// http://www.elefu.com/index.php?route=product/product&product_id=53
// REMEMBER TO INSTALL LiquidCrystal_I2C.h in your ARDUINO library folder: https://github.com/kiyoshigawa/LiquidCrystal_I2C
//#define RA_CONTROL_PANEL

// I2C Panels
//#define LCD_I2C_SAINSMART_YWROBOT

// PANELOLU2 LCD with status LEDs, separate encoder and click inputs
//#define LCD_I2C_PANELOLU2

// Panucatt VIKI LCD with status LEDs, integrated click & L/R/U/P buttons, separate encoder inputs
//#define LCD_I2C_VIKI

// Shift register panels
// ---------------------
// 2 wire Non-latching LCD SR from:
// https://bitbucket.org/fmalpartida/new-liquidcrystal/wiki/schematics#!shiftregister-connection

//#define SAV_3DLCD

// option for invert rotary switch
//#define INVERT_ROTARY_SWITCH

// Uncomment screen orientation ONLY FOR GRAPHICS DISPLAY
#define LCD_SCREEN_ROT_0
// #define LCD_SCREEN_ROT_90
// #define LCD_SCREEN_ROT_180
// #define LCD_SCREEN_ROT_270

// SPLASH SCREEN duration in millisecond
#define SPLASH_SCREEN_DURATION 2000 // Millisecond

/** Display Voltage Logic Selector on Alligator Board
 0 = Voltage level 3.3V
 1 = Voltage level 5V
 */
#define UI_VOLTAGE_LEVEL 0 // Set 5 o 3.3 V


//============================== Languages UI =========================
// 1  English
// 2  Polish
// 3  French
// 4  German
// 5  Spanish
// 6  Russian
// 7  Italian
// 8  Portuguese
// 9  Finnish
// 10 Aragonese
// 11 Dutch
// 12 Catalan
// 13 Basque-Euskera
// 14 Portuguese (Brazil)

#define LANGUAGE_CHOICE 7


//===========================================================================
//=============================Additional Features===========================
//===========================================================================

//=================================== EEPROM ================================
// The microcontroller can store settings in the EEPROM, e.g. max velocity...
// M500 - stores parameters in EEPROM
// M501 - reads parameters from EEPROM (if you need reset them after you changed them temporarily).
// M502 - reverts to the default "factory settings".  You still need to store them in EEPROM afterwards if you want to.
//define this to enable EEPROM support
#define EEPROM_SETTINGS
#define EEPROM_CHITCHAT
// to disable EEPROM Serial responses and decrease program space by ~1700 byte: comment this out:
// please keep turned on if you can.
//#define DISABLE_M503
//===========================================================================


//==================== Bowden Filament management ===========================
//#define EASY_LOAD

#define BOWDEN_LENGTH 250      // mm
#define LCD_PURGE_LENGTH 10      // mm
#define LCD_RETRACT_LENGTH 5      // mm
#define LCD_PURGE_FEEDRATE 3      // mm/s
#define LCD_RETRACT_FEEDRATE 5      // mm/s
#define LCD_LOAD_FEEDRATE 20      // mm/s
#define LCD_UNLOAD_FEEDRATE 20      // mm/s
//===========================================================================


//====================== Preheat Constants ==================================
#define PLA_PREHEAT_HOTEND_TEMP 200
#define PLA_PREHEAT_HPB_TEMP 20
#define PLA_PREHEAT_FAN_SPEED 255      // Insert Value between 0 and 255

#define ABS_PREHEAT_HOTEND_TEMP 250
#define ABS_PREHEAT_HPB_TEMP 90
#define ABS_PREHEAT_FAN_SPEED 255      // Insert Value between 0 and 255

#define GUM_PREHEAT_HOTEND_TEMP 230
#define GUM_PREHEAT_HPB_TEMP 60
#define GUM_PREHEAT_FAN_SPEED 255      // Insert Value between 0 and 255
//===========================================================================


//============================= R/C Servo support ===========================
// Number of servos
// If you select a configuration below, this will receive a default value and does not need to be set manually
// set it manually if you have more servos than extruders and wish to manually control some
// leaving it defining as 0 will disable the servo subsystem
#define NUM_SERVOS 0      // Servo index starts with 0 for M280 command

// Servo Endstops
// This allows for servo actuated endstops, primary usage is for the Z Axis to eliminate calibration or bed height changes.
// Use M666 command to correct for switch height offset to actual nozzle height. Store that setting with M500.
//
#define SERVO_ENDSTOPS {-1,-1,0}      // Servo index for X, Y, Z. Disable with -1
#define SERVO_ENDSTOP_ANGLES {0,0,0,0,90,0}      // X,Y,Z Axis Extend and Retract angles
//===========================================================================


/**********************************************************************\
 * Support for a filament diameter sensor
 * Also allows adjustment of diameter at print time (vs  at slicing)
 * Single extruder only at this point (extruder 0)
 * 
 * Motherboards
 * 34 - RAMPS1.4 - uses Analog input 5 on the AUX2 connector 
 * 81 - Printrboard - Uses Analog input 2 on the Exp1 connector (version B,C,D,E)
 * 301 - Rambo  - uses Analog input 3
 * Note may require analog pins to be defined for different motherboards
 **********************************************************************/
// Uncomment below to enable
//#define FILAMENT_SENSOR

#define FILAMENT_SENSOR_EXTRUDER_NUM 0      // number of the extruder that has the filament sensor (0,1,2,3)
#define MEASUREMENT_DELAY_CM         14     //measurement delay in cm.  This is the distance from filament sensor to middle of barrel

#define DEFAULT_NOMINAL_FILAMENT_DIA 1.75      // the diameter (in mm) of the filament generally used (3.0 mm or 1.75 mm) - this is then used in the slicer software. Used for sensor reading validation
#define MEASURED_UPPER_LIMIT 2      // limit factor used for sensor reading validation in mm
#define MEASURED_LOWER_LIMIT 1.35      // limit factor for sensor reading validation in mm
#define MAX_MEASUREMENT_DELAY        20     //delay buffer size in bytes (1 byte = 1cm)- limits maximum measurement delay allowable (must be larger than MEASUREMENT_DELAY_CM  and lower number saves RAM)

//defines used in the code
#define DEFAULT_MEASURED_FILAMENT_DIA  DEFAULT_NOMINAL_FILAMENT_DIA  //set measured to nominal initially

//When using an LCD, uncomment the line below to display the Filament sensor data on the last line instead of status.  Status will appear for 5 sec.
//#define FILAMENT_LCD_DISPLAY
//===========================================================================


/**********************************************************************\
 * Support for a current sensor (Hall effect sensor like ACS712) for measure the power consumption
 * Since it's more simple to deal with, we measure the DC current and we assume that POWER_VOLTAGE that comes from your power supply it's almost stable.
 * You have to change the SENSITIVITY with the one that you can find in the datasheet. (in case of ACS712: set to .100 for 20A version or set .066 for 30A version)
 * With this module we measure the Printer power consumption ignoring the Power Supply power consumption, so we consider the EFFICIENCY of our supply to be 100% so without
 * any power dispersion. If you want to approximately add the supply consumption you can decrease the EFFICIENCY to a value less than 100. Eg: 85 is a good value.
 * You can find a better value measuring the AC current with a good multimeter and moltiple it with the mains voltage.
 * MULTIMETER_WATT := MULTIMETER_CURRENT * MAINS_VOLTAGE
 * Now you have a Wattage value that you can compare with the one measured from ACS712.
 * NEW_EFFICENCY := (SENSOR_WATT*EFFICIENCY)/MULTIMETER_WATT
 * For now this feature is to be consider BETA as i'll have to do some accurate test to see the affidability
 **********************************************************************/
// Uncomment below to enable
//#define POWER_CONSUMPTION

#define POWER_VOLTAGE      12.00    //(V) The power supply OUT voltage
#define POWER_ZERO          2.54459 //(V) The /\V coming out from the sensor when no current flow.
#define POWER_SENSITIVITY   0.066   //(V/A) How much increase V for 1A of increase
#define POWER_OFFSET        0.015   //(A) Help to get 0A when no load is connected.
#define POWER_ERROR         3.0     //(%) Ammortize measure error.
#define POWER_EFFICIENCY  100.0     //(%) The power efficency of the power supply

//When using an LCD, uncomment the line below to display the Power consumption sensor data on the last line instead of status.  Status will appear for 5 sec.
//#define POWER_CONSUMPTION_LCD_DISPLAY
//===========================================================================


//=================================== Misc =================================

// Temperature status LEDs that display the hotend and bet temperature.
// If all hotends and bed temperature and temperature setpoint are < 54C then the BLUE led is on.
// Otherwise the RED led is on. There is 1C hysteresis.
//#define TEMP_STAT_LEDS

// Increase the FAN pwm frequency. Removes the PWM noise but increases heating in the FET/Arduino
//#define FAST_PWM_FAN

// Use software PWM to drive the fan, as for the heaters. This uses a very low frequency
// which is not ass annoying as with the hardware PWM. On the other hand, if this frequency
// is too low, you should also increment SOFT_PWM_SCALE.
//#define FAN_SOFT_PWM

// Incrementing this by 1 will double the software PWM frequency,
// affecting heaters, and the fan if FAN_SOFT_PWM is enabled.
// However, control resolution will be halved for each increment;
// at zero value, there are 128 effective control positions.
#define SOFT_PWM_SCALE 0

// M240  Triggers a camera by emulating a Canon RC-1 Remote
// Data from: http://www.doc-diy.net/photo/rc-1_hacked/
//#define PHOTOGRAPH_PIN 23

// SF send wrong arc g-codes when using Arc Point as fillet procedure
//#define SF_ARC_FIX

// Support for the BariCUDA Paste Extruder.
//#define BARICUDA

// Support for BlinkM/CyzRgb
//#define BLINKM

// Enable this option for Toshiba steppers
//#define CONFIG_STEPPERS_TOSHIBA


//===========================================================================
//============================= Filament Runout Sensor ======================
//===========================================================================
//#define FILAMENT_RUNOUT_SENSOR // Uncomment for defining a filament runout sensor such as a mechanical or opto endstop to check the existence of filament
                                 // It is assumed that when logic high = filament available
                                 //                    when logic  low = filament run out
#ifdef FILAMENT_RUNOUT_SENSOR
const bool FILRUNOUT_PIN_INVERTING = false      // Should be uncommented and true or false should assigned
  #define ENDSTOPPULLUP_FIL_RUNOUT            // Uncomment to use internal pullup for filament runout pins if the sensor is defined.
#define FILAMENT_RUNOUT_SCRIPT "M600"      // Script execute when filament run out
#endif

//===========================================================================
//============================= Laser Beam Support ==========================
//===========================================================================
//#define LASERBEAM
//===========================================================================


#include "Configuration_adv.h"
#include "thermistortables.h"
#endif //__CONFIGURATION_H


/* Below you will find the configuration string, that created with Configurator tool online marlinkimbra.it

========== Start configuration string ==========
{
"printer": "prusa_i3_r2",
"baudrates": 250000,
"testmode": "0",
"drivesystems": 0,
"processor": 0,
"motherboards": "BOARD_RAMPS_13_EFB",
"extruders": 1,
"singlenozzle": "0",
"mkr4": "0",
"npr2": "0",
"power": "0",
"defaultpower": "0",
"E0E1pin": 5,
"E0E2pin": 5,
"E1E3pin": 6,
"tempsensor0": "1",
"tempsensor1": "0",
"tempsensor2": "0",
"tempsensor3": "0",
"tempsensorbed": "1",
"temp0pin": "ORIG_TEMP_0_PIN",
"temp1pin": "ORIG_TEMP_1_PIN",
"temp2pin": "ORIG_TEMP_2_PIN",
"temp3pin": "ORIG_TEMP_3_PIN",
"tempbedpin": "ORIG_TEMP_BED_PIN",
"mintemp0": 5,
"mintemp1": 5,
"mintemp2": 5,
"mintemp3": 5,
"mintempbed": 5,
"maxtemp0": 275,
"maxtemp1": 275,
"maxtemp2": 275,
"maxtemp3": 275,
"maxtempbed": 150,
"pidtemp": "1",
"pidkp0": 14.75,
"pidki0": 0.95,
"pidkd0": 57.27,
"pidkp1": 41.51,
"pidki1": 7.28,
"pidkd1": 59.17,
"pidkp2": 41.51,
"pidki2": 7.28,
"pidkd2": 59.17,
"pidkp3": 41.51,
"pidki3": 7.28,
"pidkd3": 59.17,
"pidbedtemp": "0",
"pidbedkp": 10,
"pidbedki": 1,
"pidbedkd": 305,
"dangerousextrude": "1",
"lengthyextrude": "1",
"extrudemintemp": 170,
"autobed": "0",
"zprobingrepeat": "0",
"gridmode": "1",
"gridpoint": 2,
"Zsafehoming": "0",
"ZsafehomingX": 100,
"ZsafehomingY": 100,
"leftprobe": 20,
"rightprobe": 180,
"backprobe": 180,
"frontprobe": 20,
"Xprobe1": 15,
"Yprobe1": 180,
"Xprobe2": 15,
"Yprobe2": 15,
"Xprobe3": 180,
"Yprobe3": 15,
"Xprobeoffset": 0,
"Yprobeoffset": 0,
"Zprobeoffset": -1,
"Zraisebeforehoming": 10,
"Zraisebeforeprobe": 10,
"Zraisebetweenprobe": 10,
"Xmotor": {
  "name": "X motor",
  "step": "ORIG_X_STEP_PIN",
  "dir": "ORIG_X_DIR_PIN",
  "enable": "ORIG_X_ENABLE_PIN"
},
"Ymotor": {
  "name": "Y motor",
  "step": "ORIG_Y_STEP_PIN",
  "dir": "ORIG_Y_DIR_PIN",
  "enable": "ORIG_Y_ENABLE_PIN"
},
"Zmotor": {
  "name": "Z motor",
  "step": "ORIG_Z_STEP_PIN",
  "dir": "ORIG_Z_DIR_PIN",
  "enable": "ORIG_Z_ENABLE_PIN"
},
"E0motor": {
  "name": "Extruder 0",
  "step": "ORIG_E0_STEP_PIN",
  "dir": "ORIG_E0_DIR_PIN",
  "enable": "ORIG_E0_ENABLE_PIN"
},
"E1motor": {
  "name": "Extruder 1",
  "step": "ORIG_E1_STEP_PIN",
  "dir": "ORIG_E1_DIR_PIN",
  "enable": "ORIG_E1_ENABLE_PIN"
},
"E2motor": {
  "name": "Extruder 2",
  "step": "ORIG_E2_STEP_PIN",
  "dir": "ORIG_E2_DIR_PIN",
  "enable": "ORIG_E2_ENABLE_PIN"
},
"E3motor": {
  "name": "Extruder 3",
  "step": "ORIG_E3_STEP_PIN",
  "dir": "ORIG_E3_DIR_PIN",
  "enable": "ORIG_E3_ENABLE_PIN"
},
"Xminpos": -10,
"Xmaxpos": 240,
"Yminpos": -58,
"Ymaxpos": 270,
"Zminpos": 0,
"Zmaxpos": 240,
"maxXYjerk": 15,
"maxZjerk": 0.4,
"maxEjerk": 5,
"defaultaccelleration": 2000,
"defaultretractionaccelleration": 4000,
"defaulttravelaccelleration": 3000,
"deltasegmentpersecond": 200,
"deltadiagonalrod": 220,
"deltasmoothrodoffset": 145,
"deltaeffectoroffset": 20,
"deltacarriageoffset": 20,
"deltaprinterradius": 70,
"deltaheight": 210,
"deltaautoprecision": 0.1,
"deltaautogrid": 20,
"deltaXprobeoffset": 0,
"deltaYprobeoffset": 0,
"deltaZprobeoffset": -10,
"deltaXdeploystart": 0,
"deltaYdeploystart": 0,
"deltaZdeploystart": 30,
"deltaXdeployend": 0,
"deltaYdeployend": 0,
"deltaZdeployend": 0,
"deltaXretractstart": 0,
"deltaYretractstart": 0,
"deltaZretractstart": 30,
"deltaXretractend": 0,
"deltaYretractend": 0,
"deltaZretractend": 0,
"Xmicrostep": 16,
"Ymicrostep": 16,
"Zmicrostep": 16,
"Emicrostep": 16,
"Xcurrent": 1000,
"Ycurrent": 1000,
"Zcurrent": 1000,
"Ecurrent": 1000,
"Xstepspermm": 160,
"Ystepspermm": 160,
"Zstepspermm": 6400,
"E0stepspermm": 1240,
"E1stepspermm": 625,
"E2stepspermm": 625,
"E3stepspermm": 625,
"Xmaxspeed": 1000,
"Ymaxspeed": 1000,
"Zmaxspeed": 3,
"E0maxspeed": 50,
"E1maxspeed": 100,
"E2maxspeed": 100,
"E3maxspeed": 100,
"E0retractionspeed": 100,
"E1retractionspeed": 150,
"E2retractionspeed": 150,
"E3retractionspeed": 150,
"Xhomingspeed": 100,
"Yhomingspeed": 100,
"Zhomingspeed": 3,
"Xmaxacceleration": 5000,
"Ymaxacceleration": 5000,
"Zmaxacceleration": 50,
"E0maxacceleration": 5000,
"E1maxacceleration": 3000,
"E2maxacceleration": 3000,
"E3maxacceleration": 3000,
"Xinvert": 0,
"Yinvert": "1",
"Zinvert": 0,
"E0invert": "1",
"E1invert": 0,
"E2invert": 0,
"E3invert": 0,
"Xinvertenable": 0,
"Yinvertenable": 0,
"Zinvertenable": 0,
"Einvertenable": 0,
"disableX": 0,
"disableY": "1",
"disableZ": "1",
"disableE": 0,
"Xhoming": 0,
"Yhoming": 0,
"Zhoming": 0,
"Ehoming": 0,
"Xminendstop": "0",
"Xmaxendstop": "0",
"Yminendstop": "0",
"Ymaxendstop": "0",
"Zminendstop": "0",
"Zmaxendstop": "0",
"Eminendstop": "0",
"Zprobeendstop": "0",
"E0coolerpin": -1,
"E1coolerpin": -1,
"E2coolerpin": -1,
"E3coolerpin": -1,
"Ecoolerspeed": 255,
"Ecoolertemp": 50,
"fanpin": "ORIG_FAN_PIN",
"sdsupport": "1",
"eeprom": "1",
"eepromchitchat": "1",
"laserbeam": "0",
"laserpwrpin": 42,
"laserttlpin": 44,
"toshiba": "0",
"filamentrunout": "0",
"filrunoutpin": -1,
"filamentrunoutsensor": "0",
"filamentrunoutscript": "M600",
"servos": "0",
"numservos": 0,
"Xservo": "-1",
"Yservo": "-1",
"Zservo": "0",
"angleextendservosX": 0,
"angleretractservosX": 0,
"angleextendservosY": 0,
"angleretractservosY": 0,
"angleextendservosZ": 90,
"angleretractservosZ": 0,
"displays": 5,
"invertrotaryswitch": 0,
"uilanguages": 7,
"uiprintername": "R2",
"easyload": 0,
"bowdenlenght": 250,
"lcdpurgelenght": 10,
"lcdretractlenght": 5,
"lcdpurgefeedrate": 3,
"lcdretractfeedrate": 5,
"lcdloadfeedrate": 20,
"lcdunloadfeedrate": 20,
"lcdprogressbar": 0,
"lcdprogressbarbartime": 3,
"lcdprogressbarmsgtime": 1,
"lcdprogressbarmsgexpire": 0,
"plahotendtemp": 200,
"plabedtemp": 20,
"plafanspeed": 255,
"abshotendtemp": 250,
"absbedtemp": 90,
"absfanspeed": 255,
"gumhotendtemp": 230,
"gumbedtemp": 60,
"gumfanspeed": 255,
"filamentsensor": "0",
"filamentsensorpin": -1,
"filamentsensorlcd": "0",
"filamentsensorextruder": 0,
"filamentsensordia": 1.75,
"filamentsensormaxdia": 2,
"filamentsensormindia": 1.35,
"filamentswitch": "0",
"pausepin": 19,
"disableE0": "1",
"sdsetting": "1"
}
========== End configuration string ==========
*/
