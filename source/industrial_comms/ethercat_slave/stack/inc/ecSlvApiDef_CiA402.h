/*!
* \file ecSlvApiDef_CiA402.h
*
* \brief
* CiA402 defines.
*
* \author
* KUNBUS GmbH
*
* \date
* 2022-08-19
*
* \copyright
* Copyright (c) 2022, KUNBUS GmbH<br /><br />
* All rights reserved.<br />
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:<br />
* <ol>
* <li>Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.</li>
* <li>Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.</li>
* <li>Neither the name of the copyright holder nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission.</li>
* </ol>
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#if !(defined __ECSLVAPIDEF402_H__)
#define __ECSLVAPIDEF402_H__		1



#if (defined __cplusplus)
extern "C" {
#endif


/*-----------------------------------------------------------------------------------------
------
------    Defines and Types
------
-----------------------------------------------------------------------------------------*/

/*---------------------------------------------
-    CiA 402
-----------------------------------------------*/

/**
* \addtogroup CiA402 CiA402 Codes
* @{
*/
/*---------------------------------------------
-    ControlWord Commands Mask (IEC61800_184e)
-----------------------------------------------*/
#define CONTROLWORD_COMMAND_SHUTDOWN_MASK                   0x0087 /**< \brief Shutdown command mask*/
/*ECATCHANGE_START(V5.12) CIA402 2*/
#define CONTROLWORD_COMMAND_SWITCHON_MASK                   0x008F /**< \brief Switch on command mask*/
/*ECATCHANGE_END(V5.12) CIA402 2*/
#define CONTROLWORD_COMMAND_SWITCHON_ENABLEOPERATION_MASK   0x008F /**< \brief Switch on & Enable command mask*/
#define CONTROLWORD_COMMAND_DISABLEVOLTAGE_MASK             0x0082 /**< \brief Disable voltage command mask*/
#define CONTROLWORD_COMMAND_QUICKSTOP_MASK                  0x0086 /**< \brief Quickstop command mask*/
#define CONTROLWORD_COMMAND_DISABLEOPERATION_MASK           0x008F /**< \brief Disable operation command mask*/
#define CONTROLWORD_COMMAND_ENABLEOPERATION_MASK            0x008F /**< \brief Enable operation command mask*/
#define CONTROLWORD_COMMAND_FAULTRESET_MASK                 0x0080 /**< \brief Fault reset command mask*/


/*---------------------------------------------
-    ControlWord Commands (IEC61800_184e)
-----------------------------------------------*/
#define CONTROLWORD_COMMAND_SHUTDOWN                        0x0006 /**< \brief Shutdown command*/
#define CONTROLWORD_COMMAND_SWITCHON                        0x0007 /**< \brief Switch on command*/
#define CONTROLWORD_COMMAND_SWITCHON_ENABLEOPERATION        0x000F /**< \brief Switch on & Enable command*/
#define CONTROLWORD_COMMAND_DISABLEVOLTAGE                  0x0000 /**< \brief Disable voltage command*/
#define CONTROLWORD_COMMAND_QUICKSTOP                       0x0002 /**< \brief Quickstop command*/
#define CONTROLWORD_COMMAND_DISABLEOPERATION                0x0007 /**< \brief Disable operation command*/
#define CONTROLWORD_COMMAND_ENABLEOPERATION                 0x000F /**< \brief Enable operation command*/
#define CONTROLWORD_COMMAND_FAULTRESET                      0x0080 /**< \brief Fault reset command*/


/*---------------------------------------------
-    StatusWord Masks and Flags
-----------------------------------------------*/
#define STATUSWORD_STATE_MASK                               0x006F /**< \brief State mask*/
#define STATUSWORD_VOLTAGE_ENABLED                          0x0010 /**< \brief Indicate high voltage enabled*/
#define STATUSWORD_WARNING                                  0x0080 /**< \brief Warning active*/
#define STATUSWORD_MANUFACTORSPECIFIC                       0x0100 /**< \brief Manufacturer specific*/
#define STATUSWORD_INTERNAL_LIMIT                           0x0800 /**< \brief Internal limit*/
#define STATUSWORD_REMOTE                                   0x0200 /**< \brief Set if the control word is processed*/
#define STATUSWORD_TARGET_REACHED                           0x0400 /**< \brief Target reached*/
#define STATUSWORD_INTERNALLIMITACTIVE                      0x0800 /**< \brief Internal limit active*/
#define STATUSWORD_DRIVE_FOLLOWS_COMMAND                    0x1000 /**< \brief Drive follows command (used in cyclic synchronous modes)*/


/*---------------------------------------------
-    StatusWord
-----------------------------------------------*/
#define STATUSWORD_STATE_NOTREADYTOSWITCHON                 0x0000 /**< \brief Not ready to switch on*/
/* ECATCHANGE_START(V5.12) CIA402 1*/
#define STATUSWORD_STATE_SWITCHEDONDISABLED                 0x0040 /**< \brief Switched on but disabled*/
/* ECATCHANGE_END(V5.12) CIA402 1*/
#define STATUSWORD_STATE_READYTOSWITCHON                    0x0021 /**< \brief Ready to switch on*/
#define STATUSWORD_STATE_SWITCHEDON                         0x0023 /**< \brief Switched on*/
#define STATUSWORD_STATE_OPERATIONENABLED                   0x0027 /**< \brief Operation enabled*/
#define STATUSWORD_STATE_QUICKSTOPACTIVE                    0x0007 /**< \brief Quickstop active*/
#define STATUSWORD_STATE_FAULTREACTIONACTIVE                0x000F /**< \brief Fault reaction active*/
#define STATUSWORD_STATE_FAULT                              0x0008 /**< \brief Fault state*/


/*---------------------------------------------
-    CiA402 State machine
-----------------------------------------------*/
#define STATE_NOT_READY_TO_SWITCH_ON                        0x0001 /**< \brief Not ready to switch on (optional)*/
#define STATE_SWITCH_ON_DISABLED                            0x0002 /**< \brief Switch on but disabled (optional)*/
#define STATE_READY_TO_SWITCH_ON                            0x0004 /**< \brief Ready to switch on (mandatory)*/
#define STATE_SWITCHED_ON                                   0x0008 /**< \brief Switch on (mandatory)*/
#define STATE_OPERATION_ENABLED                             0x0010 /**< \brief Operation enabled (mandatory)*/
#define STATE_QUICK_STOP_ACTIVE                             0x0020 /**< \brief Quick stop active (optional)*/
#define STATE_FAULT_REACTION_ACTIVE                         0x0040 /**< \brief Fault reaction active (mandatory)*/
#define STATE_FAULT                                         0x0080 /**< \brief Fault state (mandatory)*/


/*---------------------------------------------
-    CiA402 Modes of Operation (object 0x6060) (IEC61800_184e)
-----------------------------------------------*/
// -128 to -1 Manufacturer-specific operation modes
#define NO_MODE                                             0     /**< \brief No mode*/
#define PROFILE_POSITION_MODE                               1     /**< \brief Position Profile mode*/
#define VELOCITY_MODE                                       2     /**< \brief Velocity mode*/
#define PROFILE_VELOCITY_MOCE                               3     /**< \brief Velocity Profile mode*/
#define PROFILE_TORQUE_MODE                                 4     /**< \brief Torque Profile mode*/
//5 reserved
#define HOMING_MODE                                         6     /**< \brief Homing mode*/
#define INTERPOLATION_POSITION_MODE                         7     /**< \brief Interpolation Position mode*/
#define CYCLIC_SYNC_POSITION_MODE                           8     /**< \brief Cyclic Synchronous Position mode*/
#define CYCLIC_SYNC_VELOCITY_MODE                           9     /**< \brief Cyclic Synchronous Velocity mode*/
#define CYCLIC_SYNC_TORQUE_MODE                             10    /**< \brief Cyclic Synchronous Torque mode*/
//+11 to +127 reserved


/***************************************
 CiA402 Error Codes (object 0x603F) (IEC61800_184e)
 ***************************************/
#define ERROR_SHORT_CIRCUIT_EARTH_LEAKAGE_INPUT             0x2110 /**< \brief Short circuit/earth leakage (input)*/
#define ERROR_EARTH_LEAKAGE_INPUT                           0x2120 /**< \brief Earth leakage (input)*/
#define ERROR_EARTH_LEAKAGE_PHASE_L1                        0x2121 /**< \brief Earth leakage phase L1*/
#define ERROR_EARTH_LEAKAGE_PHASE_L2                        0x2122 /**< \brief Earth leakage phase L2*/
#define ERROR_EARTH_LEAKAGE_PHASE_L3                        0x2123 /**< \brief Earth leakage phase L3*/
#define ERROR_SHORT_CIRCUIT_INPUT                           0x2130 /**< \brief Short circuit (input)*/
#define ERROR_SHORT_CIRCUIT_PHASES_L1_L2                    0x2131 /**< \brief Short circuit phases L1-L2*/
#define ERROR_SHORT_CIRCUIT_PHASES_L2_L3                    0x2132 /**< \brief Short circuit phases L2-L3*/
#define ERROR_SHORT_CIRCUIT_PHASES_L3_L1                    0x2133 /**< \brief Short circuit phases L3-L1*/
#define ERROR_INTERNAL_CURRENT_NO1                          0x2211 /**< \brief Internal current no 1*/
#define ERROR_INTERNAL_CURRENT_NO2                          0x2212 /**< \brief Internal current no 2*/
#define ERROR_OVER_CURRENT_IN_RAMP_FUNCTION                 0x2213 /**< \brief Over-current in ramp function*/
#define ERROR_OVER_CURRENT_IN_THE_SEQUENCE                  0x2214 /**< \brief Over-current in the sequence*/
#define ERROR_CONTINUOUS_OVER_CURRENT_DEVICE_INTERNAL       0x2220 /**< \brief Continuous over current (device internal)*/
#define ERROR_CONTINUOUS_OVER_CURRENT_DEVICE_INTERNAL_NO1   0x2221 /**< \brief Continuous over current no 1*/
#define ERROR_CONTINUOUS_OVER_CURRENT_DEVICE_INTERNAL_NO2   0x2222 /**< \brief Continuous over current no 2*/
#define ERROR_SHORT_CIRCUIT_EARTH_LEAKAGE_DEVICE_INTERNAL   0x2230 /**< \brief Short circuit/earth leakage (device internal)*/
#define ERROR_EARTH_LEAKAGE_DEVICE_INTERNAL                 0x2240 /**< \brief Earth leakage (device internal)*/
#define ERROR_SHORT_CIRCUIT_DEVICE_INTERNAL                 0x2250 /**< \brief Short circuit (device internal)*/
#define ERROR_CONTINUOUS_OVER_CURRENT                       0x2310 /**< \brief Continuous over current*/
#define ERROR_CONTINUOUS_OVER_CURRENT_NO1                   0x2311 /**< \brief Continuous over current no 1*/
#define ERROR_CONTINUOUS_OVER_CURRENT_NO2                   0x2312 /**< \brief Continuous over current no 2*/
#define ERROR_SHORT_CIRCUIT_EARTH_LEAKAGE_MOTOR_SIDE        0x2320 /**< \brief Short circuit/earth leakage (motor-side)*/
#define ERROR_EARTH_LEAKAGE_MOTOR_SIDE                      0x2330 /**< \brief Earth leakage (motor-side)*/
#define ERROR_EARTH_LEAKAGE_PHASE_U                         0x2331 /**< \brief Earth leakage phase U*/
#define ERROR_EARTH_LEAKAGE_PHASE_V                         0x2332 /**< \brief Earth leakage phase V*/
#define ERROR_EARTH_LEAKAGE_PHASE_W                         0x2333 /**< \brief Earth leakage phase W*/
#define ERROR_SHORT_CIRCUIT_MOTOR_SIDE                      0x2340 /**< \brief Short circuit (motor-side)*/
#define ERROR_SHORT_CIRCUIT_PHASES_U_V                      0x2341 /**< \brief Short circuit phases U-V*/
#define ERROR_EARTH_LEAKAGE_PHASE_V_W                       0x2342 /**< \brief Earth leakage phase V-W*/
#define ERROR_EARTH_LEAKAGE_PHASE_W_U                       0x2343 /**< \brief Earth leakage phase W-U*/
#define ERROR_LOAD_LEVEL_FAULT_I2T_THERMAL_STATE            0x2350 /**< \brief Load level fault (I2t, thermal state)*/
#define ERROR_LOAD_LEVEL_WARNING_I2T_THERMAL_STATE          0x2351 /**< \brief Load level warning (I2t, thermal state)*/
#define ERROR_MAINS_OVER_VOLTAGE                            0x3110 /**< \brief Mains over-voltage*/
#define ERROR_MAINS_OVER_VOLTAGE_PHASE_L1                   0x3111 /**< \brief Mains over-voltage phase L1*/
#define ERROR_MAINS_OVER_VOLTAGE_PHASE_L2                   0x3112 /**< \brief Mains over-voltage phase L2 */
#define ERROR_MAINS_OVER_VOLTAGE_PHASE_L3                   0x3113 /**< \brief Mains over-voltage phase L3*/
#define ERROR_MAINS_UNDER_VOLTAGE                           0x3120 /**< \brief Mains under-voltage*/
#define ERROR_MAINS_UNDER_VOLTAGE_PHASE_L1                  0x3121 /**< \brief Mains under-voltage phase L1*/
#define ERROR_MAINS_UNDER_VOLTAGE_PHASE_L2                  0x3122 /**< \brief Mains under-voltage phase L2*/
#define ERROR_MAINS_UNDER_VOLTAGE_PHASE_L3                  0x3123 /**< \brief Mains under-voltage phase L3*/
#define ERROR_PHASE_FAILURE                                 0x3130 /**< \brief Phase failure*/
#define ERROR_PHASE_FAILURE_L1                              0x3131 /**< \brief Phase failure L1*/
#define ERROR_PHASE_FAILURE_L2                              0x3132 /**< \brief Phase failure L2*/
#define ERROR_PHASE_FAILURE_L3                              0x3133 /**< \brief Phase failure L3*/
#define ERROR_PHASE_SEQUENCE                                0x3134 /**< \brief Phase sequence*/
#define ERROR_MAINS_FREQUENCY                               0x3140 /**< \brief Mains frequency*/
#define ERROR_MAINS_FREQUENCY_TOO_GREAT                     0x3141 /**< \brief Mains frequency too great*/
#define ERROR_MAINS_FREQUENCY_TOO_SMALL                     0x3142 /**< \brief Mains frequency too small*/
#define ERROR_DC_LINK_OVER_VOLTAGE                          0x3210 /**< \brief DC link over-voltage*/
#define ERROR_OVER_VOLTAGE_NO_1                             0x3211 /**< \brief Over-voltage no  1*/
#define ERROR_OVER_VOLTAGE_NO_2                             0x3212 /**< \brief Over voltage no  2 */
#define ERROR_DC_LINK_UNDER_VOLTAGE                         0x3220 /**< \brief DC link under-voltage*/
#define ERROR_UNDER_VOLTAGE_NO_1                            0x3221 /**< \brief Under-voltage no  1*/
#define ERROR_UNDER_VOLTAGE_NO_2                            0x3222 /**< \brief Under-voltage no  2*/
#define ERROR_LOAD_ERROR                                    0x3230 /**< \brief Load error*/
#define ERROR_OUTPUT_OVER_VOLTAGE                           0x3310 /**< \brief Output over-voltage*/
#define ERROR_OUTPUT_OVER_VOLTAGE_PHASE_U                   0x3311 /**< \brief Output over-voltage phase U*/
#define ERROR_OUTPUT_OVER_VOLTAGE_PHASE_V                   0x3312 /**< \brief Output over-voltage phase V*/
#define ERROR_OUTPUT_OVER_VOLTAGE_PHASE_W                   0x3313 /**< \brief Output over-voltage phase W*/
#define ERROR_ARMATURE_CIRCUIT                              0x3320 /**< \brief Armature circuit*/
#define ERROR_ARMATURE_CIRCUIT_INTERRUPTED                  0x3321 /**< \brief Armature circuit interrupted*/
#define ERROR_FIELD_CIRCUIT                                 0x3330 /**< \brief Field circuit error */
#define ERROR_FIELD_CIRCUIT_INTERRUPTED                     0x3331 /**< \brief Field circuit interrupted*/
#define ERROR_EXCESS_AMBIENT_TEMPERATURE                    0x4110 /**< \brief Excess ambient temperature*/
#define ERROR_TOO_LOW_AMBIENT_TEMPERATURE                   0x4120 /**< \brief Too low ambient temperature*/
#define ERROR_TEMPERATURE_SUPPLY_AIR                        0x4130 /**< \brief Temperature supply air*/
#define ERROR_TEMPERATURE_AIR_OUTLET                        0x4140 /**< \brief Temperature air outlet*/
#define ERROR_EXCESS_TEMPERATURE_DEVICE                     0x4210 /**< \brief Excess temperature device*/
#define ERROR_TOO_LOW_TEMPERATURE_DEVICE                    0x4220 /**< \brief Too low temperature device*/
#define ERROR_TEMPERATURE_DRIVE                             0x4300 /**< \brief Temperature drive error*/
#define ERROR_EXCESS_TEMPERATURE_DRIVE                      0x4310 /**< \brief Excess temperature drive error*/
#define ERROR_TOO_LOW_TEMPERATURE_DRIVE                     0x4320 /**< \brief Too low temperature drive error*/
#define ERROR_TEMPERATURE_SUPPLY                            0x4400 /**< \brief Temperature supply error*/
#define ERROR_EXCESS_TEMPERATURE_SUPPLY                     0x4410 /**< \brief Excess temperature supply*/
#define ERROR_TOO_LOW_TEMPERATURE_SUPPLY                    0x4420 /**< \brief Too low temperature supply*/
#define ERROR_SUPPLY_ERROR                                  0x5100 /**< \brief Supply error*/
#define ERROR_SUPPLY_LOW_VOLTAGE                            0x5110 /**< \brief Supply low voltage*/
#define ERROR_U1_SUPPLY_15V                                 0x5111 /**< \brief U1 = supply +15V/-15V*/
#define ERROR_U2_SUPPLY_24_V                                0x5112 /**< \brief U2 = supply +24 V*/
#define ERROR_U3_SUPPLY_5_V                                 0x5113 /**< \brief U3 = supply +5 V*/
#define ERROR_U4_MANUFACTURER_SPECIFIC                      0x5114 /**< \brief U4 = manufacturer-specific error*/
#define ERROR_U5_MANUFACTURER_SPECIFIC                      0x5115 /**< \brief U5 = manufacturer-specific error*/
#define ERROR_U6_MANUFACTURER_SPECIFIC                      0x5116 /**< \brief U6 = manufacturer-specific error*/
#define ERROR_U7_MANUFACTURER_SPECIFIC                      0x5117 /**< \brief U7 = manufacturer-specific error*/
#define ERROR_U8_MANUFACTURER_SPECIFIC                      0x5118 /**< \brief U8 = manufacturer-specific error*/
#define ERROR_U9_MANUFACTURER_SPECIFIC                      0x5119 /**< \brief U9 = manufacturer-specific error*/
#define ERROR_SUPPLY_INTERMEDIATE_CIRCUIT                   0x5120 /**< \brief Supply intermediate circuit*/
//#define ERROR_CONTROL                                     0x5200
#define ERROR_CONTROL_MEASUREMENT_CIRCUIT                   0x5210 /**< \brief Measurement circuit*/
#define ERROR_CONTROL_COMPUTING_CIRCUIT                     0x5220 /**< \brief Computing circuit*/
#define ERROR_OPERATING_UNIT                                0x5300 /**< \brief Operating unit error*/
#define ERROR_POWER_SECTION                                 0x5400 /**< \brief Power section error*/
#define ERROR_OUTPUT_STAGES                                 0x5410 /**< \brief Output stages error*/
#define ERROR_CHOPPER                                       0x5420 /**< \brief Chopper error*/
#define ERROR_INPUT_STAGES                                  0x5430 /**< \brief Input stages error*/
#define ERROR_CONTACTS_ERROR                                0x5440 /**< \brief Contacts error*/
#define ERROR_CONTACT_1_MANUFACTURER_SPECIFIC               0x5441 /**< \brief Contact 1 = manufacturer-specific error*/
#define ERROR_CONTACT_2_MANUFACTURER_SPECIFIC               0x5442 /**< \brief Contact 2 = manufacturer-specific error*/
#define ERROR_CONTACT_3_MANUFACTURER_SPECIFIC               0x5443 /**< \brief Contact 3 = manufacturer-specific error*/
#define ERROR_CONTACT_4_MANUFACTURER_SPECIFIC               0x5444 /**< \brief Contact 4 = manufacturer-specific error*/
#define ERROR_CONTACT_5_MANUFACTURER_SPECIFIC               0x5445 /**< \brief Contact 5 = manufacturer-specific error*/
#define ERROR_FUSES_ERROR                                   0x5450 /**< \brief Fuses error*/
#define ERROR_S1_L1                                         0x5451 /**< \brief S1 = l1 error*/
#define ERROR_S2_L2                                         0x5452 /**< \brief S2 = l2 error*/
#define ERROR_S3_L3                                         0x5453 /**< \brief S3 = l3 error*/
#define ERROR_S4_MANUFACTURER_SPECIFIC                      0x5454 /**< \brief S4 = manufacturer-specific error*/
#define ERROR_S5_MANUFACTURER_SPECIFIC                      0x5455 /**< \brief S5 = manufacturer-specific error*/
#define ERROR_S6_MANUFACTURER_SPECIFIC                      0x5456 /**< \brief S6 = manufacturer-specific error*/
#define ERROR_S7_MANUFACTURER_SPECIFIC                      0x5457 /**< \brief S7 = manufacturer-specific error*/
#define ERROR_S8_MANUFACTURER_SPECIFIC                      0x5458 /**< \brief S8 = manufacturer-specific error*/
#define ERROR_S9_MANUFACTURER_SPECIFIC                      0x5459 /**< \brief S9 = manufacturer-specific error*/
#define ERROR_HARDWARE_MEMORY                               0x5500 /**< \brief Hardware memory error*/
#define ERROR_RAM                                           0x5510 /**< \brief RAM error*/
#define ERROR_ROM_EPROM                                     0x5520 /**< \brief ROM/EPROM error*/
#define ERROR_EEPROM                                        0x5530 /**< \brief EEPROM error*/
#define ERROR_SOFTWARE_RESET_WATCHDOG                       0x6010 /**< \brief Software reset (watchdog)*/
//0x6301_TO_0x630F        ERROR_DATA_RECORD_NO_1_TO_NO_15
#define ERROR_LOSS_OF_PARAMETERS                            0x6310 /**< \brief Loss of parameters*/
#define ERROR_PARAMETER_ERROR                               0x6320 /**< \brief Parameter error*/
#define ERROR_POWER_ERROR                                   0x7100 /**< \brief Power error*/
#define ERROR_BRAKE_CHOPPER                                 0x7110 /**< \brief Brake chopper*/
#define ERROR_FAILURE_BRAKE_CHOPPER                         0x7111 /**< \brief Failure brake chopper*/
#define ERROR_OVER_CURRENT_BRAKE_CHOPPER                    0x7112 /**< \brief Over current brake chopper*/
#define ERROR_PROTECTIVE_CIRCUIT_BRAKE_CHOPPER              0x7113 /**< \brief Protective circuit brake chopper error*/
#define ERROR_MOTOR_ERROR                                   0x7120 /**< \brief Motor error*/
#define ERROR_MOTOR_BLOCKED                                 0x7121 /**< \brief Motor blocked error*/
#define ERROR_MOTOR_ERROR_OR_COMMUTATION_MALFUNC            0x7122 /**< \brief Motor error or commutation malfunc */
#define ERROR_MOTOR_TILTED                                  0x7123 /**< \brief Motor tilted*/
#define ERROR_MEASUREMENT_CIRCUIT                           0x7200 /**< \brief Measurement circuit*/
#define ERROR_SENSOR_ERROR                                  0x7300 /**< \brief Sensor error*/
#define ERROR_TACHO_FAULT                                   0x7301 /**< \brief Tacho fault*/
#define ERROR_TACHO_WRONG_POLARITY                          0x7302 /**< \brief Tacho wrong polarity*/
#define ERROR_RESOLVER_1_FAULT                              0x7303 /**< \brief Resolver 1 fault*/
#define ERROR_RESOLVER_2_FAULT                              0x7304 /**< \brief Resolver 2 fault*/
#define ERROR_INCREMENTAL_SENSOR_1_FAULT                    0x7305 /**< \brief Incremental sensor 1 fault*/
#define ERROR_INCREMENTAL_SENSOR_2_FAULT                    0x7306 /**< \brief Incremental sensor 2 fault*/
#define ERROR_INCREMENTAL_SENSOR_3_FAULT                    0x7307 /**< \brief Incremental sensor 3 fault*/
#define ERROR_SPEED                                         0x7310 /**< \brief Speed error*/
#define ERROR_POSITION                                      0x7320 /**< \brief Position error*/
#define ERROR_COMPUTATION_CIRCUIT                           0x7400 /**< \brief Computation circuit*/
#define ERROR_COMMUNICATION                                 0x7500 /**< \brief Communication error*/
#define ERROR_SERIAL_INTERFACE_NO_1                         0x7510 /**< \brief Serial interface no  1 error*/
#define ERROR_SERIAL_INTERFACE_NO_2                         0x7520 /**< \brief Serial interface no  2 error*/
#define ERROR_DATA_STORAGE_EXTERNAL                         0x7600 /**< \brief Data storage (external) error*/
#define ERROR_TORQUE_CONTROL                                0x8300 /**< \brief Torque control error*/
#define ERROR_EXCESS_TORQUE                                 0x8311 /**< \brief Excess torque error*/
#define ERROR_DIFFICULT_START_UP                            0x8312 /**< \brief Difficult start up error*/
#define ERROR_STANDSTILL_TORQUE                             0x8313 /**< \brief Standstill torque error*/
#define ERROR_INSUFFICIENT_TORQUE                           0x8321 /**< \brief Insufficient torque error*/
#define ERROR_TORQUE_FAULT                                  0x8331 /**< \brief Torque fault*/
#define ERROR_VELOCITY_SPEED_CONTROLLER                     0x8400 /**< \brief Velocity speed controller*/
#define ERROR_POSITION_CONTROLLER                           0x8500 /**< \brief Position controller*/
#define ERROR_POSITIONING_CONTROLLER                        0x8600 /**< \brief Positioning controller*/
#define ERROR_FOLLOWING_ERROR                               0x8611 /**< \brief Following error*/
#define ERROR_REFERENCE_LIMIT                               0x8612 /**< \brief Reference limit*/
#define ERROR_SYNC_CONTROLLER                               0x8700 /**< \brief Sync controller*/
#define ERROR_WINDING_CONTROLLER                            0x8800 /**< \brief Winding controller*/
#define ERROR_PROCESS_DATA_MONITORING                       0x8900 /**< \brief Process data monitoring*/
//#define ERROR_CONTROL                                     0x8A00
#define ERROR_DECELERATION                                  0xF001 /**< \brief Deceleration error*/
#define ERROR_SUB_SYNCHRONOUS_RUN                           0xF002 /**< \brief Sub-synchronous run error*/
#define ERROR_STROKE_OPERATION                              0xF003 /**< \brief Stroke operation error*/
//#define ERROR_CONTROL                                     0xF004
//0xFF00_TO_0xFFFF        MANUFACTURER_SPECIFIC

/*---------------------------------------------
-    CiA402 generic error option code values
        Note: Not all values are valid for each error option code.
        A detailed description of the option code values are listed in the specification IEC 61800-7-200
        0x605B    : action in state transition 8
        0x605C    : action in state transition 5
-----------------------------------------------*/
#define DISABLE_DRIVE                                       0      /**< \brief Disable drive (options: 0x605B; 0x605C; 0x605E)*/
#define SLOW_DOWN_RAMP                                      1      /**< \brief Slow down ramp (options: 0x605B; 0x605C; 0x605E)*/
#define QUICKSTOP_RAMP                                      2      /**< \brief Quick stop ramp (options: 0x605E)*/
#define STOP_ON_CURRENT_LIMIT                               3      /**< \brief Stop on current limit (options: 0x605E)*/
#define STOP_ON_VOLTAGE_LIMIT                               4      /**< \brief Stop on voltage limit (options: 0x605E)*/


/*---------------------------------------------
-    Specific values for Quick stop option code (object 0x605A) (IEC61800_184e)
        indicated the quick stop function
-----------------------------------------------*/
//-32768 to -1        MANUFACTURER_SPECIFIC
#define SLOWDOWN_RAMP_NO_TRANSIT                            5      /**< \brief Slow down on slow down ramp and stay in Quick Stop Active*/
#define QUICKSTOP_RAMP_NO_TRANSIT                           6      /**< \brief Slow down on quick stop ramp and stay in Quick Stop Active*/
#define CURRENT_LIMIT_NO_TRANSIT                            7      /**< \brief Slow down on current limit and stay in Quick Stop Active*/
#define VOLTAGE_LIMIT_NO_TRANSIT                            8      /**< \brief Slow down on voltage limit and stay in Quick Stop Active*/



/***************************************
 CiA402 Object Dictionary indexes
 ***************************************/
#define OBD_CIA402_AXIS_OFFSET                              0x0800

#define OBD_ABORT_CONNECTION_OPTION_CODE_INDEX(x)           (0x6007 + x * OBD_CIA402_AXIS_OFFSET)

#define OBD_CONTROLWORD_INDEX(x)                            (0x6040 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_STATUSWORD_INDEX(x)                             (0x6041 + x * OBD_CIA402_AXIS_OFFSET)

#define OBD_QUICKSTOP_INDEX(x)                              (0x605A + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_SHUTDOWN_INDEX(x)                               (0x605B + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_DISABLE_OPERATION_INDEX(x)                      (0x605C + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_HALT_OPTION_CODE_INDEX(x)                       (0x605D + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_FAULT_REACTION_INDEX(x)                         (0x605E + x * OBD_CIA402_AXIS_OFFSET)

#define OBD_MODES_OF_OPERATION_INDEX(x)                     (0x6060 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_MODES_OF_OPERATION_DISPLAY_INDEX(x)             (0x6061 + x * OBD_CIA402_AXIS_OFFSET)

#define OBD_POSITION_DEMAND_VALUE_INDEX(x)                  (0x6062 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_POSITION_ACTUAL_INTERNAL_VALUE_INDEX(x)         (0x6063 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_POSITION_ACTUAL_VALUE_INDEX(x)                  (0x6064 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_FOLLOWING_ERROR_WINDOW_INDEX(x)                 (0x6065 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_FOLLOWING_ERROR_TIMEOUT_INDEX(x)                (0x6066 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_POSITION_WINDOW_INDEX(x)                        (0x6067 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_POSITION_WINDOW_TIME_INDEX(x)                   (0x6068 + x * OBD_CIA402_AXIS_OFFSET)

#define OBD_VELOCITY_SENSOR_ACTUAL_VALUE_INDEX(x)           (0x6069 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_VELOCITY_DEMAND_VALUE_INDEX(x)                  (0x606B + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_VELOCITY_ACTUAL_VALUE_INDEX(x)                  (0x606C + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_VELOCITY_WINDOW_INDEX(x)                        (0x606D + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_VELOCITY_WINDOW_TIME_INDEX(x)                   (0x606E + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_VELOCITY_THRESHOLD_INDEX(x)                     (0x606F + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_VELOCITY_THRESHOLD_TIME_INDEX(x)                (0x6070 + x * OBD_CIA402_AXIS_OFFSET)

#define OBD_TARGET_TORQUE_INDEX(x)                          (0x6071 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_MAX_TORQUE_INDEX(x)                             (0x6072 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_TORQUE_DEMAND_INDEX(x)                          (0x6074 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_TORQUE_ACTUAL_VALUE_INDEX(x)                    (0x6077 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_CURRENT_ACTUAL_VALUE_INDEX(x)                   (0x6078 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_TARGET_POSITION_INDEX(x)                        (0x607A + x * OBD_CIA402_AXIS_OFFSET)

#define OBD_POSITION_RANGE_LIMIT_INDEX(x)                   (0x607B + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_HOME_OFFSET_INDEX(x)                            (0x607C + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_SW_POSITION_LIMIT_INDEX(x)                      (0x607D + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_POLARITY_INDEX(x)                               (0x607E + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_TARGET_VELOCITY_INDEX(x)                        (0x60FF + x * OBD_CIA402_AXIS_OFFSET)

#define OBD_MAX_MOTOR_SPEED_INDEX(x)                        (0x6080 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_PROFILE_VELOCITY_INDEX(x)                       (0x6081 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_PROFILE_ACCELERATION_INDEX(x)                   (0x6083 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_PROFILE_DECELERATION_INDEX(x)                   (0x6084 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_QUISTOP_DECELERATION_INDEX(x)                   (0x6085 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_MOTION_PROFILE_TYPE_INDEX(x)                    (0x6086 + x * OBD_CIA402_AXIS_OFFSET)

#define OBD_POSITION_ENCODER_RESOLUTION_INDEX(x)            (0x608F + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_VELOCITY_ENCONDER_RESOLUTION_INDEX(x)           (0x6090 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_GEAR_RATIO_INDEX(x)                             (0x6091 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_FEED_CONSTANT_INDEX(x)                          (0x6092 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_VELOCITY_FACTOR_INDEX(x)                        (0x6096 + x * OBD_CIA402_AXIS_OFFSET)

#define OBD_HOMING_METHOD_INDEX(x)                          (0x6098 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_HOMING_SPEEDS_INDEX(x)                          (0x6099 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_HOMING_ACCELERATION_INDEX(x)                    (0x609A + x * OBD_CIA402_AXIS_OFFSET)

#define OBD_POSITION_OFFSET_INDEX(x)                        (0x60B0 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_VELOCITY_OFFSET_INDEX(x)                        (0x60B1 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_TORQUE_OFFSET_INDEX(x)                          (0x60B2 + x * OBD_CIA402_AXIS_OFFSET)

#define OBD_TOUCH_PROBE_FUNCTION_INDEX(x)                   (0x60B8 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_TOUCH_PROBE_STATUS_INDEX(x)                     (0x60B9 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_TOUCH_PROBE_1_POS_EDGE_INDEX(x)                 (0x60BA + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_TOUCH_PROBE_1_NEG_EDGE_INDEX(x)                 (0x60BB + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_TOUCH_PROBE_2_POS_EDGE_INDEX(x)                 (0x60BC + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_TOUCH_PROBE_2_NEG_EDGE_INDEX(x)                 (0x60BD + x * OBD_CIA402_AXIS_OFFSET)

#define OBD_INTERPOLATION_TIME_PERIOD_INDEX(x)              (0x60C2 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_MAX_ACCELERATION_INDEX(x)                       (0x60C5 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_MAX_DECELERATION_INDEX(x)                       (0x60C6 + x * OBD_CIA402_AXIS_OFFSET)

#define OBD_TOUCH_PROBE_SOURCE(x)                           (0x60D0 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_TOUCH_PROBE_1_POS_EDGE_CNT_INDEX(x)             (0x60D5 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_TOUCH_PROBE_1_NEG_EDGE_CNT_INDEX(x)             (0x60D6 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_TOUCH_PROBE_2_POS_EDGE_CNT_INDEX(x)             (0x60D7 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_TOUCH_PROBE_2_NEG_EDGE_CNT_INDEX(x)             (0x60D8 + x * OBD_CIA402_AXIS_OFFSET)

#define OBD_POSITIVE_TORQUE_LIMIT_VALUE_INDEX(x)            (0x60E0 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_NEGATIVE_TORQUE_LIMIT_VALUE_INDEX(x)            (0x60E1 + x * OBD_CIA402_AXIS_OFFSET)

#define OBD_FOLLOWING_ERROR_ACTUAL_VALUE_INDEX(x)           (0x60F4 + x * OBD_CIA402_AXIS_OFFSET)
#define OBD_TARGET_VELOCITY_INDEX(x)                        (0x60FF + x * OBD_CIA402_AXIS_OFFSET)

#define OBD_SUPPORTED_DRIVE_MODES_INDEX(x)                  (0x6502 + x * OBD_CIA402_AXIS_OFFSET)


/**
* @}
*/

#if (defined __cplusplus)
}
#endif

#endif /* __ECSLVAPIDEF402_H__ */
