#include "stm32f4xx_hal.h"

/* Size (in bits) */
#define SIZE_5b 									0x05
#define SIZE_6b										0x06
#define SIZE_7b 									0x07
#define SIZE_8b 									0x08
#define SIZE_9b 									0x09
#define SIZE_10b 									0x0A
#define SIZE_11b									0x0B

// TODO Read Side
/* Command Types */
#define COMMAND_LOAD 								0x01
#define COMMAND_LOAD_RTN							0x81
#define COMMAND_SAVE 								0x02
#define COMMAND_SAVE_RTN							0x82
#define COMMAND_READ 								0x03
#define COMMAND_READ_RTN							0x83
#define COMMAND_WRITE 								0x04
#define COMMAND_WRITE_RTN							0x84
#define COMMAND_RESET 								0x05
#define COMMAND_POSITION 							0x06
#define COMMAND_POSITION_RTN						0x86

// TODO Read Side
/* Operating Modes */
#define OPERATION_MODE_RunNormal 					0x00
#define OPERATION_MODE_RunFree 						0x02
#define OPERATION_MODE_RunHold						0x03
#define OPERATION_MODE_ControlPosition 				0x00
#define OPERATION_MODE_ControlVelocity 				0x04
#define OPERATION_MODE_ControlTorque 				0x08
#define OPERATION_MODE_ControlFForward 				0x0C
#define OPERATION_MODE_ServoNormal 					0x00
#define OPERATION_MODE_ServoClone 					0x40
#define OPERATION_MODE_ServoReverse 				0x80

/* Options */
// TODO
#define OPTION_DEFAULT 								0x00

// TODO Read Side
/* System Area Address' */
#define SYSTEM_ID 									0x00 // byte   RW
#define SYSTEM_BAUDRATE 							0x01 // ulong  RW #
#define SYSTEM_POSITION_MIN 						0x05 // short  RW
#define SYSTEM_POSITION_MAX 						0x07 // short  RW
#define SYSTEM_POSITION_CENTER 						0x09 // short  RW
#define SYSTEM_MCU_TEMP_LIMIT 						0x0B // short  RW
#define SYSTEM_MCU_TEMP_LIMIT_PR 					0x0D // byte   RW
#define SYSTEM_MOTOR_TEMP_LIMIT 					0x0E // short  RW
#define SYSTEM_MOTOR_TEMP_LIMIT_PR 					0x10 // byte   RW
#define SYSTEM_CURRENT_LIMIT 						0x11 // ushort RW
#define SYSTEM_CURRENT_LIMIT_PR 					0x13 // byte   RW
#define SYSTEM_LOCKDETECT_TIME 						0x14 // byte   RW
#define SYSTEM_LOCKDETECT_OUTRATE 					0x15 // byte   RW
#define SYSTEM_LOCKDETECT_TIME_PR 					0x16 // byte   RW
#define SYSTEM_INPUT_VOLTAGE_MIN 					0x17 // ushort RW
#define SYSTEM_INPUT_VOLTAGE_MAX					0x19 // ushort RW
#define SYSTEM_TORQUE_LIMIT 						0x1B // byte   RW
#define SYSTEM_DEADBAND_WIDTH 						0x1C // ushort RW
#define SYSTEM_MOTOR_CW_RATIO 						0x22 // byte   RW #
#define SYSTEM_MOTOR_CCW_RATIO 						0x23 // byte   RW #

// TODO Read Side
// TODO Are all modes Needed such as Servo Option Servo Mode Torque ON etc
/* Servo Parameters Area Address' */
#define SERVO_SERVO_OPTION 							0x27 // byte   RW #
#define SERVO_SERVO_MODE 							0x28 // ushort RW
#define SERVO_TORQUE_ON 							0x28 // byte   RW
#define SERVO_RUN_MODE 								0x29 // byte   RW
#define SERVO_DESIRED_POSITION 						0x2A // short  RW
#define SERVO_CURRENT_POSITION 						0x2C // short  RO
#define SERVO_PREVIOUS_POSITION 					0x2E // short  RO
#define SERVO_DESIRED_VELOCITY 						0x30 // short  RW
#define SERVO_CURRENT_VELOCITY 						0x32 // short  RO
#define SERVO_PREVIOUS_VELOCITY 					0x34 // short  RO
#define SERVO_DESIRED_TIME 							0x36 // ushort RW
#define SERVO_RUNNING_TIME 							0x38 // ushort RO
#define SERVO_WORKING_TIME 							0x3A // ushort RO
#define SERVO_DESIRED_TORQUE 						0x3C // short  RW
#define SERVO_SYSTEM_CLOCK 							0x3E // ulong  RO
#define SERVO_SAMPLING_TIME 						0x42 // ushort RO
#define SERVO_MCU_TEMP 								0x44 // short  RO
#define SERVO_MOTOR_TEMP 							0x46 // short  RO
#define SERVO_INPUT_VOLTAGE 						0x4A // ushort RO
#define SERVO_PWM_DUTY 								0x4C // ushort RW
#define SERVO_PWM_FREQUENCY 						0x4E // ushort RW #
#define SERVO_ENCODER_VALUE 						0x50 // ushort RO
#define SERVO_ENCODER_COUNT 						0x52 // long   RW
#define SERVO_HALLIC_STATE 							0x56 // byte   RO

// TODO Read Side
/* Control Parameter Area Address' */
#define CONTROL_CONTROL_LOW 						0x5C // ushort RW
#define CONTROL_GAIN_PRESETNO 						0x5C // byte   RW
#define CONTROL_TYPE 								0x5D // byte   RO
#define CONTROL_KP0 								0x5E // ulong  RW
#define CONTROL_KD0 								0x62 // ulong  RW
#define CONTROL_KI0 								0x66 // ulong  RW
#define CONTROL_STATIC_FRICTION0 					0x6A // ushort RW
#define CONTROL_DYNAMIC_FRICTION0 					0x6C // ushort RW
#define CONTROL_KP1 								0x6E // ulong  RW
#define CONTROL_KD1 								0x72 // ulong  RW
#define CONTROL_KI1 								0x76 // ulong  RW
#define CONTROL_STATIC_FRICTION1 					0x7A // ushort RW
#define CONTROL_DYNAMIC_FRICTION1 					0x7C // ushort RW
#define CONTROL_KP2 								0x7E // ulong  RW
#define CONTROL_KD2 								0x82 // ulong  RW
#define CONTROL_KI2 								0x86 // ulong  RW
#define CONTROL_STATIC_FRICTION2 					0x8A // ushort RW
#define CONTROL_DYNAMIC_FRICTION2 					0x8C // ushort RW

// TODO Read Side
/* Error Status Area Address' */
#define ERROR_STATUS_BASE_ADDR 						0x9D // byte   RO
#define ERROR_STATUS_SYSTEM 						0x9E // ulong  RO
#define ERROR_STATUS_SYSTEM_ERROR 					0x9E // byte   RO
#define ERROR_STATUS_MOTOR 							0x9F // byte   RO
#define ERROR_STATUS_UART 							0XA0 // byte   RO
#define ERROR_STATUS_COMMAND 						0xA1 // byte   RO

// TODO Read Side
/* System Default Value, Version Information Area Address' */
#define CONFIG_MODEL_NUMBER 						0xA2 // ulong  RO
#define CONFIG_MODEL_NUMBER_VOLTAGE_CLASS 			0xA2 // char   RO
#define CONFIG_MODEL_NUMBER_VERSION 				0xA3 // byte   RO
#define CONFIG_MODEL_NUMBER_TORQUE 					0xA4 // byte   RO
#define CONFIG_MODEL_NUMBER_CASE 					0xA5 // byte   RO
#define CONFIG_MODEL_TYPE 							0xA6 // ulong  RO
#define CONFIG_MODEL_TYPE_MOTOR						0xA8 // char   RO
#define CONFIG_MODEL_TYPE_DEVICE 					0xA9 // char   RO
#define CONFIG_FW_VERSION 							0xAA // ulong  RO
#define CONFIG_FW_BUID 								0xAA // byte   RO
#define CONFIG_FW_REVISION 							0xAB // byte   RO
#define CONFIG_FW_MINOR 							0xAC // byte   RO
#define CONFIG_FW_MAJOR 							0xAD // byte   RO
#define CONFIG_ENC_OFFSET_CENTER		 			0xAE // short  RW # Factory Setting DO NOT OVERWRITE
#define CONFIG_ENC_OFFSET 							0xB0 // short  RW # Factory Setting DO NOT OVERWRITE

/* Servo ID */
#define SERVO_ID_0 									0x00
#define SERVO_ID_1 									0x01
#define SERVO_ID_2 									0x02
#define SERVO_ID_3 									0x03
#define SERVO_ID_4 									0x04
#define SERVO_ID_5 									0x05
#define SERVO_ID_6 									0x06
#define SERVO_ID_7 									0x07
#define SERVO_ID_8 									0x08
#define SERVO_ID_9 									0x09
#define SERVO_ID_10 								0x0A
#define SERVO_ID_11 								0x0B
#define SERVO_ID_12 								0x0C
#define SERVO_ID_13 								0x0D
#define SERVO_ID_14 								0x0E
#define SERVO_ID_15 								0x0F
#define SERVO_ID_16 								0x10
#define SERVO_ID_17 								0x11
#define SERVO_ID_18 								0x12
#define SERVO_ID_19 								0x13
#define SERVO_ID_20 								0x14

/* Read Length */
#define READ_LENGTH_1B 								0x01 // char  & byte
#define READ_LENGTH_2B 								0x02 // short & ushort
#define READ_LENGTH_4B 								0x04 // long  & ulong

/* Count */
#define COUNT 										0x01

/** Global Commands
*/

/* Single Mode Send Format */
uint8_t B3M_Load(UART_HandleTypeDef huart, uint8_t SERVO_ID);

/* Save Command */
uint8_t B3M_Save(UART_HandleTypeDef huart, uint8_t SERVO_ID);

/* Read Command */
uint8_t B3M_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint8_t READ_ADR, uint8_t READ_LENGTH);

// TODO More elegant way to read and write
/* Write Command */
uint8_t B3M_Write_1B(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint16_t DATA, uint8_t ADDR);
uint8_t B3M_Write_2B(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint16_t DATA, uint8_t ADDR);
uint8_t B3M_Write_4B(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint16_t DATA, uint8_t ADDR);

/* Reset Command */
void B3M_Reset(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint8_t TIME);

/* Set Position */
uint8_t B3M_SetDesirePostion(UART_HandleTypeDef huart, uint8_t SERVO_ID, int16_t POSITION);
uint8_t B3M_SetPosition(UART_HandleTypeDef huart, uint8_t SERVO_ID, int16_t POSITION, uint16_t TIME);

/** Operating Modes Commands
*/
uint8_t B3M_RunNormal(UART_HandleTypeDef huart, uint8_t SERVO_ID);

/* Servo Operation Modes */
uint8_t B3M_SetNormal(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_Free(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_Hold(UART_HandleTypeDef huart, uint8_t SERVO_ID);

/* Servo Control */
uint8_t B3M_ControlPosition(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_ControlVelocity(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_ControlTorque(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_ControlFeedForward(UART_HandleTypeDef huart, uint8_t SERVO_ID);

/* Servo Options */
uint8_t B3M_ServoNormal(UART_HandleTypeDef huart, uint8_t SERVO_ID);
void B3M_ServoClone(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_ServoReverse(UART_HandleTypeDef huart, uint8_t SERVO_ID);

/** System Commands
*/
uint8_t B3M_SystemID_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_SystemID_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint8_t NEW_SERVO_ID);
uint8_t B3M_SystemBaudrate_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_SystemBaudrate_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint32_t NEW_BAUDRATE);
uint8_t B3M_PositionMin_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_PositionMin_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint16_t NEW_POSITION_MIN);
uint8_t B3M_PositionMax_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_PositionMax_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint16_t NEW_POSITION_MAX);
uint8_t B3M_PositionCenter_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_PositionCenter_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint16_t NEW_POSITION_CENTER);
uint8_t B3M_MCUTempLimit_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_MCUTempLimit_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint16_t NEW_MCU_TEMP_LIMIT);
uint8_t B3M_MCUTempLimitPR_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_MCUTempLimitPR_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint8_t NEW_MCU_TEMP_LIMIT_PR);
uint8_t B3M_MotorTempLimit_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_MotorTempLimit_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint16_t NEW_MOTOR_TEMP_LIMIT);
uint8_t B3M_MotorTempLimitPR_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_MotorTempLimitPR_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint8_t NEW_MOTOR_TEMP_LIMIT_PR);
uint8_t B3M_CurrentLimit_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_CurrentLimit_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint16_t NEW_CURRENT_LIMIT);
uint8_t B3M_CurrentLimitPR_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_CurrentLimitPR_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint8_t NEW_CURRENT_LIMIT_PR);
uint8_t B3M_LockDetectTime_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_LockDetectTime_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint8_t NEW_LOCKDETECT_TIME);
uint8_t B3M_LockDetectOutrate_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_LockDetectOutrate_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint8_t NEW_LOCKDETECT_OUTRATE);
uint8_t B3M_LockDetectTimePR_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_LockDetectTimePR_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint8_t NEW_LOCKDETECT_TIME_PR);
uint8_t B3M_InputVoltageMin_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_InputVoltageMin_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint16_t NEW_INPUT_VOLTAGE_MIN);
uint8_t B3M_InputVoltageMax_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_InputVoltageMax_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint16_t NEW_INPUT_VOLTAGE_MAX);
uint8_t B3M_TorqueLimit_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_TorqueLimit_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint8_t NEW_TORQUE_LIMIT);
uint8_t B3M_DeadBandWidth_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_DeadBandWidth_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint16_t NEW_DEADBAND_WIDTH);
uint8_t B3M_MotorCWRatio_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_MotorCWRatio_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint8_t NEW_MOTOR_CW_RATIO);
uint8_t B3M_MotorCCWRatio_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_MotorCCWRatio_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint8_t NEW_MOTOR_CCW_RATIO);

/** Servo Parameter Area Address'
*/
uint8_t B3M_ServoOption_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_ServoOption_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID,  uint8_t NEW_SERVO_OPTION);
uint8_t B3M_ServoMode_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_ServoMode_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID,  uint8_t NEW_SERVO_MODE);
uint8_t B3M_TorqueOn_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_TorqueOn_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID,  uint8_t NEW_TORQUE_ON);
uint8_t B3M_RunMode_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_RunMode_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID,  uint8_t NEW_RUN_MODE);
uint8_t B3M_DesiredPosition_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_DesiredPosition_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID,  uint8_t NEW_DESIRED_POSITION );
uint8_t B3M_CurrentPosition_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_PreviousPosition_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_DisiredVelocity_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_DisiredVelocity_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID,  uint8_t NEW_DESIRED_VELOCITY);
uint8_t B3M_CurrentVelocity_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_PreviousVelocity_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_DesiredTime_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_DesiredTime_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID,  uint8_t NEW_DESIRED_TIME);
uint8_t B3M_RunningTime_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_WorkingTime_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_DesiredTorque_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_DesiredTorque_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID,  uint8_t NEW_DESIRED_TORQUE);
uint8_t B3M_SystemClock_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_SamplingTime_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_MCUTemp_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_MotorTemp_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_InputVoltage_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_PWMDuty_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_PWMDuty_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID,  uint8_t NEW_PWM_DUTY);
uint8_t B3M_PWMFrequency_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_PWMFrequency_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID,  uint8_t NEW_PWM_FREQUENCY);
uint8_t B3M_EncoderValue_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_EncoderCount_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_EncoderCount_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID,  uint8_t NEW_ENCODER_COUNT);
uint8_t B3M_HallICState_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID);

/** Control Parameter Area Address'
*/
uint8_t B3M_ControlLow_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_ControlLow_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint8_t NEW_CONTROL_LOW);
uint8_t B3M_GainPresetNO_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_GainPresetNO_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint8_t NEW_GAIN_PRESETNO);
uint8_t B3M_Type_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_KP0_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_KP0_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint8_t NEW_KP0);
uint8_t B3M_KD0_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_KD0_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint8_t NEW_KD0);
uint8_t B3M_KI0_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_KI0_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint8_t NEW_KI0);
uint8_t B3M_StaticFriction0_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_StaticFriction0_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint8_t NEW_STATIC_FRICTION0);
uint8_t B3M_DynamicFriction0_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_DynamicFriction0_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint8_t NEW_DYNAMIC_FRICTION0);
uint8_t B3M_KP1_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_KP1_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint8_t NEW_KP1);
uint8_t B3M_KD1_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_KD1_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint8_t NEW_KD1);
uint8_t B3M_KI1_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_KI1_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint8_t NEW_KI1);
uint8_t B3M_StaticFriction1_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_StaticFriction1_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint8_t NEW_STATIC_FRICTION1);
uint8_t B3M_DynamicFriction1_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_DynamicFriction1_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint8_t NEW_DYNAMIC_FRICTION1);
uint8_t B3M_KP2_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_KP2_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint8_t NEW_KP2);
uint8_t B3M_KD2_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_KD2_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint8_t NEW_KD2);
uint8_t B3M_KI2_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_KI2_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint8_t NEW_KI2);
uint8_t B3M_StaticFriction2_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_StaticFriction2_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint8_t NEW_STATIC_FRICTION2);
uint8_t B3M_DynamicFriction2_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_DynamicFriction2_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint8_t NEW_DYNAMIC_FRICTION2);

/** Error Status Area Address'
*/
uint8_t B3M_StatusBaseADDR_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_StatusSystem_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_StatusSytemError_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_StatusMotor_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_StatusUart_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_StatusCommand_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID);

/** System Default Value, Version Information Area Address'
*/
uint8_t B3M_ModelNumber_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_ModelNumberVoltageClass_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_ModelNumberVersion_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_ModelNumberTorque_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_ModelNumberCase_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_ModelType_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_ModelTypeMotor_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_ModelTypeDevice_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_FWVersion_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_FWRevision_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_FWMinor_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_FWMajor_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_EncOffsetCenter_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID);
uint8_t B3M_EncOffset_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID);
