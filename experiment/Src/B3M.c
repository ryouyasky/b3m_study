#include "B3M.h"
#include "stm32f4xx_hal.h"

typedef struct B3M_ReadStruct {
	uint8_t size;
	uint8_t command;
	uint8_t option;
	uint8_t ID;
	uint8_t sum;
} B3M_ReadStruct;

//uint8_t B3M_Load(UART_HandleTypeDef huart, uint8_t SERVO_ID)
//{
//	B3M_ReadStruct readStruct;
//
//	readStruct.size = SIZE_5b;
//	readStruct.command = COMMAND_LOAD;
//	readStruct.option = OPTION_DEFAULT;
//	readStruct.ID = SERVO_ID;
//	readStruct.sum = readStruct.size + readStruct.command + readStruct.option + readStruct.ID;
//
//	HAL_UART_Transmit(&huart, (uint8_t*)&readStruct, readStruct.size, HAL_MAX_DELAY);
////	HAL_UART_Transmit(&huart, (uint8_t*)TxData, 5, HAL_MAX_DELAY);
//	HAL_Delay(1);
//    return 0;
//}


/* Single Mode Send Format */
uint8_t B3M_Load(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	uint8_t TxData[5];
	uint8_t CheckSum = 0;

	TxData[0] = SIZE_5b;   								// SIZE
	TxData[1] = COMMAND_LOAD;   						// COMMAND
	TxData[2] = OPTION_DEFAULT;   						// OPTION
	TxData[3] = SERVO_ID;   							// ID

    /* CheckSum Calculation */
    for(uint8_t i=0; i<=4; i++){
        CheckSum = CheckSum + TxData[i];                // XOR from ID to Data
    }
    CheckSum = (uint8_t)0x00FF & CheckSum;              // CHECKSUM

    TxData[4] = CheckSum;

	HAL_UART_Transmit(&huart, (uint8_t*)TxData, 5, HAL_MAX_DELAY);
	HAL_Delay(1);
    return 0;
}

/* Save Command */
uint8_t B3M_Save(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	uint8_t TxData[5];
	uint8_t CheckSum = 0;

	TxData[0] = SIZE_5b;   								// SIZE
	TxData[1] = COMMAND_SAVE;   						// COMMAND
	TxData[2] = OPTION_DEFAULT;   						// OPTION
	TxData[3] = SERVO_ID;   							// ID

    /* CheckSum Calculation */
    for(uint8_t i=0; i<=4; i++){
        CheckSum = CheckSum + TxData[i];                // XOR from ID to Data
    }
    CheckSum = (uint8_t)0x00FF & CheckSum;              // CHECKSUM

    TxData[4] = CheckSum;

	HAL_UART_Transmit(&huart, (uint8_t*)TxData, 5, HAL_MAX_DELAY);
	HAL_Delay(1);
    return 0;
}

/* Read Command */
uint8_t B3M_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint8_t READ_ADR, uint8_t READ_LENGTH)
{
	uint8_t TxData[7];
	uint8_t CheckSum = 0;

	TxData[0] = SIZE_7b;   								// SIZE
	TxData[1] = COMMAND_READ;   						// COMMAND
	TxData[2] = OPTION_DEFAULT;   						// OPTION
	TxData[3] = SERVO_ID;   							// ID
	TxData[4] = READ_ADR;   							// DATA
	TxData[5] = READ_LENGTH;   							// LENGTH

    /* CheckSum Calculation */
    for(uint8_t i=0; i<=5; i++){
        CheckSum = CheckSum + TxData[i];                // XOR from ID to Data
    }
    CheckSum = (uint8_t)0x00FF & CheckSum;              // CHECKSUM

    TxData[6] = CheckSum;

	HAL_UART_Transmit(&huart, (uint8_t*)TxData, 7, HAL_MAX_DELAY);
	HAL_Delay(1);
	return 0;
}

/* Write Command */
uint8_t B3M_Write_1B(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint16_t DATA, uint8_t ADR)
{
	uint8_t TxData[8];
	uint8_t CheckSum = 0;

	TxData[0] = SIZE_8b;   								// SIZE
	TxData[1] = COMMAND_WRITE;   						// COMMAND
	TxData[2] = OPTION_DEFAULT;   						// OPTION
	TxData[3] = SERVO_ID;   							// ID
	TxData[4] = DATA;   								// DATA
	TxData[5] = ADR;   									// ADR
	TxData[6] = COUNT;									// CNT

    /* CheckSum Calculation */
    for(uint8_t i=0; i<=6; i++){
        CheckSum = CheckSum + TxData[i];                // XOR from ID to Data
    }
    CheckSum = (uint8_t)0x00FF & CheckSum;              // CHECKSUM

    TxData[7] = CheckSum;

	HAL_UART_Transmit(&huart, (uint8_t*)TxData, 8, HAL_MAX_DELAY);
	HAL_Delay(1);
	return 0;
}

uint8_t B3M_Write_2B(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint16_t DATA, uint8_t ADR)
{
	uint8_t TxData[9];
	uint8_t CheckSum = 0;

	TxData[0] = SIZE_9b;   								// SIZE
	TxData[1] = COMMAND_WRITE;   						// COMMAND
	TxData[2] = OPTION_DEFAULT;   						// OPTION
	TxData[3] = SERVO_ID;   							// ID
    TxData[4] = (uint8_t)0x00FF & DATA;             	// DATA_LSB
    TxData[5] = (uint8_t)0x00FF & (DATA >> 8);    		// DATA_MSB
	TxData[6] = ADR;   									// ADR
	TxData[7] = COUNT;									// CNT

    /* CheckSum Calculation */
    for(uint8_t i=0; i<=7; i++){
        CheckSum = CheckSum + TxData[i];                // XOR from ID to Data
    }
    CheckSum = (uint8_t)0x00FF & CheckSum;              // CHECKSUM

    TxData[8] = CheckSum;

	HAL_UART_Transmit(&huart, (uint8_t*)TxData, 9, HAL_MAX_DELAY);
	HAL_Delay(1);
	return 0;
}

uint8_t B3M_Write_4B(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint16_t DATA, uint8_t ADDR)
{
	uint8_t TxData[11];
	uint8_t CheckSum = 0;

	TxData[0] = SIZE_11b;   							// SIZE
	TxData[1] = COMMAND_WRITE;   						// COMMAND
	TxData[2] = OPTION_DEFAULT;   						// OPTION
	TxData[3] = SERVO_ID;   							// ID
    TxData[4] = (uint8_t)0x000000FF & DATA;             // DATA_LSB
    TxData[5] = (uint8_t)0x000000FF & (DATA >> 8);    	// DATA_MIDLE2
    TxData[6] = (uint8_t)0x000000FF & (DATA >> 12);		// DATA_MIDLE1
    TxData[7] = (uint8_t)0x000000FF & (DATA >> 16);		// DATA_MSB
	TxData[8] = ADDR;   									// ADR
	TxData[9] = COUNT;									// CNT

    /* CheckSum Calculation */
    for(uint8_t i=0; i<=9; i++){
        CheckSum = CheckSum + TxData[i];                // XOR from ID to Data
    }
    CheckSum = (uint8_t)0x00FF & CheckSum;              // CHECKSUM

    TxData[10] = CheckSum;

	HAL_UART_Transmit(&huart, (uint8_t*)TxData, 11, HAL_MAX_DELAY);
	HAL_Delay(1);
	return 0;
}

/* Reset Command */
void B3M_Reset(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint8_t TIME)
{
	uint8_t TxData[6];
	uint8_t CheckSum = 0;

	TxData[0] = SIZE_6b;   								// SIZE
	TxData[1] = COMMAND_RESET;   						// COMMAND
	TxData[2] = OPTION_DEFAULT;   						// OPTION
	TxData[3] = SERVO_ID;   							// ID
	TxData[4] = TIME;   								// TIME

    /* CheckSum Calculation */
    for(uint8_t i=0; i<=4; i++){
        CheckSum = CheckSum + TxData[i];                // XOR from ID to Data
    }
    CheckSum = (uint8_t)0x00FF & CheckSum;              // CHECKSUM

    TxData[5] = CheckSum;

	HAL_UART_Transmit(&huart, (uint8_t*)TxData, 6, HAL_MAX_DELAY);
	HAL_Delay(1);
}

/* Set Position */
uint8_t B3M_SetDesirePostion(UART_HandleTypeDef huart, uint8_t SERVO_ID, int16_t POSITION)
{
    /* Local Variable */
    //uint8_t TxData[9];
		uint8_t TxData[12];

    uint8_t CheckSum = 0;

		//
		uint8_t SERVO_subID = 10;
		//

    if(POSITION<0)
        POSITION = 65535+POSITION+1;
/*
		TxData[0] = 0x09;//=SIZE_9b                                // SIZE
    TxData[1] = COMMAND_WRITE;                          // CMD
    TxData[2] = OPTION_DEFAULT;                         // OPTION
    TxData[3] = SERVO_ID;																// ID
    TxData[4] = (uint8_t)0x00FF & POSITION;             // POSITION_LSB
    TxData[5] = (uint8_t)0x00FF & (POSITION >> 8);      // POSITION_MSB

    TxData[6] =SERVO_DESIRED_POSITION;        			// ADR
    TxData[7] = COUNT;                                  // COUNT

		for(uint8_t i=0; i<=7; i++){
				CheckSum = CheckSum + TxData[i];                // XOR from ID to Data
		}
		CheckSum = (uint8_t)0x00FF & CheckSum;              // CHECKSUM

		TxData[8] = CheckSum;

		HAL_UART_Transmit(&huart, (uint8_t*)TxData, 9, HAL_MAX_DELAY);
		HAL_Delay(0.1);
		return 0;
*/
		TxData[0] = 0x0C;                               // SIZE
		TxData[1] = COMMAND_WRITE;                          // CMD
		TxData[2] = OPTION_DEFAULT;                         // OPTION
		TxData[3] = SERVO_ID;																// ID
		TxData[4] = (uint8_t)0x00FF & POSITION;             // POSITION_LSB
		TxData[5] = (uint8_t)0x00FF & (POSITION >> 8);      // POSITION_MSB

		TxData[6] = SERVO_subID;																// ID
		TxData[7] = 0;             // POSITION_LSB
		TxData[8] = 0;      // POSITION_MSB

		TxData[9] =SERVO_DESIRED_POSITION;        			// ADR
		TxData[10] = COUNT;                                  // COUNT
    /* CheckSum Calculation */
    for(uint8_t i=0; i<=10; i++){
        CheckSum = CheckSum + TxData[i];                // XOR from ID to Data
    }
    CheckSum = (uint8_t)0x00FF & CheckSum;              // CHECKSUM

    TxData[11] = CheckSum;

    HAL_UART_Transmit(&huart, (uint8_t*)TxData, 12, HAL_MAX_DELAY);
    HAL_Delay(0.1);
    return 0;
}

uint8_t B3M_SetPosition(UART_HandleTypeDef huart, uint8_t SERVO_ID, int16_t POSITION, uint16_t TIME)
{
    /* Local Variable */
    uint8_t TxData[9];
    uint8_t CheckSum = 0;

    if(POSITION<0)
        POSITION = 65535+POSITION+1;

    TxData[0] = SIZE_9b;                                // SIZE
    TxData[1] = COMMAND_WRITE;                          // CMD
    TxData[2] = OPTION_DEFAULT;                         // OPTION
    TxData[3] = SERVO_ID;                               // ID
    TxData[4] = (uint8_t)0x00FF & POSITION;             // POSITION_LSB
    TxData[5] = (uint8_t)0x00FF & (POSITION >> 8);      // POSITION_MSB
    TxData[6] = (uint8_t)0x00FF & TIME;        			// TIME_LSB
    TxData[7] = (uint8_t)0x00FF & (TIME >> 8);          // TIME_MSB

    /* CheckSum Calculation */
    for(uint8_t i=0; i<=7; i++){
        CheckSum = CheckSum + TxData[i];                // XOR from ID to Data
    }
    CheckSum = (uint8_t)0x00FF & CheckSum;              // CHECKSUM

    TxData[8] = CheckSum;

    HAL_UART_Transmit(&huart, (uint8_t*)TxData, 9, HAL_MAX_DELAY);
    HAL_Delay(1);
    return 0;
}

/** Servo Operation Modes
*/
uint8_t B3M_RunNormal(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Write_1B(huart, SERVO_ID, OPERATION_MODE_RunNormal, SERVO_TORQUE_ON);
	return 0;
}

uint8_t B3M_RunFree(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Write_1B(huart, SERVO_ID, OPERATION_MODE_RunFree, SERVO_TORQUE_ON);
	return 0;
}

uint8_t B3M_RunHold(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Write_1B(huart, SERVO_ID, OPERATION_MODE_RunFree, SERVO_TORQUE_ON);
	return 0;
}

uint8_t B3M_ControlPosition(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Write_1B(huart, SERVO_ID, OPERATION_MODE_ControlPosition, SERVO_TORQUE_ON);
	return 0;
}

/** Servo Control
*/
uint8_t B3M_ControlVelocity(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Write_1B(huart, SERVO_ID, OPERATION_MODE_ControlVelocity, SERVO_TORQUE_ON);
	return 0;
}

uint8_t B3M_ControlTorque(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Write_1B(huart, SERVO_ID, OPERATION_MODE_ControlTorque, SERVO_TORQUE_ON);
	return 0;
}

uint8_t B3M_ControlFeedForward(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Write_1B(huart, SERVO_ID, OPERATION_MODE_ControlFForward, SERVO_TORQUE_ON);
	return 0;
}

/** Servo Options
*/
uint8_t B3M_ServoNormal(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Write_1B(huart, SERVO_ID, OPERATION_MODE_ServoNormal, SERVO_TORQUE_ON);
	return 0;
}

void B3M_ServoClone(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Write_1B(huart, SERVO_ID, OPERATION_MODE_ServoClone, SERVO_TORQUE_ON);
}

uint8_t B3M_ServoReverse(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Write_1B(huart, SERVO_ID, OPERATION_MODE_ServoReverse, SERVO_TORQUE_ON);
	return 0;
}

/** System Commands
*/
uint8_t B3M_SystemID_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Read(huart, SERVO_ID, SYSTEM_ID, READ_LENGTH_1B);
	return 0;
}

uint8_t B3M_SystemID_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint8_t NEW_SERVO_ID)
{
	B3M_Write_1B(huart, SERVO_ID, NEW_SERVO_ID, SYSTEM_ID);
	return 0;
}

uint8_t B3M_SystemBaudrate_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Read(huart, SERVO_ID, SYSTEM_BAUDRATE, READ_LENGTH_4B);
	return 0;
}

uint8_t B3M_SystemBaudrate_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint32_t NEW_BAUDRATE)
{
	B3M_Write_4B(huart, SERVO_ID, NEW_BAUDRATE, SYSTEM_BAUDRATE);
	return 0;
}

uint8_t B3M_PositionMin_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Read(huart, SERVO_ID, SYSTEM_POSITION_MIN, READ_LENGTH_2B);
	return 0;
}

uint8_t B3M_PositionMin_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint16_t NEW_POSITION_MIN)
{
	B3M_Write_2B(huart, SERVO_ID, NEW_POSITION_MIN, SYSTEM_POSITION_MIN);
	return 0;
}

uint8_t B3M_PositionMax_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Read(huart, SERVO_ID, SYSTEM_POSITION_MAX, READ_LENGTH_2B);
	return 0;
}

uint8_t B3M_PositionMax_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint16_t NEW_POSITION_MAX)
{
	B3M_Write_2B(huart, SERVO_ID, NEW_POSITION_MAX, SYSTEM_POSITION_MAX);
	return 0;
}

uint8_t B3M_PositionCenter_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Read(huart, SERVO_ID, SYSTEM_POSITION_CENTER, READ_LENGTH_2B);
	return 0;
}

uint8_t B3M_PositionCenter_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint16_t NEW_POSITION_CENTER)
{
	B3M_Write_2B(huart, SERVO_ID, NEW_POSITION_CENTER, SYSTEM_POSITION_CENTER);
	return 0;
}

uint8_t B3M_MCUTempLimit_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Read(huart, SERVO_ID, SYSTEM_MCU_TEMP_LIMIT, READ_LENGTH_2B);
	return 0;
}

uint8_t B3M_MCUTempLimit_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint16_t NEW_MCU_TEMP_LIMIT)
{
	B3M_Write_2B(huart, SERVO_ID, NEW_MCU_TEMP_LIMIT, SYSTEM_MCU_TEMP_LIMIT);
	return 0;
}

uint8_t B3M_MCUTempLimitPR_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Read(huart, SERVO_ID, SYSTEM_MCU_TEMP_LIMIT_PR, READ_LENGTH_1B);
	return 0;
}

uint8_t B3M_MCUTempLimitPR_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint8_t NEW_MCU_TEMP_LIMIT_PR)
{
	B3M_Write_1B(huart, SERVO_ID, NEW_MCU_TEMP_LIMIT_PR, SYSTEM_MCU_TEMP_LIMIT_PR);
	return 0;
}

uint8_t B3M_MotorTempLimit_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Read(huart, SERVO_ID, SYSTEM_MOTOR_TEMP_LIMIT, READ_LENGTH_2B);
	return 0;
}

uint8_t B3M_MotorTempLimit_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint16_t NEW_MOTOR_TEMP_LIMIT)
{
	B3M_Write_2B(huart, SERVO_ID, NEW_MOTOR_TEMP_LIMIT, SYSTEM_MOTOR_TEMP_LIMIT);
	return 0;
}

uint8_t B3M_MotorTempLimitPR_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Read(huart, SERVO_ID, SYSTEM_MOTOR_TEMP_LIMIT_PR, READ_LENGTH_1B);
	return 0;
}

uint8_t B3M_MotorTempLimitPR_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint8_t NEW_MOTOR_TEMP_LIMIT_PR)
{
	B3M_Write_1B(huart, SERVO_ID, NEW_MOTOR_TEMP_LIMIT_PR, SYSTEM_MOTOR_TEMP_LIMIT_PR);
	return 0;
}

uint8_t B3M_CurrentLimit_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Read(huart, SERVO_ID, SYSTEM_CURRENT_LIMIT, READ_LENGTH_2B);
	return 0;
}

uint8_t B3M_CurrentLimit_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint16_t NEW_CURRENT_LIMIT)
{
	B3M_Write_2B(huart, SERVO_ID, NEW_CURRENT_LIMIT, SYSTEM_CURRENT_LIMIT);
	return 0;
}

uint8_t B3M_CurrentLimitPR_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Read(huart, SERVO_ID, SYSTEM_CURRENT_LIMIT_PR, READ_LENGTH_1B);
	return 0;
}

uint8_t B3M_CurrentLimitPR_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint8_t NEW_CURRENT_LIMIT_PR)
{
	B3M_Write_1B(huart, SERVO_ID, NEW_CURRENT_LIMIT_PR, SYSTEM_CURRENT_LIMIT_PR);
	return 0;
}

uint8_t B3M_LockDetectTime_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Read(huart, SERVO_ID, SYSTEM_LOCKDETECT_TIME, READ_LENGTH_1B);
	return 0;
}

uint8_t B3M_LockDetectTime_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint8_t NEW_LOCKDETECT_TIME)
{
	B3M_Write_1B(huart, SERVO_ID, NEW_LOCKDETECT_TIME, SYSTEM_LOCKDETECT_TIME);
	return 0;
}
//
uint8_t B3M_LockDetectOutrate_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Read(huart, SERVO_ID, SYSTEM_LOCKDETECT_OUTRATE, READ_LENGTH_1B);
	return 0;
}

uint8_t B3M_LockDetectOutrate_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint8_t NEW_LOCKDETECT_OUTRATE)
{
	B3M_Write_1B(huart, SERVO_ID, NEW_LOCKDETECT_OUTRATE, SYSTEM_LOCKDETECT_OUTRATE);
	return 0;
}

uint8_t B3M_LockDetectTimePR_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Read(huart, SERVO_ID, SYSTEM_LOCKDETECT_TIME_PR, READ_LENGTH_1B);
	return 0;
}

uint8_t B3M_LockDetectTimePR_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint8_t NEW_LOCKDETECT_TIME_PR)
{
	B3M_Write_1B(huart, SERVO_ID, NEW_LOCKDETECT_TIME_PR, SYSTEM_LOCKDETECT_TIME_PR);
	return 0;
}

uint8_t B3M_InputVoltageMin_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Read(huart, SERVO_ID, SYSTEM_INPUT_VOLTAGE_MIN, READ_LENGTH_2B);
	return 0;
}

uint8_t B3M_InputVoltageMin_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint16_t NEW_INPUT_VOLTAGE_MIN)
{
	B3M_Write_2B(huart, SERVO_ID, NEW_INPUT_VOLTAGE_MIN, SYSTEM_INPUT_VOLTAGE_MIN);
	return 0;
}

uint8_t B3M_InputVoltageMax_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Read(huart, SERVO_ID, SYSTEM_INPUT_VOLTAGE_MAX, READ_LENGTH_2B);
	return 0;
}

uint8_t B3M_InputVoltageMax_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint16_t NEW_INPUT_VOLTAGE_MAX)
{
	B3M_Write_2B(huart, SERVO_ID, NEW_INPUT_VOLTAGE_MAX, SYSTEM_INPUT_VOLTAGE_MAX);
	return 0;
}

uint8_t B3M_TorqueLimit_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Read(huart, SERVO_ID, SYSTEM_TORQUE_LIMIT, READ_LENGTH_1B);
	return 0;
}

uint8_t B3M_TorqueLimit_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint8_t NEW_TORQUE_LIMIT)
{
	B3M_Write_1B(huart, SERVO_ID, NEW_TORQUE_LIMIT, SYSTEM_TORQUE_LIMIT);
	return 0;
}

uint8_t B3M_DeadBandWidth_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Read(huart, SERVO_ID, SYSTEM_DEADBAND_WIDTH, READ_LENGTH_2B);
	return 0;
}

uint8_t B3M_DeadBandWidth_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint16_t NEW_DEADBAND_WIDTH)
{
	B3M_Write_2B(huart, SERVO_ID, NEW_DEADBAND_WIDTH, SYSTEM_DEADBAND_WIDTH);
	return 0;
}

uint8_t B3M_MotorCWRatio_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Read(huart, SERVO_ID, SYSTEM_MOTOR_CW_RATIO, READ_LENGTH_1B);
	return 0;
}

uint8_t B3M_MotorCWRatio_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint8_t NEW_MOTOR_CW_RATIO)
{
	B3M_Write_1B(huart, SERVO_ID, NEW_MOTOR_CW_RATIO, SYSTEM_MOTOR_CW_RATIO);
	return 0;
}

uint8_t B3M_MotorCCWRatio_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Read(huart, SERVO_ID, SYSTEM_MOTOR_CCW_RATIO, READ_LENGTH_1B);
	return 0;
}

uint8_t B3M_MotorCCWRatio_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint8_t NEW_MOTOR_CCW_RATIO)
{
	B3M_Write_1B(huart, SERVO_ID, NEW_MOTOR_CCW_RATIO, SYSTEM_MOTOR_CCW_RATIO);
	return 0;
}


/** Servo Parameter Area Address'
*/
uint8_t B3M_ServoOption_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Read(huart, SERVO_ID, SERVO_SERVO_OPTION, READ_LENGTH_2B);
	return 0;
}

//uint8_t B3M_ServoOption_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID,  uint8_t NEW_SERVO_OPTION)
//{
//	B3M_Write_2B(huart, SERVO_ID, NEW_SERVO_OPTION, SERVO_SERVO_OPTION);
//}
//
//uint8_t B3M_ServoMode_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID);
//uint8_t B3M_ServoMode_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID,  uint8_t NEW_SERVO_MODE);
//uint8_t B3M_TorqueOn_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID);
//uint8_t B3M_TorqueOn_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID,  uint8_t NEW_TORQUE_ON);
//uint8_t B3M_RunMode_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID);
//uint8_t B3M_RunMode_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID,  uint8_t NEW_RUN_MODE);
//uint8_t B3M_DesiredPosition_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID);
//uint8_t B3M_DesiredPosition_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID,  uint8_t NEW_DESIRED_POSITION );
//uint8_t B3M_CurrentPosition_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID);
//uint8_t B3M_PreviousPosition_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID);
//uint8_t B3M_DisiredVelocity_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID);
//uint8_t B3M_DisiredVelocity_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID,  uint8_t NEW_DESIRED_VELOCITY);
//uint8_t B3M_CurrentVelocity_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID);
//uint8_t B3M_PreviousVelocity_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID);
//uint8_t B3M_DesiredTime_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID);
//uint8_t B3M_DesiredTime_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID,  uint8_t NEW_DESIRED_TIME);
//uint8_t B3M_RunningTime_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID);
//uint8_t B3M_WorkingTime_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID);
//uint8_t B3M_DesiredTorque_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID);
//uint8_t B3M_DesiredTorque_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID,  uint8_t NEW_DESIRED_TORQUE);
//uint8_t B3M_SystemClock_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID);
//uint8_t B3M_SamplingTime_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID);
//uint8_t B3M_MCUTemp_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID);
//uint8_t B3M_MotorTemp_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID);
//uint8_t B3M_InputVoltage_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID);
//uint8_t B3M_PWMDuty_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID);
//uint8_t B3M_PWMDuty_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID,  uint8_t NEW_PWM_DUTY);
//uint8_t B3M_PWMFrequency_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID);
//uint8_t B3M_PWMFrequency_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID,  uint8_t NEW_PWM_FREQUENCY);
//uint8_t B3M_EncoderValue_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID);
//uint8_t B3M_EncoderCount_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID);
//uint8_t B3M_EncoderCount_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID,  uint8_t NEW_ENCODER_COUNT);
//uint8_t B3M_HallICState_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID);

/** Control Parameter Area Address'
*/
uint8_t B3M_ControlLow_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Read(huart, SERVO_ID, CONTROL_CONTROL_LOW, READ_LENGTH_2B);
	return 0;
}

uint8_t B3M_ControlLow_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint8_t NEW_CONTROL_LOW)
{
	B3M_Write_2B(huart, SERVO_ID, NEW_CONTROL_LOW, CONTROL_CONTROL_LOW);
	return 0;
}

uint8_t B3M_GainPresetNO_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Read(huart, SERVO_ID, CONTROL_GAIN_PRESETNO, READ_LENGTH_1B);
	return 0;
}

uint8_t B3M_GainPresetNO_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint8_t NEW_GAIN_PRESETNO)
{
	B3M_Write_1B(huart, SERVO_ID, NEW_GAIN_PRESETNO, CONTROL_GAIN_PRESETNO);
	return 0;
}

uint8_t B3M_Type_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Read(huart, SERVO_ID, CONTROL_TYPE, READ_LENGTH_1B);
	return 0;
}

uint8_t B3M_KP0_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Read(huart, SERVO_ID, CONTROL_KP0, READ_LENGTH_4B);
	return 0;
}

uint8_t B3M_KP0_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint8_t NEW_KP0)
{
	B3M_Write_4B(huart, SERVO_ID, NEW_KP0, CONTROL_KP0);
	return 0;
}

uint8_t B3M_KD0_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Read(huart, SERVO_ID, CONTROL_KD0, READ_LENGTH_4B);
	return 0;
}

uint8_t B3M_KD0_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint8_t NEW_KD0)
{
	B3M_Write_4B(huart, SERVO_ID, NEW_KD0, CONTROL_KD0);
	return 0;
}

uint8_t B3M_KI0_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Read(huart, SERVO_ID, CONTROL_KI0, READ_LENGTH_4B);
	return 0;
}

uint8_t B3M_KI0_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint8_t NEW_KI0)
{
	B3M_Write_4B(huart, SERVO_ID, NEW_KI0, CONTROL_KI0);
	return 0;
}

uint8_t B3M_StaticFriction0_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Read(huart, SERVO_ID, CONTROL_STATIC_FRICTION0, READ_LENGTH_2B);
	return 0;
}

uint8_t B3M_StaticFriction0_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint8_t NEW_STATIC_FRICTION0)
{
	B3M_Write_2B(huart, SERVO_ID, NEW_STATIC_FRICTION0, CONTROL_STATIC_FRICTION0);
	return 0;
}

uint8_t B3M_DynamicFriction0_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Read(huart, SERVO_ID, CONTROL_DYNAMIC_FRICTION0, READ_LENGTH_2B);
	return 0;
}

uint8_t B3M_DynamicFriction0_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint8_t NEW_DYNAMIC_FRICTION0)
{
	B3M_Write_2B(huart, SERVO_ID, NEW_DYNAMIC_FRICTION0, READ_LENGTH_2B);
	return 0;
}

uint8_t B3M_KP1_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Read(huart, SERVO_ID, CONTROL_KP1, READ_LENGTH_4B);
	return 0;
}

uint8_t B3M_KP1_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint8_t NEW_KP1)
{
	B3M_Write_4B(huart, SERVO_ID, NEW_KP1, CONTROL_KP1);
	return 0;
}

uint8_t B3M_KD1_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Read(huart, SERVO_ID, CONTROL_KD1, READ_LENGTH_4B);
	return 0;
}

uint8_t B3M_KD1_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint8_t NEW_KD1)
{
	B3M_Write_4B(huart, SERVO_ID, NEW_KD1, CONTROL_KD1);
	return 0;
}

uint8_t B3M_KI1_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Read(huart, SERVO_ID, CONTROL_KI1, READ_LENGTH_4B);
	return 0;
}

uint8_t B3M_KI1_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint8_t NEW_KI1)
{
	B3M_Write_4B(huart, SERVO_ID, NEW_KI1, CONTROL_KI1);
	return 0;
}

uint8_t B3M_StaticFriction1_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Read(huart, SERVO_ID, CONTROL_STATIC_FRICTION1, READ_LENGTH_2B);
	return 0;
}

uint8_t B3M_StaticFriction1_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint8_t NEW_STATIC_FRICTION1)
{
	B3M_Write_2B(huart, SERVO_ID, NEW_STATIC_FRICTION1, CONTROL_STATIC_FRICTION1);
	return 0;
}

uint8_t B3M_DynamicFriction1_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Read(huart, SERVO_ID, CONTROL_DYNAMIC_FRICTION1, READ_LENGTH_2B);
	return 0;
}

uint8_t B3M_DynamicFriction1_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint8_t NEW_DYNAMIC_FRICTION1)
{
	B3M_Write_2B(huart, SERVO_ID, NEW_DYNAMIC_FRICTION1, CONTROL_DYNAMIC_FRICTION1);
	return 0;
}
//
uint8_t B3M_KP2_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Read(huart, SERVO_ID, CONTROL_KP2, READ_LENGTH_4B);
	return 0;
}
uint8_t B3M_KP2_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint8_t NEW_KP2)
{
	B3M_Write_4B(huart, SERVO_ID, NEW_KP2, CONTROL_KP2);
	return 0;
}
uint8_t B3M_KD2_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Read(huart, SERVO_ID, CONTROL_KD2, READ_LENGTH_4B);
	return 0;
}
uint8_t B3M_KD2_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint8_t NEW_KD2)
{
	B3M_Write_4B(huart, SERVO_ID, NEW_KD2, CONTROL_KD2);
	return 0;
}
uint8_t B3M_KI2_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Read(huart, SERVO_ID, CONTROL_KI2, READ_LENGTH_4B);
	return 0;
}
uint8_t B3M_KI2_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint8_t NEW_KI2)
{
	B3M_Write_4B(huart, SERVO_ID, NEW_KI2, CONTROL_KI2);
	return 0;
}
uint8_t B3M_StaticFriction2_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Read(huart, SERVO_ID, CONTROL_STATIC_FRICTION2, READ_LENGTH_2B);
	return 0;
}
uint8_t B3M_StaticFriction2_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint8_t NEW_STATIC_FRICTION2)
{
	B3M_Write_2B(huart, SERVO_ID, NEW_STATIC_FRICTION2, CONTROL_STATIC_FRICTION2);
	return 0;
}
uint8_t B3M_DynamicFriction2_Read(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Read(huart, SERVO_ID, CONTROL_DYNAMIC_FRICTION2, READ_LENGTH_2B);
	return 0;
}
uint8_t B3M_DynamicFriction2_Write(UART_HandleTypeDef huart, uint8_t SERVO_ID, uint8_t NEW_DYNAMIC_FRICTION2)
{
	B3M_Write_2B(huart, SERVO_ID, NEW_DYNAMIC_FRICTION2, CONTROL_DYNAMIC_FRICTION2);
	return 0;
}

/** Error Status Area Address'
*/

uint8_t B3M_StatusBaseADDR_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Read(huart, SERVO_ID, ERROR_STATUS_BASE_ADDR, READ_LENGTH_1B);
	return 0;
}

uint8_t B3M_StatusSystem_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Read(huart, SERVO_ID, ERROR_STATUS_SYSTEM, READ_LENGTH_4B);
	return 0;
}

uint8_t B3M_StatusSytemError_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Read(huart, SERVO_ID, ERROR_STATUS_SYSTEM_ERROR, READ_LENGTH_1B);
	return 0;
}

uint8_t B3M_StatusMotor_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Read(huart, SERVO_ID, ERROR_STATUS_MOTOR, READ_LENGTH_1B);
	return 0;
}

uint8_t B3M_StatusUart_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Read(huart, SERVO_ID, ERROR_STATUS_UART, READ_LENGTH_1B);
	return 0;
}

uint8_t B3M_StatusCommand_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Read(huart, SERVO_ID, ERROR_STATUS_COMMAND, READ_LENGTH_1B);
	return 0;
}

/** System Default Value, Version Information Area Address'
*/

uint8_t B3M_ModelNumber_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Read(huart, SERVO_ID, CONFIG_MODEL_NUMBER, READ_LENGTH_4B);
	return 0;
}
uint8_t B3M_ModelNumberVoltageClass_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Read(huart, SERVO_ID, CONFIG_MODEL_NUMBER_VOLTAGE_CLASS, READ_LENGTH_1B);
	return 0;
}
uint8_t B3M_ModelNumberVersion_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Read(huart, SERVO_ID, CONFIG_MODEL_NUMBER_VERSION, READ_LENGTH_1B);
	return 0;
}
uint8_t B3M_ModelNumberTorque_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Read(huart, SERVO_ID, CONFIG_MODEL_NUMBER_TORQUE, READ_LENGTH_1B);
	return 0;
}
uint8_t B3M_ModelNumberCase_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Read(huart, SERVO_ID, CONFIG_MODEL_NUMBER_CASE, READ_LENGTH_1B);
	return 0;
}
uint8_t B3M_ModelType_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Read(huart, SERVO_ID, CONFIG_MODEL_TYPE, READ_LENGTH_4B);
	return 0;
}

uint8_t B3M_ModelTypeMotor_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Read(huart, SERVO_ID, CONFIG_MODEL_TYPE_MOTOR, READ_LENGTH_1B);
	return 0;
}

uint8_t B3M_ModelTypeDevice_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Read(huart, SERVO_ID, CONFIG_MODEL_TYPE_DEVICE, READ_LENGTH_1B);
	return 0;
}

uint8_t B3M_FWVersion_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Read(huart, SERVO_ID, CONFIG_FW_VERSION, READ_LENGTH_4B);
	return 0;
}

uint8_t B3M_FWRevision_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Read(huart, SERVO_ID, CONFIG_FW_REVISION, READ_LENGTH_1B);
	return 0;
}

uint8_t B3M_FWMinor_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Read(huart, SERVO_ID, CONFIG_FW_MINOR, READ_LENGTH_1B);
	return 0;
}

uint8_t B3M_FWMajor_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Read(huart, SERVO_ID, CONFIG_FW_MAJOR, READ_LENGTH_1B);
	return 0;
}

uint8_t B3M_EncOffsetCenter_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Read(huart, SERVO_ID, CONFIG_ENC_OFFSET_CENTER, READ_LENGTH_1B);
	return 0;
}

uint8_t B3M_EncOffset_ReadOnly(UART_HandleTypeDef huart, uint8_t SERVO_ID)
{
	B3M_Read(huart, SERVO_ID, CONFIG_ENC_OFFSET, READ_LENGTH_1B);
	return 0;
}
