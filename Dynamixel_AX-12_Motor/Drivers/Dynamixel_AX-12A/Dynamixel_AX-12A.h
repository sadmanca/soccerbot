/*************** Drivers for Dynamixel AX-12A Servo Motor ***************/

/*************** Define to prevent recursive inclusion ***************/
#ifndef __DYNAMIXEL_AX_12A_H
#define __DYNAMIXEL_AX_12A_H

/*************** Includes ***************/
#include "stm32f4xx_hal.h"

/*************** Macros ***************/
#define __DYNAMIXEL_TRANSMIT() HAL_GPIO_WritePin(GPIOF, GPIO_PIN_15, 1) // Set data direction pin high (TX)
#define __DYNAMIXEL_RECEIVE() HAL_GPIO_WritePin(GPIOF, GPIO_PIN_15, 0) // Set data direction pin low (RX)

/*************** Private variables ***************/
UART_HandleTypeDef Dynamixel_huart;

/*************** Exported types ***************/
typedef struct{
	uint8_t					_ID;					/*!< Motor identification (0-252)					*/
	uint32_t				_BaudRate;				/*!< UART communication baud rate					*/
	uint16_t				_lastPosition;			/*!< Position read from motor						*/
	uint16_t				_lastVelocity;			/*!< Velocity read from motor						*/
	uint16_t				_lastLoad;				/*!< Load read from motor							*/
	uint16_t				_lastVoltage;			/*!< Voltage read from motor						*/
	uint16_t				_lastTemperature;		/*!< Temperature read from motor					*/
	uint16_t				_isJointMode;			/*!< 1 if motor is joint mode, 0 if wheel mode		*/
	UART_HandleTypeDef		*_UART_Handle;			/*!< UART handle for motor							*/
}Dynamixel_HandleTypeDef;

/*************** Private function prototypes ***************/
// Setters
void Dynamixel_SetID(Dynamixel_HandleTypeDef *hdynamixel, int ID); // (EEPROM)
void Dynamixel_SetBaudRate(Dynamixel_HandleTypeDef *hdynamixel, double baud); // (EEPROM)
void Dynamixel_SetReturnDelayTime(Dynamixel_HandleTypeDef *hdynamixel, double microSec); // (EEPROM)
void Dynamixel_SetCWAngleLimit(Dynamixel_HandleTypeDef *hdynamixel, double minAngle); // (EEPROM)
void Dynamixel_SetCCWAngleLimit(Dynamixel_HandleTypeDef *hdynamixel, double maxAngle); // (EEPROM)
void Dynamixel_SetHighestVoltageLimit(Dynamixel_HandleTypeDef *hdynamixel, double highestVoltage); // (EEPROM)
void Dynamixel_SetLowestVoltageLimit(Dynamixel_HandleTypeDef *hdynamixel, double lowestVoltage); // (EEPROM)
void Dynamixel_SetMaxTorque(Dynamixel_HandleTypeDef *hdynamixel, double maxTorque); // (EEPROM)
void Dynamixel_SetStatusReturnLevel(Dynamixel_HandleTypeDef *hdynamixel, uint8_t status_data); // (EEPROM)
void Dynamixel_SetAlarmLED(Dynamixel_HandleTypeDef *hdynamixel, uint8_t alarm_LED_data); // (EEPROM)
void Dynamixel_SetAlarmShutdown(Dynamixel_HandleTypeDef *hdynamixel, uint8_t alarm_shutdown_data); // (EEPROM)

void Dynamixel_TorqueEnable(Dynamixel_HandleTypeDef *hdynamixel, uint8_t isEnabled); // (RAM)
void Dynamixel_LEDEnable(Dynamixel_HandleTypeDef *hdynamixel, uint8_t isEnabled); // (RAM)
void Dynamixel_SetCWComplianceMargin(Dynamixel_HandleTypeDef *hdynamixel, uint8_t CWcomplianceMargin); // (RAM)
void Dynamixel_SetCCWComplianceMargin(Dynamixel_HandleTypeDef *hdynamixel, uint8_t CCWcomplianceMargin); // (RAM)
void Dynamixel_SetCWComplianceSlope(Dynamixel_HandleTypeDef *hdynamixel, uint8_t CWcomplianceSlope); // (RAM)
void Dynamixel_SetCCWComplianceMargin(Dynamixel_HandleTypeDef *hdynamixel, uint8_t CCWcomplianceSlope); // (RAM)
void Dynamixel_SetGoalPosition(Dynamixel_HandleTypeDef *hdynamixel, double goalAngle); // (RAM)
void Dynamixel_SetGoalVelocity(Dynamixel_HandleTypeDef *hdynamixel, double goalVelocity); // (RAM)
void Dynamixel_SetGoalTorque(Dynamixel_HandleTypeDef *hdynamixel, double goalTorque); // (RAM)
void Dynamixel_LockEEPROM(Dynamixel_HandleTypeDef *hdynamixel, uint8_t isLocked); // (RAM)
void Dynamixel_SetPunch(Dynamixel_HandleTypeDef *hdynamixel, double punch); // (RAM)

// Getters
void Dynamixel_GetPosition(Dynamixel_HandleTypeDef *hdynamixel); // UNIMPLEMENTED
void Dynamixel_GetVelocity(Dynamixel_HandleTypeDef *hdynamixel); // UNIMPLEMENTED
void Dynamixel_GetLoad(Dynamixel_HandleTypeDef *hdynamixel); // UNIMPLEMENTED
void Dynamixel_GetVoltage(Dynamixel_HandleTypeDef *hdynamixel); // UNIMPLEMENTED
void Dynamixel_GetTemperature(Dynamixel_HandleTypeDef *hdynamixel); // UNIMPLEMENTED
uint8_t Dynamixel_IsRegistered(Dynamixel_HandleTypeDef *hdynamixel); // UNIMPLEMENTED
uint8_t Dynamixel_IsMoving(Dynamixel_HandleTypeDef *hdynamixel); // UNIMPLEMENTED
uint8_t Dynamixel_IsJointMode(Dynamixel_HandleTypeDef *hdynamixel); // UNIMPLEMENTED

// Computation
uint8_t Dynamixel_ComputeChecksum(uint8_t *arr, int length);

// Transmission
void Dynamixel_DataWriter(Dynamixel_HandleTypeDef *hdynamixel, uint8_t arrSize, uint8_t writeAddr, uint8_t param1, uint8_t param2);

// Initialization
void Dynamixel_Init(Dynamixel_HandleTypeDef *hdynamixel, uint8_t ID, UART_HandleTypeDef* UART_Handle);

#endif /* __DYNAMIXEL_AX-12A_H */
