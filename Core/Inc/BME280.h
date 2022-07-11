/*
 * BME280.h
 *
 *  Created on: Jul 5, 2022
 *      Author: ParasolkaJeck
 */

#ifndef INC_BME280_H_
#define INC_BME280_H_
#include "stm32f4xx_hal.h"
#include "main.h"
#include <string.h>
#include <math.h>
#include <stdio.h>
#define LED_GPIO_PORT GPIOC
#define LED_PIN GPIO_PIN_13
#define LED_ON HAL_GPIO_WritePin(LED_GPIO_PORT, LED_PIN, GPIO_PIN_RESET)
#define LED_OFF HAL_GPIO_WritePin(LED_GPIO_PORT, LED_PIN, GPIO_PIN_SET)
#define LED_TGL HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN)

#define be16toword(a) ((((a)>>8)&0xff)|(((a)<<8)&0xff00))
#define be24toword(a) ((((a)>>16)&0x000000ff)|((a)&0x0000ff00)|(((a)<<16)&0x00ff0000))

#define BME280_I2C_HANDLER hi2c1

#ifndef BME280_ADDR
#define BME280_ADDR 0xEC
#endif

#define REG_ID			0xD0
#define REG_RES 		0xE0
#define REG_CTRL_HUM	0xF2
#define REG_STATUS		0xF3
#define REG_CTRL_MEAS	0xF4
#define REG_CONFIG		0xF5
#define REG_PRESS		0xF7
#define REG_TEMP		0xFA
#define REG_HUM			0xFD

#define RESET_VALUE		0xB6

#define BME280_STANDBY_TIME_05 		0x00
#define BME280_STANDBY_TIME_62 		0x01
#define BME280_STANDBY_TIME_125 	0x02
#define BME280_STANDBY_TIME_250 	0x03
#define BME280_STANDBY_TIME_500 	0x04
#define BME280_STANDBY_TIME_1000 	0x05
#define BME280_STANDBY_TIME_10 		0x06
#define BME280_STANDBY_TIME_20 		0x07

#define BME280_FILTER_OFF	0x00
#define BME280_FILTER_2		0x01
#define BME280_FILTER_4		0x02
#define BME280_FILTER_8		0x03
#define BME280_FILTER_16	0x04

#define BME280_3WIRE_SPI_ON		0x01
#define BME280_3WIRE_SPI_OFF	0x00

#define STATUS_MEASURING 	0x08
#define STATUS_IM_UPDATE 	0x01
#define STATUS_READY 		0x00

#define BME280_OVERSAMPLING_X1		0x01
#define BME280_OVERSAMPLING_X2		0x02
#define BME280_OVERSAMPLING_X4		0x03
#define BME280_OVERSAMPLING_X8		0x04
#define BME280_OVERSAMPLING_X16		0x05

#define BME280_MODE_SLEEP	0x00
#define BME280_MODE_FORCED	0x01
#define BME280_MODE_NORMAL	0x03

#define CALIBRATION_T1 0x88
#define CALIBRATION_T2 0x8A
#define CALIBRATION_T3 0x8C
#define CALIBRATION_P1 0x8E
#define CALIBRATION_P2 0x90
#define CALIBRATION_P3 0x92
#define CALIBRATION_P4 0x94
#define CALIBRATION_P5 0x96
#define CALIBRATION_P6 0x98
#define CALIBRATION_P7 0x9A
#define CALIBRATION_P8 0x9C
#define CALIBRATION_P9 0x9E
#define CALIBRATION_H1 0xA1
#define CALIBRATION_H2 0xE1
#define CALIBRATION_H3 0xE3
#define CALIBRATION_H4 0xE4
#define CALIBRATION_H5 0xE5
#define CALIBRATION_H6 0xE7

typedef struct
{
	uint16_t T1;
	int16_t T2;
	int16_t	T3;
	uint16_t P1;
	int16_t	P2;
	int16_t	P3;
	int16_t	P4;
	int16_t	P5;
	int16_t	P6;
	int16_t	P7;
	int16_t	P8;
	int16_t	P9;
	unsigned char H1;
	int16_t  H2;
	unsigned char H3;
	int16_t H4;
	int16_t H5;
	char H6;
}BME280_Calibrate_parametrs;



uint8_t BME280_Init();

void Error();

uint8_t I2Cx_ReadData(uint16_t Addr, uint8_t Reg);
void I2Cx_ReadData16(uint16_t Addr, uint8_t Reg, uint16_t *Value);
void I2Cx_ReadData24(uint16_t Addr, uint8_t Reg, uint32_t *Value);

void BME280_ReadReg_S16(uint8_t Reg, int16_t *Value);
void BME280_ReadReg_S24(uint8_t Reg, int32_t *Value);
void BME280_ReadReg_U16(uint8_t Reg, uint16_t *Value);
void BME280_ReadReg_U24(uint8_t Reg, uint32_t *Value);

uint8_t BME280_ReadReg(uint8_t Reg);

void I2Cx_WriteData(uint16_t Addr, uint8_t Reg, uint8_t Value);
void I2Cx_WriteReg(uint8_t Reg, uint8_t Value);
uint8_t BME280_GetID(void);
uint8_t BME280_GetStatus();
float BME280_GetTemperature();
float BME280_GetPressure();
int32_t BME280_ReadPressureRAW();
int32_t BME280_ReadTemperatureRAW();
int16_t BME280_ReadHumidityRAW();
float BME280_GetHumidity();
void BME280_SoftReset();
void BME280_ReadReg_BE_U24(uint8_t Reg, uint32_t *Value);
void BME280_ReadCalibration();

void BME280_SetOversamplingHum(uint8_t oversampling_hum);
void BME280_SetOversamplingTemp(uint8_t oversampling_temp);
void BME280_SetOversamplingPress(uint8_t oversampling_pres);
void BME280_SetMode(uint8_t mode);

uint8_t BME280_SetOversampling(uint8_t oversampling_temp, uint8_t oversampling_pres, uint8_t oversampling_hum, uint8_t mode);
uint8_t BME280_GetOversamplingMode(uint8_t *array);

uint8_t BME280_SetConfig(uint8_t standby_time, uint8_t filter_coeficient, uint8_t spi_3wire_mode);

void BME280_SetStandbyTime(uint8_t standby_time);
void BME280_SetFilter(uint8_t filter_coeficient);
void BME280_SPI_3Wire(uint8_t state);
void BME280_GetConfig(uint8_t *array);
#endif /* INC_BME280_H_ */
