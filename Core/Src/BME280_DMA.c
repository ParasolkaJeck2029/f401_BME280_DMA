/*
 * BME280_DMA.c
 *
 *  Created on: Jul 11, 2022
 *      Author: ParasolkaJeck
 */
#include "BME280_DMA.h"
extern I2C_HandleTypeDef BME280_I2C_HANDLER;
extern UART_HandleTypeDef huart1;
extern char uart_string[100];

uint8_t I2Cx_ReadData_DMA(uint16_t Addr, uint16_t reg, uint8_t *result){
	HAL_StatusTypeDef res = HAL_OK;
	res = HAL_I2C_Mem_Read_DMA(&BME280_I2C_HANDLER, Addr, reg, I2C_MEMADD_SIZE_8BIT, result, 1);
	if(res != HAL_OK){Error();}
	return res;
}

void BME280_ReadReg(uint16_t Reg, uint8_t *result){
	I2Cx_ReadData_DMA(BME280_ADDR, Reg, result);
}

void BME280_GetId(uint8_t * id){
	BME280_ReadReg(REG_ID, id);
}

void Error(){
	LED_OFF;
}


