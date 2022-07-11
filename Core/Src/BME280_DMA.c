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

extern DMA_HandleTypeDef hdma_i2c1_tx;
extern DMA_HandleTypeDef hdma_i2c1_rx;

void Error(){
	LED_OFF;
}

uint8_t I2Cx_ReadData_DMA(uint16_t Addr, uint16_t reg, uint8_t *result){
	HAL_StatusTypeDef res = HAL_OK;
	res = HAL_I2C_Mem_Read_DMA(&BME280_I2C_HANDLER, Addr, reg, I2C_MEMADD_SIZE_8BIT, result, 1);
	if(res != HAL_OK){Error();}
	return res;
}

void I2Cx_WriteData_DMA(uint16_t Addr, uint16_t reg, uint8_t *value){
	HAL_I2C_Mem_Write_DMA(&hi2c1, Addr, reg, I2C_MEMADD_SIZE_8BIT, value, 1);
}

void BME280_ReadReg(uint16_t Reg, uint8_t *result){
	I2Cx_ReadData_DMA(BME280_ADDR, Reg, result);
}

void BME280_WriteReg(uint16_t Reg, uint8_t * value){
	I2Cx_WriteData_DMA(BME280_ADDR, Reg, value);
}

void BME280_Init(){
	uint8_t id;
	BME280_GetId(&id);
	while(HAL_DMA_GetState(&hdma_i2c1_rx)!= HAL_DMA_STATE_READY	);
	switch (id){
	case 0x57:break;
	case 0x58:break;
	case 0x60:break;
	default: {
		sprintf(uart_string, "Incorrect ID: 0x%x\r\n", id);
		HAL_UART_Transmit_DMA(&huart1, uart_string, strlen(uart_string));
		return;
	}
	}
	BME280_SetOversampling(BME280_OVERSAMPLING_X4, BME280_OVERSAMPLING_X4, BME280_OVERSAMPLING_X4, BME280_MODE_NORMAL);
	sprintf(uart_string, "Init ok\r\nID: 0x%x\r\n", id);
	HAL_UART_Transmit_DMA(&huart1, uart_string, strlen(uart_string));
}

void BME280_GetId(uint8_t * id){
	BME280_ReadReg(REG_ID, id);
}

void BME280_SetOversampling(uint8_t oversampling_temp, uint8_t oversampling_pres, uint8_t oversampling_hum, uint8_t mode){
	switch(oversampling_temp){
	case BME280_OVERSAMPLING_X1:break;
	case BME280_OVERSAMPLING_X2:break;
	case BME280_OVERSAMPLING_X4:break;
	case BME280_OVERSAMPLING_X8:break;
	case BME280_OVERSAMPLING_X16:break;
	default: return;
	}
	switch(oversampling_pres){
	case BME280_OVERSAMPLING_X1:break;
	case BME280_OVERSAMPLING_X2:break;
	case BME280_OVERSAMPLING_X4:break;
	case BME280_OVERSAMPLING_X8:break;
	case BME280_OVERSAMPLING_X16:break;
	default: return;
	}
	switch(oversampling_hum){
	case BME280_OVERSAMPLING_X1:break;
	case BME280_OVERSAMPLING_X2:break;
	case BME280_OVERSAMPLING_X4:break;
	case BME280_OVERSAMPLING_X8:break;
	case BME280_OVERSAMPLING_X16:break;
	default: return;
	}
	switch(mode){
	case BME280_MODE_FORCED:break;
	case BME280_MODE_NORMAL:break;
	case BME280_MODE_SLEEP:break;
	default: return;
	}
	BME280_WriteReg(REG_CTRL_HUM, &oversampling_hum);
	uint8_t reg_cntl_meas_value;
	reg_cntl_meas_value = ((oversampling_temp)<<5)|((oversampling_pres)<<2)|(mode);
	BME280_WriteReg(REG_CTRL_MEAS, &reg_cntl_meas_value);
}


