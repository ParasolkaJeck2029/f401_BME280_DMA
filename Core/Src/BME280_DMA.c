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
void I2Cx_ReadData16_DMA(uint16_t Addr, uint8_t Reg, uint16_t *Value)
{
	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_I2C_Mem_Read_DMA(&BME280_I2C_HANDLER, Addr, Reg, I2C_MEMADD_SIZE_8BIT, (uint8_t *)Value, 2);
	if (status != HAL_OK) Error();
}
void I2Cx_ReadData24_DMA(uint16_t Addr, uint8_t Reg, uint32_t *Value)
{
	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_I2C_Mem_Read_DMA(&BME280_I2C_HANDLER, Addr, Reg, I2C_MEMADD_SIZE_8BIT, (uint8_t *)Value, 3);
	if (status != HAL_OK) Error();
}

void I2Cx_WriteData_DMA(uint16_t Addr, uint16_t reg, uint8_t *value){
	HAL_I2C_Mem_Write_DMA(&hi2c1, Addr, reg, I2C_MEMADD_SIZE_8BIT, value, 1);
}

void BME280_ReadReg(uint16_t Reg, uint8_t *result){
	I2Cx_ReadData_DMA(BME280_ADDR, Reg, result);
}

//==========Reading of diferent registers in BME280===========
void BME280_ReadReg_S16(uint8_t Reg, int16_t *Value){
	I2Cx_ReadData16_DMA(BME280_ADDR,Reg, (uint16_t*) Value);
}
void BME280_ReadReg_S24(uint8_t Reg, int32_t *Value){
	I2Cx_ReadData24_DMA(BME280_ADDR,Reg, (uint32_t*) Value);
	*(int32_t *) Value &= 0x00FFFFFF;
}
void BME280_ReadReg_U16(uint8_t Reg, uint16_t *Value){
	I2Cx_ReadData16_DMA(BME280_ADDR,Reg, Value);
}
void BME280_ReadReg_U24(uint8_t Reg, uint32_t *Value){
	I2Cx_ReadData24_DMA(BME280_ADDR,Reg,  Value);
	*(uint32_t *) Value &= 0x00FFFFFF;
}
void BME280_ReadReg_BE_U24(uint8_t Reg, uint32_t *Value)
{
  I2Cx_ReadData24_DMA(BME280_ADDR,Reg,Value);
  *(uint32_t *) Value = be24toword(*(uint32_t *) Value) & 0x00FFFFFF;
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
	uint8_t status;
	BME280_GetStatus(&status);
	sprintf(uart_string, "Init ok\r\nID: 0x%x\r\nStatus: 0x%x\r\n", id, status);
	HAL_UART_Transmit_DMA(&huart1, uart_string, strlen(uart_string));
}

void BME280_GetId(uint8_t * id){
	BME280_ReadReg(REG_ID, id);
}
void BME280_GetStatus(uint8_t *result)
{
	//Reading status of sensor, see #define STATUS...
	BME280_ReadReg(REG_STATUS, result);
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


