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

BME280_Calibrate_parametrs BME280_Cal_par;

int32_t temp_int;

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
	while(HAL_DMA_GetState(&hdma_i2c1_rx)!= HAL_DMA_STATE_READY);
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
	BME280_ReadCalibration();
	BME280_SetOversampling(BME280_OVERSAMPLING_X4, BME280_OVERSAMPLING_X4, BME280_OVERSAMPLING_X4, BME280_MODE_NORMAL);
	while(HAL_DMA_GetState(&hdma_i2c1_tx) != HAL_DMA_STATE_READY);
	uint8_t status;
	BME280_GetStatus(&status);
	sprintf(uart_string, "Init ok\r\nID: 0x%x\r\nStatus: 0x%x\r\n", id, status);
	HAL_UART_Transmit_DMA(&huart1, uart_string, strlen(uart_string));
}

void BME280_GetId(uint8_t * id){
	BME280_ReadReg(REG_ID, id);
}

void BME280_GetStatus(uint8_t *result)
{	//Reading status of sensor, see #define STATUS...
	BME280_ReadReg(REG_STATUS, result);
}
void BME280_SoftReset(){
	//SoftReset the sensor
	BME280_WriteReg(REG_RES, RESET_VALUE);
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
	while(HAL_DMA_GetState(&hdma_i2c1_tx) != HAL_DMA_STATE_READY);
	BME280_WriteReg(REG_CTRL_MEAS, &reg_cntl_meas_value);
	while(HAL_DMA_GetState(&hdma_i2c1_tx) != HAL_DMA_STATE_READY);

}

void BME280_GetOversamplingMode(uint8_t *array){
	//write array oversampling temperature, pressure, humidity, and mode
	uint8_t ovrs_hum, ovrs_temp, ovrs_pres, mode;
	uint8_t value_ctrl_meas;
	BME280_ReadReg(REG_CTRL_MEAS, &value_ctrl_meas);
	while(HAL_DMA_GetState(&hdma_i2c1_rx) != HAL_DMA_STATE_READY);
	ovrs_temp = (value_ctrl_meas&0b11100000)>>5;
	ovrs_pres = (value_ctrl_meas&0b00011100)>>2;
	mode = value_ctrl_meas&0b00000011;
	while(HAL_DMA_GetState(&hdma_i2c1_rx) != HAL_DMA_STATE_READY);
	BME280_ReadReg(REG_CTRL_HUM, &ovrs_hum);
	while(HAL_DMA_GetState(&hdma_i2c1_rx) != HAL_DMA_STATE_READY);
	*array = ovrs_temp;
	*(array + sizeof(uint8_t)) = ovrs_pres;
	*(array + 2 * sizeof(uint8_t)) = ovrs_hum;
	*(array + 3 * sizeof(uint8_t)) = mode;
	sprintf(uart_string,"temp: %d\r\npres: %d\r\nhum: %d\r\nmode: %d\r\n", ovrs_temp, ovrs_pres, ovrs_hum, mode);
	HAL_UART_Transmit_DMA(&huart1, uart_string, strlen(uart_string));
}
void BME280_ReadHumidityRAW(int16_t * result){
	BME280_ReadReg_S16(REG_HUM, result);
	*result = be16toword(*result);
}
void BME280_ReadTemperatureRAW(int32_t * result){ //read raw data of ADC sensor and turns over
	BME280_ReadReg_U24(REG_TEMP, result);
	*result = be24toword(*result);
}
void BME280_ReadPressureRAW(int32_t * result){//read raw data of ADC sensor and turns over
	BME280_ReadReg_U24(REG_HUM, (uint32_t *)result);
	*result = be24toword(*result);
}

void BME280_ReadCalibration(){
	//function read calibration_data from sensor, needed for correct measuring of all parameters
	BME280_ReadReg_U16(CALIBRATION_T1, & BME280_Cal_par.T1);
	while(HAL_DMA_GetState(&hdma_i2c1_rx) != HAL_DMA_STATE_READY);
	BME280_ReadReg_S16(CALIBRATION_T2, & BME280_Cal_par.T2);
	while(HAL_DMA_GetState(&hdma_i2c1_rx) != HAL_DMA_STATE_READY);
	BME280_ReadReg_S16(CALIBRATION_T3, & BME280_Cal_par.T3);
	while(HAL_DMA_GetState(&hdma_i2c1_rx) != HAL_DMA_STATE_READY);
	BME280_ReadReg_U16(CALIBRATION_P1, & BME280_Cal_par.P1);
	while(HAL_DMA_GetState(&hdma_i2c1_rx) != HAL_DMA_STATE_READY);
	BME280_ReadReg_S16(CALIBRATION_P2, & BME280_Cal_par.P2);
	while(HAL_DMA_GetState(&hdma_i2c1_rx) != HAL_DMA_STATE_READY);
	BME280_ReadReg_S16(CALIBRATION_P3, & BME280_Cal_par.P3);
	while(HAL_DMA_GetState(&hdma_i2c1_rx) != HAL_DMA_STATE_READY);
	BME280_ReadReg_S16(CALIBRATION_P4, & BME280_Cal_par.P4);
	while(HAL_DMA_GetState(&hdma_i2c1_rx) != HAL_DMA_STATE_READY);
	BME280_ReadReg_S16(CALIBRATION_P5, & BME280_Cal_par.P5);
	while(HAL_DMA_GetState(&hdma_i2c1_rx) != HAL_DMA_STATE_READY);
	BME280_ReadReg_S16(CALIBRATION_P6, & BME280_Cal_par.P6);
	while(HAL_DMA_GetState(&hdma_i2c1_rx) != HAL_DMA_STATE_READY);
	BME280_ReadReg_S16(CALIBRATION_P7, & BME280_Cal_par.P7);
	while(HAL_DMA_GetState(&hdma_i2c1_rx) != HAL_DMA_STATE_READY);
	BME280_ReadReg_S16(CALIBRATION_P8, & BME280_Cal_par.P8);
	while(HAL_DMA_GetState(&hdma_i2c1_rx) != HAL_DMA_STATE_READY);
	BME280_ReadReg_S16(CALIBRATION_P9, & BME280_Cal_par.P9);
	while(HAL_DMA_GetState(&hdma_i2c1_rx) != HAL_DMA_STATE_READY);
	BME280_ReadReg(CALIBRATION_H1, &BME280_Cal_par.H1);
	while(HAL_DMA_GetState(&hdma_i2c1_rx) != HAL_DMA_STATE_READY);
	BME280_ReadReg_S16(CALIBRATION_H2, & BME280_Cal_par.H2);
	while(HAL_DMA_GetState(&hdma_i2c1_rx) != HAL_DMA_STATE_READY);
	BME280_ReadReg(CALIBRATION_H3, &BME280_Cal_par.H3);
	while(HAL_DMA_GetState(&hdma_i2c1_rx) != HAL_DMA_STATE_READY);
	BME280_ReadReg_S16(CALIBRATION_H4, & BME280_Cal_par.H4);
	while(HAL_DMA_GetState(&hdma_i2c1_rx) != HAL_DMA_STATE_READY);
	BME280_ReadReg_S16(CALIBRATION_H5, & BME280_Cal_par.H5);
	while(HAL_DMA_GetState(&hdma_i2c1_rx) != HAL_DMA_STATE_READY);
	BME280_ReadReg(CALIBRATION_H6, &BME280_Cal_par.H6);
	while(HAL_DMA_GetState(&hdma_i2c1_rx) != HAL_DMA_STATE_READY);
	/*uncoment for check all calibration value
	sprintf(uart_string, "Printing calibration parameters:\n\rT1: %d\n\rT2: %d\n\rT3: %d\n\r", BME280_Cal_par.T1, BME280_Cal_par.T2, BME280_Cal_par.T3);
	HAL_UART_Transmit_DMA(&huart1, uart_string, strlen(uart_string));
	sprintf(uart_string, "P1: %d\n\rP2: %d\n\rP3: %d\n\rP4: %d\n\rP5: %d\n\rP6: %d\n\rP7: %d\n\rP8: %d\n\rP9: %d\n\r", BME280_Cal_par.P1, BME280_Cal_par.P2, BME280_Cal_par.P3, BME280_Cal_par.P4, BME280_Cal_par.P5, BME280_Cal_par.P6, BME280_Cal_par.P7, BME280_Cal_par.P8, BME280_Cal_par.P9);
	HAL_UART_Transmit_DMA(&huart1, uart_string, strlen(uart_string));
	sprintf(uart_string, "H1: %d\n\rH2: %d\n\rH3: %d\n\rH4: %d\n\rH5: %d\n\rH6: %d\n\r", BME280_Cal_par.H1, BME280_Cal_par.H2, BME280_Cal_par.H3, BME280_Cal_par.H4, BME280_Cal_par.H6);
	HAL_UART_Transmit_DMA(&huart1, uart_string, strlen(uart_string));
*/
}

