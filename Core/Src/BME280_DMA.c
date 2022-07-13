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
	while(HAL_DMA_GetState(&hdma_i2c1_rx)!= HAL_DMA_STATE_READY);

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
	while(HAL_DMA_GetState(&hdma_i2c1_rx)!= HAL_DMA_STATE_READY);

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
	printf("cntr meas: %d\r\n", reg_cntl_meas_value);
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

void BME280_SetOversamplingHum(uint8_t oversampling_hum){
	if (oversampling_hum != BME280_OVERSAMPLING_X1 && oversampling_hum != BME280_OVERSAMPLING_X2 && oversampling_hum != BME280_OVERSAMPLING_X4 && oversampling_hum != BME280_OVERSAMPLING_X4 && oversampling_hum != BME280_OVERSAMPLING_X8 && oversampling_hum != BME280_OVERSAMPLING_X16){
		return;
	}
	uint8_t reg_meas;
	BME280_ReadReg(REG_CTRL_MEAS, &reg_meas);
	while(HAL_DMA_GetState(&hdma_i2c1_rx) != HAL_DMA_STATE_READY);
	BME280_WriteReg(REG_CTRL_HUM, &oversampling_hum);
	while(HAL_DMA_GetState(&hdma_i2c1_tx) != HAL_DMA_STATE_READY);
	BME280_WriteReg(REG_CTRL_MEAS, &reg_meas);
}

void BME280_SetOversamplingTemp(uint8_t oversampling_temp){
	if (oversampling_temp != BME280_OVERSAMPLING_X1 && oversampling_temp != BME280_OVERSAMPLING_X2 && oversampling_temp != BME280_OVERSAMPLING_X4 && oversampling_temp != BME280_OVERSAMPLING_X4 && oversampling_temp != BME280_OVERSAMPLING_X8 && oversampling_temp != BME280_OVERSAMPLING_X16){
		return;
	}
	uint8_t current_reg, new_reg;
	BME280_ReadReg(REG_CTRL_MEAS, &current_reg);
	while(HAL_DMA_GetState(&hdma_i2c1_rx) != HAL_DMA_STATE_READY);
	//printf("current_reg: %d\r\n", current_reg);
	new_reg = current_reg & 0b00011111;
	//printf("new_reg: %d\r\n", new_reg);
	new_reg = new_reg | (oversampling_temp<<5);
	//printf("new_reg: %d\r\n", new_reg);
	BME280_WriteReg(REG_CTRL_MEAS, &new_reg);
	while(HAL_DMA_GetState(&hdma_i2c1_tx) != HAL_DMA_STATE_READY);

}
void BME280_SetOversamplingPress(uint8_t oversampling_pres){
	if (oversampling_pres != BME280_OVERSAMPLING_X1 && oversampling_pres != BME280_OVERSAMPLING_X2 && oversampling_pres != BME280_OVERSAMPLING_X4 && oversampling_pres != BME280_OVERSAMPLING_X4 && oversampling_pres != BME280_OVERSAMPLING_X8 && oversampling_pres != BME280_OVERSAMPLING_X16){
		return;
	}
	uint8_t current_reg, new_reg;
	BME280_ReadReg(REG_CTRL_MEAS, &current_reg);
	//printf("current_reg: %d\r\n", current_reg);
	new_reg = current_reg & 0b11100011;
	//printf("new_reg: %d\r\n", new_reg);
	new_reg = new_reg | (oversampling_pres<<2);
	//printf("new_reg: %d\r\n", new_reg);
	HAL_Delay(10);
	BME280_WriteReg(REG_CTRL_MEAS, &new_reg);
}
void BME280_SetMode(uint8_t mode){
	if(mode != BME280_MODE_SLEEP && mode != BME280_MODE_FORCED && mode != BME280_MODE_NORMAL){
		return;
	}
	uint8_t current_reg, new_reg;
	BME280_ReadReg(REG_CTRL_MEAS, &current_reg);
	//printf("current_reg: %d\r\n", current_reg);
	new_reg = current_reg & 0b11111100;
	//printf("new_reg: %d\r\n", new_reg);
	new_reg = new_reg | mode;
	//printf("new_reg: %d\r\n", new_reg);
	HAL_Delay(10);
	BME280_WriteReg(REG_CTRL_MEAS, &new_reg);

}

void BME280_ReadHumidityRAW(int16_t * result){
	BME280_ReadReg_S16(REG_HUM, result);
	*result = be16toword(*result);
}
void BME280_ReadTemperatureRAW(int32_t * result){ //read raw data of ADC sensor and turns over
	BME280_ReadReg_U24(REG_TEMP, result);
	while(HAL_DMA_GetState(&hdma_i2c1_rx) != HAL_DMA_STATE_READY);
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

float BME280_GetTemperature(){
	//return current temperature in float
	float temp = 0;
	int32_t temp_raw;
	int32_t var1;
	int32_t var2;
	BME280_ReadTemperatureRAW(&temp_raw);
	while(HAL_DMA_GetState(&hdma_i2c1_rx) != HAL_DMA_STATE_READY);
	temp_raw >>= 4;
	var1 = ((((temp_raw>>3) - ((int32_t)BME280_Cal_par.T1 <<1))) *	((int32_t)BME280_Cal_par.T2)) >> 11;
	var2 = (((((temp_raw>>4) - ((int32_t)BME280_Cal_par.T1)) *	((temp_raw>>4) - ((int32_t)BME280_Cal_par.T1))) >> 12) *((int32_t)BME280_Cal_par.T3)) >> 14;
	temp_int = var1 + var2;
	temp = (var1 +var2) / 5120.0;
	//printf("Temp = %.3f, %.3f *C\n\r", temper, temp);
	return temp;
}

float BME280_GetPressure(){
	uint32_t pres_int, pres_raw;
	int64_t p;
	int64_t var1, var2;
	float pres;
	BME280_GetTemperature();
	BME280_ReadReg_BE_U24(REG_PRESS, &pres_raw);
	while(HAL_DMA_GetState(&hdma_i2c1_rx) != HAL_DMA_STATE_READY);
	pres_raw = pres_raw>>4;

	var1 = ((int64_t) temp_int) - 128000;
	var2 = var1 * var1 * (int64_t)BME280_Cal_par.P6;
	var2 = var2 + ((var1 * (int64_t)BME280_Cal_par.P5) << 17);
	var2 = var2 + ((int64_t)BME280_Cal_par.P4 << 35);
	var1 = ((var1 * var1 * (int64_t)BME280_Cal_par.P3) >> 8) + ((var1 * (int64_t)BME280_Cal_par.P2) << 12);
	var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)BME280_Cal_par.P1) >> 33;
	if (var1 == 0) {
	  return 0; // avoid exception caused by division by zero
	}
	p = 1048576 - pres_raw;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = (((int64_t)BME280_Cal_par.P9) * (p >> 13) * (p >> 13)) >> 25;
	var2 = (((int64_t)BME280_Cal_par.P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + ((int64_t)BME280_Cal_par.P7 << 4);
	pres_int = ((p >> 8) * 1000) + (((p & 0xff) * 390625) / 100000);
	pres = pres_int / 100.0f;
	return pres;
}

float BME280_GetHumidity(){
	int32_t v_x1_u32r;
	int32_t adc_H;
	float hum_float;
	BME280_ReadHumidityRAW(&adc_H);
	v_x1_u32r = (temp_int - ((int32_t)76800));
	v_x1_u32r = (((((adc_H << 14) - (((int32_t)BME280_Cal_par.H4) << 20) -
				(((int32_t)BME280_Cal_par.H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) *
				(((((((v_x1_u32r * ((int32_t)BME280_Cal_par.H6)) >> 10) *
				(((v_x1_u32r * ((int32_t)BME280_Cal_par.H3)) >> 11) + ((int32_t)32768))) >> 10) +
				((int32_t)2097152)) * ((int32_t)BME280_Cal_par.H2) + 8192) >> 14));
	v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
				((int32_t)BME280_Cal_par.H1)) >> 4));
	v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
	v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;
	hum_float = (v_x1_u32r>>12);
	hum_float /= 1024.0f;
	return hum_float;
}
