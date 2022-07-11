//нестабільна версія

#include "BME280.h"

extern I2C_HandleTypeDef BME280_I2C_HANDLER;
extern UART_HandleTypeDef huart1;
extern char srt1[100];

BME280_Calibrate_parametrs BME280_Cal_par;

int32_t temp_int;

uint8_t BME280_Init(){
	LED_ON;
	HAL_I2C_Init(&BME280_I2C_HANDLER);
	uint8_t id_of_chip = BME280_GetID();
	BME280_ReadCalibration();
	BME280_SetOversampling(BME280_OVERSAMPLING_X8, BME280_OVERSAMPLING_X4, BME280_OVERSAMPLING_X4, BME280_MODE_NORMAL);
	BME280_SetConfig(BME280_STANDBY_TIME_10, BME280_FILTER_16, BME280_3WIRE_SPI_OFF);
	return id_of_chip;
}

void Error(){
	LED_OFF;
}
//===========Reading data from I2C==============================
uint8_t I2Cx_ReadData(uint16_t Addr, uint8_t Reg)
{
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t value = 0;
	status = HAL_I2C_Mem_Read(&BME280_I2C_HANDLER, Addr, Reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 0x10000);
	//printf("%d\n\r", status);
	if (status != HAL_OK) Error();
	return value;
}
uint8_t BME280_ReadReg(uint8_t Reg){
	uint8_t res = 0;
	res = I2Cx_ReadData(BME280_ADDR, Reg);
	return res;
}
void I2Cx_ReadData16(uint16_t Addr, uint8_t Reg, uint16_t *Value)
{
	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_I2C_Mem_Read(&BME280_I2C_HANDLER, Addr, Reg, I2C_MEMADD_SIZE_8BIT, (uint8_t *)Value, 2, 0x10000);
	if (status != HAL_OK) Error();
}
void I2Cx_ReadData24(uint16_t Addr, uint8_t Reg, uint32_t *Value)
{
	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_I2C_Mem_Read(&BME280_I2C_HANDLER, Addr, Reg, I2C_MEMADD_SIZE_8BIT, (uint8_t *)Value, 3, 0x10000);
	if (status != HAL_OK) Error();
}
//==========Writing data to I2C=============================
void I2Cx_WriteData(uint16_t Addr, uint8_t Reg, uint8_t Value){
	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_I2C_Mem_Write(&BME280_I2C_HANDLER, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT,  &Value, 1, 0x10000);
	if (status != HAL_OK) Error();
}
//==========Writing register to BME280=====================
void BME280_WriteReg(uint8_t Reg, uint8_t Value){
	I2Cx_WriteData(BME280_ADDR, Reg, Value);
}

//==========Reading of diferent registers in BME280===========
void BME280_ReadReg_S16(uint8_t Reg, int16_t *Value){
	I2Cx_ReadData16(BME280_ADDR,Reg, (uint16_t*) Value);
}
void BME280_ReadReg_S24(uint8_t Reg, int32_t *Value){
	I2Cx_ReadData24(BME280_ADDR,Reg, (uint32_t*) Value);
	*(int32_t *) Value &= 0x00FFFFFF;
}
void BME280_ReadReg_U16(uint8_t Reg, uint16_t *Value){
	I2Cx_ReadData16(BME280_ADDR,Reg, Value);
}
void BME280_ReadReg_U24(uint8_t Reg, uint32_t *Value){
	I2Cx_ReadData24(BME280_ADDR,Reg,  Value);
	*(uint32_t *) Value &= 0x00FFFFFF;
}
void BME280_ReadReg_BE_U24(uint8_t Reg, uint32_t *Value)
{
  I2Cx_ReadData24(BME280_ADDR,Reg,Value);
  *(uint32_t *) Value = be24toword(*(uint32_t *) Value) & 0x00FFFFFF;
}
float BME280_GetPressure(){
	uint32_t pres_int, pres_raw;
	int64_t p;
	int64_t var1, var2;
	float pres;
	BME280_GetTemperature();
	 BME280_ReadReg_BE_U24(REG_PRESS, &pres_raw);
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

int16_t BME280_ReadHumidityRAW(){
	int16_t hum_value;
	BME280_ReadReg_S16(REG_HUM, &hum_value);
	hum_value = be16toword(hum_value);
	return hum_value;
}
float BME280_GetHumidity(){
	int32_t v_x1_u32r;
	int32_t adc_H;
	float hum_float;
	adc_H = (int32_t)BME280_ReadHumidityRAW();
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
uint8_t BME280_GetID(void){
	//Reading ID of chip, can be used for check module
	uint8_t id = 0;
	id = BME280_ReadReg(REG_ID);
	if(id != 0x60 && id != 0x56 && id != 0x57 && id != 0x58){
		printf("Incorrect ID\r\n");
	}
	return id;
}

uint8_t BME280_GetStatus()
{
	//Reading status of sensor, see #define STATUS...
	uint8_t status = 0;
	status = BME280_ReadReg(REG_STATUS);
	return status;
}

int32_t BME280_ReadTemperatureRAW(){ //read raw data of ADC sensor and turns over
	int32_t temp_raw = 0;
	BME280_ReadReg_U24(REG_TEMP, &temp_raw);
	temp_raw = be24toword(temp_raw);
	return (int32_t)temp_raw;
}
int32_t BME280_ReadPressureRAW(){//read raw data of ADC sensor and turns over
	uint32_t pres_raw = 0;
	BME280_ReadReg_U24(REG_HUM, (uint32_t *)pres_raw);
	pres_raw = be24toword(pres_raw);
	return (int32_t) pres_raw;
}


float BME280_GetTemperature(){
	//return current temperature in float
	float temp = 0;
	int32_t temp_raw;
	int32_t var1;
	int32_t var2;
	temp_raw = BME280_ReadTemperatureRAW();
	temp_raw >>= 4;
	var1 = ((((temp_raw>>3) - ((int32_t)BME280_Cal_par.T1 <<1))) *	((int32_t)BME280_Cal_par.T2)) >> 11;
	var2 = (((((temp_raw>>4) - ((int32_t)BME280_Cal_par.T1)) *	((temp_raw>>4) - ((int32_t)BME280_Cal_par.T1))) >> 12) *((int32_t)BME280_Cal_par.T3)) >> 14;
	temp_int = var1 + var2;
	temp = (var1 +var2) / 5120.0;
	//printf("Temp = %.3f, %.3f *C\n\r", temper, temp);
	return temp;
}


uint8_t BME280_SetOversampling(uint8_t oversampling_temp, uint8_t oversampling_pres, uint8_t oversampling_hum, uint8_t mode)
{
	//Writing a measuring mode, use defined macros BME280_OVERSAMPLING_X... and BME280_MODE_...
	HAL_StatusTypeDef res;
	res = HAL_OK;
	if (oversampling_hum != BME280_OVERSAMPLING_X1 && oversampling_hum != BME280_OVERSAMPLING_X2 && oversampling_hum != BME280_OVERSAMPLING_X4 && oversampling_hum != BME280_OVERSAMPLING_X4 && oversampling_hum != BME280_OVERSAMPLING_X8 && oversampling_hum != BME280_OVERSAMPLING_X16){
		res = HAL_ERROR;
		return res;
	}
	if (oversampling_temp != BME280_OVERSAMPLING_X1 && oversampling_temp != BME280_OVERSAMPLING_X2 && oversampling_temp != BME280_OVERSAMPLING_X4 && oversampling_temp != BME280_OVERSAMPLING_X4 && oversampling_temp != BME280_OVERSAMPLING_X8 && oversampling_temp != BME280_OVERSAMPLING_X16){
		res = HAL_ERROR;
		return res;
	}
	if (oversampling_pres != BME280_OVERSAMPLING_X1 && oversampling_pres != BME280_OVERSAMPLING_X2 && oversampling_pres != BME280_OVERSAMPLING_X4 && oversampling_pres != BME280_OVERSAMPLING_X4 && oversampling_pres != BME280_OVERSAMPLING_X8 && oversampling_pres != BME280_OVERSAMPLING_X16){
		res = HAL_ERROR;
		return res;
	}
	if(mode != BME280_MODE_SLEEP && mode != BME280_MODE_FORCED && mode != BME280_MODE_NORMAL){
		res = HAL_ERROR;
		return res;
	}
	BME280_WriteReg(REG_CTRL_HUM, oversampling_hum);
	uint8_t reg_cntl_meas_value;
	reg_cntl_meas_value = ((oversampling_temp)<<5)|((oversampling_pres)<<2)|(mode);
	printf("ctrl_meas: %d\r\n", reg_cntl_meas_value);
	BME280_WriteReg(REG_CTRL_MEAS, reg_cntl_meas_value);
	return res;
}

void BME280_SetOversamplingHum(uint8_t oversampling_hum){
	if (oversampling_hum != BME280_OVERSAMPLING_X1 && oversampling_hum != BME280_OVERSAMPLING_X2 && oversampling_hum != BME280_OVERSAMPLING_X4 && oversampling_hum != BME280_OVERSAMPLING_X4 && oversampling_hum != BME280_OVERSAMPLING_X8 && oversampling_hum != BME280_OVERSAMPLING_X16){
		return;
	}
	uint8_t reg_meas = BME280_ReadReg(REG_CTRL_MEAS);
	HAL_Delay(10);
	BME280_WriteReg(REG_CTRL_HUM, oversampling_hum);
	HAL_Delay(10);
	BME280_WriteReg(REG_CTRL_MEAS, reg_meas);
}

void BME280_SetOversamplingTemp(uint8_t oversampling_temp){
	if (oversampling_temp != BME280_OVERSAMPLING_X1 && oversampling_temp != BME280_OVERSAMPLING_X2 && oversampling_temp != BME280_OVERSAMPLING_X4 && oversampling_temp != BME280_OVERSAMPLING_X4 && oversampling_temp != BME280_OVERSAMPLING_X8 && oversampling_temp != BME280_OVERSAMPLING_X16){
		return;
	}
	uint8_t current_reg, new_reg;
	current_reg = BME280_ReadReg(REG_CTRL_MEAS);
	//printf("current_reg: %d\r\n", current_reg);
	new_reg = current_reg & 0b00011111;
	//printf("new_reg: %d\r\n", new_reg);
	new_reg = new_reg | (oversampling_temp<<5);
	//printf("new_reg: %d\r\n", new_reg);
	HAL_Delay(10);
	BME280_WriteReg(REG_CTRL_MEAS, new_reg);
}
void BME280_SetOversamplingPress(uint8_t oversampling_pres){
	if (oversampling_pres != BME280_OVERSAMPLING_X1 && oversampling_pres != BME280_OVERSAMPLING_X2 && oversampling_pres != BME280_OVERSAMPLING_X4 && oversampling_pres != BME280_OVERSAMPLING_X4 && oversampling_pres != BME280_OVERSAMPLING_X8 && oversampling_pres != BME280_OVERSAMPLING_X16){
		return;
	}
	uint8_t current_reg, new_reg;
	current_reg = BME280_ReadReg(REG_CTRL_MEAS);
	//printf("current_reg: %d\r\n", current_reg);
	new_reg = current_reg & 0b11100011;
	//printf("new_reg: %d\r\n", new_reg);
	new_reg = new_reg | (oversampling_pres<<2);
	//printf("new_reg: %d\r\n", new_reg);
	HAL_Delay(10);
	BME280_WriteReg(REG_CTRL_MEAS, new_reg);
}
void BME280_SetMode(uint8_t mode){
	if(mode != BME280_MODE_SLEEP && mode != BME280_MODE_FORCED && mode != BME280_MODE_NORMAL){
		return;
	}
	uint8_t current_reg, new_reg;
	current_reg = BME280_ReadReg(REG_CTRL_MEAS);
	//printf("current_reg: %d\r\n", current_reg);
	new_reg = current_reg & 0b11111100;
	//printf("new_reg: %d\r\n", new_reg);
	new_reg = new_reg | mode;
	//printf("new_reg: %d\r\n", new_reg);
	HAL_Delay(10);
	BME280_WriteReg(REG_CTRL_MEAS, new_reg);

}
uint8_t BME280_GetOversamplingMode(uint8_t *array){
	//write array oversampling temperature, pressure, humidity, and mode
	uint8_t ovrs_hum, ovrs_temp, ovrs_pres, mode;
	uint8_t value_ctrl_meas;
	ovrs_hum = BME280_ReadReg(REG_CTRL_HUM);
	value_ctrl_meas = BME280_ReadReg(REG_CTRL_MEAS);
	ovrs_temp = (value_ctrl_meas&0b11100000)>>5;
	ovrs_pres = (value_ctrl_meas&0b00011100)>>2;
	mode = value_ctrl_meas&0b00000011;
	*array = ovrs_temp;
	*(array + sizeof(uint8_t)) = ovrs_pres;
	*(array + 2 * sizeof(uint8_t)) = ovrs_hum;
	*(array + 3 * sizeof(uint8_t)) = mode;
	//printf("temp: %d\r\npres: %d\r\nhum: %d\r\nmode: %d\r\n", ovrs_temp, ovrs_pres, ovrs_hum, mode);
	return value_ctrl_meas;
}

void BME280_SoftReset(){
	//SoftReset the sensor
	BME280_WriteReg(REG_RES, RESET_VALUE);
}

void BME280_ReadCalibration(){
	//function read calibration_data from sensor, needed for correct measuring of all parameters
	BME280_ReadReg_U16(CALIBRATION_T1, & BME280_Cal_par.T1);
	BME280_ReadReg_S16(CALIBRATION_T2, & BME280_Cal_par.T2);
	BME280_ReadReg_S16(CALIBRATION_T3, & BME280_Cal_par.T3);
	BME280_ReadReg_U16(CALIBRATION_P1, & BME280_Cal_par.P1);
	BME280_ReadReg_S16(CALIBRATION_P2, & BME280_Cal_par.P2);
	BME280_ReadReg_S16(CALIBRATION_P3, & BME280_Cal_par.P3);
	BME280_ReadReg_S16(CALIBRATION_P4, & BME280_Cal_par.P4);
	BME280_ReadReg_S16(CALIBRATION_P5, & BME280_Cal_par.P5);
	BME280_ReadReg_S16(CALIBRATION_P6, & BME280_Cal_par.P6);
	BME280_ReadReg_S16(CALIBRATION_P7, & BME280_Cal_par.P7);
	BME280_ReadReg_S16(CALIBRATION_P8, & BME280_Cal_par.P8);
	BME280_ReadReg_S16(CALIBRATION_P9, & BME280_Cal_par.P9);
	BME280_Cal_par.H1 = (unsigned char) BME280_ReadReg(CALIBRATION_H1);
	BME280_ReadReg_S16(CALIBRATION_H2, & BME280_Cal_par.H2);
	BME280_Cal_par.H3 = (unsigned char) BME280_ReadReg(CALIBRATION_H3);
	BME280_ReadReg_S16(CALIBRATION_H4, & BME280_Cal_par.H4);
	BME280_ReadReg_S16(CALIBRATION_H5, & BME280_Cal_par.H5);
	BME280_Cal_par.H6 = (char) BME280_ReadReg(CALIBRATION_H6);
	/* uncoment for check all calibration value
	printf("Printing calibration parameters:\n\rT1: %d\n\rT2: %d\n\rT3: %d\n\r", BME280_Cal_par.T1, BME280_Cal_par.T2, BME280_Cal_par.T3);
	printf("P1: %d\n\rP2: %d\n\rP3: %d\n\rP4: %d\n\rP5: %d\n\rP6: %d\n\rP7: %d\n\rP8: %d\n\rP9: %d\n\r", BME280_Cal_par.P1, BME280_Cal_par.P2, BME280_Cal_par.P3, BME280_Cal_par.P4, BME280_Cal_par.P5, BME280_Cal_par.P6, BME280_Cal_par.P7, BME280_Cal_par.P8, BME280_Cal_par.P9);
	printf("H1: %d\n\rH2: %d\n\rH3: %d\n\rH4: %d\n\rH5: %d\n\rH6: %d\n\r", BME280_Cal_par.H1, BME280_Cal_par.H2, BME280_Cal_par.H3, BME280_Cal_par.H4, BME280_Cal_par.H6);
	*/
}

void BME280_SetStandbyTime(uint8_t standby_time){
	switch (standby_time){
	case BME280_STANDBY_TIME_05:break;
	case BME280_STANDBY_TIME_10:break;
	case BME280_STANDBY_TIME_20:break;
	case BME280_STANDBY_TIME_62:break;
	case BME280_STANDBY_TIME_125:break;
	case BME280_STANDBY_TIME_250:break;
	case BME280_STANDBY_TIME_500:break;
	case BME280_STANDBY_TIME_1000:break;
	default: return;
	}
	uint8_t current_reg, new_reg;
	current_reg = BME280_ReadReg(REG_CONFIG);
	new_reg = current_reg & 0b00011111;
	new_reg = new_reg | (standby_time << 5);
	BME280_WriteReg(REG_CONFIG, new_reg);
}
void BME280_SetFilter(uint8_t filter_coeficient){
	switch (filter_coeficient){
	case BME280_FILTER_OFF:break;
	case BME280_FILTER_2:break;
	case BME280_FILTER_4:break;
	case BME280_FILTER_8:break;
	case BME280_FILTER_16:break;
	default: return;
	}
	uint8_t current_reg, new_reg;
	current_reg = BME280_ReadReg(REG_CONFIG);
	new_reg = current_reg & 0b11100011;
	new_reg = new_reg | (filter_coeficient << 2);
	BME280_WriteReg(REG_CONFIG, new_reg);
}
void BME280_SPI_3Wire(uint8_t state){
	if (state > 0){
		state = 1;
	}
	uint8_t current_reg, new_reg;
	current_reg = BME280_ReadReg(REG_CONFIG);
	new_reg = current_reg & 0b11111110;
	new_reg = new_reg | (state);
	BME280_WriteReg(REG_CONFIG, new_reg);
}

uint8_t BME280_SetConfig(uint8_t standby_time, uint8_t filter_coeficient, uint8_t spi_3wire_mode){
	//set standby time, coefficient of filtration, and enable the 3-wire SPI
	uint8_t reg_value;
	reg_value = (standby_time << 5)|(filter_coeficient<<2)|(spi_3wire_mode);
	//printf("reg_value: %d\n\r", reg_value);
	BME280_WriteReg(REG_CONFIG, reg_value);
	return 0;
}
void BME280_GetConfig(uint8_t *array){
	uint8_t standby_time, filter, spi3wire;
	uint8_t reg_value;
	reg_value = BME280_ReadReg(REG_CONFIG);
	standby_time = (reg_value & 0b11100000)>>5;
	filter = (reg_value & 0b00011100)>>2;
	spi3wire = (reg_value & 0b00000001);
	*array = standby_time;
	*(array + sizeof(uint8_t)) = filter;
	*(array + 2 * sizeof(uint8_t)) = spi3wire;
}
