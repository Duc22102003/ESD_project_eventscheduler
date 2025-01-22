/*
 * SON_I2C_MPU6050.h
 *
 *  Created on: Jan 7, 2025
 *      Author: Thai-Son Nguyen
 */

/*
 * Chứa các hàm để đọc giá trị gia tốc từ MPU6050.
 *
 * ### Điều kiện tiên quyết:
 *
 * Setup cổng I2C1 -> I2C cho STM32.
 *
 * Các hàm phải được gọi sau khi đã gọi  `MX_I2C1_Init();`
 *
 * ### Các hàm
 *
 * Xem các hàm và chức năng của các hàm ở phần **INTERFACE** .
 *
 * Không cần quan tâm phần **IMPLEMENTATION** .
 * */

#ifndef INC_SON_I2C_MPU6050_H_
#define INC_SON_I2C_MPU6050_H_

// ******************* DEFINE *************************
//
#define MPU6050_ADDR 0x68 << 1


#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75


//// ******************* INTERFACE **********************

//
//

int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;

int16_t Gyro_X_RAW = 0;
int16_t Gyro_Y_RAW = 0;
int16_t Gyro_Z_RAW = 0;

/**
 * Các biến luu giá trị gia tốc và góc.
 *
 * Lấy giá trị các biến này sau khi gọi làm `MPU6050_Read_Accel`
 * */
 float Ax, Ay, Az, Gx, Gy, Gz;



void MPU6050_Init (void);

void MPU6050_Read_Accel (void);

void MPU6050_Read_Gyro (void );

//// ****************** IMPLEMENTATION ******************

/*
 * Reference to initialized common hi2c1 variable in main.c
 *
 * */
extern I2C_HandleTypeDef hi2c1;

inline void MPU6050_Init (void){
	uint8_t check,Data;
	// check the sensor ID (SEE WHO AM I DATASHEET)
	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR,WHO_AM_I_REG,1, &check, 1, 1000);
	if (check == 104) // the sensor is present
	{
		// setting PWR Registers
		Data = 0;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1,&Data, 1, 1000);
		// var sample rate with SMPLRT_DIV_REG
		Data = 0x07;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);
		// var accelerometer config with ACCEL_CONFIG_REG
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);
		// var Gyro config with GYRO_CONFIG_REG
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);
	}
}


inline void MPU6050_Read_Accel ( void)
{
	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from ACCEL_XOUT_H (0x3B) register
	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, 0x3B, 1, Rec_Data, 6, 1000);

	Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	/*** convert the RAW values into acceleration in 'g'
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 16384.0
	     for more details check ACCEL_CONFIG Register              ****/

	Ax = (float)Accel_X_RAW/16384.0;
	Ay = (float)Accel_Y_RAW/16384.0;
	Az = (float)Accel_Z_RAW/16384.0;
}

inline void MPU6050_Read_Gyro (void )
{
	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from GYRO_XOUT_H register
	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, 0x43, 1, Rec_Data, 6, 1000);

	Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	/*** convert the RAW values into dps (ｰ/s)
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 131.0
	     for more details check GYRO_CONFIG Register              ****/

	Gx = (float)Gyro_X_RAW/131.0;
	Gy = (float)Gyro_Y_RAW/131.0;
	Gz = (float)Gyro_Z_RAW/131.0;
}







#endif /* INC_SON_I2C_MPU6050_H_ */
