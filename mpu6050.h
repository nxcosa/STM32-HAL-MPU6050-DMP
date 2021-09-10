
/* Includes ------------------------------------------------------------------*/

#include <stdio.h>

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void gyro_data_ready_cb(void);

int get_tick_count(unsigned long *count);
int Sensors_I2C_ReadRegister(unsigned char slave_addr,unsigned char reg_addr,unsigned short len,unsigned char *data_ptr);
int Sensors_I2C_WriteRegister(unsigned char slave_addr,
                                        unsigned char reg_addr,
                                        unsigned short len, 
                                        unsigned char *data_ptr);
																				
void mpu6050_init(void);
int fputcc(int ch);
void MPU6050_Get_Euler_Temputer(float *Pitch,float *Roll,float *Yaw,long *temperature);
