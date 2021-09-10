//使用STM32CUBE硬件I2C，MPU6050，开启DMP,FIFO,MPU6050中断 

//参考B站“愿意做我的战士吗”大神

//该代码使用串口1，i2c1，外部中断PA0,实际代码根据你自己的修改


//移植过程我录了视频，可以去B站观看
//https://www.bilibili.com/video/BV17h411p7s3?spm_id_from=333.999.0.0

//将MyCode文件夹全部添加进你自己的项目中，并在C/C++和项目目录内添加此文件夹

//宏定义中添加
,MPL_LOG_NDEBUG=1,MPU6050,EMPL,USE_DMP,EMPL_TARGET_STM32F4

//usart.c函数最后添加这个，可以使用printf函数
#include "stdio.h"

int fputc(int ch,FILE *f)
{
	HAL_UART_Transmit(&huart1,(uint8_t *)&ch,1,1000);
	return ch;
}


//MAIN函数中while函数前添加
mpu6050_init();

//主函数WHILE循环中添加下面代码
		float Pitch,Roll,Yaw;
		long Temp;
		MPU6050_Get_Euler_Temputer(&Pitch,&Roll,&Yaw,&Temp);
		printf("Pitch : %.4f     ",(float)Pitch );
		printf("Roll : %.4f    ",(float)Roll );
		printf("Yaw : %.4f   \r\n",(float)Yaw );

//中断回调函数,实际外部中断引脚根据你的定义引脚来,这个已经写入了mpu6050.c文件中
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_0)
		{
			gyro_data_ready_cb();
		}
}


#以下代码都已经修改好，无需修改

#识别I2C设备
for(uint8_t i= 0;i<255;i++)
		{
			if(HAL_I2C_IsDeviceReady(&hi2c1,i,1,1000) == HAL_OK)
				{printf("%d\r\n",i);}
		}
#识别mpu6050是否在线
uint8_t reg;
		HAL_I2C_Mem_Read(&hi2c1,0xD0,0x75,I2C_MEMADD_SIZE_8BIT,&reg,1,1000);
		printf("%d\r\n",reg);



//下面这些写进MPU6050.C中
/**
  * @brief  写寄存器，这是提供给上层的接口
	* @param  slave_addr: 从机地址
	* @param 	reg_addr:寄存器地址
	* @param len：写入的长度
	*	@param data_ptr:指向要写入的数据
  * @retval 正常为0，不正常为非0
  */
int Sensors_I2C_WriteRegister(unsigned char slave_addr,
                                        unsigned char reg_addr,
                                        unsigned short len, 
                                        unsigned char *data_ptr)
{
	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_I2C_Mem_Write(&hi2c1, slave_addr, reg_addr, I2C_MEMADD_SIZE_8BIT,(uint8_t *)data_ptr, len,1000); 
	/* 检查通讯状态 */
	if(status != HAL_OK)
	{
		/* 总线出错处理 */
		printf("I2C Write Wrong!!\r\n");
		return HAL_ERROR;
	}
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
	{
		
	}
	/* 检查SENSOR是否就绪进行下一次读写操作 */
	while (HAL_I2C_IsDeviceReady(&hi2c1, slave_addr, 50, 1000) == HAL_TIMEOUT);
	/* 等待传输结束 */
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
	{
		
	}
	return status;
}

/**
  * @brief  读寄存器，这是提供给上层的接口
	* @param  slave_addr: 从机地址
	* @param 	reg_addr:寄存器地址
	* @param len：要读取的长度
	*	@param data_ptr:指向要存储数据的指针
  * @retval 正常为0，不正常为非0
  */
int Sensors_I2C_ReadRegister(unsigned char slave_addr,unsigned char reg_addr,unsigned short len,unsigned char *data_ptr)
{
	HAL_StatusTypeDef status = HAL_OK;
	status =HAL_I2C_Mem_Read(&hi2c1,slave_addr,reg_addr,I2C_MEMADD_SIZE_8BIT,(uint8_t *)data_ptr,len,1000);    
	/* 检查通讯状态 */
	if(status != HAL_OK)
	{
		/* 总线出错处理 */
		printf("I2C Read Wrong!!\r\n");
		return HAL_ERROR;
	}
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
	{
		
	}
	/* 检查SENSOR是否就绪进行下一次读写操作 */
	while (HAL_I2C_IsDeviceReady(&hi2c1, slave_addr,  50, 1000) == HAL_TIMEOUT);
	/* 等待传输结束 */
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
	{
		
	}
	return status;
}

/**
  * @brief  获取当前毫秒值
  * @param  存储最新毫秒值的变量
  * @retval 无
  */
int get_tick_count(unsigned long *count)
{
   count[0] = HAL_GetTick();
	return 0;
}

int fputcc(int ch)
{
	HAL_UART_Transmit(&huart1,(uint8_t *)&ch,1,1000);
	return ch;
}



//这个写进MPU6050.H中
int get_tick_count(unsigned long *count);
int Sensors_I2C_ReadRegister(unsigned char slave_addr,unsigned char reg_addr,unsigned short len,unsigned char *data_ptr);
int Sensors_I2C_WriteRegister(unsigned char slave_addr,
                                        unsigned char reg_addr,
                                        unsigned short len, 
                                        unsigned char *data_ptr);



inv_mpu.c中0x68改成0xD0