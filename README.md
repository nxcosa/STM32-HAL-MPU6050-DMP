# STM32-HAL-MPU6050-DMP
STM32 HAL MPU6050 官方DMP6.12移植 FIFO 开启中断
STM32 HAL MPU6050 官方DMP6.12移植 FIFO 开启中断 //使用STM32CUBE硬件I2C，MPU6050，开启DMP,FIFO,MPU6050中断

//参考B站“愿意做我的战士吗”大神

//该代码使用串口1，i2c1，外部中断PA1,实际代码根据你自己的修改

//移植过程我录了视频，可以去B站观看 //https://www.bilibili.com/video/BV17h411p7s3?spm_id_from=333.999.0.0

//将文件全部下载下来添加进你自己的项目库目录中，并在C/C++和项目目录内添加此文件夹

//在官方DMP6.12中的...\motion_driver_6.12\mpl libraries\arm\Keil目录内解压属于你内核的lib。我这个用的是M3内核。如果你用的是M4内核，则解压M4，并将解压后的libmplib.lib替换我这个库中的

//宏定义中添加 
```
,MPL_LOG_NDEBUG=1,MPU6050,EMPL,USE_DMP,EMPL_TARGET_STM32F4
```

//usart.c函数最后添加这个，可以使用printf函数

```
#include "stdio.h"

int fputc(int ch,FILE *f) 
{ 
    HAL_UART_Transmit(&huart1,(uint8_t *)&ch,1,1000); return ch; 
} 
```

//main.c中添加
```
#include "mpu6050.h"
```

//MAIN函数中while函数前添加 
```
mpu6050_init();
```

//主函数WHILE循环中添加下面代码 
```
float Pitch,Roll,Yaw;
long Temp; 
MPU6050_Get_Euler_Temputer(&Pitch,&Roll,&Yaw,&Temp);
printf("Pitch : %.4f ",(float)Pitch ); 
printf("Roll : %.4f ",(float)Roll ); 
printf("Yaw : %.4f \r\n",(float)Yaw ); 
```

//中断回调函数,实际外部中断引脚根据你的定义引脚来,这个已经写入了mpu6050.c文件中 
```
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) 
{ 
    if(GPIO_Pin == GPIO_PIN_1)
        { gyro_data_ready_cb(); }
} 
```
