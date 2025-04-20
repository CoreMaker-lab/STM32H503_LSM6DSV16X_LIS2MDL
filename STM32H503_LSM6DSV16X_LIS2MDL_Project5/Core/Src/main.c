/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "icache.h"
#include "memorymap.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "lsm6dsv16x_reg.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int fputc(int ch, FILE *f){
	HAL_UART_Transmit(&huart1 , (uint8_t *)&ch, 1, 0xFFFF);
	return ch;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define SENSOR_BUS hi2c1


/* Private macro -------------------------------------------------------------*/
/*
 * Select FIFO samples watermark, max value is 512
 * in FIFO are stored acc, gyro and timestamp samples
 */
#define BOOT_TIME         10
#define FIFO_WATERMARK    32

/* Private variables ---------------------------------------------------------*/
static uint8_t whoamI;
static uint8_t tx_buffer[1000];

/* Private variables ---------------------------------------------------------*/
static lsm6dsv16x_fifo_sflp_raw_t fifo_sflp;

/* Extern variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/*
 *   WARNING:
 *   Functions declare in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void tx_com( uint8_t *tx_buffer, uint16_t len );
static void platform_delay(uint32_t ms);
static void platform_init(void *handle);

static float_t npy_half_to_float(uint16_t h)
{
    union { float_t ret; uint32_t retbits; } conv;
    conv.retbits = lsm6dsv16x_from_f16_to_f32(h);
    return conv.ret;
}

static void sflp2q(float_t quat[4], uint16_t sflp[3])
{
  float_t sumsq = 0;

  quat[0] = npy_half_to_float(sflp[0]);
  quat[1] = npy_half_to_float(sflp[1]);
  quat[2] = npy_half_to_float(sflp[2]);

  for (uint8_t i = 0; i < 3; i++)
    sumsq += quat[i] * quat[i];

  if (sumsq > 1.0f) {
    float_t n = sqrtf(sumsq);
    quat[0] /= n;
    quat[1] /= n;
    quat[2] /= n;
    sumsq = 1.0f;
  }

  quat[3] = sqrtf(1.0f - sumsq);
}



uint8_t fifo_flag = 0;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_ICACHE_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	printf("HELLO!\n");
  HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SA0_GPIO_Port, SA0_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(CS2_GPIO_Port, CS2_Pin, GPIO_PIN_SET);
	HAL_Delay(100);	
		
	
	
  lsm6dsv16x_fifo_status_t fifo_status;
  stmdev_ctx_t dev_ctx;
  lsm6dsv16x_reset_t rst;
  lsm6dsv16x_sflp_gbias_t gbias;

  /* Uncomment to configure INT 1 */
  //lsm6dsv16x_pin_int1_route_t int1_route;
  /* Uncomment to configure INT 2 */
  //lsm6dsv16x_pin_int2_route_t int2_route;
  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
  dev_ctx.handle = &SENSOR_BUS;

  /* Init test platform */
//  platform_init(dev_ctx.handle);
  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);

  /* Check device ID */
  lsm6dsv16x_device_id_get(&dev_ctx, &whoamI);
	printf("LSM6DSV16X_ID=0x%x,whoamI=0x%x",LSM6DSV16X_ID,whoamI);
  if (whoamI != LSM6DSV16X_ID)
    while (1);

  /* Restore default configuration */
  lsm6dsv16x_reset_set(&dev_ctx, LSM6DSV16X_RESTORE_CTRL_REGS);
  do {
    lsm6dsv16x_reset_get(&dev_ctx, &rst);
  } while (rst != LSM6DSV16X_READY);

  /* Enable Block Data Update */
  lsm6dsv16x_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  /* Set full scale */
  lsm6dsv16x_xl_full_scale_set(&dev_ctx, LSM6DSV16X_4g);
  lsm6dsv16x_gy_full_scale_set(&dev_ctx, LSM6DSV16X_2000dps);

  /*
   * Set FIFO watermark (number of unread sensor data TAG + 6 bytes
   * stored in FIFO) to FIFO_WATERMARK samples
   */
  lsm6dsv16x_fifo_watermark_set(&dev_ctx, FIFO_WATERMARK);

  /* Set FIFO batch of sflp data */
  fifo_sflp.game_rotation = 1;
  fifo_sflp.gravity = 1;
  fifo_sflp.gbias = 1;
  lsm6dsv16x_fifo_sflp_batch_set(&dev_ctx, fifo_sflp);

  /* Set FIFO mode to Stream mode (aka Continuous Mode) */
  lsm6dsv16x_fifo_mode_set(&dev_ctx, LSM6DSV16X_STREAM_MODE);

  /* Set Output Data Rate */
  lsm6dsv16x_xl_data_rate_set(&dev_ctx, LSM6DSV16X_ODR_AT_30Hz);
  lsm6dsv16x_gy_data_rate_set(&dev_ctx, LSM6DSV16X_ODR_AT_30Hz);
  lsm6dsv16x_sflp_data_rate_set(&dev_ctx, LSM6DSV16X_SFLP_30Hz);

  lsm6dsv16x_sflp_game_rotation_set(&dev_ctx, PROPERTY_ENABLE);

  /*
   * here application may initialize offset with latest values
   * calculated from previous run and saved to non volatile memory.
   */
  gbias.gbias_x = 0.0f;
  gbias.gbias_y = 0.0f;
  gbias.gbias_z = 0.0f;
  lsm6dsv16x_sflp_game_gbias_set(&dev_ctx, &gbias);



	lsm6dsv16x_pin_int_route_t pin_int;	
  pin_int.fifo_th = PROPERTY_ENABLE;
  lsm6dsv16x_pin_int1_route_set(&dev_ctx, &pin_int);


	float Yaw,Pitch,Roll;  //偏航角，俯仰角，翻滚角
	int16_t	acc_int16[3]	={0,0,0};
	int16_t	gyr_int16[3]		={0,0,0};	
	float acc[3] = {0};
	float gyr[3] = {0};	

	uint8_t data[21]={0};
	data[0]=0xAB;//帧头
	data[1]=0xFD;//源地址
	data[2]=0xFE;//目标地址		
	data[3]=0x01;//功能码ID	
	data[4]=0x0D;//数据长度LEN
	data[5]=0x00;//数据长度LEN 13

	uint8_t sumcheck = 0;
	uint8_t addcheck = 0;		


	int16_t angular_rate_raw[3]={0,0,0};	//pitch,roll,yaw
	uint8_t data_angular_rate_raw[16]={0};
	data_angular_rate_raw[0]=0xAB;//帧头
	data_angular_rate_raw[1]=0xFD;//源地址
	data_angular_rate_raw[2]=0xFE;//目标地址		
	data_angular_rate_raw[3]=0x03;//功能码ID	
	data_angular_rate_raw[4]=0x08;//数据长度LEN
	data_angular_rate_raw[5]=0x00;//数据长度LEN 8
	data_angular_rate_raw[6]=0x01;//mode = 1	

	data_angular_rate_raw[13]=0x00;//FUSION _STA：融合状态		


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(fifo_flag==1)
		{
			fifo_flag=0;		
		
    uint16_t num = 0;

    /* Read watermark flag */
    lsm6dsv16x_fifo_status_get(&dev_ctx, &fifo_status);

    if (fifo_status.fifo_th == 1) {
      num = fifo_status.fifo_level;

      printf("-- FIFO num %d \r\n", num);

      while (num--) {
        lsm6dsv16x_fifo_out_raw_t f_data;
        uint8_t *axis;
        float_t quat[4];
        float_t gravity_mg[3];
        float_t gbias_mdps[3];

        /* Read FIFO sensor value */
        lsm6dsv16x_fifo_out_raw_get(&dev_ctx, &f_data);

        switch (f_data.tag) {
        case LSM6DSV16X_SFLP_GYROSCOPE_BIAS_TAG:
//          axis = &f_data.data[0];
//          gbias_mdps[0] = lsm6dsv16x_from_fs125_to_mdps(axis[0] | (axis[1] << 8));
//          gbias_mdps[1] = lsm6dsv16x_from_fs125_to_mdps(axis[2] | (axis[3] << 8));
//          gbias_mdps[2] = lsm6dsv16x_from_fs125_to_mdps(axis[4] | (axis[5] << 8));
//          printf("GBIAS [mdps]:%4.2f\t%4.2f\t%4.2f\r\n",
//                         (double_t)gbias_mdps[0], (double_t)gbias_mdps[1], (double_t)gbias_mdps[2]);
          break;
        case LSM6DSV16X_SFLP_GRAVITY_VECTOR_TAG:
//          axis = &f_data.data[0];
//          gravity_mg[0] = lsm6dsv16x_from_sflp_to_mg(axis[0] | (axis[1] << 8));
//          gravity_mg[1] = lsm6dsv16x_from_sflp_to_mg(axis[2] | (axis[3] << 8));
//          gravity_mg[2] = lsm6dsv16x_from_sflp_to_mg(axis[4] | (axis[5] << 8));
//          printf("Gravity [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
//                         (double_t)gravity_mg[0], (double_t)gravity_mg[1], (double_t)gravity_mg[2]);

          break;
        case LSM6DSV16X_SFLP_GAME_ROTATION_VECTOR_TAG:
          sflp2q(quat, (uint16_t *)&f_data.data[0]);
//          printf("Game Rotation \tX: %2.3f\tY: %2.3f\tZ: %2.3f\tW: %2.3f\r\n",
//                  (double_t)quat[0], (double_t)quat[1], (double_t)quat[2], (double_t)quat[3]);
				
					float sx=quat[1];  
					float sy=quat[2];  
					float sz=quat[0];  
					float sw=quat[3];
				
					if (sw< 0.0f) 
					{
						sx*=-1.0f;
						sy*=-1.0f;
						sz*=-1.0f;
						sw*=-1.0f;
					}
				
					float sqx = sx * sx;
					float sqy = sy * sy;
					float sqz = sz * sz;
					float euler[3];
					euler[0] = -atan2f(2.0f* (sy*sw+sx*sz), 1.0f-2.0f*(sqy+sqx));
					euler[1] = -atan2f(2.0f * (sx*sy+sz*sw),1.0f-2.0f*(sqx+sqz));
					euler[2] = -asinf(2.0f* (sx*sw-sy*sz));
				
					if (euler[0] <0.0f)
						euler[0] +=2.0f*3.1415926;
					
					for(uint8_t i=0; i<3; i++){
							euler[i] = 57.29578 * (euler[i]);
					}
					
//					printf("euler[0]=%f,euler[1]=%f,euler[2]=%f\n",euler[0],euler[1],euler[2]);
				Roll=euler[2];
				Pitch=euler[1];
				Yaw=euler[0];					
					
				int16_t	Roll_int16;
				int16_t	Pitch_int16;					
				int16_t	Yaw_int16;	

				Roll_int16 = (int16_t)(Roll);
				Pitch_int16 = (int16_t)(Pitch);
				Yaw_int16 = (int16_t)(Yaw);		


				Roll_int16=Roll_int16*100;
				Pitch_int16=Pitch_int16*100;
				Yaw_int16=Yaw_int16*100-18000;
	
//				Roll=Roll*100;
//				Pitch=Pitch*100;
//				Yaw=Yaw*100;
				data_angular_rate_raw[7]=Roll_int16>>8;//roll
				data_angular_rate_raw[8]=Roll_int16;
				data_angular_rate_raw[9]=Pitch_int16>>8;//pitch
				data_angular_rate_raw[10]=Pitch_int16;
				data_angular_rate_raw[11]=Yaw_int16>>8;//yaw
				data_angular_rate_raw[12]=Yaw_int16;
									
				data_angular_rate_raw[13]=0;
				sumcheck = 0;
				addcheck = 0;
				for(uint16_t i=0; i < 14; i++)
				{
				sumcheck += data_angular_rate_raw[i]; //从帧头开始，对每一字节进行求和，直到 DATA 区结束
				addcheck += sumcheck; //每一字节的求和操作，进行一次 sumcheck 的累加
				}
				data_angular_rate_raw[14]=sumcheck;
				data_angular_rate_raw[15]=addcheck;					
					
				HAL_UART_Transmit(&huart1 , (uint8_t *)&data_angular_rate_raw, 16, 0xFFFF);	
//				printf("Roll=%.2f,Pitch=%.2f,Yaw=%.2f\n",Roll,Pitch,Yaw)					
					
					
          break;
        default:
         break;
        }
      }

      printf("------ \r\n\r\n");

    }		
		
		
		
	}	
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_CSI;
  RCC_OscInitStruct.CSIState = RCC_CSI_ON;
  RCC_OscInitStruct.CSICalibrationValue = RCC_CSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLL1_SOURCE_CSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 125;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1_VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1_VCORANGE_WIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the programming delay
  */
  __HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_2);
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_0)
	{
		fifo_flag=1;
		}	
}

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{
  HAL_I2C_Mem_Write(handle, LSM6DSV16X_I2C_ADD_L, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
  return 0;
}


static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
  HAL_I2C_Mem_Read(handle, LSM6DSV16X_I2C_ADD_L, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  return 0;
}



static void platform_delay(uint32_t ms)
{
  HAL_Delay(ms);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
