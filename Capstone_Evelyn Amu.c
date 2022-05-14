/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "string.h"
#include "stdlib.h"
#include "stdio.h"


uint8_t flag = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	flag = 1;
}

int nmea0183_checksum(char *msg) {
    	int checksum = 0;
        int j = 0;
        for (j = 1; j < strlen(msg); j++) {
        	checksum = checksum ^ (unsigned) msg[j];
            }
        return checksum;
}



 I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
DMA_HandleTypeDef hdma_uart4_rx;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_UART4_Init(void);
static void MX_I2C1_Init(void);
static void MX_UART5_Init(void);

void GSM_init();
void GSM_senddata(char *APIkey, int num_of_fields, float value[2]);
void GSM_SMS();
void GPGLL_Sequence();
extern void initialise_monitor_handles();

char *token;
char nmeaseq[80];
char *rawSum;
char smNmbr[3];
char *timestamp;
char *NorthorSouth;
char *EastorWest;
char *lat;
char *longi;
uint8_t buff[255];
char buffStr[255];
uint8_t counter = 0;
char hH[2]; // hours
char mM[2]; // minutes
char sS[2]; // seconds
char latDg[2];
char lonDg[3];
char lat_dec[30];
char lon_dec[30];
float resultant_lat;
float resultant_lon;

char mobileNumber[] = "+233200940841";
char ATcommand[80];
uint8_t AT_Resp = 0;
uint8_t buffer[30] = {0};

uint8_t databuffer [255];
float Value_Buf [2];


#define MPU6050_ADDR 0xD0


#define SMPLRT_DIV_REGISTER 0x19
#define GYRO_CONFIG_REGISTER 0x1B
#define ACCEL_CONFIG_REGISTER 0x1C
#define ACCEL_XOUT_H_REGISTER 0x3B //contains upper 8 bits of accelerometer measurement
#define TEMP_OUT_H_REGISTER 0x41
#define GYRO_XOUT_H_REGISTER 0x43
#define PWR_MGMT_1_REGISTER 0x6B
#define WHO_AM_I_REGISTER 0x75


int16_t Ac_x = 0; //raw accelerator value in x-axis
int16_t Ac_y = 0;
int16_t Ac_z = 0;

int16_t Gy_x = 0;
int16_t Gy_y = 0;
int16_t Gy_z = 0;

float A_x;
float A_y;
float A_z;
float Gy_x_deg_sec;
float Gy_y_deg_sec;
float Gy_z_deg_sec;

void MPU6050_Init (void) //verifying the ID of the device
{
	uint8_t check;
	uint8_t Data;


	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR,WHO_AM_I_REGISTER,1, &check, 1, 1000);

	if (check == 104)  // default value of register is 0x68, which the device is expected to return
	{

		Data = 0;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REGISTER, 1,&Data, 1, 1000); //writing all 0's to the register to wake the sensor up

		Data = 0x07;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REGISTER, 1, &Data, 1, 1000); //setting sample rate for acceleromeer

		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REGISTER, 1, &Data, 1, 1000); //sets accelerometer range to +/-2g

		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REGISTER, 1, &Data, 1, 1000); //sets gyroscope range to +/-250 degrees per second
	}

}

void Gyro_Values (void)
{
	uint8_t Rec_Data[6];


	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REGISTER, 1, Rec_Data, 6, 1000);

	Gy_x = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Gy_y = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Gy_z = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);


	// converting the raw gyroscope values into degrees per second
	//131 is the chosen full scale range of the gyroscope output
	Gy_x_deg_sec = Gy_x/131.0; //gyroscope value in x-axis
	Gy_y_deg_sec = Gy_y/131.0; //gyroscope value in y-axis
	Gy_z_deg_sec = Gy_z/131.0; //gyroscope value in z-axis
}


void Accelerometer_Values (void)
{
	uint8_t Rec_Data[6];


	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REGISTER, 1, Rec_Data, 6, 1000);


	Ac_x = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Ac_y = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Ac_z = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);


	// converting the raw accelerometer values into degrees per second
	//+/-2g is the chosen full scale range of the accelerator output, the sensitivity LSB value is 16384
	A_x = Ac_x/16384.0;
	A_y = Ac_y/16384.0;
	A_z = Ac_z/16384.0;
}



int main(void){
	initialise_monitor_handles();

	HAL_Init();

	SystemClock_Config();


	MX_GPIO_Init();
	MX_DMA_Init();
	MX_UART4_Init();
	MX_I2C1_Init();
	MX_UART5_Init();

	HAL_UART_Receive_DMA(&huart4, buff, 255);

	while (1){
		GPGLL_Sequence();
		Accelerometer_Values();
		Gyro_Values();
		Value_Buf[0] = resultant_lat;
		Value_Buf[1] = resultant_lon;
		GSM_init();
		HAL_Delay(10000);
		GSM_SMS();




  }

}

//initializing the GSM module
void GSM_init(){
	HAL_UART_Transmit(&huart5,(uint8_t*)"AT\r\n", sizeof("AT\r\n"),4000);
	HAL_Delay(10000);
	HAL_UART_Transmit(&huart5,(uint8_t*)"AT+CPIN?\r\n",sizeof("AT+CPIN?\r\n"),4000);
	HAL_Delay(3000);
	HAL_UART_Transmit(&huart5,(uint8_t*)"AT+CREG?\r\n",sizeof("AT+CREG?\r\n"),3000);
	HAL_Delay(1000);
	HAL_UART_Transmit(&huart5,(uint8_t*)"AT+CGATT=1\r\n",sizeof("AT+CGATT=1\r\n"),10000);
	HAL_Delay(1000);
	HAL_UART_Transmit(&huart5,(uint8_t*)"AT+CIPSHUT\r\n",sizeof("AT+CIPSHUT\r\n"),3000);
	HAL_Delay(1000);
	HAL_UART_Transmit(&huart5,(uint8_t*)"AT+CIPSTATUS\r\n",sizeof("AT+CIPSTATUS\r\n"),4000);
	HAL_Delay(2000);
	HAL_UART_Transmit(&huart5,(uint8_t*)"AT+CIPMUX=0\r\n",sizeof("AT+CIPMUX=0\r\n"),4000);
	HAL_Delay(2000);
	HAL_UART_Transmit(&huart5,(uint8_t*)"AT+CSTT=\"web.tigo.com.gh\"\r\n",sizeof("AT+CSTT=\"web.tigo.com.gh\"\r\n"),5000);//start task and setting the APN,
	HAL_Delay(1000);
	HAL_UART_Transmit(&huart5,(uint8_t*)"AT+CIICR\r\n",sizeof("AT+CIICR\r\n"),4000); //bring up wireless connection
	HAL_Delay(3000);
	HAL_UART_Transmit(&huart5,(uint8_t*)"AT+CIFSR\r\n",sizeof("AT+CIFSR\r\n"),3000); //get local IP address
	HAL_Delay(2000);
	HAL_UART_Transmit(&huart5,(uint8_t*)"AT+CIPSPRT=0\r\n",sizeof("AT+CIPSPRT=0\r\n"),4000); //get local IP address
	HAL_Delay(3000);
	HAL_UART_Transmit(&huart5,(uint8_t*)"AT+CIPSTART=\"TCP\",\"184.106.153.149\",80\r\n",sizeof("AT+CIPSTART=\"TCP\",\"184.106.153.149\",80\r\n"),7000); //start up the connection
	HAL_Delay(15000);
	HAL_Delay(15000);
	GSM_senddata("LFZYSTPR855KKTS8",2,Value_Buf);
	HAL_Delay(30000);

}

//sending data to the web server
void GSM_senddata(char *API, int num_of_fields, float value[2]){
	char http_buff[500] = {0};
	char data_buff[30] = {0};
	char field_buff[200] = {0};

	sprintf (http_buff, "GET https://api.thingspeak.com/update?api_key=%s", API);
	for (int i=0; i<num_of_fields; i++){
		sprintf(field_buff, "&field%d=%f",i+1, value[i]);
		strcat (http_buff, field_buff);
	}

	strcat(http_buff, "\r\n");
	int len = strlen (http_buff);
	sprintf (data_buff, "AT+CIPSEND=%d\r\n", len);
	HAL_UART_Transmit(&huart5,(uint8_t*)data_buff,sizeof(data_buff),15000);
	HAL_Delay(15000);
	HAL_UART_Transmit(&huart5,(uint8_t*)http_buff,sizeof(http_buff),15000);
	HAL_Delay(10000);

}

//sending latitude, longitude and time of fix to a trusted number.
void GSM_SMS(){

while(!AT_Resp){

		HAL_UART_Transmit(&huart5,(uint8_t *)"AT\r\n",strlen("AT\r\n"),1000);
		HAL_UART_Receive (&huart5, buffer, 30, 100);
		HAL_Delay(1000);
		if(strstr((char *)buffer,"OK")){
			AT_Resp = 1;
		}
		HAL_Delay(1000);
		memset(buffer,0,sizeof(buffer));
	}
	HAL_UART_Transmit(&huart5,(uint8_t *)"AT+CMGF=1\r\n",strlen("AT+CMGF=1\r\n"),1000);
	HAL_Delay(1000);
	sprintf(ATcommand,"AT+CMGS=\"%s\"\r\n",mobileNumber);
	HAL_UART_Transmit(&huart5,(uint8_t *)ATcommand,strlen(ATcommand),1000);
	HAL_Delay(100);
	sprintf(ATcommand,"Latitude = %f\n, Longitude = %f\n, Time = %s\n",resultant_lat,resultant_lon,timestamp);
	HAL_UART_Transmit(&huart5,(uint8_t *)ATcommand,strlen(ATcommand),1000);
	HAL_UART_Receive (&huart5, buffer, 30, 100);
	memset(buffer,0,sizeof(buffer));
	HAL_Delay(4000);

}

void GPGLL_Sequence(){

	if (flag == 1){

	  memset(buffStr, 0, 255);

	  		sprintf(buffStr, "%s", buff);
	  		char *string;

	  		string = strdup(buffStr);
	  		token = strsep(&string, "\n");

	  		while (token != NULL) {

	  			memset(nmeaseq, 0, 80);

	  			sprintf(nmeaseq, "%s", token);
	  			if ((strstr(nmeaseq, "$GPGLL") != 0) && strlen(nmeaseq) > 49 && strstr(nmeaseq, "*") != 0) {

	  				rawSum = strstr(nmeaseq, "*");

	  				memcpy(smNmbr, &rawSum[1], 2);

	  				smNmbr[2] = '\0';

	  				uint8_t intSum = nmea0183_checksum(nmeaseq);

	  				char hex[2];

	  				sprintf(hex, "%X", intSum);

	  				if (strstr(smNmbr, hex) != NULL) {


	  					counter = 0;

	  					for (char *tokens = strtok(nmeaseq,","); tokens != 0; tokens = strtok(NULL, ",")){

	  										  switch(counter){
	  										  case 1:lat = strdup(tokens); //obtain latitude coordinates
	  										  break;
	  										  case 2:NorthorSouth = strdup(tokens); //obtain direction of latitude
	  										  break;
	  										  case 3:longi = strdup(tokens); //obtain longitude coordinates
	  										  break;
	  										  case 4:EastorWest = strdup(tokens); // obtain direction of longitude
	  										  break;
	  										  case 5:timestamp = strdup(tokens); //obtain time of GPS fix
	  										  break;


	  										  }
	  										  counter++;


	  							            }



			  	memcpy(latDg, &lat[0], 2); //first two characters of latitude string represent the degrees

				latDg[2] = '\0';

				memcpy(lonDg, &longi[0], 3); //first three characters of longitude string represent the degrees
				lonDg[3] = '\0';

				//splitting the timestamp into hours, minutes and seconds
				memcpy(hH, &timestamp[0], 2);
				hH[2] = '\0';

				memcpy(mM, &timestamp[2], 2);
				mM[2] = '\0';

				memcpy(sS, &timestamp[4], 2);
				sS[2] = '\0';

				//concatenating timestamp into readable hh:mm:ss format
				strcpy(timestamp, hH);
				strcat(timestamp, ":");
				strcat(timestamp, mM);
				strcat(timestamp, ":");
				strcat(timestamp, sS);
				timestamp[8] = '\0';



			  printf("Time in hhmmss format is %s\n",timestamp);
			  printf("Latitude is %s\n",lat);
	          printf("Direction is %s\n",NorthorSouth);
	          printf("Longitude is %s\n",longi);
	          printf("Direction is %s\n",EastorWest);


	          //obtaining latitude value in decimal form
	          memcpy(lat_dec, &lat[2], 7);
	          float x;
	          printf("value is %s\n",lat_dec);
	          sscanf(lat_dec, "%f", &x);
	          float y = x/60;
	          int d;
	          sscanf(latDg, "%d", &d);
	          resultant_lat = d + y;
	          printf("Actual latitude in integer form is %f\n",resultant_lat);

	          HAL_Delay(2000);

	          //obtaining longitude value in decimal form
	          memcpy(lon_dec, &longi[3], 7);
	          float a;
	          printf("value is %s\n",lon_dec);
	          sscanf(lon_dec, "%f", &a);
	          float b = a/60;
	          int f;
	          sscanf(lonDg, "%d", &f);
	          resultant_lon = f + b;
	          printf("Actual longitude in integer form is %f\n",resultant_lon);

	          //condition for if latitude direction is north or south, south latitude values are negative in nature.
	          if( (strcmp(NorthorSouth,"S") == 0) ){
	        	  resultant_lat = -1 * resultant_lat;
	          }

	          //condition for if longitude direction is east or west, west longitude values are negative in nature.
	          if( (strcmp(EastorWest,"W") == 0) ){
	        	  resultant_lon = -1 * resultant_lon;


	          }

	    }
	  }


}
	  		flag = 0;


}
HAL_Delay(200);

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 9600;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

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
