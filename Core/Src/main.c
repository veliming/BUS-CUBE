/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


typedef enum {false = 0,true = 1} bool;

//GNSS NMEA-0183协议重要参数结构体定�????????????
//卫星信息
typedef struct
{
 	uint8_t num;		//卫星编号
	uint8_t eledeg;	//卫星仰角
	uint16_t azideg;	//卫星方位角
	uint8_t sn;		//信噪比
}nmea_slmsg;

//UTC时间信息
typedef struct
{
 	uint16_t year;	//年份
	uint8_t month;	//月份
	uint8_t date;	//日期
	uint8_t hour; 	//小时
	uint8_t min; 	//分钟
	uint8_t sec; 	//秒钟
}nmea_utc_time;

//NMEA 0183 协议解析后数据存放结构体
typedef struct
{
 	uint8_t svnum;					//可见卫星数
	nmea_slmsg slmsg[12];		//最多12颗卫星
	nmea_utc_time utc;			//UTC时间
	uint32_t latitude;				//纬度 分扩大100000倍,实际要除以100000
	uint8_t nshemi;					//北纬/南纬,N:北纬;S:南纬
	uint32_t longitude;			    //经度 分扩大100000倍,实际要除以100000
	uint8_t ewhemi;					//东经/西经,E:东经;W:西经
	uint8_t gnsssta;					//GPS状态:0,未定位;1,非差分定位;2,差分定位;6,正在估算.
 	uint8_t posslnum;				//用于定位的卫星数,0~12.
 	uint8_t possl[12];				//用于定位的卫星编号
	uint8_t fixmode;					//定位类型:1,没有定位;2,2D定位;3,3D定位
	uint16_t pdop;					//位置精度因子 0~500,对应实际值0~50.0
	uint16_t hdop;					//水平精度因子 0~500,对应实际值0~50.0
	uint16_t vdop;					//垂直精度因子 0~500,对应实际值0~50.0

	int altitude;			 	//海拔高度,放大了10倍,实际除以10.单位:0.1m
	uint16_t speed;					//地面速率,放大了1000倍,实际除以10.单位:0.001公里/小时
}nmea_msg;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USART_REC_LEN  			1200  	//定义�??????大接收字节数 1200
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
uint8_t USART_RX_BUF[USART_REC_LEN];     //接收缓冲,�??????大USART_REC_LEN个字�??????.

double pi = 3.14159265358979324;
double a = 6378245.0;
double ee = 0.00669342162296594323;
double x_pi = 3.14159265358979324 * 3000.0 / 180;
int iter = 0;

char n58_sdata[100] = {0};
char n58_sdataLAS[100] = {0};
double InLat=0.0;
double InLon=0.0;
double OutLat=0.0;
double OutLon=0.0;
float Speed;

unsigned short i =0;
unsigned char r =0;

nmea_msg gnssx; //GNSS�????????????
int firstgo=0;
int firstgnss=0;
uint8_t Time1;
uint8_t Time2;
int ret = -1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
uint8_t NMEA_Comma_Pos(uint8_t *buf,uint8_t cx);
uint32_t NMEA_Pow(uint8_t m,uint8_t n);
int NMEA_Str2num(uint8_t *buf,uint8_t*dx);
void NMEA_GNGSV_Analysis(nmea_msg *gnssx,uint8_t *buf);
void NMEA_GNGGA_Analysis(nmea_msg *gnssx,uint8_t *buf);
void NMEA_GNGSA_Analysis(nmea_msg *gnssx,uint8_t *buf);
void NMEA_GNRMC_Analysis(nmea_msg *gnssx,uint8_t *buf);
void NMEA_GNVTG_Analysis(nmea_msg *gnssx,uint8_t *buf);
void GNSS_Analysis(nmea_msg *gnssx,uint8_t *buf);
void Ublox_CheckSum(uint8_t *buf,uint16_t len,uint8_t* cka,uint8_t*ckb);
int outOfChina(double lat, double lon);
double transformLat(double x, double y);
double transformLon(double x, double y);
int wgs2gcj(double lat, double lon, double* pLat, double* pLon);
int n58_normal_check(char *in_cmd);
int n58_check_sim_card();
int n58_check_cgatt();
int n58_check_creg();
int n58_check_xiic();


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_DMA(&huart1,USART_RX_BUF, sizeof(USART_RX_BUF));
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);
  HAL_Delay(10000);
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET);

  first:
  	  firstgo=0;
  	if(firstgo!=0){
  			HAL_Delay(500);

  			n58_normal_check("AT+XIIC=0\r\n");

  			HAL_Delay(500);

  			n58_normal_check("AT+CGATT=0\r\n");
  	}
  	firstgo=1;
  	iter = 0;
      while(iter++ <10) {
          ret = n58_normal_check("AT\r\n");
          if(ret == 0) {
              break;
          }
          HAL_Delay(200);
      }
      if(ret == -1) {
          goto first ;
      }
      HAL_Delay(150);

      iter = 0;
      while(iter++ <10) {
          ret = n58_normal_check("ATE0\r\n");
          if(ret == 0) {
              break;
          }
          HAL_Delay(200);
      }
      if(ret == -1) {
          goto first ;
      }
      HAL_Delay(150);


      iter = 0;
      while(iter++ <10) {
          ret = n58_normal_check("AT+CGSN\r\n");
          if(ret == 0) {
              break;
          }
          HAL_Delay(10);
      }
      if(ret == -1) {
          goto first ;
      }
      HAL_Delay(150);


      iter = 0;
      while(iter++ <10) {
          ret = n58_check_sim_card();
          if(ret == 0) {
              break;
          }
          HAL_Delay(200);
      }
      if(ret == -1) {
          goto first;
      }
      HAL_Delay(150);


      iter = 0;
      while(iter++ <10) {
          ret = n58_normal_check("AT+CIMI\r\n");
          if(ret == 0) {
              break;
          }
          HAL_Delay(200);
      }
      if(ret == -1) {
          goto first ;
      }
      HAL_Delay(150);


      iter = 0;
      while(iter++ <10) {
          ret = n58_normal_check("AT+CREG=2\r\n");
          if(ret == 0) {
              break;
          }
          HAL_Delay(200);
      }
      if(ret == -1) {
          goto first ;
      }
      HAL_Delay(150);


      iter = 0;
      while(iter++ <10) {
          ret = n58_check_creg();
          if(ret == 0) {
              break;
          }
          HAL_Delay(200);
      }
      if(ret == -1) {
          goto first ;
      }



      HAL_Delay(150);
      iter = 0;
      while(iter++ <10) {
          ret = n58_normal_check("AT+CREG=0\r\n");
          if(ret == 0) {
              break;
          }
          HAL_Delay(200);
      }
      if(ret == -1) {
          goto first ;
      }
      HAL_Delay(150);


      iter = 0;
      while(iter++ <10) {
          ret = n58_normal_check("AT+CGATT=1\r\n");
          if(ret == 0) {
              break;
          }
          HAL_Delay(200);
      }
      if(ret == -1) {
          goto first ;
      }
      HAL_Delay(150);



      iter = 0;
      while(iter++ <10) {
          ret = n58_check_cgatt();
          if(ret == 0) {
              break;
          }
          HAL_Delay(200);
      }
      if(ret == -1) {
          goto first;
      }
      HAL_Delay(150);


      iter = 0;
      while(iter++ <10) {
          ret = n58_normal_check("AT+CGDCONT=1,\"IP\", \"CMIOT\"\r\n");
          if(ret == 0) {
              break;
          }
          HAL_Delay(200);
      }
      if(ret == -1) {
          goto first ;
      }
      HAL_Delay(150);



      iter = 0;
      while(iter++ <10) {
          ret = n58_normal_check("AT+XIIC=1\r\n");
          if(ret == 0) {
              break;
          }
          HAL_Delay(200);
      }
      if(ret == -1) {
          goto first ;
      }
      HAL_Delay(150);



      iter = 0;
      while(iter++ <100) {
          ret = n58_check_xiic();
          if(ret == 0) {
              break;
          }
          HAL_Delay(200);
      }

      if(ret == -1) {
          goto first ;
      }


      HAL_Delay(150);
      iter = 0;
      while(iter++ <10) {
          ret = n58_normal_check("AT+HTTPSCFG=\"sslversion\",3\r\n");
          if(ret == 0) {
              break;
          }
          HAL_Delay(200);
      }
      if(ret == -1) {
          goto first ;
      }


      HAL_Delay(150);
      iter = 0;
      while(iter++ <10) {
          ret = n58_normal_check("AT+HTTPSCFG =\"authmode\",0\r\n");
          if(ret == 0) {
              break;
          }
          HAL_Delay(200);
      }
      if(ret == -1) {
          goto first ;
      }


  		HAL_Delay(150);
      iter = 0;
      while(iter++ <10) {
          ret = n58_normal_check("AT$MYGPSPWR=1\r\n");
          if(ret == 0) {
              break;
          }
          HAL_Delay(1000);
      }
      if(ret == -1) {
          goto first ;
      }
  	HAL_Delay(150);


    iter = 0;
    while(iter++ <10) {
        ret = n58_normal_check("AT$MYGNSSSEL=0\r\n");
        if(ret == 0) {
            break;
        }
        HAL_Delay(200);
    }
    if(ret == -1) {
        goto first;
    }
  	HAL_Delay(150);


      iter = 0;
      while(iter++ <10) {
          ret = n58_normal_check("AT+HTTPSPARA=url,\"bus.bitworkshop.cn/bus/position\"\r\n");
          if(ret == 0) {
              break;
          }
          HAL_Delay(200);
      }
      if(ret == -1) {
          goto first ;
      }
      HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_RESET);
	  			GNSSopen:
				HAL_Delay(1000);
	  			memset(USART_RX_BUF, 0, sizeof(USART_RX_BUF));
	  		    HAL_UART_DMAStop(&huart1);
	  		    HAL_UART_Receive_DMA(&huart1,USART_RX_BUF, sizeof(USART_RX_BUF));
	  			strcpy((char *)n58_sdata,"AT$MYGNSSMSG\r\n");
	  			HAL_UART_Transmit_DMA(&huart1, (uint8_t*)n58_sdata,strlen(n58_sdata));//发�??
	  			HAL_Delay(1000);
	  			GNSS_Analysis(&gnssx,USART_RX_BUF);//GNSS分析
	  			Speed=gnssx.speed;								//速度
	  			InLon=gnssx.longitude;
	  			InLat=gnssx.latitude;			//经纬
	  			wgs2gcj(InLat/100000,InLon/100000, &OutLat,&OutLon);  //坐标系转�??????
	  			if(OutLat==0.0||OutLon==0.0)
	  					{ firstgnss=1;goto GNSSopen;}

	  			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_SET);
	  			HAL_Delay (100);
	  			i=0;
	  			memset(n58_sdata, 0, sizeof(n58_sdata));
	  			strcpy((char *)n58_sdata,"AT+HTTPSCLOSE\r\n");
	  			HAL_UART_Transmit_DMA(&huart1, (uint8_t*)n58_sdata,strlen(n58_sdata));//�??????

	  			memset(USART_RX_BUF, 0, sizeof(USART_RX_BUF));
	  		    HAL_UART_DMAStop(&huart1);
	  		    HAL_UART_Receive_DMA(&huart1,USART_RX_BUF, sizeof(USART_RX_BUF));

	  		    memset(n58_sdata, 0, sizeof(n58_sdata));
	  			strcpy((char *)n58_sdata,"AT+HTTPSSETUP\r\n");
	  			HAL_UART_Transmit_DMA(&huart1, (uint8_t*)n58_sdata,strlen(n58_sdata));//�??????

	  			SETUP:

	  					if(i<500)
	  						{
	  							HAL_Delay (10);
	  							if(!strstr((char*)USART_RX_BUF, "OK"))
	  							{
	  								i++;
	  								goto SETUP;
	  							}
	  							else{i=0;goto next;	}
	  						}

	  				else
	  				{
	  					i=0;
	  					memset(USART_RX_BUF, 0, sizeof(USART_RX_BUF));
	  				    HAL_UART_DMAStop(&huart1);
	  				    HAL_UART_Receive_DMA(&huart1,USART_RX_BUF, sizeof(USART_RX_BUF));
	  		  			strcpy((char *)n58_sdata,"AT+HTTPSCLOSE\r\n");
	  		  			HAL_UART_Transmit_DMA(&huart1, (uint8_t*)n58_sdata,strlen(n58_sdata));//�??????
	  					goto first;
	  				}




	  				next:

	  					HAL_Delay (300);
	  				memset(n58_sdataLAS, 0, sizeof(n58_sdataLAS));
	  				sprintf((char *)n58_sdataLAS,"{\"i\":\"606c7c1a25985565e23f18a7\",\"p\":{\"la\":%.6f,\"lo\":%.6f},\"s\":%.2f}\r\n",OutLat,OutLon,Speed/=1000);//OutLat,OutLon  InLatb/100000,InLonb/100000

	  				memset(n58_sdata, 0, sizeof(n58_sdata));
	  				sprintf((char *)n58_sdata,"AT+HTTPSACTION=2,%d,2\r\n",strlen(n58_sdataLAS)-2);//OutLat,OutLon  InLatb/100000,InLonb/100000

	  				memset(USART_RX_BUF, 0, sizeof(USART_RX_BUF));
	  			    HAL_UART_DMAStop(&huart1);
	  			    HAL_UART_Receive_DMA(&huart1,USART_RX_BUF, sizeof(USART_RX_BUF));
	  				HAL_UART_Transmit_DMA(&huart1, (uint8_t*)n58_sdata,strlen(n58_sdata));//发�??
	  				//user_send_data(n58_sdata);//发�?�请�????????????2
	  				HAL_Delay (100);
	  				SETUP2:


  					if(i<100)
  						{
  							HAL_Delay (10);

  							if(!strstr((char*)USART_RX_BUF, ">"))
  							{
  								i++;

  								goto SETUP2;
  							}
  							else{i=0;goto next2;}
  						}

  					else
  					{
  						i=0;
  						memset(USART_RX_BUF, 0, sizeof(USART_RX_BUF));
  					    HAL_UART_DMAStop(&huart1);
  					    HAL_UART_Receive_DMA(&huart1,USART_RX_BUF, sizeof(USART_RX_BUF));
  			  			strcpy((char *)n58_sdata,"AT+HTTPSCLOSE\r\n");
  			  			HAL_UART_Transmit_DMA(&huart1, (uint8_t*)n58_sdata,strlen(n58_sdata));//�??????
  						goto first;
  					}
  					next2:
	  				memset(USART_RX_BUF, 0, sizeof(USART_RX_BUF));
	  			    HAL_UART_DMAStop(&huart1);
	  			    HAL_UART_Receive_DMA(&huart1,USART_RX_BUF, sizeof(USART_RX_BUF));
		  			HAL_UART_Transmit_DMA(&huart1, (uint8_t*)n58_sdataLAS,strlen(n58_sdataLAS));//�??????

	  				HAL_Delay (100);
	  				memset(USART_RX_BUF, 0, sizeof(USART_RX_BUF));
	  			    HAL_UART_DMAStop(&huart1);
	  			    HAL_UART_Receive_DMA(&huart1,USART_RX_BUF, sizeof(USART_RX_BUF));

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED0_Pin */
  GPIO_InitStruct.Pin = LED0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED0_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */



uint8_t NMEA_Comma_Pos(uint8_t *buf,uint8_t cx)
{
	uint8_t *p=buf;
	while(cx)
	{
		if(*buf=='*'||*buf<' '||*buf>'z')return 0XFF;//遇到'*'或者非法字符,则不存在第cx个逗号
		if(*buf==',')cx--;
		buf++;
	}
	return buf-p;
}

//m^n函数
//返回�????????????:m^n次方.
uint32_t NMEA_Pow(uint8_t m,uint8_t n)
{
	uint32_t result=1;
	while(n--)result*=m;
	return result;
}

//str转换为数�????????????,�????????????','或�??'*'结束
//buf:数字存储�????????????
//dx:小数点位�????????????,返回给调用函�????????????
//返回�????????????:转换后的数�??
int NMEA_Str2num(uint8_t *buf,uint8_t*dx)
{
	uint8_t *p=buf;
	uint32_t ires=0,fres=0;
	uint8_t ilen=0,flen=0,i;
	uint8_t mask=0;
	int res;
	while(1) //得到整数和小数的长度
	{
		if(*p=='-'){mask|=0X02;p++;}//是负�????????????
		if(*p==','||(*p=='*'))break;//遇到结束�????????????
		if(*p=='.'){mask|=0X01;p++;}//遇到小数点了
		else if(*p>'9'||(*p<'0'))	//有非法字�????????????
		{
			ilen=0;
			flen=0;
			break;
		}
		if(mask&0X01)flen++;
		else ilen++;
		p++;
	}
	if(mask&0X02)buf++;	//去掉负号
	for(i=0;i<ilen;i++)	//得到整数部分数据
	{
		ires+=NMEA_Pow(10,ilen-1-i)*(buf[i]-'0');
	}
	if(flen>8)flen=8;	//�????????????多取8位小�????????????
	*dx=flen;	 		//小数点位�????????????
	for(i=0;i<flen;i++)	//得到小数部分数据
	{
		fres+=NMEA_Pow(10,flen-1-i)*(buf[ilen+1+i]-'0');
	}
	res=ires*NMEA_Pow(10,flen)+fres;
	if(mask&0X02)res=-res;
	return res;
}

//分析GNGSV信息
//gnssx:nmea信息结构�????????????
//buf:接收到的GNSS数据缓冲区首地址
void NMEA_GNGSV_Analysis(nmea_msg *gnssx,uint8_t *buf)
{
	uint8_t *p,*p1,dx;
	uint8_t len,i,j,slx=0;
	uint8_t posx;
	p=buf;
	p1=(uint8_t*)strstr((const char *)p,"&GPGSV");
	len=p1[7]-'0';								//得到GPGSV的条数
	posx=NMEA_Comma_Pos(p1,3); 					//得到可见卫星总数
	if(posx!=0XFF)gnssx->svnum=NMEA_Str2num(p1+posx,&dx);
	for(i=0;i<len;i++)
	{
		p1=(uint8_t*)strstr((const char *)p,"&GPGSV");
		for(j=0;j<4;j++)
		{
			posx=NMEA_Comma_Pos(p1,4+j*4);
			if(posx!=0XFF)gnssx->slmsg[slx].num=NMEA_Str2num(p1+posx,&dx);	//得到卫星编号
			else break;
			posx=NMEA_Comma_Pos(p1,5+j*4);
			if(posx!=0XFF)gnssx->slmsg[slx].eledeg=NMEA_Str2num(p1+posx,&dx);//得到到卫星仰角
			else break;
			posx=NMEA_Comma_Pos(p1,6+j*4);
			if(posx!=0XFF)gnssx->slmsg[slx].azideg=NMEA_Str2num(p1+posx,&dx);//得到卫星方位�????????????
			else break;
			posx=NMEA_Comma_Pos(p1,7+j*4);
			if(posx!=0XFF)gnssx->slmsg[slx].sn=NMEA_Str2num(p1+posx,&dx);	//得到卫星信噪�????????????
			else break;
			slx++;
		}
 		p=p1+1;//切换到下�????????????个GNGSV信息
	}
}

//分析GNGGA信息
//gnssx:nmea信息结构�????????????
//buf:接收到的GNSS数据缓冲区首地址
void NMEA_GNGGA_Analysis(nmea_msg *gnssx,uint8_t *buf)
{
	uint8_t *p1,dx;
	uint8_t posx;
	p1=(uint8_t*)strstr((const char *)buf,"$GNGGA");
	posx=NMEA_Comma_Pos(p1,6);								//得到GNSS状�??
	if(posx!=0XFF)gnssx->gnsssta=NMEA_Str2num(p1+posx,&dx);
	posx=NMEA_Comma_Pos(p1,7);								//得到用于定位的卫星数
	if(posx!=0XFF)gnssx->posslnum=NMEA_Str2num(p1+posx,&dx);
	posx=NMEA_Comma_Pos(p1,9);								//得到海拔高度
	if(posx!=0XFF)gnssx->altitude=NMEA_Str2num(p1+posx,&dx);
}

//分析GNGSA信息
//gnssx:nmea信息结构�????????????
//buf:接收到的GNSS数据缓冲区首地址
void NMEA_GNGSA_Analysis(nmea_msg *gnssx,uint8_t *buf)
{
	uint8_t *p1,dx;
	uint8_t posx;
	uint8_t i;
	p1=(uint8_t*)strstr((const char *)buf,"$GNGSA");
	posx=NMEA_Comma_Pos(p1,2);								//得到定位类型
	if(posx!=0XFF)gnssx->fixmode=NMEA_Str2num(p1+posx,&dx);
	for(i=0;i<12;i++)										//得到定位卫星编号
	{
		posx=NMEA_Comma_Pos(p1,3+i);
		if(posx!=0XFF)gnssx->possl[i]=NMEA_Str2num(p1+posx,&dx);
		else break;
	}
	posx=NMEA_Comma_Pos(p1,15);								//得到PDOP位置精度因子
	if(posx!=0XFF)gnssx->pdop=NMEA_Str2num(p1+posx,&dx);
	posx=NMEA_Comma_Pos(p1,16);								//得到HDOP位置精度因子
	if(posx!=0XFF)gnssx->hdop=NMEA_Str2num(p1+posx,&dx);
	posx=NMEA_Comma_Pos(p1,17);								//得到VDOP位置精度因子
	if(posx!=0XFF)gnssx->vdop=NMEA_Str2num(p1+posx,&dx);
}

//分析GNRMC信息
//gnssx:nmea信息结构�????????????
//buf:接收到的GNSS数据缓冲区首地址
void NMEA_GNRMC_Analysis(nmea_msg *gnssx,uint8_t *buf)
{
	uint8_t *p1,dx;
	uint8_t posx;
	uint32_t temp;
	float rs;
	p1=(uint8_t*)strstr((const char *)buf,"GNRMC");//"$GNRMC",经常�????????????&和GNRMC分开的情�????????????,故只判断GNRMC.
	posx=NMEA_Comma_Pos(p1,1);								//得到UTC时间
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num(p1+posx,&dx)/NMEA_Pow(10,dx);	 	//得到UTC时间,去掉ms
		gnssx->utc.hour=temp/10000;
		gnssx->utc.min=(temp/100)%100;
		gnssx->utc.sec=temp%100;
	}
	posx=NMEA_Comma_Pos(p1,3);								//得到纬度
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num(p1+posx,&dx);
		gnssx->latitude=temp/NMEA_Pow(10,dx+2);	//得到°
		rs=temp%NMEA_Pow(10,dx+2);				//得到'
		gnssx->latitude=gnssx->latitude*NMEA_Pow(10,5)+(rs*NMEA_Pow(10,5-dx))/60;//转换为�?
	}
	posx=NMEA_Comma_Pos(p1,4);								//南纬还是北纬
	if(posx!=0XFF)gnssx->nshemi=*(p1+posx);
 	posx=NMEA_Comma_Pos(p1,5);								//得到经度
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num(p1+posx,&dx);
		gnssx->longitude=temp/NMEA_Pow(10,dx+2);	//得到°
		rs=temp%NMEA_Pow(10,dx+2);				//得到'
		gnssx->longitude=gnssx->longitude*NMEA_Pow(10,5)+(rs*NMEA_Pow(10,5-dx))/60;//转换为�?
	}
	posx=NMEA_Comma_Pos(p1,6);								//东经还是西经
	if(posx!=0XFF)gnssx->ewhemi=*(p1+posx);
	posx=NMEA_Comma_Pos(p1,9);								//得到UTC日期
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num(p1+posx,&dx);		 				//得到UTC日期
		gnssx->utc.date=temp/10000;
		gnssx->utc.month=(temp/100)%100;
		gnssx->utc.year=2000+temp%100;
	}
}

//分析GNVTG信息
//gnssx:nmea信息结构�????????????
//buf:接收到的GNSS数据缓冲区首地址
void NMEA_GNVTG_Analysis(nmea_msg *gnssx,uint8_t *buf)
{
	uint8_t *p1,dx;
	uint8_t posx;
	p1=(uint8_t*)strstr((const char *)buf,"$GNVTG");
	posx=NMEA_Comma_Pos(p1,7);								//得到地面速率
	if(posx!=0XFF)
	{
		gnssx->speed=NMEA_Str2num(p1+posx,&dx);
		if(dx<3)gnssx->speed*=NMEA_Pow(10,3-dx);	 	 		//确保扩大1000
	}
}

//提取NMEA-0183信息
//gnssx:nmea信息结构�????????????
//buf:接收到的GNSS数据缓冲区首地址
void GNSS_Analysis(nmea_msg *gnssx,uint8_t *buf)
{
//	NMEA_GNGSV_Analysis(gnssx,buf);	//GNGSV解析
	NMEA_GNGGA_Analysis(gnssx,buf);	//GNGGA解析
	NMEA_GNGSA_Analysis(gnssx,buf);	//GNGSA解析
	NMEA_GNRMC_Analysis(gnssx,buf);	//GNRMC解析
	NMEA_GNVTG_Analysis(gnssx,buf);	//GNVTG解析
}

//GNSS校验和计�????????????
//buf:数据缓存区首地址
//len:数据长度
//cka,ckb:两个校验结果.
void Ublox_CheckSum(uint8_t *buf,uint16_t len,uint8_t* cka,uint8_t*ckb)
{
	uint16_t i;
	*cka=0;*ckb=0;
	for(i=0;i<len;i++)
	{
		*cka=*cka+buf[i];
		*ckb=*ckb+*cka;
	}
}

int outOfChina(double lat, double lon) {
    if (lon < 72.004 || lon > 137.8347)
        return true;
    if (lat < 0.8293 || lat > 55.8271)
        return true;
    return false;
}

// 纬度偏移�????????????
double transformLat(double x, double y) {
       double ret = 0.0;
       ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * sqrt(fabs(x));
       ret += (20.0 * sin(6.0 * x * pi) + 20.0 * sin(2.0 * x * pi)) * 2.0 / 3.0;
       ret += (20.0 * sin(y * pi) + 40.0 * sin(y / 3.0 * pi)) * 2.0 / 3.0;
       ret += (160.0 * sin(y / 12.0 * pi) + 320 * sin(y * pi  / 30.0)) * 2.0 / 3.0;
       return ret;
}

// 经度偏移�????????????
double transformLon(double x, double y) {
       double ret = 0.0;
       ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * sqrt(fabs(x));
       ret += (20.0 * sin(6.0 * x * pi) + 20.0 * sin(2.0 * x * pi)) * 2.0 / 3.0;
       ret += (20.0 * sin(x * pi) + 40.0 * sin(x / 3.0 * pi)) * 2.0 / 3.0;
       ret += (150.0 * sin(x / 12.0 * pi) + 300.0 * sin(x / 30.0 * pi)) * 2.0 / 3.0;
       return ret;
}

int wgs2gcj(double lat, double lon, double* pLat, double* pLon) {
    if (outOfChina(lat,lon))
    {
        *pLat = lat;
        *pLon = lon;
        return 0;
    }
   double dLat = transformLat(lon - 105.0, lat - 35.0);
   double dLon = transformLon(lon - 105.0, lat - 35.0);
   double radLat = lat / 180.0 * pi;
   double magic = sin(radLat);
   magic = 1 - ee * magic * magic;
   double sqrtMagic = sqrt(magic);
   dLat = (dLat * 180.0) / ((a * (1 - ee)) / (magic * sqrtMagic) * pi);
   dLon = (dLon * 180.0) / (a / sqrtMagic * cos(radLat) * pi);
   *pLat = lat + dLat;
   *pLon = lon + dLon;
   return 0;
}



int n58_normal_check(char *in_cmd) {

	memset(USART_RX_BUF, 0, sizeof(USART_RX_BUF));
    HAL_UART_DMAStop(&huart1);
    HAL_UART_Receive_DMA(&huart1,USART_RX_BUF, sizeof(USART_RX_BUF));
	HAL_UART_Transmit_DMA(&huart1, (uint8_t*)in_cmd,strlen(in_cmd));//�??????
	HAL_Delay(500);
    if(strstr((char *)USART_RX_BUF, "OK") != NULL) {
        return 0;
    }
    return -1;
}

int n58_check_sim_card() {
	memset(USART_RX_BUF, 0, sizeof(USART_RX_BUF));
    HAL_UART_DMAStop(&huart1);
    HAL_UART_Receive_DMA(&huart1,USART_RX_BUF, sizeof(USART_RX_BUF));
	strcpy((char *)n58_sdata,"AT+CPIN?\r\n");
	HAL_UART_Transmit_DMA(&huart1, (uint8_t*)n58_sdata,strlen(n58_sdata));//�??????
	HAL_Delay(500);
    if(strstr((char *)USART_RX_BUF, "READY") != NULL) {
        return 0;
    }
    return -1;
}

int n58_check_cgatt() {
	memset(USART_RX_BUF, 0, sizeof(USART_RX_BUF));
    HAL_UART_DMAStop(&huart1);
    HAL_UART_Receive_DMA(&huart1,USART_RX_BUF, sizeof(USART_RX_BUF));
	strcpy((char *)n58_sdata,"AT+CGATT?\r\n");
	HAL_UART_Transmit_DMA(&huart1, (uint8_t*)n58_sdata,strlen(n58_sdata));
	HAL_Delay(500);
    if(strstr((char *)USART_RX_BUF, "CGATT: 1") != NULL) {
        return 0;
    }
    return -1;
}

int n58_check_creg() {
	memset(USART_RX_BUF, 0, sizeof(USART_RX_BUF));
    HAL_UART_DMAStop(&huart1);
    HAL_UART_Receive_DMA(&huart1,USART_RX_BUF, sizeof(USART_RX_BUF));
	strcpy((char *)n58_sdata,"AT+CREG?\r\n");
	HAL_UART_Transmit_DMA(&huart1, (uint8_t*)n58_sdata,strlen(n58_sdata));
	HAL_Delay(500);
    if(strstr((char *)USART_RX_BUF, "CREG: 2") != NULL) {
        return 0;
    }
    return -1;
}


int n58_check_xiic() {
	memset(USART_RX_BUF, 0, sizeof(USART_RX_BUF));
    HAL_UART_DMAStop(&huart1);
    HAL_UART_Receive_DMA(&huart1,USART_RX_BUF, sizeof(USART_RX_BUF));
	strcpy((char *)n58_sdata,"AT+XIIC?\r\n");
	HAL_UART_Transmit_DMA(&huart1, (uint8_t*)n58_sdata,strlen(n58_sdata));
	HAL_Delay(500);
    if(strstr((char *)USART_RX_BUF, "+XIIC:    1") != NULL) {
        return 0;
    }
    return -1;
}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//
//{
//    if(huart->Instance == USART1)
//
//    {
//    //	HAL_UART_DMAResume(&huart1);
//    	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
//    }
//}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
