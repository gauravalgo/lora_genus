/******************************************************************************
  * @file    main.c
  * @author  MCD Application Team
  * @version V1.1.3
  * @date    20-December-2017
  * @brief   this is the main!
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "hw.h"
#include "low_power_manager.h"
#include "lora.h"
#include "bsp.h"
#include "timeServer.h"
#include "vcom.h"
#include "version.h"
typedef enum
{
  ATCTL_RET_TIMEOUT = -3,         /* RX data timeout */
  ATCTL_RET_ERR = -2,             /* Unknown command */
  ATCTL_RET_CMD_ERR = -1,         /* Get command +CMD: ERROR(x) */
  ATCTL_RET_IDLE = 0,
  ATCTL_RET_CMD_OK,               /* Command is OK, but can't parse */
  ATCTL_RET_CMD_AT,
  ATCTL_RET_CMD_ID,
  ATCTL_RET_CMD_VER,
  ATCTL_RET_CMD_RTC,
  ATCTL_RET_CMD_VDD,
  ATCTL_RET_CMD_MSG,
  ATCTL_RET_CMD_JOIN,
  ATCTL_RET_CMD_EEPROM,
  ATCTL_RET_CMD_DR,
  ATCTL_RET_CMD_MODE,
  ATCTL_RET_CMD_LW,
  ATCTL_RET_CMD_DELAY,
}atctl_ret_t,ATEerror_t;
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define DATA_RX_MAX_BUFF_SIZE    300 
/*!
 * CAYENNE_LPP is myDevices Application server.
 */

#define LPP_DATATYPE_DIGITAL_INPUT  0x0
#define LPP_DATATYPE_DIGITAL_OUTPUT 0x1
#define LPP_DATATYPE_HUMIDITY       0x68
#define LPP_DATATYPE_TEMPERATURE    0x67
#define LPP_DATATYPE_BAROMETER      0x73
#define LPP_APP_PORT 97
/*!
 * Defines the application data transmission duty cycle. 5s, value in [ms].
 */
#define APP_TX_DUTYCYCLE                            10000
/*!
 * LoRaWAN Adaptive Data Rate
 * @note Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_STATE LORAWAN_ADR_ON
/*!
 * LoRaWAN Default data Rate Data Rate
 * @note Please note that LORAWAN_DEFAULT_DATA_RATE is used only when ADR is disabled 
 */
#define LORAWAN_DEFAULT_DATA_RATE DR_0
/*!
 * LoRaWAN application port
 * @note do not use 224. It is reserved for certification
 */
#define LORAWAN_APP_PORT                            50
/*!
 * Number of trials for the join request.
 */
#define JOINREQ_NBTRIALS                            1
/*!
 * LoRaWAN default endNode class port
 */
#define LORAWAN_DEFAULT_CLASS                       CLASS_C
/*!
 * LoRaWAN default confirm state
 *///LORAWAN_CONFIRMED_MSG
#define LORAWAN_DEFAULT_CONFIRM_MSG_STATE           LORAWAN_CONFIRMED_MSG //LORAWAN_UNCONFIRMED_MSG 
/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_BUFF_SIZE                           64
/*!
 * User application data
 */
static uint8_t AppDataBuff[LORAWAN_APP_DATA_BUFF_SIZE];
static int8_t transmit_buff_size =0;
/*!
 * User application data structure
 */
 static uint8_t response[DATA_RX_MAX_BUFF_SIZE];  
 static uint8_t aRxBuffer[5];  /* Buffer used for Rx input character */   
static lora_AppData_t AppData={ response,  255 ,0 };
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* call back when LoRa endNode has received a frame*/
static void LORA_RxData( lora_AppData_t *AppData);

/* call back when LoRa endNode has just joined*/
static void LORA_HasJoined( void );

/* call back when LoRa endNode has just switch the class*/
static void LORA_ConfirmClass ( DeviceClass_t Class );

/* LoRa endNode send request*/
static void Send( void );

/* start the tx process*/
static void LoraStartTx(TxEventType_t EventType);
extern UART_HandleTypeDef UartHandle;
/* tx timer callback function*/
static void OnTxTimerEvent( void );

/* Private variables ---------------------------------------------------------*/
/* load Main call backs structure*/
static LoRaMainCallback_t LoRaMainCallbacks ={ HW_GetBatteryLevel,
                                               HW_GetTemperatureLevel,
                                               HW_GetUniqueId,
                                               HW_GetRandomSeed,
                                               LORA_RxData,
                                               LORA_HasJoined,
                                               LORA_ConfirmClass};

/*!
 * Specifies the state of the application LED
 */
static uint8_t AppLedStateOn = RESET;
                                             
static TimerEvent_t TxTimer;

#ifdef USE_B_L072Z_LRWAN1
/*!
 * Timer to handle the application Tx Led to toggle
 */
static TimerEvent_t TxLedTimer;
																							 static uint8_t aRxBuffer[5];  /* Buffer used for Rx input character */   
static void OnTimerLedEvent( void );
																							 
#endif
																							 static ATEerror_t at_cmd_receive_async_event(void);
/* !
 *Initialises the Lora Parameters
 */
static  LoRaParam_t LoRaParamInit= {LORAWAN_ADR_STATE,
                                    LORAWAN_DEFAULT_DATA_RATE,  
                                    LORAWAN_PUBLIC_NETWORK,
                                    JOINREQ_NBTRIALS};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main( void )
{
  /* STM32 HAL library initialization*/
  HAL_Init( );
  
  /* Configure the system clock*/
  SystemClock_Config( );
  
  /* Configure the debug mode*/
 // DBG_Init( );
  
  /* Configure the hardware*/
  HW_Init( );
  
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */
  
  /*Disbale Stand-by mode*/
  LPM_SetOffMode(LPM_APPLI_Id , LPM_Disable );
  
  /* Configure the Lora Stack*/
  LORA_Init( &LoRaMainCallbacks, &LoRaParamInit);
  
  PRINTF("VERSION: %X\n\r", VERSION);
  
  LORA_Join( );
  
  LoraStartTx(TX_ON_TIMER) ;
	uint8_t aTxStartMessage[] = "\r\nmain\r\n";
		HAL_UART_Transmit_IT(&UartHandle, (uint8_t *)aTxStartMessage, sizeof(aTxStartMessage));
  at_cmd_receive_async_event();
				
  while( 1 )
  {
    
		DISABLE_IRQ( );
    /* if an interrupt has occurred after DISABLE_IRQ, it is kept pending 
     * and cortex will not enter low power anyway  */

#ifndef LOW_POWER_DISABLE
    LPM_EnterLowPower( );
#endif

    ENABLE_IRQ();
    
    /* USER CODE BEGIN 2 */
    /* USER CODE END 2 */
  }
}
static void LORA_HasJoined( void )
{
#if( OVER_THE_AIR_ACTIVATION != 0 )
  PRINTF("JOINED\n\r");
#endif
  LORA_RequestClass( LORAWAN_DEFAULT_CLASS );
}
static ATEerror_t at_cmd_receive_async_event(void)
{
uint8_t  ResponseComplete = 0;
int8_t i = 0;
int8_t charnumber = 0;
char *ptrChr;
ATEerror_t RetCode;
uint8_t NoReturnCode =1;   /*too discriminate the Get reurn code from return value*/
 char buf [10]; 
  /*cleanup the response buffer*/
  memset(response, 0x00, DATA_RX_MAX_BUFF_SIZE); 
	int j=0;
  /*UART peripheral in reception process for response returned by slave*/  
  if(HAL_UART_Receive_IT(&UartHandle, (uint8_t *)aRxBuffer,1) != HAL_OK)
  {
   while (1);
  } 
  
  while (!ResponseComplete)
  {  
   
	
		while (HW_UART_Modem_IsNewCharReceived() == RESET);   
    
    /*process the response*/    
    response[i] = HW_UART_Modem_GetNewChar();
     sprintf (buf, "%d\n", i);
		HAL_UART_Transmit_IT(&UartHandle, (uint8_t *)buf, 3);   
		HAL_UART_Transmit_IT(&UartHandle, (uint8_t *)&response[i], 1);     
    transmit_buff_size=i;
		/*wait up to carriage return OR the line feed marker*/
    if (/*(response[i] =='\r') || */response[i] == '\n')       
    {
      if (i!= 0)      /*trap the asynchronous event*/
      {
        /*first statement to get back the return value*/
        response[i] = '\0';
        break;
      }
      
    }   
    else
    {
      if (i ==  (DATA_RX_MAX_BUFF_SIZE-1)) /* frame overflow */           
      {  
        i = 0; 
      } 
    }
      i++;
      HAL_UART_Receive_IT(&UartHandle, (uint8_t *)aRxBuffer,1) ;
      charnumber++;
  } 
   
	UartHandle.gState = HAL_UART_STATE_READY;
      UartHandle.RxState = HAL_UART_STATE_READY;        /*to be checked since was validated with previous */
      
	return ( RetCode);                            /*version of HAL .. there was not Rx field state*/
}

static void Send( void )
{
  /* USER CODE BEGIN 3 */
  uint16_t pressure = 0;
  int16_t temperature = 0;
  uint16_t humidity = 0;
  uint8_t batteryLevel;
  sensor_t sensor_data;
  
  if ( LORA_JoinStatus () != LORA_SET)
  {
    /*Not joined, try again later*/
    return;
  }
  
//  DBG_PRINTF("SEND REQUEST\n\r");
	//LED_On( LED_RED1 ) ; 
#ifndef CAYENNE_LPP
  int32_t latitude, longitude = 0;
  uint16_t altitudeGps = 0;
#endif
  
#ifdef USE_B_L072Z_LRWAN1
  TimerInit( &TxLedTimer, OnTimerLedEvent );
  
  TimerSetValue(  &TxLedTimer, 200);
  
  LED_On( LED_RED1 ) ; 
  
  TimerStart( &TxLedTimer );  
#endif

  BSP_sensor_Read( &sensor_data );

#ifdef CAYENNE_LPP
  uint8_t cchannel=0;
  temperature = ( int16_t )( sensor_data.temperature * 10 );     /* in °C * 10 */
  pressure    = ( uint16_t )( sensor_data.pressure * 100 / 10 );  /* in hPa / 10 */
  humidity    = ( uint16_t )( sensor_data.humidity * 2 );        /* in %*2     */
  uint32_t i = 0;

  batteryLevel = HW_GetBatteryLevel( );                     /* 1 (very low) to 254 (fully charged) */

  AppData.Port = LPP_APP_PORT;
while(i<transmit_buff_size)
{
	AppData.Buff[i]=response[i];
	i++;
}
//  AppData.Buff[i++] = 1;//cchannel++;
//  AppData.Buff[i++] = 2;//LPP_DATATYPE_BAROMETER;
//  AppData.Buff[i++] = 3;//( pressure >> 8 ) & 0xFF;
//  AppData.Buff[i++] = 4;//pressure & 0xFF;
//  AppData.Buff[i++] = 5;//cchannel++;
//  AppData.Buff[i++] = 6;//LPP_DATATYPE_TEMPERATURE; 
//  AppData.Buff[i++] = 7;//( temperature >> 8 ) & 0xFF;
//  AppData.Buff[i++] = 8;//temperature & 0xFF;
//  AppData.Buff[i++] = 9;//cchannel++;
//  AppData.Buff[i++] = 10;//LPP_DATATYPE_HUMIDITY;
//  AppData.Buff[i++] = 11;//humidity & 0xFF;
#if defined( REGION_US915 ) || defined( REGION_US915_HYBRID ) || defined ( REGION_AU915 )
  /* The maximum payload size does not allow to send more data for lowest DRs */
#else
//  AppData.Buff[i++] = cchannel++;
//  AppData.Buff[i++] = LPP_DATATYPE_DIGITAL_INPUT; 
//  AppData.Buff[i++] = batteryLevel*100/254;
//  AppData.Buff[i++] = cchannel++;
//  AppData.Buff[i++] = LPP_DATATYPE_DIGITAL_OUTPUT; 
//  AppData.Buff[i++] = AppLedStateOn;
#endif  /* REGION_XX915 */
#else  /* not CAYENNE_LPP */

  temperature = ( int16_t )( sensor_data.temperature * 100 );     /* in °C * 100 */
  pressure    = ( uint16_t )( sensor_data.pressure * 100 / 10 );  /* in hPa / 10 */
  humidity    = ( uint16_t )( sensor_data.humidity * 10 );        /* in %*10     */
  latitude = sensor_data.latitude;
  longitude= sensor_data.longitude;
  uint32_t i = 0;

  batteryLevel = HW_GetBatteryLevel( );                     /* 1 (very low) to 254 (fully charged) */

  AppData.Port = LORAWAN_APP_PORT;

#if defined( REGION_US915 ) || defined( REGION_US915_HYBRID ) || defined ( REGION_AU915 )
//  AppData.Buff[i++] = 0;//AppLedStateOn;
//  AppData.Buff[i++] = 0;//( pressure >> 8 ) & 0xFF;
//  AppData.Buff[i++] = 0;//pressure & 0xFF;
//  AppData.Buff[i++] = 0;//( temperature >> 8 ) & 0xFF;
//  AppData.Buff[i++] = 0;//temperature & 0xFF;
//  AppData.Buff[i++] = 0;//( humidity >> 8 ) & 0xFF;
//  AppData.Buff[i++] = 0;//humidity & 0xFF;
//  AppData.Buff[i++] = 0;//batteryLevel;
//  AppData.Buff[i++] = 0;
//  AppData.Buff[i++] = 0;
//  AppData.Buff[i++] = 0;
#else  /* not REGION_XX915 */
//  AppData.Buff[i++] = 1;//AppLedStateOn;
//  AppData.Buff[i++] = 2; //( pressure >> 8 ) & 0xFF;
//  AppData.Buff[i++] = 3;//pressure & 0xFF;
//  AppData.Buff[i++] = 4;//( temperature >> 8 ) & 0xFF;
//  AppData.Buff[i++] = 5;//temperature & 0xFF;
//  AppData.Buff[i++] = 6;//( humidity >> 8 ) & 0xFF;
//  AppData.Buff[i++] = 7;//humidity & 0xFF;
//  AppData.Buff[i++] = 8;//batteryLevel;
//  AppData.Buff[i++] = 9;//( latitude >> 16 ) & 0xFF;
//  AppData.Buff[i++] = 10;//( latitude >> 8 ) & 0xFF;
//  AppData.Buff[i++] = 11;//latitude & 0xFF;
//  AppData.Buff[i++] = 12;//( longitude >> 16 ) & 0xFF;
//  AppData.Buff[i++] = 13;//( longitude >> 8 ) & 0xFF;
//  AppData.Buff[i++] = 14;//longitude & 0xFF;
//  AppData.Buff[i++] = 15;//( altitudeGps >> 8 ) & 0xFF;
//  AppData.Buff[i++] = 16;//altitudeGps & 0xFF;
#endif  /* REGION_XX915 */
#endif  /* CAYENNE_LPP */
 // AppData.BuffSize = i;
  AppData.BuffSize = transmit_buff_size;
  LORA_send( &AppData, LORAWAN_DEFAULT_CONFIRM_MSG_STATE);
  
  /* USER CODE END 3 */
}


static void LORA_RxData( lora_AppData_t *AppData )
{
  /* USER CODE BEGIN 4 */
//  DBG_PRINTF("PACKET RECEIVED ON PORT %d\n\r", AppData->Port);

  switch (AppData->Port)
  {
    case 3:
    /*this port switches the class*/
    if( AppData->BuffSize == 1 )
    {
      switch (  AppData->Buff[0] )
      {
        case 0:
        {
          LORA_RequestClass(CLASS_A);
          break;
        }
        case 1:
        {
          LORA_RequestClass(CLASS_B);
          break;
        }
        case 2:
        {
          LORA_RequestClass(CLASS_C);
          break;
        }
        default:
          break;
      }
    }
    break;
    case LORAWAN_APP_PORT:
    if( AppData->BuffSize == 1 )
    {
      AppLedStateOn = AppData->Buff[0] & 0x01;
      if ( AppLedStateOn == RESET )
      {
        PRINTF("LED OFF\n\r");
        LED_Off( LED_BLUE ) ; 
      }
      else
      {
        PRINTF("LED ON\n\r");
        LED_On( LED_BLUE ) ; 
      }
    }
    break;
  case LPP_APP_PORT:
  {
    AppLedStateOn= (AppData->Buff[2] == 100) ?  0x01 : 0x00;
    if ( AppLedStateOn == RESET )
    {
      PRINTF("LED OFF\n\r");
      LED_Off( LED_BLUE ) ; 
      
    }
    else
    {
      PRINTF("LED ON\n\r");
      LED_On( LED_BLUE ) ; 
    }
    break;
  }
  default:
    break;
  }
  /* USER CODE END 4 */
}

static void OnTxTimerEvent( void )
{
  Send( );
  /*Wait for next tx slot*/
  TimerStart( &TxTimer);
}

static void LoraStartTx(TxEventType_t EventType)
{
  if (EventType == TX_ON_TIMER)
  {
    /* send everytime timer elapses */
//    DBG_PRINTF( "\n\r*** gaurav **\n\r" );
		TimerInit( &TxTimer, OnTxTimerEvent );
    TimerSetValue( &TxTimer,  APP_TX_DUTYCYCLE); 
    OnTxTimerEvent();
  }
  else
  {
//    DBG_PRINTF( "\n\r*** gaurav irq**\n\r" );
		/* send everytime button is pushed */
   // GPIO_InitTypeDef initStruct={0};
  
  //  initStruct.Mode =GPIO_MODE_IT_RISING;
  //  initStruct.Pull = GPIO_PULLUP;
//initStruct.Speed = GPIO_SPEED_HIGH;

   // HW_GPIO_Init( USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, &initStruct );
   // HW_GPIO_SetIrq( USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, 0, Send );
  }
}

static void LORA_ConfirmClass ( DeviceClass_t Class )
{
  PRINTF("switch to class %c done\n\r","ABC"[Class] );

  /*Optionnal*/
  /*informs the server that switch has occurred ASAP*/
  AppData.BuffSize = 0;
  AppData.Port = LORAWAN_APP_PORT;
  
  LORA_send( &AppData, LORAWAN_UNCONFIRMED_MSG);
}

#ifdef USE_B_L072Z_LRWAN1
static void OnTimerLedEvent( void )
{
  LED_Off( LED_RED1 ) ; 
}
#endif
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
