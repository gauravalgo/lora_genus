/******************************************************************************
 * @file    hw_usart.c
 * @author  MCD Application Team
 * @version V1.1.3
 * @date    20-December-2017
 * @brief   This file provides code for the configuration of the USART
 *          instances.
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


/*private variables*/
static struct {
  char buffTx[256];                         /* structure have to be simplified*/
  char buffRx[256];
  //int rx_idx_free;
  //int rx_idx_toread;
  HW_LockTypeDef Lock;
  __IO HAL_UART_StateTypeDef gState;
  __IO HAL_UART_StateTypeDef RxState;
} uart_context;
static int rx_idx_free=0;
static int rx_idx_toread=0;
/* private function */
static void receive(char rx);
static void g_UART_DMAAbortOnError(DMA_HandleTypeDef *hdma)
{
  UART_HandleTypeDef* huart = (UART_HandleTypeDef*)(hdma->Parent);
  huart->RxXferCount = 0;
  huart->TxXferCount = 0;

  HAL_UART_ErrorCallback(huart);
}

static void g_UART_EndRxTransfer(UART_HandleTypeDef *huart)
{
  /* Disable RXNE, PE and ERR (Frame error, noise error, overrun error) interrupts */
  CLEAR_BIT(huart->Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));
  CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);

  /* At end of Rx process, restore huart->RxState to Ready */
  huart->RxState = HAL_UART_STATE_READY;
}
static HAL_StatusTypeDef g_UART_Receive_IT(UART_HandleTypeDef *huart)
{
  uint16_t* tmp;
  uint16_t  uhMask = huart->Mask;
  uint16_t  uhdata;

  /* Check that a Rx process is ongoing */
  if(huart->RxState == HAL_UART_STATE_BUSY_RX)
  {
    uhdata = (uint16_t) READ_REG(huart->Instance->RDR);
    if ((huart->Init.WordLength == UART_WORDLENGTH_9B) && (huart->Init.Parity == UART_PARITY_NONE))
    {
      tmp = (uint16_t*) huart->pRxBuffPtr ;
      *tmp = (uint16_t)(uhdata & uhMask);
			//receive(*((huart->pRxBuffPtr)-2));
			//receive(*((huart->pRxBuffPtr)-1));
      huart->pRxBuffPtr +=2;
    }
    else
    {
      *huart->pRxBuffPtr++ = (uint8_t)(uhdata & (uint8_t)uhMask);
			receive(*((huart->pRxBuffPtr)-1));
			//uint8_t aTxStartMessage[] = "\r\nreceive\r\n";
		//HAL_UART_Transmit_IT(huart, (uint8_t *)aTxStartMessage, sizeof(aTxStartMessage));
    }

    if(--huart->RxXferCount == 0U)
    {
      /* Disable the UART Parity Error Interrupt and RXNE interrupt*/
      CLEAR_BIT(huart->Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));

      /* Disable the UART Error Interrupt: (Frame error, noise error, overrun error) */
      CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);

      /* Rx process is completed, restore huart->RxState to Ready */
      huart->RxState = HAL_UART_STATE_READY;

      HAL_UART_RxCpltCallback(huart);

      return HAL_OK;
    }

    return HAL_OK;
  }
  else
  {
    /* Clear RXNE interrupt flag */
    __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);

    return HAL_BUSY;
  }
}
static HAL_StatusTypeDef g_UART_Transmit_IT(UART_HandleTypeDef *huart)
{
  uint16_t* tmp;

  /* Check that a Tx process is ongoing */
  if (huart->gState == HAL_UART_STATE_BUSY_TX)
  {
    if(huart->TxXferCount == 0U)
    {
      /* Disable the UART Transmit Data Register Empty Interrupt */
      CLEAR_BIT(huart->Instance->CR1, USART_CR1_TXEIE);

      /* Enable the UART Transmit Complete Interrupt */
      SET_BIT(huart->Instance->CR1, USART_CR1_TCIE);

      return HAL_OK;
    }
    else
    {
      if ((huart->Init.WordLength == UART_WORDLENGTH_9B) && (huart->Init.Parity == UART_PARITY_NONE))
      {
        tmp = (uint16_t*) huart->pTxBuffPtr;
        huart->Instance->TDR = (*tmp & (uint16_t)0x01FFU);
        huart->pTxBuffPtr += 2U;
      }
      else
      {
        huart->Instance->TDR = (uint8_t)(*huart->pTxBuffPtr++ & (uint8_t)0xFFU);
      }
      huart->TxXferCount--;

      return HAL_OK;
    }
  }
  else
  {
    return HAL_BUSY;
  }
}
static HAL_StatusTypeDef g_UART_EndTransmit_IT(UART_HandleTypeDef *huart)
{
  /* Disable the UART Transmit Complete Interrupt */
  CLEAR_BIT(huart->Instance->CR1, USART_CR1_TCIE);

  /* Tx process is ended, restore huart->gState to Ready */
  huart->gState = HAL_UART_STATE_READY;

  HAL_UART_TxCpltCallback(huart);

  return HAL_OK;
}
/******************************************************************************
  * @brief Handler on Rx IRQ
  * @param handle to the UART
  * @retval void
******************************************************************************/
void HW_UART_Modem_IRQHandler(UART_HandleTypeDef *huart)
{
//  uint32_t isrflags   = READ_REG(huart->Instance->ISR);
//  uint32_t cr1its     = READ_REG(huart->Instance->CR1);
//  uint32_t cr3its = READ_REG(huart->Instance->CR3);;
//  uint32_t errorflags;
////  uint16_t  uhMask = huart->Mask;
////  uint16_t  uhdata;
//  int rx_ready = 0;


//uint8_t aTxStartMessage[] = "\r\nHW_UART_Modem_IRQHandler\r\n";
//		HAL_UART_Transmit_IT(huart, (uint8_t *)aTxStartMessage, sizeof(aTxStartMessage));	
//    /* UART wakeup from Stop mode interrupt occurred ---------------------------*/
//    if(((isrflags & USART_ISR_WUF) != RESET) && ((cr3its & USART_CR3_WUFIE) != RESET))
//    {
//      	uint8_t aTxStartMessage[] = "\r\nUSART_ISR_WUF\r\n";
//		HAL_UART_Transmit_IT(huart, (uint8_t *)aTxStartMessage, sizeof(aTxStartMessage));	
//			__HAL_UART_CLEAR_IT(huart, UART_CLEAR_WUF);

//       /* forbid stop mode */
//			 LPM_SetStopMode(LPM_UART_RX_Id , LPM_Disable );

//      /* Enable the UART Data Register not empty Interrupts */
//      SET_BIT(huart->Instance->CR1, USART_CR1_RXNEIE);

//      /* Set the UART state ready to be able to start again the process */
//      huart->gState  = HAL_UART_STATE_READY;
//      huart->RxState = HAL_UART_STATE_READY;

//    }


//	/* UART in mode Receiver ---------------------------------------------------*/
//    if(((isrflags & USART_ISR_RXNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET))
//    {
//			uint8_t aTxStartMessage[] = "\r\nUSART_ISR_RXNE\r\n";
//		HAL_UART_Transmit_IT(huart, (uint8_t *)aTxStartMessage, sizeof(aTxStartMessage));
//		/* Check that a Rx process is ongoing */
//		if(huart->RxState == HAL_UART_STATE_BUSY_RX)
//		{
//			uint8_t aTxStartMessage[] = "\r\nHAL_UART_STATE_BUSY_RX\r\n";
//		HAL_UART_Transmit_IT(huart, (uint8_t *)aTxStartMessage, sizeof(aTxStartMessage));	
//		        /*RXNE flag is auto cleared by reading the data*/
//                        *huart->pRxBuffPtr++ = (uint8_t)READ_REG(huart->Instance->RDR);

//                        /* allow stop mode*/
//                        LPM_SetStopMode(LPM_UART_RX_Id , LPM_Enable );

//			if(--huart->RxXferCount == 0U)
//                        {
//				CLEAR_BIT(huart->Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));
//				CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);
//				huart->RxState = HAL_UART_STATE_READY;
//				rx_ready = 1;  /* not used RxTC callback*/
//			}
//		}
//		else
//		{
//						uint8_t aTxStartMessage[] = "\r\nelse\r\n";
//		HAL_UART_Transmit_IT(huart, (uint8_t *)aTxStartMessage, sizeof(aTxStartMessage));			
//			/* Clear RXNE interrupt flag */
//                   __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);
//                  return;
//		}
//    }

//	  /* If error occurs */
//     errorflags = (isrflags & (uint32_t)(USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE));
//     if (errorflags != RESET)
//     {
//	   uint8_t aTxStartMessage[] = "\r\nerror\r\n";
//		HAL_UART_Transmit_IT(huart, (uint8_t *)aTxStartMessage, sizeof(aTxStartMessage));	
//			 /* Error on receiving */
//        __HAL_UART_CLEAR_IT(huart, UART_CLEAR_PEF);
//        __HAL_UART_CLEAR_IT(huart, UART_CLEAR_FEF);
//        __HAL_UART_CLEAR_IT(huart, UART_CLEAR_OREF);
//        __HAL_UART_CLEAR_IT(huart, UART_CLEAR_NEF);
////	   *((huart->pRxBuffPtr)-1) = 0x01;           /*we skip the overrun case*/
//	   rx_ready = 1;
//	 }

//	if(rx_ready)
//	{
//	  	
//		/*character in the ring buffer*/
//	  receive(*((huart->pRxBuffPtr)-1));
//	}
 uint32_t isrflags   = READ_REG(huart->Instance->ISR);
  uint32_t cr1its     = READ_REG(huart->Instance->CR1);
  uint32_t cr3its;
  uint32_t errorflags;

  /* If no error occurs */
  errorflags = (isrflags & (uint32_t)(USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE));
  if (errorflags == RESET)
  {
    /* UART in mode Receiver ---------------------------------------------------*/
    if(((isrflags & USART_ISR_RXNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET))
    {
      g_UART_Receive_IT(huart);
      return;
    }
  }  

  /* If some errors occur */
  cr3its = READ_REG(huart->Instance->CR3);
  if(   (errorflags != RESET)
     && (   ((cr3its & USART_CR3_EIE) != RESET)
         || ((cr1its & (USART_CR1_RXNEIE | USART_CR1_PEIE)) != RESET)) )
  {
    /* UART parity error interrupt occurred -------------------------------------*/
    if(((isrflags & USART_ISR_PE) != RESET) && ((cr1its & USART_CR1_PEIE) != RESET))
    {
      __HAL_UART_CLEAR_IT(huart, UART_CLEAR_PEF);

      huart->ErrorCode |= HAL_UART_ERROR_PE;
    }

    /* UART frame error interrupt occurred --------------------------------------*/
    if(((isrflags & USART_ISR_FE) != RESET) && ((cr3its & USART_CR3_EIE) != RESET))
    {
      __HAL_UART_CLEAR_IT(huart, UART_CLEAR_FEF);

      huart->ErrorCode |= HAL_UART_ERROR_FE;
    }

    /* UART noise error interrupt occurred --------------------------------------*/
    if(((isrflags & USART_ISR_NE) != RESET) && ((cr3its & USART_CR3_EIE) != RESET))
    {
      __HAL_UART_CLEAR_IT(huart, UART_CLEAR_NEF);

      huart->ErrorCode |= HAL_UART_ERROR_NE;
    }
    
    /* UART Over-Run interrupt occurred -----------------------------------------*/
    if(((isrflags & USART_ISR_ORE) != RESET) &&
       (((cr1its & USART_CR1_RXNEIE) != RESET) || ((cr3its & USART_CR3_EIE) != RESET)))
    {
      __HAL_UART_CLEAR_IT(huart, UART_CLEAR_OREF);

      huart->ErrorCode |= HAL_UART_ERROR_ORE;
    }

    /* Call UART Error Call back function if need be --------------------------*/
    if(huart->ErrorCode != HAL_UART_ERROR_NONE)
    {
      /* UART in mode Receiver ---------------------------------------------------*/
      if(((isrflags & USART_ISR_RXNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET))
      {
        g_UART_Receive_IT(huart);
      }

      /* If Overrun error occurs, or if any error occurs in DMA mode reception,
         consider error as blocking */
      if (((huart->ErrorCode & HAL_UART_ERROR_ORE) != RESET) ||
          (HAL_IS_BIT_SET(huart->Instance->CR3, USART_CR3_DMAR)))
      {  
        /* Blocking error : transfer is aborted
           Set the UART state ready to be able to start again the process,
           Disable Rx Interrupts, and disable Rx DMA request, if ongoing */
        g_UART_EndRxTransfer(huart);

        /* Disable the UART DMA Rx request if enabled */
        if (HAL_IS_BIT_SET(huart->Instance->CR3, USART_CR3_DMAR))
        {
          CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAR);

          /* Abort the UART DMA Rx channel */
          if(huart->hdmarx != NULL)
          {
            /* Set the UART DMA Abort callback : 
               will lead to call HAL_UART_ErrorCallback() at end of DMA abort procedure */
            huart->hdmarx->XferAbortCallback = g_UART_DMAAbortOnError;

            /* Abort DMA RX */
            if(HAL_DMA_Abort_IT(huart->hdmarx) != HAL_OK)
            {
              /* Call Directly huart->hdmarx->XferAbortCallback function in case of error */
              huart->hdmarx->XferAbortCallback(huart->hdmarx);
            }
          }
          else
          {
            /* Call user error callback */
            HAL_UART_ErrorCallback(huart);
          }
        }
        else
        {
          /* Call user error callback */
          HAL_UART_ErrorCallback(huart);
        }
      }
      else
      {
        /* Non Blocking error : transfer could go on. 
           Error is notified to user through user error callback */
        HAL_UART_ErrorCallback(huart);
        huart->ErrorCode = HAL_UART_ERROR_NONE;
      }
    }
    return;

  } /* End if some error occurs */

  /* UART wakeup from Stop mode interrupt occurred ---------------------------*/
  if(((isrflags & USART_ISR_WUF) != RESET) && ((cr3its & USART_CR3_WUFIE) != RESET))
  {
    __HAL_UART_CLEAR_IT(huart, UART_CLEAR_WUF);
    /* Set the UART state ready to be able to start again the process */
    huart->gState  = HAL_UART_STATE_READY;
    huart->RxState = HAL_UART_STATE_READY;
    HAL_UARTEx_WakeupCallback(huart);
    return;
  }

  /* UART in mode Transmitter ------------------------------------------------*/
  if(((isrflags & USART_ISR_TXE) != RESET) && ((cr1its & USART_CR1_TXEIE) != RESET))
  {
    g_UART_Transmit_IT(huart);
    return;
  }

  /* UART in mode Transmitter (transmission end) -----------------------------*/
  if(((isrflags & USART_ISR_TC) != RESET) && ((cr1its & USART_CR1_TCIE) != RESET))
  {
    g_UART_EndTransmit_IT(huart);
    return;
  }

}


/******************************************************************************
  * @brief To check if data has been received
  * @param none
  * @retval Reset no data / set data
******************************************************************************/
FlagStatus HW_UART_Modem_IsNewCharReceived(void)
{
  FlagStatus status;

//  BACKUP_PRIMASK();
  uint32_t primask_bit= __get_PRIMASK();
//  DISABLE_IRQ();
  __disable_irq();

  status = ((rx_idx_toread == rx_idx_free) ? RESET : SET);

//  RESTORE_PRIMASK();
  __set_PRIMASK(primask_bit);
  return status;
}




/******************************************************************************
  * @brief Get the received character
  * @param none
  * @retval Return the data received
******************************************************************************/
uint8_t HW_UART_Modem_GetNewChar(void)
{
  uint8_t NewChar;

//  BACKUP_PRIMASK();
  uint32_t primask_bit= __get_PRIMASK();
//  DISABLE_IRQ();
  __disable_irq();

  NewChar = uart_context.buffRx[rx_idx_toread];
  rx_idx_toread = (rx_idx_toread + 1) % sizeof(uart_context.buffRx);

//  RESTORE_PRIMASK();
  __set_PRIMASK(primask_bit);
  return NewChar;
}



/******************************************************************************
  * @brief Store in ring buffer the received character
  * @param none
  * @retval none
******************************************************************************/

static void receive(char rx)
{
  int next_free;
	//uart_context.rx_idx_free=0;
  /** no need to clear the RXNE flag because it is auto cleared by reading the data*/
  uart_context.buffRx[rx_idx_free] = rx;
//  //DBG_PRINTF( "\n\r*** receive **\n\r" );
	
	next_free = (rx_idx_free + 1) % sizeof(uart_context.buffRx);
  if (next_free != rx_idx_toread)
  {
    /* this is ok to read as there is no buffer overflow in input */
    rx_idx_free = next_free;
  }
//  else
//  {
//    /* force the end of a command in case of overflow so that we can process it */
//    uart_context.buffRx[uart_context.rx_idx_free] = '\r';
//    PRINTF("uart_context.buffRx buffer overflow %d\r\n");
//  }
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
