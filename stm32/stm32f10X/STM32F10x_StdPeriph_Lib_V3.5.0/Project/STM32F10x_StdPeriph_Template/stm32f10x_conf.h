/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_conf.h 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Library configuration file.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10x_CONF_H
#define __STM32F10x_CONF_H

/* Includes ------------------------------------------------------------------*/
/* Uncomment/Comment the line below to enable/disable peripheral header file inclusion */
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_adc.h"
#include "stm32f1xx_hal_can.h"
#include "stm32f1xx_hal_cec.h"
#include "stm32f1xx_hal_cortex.h"
#include "stm32f1xx_hal_crc.h"
#include "stm32f1xx_hal_dac.h"
#include "stm32f1xx_hal_dma.h"
#include "stm32f1xx_hal_eth.h"
#include "stm32f1xx_hal_exti.h"
#include "stm32f1xx_hal_flash.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_i2c.h"
#include "stm32f1xx_hal_i2s.h"
#include "stm32f1xx_hal_irda.h"
#include "stm32f1xx_hal_iwdg.h"
#include "stm32f1xx_hal_mmc.h"
#include "stm32f1xx_hal_nand.h"
#include "stm32f1xx_hal_nor.h"
#include "stm32f1xx_hal_pccard.h"
#include "stm32f1xx_hal_pcd.h"
#include "stm32f1xx_hal_pwr.h"
#include "stm32f1xx_hal_rcc.h"
#include "stm32f1xx_hal_rtc.h"
#include "stm32f1xx_hal_sd.h"
#include "stm32f1xx_hal_smartcard.h"
#include "stm32f1xx_hal_spi.h"
#include "stm32f1xx_hal_sram.h"
#include "stm32f1xx_hal_tim.h"
#include "stm32f1xx_hal_uart.h"
#include "stm32f1xx_hal_usart.h"
#include "stm32f1xx_hal_wwdg.h"
#include "stm32f1xx_ll_adc.h"
#include "stm32f1xx_ll_crc.h"
#include "stm32f1xx_ll_dac.h"
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx_ll_exti.h"
#include "stm32f1xx_ll_fsmc.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_i2c.h"
#include "stm32f1xx_ll_pwr.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_rtc.h"
#include "stm32f1xx_ll_sdmmc.h"
#include "stm32f1xx_ll_spi.h"
#include "stm32f1xx_ll_tim.h"
#include "stm32f1xx_ll_usart.h"
#include "stm32f1xx_ll_usb.h"
#include "stm32f1xx_ll_utils.h"
#include "misc.h" /* High level functions for NVIC and SysTick (add-on to CMSIS functions) */

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Uncomment the line below to expanse the "assert_param" macro in the 
   Standard Peripheral Library drivers code */
/* #define USE_FULL_ASSERT    1 */

/* Exported macro ------------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT

/**
  * @brief  The assert_param macro is used for function's parameters check.
  * @param  expr: If expr is false, it calls assert_failed function which reports 
  *         the name of the source file and the source line number of the call 
  *         that failed. If expr is true, it returns no value.
  * @retval None
  */
  #define assert_param(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
  void assert_failed(uint8_t* file, uint32_t line);
#else
  #define assert_param(expr) ((void)0)
#endif /* USE_FULL_ASSERT */

#endif /* __STM32F10x_CONF_H */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
