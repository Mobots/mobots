/**
  ******************************************************************************
  * @file    Demo/src/main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    09/13/2010
  * @brief   Main program body
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "system_stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "usart.h"
#include "printf.h"
#include "servo.h"
#include "mousesensor.h"
#include "spi_1.h"
#include "misc.h"
#include "led.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define  LSE_FAIL_FLAG  0x80
#define  LSE_PASS_FLAG  0x100
/* Private macro -------------------------------------------------------------*/
/* Private consts ------------------------------------------------------------*/
void Delay(uint32_t nTime);


/* Private variables ---------------------------------------------------------*/
unsigned int time_in_ms = 0;
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */

int main(void)
{
//SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
	led_init();
	GPIO_ResetBits(GPIOC, GPIO_Pin_9);
	GPIO_ResetBits(GPIOC, GPIO_Pin_8);
//	SysTick_Config(SystemCoreClock / 1000);
	USART1_Init(USART_USE_INTERRUPTS);
//	servo_init();
	Sensor_init();

	GPIO_SetBits(GPIOC, GPIO_Pin_9);
	while(1)
	{
		if(spi1_datastat == UPDATED){
			s16 delta_x, delta_y;

			delta_x = (s16)mouse_data.d_xl | ((s16)mouse_data.d_xl<<8);
			delta_y = (s16)mouse_data.d_yl | ((s16)mouse_data.d_yl<<8);

			printf("DeltaX: %hi\n DeltaY: %hi", delta_x, delta_y);
		}

	}

}


void SysTick_Handler(void)
{
	time_in_ms++;
}

unsigned int getTime(){
	return time_in_ms;
}

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in microseconds.
  * @retval None
  */


/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */


#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
