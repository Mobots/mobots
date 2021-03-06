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
#include "util.h"
#include "protocol.h"
#include "controller.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define  LSE_FAIL_FLAG  0x80
#define  LSE_PASS_FLAG  0x100
/* Private macro -------------------------------------------------------------*/
/* Private consts ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
unsigned int time_in_ms = 0;

static const int CONTROL_TIME=10;		//in ms, every X control the velocity
static const int SEND_MESSAGE_TIME=100;	//in ms, every X send messages
static const float CONTORL_T_I=0.1;	//in s
static const float CONTROL_K_P=15;	//gain factor

static struct MouseData output_ges = {0, 0, 0};

static struct DualMouseData cache_ges = {0, 0, 0, 0};
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Main program.
 * @param  None
 * @retval None
 */

int main() {
	//-------------------------------------------------------------------
	SystemInit(); // config of all clocks
	led_init();

	USART1_Init(USART_USE_INTERRUPTS);
	SPI_init();
	delay_ms(100);
	protocol_init(TRUE);

	servo_init();
	struct ServoSpeed initial_speed = {{0, 0, 0}};
	servo_set(&initial_speed);

	int aasd = sizeof(enum PROTOCOL_IDS);

	if(!Sensor_init(SPI_1)) {
		print("Maussensor 1 geht nicht.");
		assert_param(0);
	}
	if(!Sensor_init(SPI_2)) {
		print("Maussensor 2 geht nicht.");
        assert_param(0);
	}



	//assert_param( Sensor_init(SPI_1) && Sensor_init(SPI_2) );

	SysTick_Config(SystemCoreClock / 100); // Systick auf 10 ms stellen

	control_init(CONTORL_T_I, CONTROL_K_P, CONTROL_TIME);	//configures controller

	//---------------------------------------------------------------------

	print("Initialisation done.\n");


	while (1)
	{
		protocol_receiveData();
	}

	return 0;
}

void SysTick_Handler() {
	time_in_ms += 10;
	if (time_in_ms >= 1000)
		time_in_ms = 0;


	if (spi_ReadRegister(REG_Motion, SPI_1)) {
		mouse_integral.x1 += (s16) spi_ReadRegister(REG_Delta_X_L, SPI_1)	| (s16) (spi_ReadRegister(REG_Delta_X_H, SPI_1) << 8);
		mouse_integral.y1 += (s16) spi_ReadRegister(REG_Delta_Y_L, SPI_1)	| (s16) (spi_ReadRegister(REG_Delta_Y_H, SPI_1) << 8);
	}
	if (spi_ReadRegister(REG_Motion, SPI_2)) {
		mouse_integral.x2 += (s16) spi_ReadRegister(REG_Delta_X_L, SPI_2)	| (s16) (spi_ReadRegister(REG_Delta_X_H, SPI_2) << 8);
		mouse_integral.y2 += (s16) spi_ReadRegister(REG_Delta_Y_L, SPI_2)	| (s16) (spi_ReadRegister(REG_Delta_Y_H, SPI_2) << 8);
	}

		static const struct DualMouseData reset = {0, 0, 0, 0};
		static struct DualMouseData cache = {0, 0, 0, 0};
		static struct MouseData output = {0, 0, 0};
		static struct MouseData output_temp = {0, 0, 0};

		cache = mouse_integral;

		cache_ges.x1+=cache.x1;
		cache_ges.y1+=cache.y1;
		cache_ges.x2+=cache.x2;
		cache_ges.y2+=cache.y2;
		output_ges.x+=output.x;
		output_ges.y+=output.y;
		output_ges.theta+=output.theta;
		mouse_transformation(&cache_ges, &output_temp);

		mouse_transformation(&cache, &output);
		mouse_integral = reset;

		if (time_in_ms % CONTROL_TIME == 0) {
			//control(&output);
		}


		if (time_in_ms % SEND_MESSAGE_TIME == 0) {
			if (output.x != 0 || output.y != 0 || output.theta !=0)
				protocol_sendData(MOUSE_DATA, (unsigned char*) &output, sizeof(struct MouseData));
		}



	// LED mit 10 Hz blinken lassen
	if (time_in_ms == 10) {
		GPIO_SetBits(GPIOC, GPIO_Pin_8);
	}
	if (time_in_ms == 510) {
		GPIO_ResetBits(GPIOC, GPIO_Pin_8);
	}
}

unsigned int getTime() {
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
		GPIO_SetBits(GPIOC, GPIO_Pin_9);
		delay_ms(50);
		GPIO_ResetBits(GPIOC, GPIO_Pin_9);
		delay_ms(50);

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
