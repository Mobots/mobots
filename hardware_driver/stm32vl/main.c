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
#include "fixmath.h"
#include "util.h"
#include "protocol.h"
#include "engine.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define  LSE_FAIL_FLAG  0x80
#define  LSE_PASS_FLAG  0x100
/* Private macro -------------------------------------------------------------*/
/* Private consts ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
unsigned int time_in_ms = 0;
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
	servo_setAngle(Servo_1, 0);
	servo_setAngle(Servo_2, 0);
	servo_setAngle(Servo_3, 0);
	if (Sensor_init(SPI_1) && Sensor_init(SPI_2)) {
		//print("Sensor Initialisierung erfolgreich!\n");
		GPIO_SetBits(GPIOC, GPIO_Pin_9); // läuft der Sensorinit durch geht die grüne led an
	} else {
		//print("Sensor Initialisierung fehlgeschlagen!\n");
	}
	SysTick_Config(SystemCoreClock / 200); // Systick auf 10ms stellen

	GPIO_SetBits(GPIOC, GPIO_Pin_8);
	//---------------------------------------------------------------------

	print("Init done\n");

	printf("%d", sizeof(struct Mouse_Data_DeltaVal));

	struct Mouse_Data_DeltaVal temp, null;
	struct Mouse_Data_DeltaValOut out;
	temp.delta_x1 = 0;
	temp.delta_y1 = 0;
	null.delta_x1 = 0;
	null.delta_y1 = 0;
	temp.delta_x2 = 0;
	temp.delta_y2 = 0;
	null.delta_x2 = 0;
	null.delta_y2 = 0;
	while (1) {
		delay_ms(1000);
		protocol_receiveData();

		if (delta_vals.delta_x1 || delta_vals.delta_y1 || delta_vals.delta_x2 || delta_vals.delta_y2) {
			temp = delta_vals;
			delta_vals = null;
			transformMouseToCoordinateSystem(&temp,&out);
			protocol_sendData(SensorData_DeltaVal, (unsigned char*) &out,
					sizeof(struct Mouse_Data_DeltaValOut));
		}
	}
	return 0;
}

void SysTick_Handler() {
	//time_in_ms++;
	if (spi_ReadRegister(REG_Motion, SPI_1)) {
		delta_vals.delta_x1 += (s16) spi_ReadRegister(REG_Delta_X_L, SPI_1)
				| (s16) (spi_ReadRegister(REG_Delta_X_H, SPI_1) << 8);
		delta_vals.delta_y1 += (s16) spi_ReadRegister(REG_Delta_Y_L, SPI_1)
				| (s16) (spi_ReadRegister(REG_Delta_Y_H, SPI_1) << 8);
	}
	if (spi_ReadRegister(REG_Motion, SPI_2)) {
		delta_vals.delta_x2 += (s16) spi_ReadRegister(REG_Delta_X_L, SPI_2)
				| (s16) (spi_ReadRegister(REG_Delta_X_H, SPI_2) << 8);
		delta_vals.delta_y2 += (s16) spi_ReadRegister(REG_Delta_Y_L, SPI_2)
				| (s16) (spi_ReadRegister(REG_Delta_Y_H, SPI_2) << 8);
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
