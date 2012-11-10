#include "stm32f10x_exti.h"
#include "stm32f10x_rcc.h"
#include "misc.h"
#include "mousesensor.h"
#include "srom.h"
#include "stm32f10x.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_it.h"
#include "printf.h"
#include "stm32f10x_gpio.h"
#include "util.h"
#include "protocol.h"
#include <math.h>

volatile struct Mouse_Data_All mouse_data;
volatile struct DualMouseData mouse_integral = {0, 0, 0, 0};

DATA_STAT spi1_datastat;
DATA_STAT spi2_datastat;

#define r_aussen 0.125
#define r_innen  0.10
#define v_max 0.15
#define sqrt3 1,73205081
#define omega -r_innen/r_aussen*y1/sqrt3-r_innen/r_aussen*2/3*x2-r_innen/r_aussen*x1/3

/* gibt die pixel als strecke in meter im mobot_koordinatensystem aus */
void mouse_transformation(const struct DualMouseData * const dual, struct MouseData * const fusion) {
	// Workaround: Rückkopplung der Sollwerte als Mausmesswerte
	fusion->x = last_velocity_command.x * 0.100;
	fusion->y = last_velocity_command.y * 0.100;
	fusion->theta = last_velocity_command.theta * 0.100;


	/*static const float ALPHA = 30;

	fusion->x = cos(ALPHA) * dual->x2 - sin(ALPHA) * dual->y2;
	fusion->y = sin(ALPHA) * dual->x2 + cos(ALPHA) * dual->y2;
	fusion->theta = 0;

	return;

	fusion->x = (dual->x1 + dual->x2) / 2.0;
	fusion->y = (dual->y1 + dual->y2) / 2.0;
	fusion->theta = 0;
	*/

	/*float x1 = -dual->y1;
	float y1 = -dual->x1;
	float x2 = -dual->y2;

	//transform
	dataOut->x = ((x1 - x2) / 3 - y1 * 1.1547) * 5040 / 0.0254; //TODO correct dpi insert
	dataOut->y = (x1 - x2) / 3 * 5040 / 0.0254;
	dataOut->theta = (y1 * 0.5774 + 0.6667 * x2 + x1 * 0.3333) / r_aussen	* 5040 / 0.0254; //TODO eventuell nicht bogenmass
	*/
}


/*void transformToServoSpeed(struct Mouse_Data_DeltaVal* data,
		struct ServoSpeed* sOut, float totzeit) {

	float x1 = -data->delta_y1;
	float y1 = -data->delta_x1;
	float x2 = -data->delta_y2;



	sOut->s1 = (short) ((x1 / 3 - x2 / 3 - 2 / sqrt3 * y1 + omega) / v_max
			* 1000 / totzeit);
	sOut->s2 = (x1 / sqrt3 - x2 / 3 + y1 / sqrt3 + omega) / v_max * 1000
			/ totzeit;
	sOut->s3 = (-y1 / sqrt3 - 2 / 3 * x1 + 2 / 3 * x2 + omega) / v_max * 1000
			/ totzeit; //TODO, nicht sicher, ob richtig abgeschrieben

}*/

//void DMA_init() {
//	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
//	DMA_InitTypeDef DMA_InitStructure;
//
//	// DMA1 Channel2 Config must be used with SPI1_Rx Update
//	DMA_DeInit(DMA1_Channel2);
//
//	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32) &(SPI1->DR);
//	DMA_InitStructure.DMA_MemoryBaseAddr = (u32) mouse_data.motion;
//	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
//	DMA_InitStructure.DMA_BufferSize = sizeof(mouse_data);
//	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
//	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
//	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
//
//	DMA_Init(DMA1_Channel2, &DMA_InitStructure);
//
//	NVIC_InitTypeDef NVIC_InitStructure;
//
//	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel2_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
//
//	DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, ENABLE);
//
//	////DMA1 Channel2 enable
//	DMA_Cmd(DMA1_Channel2, ENABLE);
//
//}

void SPI_init() {
	spi1_datastat = OUTDATED;
	spi2_datastat = OUTDATED;
	spi_init();
}

unsigned char download_firmware(SPI spi) {

	spi_DeselectChip(spi);
	delay_us(2);

	spi_WriteRegister(REG_Configuration_IV, 0x02, spi);
	spi_WriteRegister(REG_SROM_Enable, 0x1d, spi);
	delay_us(800);
	spi_WriteRegister(REG_SROM_Enable, 0x18, spi);
	delay_us(150);

	spi_SelectChip(spi);
	delay_us(1);

	// Wait for aSPI Tx buffer empty
	while (!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE))
		;

	// Send aSPI data
	SPI_I2S_SendData(SPI1, (REG_SROM_Load_Burst | 0x80)); //Address of SROM_Load_Burst register // MSB bit set

	while (!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE))
		;
	delay_us(20); //delay >=15us after the address

	spi_SendData((const unsigned char*) &firmware_3k, firmware_3k_length);
	delay_us(25); //tSCLK-NCS //20

	spi_DeselectChip(spi);
	delay_us(200); //soonest to read SROM_ID is 160uS
	unsigned char id = spi_ReadRegister(REG_SROM_ID, spi);
	return id;
}

void EXIT_init() {
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource3);

	EXTI_InitTypeDef EXTI_InitStructure;

	EXTI_InitStructure.EXTI_Line = EXTI_Line3;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 15;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	//GPIO_SetBits(GPIOC, GPIO_Pin_8);
}

/* check if sensor attached is recognized correctly */
uint8_t check_srom_crc(SPI spi) {

	//Let's test
	spi_WriteRegister(REG_SROM_Enable, 0x15, spi); //write 0x15 to SROM_ENABLE
	delay_ms(20); //wait >=10ms

	uint8_t data_out_lower = spi_ReadRegister(REG_Data_Out_Lower, spi);
	uint8_t data_out_upper = spi_ReadRegister(REG_Data_Out_Upper, spi);

	if ((data_out_upper == 0xbe) && (data_out_lower == 0xef)) {
		return (1);
	} else {
		return (0);
	}

}
/* check if communication with the sensor is available */
uint8_t comm_ok(SPI spi) {

	return ((spi_ReadRegister(REG_Product_ID, spi) == 0x33)
			&& (spi_ReadRegister(REG_Inverse_Product_ID, spi) == 0xcc));
}
/** perform soft reset as described in adns9500-datasheet
 unsigned int timeut		time to wait after powering up the sensor
 */
void soft_reset_adns9500(unsigned int timeout, SPI spi) {
	spi_WriteRegister(REG_Power_Up_Reset, 0x5a, spi); //write to the Power_Up_Reset register
	delay_ms(timeout); //wait for at least 50mS //increasing this to 500mS improved stability

	spi_ReadRegister(REG_SROM_ID, spi);
	spi_ReadRegister(0x02, spi);
	spi_ReadRegister(0x03, spi);
	spi_ReadRegister(0x04, spi);
	spi_ReadRegister(0x05, spi);
	spi_ReadRegister(0x06, spi);
	char out = spi_ReadRegister(REG_LASER_CTRL0, spi);
	spi_WriteRegister(REG_LASER_CTRL0, out & 0xf0, spi);

}
/* hard reset using special protocol
 time consuming and not always necessary
 */
void reset_adns9500(SPI spi) {
	uint8_t do_init = 1;

	while (do_init) {
		//Reset the serial port
		spi_DeselectChip(spi);
		delay_ms(40); //time to wake up
		spi_SelectChip(spi);
		delay_ms(2);
		spi_DeselectChip(spi);
		delay_ms(2);

		soft_reset_adns9500(50, spi); //CHECK TIME

		if (comm_ok(spi)) {
			do_init = 0;
		} else {
			delay_ms(100);
		}
	}
}
/* idle loop until communication with sensor possible
 resets sensor after 20 tries to prevent infinite loops */
void wait_for_comm(SPI spi) {
	uint8_t do_wait_for_comm = 1;
	unsigned int counter = 0;
	while (do_wait_for_comm) {
		counter++;

		if (comm_ok(spi)) {
			do_wait_for_comm = 0;
		} else {
			delay_ms(100);
		}
		if (counter > 20) {
			counter = 0;
			reset_adns9500(spi);
		}
	}
}

void Sensor_ReadByBurst(SPI spi) {
	spi_SelectChip(spi);
	delay_us(2);
	while (!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE))
		;

	SPI_I2S_SendData(SPI1, REG_Motion_Burst);
	while (!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE))
		;
	delay_us(120);

	spi_GetData((unsigned char *) &mouse_data, 14);
	spi_DeselectChip(spi);
	delay_us(15);
}

void EXTI15_10_IRQHandler(void) {
	if (EXTI_GetITStatus(EXTI_Line11) != RESET) {

		EXTI_ClearITPendingBit(EXTI_Line11);
	}
}

void EXTI3_IRQHandler(void) {
	if (EXTI_GetITStatus(EXTI_Line3)) {
		spi1_datastat = UPDATED;
		EXTI_ClearITPendingBit(EXTI_Line3);
	}
}

int Sensor_init(SPI spi) {

	delay_ms(100);
	//EXIT_init();

	soft_reset_adns9500(80, spi);
	uint8_t check = 10;
	// perform up to 10 communication requests
	for (; check > 0 && !comm_ok(spi); check--) {
		delay_ms(100);
	}
	if (!comm_ok(spi)) { // no communication possible
		return 0; // return FALSE
	}

	uint8_t go_on = 1;
	while (go_on) {
		download_firmware(spi);

		wait_for_comm(spi);
		if (check_srom_crc(spi)) {
			go_on = 0;
		}
	}

	clear_bits_addr(REG_Configuration_II, 0x18, spi);

	spi_WriteRegister(REG_Shutter_Max_Bound_Lower, 0x20, spi); //0x20
	spi_WriteRegister(REG_Shutter_Max_Bound_Upper, 0x4e, spi); //0x4e  0x6e
	//
	//	//Frame_Period_Min  0x0fa0 - 61a8
	spi_WriteRegister(REG_Frame_Period_Min_Bound_Lower, 0xa0, spi); //0xa0
	spi_WriteRegister(REG_Frame_Period_Min_Bound_Lower, 0x0f, spi); //0x0f
	//
	//	//Frame_Period_Max  Framerate=47000000/frame_period_max  (fps)
	spi_WriteRegister(REG_Frame_Period_Max_Bound_Lower, 0xc0, spi); //0xc0
	spi_WriteRegister(REG_Frame_Period_Max_Bound_Upper, 0x5d, spi); //0x5d

	//Resolution
	spi_WriteRegister(REG_Configuration_I, 0x38, spi); // auflösung auf ~5040 dpi => 1 pixel ~10,08um

	clear_bits_addr(REG_LASER_CTRL0, 0x01, spi);
	clear_bits_addr(REG_LASER_CTRL0, 0x0e, spi);

	clear_bits_addr(REG_LASER_CTRL0, 0x01, spi);

	delay_ms(10);

	//printf("Frame Period: %hi\n", frmae_period);

	return 1;
}

