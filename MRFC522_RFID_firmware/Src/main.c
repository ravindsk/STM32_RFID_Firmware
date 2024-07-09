#include <stdio.h>
#include "stm32f4xx.h"
#include "fpu.h"
#include "uart.h"
#include "timebase.h"
#include "bsp.h"
#include "adc.h"
#include "mfrc522.h"
#include "string.h"
#include "stdio.h"
/*Modules:
 * FPU
 * UART
 * TIMEBASE
 * GPIO (BSP)
 * ADC
 * */
#define TEMP_BUFF_SIZE  10

uint8_t temp_buffer [TEMP_BUFF_SIZE];

uint8_t status;
uint8_t curr_card_id[CARD_ID_LEN];
static void print_array_hex(const uint8_t *array,size_t length);

#define  GPIOAEN		(1U<<0)
#define  PIN5			(1U<<5)
#define  LED_PIN		PIN5

bool btn_state;
uint32_t sensor_value;
int main()
{
	/*Enable FPU*/
	fpu_enable();

	/*Initialize debug UART*/
	debug_uart_init();

	/*Initialize timebase*/
	timebase_init();

	/*Initialize LED*/
	led_init();

	/*Initialize Push button*/
	button_init();

	/*Initialize ADC*/
	pa1_adc_init();

	/*Start conversion*/
	start_conversion();

	/*Initiliaze NFC module*/
	mfrc522_init();

	printf("Initializing....\n\r");

	delay(100);

	while(1)
	{

		status = mfrc522_request(PICC_REQIDL,temp_buffer);
		status = mfrc522_anticoll(temp_buffer);

		if(status == MI_OK)
		{
			memcpy(curr_card_id,temp_buffer,CARD_ID_LEN);
			print_array_hex(curr_card_id,CARD_ID_LEN);
		}
	}
}


static void print_array_hex(const uint8_t *array,size_t length)
{
	for(size_t i=0;i<length;i++)
	{
		printf("%02X  ", array[i]);
	}
	printf("\r\n");

}
