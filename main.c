/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "bboard_system.h"
#include "stm32f1xx_hal.h"
#include "crc.h"


#define DATA_SIZE 4
#define BAUDRATE 115200
#define WINDOW_SIZE 4
#define MAX_SEQUENCE 7
#define RR 0
#define REJ 3
#define START_BYTE 0x02
#define STOP_BYTE 0x03
#define PC2ARM 0xC0
#define ARM2PC 0xD0
#define S_FRAME_SIZE 5
#define I_FRAME_SIZE (DATA_SIZE + S_FRAME_SIZE) 
#define TIMEOUT_FREQ (BAUDRATE / (4 * I_FRAME_SIZE))

void SystemClock_Config(void);
void SPI_Init(void);
void TIMER_Init(void);
void receiveFunc(void);
void transmitFunc(void);
void sendACK(uint8_t, uint8_t);
void eepromWrite(void);

uint8_t isTransmit, timerOn;
uint8_t timeout_flag;
uint8_t frameBuffer[WINDOW_SIZE][DATA_SIZE + 5];
uint8_t sendBuffer[WINDOW_SIZE][DATA_SIZE + 5];
uint8_t receiveByte, nByte, seq_number;
uint32_t headReceive, tailReceive, headTransmit, tailTransmit;

SPI_HandleTypeDef spiHandle;
TIM_HandleTypeDef timerHandle;
/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
	/* Configure the system clock to 72 MHz */
	SystemClock_Config();
    
    /* Init CRC */
    crcInit();
	
	HAL_Init();
	bboard_usart1_init(115200);
	bboard_led_green_init();
	bboard_led_red_init();
	headReceive = 0;
	tailReceive = 0;
	nByte = 0;
	seq_number = 0;
	
	HAL_UART_Receive_IT(&bboard_uart1_handle, &receiveByte, 1);
	
	/* Infinite loop */
	while (1)
	{
		if (headReceive < tailReceive)
		{
			receiveFunc();
		}
		
		if (isTransmit)
		{
			if (tailTransmit - headTransmit < WINDOW_SIZE)
			{
				transmitFunc();
			}
			else 
			{
				if (!timerOn)
				{
					timerOn = 1;
					timeout_flag = 0;
					__HAL_TIM_CLEAR_IT(&timerHandle, TIM_IT_UPDATE);
					__HAL_TIM_CLEAR_FLAG(&timerHandle, TIM_FLAG_UPDATE);
					__HAL_TIM_SET_COUNTER(&timerHandle, 0);
					HAL_TIM_Base_Start_IT(&timerHandle);
				}
				if (timeout_flag)
				{
					sendACK(1, RR);
					timeout_flag = 0;
					__HAL_TIM_CLEAR_IT(&timerHandle, TIM_IT_UPDATE);
					__HAL_TIM_CLEAR_FLAG(&timerHandle, TIM_FLAG_UPDATE);
					__HAL_TIM_SET_COUNTER(&timerHandle, 0);
					HAL_TIM_Base_Start_IT(&timerHandle);
				}
			}
		}
	}
}

inline void sendACK(uint8_t poll, uint8_t status)
{
	uint8_t ack[S_FRAME_SIZE];
	uint16_t myCrc;
	
	ack[0] = START_BYTE;
	ack[1] = 0x80 | status << 5 | poll << 3 | seq_number;
	myCrc = crcFast(ack, S_FRAME_SIZE - 3);
	ack[2] = myCrc & 0x00FF;
	ack[3] = myCrc >> 8;
	ack[4] = STOP_BYTE;
	HAL_UART_Transmit_IT(&bboard_uart1_handle, ack, S_FRAME_SIZE);
}

inline void receiveFunc()
{
	uint8_t frame[I_FRAME_SIZE];
	
	uint16_t myCrc, receiveCrc;
	strcpy((char *)frame, (char *)frameBuffer[headReceive % WINDOW_SIZE]);
	headReceive++;
	
	//I-frame
	if (frame[1] >> 7 == 0)
	{
		//check crc
		myCrc = crcFast(frame, I_FRAME_SIZE - 3);
		receiveCrc = frame[I_FRAME_SIZE - 2] << 8 | frame[I_FRAME_SIZE - 3];
		if (myCrc != receiveCrc)
		{
			sendACK(0, REJ);
			return;
		}

		//check sequence number
		if ( ((frame[1] >> 4) & 0x07) != seq_number)
		{
			sendACK(0, REJ);
			return;
		}
				
		//write to EPPROM
		eepromWrite();
		//new sequence number
		seq_number =  (seq_number + 1) % (MAX_SEQUENCE + 1);
		//RR
		sendACK(0, RR);
	}
	else if (frame[1] >> 6 == 0x02) //S-Frame
	{
		//check crc
		myCrc = crcFast(frame, S_FRAME_SIZE - 3);
		receiveCrc = frame[S_FRAME_SIZE - 2] << 8 | frame[S_FRAME_SIZE - 3];
		if (myCrc != receiveCrc)
		{
			sendACK(0, REJ);
			return;
		}
		
		//if poll
		if ( ((frame[1] >> 3) & 0x01) == 1) //poll
		{
			//RR seq_number
			sendACK(0, RR);
		}
		else//if ACK (in transmit mode)
		{
			if (frame[1] >> 4 == RR)//receive ready
			{
				uint8_t seq_receive = frame[1] & 0x07;
				while ((sendBuffer[headTransmit % WINDOW_SIZE][1] & 0x07) != seq_receive) headTransmit++;
			}
			else
			{
				//send back from seq_receive
			}
		}
	}
	else //U-frame
	{
		//check crc
		myCrc = crcFast(frame, S_FRAME_SIZE - 3);
		receiveCrc = frame[S_FRAME_SIZE - 2] << 8 | frame[S_FRAME_SIZE - 3];
		if (myCrc != receiveCrc)
		{
			sendACK(0, REJ);
			return;
		}
		
		//if stop
		if (((frame[1] >> 5) & 0x01) == 1)
		{	
			//stop
		}
		else //if start (on ARM only!)
		{
			//check PC2ARM or ARM2PC, then initialize the buffer
		}
 	}
}

inline void transmitFunc()
{
	HAL_TIM_Base_Stop_IT(&timerHandle);
	timerOn = 0;
	//add data to buffer
	//send data

}

inline void eepromWrite()
{
}

inline void SPI_Init()
{
	__GPIOA_CLK_ENABLE();
	__GPIOC_CLK_ENABLE();
	
	GPIO_InitTypeDef gpio_init_s;
	
	gpio_init_s.Mode = GPIO_MODE_AF_PP;
	gpio_init_s.Pull = GPIO_PULLUP;
	gpio_init_s.Speed = GPIO_SPEED_HIGH;
	gpio_init_s.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
	HAL_GPIO_Init(GPIOA, &gpio_init_s);
	
	/* SS0 */
	gpio_init_s.Mode = GPIO_MODE_OUTPUT_PP;
	gpio_init_s.Pin = GPIO_PIN_4;
	HAL_GPIO_Init(GPIOA, &gpio_init_s);
	
	/* SS1 */
	gpio_init_s.Pin = GPIO_PIN_11;
	HAL_GPIO_Init(GPIOC, &gpio_init_s);
	
	//Configure SPI1
	__SPI1_CLK_ENABLE();
	
	spiHandle.Instance = SPI1;
	spiHandle.Init.Mode = SPI_MODE_MASTER;
	spiHandle.Init.Direction = SPI_DIRECTION_2LINES;
	spiHandle.Init.DataSize = SPI_DATASIZE_8BIT;
	spiHandle.Init.CLKPolarity = SPI_POLARITY_LOW;
	spiHandle.Init.CLKPhase = SPI_PHASE_1EDGE;
	spiHandle.Init.NSS = SPI_NSS_SOFT;
	spiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
	spiHandle.Init.FirstBit = SPI_FIRSTBIT_MSB;
	spiHandle.Init.TIMode = SPI_TIMODE_DISABLE;
	spiHandle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	spiHandle.Init.CRCPolynomial = 0;
	if (HAL_SPI_Init(&spiHandle) != HAL_OK) 
	{
		printf("Error when init SPI.\n");
		while(1);
	}
}

inline void TIMER_Init()
{
	__TIM3_CLK_ENABLE();
	
    /* Init NVIC (Nested vectored interrupt controller) */
    HAL_NVIC_SetPriority(TIM3_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
	
	 /* TIM3 clocks:
     * - PCLK1 = SYSCLK/2, APB1 prescaler = 2. Therefore, TIM3CLK = PCLK1 * 2 = SYSCLK
     * - We want CK_CNT (Counter clock) = 1 MHz. => Prescaler = SYSCLK/1000000 - 1
     */
    timerHandle.Instance = TIM3;
     
    timerHandle.Init.Period            = 1000000 / TIMEOUT_FREQ - 1;
    timerHandle.Init.Prescaler         = HAL_RCC_GetSysClockFreq()/1000000 - 1;
    timerHandle.Init.ClockDivision     = 0; /* Don't care */
    timerHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
    timerHandle.Init.RepetitionCounter = 0; /* Don't care */
    
    if (HAL_TIM_Base_Init(&timerHandle) != HAL_OK) {
        printf("Err: HAL_TIM_Base_Init failed.\n");
        while (1);
    }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 72000000
  *            HCLK(Hz)                       = 72000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            HSE Frequency(Hz)              = 8000000
  *            HSE PREDIV1                    = 1
  *            PLLMUL                         = 9
  *            Flash Latency(WS)              = 2
  * @param  None
  * @retval None
  */
inline void SystemClock_Config(void)
{
	RCC_ClkInitTypeDef clkinitstruct = {0};
	RCC_OscInitTypeDef oscinitstruct = {0};

	HAL_RCC_DeInit();

	oscinitstruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	oscinitstruct.HSEState = RCC_HSE_ON;
	oscinitstruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	oscinitstruct.PLL.PLLState = RCC_PLL_ON;
	oscinitstruct.PLL.PLLMUL = RCC_PLL_MUL9;
	oscinitstruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	if (HAL_RCC_OscConfig(&oscinitstruct) != HAL_OK)
	{
	  //error
	  while(1);
	}

	clkinitstruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_SYSCLK;
	clkinitstruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	clkinitstruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	clkinitstruct.APB1CLKDivider = RCC_SYSCLK_DIV2;
	clkinitstruct.APB2CLKDivider = RCC_SYSCLK_DIV1;
	if (HAL_RCC_ClockConfig(&clkinitstruct, FLASH_LATENCY_2) != HAL_OK)
	{
	  //error
	  while(1);
	}

	SystemCoreClockUpdate();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) 
{
	if (huart->Instance == USART1)
	{
		frameBuffer[tailReceive % WINDOW_SIZE][nByte] = receiveByte;
		nByte = (nByte + 1) % (DATA_SIZE + 5);
		if (receiveByte == STOP_BYTE || nByte == 0)
		{
			tailReceive++;
			nByte = 0;
		}
		if (tailReceive - headReceive < WINDOW_SIZE) HAL_UART_Receive_IT(&bboard_uart1_handle, &receiveByte, 1);
	}
	
}


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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
