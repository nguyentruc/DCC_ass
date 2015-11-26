/* Includes ------------------------------------------------------------------*/
#include "spi_eeprom.h"

uint8_t eepromReadStatus(SPI_HandleTypeDef *spiHandle) 
{
	uint8_t cmd = cmdRDSR;
	uint8_t res;
	HAL_GPIO_WritePin(GPIOA , GPIO_PIN_4 , GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC , GPIO_PIN_11 , GPIO_PIN_SET);
	
	HAL_SPI_Transmit(spiHandle, &cmd, 1, 0xFFFFFFFF);
	HAL_SPI_Receive(spiHandle, &res, 1, 0xFFFFFFFF);
	
	HAL_GPIO_WritePin(GPIOA , GPIO_PIN_4 , GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC , GPIO_PIN_11 , GPIO_PIN_RESET);
	
	return res;
}

void eepromWriteEnable(SPI_HandleTypeDef *spiHandle) 
{
	uint8_t cmd = cmdWREN;
	
	while(eepromReadStatus(spiHandle) & 0x01);
	
	HAL_GPIO_WritePin(GPIOA , GPIO_PIN_4 , GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC , GPIO_PIN_11 , GPIO_PIN_SET);
	
	HAL_SPI_Transmit(spiHandle, &cmd, 1, 0xFFFFFFFF);
	
	HAL_GPIO_WritePin(GPIOA , GPIO_PIN_4 , GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC , GPIO_PIN_11 , GPIO_PIN_RESET);
}

int eepromWrite(SPI_HandleTypeDef *spiHandle, uint8_t *buf, uint16_t cnt, uint16_t address)
{
	uint8_t cmd = cmdWRITE;
	
	// Check if end of memory is not reached
	const uint16_t lastMemoryAdress = MEM_SIZE - 1;
	uint16_t lastAskedAdress = address + cnt - 1;
	
	const uint16_t pageMask = PAGE_SIZE - 1;

	uint16_t currentOffset = address;
	
	uint16_t bufferOffset = 0;
	
	uint16_t memoryWrap = 0;
	
	// If last asked adress is higher than last memory adress, no data write
	if (lastAskedAdress > lastMemoryAdress)
	{
		memoryWrap = lastAskedAdress - lastMemoryAdress;
	}
	else
	{
		uint8_t anotherPageNeeded;
		
		do
		{
			// Number of left bytes to write
			uint16_t nbLeftBytesToWrite = lastAskedAdress - currentOffset + 1;
			
			// Number of bytes available in current page
			uint16_t currentOffsetAndPageMask = currentOffset & pageMask;
			uint16_t nbBytesAvailableInCurrentPage = pageMask - currentOffsetAndPageMask + 1;
			
			// Number of bytes to write in this page
			uint16_t nbBytesToWriteInThisPage;
			
			// Compute if all datas to write fit in current page	
			if (nbLeftBytesToWrite > nbBytesAvailableInCurrentPage)
			{
				anotherPageNeeded = 1;
				nbBytesToWriteInThisPage = nbBytesAvailableInCurrentPage;
			}
			else
			{
				anotherPageNeeded = 0;
				nbBytesToWriteInThisPage = nbLeftBytesToWrite;
			}
			
			// Enable write
			eepromWriteEnable(spiHandle);
				
			// Reset CS
			HAL_GPIO_WritePin(GPIOA , GPIO_PIN_4 , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC , GPIO_PIN_11 , GPIO_PIN_SET);
		
			// Send WRITE command
			HAL_SPI_Transmit(spiHandle, &cmd, 1, 0xFFFFFFFF);
			
			// Send address
			uint8_t addr[2];
			addr[0] = currentOffset >> 8;
			addr[1] = currentOffset & 0x00FF;
			HAL_SPI_Transmit(spiHandle, addr, 2, 0xFFFFFFFF);
	
			// Send data
			HAL_SPI_Transmit(spiHandle, buf + bufferOffset, nbBytesToWriteInThisPage, 0xFFFFFFFF);
		
			// Set CS
			HAL_GPIO_WritePin(GPIOA , GPIO_PIN_4 , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC , GPIO_PIN_11 , GPIO_PIN_RESET);
			
			bufferOffset += nbBytesToWriteInThisPage;
			currentOffset += nbBytesToWriteInThisPage;
		} while(anotherPageNeeded);

	}

	return memoryWrap;
}

int eepromRead(SPI_HandleTypeDef *spiHandle, uint8_t *buf, uint16_t cnt, uint16_t address)
{
	// Check if end of memory is not reached
	uint16_t lastMemoryAdress = MEM_SIZE - 1;
	uint16_t lastAskedAdress = address + cnt - 1;
	
	// If last asked adress is higher than last memory adress, no data read
	uint16_t memoryWrap = 0;
	
	if (lastAskedAdress > lastMemoryAdress)
	{
		memoryWrap = lastAskedAdress - lastMemoryAdress;
	}
	else
	{
		uint8_t cmd = cmdREAD;
		
		while(eepromReadStatus(spiHandle) & 0x01);

		HAL_GPIO_WritePin(GPIOA , GPIO_PIN_4 , GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC , GPIO_PIN_11 , GPIO_PIN_SET);
	
		// Send READ command
		HAL_SPI_Transmit(spiHandle, &cmd, 1, 0xFFFFFFFF);
		
		// Send address
		uint8_t addr[2];
		addr[0] = address >> 8;
		addr[1] = address & 0x00FF;
		HAL_SPI_Transmit(spiHandle, addr, 2, 0xFFFFFFFF);
		
		// Read data
		HAL_SPI_Receive(spiHandle, buf, cnt, 0xFFFFFFFF);

		HAL_GPIO_WritePin(GPIOA , GPIO_PIN_4 , GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC , GPIO_PIN_11 , GPIO_PIN_RESET);

	}
	
	return memoryWrap;
}
