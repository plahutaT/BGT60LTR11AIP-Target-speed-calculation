/*
 * bgt60ltr11_spi.c
 *
 *  Created on: Nov 20, 2024
 *      Author: tilen
 */
#include "bgt60ltr11_spi.h"

uint8_t bgt60ltr11_spi_read(uint8_t reg_addr, uint16_t *data)
{

    uint8_t tx_data[3];
    uint8_t rx_data[3] = {0, 0, 0};

    /* 	We send the register address from where we want to read
     * 	and than we read 2 bytes
     * 	so we need to send 2 dummy bytes
     * */
    tx_data[0] = (uint8_t)((reg_addr << 1) & 0xFE);
    tx_data[1] = 0;
    tx_data[2] = 0;

    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
    if(HAL_SPI_TransmitReceive(&hspi2, tx_data, rx_data, sizeof(tx_data)/sizeof(uint8_t), 1000) != HAL_OK)
    {
    	return HAL_ERROR;
    }
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);

    *data = ((uint16_t)(rx_data[1] << 8) | (uint16_t)(rx_data[2]));

    return HAL_OK;

}

uint8_t bgt60ltr11_spi_write(uint8_t reg_addr, uint16_t data)
{

    uint8_t tx_data[3];
    uint16_t wrdata = data;

    tx_data[0] = (uint8_t)((reg_addr << 1) | 0x01);
    tx_data[1] = (uint8_t)((wrdata >> 8) & 0xFF);
    tx_data[2] = (uint8_t)(wrdata & 0xFF);

    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
    if(HAL_SPI_Transmit(&hspi2, tx_data, sizeof(tx_data)/sizeof(uint8_t), 1000) != HAL_OK)
	{
		return HAL_ERROR;
	}
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);

    return HAL_OK;
}

/* ADC status */
uint8_t bgt60ltr11_HW_reset(void)
{

	 HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
	 HAL_Delay(10);
	 HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);

	return HAL_OK;
}




/* ADC status */
uint8_t bgt60ltr11_adc_status(void)
{

	uint16_t adc_status;

	if(bgt60ltr11_spi_read(0x24, &adc_status) != HAL_OK)
	{
		return HAL_ERROR;
	}

	if(adc_status == 0)
	{
		return HAL_ERROR;
	}

	return HAL_OK;
}


uint8_t bgt60ltr11_soft_reset(uint8_t wait)
{
	// 15 bit is soft_reset, active high
	bgt60ltr11_spi_write(0x0F, (1 << 15));
	uint16_t reg56 = 0;
	uint16_t reg0 = 0;

	if (wait)
	{
		// wait till init_done in REG56 is set
		for (volatile uint16_t i = 0; i < 2048; i++)
		{
			 bgt60ltr11_spi_read(0x38, &reg56);
			 bgt60ltr11_spi_read(0x00, &reg0);

			// check if REG0 has default values and REG56 bit init_done is set
			if (reg0 == 0 && reg56 & (1 << 13))
			{
				return HAL_OK;
			}
		}

		return HAL_ERROR;
	}

	return HAL_OK;
}

uint8_t bgt60ltr11_test(void)
{
	// convert ALL ADC channels

	uint16_t data = 0;
	if (bgt60ltr11_spi_read(0x02, &data) != HAL_OK) return HAL_ERROR;
	HAL_Delay(1);
	if (data != 0x2A00)
	{
		HAL_GPIO_WritePin(PE3_GPIO_Port, PE3_Pin, GPIO_PIN_SET);
	}


	return HAL_OK;
}


uint8_t bgt60ltr11_ADC_Convert(void)
{
	// convert ALL ADC channels

	if (bgt60ltr11_spi_write(0x23, 0010) != HAL_OK) return HAL_ERROR;
	HAL_Delay(1);

	return HAL_OK;
}


uint8_t bgt60ltr11_get_RAW_data(uint16_t *ifi, uint16_t *ifq)
{
    // Read ADC channel data directly into the provided pointers
    if (bgt60ltr11_spi_read(0x28, ifi) != HAL_OK) return HAL_ERROR;
    if (bgt60ltr11_spi_read(0x29, ifq) != HAL_OK) return HAL_ERROR;

    return HAL_OK;

	return HAL_OK;
}





uint8_t bgt60ltr11_pulsed_mode_init(void)
{

    // Perform soft reset
    if (bgt60ltr11_soft_reset(0) != HAL_OK) return HAL_ERROR;
    HAL_Delay(1);

    // Write to each register and check the result
    if (bgt60ltr11_spi_write(0x00, 0x0000) != HAL_OK) return HAL_ERROR;
    HAL_Delay(1);

    if (bgt60ltr11_spi_write(0x01, 0x0000) != HAL_OK) return HAL_ERROR;
    HAL_Delay(1);

    if (bgt60ltr11_spi_write(0x02, 0x2A00) != HAL_OK) return HAL_ERROR;
    HAL_Delay(1);

    // TODO need to check the value for the REG3

    if (bgt60ltr11_spi_write(0x04, 0x0F3A) != HAL_OK) return HAL_ERROR;
    HAL_Delay(1);

    if (bgt60ltr11_spi_write(0x05, 0x0FB0) != HAL_OK) return HAL_ERROR;
    HAL_Delay(1);

    if (bgt60ltr11_spi_write(0x06, 0x6800) != HAL_OK) return HAL_ERROR;
    HAL_Delay(1);

    if (bgt60ltr11_spi_write(0x07, 0x0557) != HAL_OK) return HAL_ERROR;
    HAL_Delay(1);

    if (bgt60ltr11_spi_write(0x08, 0x000E) != HAL_OK) return HAL_ERROR;
    HAL_Delay(1);

    if (bgt60ltr11_spi_write(0x09, 0x00E8) != HAL_OK) return HAL_ERROR;
    HAL_Delay(1);

    if (bgt60ltr11_spi_write(0x0A, 0x004F) != HAL_OK) return HAL_ERROR;
    HAL_Delay(1);

    if (bgt60ltr11_spi_write(0x0C, 0x0000) != HAL_OK) return HAL_ERROR;
    HAL_Delay(1);

    if (bgt60ltr11_spi_write(0x0D, 0x0000) != HAL_OK) return HAL_ERROR;
    HAL_Delay(1);

    if (bgt60ltr11_spi_write(0x0E, 0x0000) != HAL_OK) return HAL_ERROR;
    HAL_Delay(1);

    if (bgt60ltr11_spi_write(0x0F, 0x0000) != HAL_OK) return HAL_ERROR;
    HAL_Delay(1);

    if (bgt60ltr11_spi_write(0x22, 0x0000) != HAL_OK) return HAL_ERROR;
    HAL_Delay(1);

    if (bgt60ltr11_spi_write(0x23, 0x0000) != HAL_OK) return HAL_ERROR;
    HAL_Delay(1);

    if (bgt60ltr11_spi_write(0x24, 0x0000) != HAL_OK) return HAL_ERROR;
    HAL_Delay(1);
    /*
    // ADC clock EN, bandgap EN, ADC EN
    if (bgt60ltr11_spi_write(0x22, 0x0007) != HAL_OK) return HAL_ERROR;
    HAL_Delay(1);
    */
    if (bgt60ltr11_spi_write(0x0F, 0x4000) != HAL_OK) return HAL_ERROR;
    HAL_Delay(1);



    return HAL_OK;
}




