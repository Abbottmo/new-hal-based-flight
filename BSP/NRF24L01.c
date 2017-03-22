#include "NRF24L01.h"
#include "spi.h"


uint8_t NRF24L01_RXDATA[RX_PLOAD_WIDTH];
uint8_t NRF24L01_TXDATA[RX_PLOAD_WIDTH];




u8  RX_ADDRESS[RX_ADR_WIDTH]= {0x34,0xc3,0x10,0x10,0x00};	


uint8_t NRF_Write_Reg(uint8_t reg, uint8_t value)
{
    uint8_t status;
    SPI_CSN_L();					  
    status = SPI_RW(reg);  
    SPI_RW(value);		
    SPI_CSN_H();					
    return 	status;
}

