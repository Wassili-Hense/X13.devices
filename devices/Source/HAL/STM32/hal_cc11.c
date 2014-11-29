#include "../../config.h"

#if (defined CC11_PHY)

void hal_cc11_init_hw(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    // Enable Periphery Clocks
#if   (CC11_USE_SPI == 1)
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
#else
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
#endif  //  CC11_USE_SPI

    // ENC_NSS_PIN
    GPIO_InitStructure.GPIO_Pin     = CC11_NSS_PIN;
#if (defined __STM32F0XX_H)
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType   = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd    = GPIO_PuPd_NOPULL;
#elif (defined __STM32F10x_H)
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_Out_PP;
#endif
    GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_50MHz;
    GPIO_Init(CC11_NSS_PORT, &GPIO_InitStructure);

    CC11_NSS_PORT->BSRR = CC11_NSS_PIN;

    // Configure SPI pins
    GPIO_InitStructure.GPIO_Pin     = SPIc_SCK_PIN | SPIc_MISO_PIN | SPIc_MOSI_PIN;
#if (defined __STM32F0XX_H)
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AF;
#elif (defined __STM32F10x_H)
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AF_PP;
#endif
    GPIO_Init(SPIc_PORT, &GPIO_InitStructure);
#if (defined __STM32F0XX_H)
    GPIO_PinAFConfig(SPIc_PORT, SPIc_SCK_PINSRC,  GPIO_AF_0);
    GPIO_PinAFConfig(SPIc_PORT, SPIc_MISO_PINSRC, GPIO_AF_0);
    GPIO_PinAFConfig(SPIc_PORT, SPIc_MOSI_PINSRC, GPIO_AF_0);
#endif

#if (defined __STM32F0XX_H)
#if (CC11_USE_SPI == 1)
    SPIc->CR1 = (uint16_t)(SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_BR_0 | SPI_CR1_MSTR);    // Prescaler 4, 24/4
#else   //  SPI2
    SPIc->CR1 = (uint16_t)(SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_BR_1 | SPI_CR1_MSTR);    // Prescaler 8  48/4
#endif  //  CC11_USE_SPI
    SPIc->CR2 = (uint16_t)(SPI_CR2_FRXTH | SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0); // 8 bit
#elif (defined __STM32F10x_H)
    SPIc->CR1 = (uint16_t)(SPI_CR1_MSTR | SPI_CR1_BR_1 | SPI_CR1_BR_0 | SPI_CR1_SSI | SPI_CR1_SSM);    // Prescaler 16  72/16
#endif

    SPIc->CRCPR =  7;
    SPIc->I2SCFGR &= (uint16_t)~((uint16_t)SPI_I2SCFGR_I2SMOD);
    SPIc->CR1 |= SPI_CR1_SPE;       // SPIc enable
}

uint8_t hal_cc11_spiExch(uint8_t data)
{
#if   (defined __STM32F0XX_H)
    uint32_t spixbase = (uint32_t)SPIc + 0x0C;
    *(__IO uint8_t *)spixbase = data;
    while((SPIc->SR & SPI_SR_RXNE) == 0);
    return *(__IO uint8_t *)spixbase;
#elif (defined __STM32F10x_H)
    SPIc->DR = data;
    while((SPIc->SR & SPI_SR_RXNE) == 0);
    return (SPIc->DR & 0xFF);
#endif
}

#endif  //  CC11_PHY
