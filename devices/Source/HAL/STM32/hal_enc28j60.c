#include "../../config.h"

#if (defined ENC28J60_PHY)

#if (ENC_USE_SPI == 1)

#define SPIe_PORT                   GPIOA
#define SPIe_SCK_PIN                GPIO_Pin_5
#define SPIe_MISO_PIN               GPIO_Pin_6
#define SPIe_MOSI_PIN               GPIO_Pin_7

#elif (ENC_USE_SPI == 2)

#define SPIe_PORT                   GPIOB
#define SPIe_SCK_PIN                GPIO_Pin_13
#define SPIe_MISO_PIN               GPIO_Pin_14
#define SPIe_MOSI_PIN               GPIO_Pin_15

#else
#error unknown enc configuration
#endif

void hal_enc28j60_init_hw(void)
{
    // Enable Periphery Clocks
#if (ENC_USE_SPI == 1)
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
#else
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
#endif  //  ENC_USE_SPI

    // ENC_NSS_PIN
    hal_dio_gpio_cfg(ENC_NSS_PORT, ENC_NSS_PIN, DIO_MODE_OUT_HS);
    ENC_NSS_PORT->BSRR = ENC_NSS_PIN;

    // Configure SPI pins
#if (defined __STM32F0XX_H)
    hal_dio_gpio_cfg(SPIe_PORT, (SPIe_SCK_PIN | SPIe_MISO_PIN | SPIe_MOSI_PIN), DIO_MODE_AF0);
#elif (defined __STM32F10x_H)
    #error hal_enc28j60_init_hw, SPI Pin configuration
#endif

#if (defined __STM32F0XX_H)
#if (ENC_USE_SPI == 1)
    SPIe->CR1 = (uint16_t)(SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR);                   // Prescaler 2
#else   //  SPI2
    SPIe->CR1 = (uint16_t)(SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_BR_0 | SPI_CR1_MSTR);    // Prescaler 4
#endif  //  ENC_USE_SPI
    SPIe->CR2 = (uint16_t)(SPI_CR2_FRXTH | SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0); // 8 bit
#elif (defined __STM32F10x_H)
    SPIe->CR1 = (uint16_t)(SPI_CR1_MSTR | SPI_CR1_BR_1 | SPI_CR1_SSI | SPI_CR1_SSM);    // Prescaler 8
#endif

    SPIe->CRCPR =  7;
    SPIe->I2SCFGR &= (uint16_t)~((uint16_t)SPI_I2SCFGR_I2SMOD);
    SPIe->CR1 |= SPI_CR1_SPE;       // SPIe enable
}

uint8_t hal_enc28j60exchg(uint8_t data)
{
#if   (defined __STM32F0XX_H)
    uint32_t spixbase = (uint32_t)SPIe + 0x0C;
    *(__IO uint8_t *)spixbase = data;
    while((SPIe->SR & SPI_SR_RXNE) == 0);
    return *(__IO uint8_t *)spixbase;
#elif (defined __STM32F10x_H)
    SPIe->DR = data;
    while((SPIe->SR & SPI_SR_RXNE) == 0);
    return (SPIe->DR & 0xFF);
#endif
}

#endif  //  ENC28J60_PHY