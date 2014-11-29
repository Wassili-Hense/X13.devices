#include "../../config.h"

#if (defined ENC28J60_PHY)

#if (ENC_USE_SPI == 1)

#define SPIe_PORT                   GPIOA
#define SPIe_SCK_PIN                GPIO_Pin_5
#define SPIe_SCK_PINSRC             GPIO_PinSource5
#define SPIe_MISO_PIN               GPIO_Pin_6
#define SPIe_MISO_PINSRC            GPIO_PinSource6
#define SPIe_MOSI_PIN               GPIO_Pin_7
#define SPIe_MOSI_PINSRC            GPIO_PinSource7

#elif (ENC_USE_SPI == 2)

#define SPIe_PORT                   GPIOB
#define SPIe_SCK_PIN                GPIO_Pin_13
#define SPIe_SCK_PINSRC             GPIO_PinSource13
#define SPIe_MISO_PIN               GPIO_Pin_14
#define SPIe_MISO_PINSRC            GPIO_PinSource14
#define SPIe_MOSI_PIN               GPIO_Pin_15
#define SPIe_MOSI_PINSRC            GPIO_PinSource15

#else
#error unknown enc configuration
#endif

void hal_enc28j60_init_hw(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    // Enable Periphery Clocks
#if (ENC_USE_SPI == 1)
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
#else
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
#endif  //  ENC_USE_SPI

    // ENC_NSS_PIN
    GPIO_InitStructure.GPIO_Pin     = ENC_NSS_PIN;
#if (defined __STM32F0XX_H)
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType   = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd    = GPIO_PuPd_NOPULL;
#elif (defined __STM32F10x_H)
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_Out_PP;
#endif
    GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_50MHz;
    GPIO_Init(ENC_NSS_PORT, &GPIO_InitStructure);

    ENC_NSS_PORT->BSRR = ENC_NSS_PIN;

    // Configure SPI pins
    GPIO_InitStructure.GPIO_Pin     = SPIe_SCK_PIN | SPIe_MISO_PIN | SPIe_MOSI_PIN;
#if (defined __STM32F0XX_H)
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AF;
#elif (defined __STM32F10x_H)
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AF_PP;
#endif
    GPIO_Init(SPIe_PORT, &GPIO_InitStructure);
#if (defined __STM32F0XX_H)
    GPIO_PinAFConfig(SPIe_PORT, SPIe_SCK_PINSRC,  GPIO_AF_0);
    GPIO_PinAFConfig(SPIe_PORT, SPIe_MISO_PINSRC, GPIO_AF_0);
    GPIO_PinAFConfig(SPIe_PORT, SPIe_MOSI_PINSRC, GPIO_AF_0);
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