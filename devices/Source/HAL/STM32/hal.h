#ifndef __HAL_H
#define __HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#if defined(NDEBUG)
    #define assert(e)   ((void)0)
#else // DEBUG
#if defined(__ASSERT_USE_STDERR)
    #define assert(e)   ((e) ? (void)0 : __assert(__func__, __FILE__, __LINE__, #e))
#else // !__ASSERT_USE_STDERR
    #define assert(e)   { if (!(e)) { while (1); } }
#endif  // __ASSERT_USE_STDERR
#endif  // NDEBUG

#if (defined STM32F0XX_LD) || (defined STM32F0XX_MD) || \
    (defined STM32F030X8)  || (defined STM32F030X6)
#include "stm32f0xx.h"
#elif defined (STM32F10X_LD) || defined (STM32F10X_LD_VL) || \
      defined (STM32F10X_MD) || defined (STM32F10X_MD_VL) || \
      defined (STM32F10X_HD) || defined (STM32F10X_HD_VL) || \
      defined (STM32F10X_XL) || defined (STM32F10X_CL) 
#include "stm32f10x.h"
#endif  //  uC Familie

void INIT_SYSTEM(void);

void halEnterCritical(void);
void halLeaveCritical(void);
#define ENTER_CRITICAL_SECTION      halEnterCritical
#define LEAVE_CRITICAL_SECTION      halLeaveCritical

// Hardware specific options
#define DIO_PORT_SIZE               16
#define portBYTE_ALIGNMENT          8
#define configTOTAL_HEAP_SIZE       2048

// GPIO compatibility
#define GPIO_Pin_0                 ((uint16_t)0x0001)  /*!< Pin 0 selected */
#define GPIO_Pin_1                 ((uint16_t)0x0002)  /*!< Pin 1 selected */
#define GPIO_Pin_2                 ((uint16_t)0x0004)  /*!< Pin 2 selected */
#define GPIO_Pin_3                 ((uint16_t)0x0008)  /*!< Pin 3 selected */
#define GPIO_Pin_4                 ((uint16_t)0x0010)  /*!< Pin 4 selected */
#define GPIO_Pin_5                 ((uint16_t)0x0020)  /*!< Pin 5 selected */
#define GPIO_Pin_6                 ((uint16_t)0x0040)  /*!< Pin 6 selected */
#define GPIO_Pin_7                 ((uint16_t)0x0080)  /*!< Pin 7 selected */
#define GPIO_Pin_8                 ((uint16_t)0x0100)  /*!< Pin 8 selected */
#define GPIO_Pin_9                 ((uint16_t)0x0200)  /*!< Pin 9 selected */
#define GPIO_Pin_10                ((uint16_t)0x0400)  /*!< Pin 10 selected */
#define GPIO_Pin_11                ((uint16_t)0x0800)  /*!< Pin 11 selected */
#define GPIO_Pin_12                ((uint16_t)0x1000)  /*!< Pin 12 selected */
#define GPIO_Pin_13                ((uint16_t)0x2000)  /*!< Pin 13 selected */
#define GPIO_Pin_14                ((uint16_t)0x4000)  /*!< Pin 14 selected */
#define GPIO_Pin_15                ((uint16_t)0x8000)  /*!< Pin 15 selected */

// Configure GPIO
// External procedure defined in hal_dio.c
void hal_dio_gpio_cfg(GPIO_TypeDef * GPIOx, uint16_t Mask, uint8_t Mode);

void StartSheduler(void);

void eeprom_init_hw(void);
void eeprom_read(uint8_t *pBuf, uint32_t Addr, uint32_t Len);
void eeprom_write(uint8_t *pBuf, uint32_t Addr, uint32_t Len);

void _delay_ms(uint32_t ms);
void _delay_us(uint32_t us);

uint16_t halRNG();

#ifdef __cplusplus
}
#endif

#endif  //  __HAL_H
