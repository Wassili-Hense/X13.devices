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
