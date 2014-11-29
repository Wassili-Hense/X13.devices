#ifndef __HAL_H
#define __HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <assert.h>
#include <avr/io.h>
#include <avr/eeprom.h>

#define INIT_SYSTEM()

#define ENTER_CRITICAL_SECTION()    asm volatile ( "in      __tmp_reg__, __SREG__" :: );    \
                                    asm volatile ( "cli" :: );                              \
                                    asm volatile ( "push    __tmp_reg__" :: )

#define LEAVE_CRITICAL_SECTION()    asm volatile ( "pop     __tmp_reg__" :: );              \
                                    asm volatile ( "out     __SREG__, __tmp_reg__" :: )

#define eeprom_init_hw()
#define eeprom_read(pBuf, Addr, Len)  eeprom_read_block((void *)pBuf, (const void *)Addr, (size_t)Len)
#define eeprom_write(pBuf, Addr, Len) eeprom_write_block((const void *)pBuf, (void *)Addr, (size_t)Len)

// AVR Architecture specifics.
#define DIO_PORT_SIZE               8
#define portBYTE_ALIGNMENT          1
#define portPOINTER_SIZE_TYPE       uintptr_t
#define configTOTAL_HEAP_SIZE       1024

void StartSheduler(void);

uint16_t halRNG();

#ifdef __cplusplus
}
#endif


#endif  //  __HAL_H