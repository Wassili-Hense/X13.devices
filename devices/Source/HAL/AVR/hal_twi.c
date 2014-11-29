#include "../../config.h"

#ifdef EXTTWI_USED

#include "../../EXT/exttwi.h"

#include <avr/interrupt.h>
#include <util/twi.h>

// Global variable defined in exttwi.c
extern volatile TWI_QUEUE_t * pTwi_exchange;

// HAL
bool hal_twi_configure(uint8_t enable)
{
    if(enable)
    {
        // Disable TWI
        TWCR = (1<<TWINT);
        // Set Speed
        TWBR = (((F_CPU/100000UL)-16)/2);   // 100kHz
        // Clear data register
        TWDR = 0xFF;
        // Enable TWI & Interrupt
        TWCR = (1<<TWEN) | (1<<TWIE);
    }
    else
    {
        // Disable TWI
        TWCR = (1<<TWINT);
    }

    if(TWIM_SCL_STAT() == 0)
        return false;

    return true;
}

void hal_twi_tick(void)
{
    assert(pTwi_exchange != NULL);

    static uint8_t twi_wd = 0;

    uint8_t access = pTwi_exchange->frame.access;
    
    if((access & 0xF8) == 0)
    {
        twi_wd = 0;
    }
    else
    {
        twi_wd--;
        if(twi_wd == 0) // Timeout
        {
            pTwi_exchange->frame.access |= TWI_WD;
            if(access & TWI_BUSY)
            {
                TWCR &= ~((1<<TWEN) | (1<<TWSTO));
                TWCR |= (1<<TWINT) | (1<<TWEN);
            }
        }
    }

    if(access & TWI_BUSY)
        return;

    if(TWIM_SCL_STAT() != 0)    // Bus Free
    {
        twi_wd = 0;

        pTwi_exchange->frame.access |= TWI_BUSY;

        TWCR = (1<<TWEN) |                      // TWI Interface enabled.
               (1<<TWIE) | (1<<TWINT) |         // Enable TWI Interrupt and clear the flag.
               (1<<TWSTA);                      // Initiate a START condition.
    }
}

ISR(TWI_vect)
{
    static uint8_t twi_ptr;

    switch(TW_STATUS)
    {
        // Master
        case TW_START:                              // start condition transmitted
        case TW_REP_START:                          // repeated start condition transmitted
            twi_ptr = 0;
            
            if(pTwi_exchange->frame.access & TWI_WRITE)
                TWDR = (pTwi_exchange->frame.address<<1);
            else
                TWDR = (pTwi_exchange->frame.address<<1) | TW_READ;
            TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE);
            break;
        // Master Send
        case TW_MT_SLA_NACK:                        // SLA+W transmitted, NACK received
        case TW_MR_SLA_NACK:                        // SLA+R transmitted, NACK received
            pTwi_exchange->frame.read = 0;
            pTwi_exchange->frame.access |= TWI_SLANACK;
            TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN) | (1<<TWIE); // Send Stop
            break;
        case TW_MT_SLA_ACK:                         // SLA+W transmitted, ACK received
        case TW_MT_DATA_ACK:                        // data transmitted, ACK received
            if(twi_ptr < pTwi_exchange->frame.write)
            {
                TWDR = pTwi_exchange->frame.data[twi_ptr];
                twi_ptr++;
                TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE);
                break;
            }
            // else, ACK received but should be NACK
        case TW_MT_DATA_NACK:                       // data transmitted, NACK received
            pTwi_exchange->frame.write = twi_ptr;
            pTwi_exchange->frame.access &= ~TWI_WRITE;
            if((pTwi_exchange->frame.access & TWI_READ) == 0)
            {
                pTwi_exchange->frame.read = 0;
                pTwi_exchange->frame.access = TWI_RDY;
                TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN) | (1<<TWIE); // Send Stop
            }
            else
                TWCR = (1<<TWEN) | (1<<TWIE) | (1<<TWINT) | (1<<TWSTA); // Send RepSTART
            break;
        // Master Receive
        case TW_MR_DATA_ACK:                        // Data byte has been received and ACK transmitted
            pTwi_exchange->frame.data[twi_ptr++] = TWDR;
        case TW_MR_SLA_ACK:                         // SLA+R has been transmitted and ACK received
            if((twi_ptr + 1) < pTwi_exchange->frame.read)
            {
                TWCR = (1<<TWEN) |                  // TWI Interface enabled
                       (1<<TWIE) | (1<<TWINT) |     // Enable TWI Interrupt and clear the flag to read next byte
                       (1<<TWEA);                   // Send ACK after reception
            }
            else                                    // Send NACK after next reception
            {
                TWCR = (1<<TWEN) |                  // TWI Interface enabled
                       (1<<TWIE) | (1<<TWINT);      // Enable TWI Interrupt and clear the flag to read last byte
            }
            break;
        case TW_MR_DATA_NACK:                       // Data byte has been received and NACK transmitted
            pTwi_exchange->frame.data[twi_ptr++] = TWDR;
            pTwi_exchange->frame.read = twi_ptr;
            pTwi_exchange->frame.access = TWI_RDY;
            TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN) | (1<<TWIE); // Send Stop
            break;
        default:                                    // Error
            TWCR &= ~((1<<TWEN) | (1<<TWSTO));
            TWCR |= (1<<TWINT) | (1<<TWEN);
            pTwi_exchange->frame.write = twi_ptr;
            pTwi_exchange->frame.read = 1;
            pTwi_exchange->frame.data[0] = TWSR;
            pTwi_exchange->frame.access |= TWI_ERROR;
            break;
    }
}
#endif  //  EXTTWI_USED
