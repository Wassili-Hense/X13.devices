#include "../../config.h"

#if ((defined UART_PHY) || (defined EXTSER_USED))

#include <avr/interrupt.h>

#define HAL_SIZEOF_UART_RX_FIFO     32      // Should be 2^n

typedef struct
{
    uint8_t             rx_fifo[HAL_SIZEOF_UART_RX_FIFO];
    volatile uint8_t    rx_head;
    uint8_t             rx_tail;
    
    uint8_t         *   pTxBuf;
    uint8_t             tx_len;
    uint8_t             tx_pos;
}HAL_UART_t;

static HAL_UART_t * hal_UARTv[4] = {NULL, NULL, NULL, NULL};

static volatile uint8_t *hal_pUART[] =
            {
            #ifdef UCSR0A
                &UCSR0A,
            #else
                NULL,
            #endif  //  UCSR0A
            #ifdef UCSR1A
                &UCSR1A, 
            #else
                NULL,
            #endif  //  UCSR1A
            #ifdef UCSR2A
                &UCSR2A, 
            #else
                NULL,
            #endif  //  UCSR2A
            #ifdef UCSR3A
                &UCSR3A
            #else
                NULL
            #endif  //  UCSR3A
            };

static const uint16_t hal_baud_list[] = {((F_CPU/16/2400) - 1),
                                         ((F_CPU/16/4800) - 1),
                                         ((F_CPU/16/9600) - 1),
                                         ((F_CPU/16/19200) - 1),
                                         ((F_CPU/16/38400) - 1)};

#if (defined UCSR0A) && (defined UART0_PORT)

#ifndef USART0_RX_vect
#define USART0_RX_vect              USART_RX_vect
#endif  //  USART_RX_vect

#ifndef USART0_UDRE_vect
#define USART0_UDRE_vect            USART_UDRE_vect
#endif  //  USART_UDRE_vect

ISR(USART0_RX_vect)
{
    uint8_t data = UDR0;
    uint8_t tmp_head = (hal_UARTv[0]->rx_head + 1) & (uint8_t)(HAL_SIZEOF_UART_RX_FIFO - 1);
    if(tmp_head == hal_UARTv[0]->rx_tail)        // Overflow
        return;

    hal_UARTv[0]->rx_fifo[hal_UARTv[0]->rx_head] = data;
    hal_UARTv[0]->rx_head = tmp_head;
}

ISR(USART0_UDRE_vect)
{
    if(hal_UARTv[0]->tx_len == hal_UARTv[0]->tx_pos)
    {
        hal_UARTv[0]->tx_len = 0;
        UCSR0B &= ~(1<<UDRIE0);
        return;
    }

    UDR0 = hal_UARTv[0]->pTxBuf[hal_UARTv[0]->tx_pos++];
}
#endif  //  UCSR0A

#if (defined UCSR1A) && (defined UART1_PORT)
ISR(USART1_RX_vect)
{
    uint8_t data = UDR1;
    uint8_t tmp_head = (hal_UARTv[1]->rx_head + 1) & (uint8_t)(HAL_SIZEOF_UART_RX_FIFO - 1);
    if(tmp_head == hal_UARTv[1]->rx_tail)        // Overflow
        return;

    hal_UARTv[1]->rx_fifo[hal_UARTv[1]->rx_head] = data;
    hal_UARTv[1]->rx_head = tmp_head;
}

ISR(USART1_UDRE_vect)
{
    if(hal_UARTv[1]->tx_len == hal_UARTv[1]->tx_pos)
    {
        hal_UARTv[1]->tx_len = 0;
        UCSR1B &= ~(1<<UDRIE1);
        return;
    }

    UDR1 = hal_UARTv[1]->pTxBuf[hal_UARTv[1]->tx_pos++];
}
#endif  //  UCSR1A

#if (defined UCSR2A) && (defined UART2_PORT)
ISR(USART2_RX_vect)
{
    uint8_t data = UDR2;
    uint8_t tmp_head = (hal_UARTv[2]->rx_head + 1) & (uint8_t)(HAL_SIZEOF_UART_RX_FIFO - 1);
    if(tmp_head == hal_UARTv[2]->rx_tail)        // Overflow
        return;

    hal_UARTv[2]->rx_fifo[hal_UARTv[2]->rx_head] = data;
    hal_UARTv[2]->rx_head = tmp_head;
}

ISR(USART2_UDRE_vect)
{
    if(hal_UARTv[2]->tx_len == hal_UARTv[2]->tx_pos)
    {
        hal_UARTv[2]->tx_len = 0;
        UCSR2B &= ~(1<<UDRIE2);
        return;
    }

    UDR2 = hal_UARTv[2]->pTxBuf[hal_UARTv[2]->tx_pos++];
}
#endif  //  UCSR2A

#if (defined UCSR3A) && (defined UART3_PORT)
ISR(USART3_RX_vect)
{
    uint8_t data = UDR3;
    uint8_t tmp_head = (hal_UARTv[3]->rx_head + 1) & (uint8_t)(HAL_SIZEOF_UART_RX_FIFO - 1);
    if(tmp_head == hal_UARTv[3]->rx_tail)        // Overflow
        return;

    hal_UARTv[3]->rx_fifo[hal_UARTv[3]->rx_head] = data;
    hal_UARTv[3]->rx_head = tmp_head;
}

ISR(USART3_UDRE_vect)
{
    if(hal_UARTv[3]->tx_len == hal_UARTv[3]->tx_pos)
    {
        hal_UARTv[3]->tx_len = 0;
        UCSR3B &= ~(1<<UDRIE3);
        return;
    }

    UDR3 = hal_UARTv[3]->pTxBuf[hal_UARTv[3]->tx_pos++];
}
#endif  //  UCSR3A

// HAL API
void hal_uart_deinit(uint8_t port)
{
    switch(port)
    {
#if (defined UCSR0A) && (defined UART0_PORT)
        case 0:
            {
            UCSR0B = 0;
            UART0_PORT &= ~((1<<UART0_RX_PIN) | (1<<UART0_TX_PIN));
            UART0_DDR &= ~(1<<UART0_TX_PIN);
            }
            break;
#endif  //  (defined UCSR0A) && (defined UART0_PORT)
#if (defined UCSR1A) && (defined UART1_PORT)
        case 1:
            {
            UCSR1B = 0;
            UART1_PORT &= ~((1<<UART1_RX_PIN) | (1<<UART1_TX_PIN));
            UART1_DDR &= ~(1<<UART1_TX_PIN);
            }
            break;
#endif  //  (defined UCSR1A) && (defined UART1_PORT)
#if (defined UCSR2A) && (defined UART2_PORT)
        case 2:
            {
            UCSR2B = 0;
            UART2_PORT &= ~((1<<UART2_RX_PIN) | (1<<UART2_TX_PIN));
            UART2_DDR &= ~(1<<UART2_TX_PIN);
            }
            break;
#endif  //  (defined UCSR2A) && (defined UART2_PORT)
#if (defined UCSR3A) && (defined UART3_PORT)
        case 3:
            {
            UCSR3B = 0;
            UART3_PORT &= ~((1<<UART3_RX_PIN) | (1<<UART3_TX_PIN));
            UART3_DDR &= ~(1<<UART3_TX_PIN);
            }
            break;
#endif  //  (defined UCSR3A) && (defined UART3_PORT)
        default:
            assert(0);
    }

    if(hal_UARTv[port] != NULL)
    {
        mqFree(hal_UARTv[port]);
        hal_UARTv[port] = NULL;
    }
}

void hal_uart_init_hw(uint8_t port, uint8_t nBaud)
{
    assert(nBaud <= 4);

    switch(port)
    {
#if (defined UCSR0A) && (defined UART0_PORT)
        case 0:
            {
            UCSR0B = 0;
            UART0_PORT |= (1<<UART0_RX_PIN) | (1<<UART0_TX_PIN);
            UART0_DDR |= (1<<UART0_TX_PIN);
            UART0_DDR &= ~(1<<UART0_RX_PIN);
            }
            break;
#endif  //  (defined UCSR0A) && (defined UART0_PORT)
#if (defined UCSR1A) && (defined UART1_PORT)
        case 1:
            {
            UCSR1B = 0;
            UART1_PORT |= (1<<UART1_RX_PIN) | (1<<UART1_TX_PIN);
            UART1_DDR |= (1<<UART1_TX_PIN);
            UART1_DDR &= ~(1<<UART1_RX_PIN);
            }
            break;
#endif  //  (defined UCSR1A) && (defined UART1_PORT)
#if (defined UCSR2A) && (defined UART2_PORT)
        case 2:
            {
            UCSR2B = 0;
            UART2_PORT |= (1<<UART2_RX_PIN) | (1<<UART2_TX_PIN);
            UART2_DDR |= (1<<UART2_TX_PIN);
            UART2_DDR &= ~(1<<UART2_RX_PIN);
            }
            break;
#endif  //  (defined UCSR2A) && (defined UART2_PORT)
#if (defined UCSR3A) && (defined UART3_PORT)
        case 3:
            {
            UCSR3B = 0;
            UART3_PORT |= (1<<UART3_RX_PIN) | (1<<UART3_TX_PIN);
            UART3_DDR |= (1<<UART3_TX_PIN);
            UART3_DDR &= ~(1<<UART3_RX_PIN);
            }
            break;
#endif  //  (defined UCSR3A) && (defined UART3_PORT)
        default:
            assert(0);
    }

    if(hal_UARTv[port] == NULL)
    {
        hal_UARTv[port] = mqAlloc(sizeof(HAL_UART_t));
        assert(hal_UARTv[port] != NULL);
    }

    hal_UARTv[port]->rx_head = 0;
    hal_UARTv[port]->rx_tail = 0;
    hal_UARTv[port]->tx_len = 0;
    hal_UARTv[port]->tx_pos = 0;

    uint16_t baud = hal_baud_list[nBaud];

    *(hal_pUART[port] + 5) = (baud >> 8);                                 // UBRRH
    *(hal_pUART[port] + 4) = (baud & 0xFF);                               // UBRRL
    *(hal_pUART[port] + 1) = ((1<<RXCIE0) | (1<<RXEN0) | (1<<TXEN0));     // UCSRB
    *(hal_pUART[port] + 2) = (3<<UCSZ00);                                 // UCSRC
}

// Tx free
bool hal_uart_free(uint8_t port)
{
    return (hal_UARTv[port]->tx_len == 0);
}

void hal_uart_send(uint8_t port, uint8_t len, uint8_t * pBuf)
{
    hal_UARTv[port]->tx_len = len;
    hal_UARTv[port]->tx_pos = 1;
    hal_UARTv[port]->pTxBuf = pBuf;

    *(hal_pUART[port] + 6) = *pBuf;
    *(hal_pUART[port] + 1) |= (1<<UDRIE0);
}

bool hal_uart_datardy(uint8_t port)
{
    return (hal_UARTv[port]->rx_head != hal_UARTv[port]->rx_tail);
}

uint8_t hal_uart_get(uint8_t port)
{
    if(hal_UARTv[port]->rx_head == hal_UARTv[port]->rx_tail)
        return 0;
        
    uint8_t data = hal_UARTv[port]->rx_fifo[hal_UARTv[port]->rx_tail];
    hal_UARTv[port]->rx_tail++;
    hal_UARTv[port]->rx_tail &= (uint8_t)(HAL_SIZEOF_UART_RX_FIFO - 1);

    return data;
}

#endif  //  ((defined UART_PHY) || (defined EXTSER_USED))
