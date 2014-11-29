#include "../../config.h"

#if ((defined UART_PHY) || (defined EXTSER_USED))

#define HAL_SIZEOF_UART_RX_FIFO         32      // Should be 2^n

#if (defined __STM32F0XX_H)
    #define HAL_USART_RX_DATA           RDR
    #define HAL_USART_TX_DATA           TDR
#elif (defined __STM32F10x_H)
    #define HAL_USART_RX_DATA           DR
    #define HAL_USART_TX_DATA           DR
#else
    #error unknown uC familie
#endif  //  STM32

typedef struct
{
    uint8_t             rx_fifo[HAL_SIZEOF_UART_RX_FIFO];
    volatile uint8_t    rx_head;
    uint8_t             rx_tail;

    uint8_t         *   pTxBuf;
    uint8_t             tx_len;
    uint8_t             tx_pos;
}HAL_UART_t;

static HAL_UART_t * hal_UARTv[] = {NULL, NULL};

static USART_TypeDef * hal_pUART[] =
            {
            #ifdef USART1
                USART1,
            #else
                NULL,
            #endif  //  USART1
            #ifdef USART2
                USART2
            #else
                NULL
            #endif  //  USART2
            };

static const uint16_t hal_baud_list[] = {2400, 4800, 9600, 19200, 38400};

// IRQ handlers
static inline void hal_uart_irq_handler(uint8_t port)
{
    assert(hal_UARTv[port] != NULL);
    
    uint8_t data;
    uint32_t itstat;
#if (defined __STM32F0XX_H)                      // STM32F0
    itstat = hal_pUART[port]->ISR;
    if(itstat & USART_ISR_ORE)
    {
        hal_pUART[port]->ICR = USART_ICR_ORECF;
        return;
    }
#elif (defined __STM32F10x_H)                   // STM32F1xx    
    itstat = hal_pUART[port]->SR;
    if(itstat & USART_SR_ORE)
    {
        data = hal_pUART[port]->HAL_USART_RX_DATA;
        return;
    }
#endif  //  STM32
    
    // Received data is ready to be read
    if(itstat & USART_CR1_RXNEIE)
    {
        data = hal_pUART[port]->HAL_USART_RX_DATA;
        uint8_t tmp_head = (hal_UARTv[port]->rx_head + 1) & (uint8_t)(HAL_SIZEOF_UART_RX_FIFO - 1);
        if(tmp_head == hal_UARTv[port]->rx_tail)        // Overflow
            return;
            
        hal_UARTv[port]->rx_fifo[hal_UARTv[port]->rx_head] = data;
        hal_UARTv[port]->rx_head = tmp_head;
    }

    // Transmit data register empty
    if((itstat & USART_CR1_TXEIE) && (hal_pUART[port]->CR1 & USART_CR1_TXEIE))
    {
        if(hal_UARTv[port]->tx_pos == hal_UARTv[port]->tx_len)
        {
            hal_UARTv[port]->tx_len = 0;
            hal_pUART[port]->CR1 &= ~(uint32_t)USART_CR1_TXEIE;
            return;
        }

        hal_pUART[port]->HAL_USART_TX_DATA = hal_UARTv[port]->pTxBuf[hal_UARTv[port]->tx_pos];
        hal_UARTv[port]->tx_pos++;
    }
}

#ifdef USART1
void USART1_IRQHandler(void)
{
    hal_uart_irq_handler(0);
}
#endif  // USART1_IRQHandler

#ifdef USART2
void USART2_IRQHandler(void)
{
    hal_uart_irq_handler(1);
}
#endif  //  USART2_IRQHandler

void hal_uart_deinit(uint8_t port)
{
    switch(port)
    {
#ifdef USART1
        case 0:
            {
            USART1->CR1 &= ~USART_CR1_UE;               // Disable USART
            NVIC_DisableIRQ(USART1_IRQn);               // Disable USART IRQ

            RCC->APB2RSTR |= RCC_APB2RSTR_USART1RST;    // Reset USART
            RCC->APB2RSTR &= ~RCC_APB2RSTR_USART1RST;

            RCC->APB2ENR  &= ~RCC_APB2ENR_USART1EN;     // Disable UART1 Clock
            }
            break;
#endif  //  USART1
#ifdef USART2
        case 1:
            {
            USART2->CR1 &= ~USART_CR1_UE;               // Disable USART
            NVIC_DisableIRQ(USART2_IRQn);               // Disable USART IRQ

            RCC->APB1RSTR |= RCC_APB1RSTR_USART2RST;    // Reset USART
            RCC->APB1RSTR &= ~RCC_APB1RSTR_USART2RST;

            RCC->APB1ENR   &= ~RCC_APB1ENR_USART2EN;    // Disable UART2 Clock
            }
            break;
#endif  //  USART2
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
    assert(nBaud < 5);
    
    IRQn_Type           UARTx_IRQn;
    RCC_ClocksTypeDef   RCC_ClocksStatus;
    
    uint32_t            uart_clock;
    uint16_t            baud = hal_baud_list[nBaud];

    RCC_GetClocksFreq(&RCC_ClocksStatus);

    switch(port)
    {
#ifdef USART1
        case 0:
            {
            UARTx_IRQn = USART1_IRQn;
            RCC->APB2ENR   |= RCC_APB2ENR_USART1EN;         // Enable UART1 Clock

#if (defined __STM32F0XX_H)
            uart_clock = RCC_ClocksStatus.USART1CLK_Frequency;
            
            GPIOA->MODER   |= GPIO_MODER_MODER9_1;          // PA9  (TX) - Alternate function mode
            GPIOA->MODER   |= GPIO_MODER_MODER10_1;         // PA10 (RX) - Alternate function mode
            GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR9;       // PA9  (TX) - High speed
            GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR10;      // PA10 (RX) - High speed
            GPIOA->AFR[1]  |= 0x0110;                       // PA9, PA10 - AF1
#elif (defined __STM32F10x_H)
            uart_clock = RCC_ClocksStatus.PCLK2_Frequency;
            
            // Configure GPIO, Tx on PA9, Rx on PA10
            GPIOA->CRH &= ~GPIO_CRH_CNF9_0;
            GPIOA->CRH |= GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9; // AF Push-Pull out (TX)
#endif
            }
            break;
#endif  //  USART1
#ifdef USART2
        case 1:
            {
            UARTx_IRQn = USART2_IRQn;
            RCC->APB1ENR   |= RCC_APB1ENR_USART2EN;         // Enable UART2 Clock
            
#if (defined __STM32F0XX_H)
            uart_clock = RCC_ClocksStatus.PCLK_Frequency;
            
            GPIOA->MODER   |= GPIO_MODER_MODER2_1;          // PA2  (TX) - Alternate function mode
            GPIOA->MODER   |= GPIO_MODER_MODER3_1;          // PA3  (RX) - Alternate function mode
            GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR2;       // PA2  (TX) - High speed
            GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR3;       // PA3  (RX) - High speed
            GPIOA->AFR[0]  |= 0x1100;                       // PA2, PA3  - AF1
#elif (defined __STM32F10x_H)
            uart_clock = RCC_ClocksStatus.PCLK1_Frequency;
            
            // Configure GPIO, Tx on PA2, Rx on PA3
            GPIOA->CRL &= ~GPIO_CRL_CNF2_0;
            GPIOA->CRL |= GPIO_CRL_CNF2_1 | GPIO_CRL_MODE2; // AF Push-Pull out (TX)
#endif
            }
            break;
#endif  //  USART2
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
    
    hal_UARTv[port]->pTxBuf = NULL;
    hal_UARTv[port]->tx_len = 0;
    hal_UARTv[port]->tx_pos = 0;

    hal_pUART[port]->CR1 = 0;                               // Disable USART1
    hal_pUART[port]->CR2 = 0;                               // 8N1
    hal_pUART[port]->CR3 = 0;                               // Without flow control
    hal_pUART[port]->BRR  = ((uart_clock + baud/2)/baud);   // Speed
    hal_pUART[port]->CR1 |= USART_CR1_TE |                  // Enable TX
                            USART_CR1_RE |                  // Enable RX
                            USART_CR1_RXNEIE;               // Enable RX Not Empty IRQ
    NVIC_EnableIRQ(UARTx_IRQn);                             // Enable UASRT IRQ
    hal_pUART[port]->CR1 |= USART_CR1_UE;                   // Enable USART
}

bool hal_uart_datardy(uint8_t port)
{
    assert(hal_UARTv[port] != NULL);
    return (hal_UARTv[port]->rx_head != hal_UARTv[port]->rx_tail);
}

uint8_t hal_uart_get(uint8_t port)
{
    assert(hal_UARTv[port] != NULL);

    if(hal_UARTv[port]->rx_head == hal_UARTv[port]->rx_tail)
        return 0;
    
    uint8_t data = hal_UARTv[port]->rx_fifo[hal_UARTv[port]->rx_tail];
    hal_UARTv[port]->rx_tail++;
    hal_UARTv[port]->rx_tail &= (uint8_t)(HAL_SIZEOF_UART_RX_FIFO - 1);

    return data;
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

    hal_pUART[port]->HAL_USART_TX_DATA = *pBuf;
    hal_pUART[port]->CR1 |= USART_CR1_TXEIE;
}

#endif  //  ((defined UART_PHY) || (defined EXTSER_USED))
