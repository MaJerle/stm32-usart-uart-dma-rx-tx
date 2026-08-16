/* Includes */
#include "main.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/* Private function prototypes */
void SystemClock_Config(void);

/* USART related functions */
void usart_init(void);
void usart_rx_check(void);
void usart_process_data(const void* data, size_t len);
void usart_send_string(const char* str);

/**
 * \brief           Calculate length of statically allocated array
 */
#define ARRAY_LEN(x)            (sizeof(x) / sizeof((x)[0]))

/**
 * \brief           USART RX buffer for DMA to transfer every received byte
 * \note            Contains raw data that are about to be processed by different events
 */
uint8_t
usart_rx_dma_buffer[64];

/**
 * \brief           Application entry point
 */
int
main(void) {
    /* MCU Configuration */

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
    NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    usart_init();
    usart_send_string("USART DMA example: DMA HT & TC + USART IDLE LINE interrupts\r\n");
    usart_send_string("Start sending data to STM32\r\n");

    /* Infinite loop */
    while (1) {
        /* Nothing to process here */
        /* Everything is processed either by DMA or USART interrupts */

        /* Do task 1 */
        /* Do task 2 */
        /* Do task 3 */
        /* Do task 4 */
        /* Do task 5 */
    }
}

/**
 * \brief           Check for new data received with DMA
 *
 * User must select context to call this function from:
 * - Only interrupts (DMA HT, DMA TC, UART IDLE) with same preemption priority level
 * - Only thread context (outside interrupts)
 *
 * If called from both context-es, exclusive access protection must be implemented
 * This mode is not advised as it usually means architecture design problems
 *
 * When IDLE interrupt is not present, application must rely only on thread context,
 * by manually calling function as quickly as possible, to make sure
 * data are read from raw buffer and processed.
 *
 * Not doing reads fast enough may cause DMA to overflow unread received bytes,
 * hence application will lost useful data.
 *
 * Solutions to this are:
 * - Improve architecture design to achieve faster reads
 * - Increase raw buffer size and allow DMA to write more data before this function is called
 */
void
usart_rx_check(void) {
    static size_t old_pos;
    size_t pos;

    /* Calculate current position in buffer and check for new data available */
    pos = ARRAY_LEN(usart_rx_dma_buffer) - LL_DMA_GetDataLength(DMA1, LL_DMA_STREAM_1);
    if (pos != old_pos) {                       /* Check change in received data */
        if (pos > old_pos) {                    /* Current position is over previous one */
            /*
             * Processing is done in "linear" mode.
             *
             * Application processing is fast with single data block,
             * length is simply calculated by subtracting pointers
             *
             * [   0   ]
             * [   1   ] <- old_pos |------------------------------------|
             * [   2   ]            |                                    |
             * [   3   ]            | Single block (len = pos - old_pos) |
             * [   4   ]            |                                    |
             * [   5   ]            |------------------------------------|
             * [   6   ] <- pos
             * [   7   ]
             * [ N - 1 ]
             */
            usart_process_data(&usart_rx_dma_buffer[old_pos], pos - old_pos);
        } else {
            /*
             * Processing is done in "overflow" mode..
             *
             * Application must process data twice,
             * since there are 2 linear memory blocks to handle
             *
             * [   0   ]            |---------------------------------|
             * [   1   ]            | Second block (len = pos)        |
             * [   2   ]            |---------------------------------|
             * [   3   ] <- pos
             * [   4   ] <- old_pos |---------------------------------|
             * [   5   ]            |                                 |
             * [   6   ]            | First block (len = N - old_pos) |
             * [   7   ]            |                                 |
             * [ N - 1 ]            |---------------------------------|
             */
            usart_process_data(&usart_rx_dma_buffer[old_pos], ARRAY_LEN(usart_rx_dma_buffer) - old_pos);
            if (pos > 0) {
                usart_process_data(&usart_rx_dma_buffer[0], pos);
            }
        }
        old_pos = pos;                          /* Save current position as old for next transfers */
    }
}

/**
 * \brief           Process received data over UART
 * \note            Either process them directly or copy to other bigger buffer
 * \param[in]       data: Data to process
 * \param[in]       len: Length in units of bytes
 */
void
usart_process_data(const void* data, size_t len) {
    const uint8_t* d = data;
    
    /*
     * This function is called on DMA TC or HT events, and on UART IDLE (if enabled) event.
     * 
     * For the sake of this example, function does a loop-back data over UART in polling mode.
     * Check ringbuff RX-based example for implementation with TX & RX DMA transfer.
     */
    
    for (; len > 0; --len, ++d) {
        LL_USART_TransmitData8(USART3, *d);
        while (!LL_USART_IsActiveFlag_TXE(USART3)) {}
    }
    while (!LL_USART_IsActiveFlag_TC(USART3)) {}
}

/**
 * \brief           Send string to USART
 * \param[in]       str: String to send
 */
void
usart_send_string(const char* str) {
    usart_process_data(str, strlen(str));
}

/**
 * \brief           USART3 Initialization Function
 */
void
usart_init(void) {
    LL_USART_InitTypeDef USART_InitStruct = {0};
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Peripheral clock enable */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

    /*
     * USART3 GPIO Configuration
     *
     * PD8   ------> USART3_TX
     * PD9   ------> USART3_RX
     */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_8 | LL_GPIO_PIN_9;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
    LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* USART3 DMA Init */
    LL_DMA_SetChannelSelection(DMA1, LL_DMA_STREAM_1, LL_DMA_CHANNEL_4);
    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_1, LL_DMA_PRIORITY_LOW);
    LL_DMA_SetMode(DMA1, LL_DMA_STREAM_1, LL_DMA_MODE_CIRCULAR);
    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_1, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_1, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_1, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_1, LL_DMA_MDATAALIGN_BYTE);
    LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_1);

    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_1, LL_USART_DMA_GetRegAddr(USART3));
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_1, (uint32_t)usart_rx_dma_buffer);
    LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_1, ARRAY_LEN(usart_rx_dma_buffer));

    /* Enable HT & TC interrupts */
    LL_DMA_EnableIT_HT(DMA1, LL_DMA_STREAM_1);
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_1);

    /* DMA interrupt init */
    NVIC_SetPriority(DMA1_Stream1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(DMA1_Stream1_IRQn);

    /* USART configuration */
    USART_InitStruct.BaudRate = 115200;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
    LL_USART_Init(USART3, &USART_InitStruct);
    LL_USART_ConfigAsyncMode(USART3);
    LL_USART_EnableDMAReq_RX(USART3);
    LL_USART_EnableIT_IDLE(USART3);

    /* USART interrupt */
    NVIC_SetPriority(USART3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(USART3_IRQn);

    /* Enable USART and DMA */
    LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_1);
    LL_USART_Enable(USART3);
}

/* Interrupt handlers here */

/**
 * \brief           DMA1 stream1 interrupt handler for USART3 RX
 */
void
DMA1_Stream1_IRQHandler(void) {
    /* Check half-transfer complete interrupt */
    if (LL_DMA_IsEnabledIT_HT(DMA1, LL_DMA_STREAM_1) && LL_DMA_IsActiveFlag_HT1(DMA1)) {
        LL_DMA_ClearFlag_HT1(DMA1);             /* Clear half-transfer complete flag */
        usart_rx_check();                       /* Check for data to process */
    }

    /* Check transfer-complete interrupt */
    if (LL_DMA_IsEnabledIT_TC(DMA1, LL_DMA_STREAM_1) && LL_DMA_IsActiveFlag_TC1(DMA1)) {
        LL_DMA_ClearFlag_TC1(DMA1);             /* Clear transfer complete flag */
        usart_rx_check();                       /* Check for data to process */
    }

    /* Implement other events when needed */
}

/**
 * \brief           USART3 global interrupt handler
 */
void
USART3_IRQHandler(void) {
    /* Check for IDLE line interrupt */
    if (LL_USART_IsEnabledIT_IDLE(USART3) && LL_USART_IsActiveFlag_IDLE(USART3)) {
        LL_USART_ClearFlag_IDLE(USART3);        /* Clear IDLE line flag */
        usart_rx_check();                       /* Check for data to process */
    }

    /* Implement other events when needed */
}


/**
 * \brief           System Clock Configuration
 */
void
SystemClock_Config(void) {
    /* Configure flash latency */
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_3);
    if (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_3) {
        while (1) {}
    }

    /* Configure voltage scaling */
    LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);

    /* Configure HSI */
    LL_RCC_HSI_SetCalibTrimming(16);
    LL_RCC_HSI_Enable();
    while (LL_RCC_HSI_IsReady() != 1) {}

    /* Configure PLL */
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_8, 100, LL_RCC_PLLP_DIV_2);
    LL_RCC_PLL_Enable();
    while (LL_RCC_PLL_IsReady() != 1) {}

    /* Configure system prescalers */
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {}

    /* Configure systick */
    LL_Init1msTick(100000000);
    LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);
    LL_SYSTICK_EnableIT();
    LL_SetSystemCoreClock(100000000);
    LL_RCC_SetTIMPrescaler(LL_RCC_TIM_PRESCALER_TWICE);
}
