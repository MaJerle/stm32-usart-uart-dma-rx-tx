/* Includes */
#include "main.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "cmsis_os.h"

/* Private function prototypes */
void SystemClock_Config(void);

/**
 * \brief           Volatile data structure
 *
 * Variables declared using this structure can not be put to non-volatile memory (such as flash, EEPROM, ..)
 */
typedef struct {
    /* OS queue */
    osMessageQueueId_t queue;                   /*!< Message queue */

    /* Raw data buffer */
    uint8_t dma_rx_buffer[64];                  /*!< DMA buffer for receive data */
    size_t old_pos;                             /*!< Position for data */
} uart_desc_volatile_t;

/**
 * \brief           Non-Volatile data structure
 *
 * Variables declared using this structure may be put to non-volative memory.
 * This is to avoid using RAM for constant data
 */
typedef struct {
    /* UART config */
    USART_TypeDef* uart;                        /*!< UART/USART/LPUART instance */
    GPIO_TypeDef* uart_tx_port;
    GPIO_TypeDef* uart_rx_port;
    uint16_t uart_tx_pin;
    uint16_t uart_rx_pin;
    uint16_t uart_tx_pin_af;
    uint16_t uart_rx_pin_af;

    /* DMA config & flags management */
    DMA_TypeDef* dma_rx;                        /*!< RX DMA instance */
    uint32_t dma_rx_ch;                         /*!< RX DMA channel */
    uint32_t dma_rx_req;                        /*!< RX DMA request */
    void (*dma_rx_clear_tc_fn)(DMA_TypeDef *);
    void (*dma_rx_clear_ht_fn)(DMA_TypeDef *);
    uint32_t (*dma_rx_is_tc_fn)(DMA_TypeDef *);
    uint32_t (*dma_rx_is_ht_fn)(DMA_TypeDef *);

    /* Interrupts config */
    uint8_t prio;                               /*!< Preemption priority number */
    IRQn_Type uart_irq;                         /*!< UART IRQ instance */
    IRQn_Type dma_irq;                          /*!< DMA IRQ instance */

    uart_desc_volatile_t* data;                 /*!< Pointer to volatile data */
} uart_desc_t;

/* USART related functions */
void usart_init(const uart_desc_t* uart);
void usart_rx_check(const uart_desc_t* uart);
void usart_process_data(const uart_desc_t* uart, const void* data, size_t len);
void usart_send_string(const uart_desc_t* uart, const char* str);
void usart_dma_irq_handler(const uart_desc_t* uart);
void usart_irq_handler(const uart_desc_t* uart);

/**
 * \brief           Calculate length of statically allocated array
 */
#define ARRAY_LEN(x)            (sizeof(x) / sizeof((x)[0]))

/* Thread function entry point */
void init_thread(void* arg);
void usart_rx_dma_thread(void* arg);

/**
 * \brief           USART volatile data
 */
static uart_desc_volatile_t uart1_desc_data;
static uart_desc_volatile_t uart2_desc_data;

/**
 * \brief           USART1 setup
 */
static const uart_desc_t
uart1_desc = {
    /* UART config */
    .uart = USART1,
    .uart_tx_port = GPIOA,
    .uart_tx_pin = LL_GPIO_PIN_9,
    .uart_tx_pin_af = LL_GPIO_AF_7,
    .uart_rx_port = GPIOA,
    .uart_rx_pin = LL_GPIO_PIN_10,
    .uart_rx_pin_af = LL_GPIO_AF_7,

    /* DMA config */
    .dma_rx = DMA1,
    .dma_rx_ch = LL_DMA_CHANNEL_5,
    .dma_rx_req = LL_DMA_REQUEST_2,
    .dma_irq = DMA1_Channel5_IRQn,
    .uart_irq = USART1_IRQn,
    .prio = 5,
    .dma_rx_clear_tc_fn = LL_DMA_ClearFlag_TC5,
    .dma_rx_clear_ht_fn = LL_DMA_ClearFlag_HT5,
    .dma_rx_is_tc_fn = LL_DMA_IsActiveFlag_TC5,
    .dma_rx_is_ht_fn = LL_DMA_IsActiveFlag_HT5,

    /* Volatile data */
    .data = &uart1_desc_data,
};

/**
 * \brief           USART2 setup
 */
static const uart_desc_t
uart2_desc = {
    /* UART config */
    .uart = USART2,
    .uart_tx_port = GPIOA,
    .uart_tx_pin = LL_GPIO_PIN_2,
    .uart_tx_pin_af = LL_GPIO_AF_7,
    .uart_rx_port = GPIOA,
    .uart_rx_pin = LL_GPIO_PIN_15,
    .uart_rx_pin_af = LL_GPIO_AF_3,

    /* DMA config */
    .dma_rx = DMA1,
    .dma_rx_ch = LL_DMA_CHANNEL_6,
    .dma_rx_req = LL_DMA_REQUEST_2,
    .dma_irq = DMA1_Channel6_IRQn,
    .uart_irq = USART2_IRQn,
    .prio = 5,
    .dma_rx_clear_tc_fn = LL_DMA_ClearFlag_TC6,
    .dma_rx_clear_ht_fn = LL_DMA_ClearFlag_HT6,
    .dma_rx_is_tc_fn = LL_DMA_IsActiveFlag_TC6,
    .dma_rx_is_ht_fn = LL_DMA_IsActiveFlag_HT6,

    /* Volatile data */
    .data = &uart2_desc_data,
};

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

    /* Init kernel, create thread(s) and start it */
    osKernelInitialize();
    osThreadNew(init_thread, NULL, NULL);
    osKernelStart();

    while (1) {}
}

/**
 * \brief           Init thread
 * \param[in]       arg: Thread argument
 */
void
init_thread(void* arg) {
    /* Initialize all configured peripherals */

    /* Initialize both UARTs */
    /* It will start separate thread for each instance */
    usart_init(&uart1_desc);
    usart_init(&uart2_desc);

    /* Do other initializations if needed */

    /* Terminate this thread */
    osThreadExit();
}

/**
 * \brief           USART DMA check thread
 * \param[in]       arg: Thread argument
 */
void
usart_rx_dma_thread(void* arg) {
    uart_desc_t* uart = arg;
    void* d;

    /* Notify user to start sending data */
    usart_send_string(uart, "USART DMA example: DMA HT & TC + USART IDLE LINE IRQ + RTOS processing\r\n");
    usart_send_string(uart, "Start sending data to STM32\r\n");

    while (1) {
        /* Block thread and wait for event to process USART data */
        osMessageQueueGet(uart->data->queue, &d, NULL, osWaitForever);

        /* Simply call processing function */
        usart_rx_check(uart);

        (void)d;
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
usart_rx_check(const uart_desc_t* uart) {
    size_t pos;

    /* Calculate current position in buffer and check for new data available */
    pos = ARRAY_LEN(uart->data->dma_rx_buffer) - LL_DMA_GetDataLength(uart->dma_rx, uart->dma_rx_ch);
    if (pos != uart->data->old_pos) {           /* Check change in received data */
        if (pos > uart->data->old_pos) {        /* Current position is over previous one */
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
            usart_process_data(uart, &uart->data->dma_rx_buffer[uart->data->old_pos], pos - uart->data->old_pos);
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
            usart_process_data(uart, &uart->data->dma_rx_buffer[uart->data->old_pos], ARRAY_LEN(uart->data->dma_rx_buffer) - uart->data->old_pos);
            if (pos > 0) {
                usart_process_data(uart, &uart->data->dma_rx_buffer[0], pos);
            }
        }
        uart->data->old_pos = pos;              /* Save current position as old for next transfers */
    }
}

/**
 * \brief           Process received data over UART
 * \note            Either process them directly or copy to other bigger buffer
 * \param[in]       data: Data to process
 * \param[in]       len: Length in units of bytes
 */
void
usart_process_data(const uart_desc_t* uart, const void* data, size_t len) {
    const uint8_t* d = data;
    
    /*
     * This function is called on DMA TC or HT events, and on UART IDLE (if enabled) event.
     * 
     * For the sake of this example, function does a loop-back data over UART in polling mode.
     * Check ringbuff RX-based example for implementation with TX & RX DMA transfer.
     */
    
    for (; len > 0; --len, ++d) {
        LL_USART_TransmitData8(uart->uart, *d);
        while (!LL_USART_IsActiveFlag_TXE(uart->uart)) {}
    }
    while (!LL_USART_IsActiveFlag_TC(uart->uart)) {}
}

/**
 * \brief           Send string to USART
 * \param[in]       str: String to send
 */
void
usart_send_string(const uart_desc_t* uart, const char* str) {
    usart_process_data(uart, str, strlen(str));
}

/**
 * \brief           USART2 Initialization Function
 */
void
usart_init(const uart_desc_t* uart) {
    LL_USART_InitTypeDef USART_InitStruct = {0};
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Peripheral clock enable */
    /* Enable all for simplicity */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);

    /* USART GPIO Configuration */
    /* RX pin */
    GPIO_InitStruct.Pin = uart->uart_rx_pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = uart->uart_rx_pin_af;
    LL_GPIO_Init(uart->uart_rx_port, &GPIO_InitStruct);

    /* Optional TX pin */
    if (uart->uart_tx_port != NULL) {
        GPIO_InitStruct.Pin = uart->uart_tx_pin;
        GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
        GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
        GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
        GPIO_InitStruct.Alternate = uart->uart_tx_pin_af;
        LL_GPIO_Init(uart->uart_tx_port, &GPIO_InitStruct);
    }

    /* USART DMA init */
    LL_DMA_SetPeriphRequest(uart->dma_rx, uart->dma_rx_ch, uart->dma_rx_req);
    LL_DMA_SetDataTransferDirection(uart->dma_rx, uart->dma_rx_ch, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetChannelPriorityLevel(uart->dma_rx, uart->dma_rx_ch, LL_DMA_PRIORITY_LOW);
    LL_DMA_SetMode(uart->dma_rx, uart->dma_rx_ch, LL_DMA_MODE_CIRCULAR);
    LL_DMA_SetPeriphIncMode(uart->dma_rx, uart->dma_rx_ch, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(uart->dma_rx, uart->dma_rx_ch, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(uart->dma_rx, uart->dma_rx_ch, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(uart->dma_rx, uart->dma_rx_ch, LL_DMA_MDATAALIGN_BYTE);
    LL_DMA_SetPeriphAddress(uart->dma_rx, uart->dma_rx_ch, (uint32_t)&uart->uart->RDR);
    LL_DMA_SetMemoryAddress(uart->dma_rx, uart->dma_rx_ch, (uint32_t)uart->data->dma_rx_buffer);
    LL_DMA_SetDataLength(uart->dma_rx, uart->dma_rx_ch, ARRAY_LEN(uart->data->dma_rx_buffer));

    /* Enable HT & TC interrupts */
    LL_DMA_EnableIT_HT(uart->dma_rx, uart->dma_rx_ch);
    LL_DMA_EnableIT_TC(uart->dma_rx, uart->dma_rx_ch);

    /* DMA interrupt init */
    NVIC_SetPriority(uart->dma_irq, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), uart->prio, 0));
    NVIC_EnableIRQ(uart->dma_irq);

    /* USART configuration */
    USART_InitStruct.BaudRate = 115200;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
    LL_USART_Init(uart->uart, &USART_InitStruct);
    LL_USART_ConfigAsyncMode(uart->uart);
    LL_USART_EnableDMAReq_RX(uart->uart);
    LL_USART_EnableIT_IDLE(uart->uart);

    /* USART interrupt */
    NVIC_SetPriority(uart->uart_irq, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), uart->prio, 1));
    NVIC_EnableIRQ(uart->uart_irq);

    /* Create message queue before enabling UART or DMA */
    /* This is to make sure message queue is ready before UART/DMA interrupts are enabled */
    uart->data->queue = osMessageQueueNew(10, sizeof(void *), NULL);

    /* Enable USART and DMA */
    LL_DMA_EnableChannel(uart->dma_rx, uart->dma_rx_ch);
    LL_USART_Enable(uart->uart);

    /* Create new thread for USART RX DMA processing */
    osThreadNew(usart_rx_dma_thread, (void *)uart, NULL);
}

/**
 * \brief           General purpose DMA interrupt handler
 * \note            Function must be called from DMA interrupt
 *
 * It handles half-transfer and transfer-complete interrupts and does the job accordingly
 *
 * \param[in]       uart: Uart description to handle
 */
void
usart_dma_irq_handler(const uart_desc_t* uart) {
    void* d = (void *)1;

    /* Check half-transfer complete interrupt */
    if (LL_DMA_IsEnabledIT_HT(uart->dma_rx, uart->dma_rx_ch) && uart->dma_rx_is_ht_fn(uart->dma_rx)) {
        uart->dma_rx_clear_ht_fn(uart->dma_rx); /* Clear half-transfer complete flag */
        osMessageQueuePut(uart->data->queue, &d, 0, 0); /* Write data to queue. Do not use wait function! */
    }

    /* Check transfer-complete interrupt */
    if (LL_DMA_IsEnabledIT_TC(uart->dma_rx, uart->dma_rx_ch) && uart->dma_rx_is_tc_fn(uart->dma_rx)) {
        uart->dma_rx_clear_tc_fn(uart->dma_rx); /* Clear transfer complete flag */
        osMessageQueuePut(uart->data->queue, &d, 0, 0); /* Write data to queue. Do not use wait function! */
    }
}

/**
 * \brief           General purpose UART interrupt handler
 * \note            Function must be called from UART interrupt
 *
 * It handles IDLE line detection interrupt and does the job accordingly
 *
 * \param[in]       uart: Uart description to handle
 */
void
usart_irq_handler(const uart_desc_t* uart) {
    void* d = (void *)1;

    /* Check for IDLE line interrupt */
    if (LL_USART_IsEnabledIT_IDLE(uart->uart) && LL_USART_IsActiveFlag_IDLE(uart->uart)) {
        LL_USART_ClearFlag_IDLE(uart->uart);    /* Clear IDLE line flag */
        osMessageQueuePut(uart->data->queue, &d, 0, 0);  /* Write data to queue. Do not use wait function! */
    }
}


/* Interrupt handlers here */

/**
 * \brief           DMA1 channel5 interrupt handler for USART1 RX
 */
void
DMA1_Channel5_IRQHandler(void) {
    usart_dma_irq_handler(&uart1_desc);
}

/**
 * \brief           DMA1 channel6 interrupt handler for USART2 RX
 */
void
DMA1_Channel6_IRQHandler(void) {
    usart_dma_irq_handler(&uart2_desc);
}

/**
 * \brief           USART1 global interrupt handler
 */
void
USART1_IRQHandler(void) {
    usart_irq_handler(&uart1_desc);
}

/**
 * \brief           USART2 global interrupt handler
 */
void
USART2_IRQHandler(void) {
    usart_irq_handler(&uart2_desc);
}


/* System clock config here */

/**
 * \brief           System Clock Configuration
 */
void
SystemClock_Config(void) {
    /* Configure flash latency */
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
    if (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_4) {
        while (1) {}
    }

    /* Configure voltage scaling */
    LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);

    /* Configure MSI */
    LL_RCC_MSI_Enable();
    while (LL_RCC_MSI_IsReady() != 1) {}
    LL_RCC_MSI_EnableRangeSelection();
    LL_RCC_MSI_SetRange(LL_RCC_MSIRANGE_11);
    LL_RCC_MSI_SetCalibTrimming(0);

    /* Configure system clock */
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_MSI);
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_MSI) {}
    
    /* Configure prescalers */
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

    /* Configure systick */
    LL_Init1msTick(48000000);
    LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);
    LL_SetSystemCoreClock(48000000);
    LL_RCC_SetUSARTClockSource(LL_RCC_USART2_CLKSOURCE_PCLK1);
}
