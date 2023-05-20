#include "main.h"
#include "lwrb/lwrb.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/* System private function */
void SystemClock_Config(void);
static void SystemPower_Config(void);

/* USART related functions */
void usart_init(void);
void usart_rx_check(void);
void usart_process_data(const void* data, size_t len);
void usart_send_string(const char* str);
uint8_t usart_start_tx_dma_transfer(void);

/* Static & global variable for linked list node */
static LL_DMA_LinkNodeTypeDef Node_GPDMA1_Channel0;

/**
 * \brief           Ring buffer instance for TX data
 */
lwrb_t usart_tx_rb;

/**
 * \brief           Ring buffer data array for TX DMA
 */
uint8_t usart_tx_rb_data[128];

/**
 * \brief           Length of currently active TX DMA transfer
 */
volatile size_t usart_tx_dma_current_len;

/**
 * \brief           USART RX buffer for DMA to transfer every received byte
 * \note            Contains raw data that are about to be processed by different events
 */
uint8_t usart_rx_dma_buffer[64];

/**
 * \brief           The application entry point
 */
int
main(void) {
    /* MCU Configuration */

    /* System interrupt init */
    NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
    NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 15, 0));
    LL_AHB3_GRP1_EnableClock(LL_AHB3_GRP1_PERIPH_PWR);

    /* Configure the system clock and power */
    SystemClock_Config();
    SystemPower_Config();

    /* Enable instruction cache */
    LL_ICACHE_SetMode(LL_ICACHE_1WAY);
    LL_ICACHE_Enable();

    /* Initialize ringbuff */
    lwrb_init(&usart_tx_rb, usart_tx_rb_data, sizeof(usart_tx_rb_data));

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
    pos = sizeof(usart_rx_dma_buffer) - LL_DMA_GetBlkDataLength(GPDMA1, LL_DMA_CHANNEL_0);
    if (pos != old_pos) {    /* Check change in received data */
        if (pos > old_pos) { /* Current position is over previous one */
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
            usart_process_data(&usart_rx_dma_buffer[old_pos], sizeof(usart_rx_dma_buffer) - old_pos);
            if (pos > 0) {
                usart_process_data(&usart_rx_dma_buffer[0], pos);
            }
        }
        old_pos = pos; /* Save current position as old for next transfers */
    }
}

/**
 * \brief           Check if DMA is active and if not try to send data
 * \return          `1` if transfer just started, `0` if on-going or no data to transmit
 */
uint8_t
usart_start_tx_dma_transfer(void) {
    uint32_t primask;
    uint8_t started = 0;

    /*
     * First check if transfer is currently in-active,
     * by examining the value of usart_tx_dma_current_len variable.
     *
     * This variable is set before DMA transfer is started and cleared in DMA TX complete interrupt.
     *
     * It is not necessary to disable the interrupts before checking the variable:
     *
     * When usart_tx_dma_current_len == 0
     *    - This function is called by either application or TX DMA interrupt
     *    - When called from interrupt, it was just reset before the call,
     *         indicating transfer just completed and ready for more
     *    - When called from an application, transfer was previously already in-active
     *         and immediate call from interrupt cannot happen at this moment
     *
     * When usart_tx_dma_current_len != 0
     *    - This function is called only by an application.
     *    - It will never be called from interrupt with usart_tx_dma_current_len != 0 condition
     *
     * Disabling interrupts before checking for next transfer is advised
     * only if multiple operating system threads can access to this function w/o
     * exclusive access protection (mutex) configured,
     * or if application calls this function from multiple interrupts.
     *
     * This example assumes worst use case scenario,
     * hence interrupts are disabled prior every check
     */
    primask = __get_PRIMASK();
    __disable_irq();
    if (usart_tx_dma_current_len == 0
        && (usart_tx_dma_current_len = lwrb_get_linear_block_read_length(&usart_tx_rb)) > 0) {
        /* Disable channel if enabled */
        LL_DMA_DisableChannel(GPDMA1, LL_DMA_CHANNEL_1);

        /* Clear all flags */
        LL_DMA_ClearFlag_HT(GPDMA1, LL_DMA_CHANNEL_1);
        LL_DMA_ClearFlag_DTE(GPDMA1, LL_DMA_CHANNEL_1);
        LL_DMA_ClearFlag_SUSP(GPDMA1, LL_DMA_CHANNEL_1);
        LL_DMA_ClearFlag_TC(GPDMA1, LL_DMA_CHANNEL_1);
        LL_DMA_ClearFlag_TO(GPDMA1, LL_DMA_CHANNEL_1);
        LL_DMA_ClearFlag_ULE(GPDMA1, LL_DMA_CHANNEL_1);
        LL_DMA_ClearFlag_USE(GPDMA1, LL_DMA_CHANNEL_1);

        /* Prepare DMA data and length */
        LL_DMA_SetBlkDataLength(GPDMA1, LL_DMA_CHANNEL_1, usart_tx_dma_current_len);
        LL_DMA_SetSrcAddress(GPDMA1, LL_DMA_CHANNEL_1, (uint32_t)lwrb_get_linear_block_read_address(&usart_tx_rb));

        /* Start transfer */
        LL_DMA_EnableChannel(GPDMA1, LL_DMA_CHANNEL_1);
        started = 1;
    }
    __set_PRIMASK(primask);
    return started;
}

/**
 * \brief           Process received data over UART
 * \note            Either process them directly or copy to other bigger buffer
 * \param[in]       data: Data to process
 * \param[in]       len: Length in units of bytes
 */
void
usart_process_data(const void* data, size_t len) {
    lwrb_write(&usart_tx_rb, data, len); /* Write data to TX buffer for loopback */
    usart_start_tx_dma_transfer();       /* Then try to start transfer */
}

/**
 * \brief           Send string to USART
 * \param[in]       str: String to send
 */
void
usart_send_string(const char* str) {
    lwrb_write(&usart_tx_rb, str, strlen(str)); /* Write data to TX buffer for loopback */
    usart_start_tx_dma_transfer();              /* Then try to start transfer */
}

/**
 * \brief           USART1 Initialization Function
 */
void
usart_init(void) {
    LL_USART_InitTypeDef USART_InitStruct = {0};
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    LL_DMA_InitTypeDef DMA_InitStruct = {0};
    LL_DMA_InitNodeTypeDef NodeConfig = {0};
    LL_DMA_InitLinkedListTypeDef DMA_InitLinkedListStruct = {0};

    /* Set UART kernel clock */
    LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_PCLK2);

    /* Peripheral clock enable */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPDMA1);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);

    /*
     * USART1 GPIO Configuration
     *
     * PA9    ------> USART1_TX
     * PA10   ------> USART1_RX
     */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_9 | LL_GPIO_PIN_10;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*
     * Configure GPDMA CH1 for USART TX operation in normal mode
     *
     * - Normal mode - no linked list operation
     * - Source address increase from memory
     * - Destination address no-increase to peripheral (USART TDR)
     * - Length set before channel enabled
     * - Source address set before TX operation
     * - Source uses PORT1 - fast port to access RAM through bus matrix
     * - Destination uses PORT0 - fast port for APB access bypassing bus matrix
     * 
     * Have a look at AN5593 for GPDMA1 use case.
     * https://www.st.com/resource/en/application_note/an5593-how-to-use-the-gpdma-for-stm32u575585-microcontrollers-stmicroelectronics.pdf 
     */
    DMA_InitStruct.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
    DMA_InitStruct.BlkHWRequest = LL_DMA_HWREQUEST_SINGLEBURST;
    DMA_InitStruct.DataAlignment = LL_DMA_DATA_ALIGN_ZEROPADD;
    DMA_InitStruct.SrcBurstLength = 1;
    DMA_InitStruct.DestBurstLength = 1;
    DMA_InitStruct.SrcDataWidth = LL_DMA_SRC_DATAWIDTH_BYTE;
    DMA_InitStruct.DestDataWidth = LL_DMA_DEST_DATAWIDTH_BYTE;
    DMA_InitStruct.SrcIncMode = LL_DMA_SRC_INCREMENT;
    DMA_InitStruct.DestIncMode = LL_DMA_DEST_FIXED;
    DMA_InitStruct.Priority = LL_DMA_LOW_PRIORITY_MID_WEIGHT;
    DMA_InitStruct.TriggerMode = LL_DMA_TRIGM_BLK_TRANSFER;
    DMA_InitStruct.TriggerPolarity = LL_DMA_TRIG_POLARITY_MASKED;
    DMA_InitStruct.TriggerSelection = 0x00000000U;
    DMA_InitStruct.Request = LL_GPDMA1_REQUEST_USART1_TX;
    DMA_InitStruct.TransferEventMode = LL_DMA_TCEM_BLK_TRANSFER;
    DMA_InitStruct.SrcAllocatedPort = LL_DMA_SRC_ALLOCATED_PORT1;
    DMA_InitStruct.DestAllocatedPort = LL_DMA_DEST_ALLOCATED_PORT0;
    DMA_InitStruct.LinkAllocatedPort = LL_DMA_LINK_ALLOCATED_PORT1;
    DMA_InitStruct.LinkStepMode = LL_DMA_LSM_FULL_EXECUTION;
    DMA_InitStruct.LinkedListBaseAddr = 0x00000000U;
    DMA_InitStruct.LinkedListAddrOffset = 0x00000000U;
    DMA_InitStruct.DestAddress = LL_USART_DMA_GetRegAddr(USART1, LL_USART_DMA_REG_DATA_TRANSMIT);
    /* Default settings - updated prior start of transmission */
    DMA_InitStruct.SrcAddress = 0x00000000U;
    DMA_InitStruct.BlkDataLength = 0x00000000U;
    LL_DMA_Init(GPDMA1, LL_DMA_CHANNEL_1, &DMA_InitStruct);

    /* Enable TC interrupt */
    LL_DMA_EnableIT_TC(GPDMA1, LL_DMA_CHANNEL_1);

    /* GPDMA1 interrupt Init */
    NVIC_SetPriority(GPDMA1_Channel1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
    NVIC_EnableIRQ(GPDMA1_Channel1_IRQn);

    /*
     * Configure linked list structure for GPDMA CH0 - used for USART RX operation
     *
     * - Linked-List mode for circular operation
     * - Source address fixed (USART RXD)
     * - Destination address increased after every received byte
     * - Destination address set to start of raw buffer
     * - Length set to raw buffer size
     * - Destination uses PORT1 - fast port to access RAM through bus matrix
     * - Source uses PORT0 - fast port for APB access bypassing bus matrix
     * 
     * Have a look at AN5593 for GPDMA1 use case.
     * https://www.st.com/resource/en/application_note/an5593-how-to-use-the-gpdma-for-stm32u575585-microcontrollers-stmicroelectronics.pdf 
     */
    NodeConfig.DestAllocatedPort = LL_DMA_DEST_ALLOCATED_PORT1;
    NodeConfig.DestHWordExchange = LL_DMA_DEST_HALFWORD_PRESERVE;
    NodeConfig.DestByteExchange = LL_DMA_DEST_BYTE_PRESERVE;
    NodeConfig.DestBurstLength = 1;
    NodeConfig.DestIncMode = LL_DMA_DEST_INCREMENT;
    NodeConfig.DestDataWidth = LL_DMA_DEST_DATAWIDTH_BYTE;
    NodeConfig.SrcAllocatedPort = LL_DMA_SRC_ALLOCATED_PORT0;
    NodeConfig.SrcByteExchange = LL_DMA_SRC_BYTE_PRESERVE;
    NodeConfig.DataAlignment = LL_DMA_DATA_ALIGN_ZEROPADD;
    NodeConfig.SrcBurstLength = 1;
    NodeConfig.SrcIncMode = LL_DMA_SRC_FIXED;
    NodeConfig.SrcDataWidth = LL_DMA_SRC_DATAWIDTH_BYTE;
    NodeConfig.TransferEventMode = LL_DMA_TCEM_BLK_TRANSFER;
    NodeConfig.TriggerPolarity = LL_DMA_TRIG_POLARITY_MASKED;
    NodeConfig.BlkHWRequest = LL_DMA_HWREQUEST_SINGLEBURST;
    NodeConfig.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
    NodeConfig.Request = LL_GPDMA1_REQUEST_USART1_RX;
    NodeConfig.UpdateRegisters = (LL_DMA_UPDATE_CTR1 | LL_DMA_UPDATE_CTR2 | LL_DMA_UPDATE_CBR1 | LL_DMA_UPDATE_CSAR
                                  | LL_DMA_UPDATE_CDAR | LL_DMA_UPDATE_CTR3 | LL_DMA_UPDATE_CBR2 | LL_DMA_UPDATE_CLLR);
    NodeConfig.NodeType = LL_DMA_GPDMA_LINEAR_NODE;
    /* Additional settings */
    NodeConfig.SrcAddress = LL_USART_DMA_GetRegAddr(USART1, LL_USART_DMA_REG_DATA_RECEIVE);
    NodeConfig.DestAddress = (uint32_t)usart_rx_dma_buffer;
    /* Size is always in bytes! Width is determined by source and destination numbers */
    NodeConfig.BlkDataLength = sizeof(usart_rx_dma_buffer);
    LL_DMA_CreateLinkNode(&NodeConfig, &Node_GPDMA1_Channel0);

    /* Connect node to next node = to itself to achieve circular mode with one configuration */
    LL_DMA_ConnectLinkNode(&Node_GPDMA1_Channel0, LL_DMA_CLLR_OFFSET5, &Node_GPDMA1_Channel0, LL_DMA_CLLR_OFFSET5);

    /* 
     * Set first linked list address to DMA channel
     *
     * Set link update mechanism - DMA fetches first configuration from first node
     * on the start of DMA operation
     */
    LL_DMA_SetLinkedListBaseAddr(GPDMA1, LL_DMA_CHANNEL_0, (uint32_t)&Node_GPDMA1_Channel0);
    LL_DMA_ConfigLinkUpdate(GPDMA1, LL_DMA_CHANNEL_0,
                            (LL_DMA_UPDATE_CTR1 | LL_DMA_UPDATE_CTR2 | LL_DMA_UPDATE_CBR1 | LL_DMA_UPDATE_CSAR
                             | LL_DMA_UPDATE_CDAR | LL_DMA_UPDATE_CTR3 | LL_DMA_UPDATE_CBR2 | LL_DMA_UPDATE_CLLR),
                            (uint32_t)&Node_GPDMA1_Channel0);

    /* Initialize linked list general setup for GPDMA CH0 - the way transfers are done */
    DMA_InitLinkedListStruct.Priority = LL_DMA_LOW_PRIORITY_LOW_WEIGHT;
    DMA_InitLinkedListStruct.LinkStepMode = LL_DMA_LSM_FULL_EXECUTION;
    DMA_InitLinkedListStruct.LinkAllocatedPort = LL_DMA_LINK_ALLOCATED_PORT0;
    DMA_InitLinkedListStruct.TransferEventMode = LL_DMA_TCEM_LAST_LLITEM_TRANSFER;
    LL_DMA_List_Init(GPDMA1, LL_DMA_CHANNEL_0, &DMA_InitLinkedListStruct);

    /* Enable HT&TC interrupt */
    LL_DMA_EnableIT_HT(GPDMA1, LL_DMA_CHANNEL_0);
    LL_DMA_EnableIT_TC(GPDMA1, LL_DMA_CHANNEL_0);

    /* Enable DMA interrupts */
    NVIC_SetPriority(GPDMA1_Channel0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
    NVIC_EnableIRQ(GPDMA1_Channel0_IRQn);

    /* Enable USART interrupts */
    NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
    NVIC_EnableIRQ(USART1_IRQn);

    /* USART settings */
    USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
    USART_InitStruct.BaudRate = 115200;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
    LL_USART_Init(USART1, &USART_InitStruct);
    LL_USART_SetTXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
    LL_USART_SetRXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
    LL_USART_DisableFIFO(USART1);
    LL_USART_ConfigAsyncMode(USART1);

    /* Enable UART DMA requests */
    LL_USART_EnableDMAReq_RX(USART1);
    LL_USART_EnableDMAReq_TX(USART1);

    /* Enable UART IDLE line interrupt */
    LL_USART_EnableIT_IDLE(USART1);

    /* Start UART and enable DMA channel for RX */
    LL_USART_Enable(USART1);
    LL_DMA_EnableChannel(GPDMA1, LL_DMA_CHANNEL_0);

    /* TX DMA is started prior every transmission */
}

/**
 * \brief           GPDMA1 channel0 interrupt handler for USART1 RX
 */
void
GPDMA1_Channel0_IRQHandler(void) {
    /* Check for half-transfer interrupt */
    if (LL_DMA_IsEnabledIT_HT(GPDMA1, LL_DMA_CHANNEL_0) && LL_DMA_IsActiveFlag_HT(GPDMA1, LL_DMA_CHANNEL_0)) {
        LL_DMA_ClearFlag_HT(GPDMA1, LL_DMA_CHANNEL_0);
        usart_rx_check(); /* Check for data to process */
    }

    /* Check for transfer-complete interrupt */
    if (LL_DMA_IsEnabledIT_TC(GPDMA1, LL_DMA_CHANNEL_0) && LL_DMA_IsActiveFlag_TC(GPDMA1, LL_DMA_CHANNEL_0)) {
        LL_DMA_ClearFlag_TC(GPDMA1, LL_DMA_CHANNEL_0);
        usart_rx_check(); /* Check for data to process */
    }

    /* Implement other events when needed */
}

/**
 * \brief           GPDMA1 channel1 interrupt handler for USART1 TX
 */
void
GPDMA1_Channel1_IRQHandler(void) {
    /* Check transfer complete interrupt */
    if (LL_DMA_IsEnabledIT_TC(GPDMA1, LL_DMA_CHANNEL_1) && LL_DMA_IsActiveFlag_TC(GPDMA1, LL_DMA_CHANNEL_1)) {
        LL_DMA_ClearFlag_TC(GPDMA1, LL_DMA_CHANNEL_1);

        /* Skip data in memory - mark it sent */
        lwrb_skip(&usart_tx_rb, usart_tx_dma_current_len);
        usart_tx_dma_current_len = 0;
        usart_start_tx_dma_transfer();
    }

    /* Implement other events when needed */
}

/**
 * \brief           USART1 global interrupt handler
 */
void
USART1_IRQHandler(void) {
    /* Check for IDLE line interrupt */
    if (LL_USART_IsActiveFlag_IDLE(USART1)) {
        LL_USART_ClearFlag_IDLE(USART1); /* Clear IDLE flag */
        usart_rx_check();                /* Check for data to process */
    }

    /* Implement other events when needed */
}

/**
 * \brief           System Clock Configuration
 */
void
SystemClock_Config(void) {
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
    while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_4) {}

    /* Set regulator voltage */
    LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);

    /* Enable MSIS */
    LL_RCC_MSIS_Enable();
    while (LL_RCC_MSIS_IsReady() != 1) {}

    /* Configure MSI & PLL oscillators */
    LL_RCC_MSI_EnableRangeSelection();
    LL_RCC_MSIS_SetRange(LL_RCC_MSISRANGE_0);
    LL_RCC_MSI_SetCalibTrimming(16, LL_RCC_MSI_OSCILLATOR_0);
    LL_RCC_PLL1_ConfigDomain_SYS(LL_RCC_PLL1SOURCE_MSIS, 3, 10, 1);
    LL_RCC_PLL1_EnableDomain_SYS();
    LL_RCC_SetPll1EPodPrescaler(LL_RCC_PLL1MBOOST_DIV_4);
    LL_RCC_PLL1_SetVCOInputRange(LL_RCC_PLLINPUTRANGE_8_16);
    LL_RCC_PLL1_Enable();

    /* Wait till PLL is ready */
    while (!LL_RCC_PLL1_IsReady()) {}

    /* Intermediate AHB prescaler 2 when target frequency clock is higher than 80 MHz */
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_2);
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL1);

    /* Wait till System clock is ready */
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL1) {}

    /* Insure 1�s transition state at intermediate medium speed clock based on DWT*/
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;
    while (DWT->CYCCNT < 100) {}

    /* Set prescalers */
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
    LL_RCC_SetAPB3Prescaler(LL_RCC_APB3_DIV_1);

    LL_Init1msTick(160000000);
    LL_SetSystemCoreClock(160000000);
}

/**
 * \brief           Power Configuration
 */
static void
SystemPower_Config(void) {
    /* Disable the internal Pull-Up in Dead Battery pins of UCPD peripheral */
    LL_PWR_DisableUCPDDeadBattery();

    /* Switch to SMPS regulator instead of LDO */
    LL_PWR_SetRegulatorSupply(LL_PWR_SMPS_SUPPLY);
    while (!LL_PWR_IsActiveFlag_REGULATOR()) {}
}
