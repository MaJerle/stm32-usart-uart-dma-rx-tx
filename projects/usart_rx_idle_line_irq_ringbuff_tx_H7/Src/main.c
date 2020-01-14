/*
 * This example shows how can application implement RX and TX DMA for UART.
 * It uses simple packet example approach and 3 separate buffers:
 *
 * - Raw DMA RX buffer where DMA transfers data from UART to memory
 * - Ringbuff for RX data which are processed by application
 * - Ringbuff for TX data to send using TX DMA
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "ringbuff/ringbuff.h"

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USART related functions */
void usart_init(void);
void usart_rx_check(void);
void usart_process_data(const void* data, size_t len);
void usart_send_string(const char* str);
uint8_t usart_start_tx_dma_transfer(void);

/**
 * \brief           Calculate length of statically allocated array
 */
#define ARRAY_LEN(x)            (sizeof(x) / sizeof((x)[0]))

/**
 * \brief           Buffer for USART DMA RX
 * \note            Contains RAW unprocessed data received by UART and transfered by DMA
 */
uint8_t usart_rx_dma_buffer[64];

/**
 * \brief           Create ring buffer for received data
 */
static ringbuff_t
usart_rx_dma_ringbuff;

/**
 * \brief           Ring buffer data array for RX DMA
 */
static uint8_t
usart_rx_dma_ringbuff_data[128];

/**
 * \brief           Create ring buffer for TX DMA
 */
static ringbuff_t
usart_tx_dma_ringbuff;

/**
 * \brief           Ring buffer data array for TX DMA
 */
static uint8_t
usart_tx_dma_ringbuff_data[128];

/**
 * \brief           Length of TX DMA transfer
 */
static size_t
usart_tx_dma_current_len;

/**
 * \brief           Application entry point
 */
int
main(void) {
    uint8_t state, cmd, len;

    /* MCU Configuration--------------------------------------------------------*/
//    SCB_DisableDCache();
//    SCB_DisableICache();

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    LL_APB4_GRP1_EnableClock(LL_APB4_GRP1_PERIPH_SYSCFG);

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize ringbuff for TX & RX */
    ringbuff_init(&usart_tx_dma_ringbuff, usart_tx_dma_ringbuff_data, sizeof(usart_tx_dma_ringbuff_data));
    ringbuff_init(&usart_rx_dma_ringbuff, usart_rx_dma_ringbuff_data, sizeof(usart_rx_dma_ringbuff_data));

    /* Initialize all configured peripherals */
    usart_init();
    usart_send_string("USART DMA example: DMA HT & TC + USART IDLE LINE interrupts\r\n");
    usart_send_string("Start sending data to STM32\r\n");

    /* After this point, do not use usart_send_string function anymore */
    /* Send packet data over UART from PC (or other STM32 device) */

    /* Infinite loop */
    state = 0;
    while (1) {
        uint8_t b;

        /* Process RX ringbuffer */

        /* Packet format: START_BYTE, CMD, LEN[, DATA[0], DATA[len - 1]], STOP BYTE */
        /* DATA bytes are included only if LEN > 0 */
        /* An example, send sequence of these bytes: 0x55, 0x01, 0x01, 0xFF, 0xAA */

        /* Read byte by byte */

        if (ringbuff_read(&usart_rx_dma_ringbuff, &b, 1) == 1) {
            ringbuff_write(&usart_tx_dma_ringbuff, &b, 1);   /* Write data to transmit buffer */
            usart_start_tx_dma_transfer();
            switch (state) {
                case 0: {           /* Wait for start byte */
                    if (b == 0x55) {
                        ++state;
                    }
                    break;
                }
                case 1: {           /* Check packet command */
                    cmd = b;
                    ++state;
                    break;
                }
                case 2: {           /* Packet data length */
                    len = b;
                    ++state;
                    if (len == 0) {
                        ++state;    /* Ignore data part if len = 0 */
                    }
                    break;
                }
                case 3: {           /* Data for command */
                    --len;          /* Decrease for received character */
                    if (len == 0) {
                        ++state;
                    }
                    break;
                }
                case 4: {           /* End of packet */
                    if (b == 0xAA) {
                        /* Packet is valid */

                        /* Send out response with CMD = 0xFF */
                        b = 0x55;   /* Start byte */
                        ringbuff_write(&usart_tx_dma_ringbuff, &b, 1);
                        cmd = 0xFF; /* Command = 0xFF = OK response */
                        ringbuff_write(&usart_tx_dma_ringbuff, &cmd, 1);
                        b = 0x00;   /* Len = 0 */
                        ringbuff_write(&usart_tx_dma_ringbuff, &b, 1);
                        b = 0xAA;   /* Stop byte */
                        ringbuff_write(&usart_tx_dma_ringbuff, &b, 1);

                        /* Flush everything */
                        usart_start_tx_dma_transfer();
                    }
                    state = 0;
                    break;
                }
            }
        }

        /* Do other tasks ... */
    }
}

/**
 * \brief           Check for new data received with DMA
 */
void
usart_rx_check(void) {
    static size_t old_pos;
    size_t pos;

    /* Calculate current position in buffer */
    pos = ARRAY_LEN(usart_rx_dma_buffer) - LL_DMA_GetDataLength(DMA1, LL_DMA_STREAM_0);
    if (pos != old_pos) {                       /* Check change in received data */
        if (pos > old_pos) {                    /* Current position is over previous one */
            /* We are in "linear" mode */
            /* Process data directly by subtracting "pointers" */
            usart_process_data(&usart_rx_dma_buffer[old_pos], pos - old_pos);
        } else {
            /* We are in "overflow" mode */
            /* First process data to the end of buffer */
            usart_process_data(&usart_rx_dma_buffer[old_pos], ARRAY_LEN(usart_rx_dma_buffer) - old_pos);
            /* Check and continue with beginning of buffer */
            if (pos > 0) {
                usart_process_data(&usart_rx_dma_buffer[0], pos);
            }
        }
    }
    old_pos = pos;                              /* Save current position as old */

    /* Check and manually update if we reached end of buffer */
    if (old_pos == ARRAY_LEN(usart_rx_dma_buffer)) {
        old_pos = 0;
    }
}

/**
 * \brief           Check if DMA is active and if not try to send data
 */
uint8_t
usart_start_tx_dma_transfer(void) {
    uint32_t old_primask;
    uint8_t started = 0;

    /* Check if DMA is active */
    /* Must be set to 0 */
    old_primask = __get_PRIMASK();
    __disable_irq();

    /* Check if transfer is not active */
    if (usart_tx_dma_current_len == 0) {
        /* Check if something to send  */
        usart_tx_dma_current_len = ringbuff_get_linear_block_read_length(&usart_tx_dma_ringbuff);
        if (usart_tx_dma_current_len > 0) {
            /* Disable channel if enabled */
            LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_1);

            /* Clear all flags */
            LL_DMA_ClearFlag_TC1(DMA1);
            LL_DMA_ClearFlag_HT1(DMA1);
            LL_DMA_ClearFlag_TE1(DMA1);
            LL_DMA_ClearFlag_DME1(DMA1);
            LL_DMA_ClearFlag_FE1(DMA1);

            /* Start DMA transfer */
            LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_1, usart_tx_dma_current_len);
            LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_1, (uint32_t)ringbuff_get_linear_block_read_address(&usart_tx_dma_ringbuff));

            /* Start new transfer */
            LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_1);
            started = 1;
        }
    }

    __set_PRIMASK(old_primask);
    return started;
}

/**
 * \brief           Process received data over UART
 * Data are written to RX ringbuffer for application processing at latter stage
 * \param[in]       data: Data to process
 * \param[in]       len: Length in units of bytes
 */
void
usart_process_data(const void* data, size_t len) {
    ringbuff_write(&usart_rx_dma_ringbuff, data, len);  /* Write data to receive buffer */
}

/**
 * \brief           Send string over USART
 * \param[in]       str: String to send
 */
void
usart_send_string(const char* str) {
    ringbuff_write(&usart_tx_dma_ringbuff, str, strlen(str));   /* Write data to transmit buffer */
    usart_start_tx_dma_transfer();
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
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOD);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

    /*
     * USART3 GPIO Configuration
     *
     * PD8   ------> USART3_TX
     * PD9   ------> USART3_RX
     */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_8 | LL_GPIO_PIN_9;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
    LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* USART3_RX Init */
    LL_DMA_SetPeriphRequest(DMA1, LL_DMA_STREAM_0, LL_DMAMUX1_REQ_USART3_RX);
    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_0, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_0, LL_DMA_PRIORITY_LOW);
    LL_DMA_SetMode(DMA1, LL_DMA_STREAM_0, LL_DMA_MODE_CIRCULAR);
    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_0, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_0, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_0, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_0, LL_DMA_MDATAALIGN_BYTE);
    LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_0);
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_0, LL_USART_DMA_GetRegAddr(USART3, LL_USART_DMA_REG_DATA_RECEIVE));
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_0, (uint32_t)usart_rx_dma_buffer);
    LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_0, ARRAY_LEN(usart_rx_dma_buffer));

    /* USART3_TX Init */
    LL_DMA_SetPeriphRequest(DMA1, LL_DMA_STREAM_1, LL_DMAMUX1_REQ_USART3_TX);
    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_1, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_1, LL_DMA_PRIORITY_LOW);
    LL_DMA_SetMode(DMA1, LL_DMA_STREAM_1, LL_DMA_MODE_NORMAL);
    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_1, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_1, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_1, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_1, LL_DMA_MDATAALIGN_BYTE);
    LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_1);
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_1, LL_USART_DMA_GetRegAddr(USART3, LL_USART_DMA_REG_DATA_TRANSMIT));

    /* Enable DMA RX HT & TC interrupts */
    LL_DMA_EnableIT_HT(DMA1, LL_DMA_STREAM_0);
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_0);
    /* Enable DMA TX TC interrupts */
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_1);

    /* DMA interrupt init */
    NVIC_SetPriority(DMA1_Stream0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(DMA1_Stream0_IRQn);
    NVIC_SetPriority(DMA1_Stream1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(DMA1_Stream1_IRQn);

    /* Configure USART3 */
    USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
    USART_InitStruct.BaudRate = 115200;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
    LL_USART_Init(USART3, &USART_InitStruct);
    LL_USART_SetTXFIFOThreshold(USART3, LL_USART_FIFOTHRESHOLD_7_8);
    LL_USART_SetRXFIFOThreshold(USART3, LL_USART_FIFOTHRESHOLD_7_8);
    LL_USART_EnableFIFO(USART3);
    LL_USART_ConfigAsyncMode(USART3);
    LL_USART_EnableDMAReq_RX(USART3);
    LL_USART_EnableDMAReq_TX(USART3);
    LL_USART_EnableIT_IDLE(USART3);

    /* USART interrupt, same priority as DMA channel */
    NVIC_SetPriority(USART3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(USART3_IRQn);

    /* Enable USART and DMA RX */
    LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_0);
    LL_USART_Enable(USART3);

    /* Polling USART3 initialization */
    while (!LL_USART_IsActiveFlag_TEACK(USART3) || !LL_USART_IsActiveFlag_REACK(USART3)) {}
}

/* Interrupt handlers here */

/**
 * \brief           DMA1 stream1 interrupt handler for USART3 RX
 */
void
DMA1_Stream0_IRQHandler(void) {
    /* Check half-transfer complete interrupt */
    if (LL_DMA_IsEnabledIT_HT(DMA1, LL_DMA_STREAM_0) && LL_DMA_IsActiveFlag_HT0(DMA1)) {
        LL_DMA_ClearFlag_HT0(DMA1);             /* Clear half-transfer complete flag */
        usart_rx_check();                       /* Check for data to process */
    }

    /* Check transfer-complete interrupt */
    if (LL_DMA_IsEnabledIT_TC(DMA1, LL_DMA_STREAM_0) && LL_DMA_IsActiveFlag_TC0(DMA1)) {
        LL_DMA_ClearFlag_TC0(DMA1);             /* Clear transfer complete flag */
        usart_rx_check();                       /* Check for data to process */
    }

    /* Implement other events when needed */
}

/**
 * \brief           DMA1 stream1 interrupt handler for USART3 TX
 */
void
DMA1_Stream1_IRQHandler(void) {
    /* Check transfer complete */
    if (LL_DMA_IsEnabledIT_TC(DMA1, LL_DMA_STREAM_1) && LL_DMA_IsActiveFlag_TC1(DMA1)) {
        LL_DMA_ClearFlag_TC1(DMA1);             /* Clear transfer complete flag */
        ringbuff_skip(&usart_tx_dma_ringbuff, usart_tx_dma_current_len);/* Skip sent data, mark as read */
        usart_tx_dma_current_len = 0;           /* Clear length variable */
        usart_start_tx_dma_transfer();          /* Start sending more data */
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
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
    if (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_4) {
        while (1) {}
    }

    /* Configure power supply and voltage scale */
    LL_PWR_ConfigSupply(LL_PWR_LDO_SUPPLY);
    LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE0);

    /* Uncomment if used on STM32H745/H755 Nucleo */
    /* Dual-Core Nucleo board used external SMPS instead of LDO */
    /* Manually enable it */
    //PWR->CR3 |= 1 << 2;

    /* Configure HSE */
    LL_RCC_HSE_EnableBypass();
    LL_RCC_HSE_Enable();
    while (!LL_RCC_HSE_IsReady()) {}

    /* Configure PLL */
    LL_RCC_PLL_SetSource(LL_RCC_PLLSOURCE_HSE);
    LL_RCC_PLL1P_Enable();
    LL_RCC_PLL1Q_Enable();
    LL_RCC_PLL1_SetVCOInputRange(LL_RCC_PLLINPUTRANGE_8_16);
    LL_RCC_PLL1_SetVCOOutputRange(LL_RCC_PLLVCORANGE_WIDE);
    LL_RCC_PLL1_SetM(1);
    LL_RCC_PLL1_SetN(120);
    LL_RCC_PLL1_SetP(2);
    LL_RCC_PLL1_SetQ(20);
    LL_RCC_PLL1_SetR(2);
    LL_RCC_PLL1_Enable();
    while (!LL_RCC_PLL1_IsReady()) {}

    /* Intermediate AHB prescaler 2 when target frequency clock is higher than 80 MHz */
    LL_RCC_SetAHBPrescaler(LL_RCC_AHB_DIV_2);
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL1);
    LL_RCC_SetSysPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAHBPrescaler(LL_RCC_AHB_DIV_2);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
    LL_RCC_SetAPB3Prescaler(LL_RCC_APB3_DIV_2);
    LL_RCC_SetAPB4Prescaler(LL_RCC_APB4_DIV_2);

    /* Configure systick */
    LL_Init1msTick(480000000);
    LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);
    LL_SetSystemCoreClock(480000000);
    LL_RCC_SetUSARTClockSource(LL_RCC_USART234578_CLKSOURCE_PCLK1);
}
