/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "cmsis_os.h"
#include "cpu_utils.h"
#include "lwrb/lwrb.h"

/*
 * Set this to `1` to enable DMA for TX for UART
 * Set this to `0` to send data using polling mode
 *
 * Observe output on UART and its CPU load
 */
#define USE_DMA_TX              0

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USART related functions */
void    usart_init(void);
uint8_t usart_send_string(const char* str);
void    usart_start_tx_dma(void);

/**
 * \brief           Calculate length of statically allocated array
 */
#define ARRAY_LEN(x)            (sizeof(x) / sizeof((x)[0]))

/* Thread function entry point */
void init_thread(void* arg);

/* Buffer for data before transmitted over DMA */
static char buff[256];

/* Ring buffer for TX data */
static lwrb_t usart_tx_buff;
static uint8_t usart_tx_buff_data[1024];
static size_t usart_dma_tx_len;

/**
 * \brief           Long text to be transmitted
 */
static const char
long_string[] = ""
"Miusov, as a man man of breeding and deilcacy,"
"could not but feel some inwrd qualms, when he reached the Father Superior's with Ivan:"
"he felt ashamed of havin lost his temper."
"He felt that he ought to have disdaimed that despicable wretch,"
"Fyodor Pavlovitch, too much to have been upset by him in Father Zossima's cell,"
"and so to have forgotten himself. \"Teh monks were not to blame,"
"in any case,\" he reflected, on the steps."
"\"And if they\'re decent people here (and the Father Superior,"
"I understand, is a nobleman) why not be friendly and courteous withthem?"
"I won't argue, I'll fall in with everything, I'll win them by politness,"
"and show them that I've nothing to do with that Aesop, thta buffoon,"
"that Pierrot, and have merely been takken in over this affair, just as they have.\r\n"
"";

/**
 * \brief           Application entry point
 */
int
main(void) {
    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
    NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize ringbuff */
    lwrb_init(&usart_tx_buff, usart_tx_buff_data, sizeof(usart_tx_buff_data));

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
    uint32_t ticks;
    uint16_t cpu_usage;

    /* Initialize all configured peripherals */
    usart_init();

    /* Do other initializations if needed */

    /* Main application loop */
    ticks = osKernelGetTickCount();
    while (1) {
        /* Send test data */
        usart_send_string("\r\n-----\r\n");
        usart_send_string(long_string);
        usart_send_string(long_string);
        usart_send_string("\r\n-----\r\n");

        /* Thread must be executed maximum once per second */
        ticks += 1000;
        osDelayUntil(ticks);

        cpu_usage = osGetCPUUsage();            /* Get CPU load */
        sprintf(buff, "CPU Load: %d%%\r\n", (int)cpu_usage);
        usart_send_string(buff);
    }

    __get_IPSR();
}

/**
 * \brief           Send debug string over UART
 * \param[in]       str: String to send
 */
uint8_t
usart_send_string(const char* str) {
    size_t len = strlen(str);
    uint8_t ret = 0;

#if USE_DMA_TX
    if (lwrb_get_free(&usart_tx_buff) >= len) {
        lwrb_write(&usart_tx_buff, str, len);
        usart_start_tx_dma();
        ret = 1;
    }
#else /* USE_DMA_TX */
    for (; len > 0; --len, ++str) {
        LL_USART_TransmitData8(USART3, *str);
        while (!LL_USART_IsActiveFlag_TXE(USART3)) {}
    }
    while (!LL_USART_IsActiveFlag_TC(USART3)) {}
    return 1;
#endif /* !USE_DMA_TX */

    return ret;
}

/**
 * \brief           Checks for data in buffer and starts transfer if not in progress
 * \note            It disables interrupts to prevent race condition
 */
void
usart_start_tx_dma(void) {
    uint32_t primask;

    /* Check if transfer active */
    if (usart_dma_tx_len > 0) {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    /* If transfer is not on-going */
    if (usart_dma_tx_len == 0
            && (usart_dma_tx_len = lwrb_get_linear_block_read_length(&usart_tx_buff)) > 0) {
        /* Limit maximal size to transmit at a time */
        if (usart_dma_tx_len > 32) {
            usart_dma_tx_len = 32;
        }

        /* Configure DMA */
        LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_3, usart_dma_tx_len);
        LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_3, (uint32_t)lwrb_get_linear_block_read_address(&usart_tx_buff));

        /* Clear all flags */
        LL_DMA_ClearFlag_TC3(DMA1);
        LL_DMA_ClearFlag_HT3(DMA1);
        LL_DMA_ClearFlag_DME3(DMA1);
        LL_DMA_ClearFlag_FE3(DMA1);
        LL_DMA_ClearFlag_TE3(DMA1);

        /* Start transfer */
        LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_3);
    }

    __set_PRIMASK(primask);
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
    LL_DMA_SetChannelSelection(DMA1, LL_DMA_STREAM_3, LL_DMA_CHANNEL_4);
    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_3, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_3, LL_DMA_PRIORITY_LOW);
    LL_DMA_SetMode(DMA1, LL_DMA_STREAM_3, LL_DMA_MODE_NORMAL);
    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_3, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_3, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_3, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_3, LL_DMA_MDATAALIGN_BYTE);
    LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_3);

    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_3, (uint32_t)&USART3->DR);

    /* Enable TC interrupt */
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_3);

    /* DMA interrupt init */
    NVIC_SetPriority(DMA1_Stream3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
    NVIC_EnableIRQ(DMA1_Stream3_IRQn);

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
    LL_USART_EnableDMAReq_TX(USART3);

    /* USART interrupt */
    NVIC_SetPriority(USART3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 1));
    NVIC_EnableIRQ(USART3_IRQn);

    /* Enable USART */
    LL_USART_Enable(USART3);
}

/* Interrupt handlers here */

/**
 * \brief           DMA1 stream1 interrupt handler for USART3 TX
 */
void
DMA1_Stream3_IRQHandler(void) {
    /* Check transfer-complete interrupt */
    if (LL_DMA_IsEnabledIT_TC(DMA1, LL_DMA_STREAM_3) && LL_DMA_IsActiveFlag_TC3(DMA1)) {
        LL_DMA_ClearFlag_TC3(DMA1);             /* Clear transfer complete flag */
        lwrb_skip(&usart_tx_buff, usart_dma_tx_len);/* Data sent, ignore these */
        usart_dma_tx_len = 0;
        usart_start_tx_dma();                   /* Try to send more data */
    }

    /* Implement other events when needed */
}

/**
 * \brief           USART3 global interrupt handler
 */
void
USART3_IRQHandler(void) {
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
