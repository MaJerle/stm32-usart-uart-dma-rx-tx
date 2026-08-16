/**
 * \brief           DMA + UART example for STM32C5xx devices
 * \license         MIT
 * 
 * This example runs on one of the 3 boards:
 * 
 * - NUCLEO-C562RE with STM32C562RET6 MCU
 * - NUCLEO-C542RC with STM32C542RCT6 MCU
 * - NUCLEO-C5A3ZG with STM32C5A3ZGT6 MCU
 * 
 * This project does not explain all the concepts, please follow
 * repository documentation or open other projects (like STM32U5 example) 
 * for full understanding of the key principle
 */
#include "lwrb/lwrb.h"
#include "stm32c5xx.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/* Include drivers */
#include "stm32c5xx_ll_bus.h"
#include "stm32c5xx_ll_dma.h"
#include "stm32c5xx_ll_flash.h"
#include "stm32c5xx_ll_gpio.h"
#include "stm32c5xx_ll_icache.h"
#include "stm32c5xx_ll_pwr.h"
#include "stm32c5xx_ll_rcc.h"
#include "stm32c5xx_ll_system.h"
#include "stm32c5xx_ll_usart.h"
#include "stm32c5xx_ll_utils.h"

/* Define the USART variables */
#if NUCLEO_C562RE > 0
#define USART_INSTANCE            USART2
#define USART_INSTANCE_IRQn       USART2_IRQn
#define USART_INSTANCE_IRQHandler USART2_IRQHandler
#define USART_TX_PORT             GPIOA
#define USART_TX_PIN              LL_GPIO_PIN_2
#define USART_TX_AF               LL_GPIO_AF_7
#define USART_RX_PORT             GPIOA
#define USART_RX_PIN              LL_GPIO_PIN_3
#define USART_RX_AF               LL_GPIO_AF_7
#define USART_TX_DMA_REQUEST      LL_LPDMA1_REQUEST_USART2_TX
#define USART_RX_DMA_REQUEST      LL_LPDMA1_REQUEST_USART2_RX
#elif NUCLEO_C542RC
#define USART_INSTANCE            USART2
#define USART_INSTANCE_IRQn       USART2_IRQn
#define USART_INSTANCE_IRQHandler USART2_IRQHandler
#define USART_TX_PORT             GPIOA
#define USART_TX_PIN              LL_GPIO_PIN_2
#define USART_TX_AF               LL_GPIO_AF_7
#define USART_RX_PORT             GPIOA
#define USART_RX_PIN              LL_GPIO_PIN_3
#define USART_RX_AF               LL_GPIO_AF_7
#define USART_TX_DMA_REQUEST      LL_LPDMA1_REQUEST_USART2_TX
#define USART_RX_DMA_REQUEST      LL_LPDMA1_REQUEST_USART2_RX
#elif NUCLEO_C5A3ZG
#define USART_INSTANCE            USART2
#define USART_INSTANCE_IRQn       USART2_IRQn
#define USART_INSTANCE_IRQHandler USART2_IRQHandler
#define USART_TX_PORT             GPIOA
#define USART_TX_PIN              LL_GPIO_PIN_2
#define USART_TX_AF               LL_GPIO_AF_7
#define USART_RX_PORT             GPIOA
#define USART_RX_PIN              LL_GPIO_PIN_3
#define USART_RX_AF               LL_GPIO_AF_7
#define USART_TX_DMA_REQUEST      LL_LPDMA1_REQUEST_USART2_TX
#define USART_RX_DMA_REQUEST      LL_LPDMA1_REQUEST_USART2_RX
#else
#error "Invalid board"
#endif

void
LL_GPIO_SetAFPin(GPIO_TypeDef* GPIOx, uint32_t pin, uint32_t alternate) {
    if (pin > LL_GPIO_PIN_7) {
        LL_GPIO_SetAFPin_8_15(GPIOx, pin, alternate);
    } else {
        LL_GPIO_SetAFPin_0_7(GPIOx, pin, alternate);
    }
}

/* System private function */
static void systemclock_config(void);

/* USART related functions */
void usart_init(void);
void usart_rx_check(void);
void usart_process_data(const void* data, size_t len);
void usart_send_string(const char* str);
uint8_t usart_start_tx_dma_transfer(void);

/**
 * \brief           Static & global variable for the self-referencing linked-list node
 *
 *                  This is the simple array to store configuration to apply
 *                  when DMA finishes the cycle (reload with new cycle)
 */
static uint32_t Node_LPDMA1_Channel0[LL_DMA_NODE_REGISTER_NUM];

/**
 * \brief           Application context variables
 */
static struct {
    lwrb_t usart_tx_rb;                       /*!< TX DMA buffer */
    uint8_t usart_tx_rb_data[128];            /*!< TX DMA buffer data */
    volatile size_t usart_tx_dma_current_len; /*!< Current TX DMA transfer length */
    uint8_t usart_rx_dma_buffer[64];          /*!< Circular DMA receive buffer */
} ctx;

/**
 * \brief           The application entry point
 */
int
main(void) {
    /* System interrupt init */
    NVIC_SetPriorityGrouping(3);
    LL_AHB1_EnableBusClock();

    /* Configure the system clock and power */
    systemclock_config();

    /* Enable instruction cache */

    /* Initialize ringbuff */
    lwrb_init(&ctx.usart_tx_rb, ctx.usart_tx_rb_data, sizeof(ctx.usart_tx_rb_data));

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
 * \brief           USART check on the interrupts
 */
void
usart_rx_check(void) {
    static size_t old_pos;
    size_t pos;

    /* Calculate current position in buffer and check for new data available */
    pos = sizeof(ctx.usart_rx_dma_buffer) - LL_DMA_GetBlkDataLength(LPDMA1_CH0);
    if (pos != old_pos) {
        if (pos > old_pos) {
            usart_process_data(&ctx.usart_rx_dma_buffer[old_pos], pos - old_pos);
        } else {
            usart_process_data(&ctx.usart_rx_dma_buffer[old_pos], sizeof(ctx.usart_rx_dma_buffer) - old_pos);
            if (pos > 0) {
                usart_process_data(&ctx.usart_rx_dma_buffer[0], pos);
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

    primask = __get_PRIMASK();
    __disable_irq();
    if (ctx.usart_tx_dma_current_len == 0
        && (ctx.usart_tx_dma_current_len = lwrb_get_linear_block_read_length(&ctx.usart_tx_rb)) > 0) {
        /* Disable channel if enabled, clear all flags, prepare new transfer, and start */
        LL_DMA_DisableChannel(LPDMA1_CH1);

        /* We must clear flags */
        LL_DMA_ClearFlag(LPDMA1_CH1, LL_DMA_FLAG_ALL);

        /* Set new source address and length to transfer */
        LL_DMA_SetBlkDataLength(LPDMA1_CH1, ctx.usart_tx_dma_current_len);
        LL_DMA_SetSrcAddress(LPDMA1_CH1, (uint32_t)lwrb_get_linear_block_read_address(&ctx.usart_tx_rb));

        /* GO */
        LL_DMA_EnableChannel(LPDMA1_CH1);
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
    lwrb_write(&ctx.usart_tx_rb, data, len); /* Write data to TX buffer for loopback */
    usart_start_tx_dma_transfer();           /* Then try to start transfer */
}

/**
 * \brief           Send string to USART
 * \param[in]       str: String to send
 */
void
usart_send_string(const char* str) {
    lwrb_write(&ctx.usart_tx_rb, str, strlen(str)); /* Write data to TX buffer for loopback */
    usart_start_tx_dma_transfer();                  /* Then try to start transfer */
}

/**
 * \brief           USART_INSTANCE Initialization Function
 */
void
usart_init(void) {
    /* Set UART kernel clock */
    LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_PCLK2);
    LL_RCC_SetUSARTClockSource(LL_RCC_USART2_CLKSOURCE_PCLK1);

    /* Peripheral clock enable */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_LPDMA1);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);

    /*
     * USART GPIO configuration
     *
     * USART_TX_PIN ------> USART_INSTANCE_TX
     * USART_RX_PIN ------> USART_INSTANCE_RX
     */
    LL_GPIO_SetPinMode(USART_TX_PORT, USART_TX_PIN, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinSpeed(USART_TX_PORT, USART_TX_PIN, LL_GPIO_SPEED_FREQ_MEDIUM);
    LL_GPIO_SetPinOutputType(USART_TX_PORT, USART_TX_PIN, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinPull(USART_TX_PORT, USART_TX_PIN, LL_GPIO_PULL_NO);
    LL_GPIO_SetAFPin_0_7(USART_TX_PORT, USART_TX_PIN, USART_TX_AF);
    LL_GPIO_SetPinMode(USART_RX_PORT, USART_RX_PIN, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinSpeed(USART_RX_PORT, USART_RX_PIN, LL_GPIO_SPEED_FREQ_MEDIUM);
    LL_GPIO_SetPinOutputType(USART_RX_PORT, USART_RX_PIN, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinPull(USART_RX_PORT, USART_RX_PIN, LL_GPIO_PULL_NO);
    LL_GPIO_SetAFPin_0_7(USART_RX_PORT, USART_RX_PIN, USART_RX_AF);

    /*
     * Configure GPDMA CH1 for USART TX operation in direct (non-linked-list) mode
     *
     * - Direct mode - no linked list operation
     * - Source address increase from memory
     * - Destination address no-increase to peripheral (USART TDR)
     * - Length set before channel enabled
     * - Source address set before TX operation
     *
     * This LL driver (unlike older LL_DMA generations) has no LL_DMA_InitTypeDef/LL_DMA_Init() -
     * every field is configured through its own setter instead.
     */
    LL_DMA_ResetChannel(LPDMA1_CH1);
    LL_DMA_SetDataTransferDirection(LPDMA1_CH1, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    LL_DMA_SetHWRequestMode(LPDMA1_CH1, LL_DMA_HARDWARE_REQUEST_BURST);
    LL_DMA_SetDataAlignment(LPDMA1_CH1, LL_DMA_DEST_DATA_TRUNC_LEFT_PADD_ZERO);
    LL_DMA_SetSrcDataWidth(LPDMA1_CH1, LL_DMA_SRC_DATA_WIDTH_BYTE);
    LL_DMA_SetDestDataWidth(LPDMA1_CH1, LL_DMA_DEST_DATA_WIDTH_BYTE);
    LL_DMA_SetSrcIncMode(LPDMA1_CH1, LL_DMA_SRC_ADDR_INCREMENTED);
    LL_DMA_SetDestIncMode(LPDMA1_CH1, LL_DMA_DEST_ADDR_FIXED);
    LL_DMA_SetChannelPriorityLevel(LPDMA1_CH1, LL_DMA_PRIORITY_LOW_WEIGHT_MID);
    LL_DMA_SetTriggerPolarity(LPDMA1_CH1, LL_DMA_TRIGGER_POLARITY_MASKED); /* Trigger feature unused */
    LL_DMA_SetPeriphRequest(LPDMA1_CH1, USART_TX_DMA_REQUEST);
    LL_DMA_SetTransferEventMode(LPDMA1_CH1, LL_DMA_DIRECT_XFER_EVENT_BLOCK);
    LL_DMA_SetDestAddress(LPDMA1_CH1, LL_USART_DMA_GetRegAddr(USART_INSTANCE, LL_USART_DMA_REG_DATA_TRANSMIT));
    /* Source address & length are set right before every transmission, in usart_start_tx_dma_transfer() */

    /* Enable TC interrupt */
    LL_DMA_EnableIT_TC(LPDMA1_CH1);

    /* LPDMA1 interrupt Init */
    NVIC_SetPriority(LPDMA1_CH1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
    NVIC_EnableIRQ(LPDMA1_CH1_IRQn);

    /*
     * Configure GPDMA CH0 for USART RX operation, in circular mode via a self-referencing linked-list node
     *
     * - Source address fixed (USART RDR)
     * - Destination address increased after every received byte
     * - Destination address set to start of raw buffer
     * - Length set to raw buffer size
     *
     * This LL driver has no node-builder helpers (LL_DMA_CreateLinkNode/ConnectLinkNode) like older LL_DMA
     * generations - a node is just LL_DMA_NODE_REGISTER_NUM raw register words. The approach here:
     *   1) Configure the channel normally, through the same per-field setters as any direct transfer.
     *   2) Read back the live CTR1/CTR2/CBR1/CSAR/CDAR registers into the node array - this captures the
     *      "reset" configuration (full buffer, start address) as the node's content.
     *   3) Make the node's own CLLR word point back to itself with every register's update bit set,
     *      including CLLR's own - so every time the channel finishes the current block, it reloads
     *      this very node and restarts identically, forever (i.e. circular wraparound).
     *
     * Node addressing is split: CLBAR holds the address's top 16 bits (64 KB aligned base address), 
     * while the CLLR holds bottom address (4-bytes aligned) and the associated update flags
     */
    LL_DMA_ResetChannel(LPDMA1_CH0);
    LL_DMA_SetDataTransferDirection(LPDMA1_CH0, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetHWRequestMode(LPDMA1_CH0, LL_DMA_HARDWARE_REQUEST_BURST);
    LL_DMA_SetDataAlignment(LPDMA1_CH0, LL_DMA_DEST_DATA_TRUNC_LEFT_PADD_ZERO);
    LL_DMA_SetSrcDataWidth(LPDMA1_CH0, LL_DMA_SRC_DATA_WIDTH_BYTE);
    LL_DMA_SetDestDataWidth(LPDMA1_CH0, LL_DMA_DEST_DATA_WIDTH_BYTE);
    LL_DMA_SetSrcIncMode(LPDMA1_CH0, LL_DMA_SRC_ADDR_FIXED);
    LL_DMA_SetDestIncMode(LPDMA1_CH0, LL_DMA_DEST_ADDR_INCREMENTED);
    LL_DMA_SetChannelPriorityLevel(LPDMA1_CH0, LL_DMA_PRIORITY_LOW_WEIGHT_LOW);
    LL_DMA_SetTriggerPolarity(LPDMA1_CH0, LL_DMA_TRIGGER_POLARITY_MASKED);
    LL_DMA_SetPeriphRequest(LPDMA1_CH0, USART_RX_DMA_REQUEST);

    /* TC (and HT) event generated at the end (and half) of every linked-list item, i.e. every buffer wrap */
    LL_DMA_SetTransferEventMode(LPDMA1_CH0, LL_DMA_LINKEDLIST_XFER_EVENT_NODE);

    /* Channel keeps auto-executing the linked list (here: itself) without CPU intervention */
    LL_DMA_SetLinkStepMode(LPDMA1_CH0, LL_DMA_LINKEDLIST_EXECUTION_Q);
    LL_DMA_ConfigAddresses(LPDMA1_CH0, LL_USART_DMA_GetRegAddr(USART_INSTANCE, LL_USART_DMA_REG_DATA_RECEIVE),
                           (uint32_t)ctx.usart_rx_dma_buffer);

    /* Size is always in bytes! Width is determined by source and destination data width */
    LL_DMA_SetBlkDataLength(LPDMA1_CH0, sizeof(ctx.usart_rx_dma_buffer));

    /* Capture the just-configured "reset" register values into the node's first 5 words */
    Node_LPDMA1_Channel0[LL_DMA_NODE_CTR1_REG_OFFSET] = LPDMA1_CH0->CTR1;
    Node_LPDMA1_Channel0[LL_DMA_NODE_CTR2_REG_OFFSET] = LPDMA1_CH0->CTR2;
    Node_LPDMA1_Channel0[LL_DMA_NODE_CBR1_REG_OFFSET] = LPDMA1_CH0->CBR1;
    Node_LPDMA1_Channel0[LL_DMA_NODE_CSAR_REG_OFFSET] = LPDMA1_CH0->CSAR;
    Node_LPDMA1_Channel0[LL_DMA_NODE_CDAR_REG_OFFSET] = LPDMA1_CH0->CDAR;

    /* Self-link: node's own CLLR points back at itself, reloading every register (incl. CLLR) each time */
    Node_LPDMA1_Channel0[LL_DMA_NODE_CLLR_REG_OFFSET] = (LL_DMA_UPDATE_CTR1 | LL_DMA_UPDATE_CTR2 | LL_DMA_UPDATE_CBR1
                                                         | LL_DMA_UPDATE_CSAR | LL_DMA_UPDATE_CDAR | LL_DMA_UPDATE_CLLR)
                                                        | ((uint32_t)Node_LPDMA1_Channel0 & DMA_CLLR_LA);

    /* Point the channel itself at the node, so it reloads from it once the current block completes */
    LL_DMA_SetLinkedListBaseAddr(LPDMA1_CH0, (uint32_t)Node_LPDMA1_Channel0 & DMA_CLBAR_LBA);
    LL_DMA_ConfigLinkUpdate(LPDMA1_CH0,
                            (LL_DMA_UPDATE_CTR1 | LL_DMA_UPDATE_CTR2 | LL_DMA_UPDATE_CBR1 | LL_DMA_UPDATE_CSAR
                             | LL_DMA_UPDATE_CDAR | LL_DMA_UPDATE_CLLR),
                            (uint32_t)Node_LPDMA1_Channel0 & DMA_CLLR_LA);

    /* Enable HT&TC interrupt */
    LL_DMA_EnableIT_HT(LPDMA1_CH0);
    LL_DMA_EnableIT_TC(LPDMA1_CH0);

    /* Enable DMA interrupts */
    NVIC_SetPriority(LPDMA1_CH0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
    NVIC_EnableIRQ(LPDMA1_CH0_IRQn);

    /* Enable USART interrupts */
    NVIC_SetPriority(USART_INSTANCE_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
    NVIC_EnableIRQ(USART_INSTANCE_IRQn);

    /* USART settings */
    LL_USART_SetPrescaler(USART_INSTANCE, LL_USART_PRESCALER_DIV1);
    LL_USART_SetOverSampling(USART_INSTANCE, LL_USART_OVERSAMPLING_16);
    LL_USART_SetBaudRate(USART_INSTANCE, SystemCoreClock, LL_USART_GetPrescaler(USART_INSTANCE),
                         LL_USART_GetOverSampling(USART_INSTANCE), 115200U);
    LL_USART_SetDataWidth(USART_INSTANCE, LL_USART_DATAWIDTH_8_BIT);
    LL_USART_SetStopBitsLength(USART_INSTANCE, LL_USART_STOP_BIT_1);
    LL_USART_SetParity(USART_INSTANCE, LL_USART_PARITY_NONE);
    LL_USART_SetTransferDirection(USART_INSTANCE, LL_USART_DIRECTION_TX_RX);
    LL_USART_SetTransferBitOrder(USART_INSTANCE, LL_USART_BITORDER_LSB_FIRST);
    LL_USART_SetHWFlowCtrl(USART_INSTANCE, LL_USART_HWCONTROL_NONE);
    LL_USART_SetTXFIFOThreshold(USART_INSTANCE, LL_USART_FIFO_THRESHOLD_1_8);
    LL_USART_SetRXFIFOThreshold(USART_INSTANCE, LL_USART_FIFO_THRESHOLD_1_8);
    LL_USART_DisableFIFO(USART_INSTANCE);
    LL_USART_ConfigAsyncMode(USART_INSTANCE);

    /* Enable UART DMA requests */
    LL_USART_EnableDMAReq_RX(USART_INSTANCE);
    LL_USART_EnableDMAReq_TX(USART_INSTANCE);

    /* Enable UART IDLE line interrupt */
    LL_USART_EnableIT_IDLE(USART_INSTANCE);

    /* Start UART and enable DMA channel for RX */
    LL_USART_Enable(USART_INSTANCE);
    LL_DMA_EnableChannel(LPDMA1_CH0);

    /* TX DMA is started prior every transmission */
}

/**
 * \brief           LPDMA1 channel0 interrupt handler for USART_INSTANCE RX
 */
void
LPDMA1_CH0_IRQHandler(void) {
    uint8_t run_check = 0;

    /* Check for half-transfer interrupt */
    if (LL_DMA_IsEnabledIT_HT(LPDMA1_CH0) && LL_DMA_IsActiveFlag_HT(LPDMA1_CH0)) {
        LL_DMA_ClearFlag_HT(LPDMA1_CH0);
        run_check = 1;
    }

    /* Check for transfer-complete interrupt */
    if (LL_DMA_IsEnabledIT_TC(LPDMA1_CH0) && LL_DMA_IsActiveFlag_TC(LPDMA1_CH0)) {
        LL_DMA_ClearFlag_TC(LPDMA1_CH0);
        run_check = 1;
    }
    if (run_check) {
        usart_rx_check();
    }

    /* Implement other events when needed */
}

/**
 * \brief           LPDMA1 channel1 interrupt handler for USART_INSTANCE TX
 */
void
LPDMA1_CH1_IRQHandler(void) {
    /* Check transfer complete interrupt */
    if (LL_DMA_IsEnabledIT_TC(LPDMA1_CH1) && LL_DMA_IsActiveFlag_TC(LPDMA1_CH1)) {
        LL_DMA_ClearFlag_TC(LPDMA1_CH1);

        /* Skip data in memory - mark it as sent, then reset active transfer */
        lwrb_skip(&ctx.usart_tx_rb, ctx.usart_tx_dma_current_len);
        ctx.usart_tx_dma_current_len = 0;
        usart_start_tx_dma_transfer();
    }

    /* Implement other events when needed */
}

/**
 * \brief           USART_INSTANCE global interrupt handler
 * 
 * Function name is a macro and is defined at the beginning of this file
 */
void
USART_INSTANCE_IRQHandler(void) {
    /* Check for IDLE line interrupt */
    if (LL_USART_IsActiveFlag_IDLE(USART_INSTANCE)) {
        LL_USART_ClearFlag_IDLE(USART_INSTANCE); /* Clear IDLE flag */
        usart_rx_check();                        /* Check for data to process */
    }

    /* Implement other events when needed */
}

/**
  * Configure the system core clock only and activate it using the LL RCC unitary APIs (footprint optimization)
  *         The system Clock is configured as follow :
  *            System Clock source            = PSIS
  *            SYSCLK(Hz)                     = 144000000
  *            HCLK(Hz)                       = 144000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            APB3 Prescaler                 = 1
  *            Flash Latency(WS)              = 4
  */
static void
systemclock_config(void) {
    /* Enable external clock */
    LL_RCC_HSE_Enable();
    while (LL_RCC_HSE_IsReady() != 1U) {}

    /* Configure and enable PSI oscillator */
    LL_RCC_ConfigPSI(LL_RCC_PSIFREQ_144MHZ, NUCLEO_C5A3ZG ? LL_RCC_PSIREF_48MHZ : LL_RCC_PSIREF_24MHZ,
                     LL_RCC_PSISOURCE_HSE);
    LL_RCC_PSIS_Enable();
    while (LL_RCC_PSIS_IsReady() != 1U) {}

    /* Initializes the CPU, AHB and APB busses clocks */
    LL_RCC_ConfigBusClock(LL_RCC_HCLK_PRESCALER_1 | LL_RCC_APB1_PRESCALER_1 | LL_RCC_APB2_PRESCALER_1
                          | LL_RCC_APB3_PRESCALER_1);

    /* Frequency will be increased */
    LL_FLASH_SetLatency(FLASH, LL_FLASH_LATENCY_4WS);

    /* Set the source clock now */
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PSIS);
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PSIS) {}

    LL_FLASH_SetProgrammingDelay(FLASH, LL_FLASH_PROGRAM_DELAY_2);

    LL_SetSystemCoreClock(144000000U);
    LL_Init1msTick(SystemCoreClock);
}