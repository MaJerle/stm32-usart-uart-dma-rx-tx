/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

LL_DMA_LinkNodeTypeDef Node_GPDMA1_Channel0;

void SystemClock_Config(void);
static void SystemPower_Config(void);
static void usart_init(void);

static uint8_t data_array[8];

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* System interrupt init*/
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 15, 0));

  /* Enable PWR clock interface */
  LL_AHB3_GRP1_EnableClock(LL_AHB3_GRP1_PERIPH_PWR);

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the System Power */
  SystemPower_Config();

  /* Enable instruction cache */
  LL_ICACHE_SetMode(LL_ICACHE_1WAY);
  LL_ICACHE_Enable();

  /* Initialize USART */
  usart_init();

  /* Infinite loop */
  while (1)
  {

  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_4)
  {
  }

  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_MSIS_Enable();

   /* Wait till MSIS is ready */
  while(LL_RCC_MSIS_IsReady() != 1)
  {
  }

  LL_RCC_MSI_EnableRangeSelection();
  LL_RCC_MSIS_SetRange(LL_RCC_MSISRANGE_0);
  LL_RCC_MSI_SetCalibTrimming(16, LL_RCC_MSI_OSCILLATOR_0);
  LL_RCC_PLL1_ConfigDomain_SYS(LL_RCC_PLL1SOURCE_MSIS, 3, 10, 1);
  LL_RCC_PLL1_EnableDomain_SYS();
  LL_RCC_SetPll1EPodPrescaler(LL_RCC_PLL1MBOOST_DIV_4);
  LL_RCC_PLL1_SetVCOInputRange(LL_RCC_PLLINPUTRANGE_8_16);
  LL_RCC_PLL1_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL1_IsReady() != 1)
  {
  }

   /* Intermediate AHB prescaler 2 when target frequency clock is higher than 80 MHz */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_2);

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL1);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL1)
  {
  }

  /* Insure 1�s transition state at intermediate medium speed clock based on DWT*/
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  DWT->CYCCNT = 0;
  while(DWT->CYCCNT < 100);

  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetAPB3Prescaler(LL_RCC_APB3_DIV_1);

  LL_Init1msTick(160000000);

  LL_SetSystemCoreClock(160000000);
}

/**
  * @brief Power Configuration
  * @retval None
  */
static void
SystemPower_Config(void)
{
  /* Disable the internal Pull-Up in Dead Battery pins of UCPD peripheral */
  LL_PWR_DisableUCPDDeadBattery();

  /* Switch to SMPS regulator instead of LDO */
  LL_PWR_SetRegulatorSupply(LL_PWR_SMPS_SUPPLY);
  while (!LL_PWR_IsActiveFlag_REGULATOR()) {}
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void
usart_init(void) {
    LL_USART_InitTypeDef USART_InitStruct = {0};
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    LL_DMA_InitTypeDef DMA_InitStruct = {0};
    LL_DMA_InitNodeTypeDef NodeConfig = {0};
    LL_DMA_LinkNodeTypeDef Node_GPDMA1_Channel0 = {0};
    LL_DMA_InitLinkedListTypeDef DMA_InitLinkedListStruct = {0};

    /* Set UART kernel clock */
    LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_PCLK2);

    /* Peripheral clock enable */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPDMA1);

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
     * Configure GPDMA CH1 for USART TX operation
     *
     * - Normal mode - no linked list operation
     * - Source address increase from memory
     * - Destination address no-increase to peripheral (USART TDR)
     * - Length set before channel enabled
     * - Source address set before TX operation
     * - Source uses PORT1 - fast port
     * - Destination uses PORT0 - fast port for APB access
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
    /* Default settings */
    DMA_InitStruct.SrcAddress = 0x00000000U;
    DMA_InitStruct.DestAddress = LL_USART_DMA_GetRegAddr(USART1, LL_USART_DMA_REG_DATA_TRANSMIT);
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
     * - Destination address increased
     * - Destination address set to start of raw buffer
     * - Length set to raw buffer size
     * - Destination uses PORT1 - fast port
     * - Source uses PORT0 - fast port for APB access
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
    NodeConfig.UpdateRegisters = (LL_DMA_UPDATE_CTR1 | LL_DMA_UPDATE_CTR2 | LL_DMA_UPDATE_CBR1 | LL_DMA_UPDATE_CSAR | LL_DMA_UPDATE_CDAR | LL_DMA_UPDATE_CTR3 | LL_DMA_UPDATE_CBR2 | LL_DMA_UPDATE_CLLR);
    NodeConfig.NodeType = LL_DMA_GPDMA_LINEAR_NODE;
    /* Additional settings */
    NodeConfig.SrcAddress = LL_USART_DMA_GetRegAddr(USART1, LL_USART_DMA_REG_DATA_RECEIVE);
    NodeConfig.DestAddress = (uint32_t)data_array;
    NodeConfig.BlkDataLength = sizeof(data_array) / sizeof(data_array[0]);
    LL_DMA_CreateLinkNode(&NodeConfig, &Node_GPDMA1_Channel0);

    /* Connect node to next node = to itself to achieve circular mode */
    LL_DMA_ConnectLinkNode(&Node_GPDMA1_Channel0, LL_DMA_CLLR_OFFSET5, &Node_GPDMA1_Channel0, LL_DMA_CLLR_OFFSET5);

    /* Set first linked list address to DMA channel */
    LL_DMA_SetLinkedListBaseAddr(GPDMA1, LL_DMA_CHANNEL_0, (uint32_t)&Node_GPDMA1_Channel0);
    //LL_DMA_SetLinkedListAddrOffset(GPDMA1, LL_DMA_CHANNEL_0, (uint32_t)&Node_GPDMA1_Channel0);

    /* Initialize linked list general setup for GPDMA CH0 */
    DMA_InitLinkedListStruct.Priority = LL_DMA_LOW_PRIORITY_LOW_WEIGHT;
    DMA_InitLinkedListStruct.LinkStepMode = LL_DMA_LSM_FULL_EXECUTION;
    DMA_InitLinkedListStruct.LinkAllocatedPort = LL_DMA_LINK_ALLOCATED_PORT0;
    DMA_InitLinkedListStruct.TransferEventMode = LL_DMA_TCEM_LAST_LLITEM_TRANSFER;
    LL_DMA_List_Init(GPDMA1, LL_DMA_CHANNEL_0, &DMA_InitLinkedListStruct);

    /* Enable TC interrupt */
    LL_DMA_EnableIT_HT(GPDMA1, LL_DMA_CHANNEL_0);
    LL_DMA_EnableIT_TC(GPDMA1, LL_DMA_CHANNEL_0);

    /* Enable DMA interrupts */
    NVIC_SetPriority(GPDMA1_Channel0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
    NVIC_EnableIRQ(GPDMA1_Channel0_IRQn);

    /* Configure USART */
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
    LL_USART_EnableDMAReq_RX(USART1);
    LL_USART_EnableDMAReq_TX(USART1);
#if 0
    LL_DMA_SetSrcAddress(GPDMA1, LL_DMA_CHANNEL_1, "Hello world\r\n");
    LL_DMA_SetBlkDataLength(GPDMA1, LL_DMA_CHANNEL_1, 13);
    LL_DMA_EnableChannel(GPDMA1, LL_DMA_CHANNEL_1);
#endif
    LL_USART_Enable(USART1);
    LL_DMA_EnableChannel(GPDMA1, LL_DMA_CHANNEL_0);
}

/**
  * @brief This function handles GPDMA1 Channel 0 global interrupt.
  */
void
GPDMA1_Channel0_IRQHandler(void) {
    /* Check for half-transfer interrupt */
    if (LL_DMA_IsEnabledIT_HT(GPDMA1, LL_DMA_CHANNEL_0)
            && LL_DMA_IsActiveFlag_HT(GPDMA1, LL_DMA_CHANNEL_0)) {
        LL_DMA_ClearFlag_HT(GPDMA1, LL_DMA_CHANNEL_0);
        /* TODO: Check DMA position */
    }

    /* Check for transfer-complete interrupt */
    if (LL_DMA_IsEnabledIT_TC(GPDMA1, LL_DMA_CHANNEL_0)
            && LL_DMA_IsActiveFlag_TC(GPDMA1, LL_DMA_CHANNEL_0)) {
        LL_DMA_ClearFlag_TC(GPDMA1, LL_DMA_CHANNEL_0);
        /* TODO: Check DMA position */
    }
}

/**
  * @brief This function handles GPDMA1 Channel 1 global interrupt.
  */
void
GPDMA1_Channel1_IRQHandler(void) {
    if (LL_DMA_IsEnabledIT_TC(GPDMA1, LL_DMA_CHANNEL_1)
            && LL_DMA_IsActiveFlag_TC(GPDMA1, LL_DMA_CHANNEL_1)) {
        LL_DMA_ClearFlag_TC(GPDMA1, LL_DMA_CHANNEL_1);
        /* TODO: Skip transmitted data from buffer */
        /* TODO: Try to send more data */
    }
}

void
USART1_IRQHandler(void) {

}
