# STM32 + UART + DMA RX + unknown length

This repository may give you information about how to read data on UART by using DMA when number of bytes to receive is not known in advance.

In STM32 microcontroller family, U(S)ART reception can work in different modes:

- Polling mode (no DMA, no IRQ)
	- Application is polling for status bits to check if any character has been received and read it fast enough in order to get all bytes
	- PROS
		- Easy to implementation
	- CONS
		- Easy to miss received characters in bursts
		- Works only for low baudrates
		- Application must very quickyl check for new characters
- Interrupt mode (no DMA)
	- UART triggers interrupt and CPU jumps to service routine to handle each received byte separately
	- PROS
		- Commonly used approach in embedded applications
		- Works well in common baudrate, `115200` bauds
	- CONS
		- Interrupt service routine is executed for every received character
		- May stall other tasks in high-performance MCUs with many interrupts
		- May stall operating system when receiving burst of data
- DMA mode
	- DMA is used to transfer data from USART RX data register to user memory on hardware level. No application interaction is needed at this point except processing received data by application once necessary
	- PROS
		- Transfer from USART peripheral to memory is done on hardware level without CPU interaction
		- Can work very easily with operating systems
		- Optimized for highest baudrates `> 1Mbps` and low-power applications
		- In case of big bursts of data, increasing data buffer size can improve functionality
	- CONS
		- Number of bytes to transfer must be known in advance by DMA hardware
		- If communication fails, DMA may not notify application about all bytes transferred

This article focuses only on *DMA mode* with unknown data length to receive.

### About DMA

DMA in STM32 can work in `normal` or `circular` mode. For each mode, it requires number of *elements* to transfer before events (such as transfer complete) are triggered.

- *Normal mode*: In this mode, DMA starts transferring data and when it transfers all elements, it stops.
- *Circular mode*: In this mode, DMA starts with transfer, but when it reaches to the end, it jumps back on top of memory and continues to write

While transfer is active, `2` of many different interrupts may get triggered:

- *Half-Transfer complete `HT`*: Triggers when DMA transfers half count of elements
- *Transfer-Complete `TC`*: Triggers when DMA transfers all elements

> When DMA operates in *circular* mode, these interrupts are triggered periodically
> Number of elements to transfer by DMA hardware must be written to relevant DMA register before start of transfer

### About U(S)ART

Most of STM32 series have U(S)ARTs with IDLE line detection. If IDLE line detection is not available, some of them have *Receiver Timeout* feature with programmable delay. If even this is not available, then application may use only *polling modes with DMA*, see examples below.

IDLE line detection (or Receiver Timeout) can trigger USART interrupt when receive line is steady without any communication for at least *1* character length.
Practicle example: To transmit `1` at `115200` bauds, it takes approximately `10us`, for `10 bytes`, this is `1ms`. IDLE line interrupt notifies application when it detects for `1` character inactivity on RX line, meaning after `100us` after last character. Application may react on this event and process data accordingly.

### Connect DMA + USARTs together

Now it is time to use all these features of DMA and USARTs in single application.
If the DMA

- Application writes `20` to relevant DMA register for data length
- Application is notified after first `10` bytes are received by `HT` event
- Application is notified after the rest `4` bytes are received by USART IDLE line detection (IDLE LINE)
- Application is not notified by `TC` event as DMA did not transfer `20` bytes

### Final configuration

- Put DMA to `circular` mode to avoid race conditions after DMA transfer completes and before application starts a new transfer
- Set memory length big enough to be able to receive all bytes while processing another
    - It is good to set receive buffer to at least `100` bytes unless application can make sure processing approach is faster than burst of data
    - At `115200` bauds, `100` bytes means `10ms` time, application must be able to process `100` bytes within at least `10ms` timeframe

### DMA HT/TC and USART IDLE explanation

This section describes possible `4` possible cases and one additional which explains why *HT/TC* events are necessary by application

![DMA events](https://raw.githubusercontent.com/MaJerle/STM32_USART_DMA_RX/master/docs/dma_events.svg?sanitize=true)

Abbrevations used on image:
- `R`: `R`ead pointer, used by application to read data from memory. Later also used as `old_ptr`
- `W`: `W`rite pointer, used by DMA to write next byte to. Increased every time DMA writes new byte. Later also used as `new_ptr`
- `HT`: Half-Transfer event triggered by DMA
- `TC`: Transfer-Complete event triggered by DMA
- `I`: IDLE line detection event triggered by USART

DMA configuration:
- Circular mode
- `20` bytes length memory
    - `HT` event triggers at `10` bytes
    - `TC` event triggers at `20` bytes

Possible cases:
- Case *A*: DMA transfers `10` bytes. Application gets notification by `HT` event and may process received data
- Case *B*: DMA transfers next `10` bytes. Application gets notification by `TC` event. Processing now starts from last known position until the end of memory
    - DMA is in circular mode, thus it will continue from beginning of buffer to transfer next byte
- Case *C*: DMA transfers `10` bytes, but not aligned with `HT` nor `TC` events
    - Application gets notification by `HT` event when first `6` bytes are transfered. Processing may start from last known read location
    - Application gets `IDLE` event after  next `4` bytes are transfered
- Case *D*: DMA transfers `10` bytes in *overflow* mode and but not aligned with `HT` nor `TC events`
    - Application gets notification by `TC` event when first `4` bytes are transfered. Processing may start from last known read location
    - Application gets notification by `IDLE` event after next `6` bytes are transfered. Processing may start from beginning of buffer
- Case *E*: Example what may happen when application relies only on `IDLE` event
    - If application receives `30` bytes in burst, `10` bytes get overwritten by DMA as application did not process it quickly
    - Application gets `IDLE` line event once there is steady RX line for `1` byte timeframe
    - Red part of data represents first `10` received bytes from burst which were overwritten by last `10` bytes in burst
    - Option to avoid such scenario is to poll for DMA changes quicker than burst of `20` bytes take; or by using `TC` and `HT` events

Example code to read data from memory and process it, for cases *A-D*

```c
/**
 * \brief           Check for new data received with DMA
 * \note            This function must be called from DMA HT/TC and USART IDLE events
 * \note            Full source code is available in examples
 */
void
usart_rx_check(void) {
    static size_t old_pos;
    size_t pos;

    /* Calculate current position in buffer */
    pos = ARRAY_LEN(usart_rx_dma_buffer) - LL_DMA_GetDataLength(DMA1, LL_DMA_STREAM_1);
    if (pos != old_pos) {                       /* Check change in received data */
        if (pos > old_pos) {                    /* Current position is over previous one */
            /* We are in "linear" mode, case P1, P2, P3 */
            /* Process data directly by subtracting "pointers" */
            usart_process_data(&usart_rx_dma_buffer[old_pos], pos - old_pos);
        } else {
            /* We are in "overflow" mode, case P4 */
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
```

# Examples

- Developed in STM32CubeIDE for easier evaluation on STM32 boards
- Fully developed using LL drivers
- USART common configuration: `115200` bauds, `1` stop bit, no-parity
- DMA common configuration: Circular mode, `TC` and `HT` events enabled
- All examples implement loop-back with polling. Every character received by UART and transfered by DMA is sent back to same UART

| STM32 family | Board name        | USART     | STM32 TX  | STM32 RX  | DMA settings                          |
|--------------|-------------------|-----------|-----------|-----------|---------------------------------------|
| STM32F1xx    | `BluePill-F103C8` | `USART1`  | `PA9`     | `PA10`    | *`DMA1`, `Channel 5`*                 |
| STM32F4xx    | `NUCLEO-F413ZH`   | `USART3`  | `PD8`     | `PD9`     | *`DMA1`, `Stream 1`, `Channel 4`*     |
| STM32G0xx    | `NUCLEO-G071RB`   | `USART2`  | `PA2`     | `PA3`     | *`DMA1`, `Channel 1`*                 |
| STM32G4xx    | `NUCLEO-G474RE`   | `LPUART1` | `PA2`     | `PA3`     | *`DMA1`, `Channel 1`*                 |
| STM32L4xx    | `NUCLEO-L432KC`   | `USART2`  | `PA2`     | `PA15`    | *`DMA1`, `Channel 6`, `Request 2`*    |

Examples show different use cases:

### Polling for changes

- DMA hardware takes care to transfer received data to memory
- Application must constantly poll for new changes and read received data quick enough to make sure DMA will not overwrite data in buffer
- Processing of received data is in thread mode
- PROS
	- Easy to implement
	- No interrupts
	- Fits for devices without *USART IDLE* line detection
- CONS
	- Application takes care of data periodically

### Polling for changes with operating system

- Same as polling for changes but with dedicated thread in operating system to process data
- PROS
	- Easy to implement to RTOS systems, uses single thread without additional RTOS features
	- No interrupts
	- Data processing always *on-time* with maximum delay given by thread thus with known maximum latency between received character and processed time
	- Fits for devices without *USART IDLE* line detection
- CONS
	- Application takes care of data periodically
	- Uses memory resources dedicated for separate thread for data processing

### USART Idle line detection + DMA HT&TC interrupts

- Application gets notification from IDLE line detection or DMA TC/HT events
- Application has to process data only when it receives any of the `3` events
- PROS
	- Application does not need to poll for new changes
	- Application receives interrupts on event
	- Application may enter low-power modes to increase battery life (if operated on battery)
- CONS
	- Processing of data in this mode is in interrupt
	- Long interrupt execution may break other compatibility in the application

### USART Idle line detection + DMA HT&TC interrupts with RTOS

- Application gets notification from IDLE line detection or DMA TC/HT events
- Application uses separate thread to process the data
- PROS
	- Processing is not in the interrupt but in separate interrupts
	- Interrupt only informs processing thread to process
- CONS
	- Memory usage for separate thread + message queue (or semaphore)

## How to use

1. run `git clone --recurse-submodules https://github.com/MaJerle/STM32_USART_DMA_RX` to clone repository including submodules
2. run examples from `projects` directory using Atollic TrueSTUDIO
