# STM32 + UART + DMA RX + unknown length

This repository may give you information about how to read data on UART by using DMA when number of bytes to receive is now known in advance.

In STM32 microcontroller family, U(S)ART reception can work in different modes:

- Polling mode (no DMA, no IRQ): Application must poll for status bits to check if new character has been received and read it fast enough in order to get all bytes
	- PROS
		- Very easy implementation
	- CONS
		- Very easy to loose receive characters
		- Works only in low baudrates
		- Application must very fast check for new character
- Interrupt mode (no DMA): UART triggers interrupt and CPU jumps to service routine to handle data reception
	- PROS
		- Most commonly used approach across all applications today
		- Works well in common baudrate, `115200` bauds
	- CONS
		- Interrupt service routing is executed for every received character
		- May stall other tasks in high-performance MCUs with many interrupts
		- May stall operating system when receiving burst of data at a time
- DMA mode: DMA is used to transfer data from USART RX data register to user memory on hardware level. No application interaction is needed at this point except processing received data by application once necessary
	- PROS
		- Transfer from USART peripheral to memory is done on hardware level without CPU interaction
		- Can work very easily with operating systems
		- Optimized for highest baudrates `> 1Mbps` and low-power applications
		- In case of big bursts of data, increasing data buffer size can improve functionality
	- CONS
		- Number of bytes to transfer must be known in advance by DMA hardware
		- If communication fails, DMA may not notify application about all bytes transferred

This article focuses only on *DMA mode* with unknown data length to receive.

### Important facts about DMA

DMA in STM32 can work in `normal` or `circular` mode. For each mode, it requires number of *elements* to transfer before events are triggered.

- *Normal mode*: In this mode, DMA starts transferring data and when it transfers all elements, it stops.
- *Circular mode*: In this mode, DMA starts with transfer, but when it reaches to the end, it jumps back on top of memory and continues to write

While transfer is active, `2` of many interrupts may be triggered:

- *Half-Transfer complete (HT)* interrupt: Executed when half of elements were transferred by DMA hardware
- *Transfer-Complete (TC)* interrupt: Executed when all elements transferred by DMA hardware
	- When DMA operates in *circular* mode, these interrupts are executed periodically

*Number of elements to transfer by DMA hardware must be written to relevant DMA registers!*

As you can see, we get notification by DMA on *HT* or *TC* events. Imagine application assumes it will receive `20` bytes, but it receives only `14`:
- Application would write `20` to relevant register for number of bytes to receive
- Application would be notfified after first `10` bytes are received (HT event)
- **Application is never notified for the rest of `4` bytes**
    - **Application must solve this case!**

### Important facts about U(S)ART

Most of STM32 series have U(S)ARTs with IDLE line detection. If IDLE line detection is not available, some of them have *Receiver Timeout* feature with programmable delay. If even this is not available, then application may use only *polling modes with DMA*, with examples provided below.

IDLE line detection (or Receiver Timeout) can trigger USART interrupt when receive line is steady without any communication for at least *1* character for reception.
Practicle example: Imagine we received *10* bytes at *115200* bauds. Each byte at *115200* bauds takes about `10us` on UART line, total `100us`. IDLE line interrupt will notify application when it will detect for `1` character inactivity on RX line, meaning after `10us` after last character. Application may react on this event and process data accordingly.

### Connect DMA + USARTs together

Now it is time to use all these features of DMA and USARTs in single application.
If we move to previous example of expecting to receive `20` bytes by application (and actually receiving only `14`), we can now:
- Application would write `20` to relevant register for number of bytes to receive
- Application would be notfified after first `10` bytes are received (HT event)
- Application would be notified after the rest `4` bytes because of USART IDLE line detection (IDLE LINE)

### Final configuration

- Put DMA to `circular` mode to avoid race conditions after DMA transfer completes and before user starts a new transfer
- Set memory length big enough to be able to receive all bytes while processing another.
    - Imagine you receive data at `115200` bauds, bursts of `100` bytes at a time.
    - It is good to set receive buffer to at least `100` bytes unless you can make sure your processing approach is faster than burst of data
    - At `115200` bauds, `100` bytes means `1ms` time

### DMA HT/TC and USART IDLE explanation

This section describes possible `4` possible cases and one additional which explains why *HT/TC* events are necessary by application

![DMA events](https://raw.githubusercontent.com/MaJerle/STM32_USART_DMA_RX/master/docs/dma_events.svg?sanitize=true)

Abbrevations used on image:
- `old_ptr`: Information about last used position to read data
- `new_ptr`: Information where will DMA save next byte in memory
- `HT`: Half-Transfer event triggered by DMA
- `TC`: Transfer-Complete event triggered by DMA
- `IDLE`: IDLE line detection by USART peripheral

DMA information:
- Circular mode
- `20` bytes memory depth, `HT` event received at `10` bytes

Possible cases:
- *P1*: DMA transfered `10` bytes. Application is notified by `HT` event and can read/process data, received by UART
- *P2*: DMA transfered `10` next `10` bytes. In this case, reading/processing starts from last known position until the end of memory
    - DMA is in circular mode, it will go automatically on beginning on memory to transfer more data
- *P3*: DMA transfered `10` bytes, but not aligned with `HT` nor `TC` events
    - Application will get `HT` event when first `6` bytes are received
    - Application will get `IDLE` event when next `4` bytes are received. By using `IDLE` interrupt, we can prevent application to think there was no data on USART and can possibly return `timeout` to other side in case of packet communication
- *P4*: DMA transfered `10` bytes in *overflow* mode, but not alighed with `HT` nor `TC events`
    - Application will get `TC` event when first `4` bytes are received
    - Application will get `IDLE` event when next `6` bytes are received. By using `IDLE` interrupt, we can prevent application to think there was no data on USART and can possibly return `timeout` to other side in case of packet communication
- *P5*: In case we rely only on *IDLE* line detection. What would happen if we receive more bytes in a burst than DMA can hold? In this case we can hold `20` bytes, but we received `30` bytes in burst
    - Application will get IDLE line event once there is steady RX line for `1` byte timeframe
    - Red part of data represents last data which overflowed previous one = *we lost `10` bytes*
    - Option is to poll for DMA changes faster than receiving burst of data may happen

For cases *P1-4*, below snippet shows how to get DMA positions and how much data to process.

```c
/**
 * \brief           Check for new data received with DMA
 * \note            This function must be called from DMA HT/TC and USART IDLE events
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
            /* Continue with beginning of buffer */
            usart_process_data(&usart_rx_dma_buffer[0], pos);
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

All examples were originally developed on `NUCLEO-F413ZH` development board in configuration:

- Developed in TrueSTUDIO and may be directly opened there
- Use of LL drivers
- USART settings
    - `USART3`, `115200` bauds, `1` stop bit, no-parity
    - STM32 TX pin: `PD8`
    - STM32 RX pin: `PD9`
- DMA settings
    - `DMA1`, `STREAM1`, `CHANNEL4`
    - Circular mode
- All examples implement loop-back terminology with polling approach

Examples show different use cases:

### Polling for changes

DMA hardware takes care of transferring received data to memory but application must constantly poll for new changes and read received data fast enough to not get overwritten. Processing of received data is in thread mode
- PROS
	- Easy to implement
	- No interrupts
	- Suitable for devices without *USART IDLE* line detection
- CONS
	- Application must take care of data periodically, fast enough, otherwise data may get overwritten by DMA hardware
	- Harder to get immediate reply when using *USART* based communication

### Polling for changes RTOS

Idea is completely the same as in previous case (*polling only*) but it uses separate thread for data processing
- PROS
	- Easy to implement to RTOS systems, uses only single thread without additional RTOS features
	- No interrupts
	- Data processing always *on-time* with maximum delay given by thread thus with known maximum latency between received character and processing time
	- Suitable for devices without *USART IDLE* line detection
- CONS
	- Uses more memory resources dedicated for separate thread for data processing

### USART Idle line detection + DMA HT&TC interrupts

Similar to `polling` except in this case user gets notification from `3` different sources:
	
- USART idle line detection: Some data was received but now receive line is steady. Interrupt will notify application to process currently received data immediately
	- DMA Half-Transfer (*HT*): DMA hardware reached middle point of received length. Interrupt will norify application to process data immediately. This is to make sure we process data fast enough if we receive burst of data and number of bytes is higher than rolling receive buffer
	- DMA Transfer-Complete (*TC*): Exactly the same like `HT` event, except that it happens at the end of received rolling buffer. Once this event happens, DMA will start receiving from beginning of buffer
- PROS
	- User do not need to check for new changes
	- Relevant interrupts are triggered on which user must immediate react
- CONS
	- Processing of data in this mode is always in interrupt. This may have negative effects on application if there is too much data to process at a time. Doing this may stall CPU and processing of other interrupts

### USART Idle line detection + DMA HT&TC interrupts with RTOS

- The same as `idle_line_irq` type, except it only writes notification to message queue. Data processing is done in separate thread which offloads interrupts for other tasks
- PROS
	- Processing is not in the interrupt, it is in separate thread
- CONS
	- Memory usage for separate thread + message queue (or semaphore)
	- Increases RAM footprint