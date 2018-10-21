# Use DMA to receive UART data on STM32 with unknown data length

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

Examples show `3` different use cases:

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