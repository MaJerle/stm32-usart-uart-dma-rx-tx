
# Examples

All examples were originally developed on `NUCLEO-F413ZH` development board in configuration:

- USART settings
    - `USART3`, `115200` bauds, `1` stop bit, no-parity
    - STM32 TX pin: `PD8`
    - STM32 RX pin: `PD9`
- DMA settings
    - `DMA1`, `STREAM1`, `CHANNEL4`
    - Circular mode

Examples show `3` different use cases:

- `polling`: DMA hardware takes care to transfer received data to memory but application must constantly poll for new changes and read the received data fast enough to not get overwritten. Processing of received data is not in interrupt but in thread mode.
- `idle_line_irq`: Similar to `polling` except in this case user gets notification from `3` different sources:
	
	- USART idle line detection: Some data was received but now receive line is steady. Interrupt will notify application to process currently received data immediately
	- DMA Half-Transfer (HT): DMA hardware reached middle point of received length. Interrupt will norify application to process data immediately. This is to make sure we process data fast enough if we receive burst of data and number of bytes is higher than rolling receive buffer
	- DMA Transfer-Complete (TC): Exactly the same like `HT` event, except that it happens at the end of received rolling buffer. Once this event happens, DMA will start receiving from beginning of buffer
	- *Processing of data in this mode is always in interrupt. This may have negative effects on application if there is too much data to process at a time*.

- `idle_line_irq_rtos`

	- The same as `idle_line_irq` type, except it only writes notification to message queue. Data processing is done in separate thread which offloads interrupts for other tasks