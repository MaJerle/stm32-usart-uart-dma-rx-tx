# STM32 UART DMA RX and TX

This application note contains explanations with examples for two distinct topics:

- Data reception with UART and DMA when the application does not know the number of bytes to receive in advance
- Data transmission with UART and DMA to avoid CPU stalling and free the CPU for other tasks

## How to use this repository

> We start with the examples instructions. The most attractive part of the repository.

All examples are developed with *CMake* build system generation, *Ninja* build system and *GCC* compiler.
Each example comes with `.vscode` folder and provides basic set of files for recommended extensions, simple tasks, launch/debug config and C/C++ extension intellisense configuration for *CMake* data provider.

1. Clone the repository: `git clone https://github.com/MaJerle/stm32-usart-uart-dma-rx-tx` or download the zip package.
2. Install *CMake*, *Ninja* and the *ARM GCC compiler* (`arm-none-eabi-gcc`) and make sure they are available on your `PATH`. Either install each of them separately or download and install [STM32CubeCLT](https://www.st.com/en/development-tools/stm32cubeclt.html), CLI tools for STM32 development.
3. Open the project folder with *Visual Studio code* or use command line tool to build the project with cmake

To build all examples at once with python helper script: 
```
python3 scripts/build.py [--clean] [--path projects/project-folder]
```

To manually build selected project with cmake

```
cd projects/projects-folder
cmake --list-presets
cmake --preset <preset_name>
cmake --build --preset <preset_name>
```

## Table of Contents

GitHub automatically generates a table of contents, available in the top-left corner of this document.

## Abbreviations

- DMA: Direct Memory Access controller in STM32
- UART: Universal Asynchronous Receiver Transmitter
- USART: Universal Synchronous Asynchronous Receiver Transmitter
- TX: Transmit
- RX: Receive
- HT: Half-Transfer Complete DMA event/flag
- TC: Transfer Complete DMA event/flag
- RTO: Receiver Timeout UART event/flag
- IRQ: Interrupt

## General about UART

> STM32 includes peripherals like USART, UART, and LPUART. For the purposes of this example, the specific differences between them aren't important, since the same concept applies to all. In a few words, USART supports synchronous operation on top of asynchronous (UART) and LPUART supports Low-Power operation in STOP mode. When synchronous mode or low-power mode is not used, USART, UART and LPUART can be considered identical. For a complete set of details, check the product's reference manual and datasheet.

> For the sake of this application note, we will only use the term **UART**.

UART in STM32 allows configuration using different transmit (`TX`) and receive (`RX`) modes:

- Polling mode (no DMA, no IRQ)
    - Advantages:
        - The application polls status bits to check whether a character has been transmitted or received, and reads it quickly enough to avoid missing any byte
        - Easy to implement, just a few lines of code
    - Disadvantages:
        - Can easily miss received data in a complex application if the CPU cannot read registers quickly enough
        - Works only for low baudrates, `9600` or lower
- Interrupt mode (no DMA)
    - Advantages:
        - UART triggers an interrupt and the CPU jumps to a service routine that handles each received byte separately
        - Commonly used approach in embedded applications
        - Works well with common baudrates, `115200`, up to `~921600` baud
    - Disadvantages:
        - Interrupt service routine is executed for every received character
        - May decrease system performance if interrupts are triggered for every character at high-speed baudrates
- DMA mode
    - DMA is used to transfer data from the USART RX data register to user memory at the hardware level. No application interaction is needed at this point, except when the application needs to process the received data
    - Advantages:
        - Transfer from the USART peripheral to memory is done at the hardware level, without CPU interaction
        - Works well with operating systems
        - Optimized for the highest baudrates (`> 1Mbps`) and for low-power applications
        - For large bursts of data, increasing the buffer size can improve throughput
    - Disadvantages:
        - The number of bytes to transfer must be known by the DMA hardware in advance
        - If communication fails, DMA may not notify the application about all bytes transferred

> This guide focuses exclusively on DMA-based RX operation and explains how to handle cases where the data length is unknown.

Every STM32 has at least one UART peripheral and at least one DMA controller built in.
This is all we need for successful data transmission.
The application uses default features to implement a very efficient DMA-based transmit system.

The implementation for TX operation is fairly straightforward (set the pointer to the data, define its length, and go), but this may not be the case for receive.
When implementing DMA receive, the application needs to know the number of received bytes to be processed by DMA before it is considered *done*. However, the UART protocol does not offer such information. (A higher-level protocol could provide it, but that is a separate topic not covered here — we assume we need to implement a very reliable low-level communication protocol.)

## Idle Line or Receiver Timeout events

STM32 UART peripherals can detect when the *RX* line remains inactive for a certain period of time. This is done using two methods:
- *IDLE line event*: Triggered when the RX line has been in idle state (normally high) for one frame time after the last received byte. Frame time is based on the baudrate — a higher baudrate means a shorter frame time for a single byte.
- *RTO (Receiver Timeout) event*: Triggered when the line has been idle for a programmable time. It is fully configurable by firmware.

Both events can trigger an interrupt, which is an essential feature for effective receive operation.

> Not all STM32 have *IDLE line* or *RTO* features available. When not available, examples concerning these features may not be used.

An example: transmitting `1` byte at `115200` baud takes approximately `~100us`; `3` bytes would take `~300us` in total.
The IDLE line event triggers an interrupt when the line has been idle for `1` frame time (in this case `~100us`) after the third byte has been received.

![IDLE LINE DEMO](docs/idle_line_demo.png)

This is a real experiment using *STM32F4* and the *IDLE line* event. After the *IDLE* event is triggered, data is echoed back (loopback mode):

- The application receives `3` bytes, taking approx `~300us` at `115200` baud
- *RX* goes to high state (yellow rectangle) and *UART RX* detects it has been idle for at least `1` frame time (approx `~100us`)
    - The width of the yellow rectangle represents `1` frame time
- The *IDLE line* interrupt is triggered at the green arrow
- The application echoes the data back from interrupt context

## General about DMA

DMA in STM32 can be configured in `normal` or `circular` mode.
For each mode, *DMA* requires the number of *elements* to transfer before its events (half-transfer complete, transfer complete) are triggered.

- *Normal mode*: DMA starts the data transfer; once it has transferred all elements, it stops and sets the enable bit to `0`.
    - The application uses this mode when transmitting data
- *Circular mode*: DMA starts the transfer; once it has transferred all elements (as written in the corresponding length register), it starts again from the beginning of memory and transfers more
    - The application uses this mode when receiving data

While a transfer is active, `2` (among others) interrupts may be triggered:

- *Half-Transfer complete (`HT`)*: Triggers when DMA has transferred half the elements
- *Transfer-Complete (`TC`)*: Triggers when DMA has transferred all elements

> When DMA operates in *circular* mode, these interrupts are triggered periodically.

> The number of elements to transfer must be written to the relevant DMA register before the start of the transfer.

### Combine UART + DMA for data reception

Now it is time to understand which features to use to receive data with UART and DMA to offload the CPU.
For the sake of this example, we use a memory buffer array of `20` bytes. DMA will transfer data received from UART to this buffer.

Listed are the steps to begin. The initial assumption is that UART has been initialized prior to reaching this step, and the same for basic DMA setup:

- The application writes `20` to the relevant DMA register for data length
- The application writes the memory and peripheral addresses to the relevant DMA registers
- The application sets the DMA direction to *peripheral-to-memory* mode
- The application puts DMA into *circular* mode. This ensures DMA does not stop transferring data after it reaches the end of memory. Instead, it rolls over and continues transferring any further data from UART to memory
- The application enables DMA and UART in reception mode. Reception cannot start until DMA waits for UART to receive the first character and transmit it to the array; this happens for every received byte
- The application is notified by the DMA `HT` event (or interrupt) after the first `10` bytes have been transferred from UART to memory
- The application is notified by the DMA `TC` event (or interrupt) after `20` bytes have been transferred from UART to memory
- The application is notified by the UART IDLE line (or RTO) event in case of IDLE line detection or a timeout on the RX line
- The application needs to rely on all of these events for the most efficient reception

> This configuration is important, as we do not know the length in advance. The application must assume it may receive an endless number of bytes, so DMA must operate endlessly.

> We used a `20`-byte-long array for demonstration purposes. In a real application, this size may need to be increased. It depends on the UART baudrate (a higher speed means more data may be received in a fixed window) and on how fast the application can process the received data (using interrupt notification, RTOS, or polling mode)

### Combine UART + DMA for data transmission

Everything gets simpler when the application transmits data: the length of the data is known in advance, and the memory to transmit is ready.
For the sake of this example, we use memory for the `HelloWorld` message. In *C language* it would be:

```c
const char
hello_world_arr[] = "HelloWorld";
```

- The application writes the number of bytes to transmit to the relevant DMA register, that would be `strlen(hello_world_arr)` or `10`
- The application writes the memory and peripheral addresses to the relevant DMA registers
- The application sets the DMA direction to *memory-to-peripheral* mode
- The application sets DMA to *normal* mode. This effectively disables DMA once all the bytes are successfully transferred
- The application enables DMA and UART in transmitter mode. Transmission starts immediately when UART requests the first byte via DMA to be shifted into the UART TX register
- The application is notified by the `TC` event (or interrupt) after all bytes have been transmitted from memory to UART via DMA
- DMA is stopped and the application may prepare the next transfer immediately

> Please note that the `TC` event is triggered before the last UART byte has been fully transmitted over UART.
> That is because the `TC` event is part of DMA and not part of UART.
> It is triggered when DMA transfers all the bytes from point *A* to point *B*. That is, point *A* for DMA is memory, and point *B* is the UART data register.
> It is then up to UART to clock the byte out to the GPIO pin.

### DMA HT/TC and UART IDLE combination details

This section describes `4` possible cases, plus one additional case that explains why *HT* and *TC* events are both necessary in the application.

![DMA events](https://raw.githubusercontent.com/MaJerle/stm32-usart-uart-dma-rx-tx/master/docs/dma_events.svg?sanitize=true)

Abbreviations used for the image:
- `R`: `R`ead pointer, used by the application to read data from memory. Later also used as `old_ptr`
- `W`: `W`rite pointer, used by the DMA to write next byte to. Increased every time DMA writes new byte. Later also used as `new_ptr`
- `HT`: `H`alf-`T`ransfer Complete event triggered by DMA
- `TC`: `T`ransfer-`C`omplete event - triggered by DMA
- `I`: `I`DLE line event - triggered by USART

DMA configuration:
- Circular mode
- `20` bytes data length
    - Consequently `HT` event gets triggered at `10` bytes being transmitted
    - Consequently `TC` event gets triggered at `20` bytes being transmitted

Possible cases during real-life execution:
- Case *A*: DMA transfers `10` bytes. The application receives a notification via the `HT` event and may process the received data
- Case *B*: DMA transfers the next `10` bytes. The application receives a notification via the `TC` event. Processing now starts from the last known position to the end of memory
    - DMA is in circular mode, so it continues right from the beginning of the buffer, at the top of the picture
- Case *C*: DMA transfers `10` bytes, but not aligned with `HT` or `TC` events
    - The application gets notified with the `HT` event when the first `6` bytes are transferred. Processing may start from the last known read location
    - The application receives the `IDLE` line event after the next `4` bytes are successfully transferred to memory
- Case *D*: DMA transfers `10` bytes in *overflow* mode and not aligned with `HT` or `TC` events
    - The application receives a notification via the `TC` event when the first `4` bytes are transferred. Processing may start from the last known read location
    - The application receives a notification via the `IDLE` event after the next `6` bytes are transferred. Processing may start from the beginning of the buffer
- Case *E*: An example of what may happen when the application relies only on the `IDLE` event
    - If the application receives `30` bytes in a burst, `10` bytes get overwritten by DMA because the application did not process them quickly enough
    - The application gets the `IDLE` line event once the RX line has been steady for `1` byte timeframe
    - The red part of the data represents the first `10` received bytes from the burst, which were overwritten by the last `10` bytes in the burst
    - An option to avoid this scenario is to poll for DMA changes faster than a `20`-byte burst takes, or to use the `TC` and `HT` events

Example code to read data from memory and process it, for cases *A-D*

```c
/**
 * \brief           Check for new data received with DMA
 *
 * The user must select the context from which to call this function:
 * - Only interrupts (DMA HT, DMA TC, UART IDLE) with the same preemption priority level
 * - Only thread context (outside interrupts)
 *
 * If called from both contexts, exclusive access protection must be implemented.
 * This mode is not advised, as it usually means architecture design problems.
 *
 * When the IDLE interrupt is not present, the application must rely only on thread context,
 * by manually calling this function as quickly as possible, to make sure
 * data is read from the raw buffer and processed.
 *
 * Not reading fast enough may cause DMA to overwrite unread received bytes,
 * causing the application to lose useful data.
 *
 * Solutions to this are:
 * - Improve architecture design to achieve faster reads
 * - Increase raw buffer size and allow DMA to write more data before this function is called
 */
void
usart_rx_check(void) {
    /*
     * Set old position variable as static.
     *
     * Linker should (with default C configuration) set this variable to `0`.
     * It is used to keep the latest read start position,
     * which makes this function neither reentrant nor thread-safe
     */
    static size_t old_pos;
    size_t pos;

    /* Calculate current position in buffer and check for new data available */
    pos = ARRAY_LEN(usart_rx_dma_buffer) - LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_5);
    if (pos != old_pos) {                       /* Check change in received data */
        if (pos > old_pos) {                    /* Current position is over previous one */
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
             * Processing is done in "overflow" mode.
             *
             * The application must process data twice,
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
            usart_process_data(&usart_rx_dma_buffer[old_pos], ARRAY_LEN(usart_rx_dma_buffer) - old_pos);
            if (pos > 0) {
                usart_process_data(&usart_rx_dma_buffer[0], pos);
            }
        }
        old_pos = pos;                          /* Save current position as old for next transfers */
    }
}
```

### Interrupt priorities are important

Thanks to the Cortex-M NVIC's (Nested Vectored Interrupt Controller) flexibility,
the user can configure a priority level for each NVIC interrupt line, giving full control over the execution profile of each interrupt separately.

There are `2` priority types in Cortex-M:
- Preemption priority: An interrupt with a higher logical priority level can preempt an already running lower-priority interrupt
- Subpriority: An interrupt with a higher subpriority (but the same preemption priority) will execute first when `2` (or more) interrupt lines become active at the same time; such an interrupt will also never preempt an interrupt already being executed by the CPU

STM32s have separate interrupt lines (and interrupt service routines) for DMA and UART, one for each peripheral, and their priorities can be configured in software.

The function called to process received data must keep track of the *last read position*; hence, this processing function is neither thread-safe nor reentrant and requires special attention.

> The application must ensure the DMA and UART interrupts use the same preemption priority level.
> This is the only configuration that guarantees the processing function never gets preempted by itself (the DMA interrupt preempting UART, or vice versa); otherwise, the last-known read position may become corrupted and the application will operate on wrong data.

## Examples

Examples can be used as reference code to implement your own DMA TX and RX functionality.

There are two sets of examples:
- Examples for RX only
    - Available in the `projects` folder with the `usart_rx_` prefix
    - DMA is used to receive data, and polling is used to echo the data back
- Examples for RX & TX
    - DMA is used to receive data and to transmit data back
    - It uses a ring buffer to copy data from the DMA buffer to the application before it is sent back

Common for all examples:
- Developed in [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) for easier evaluation on STM32 boards
- Fully developed using LL drivers for various STM32 families
- UART common configuration: `115200` baud, `1` stop bit, no parity
- DMA RX common configuration: Circular mode, `TC` and `HT` events enabled
- DMA TX common configuration: Normal mode, `TC` event enabled
- All RX examples implement loop-back functionality: every character received by UART and transferred by DMA is sent back to the same UART.

| STM32 family | Board name         | USART     | STM32 TX | STM32 RX | RX DMA settings                    | TX DMA settings                   |
| ------------ | ------------------ | --------- | -------- | -------- | ---------------------------------- | --------------------------------- |
| STM32F1xx    | `BluePill-F103C8`  | `USART1`  | `PA9`    | `PA10`   | *`DMA1`, `Channel 5`*              |                                   |
| STM32F4xx    | `NUCLEO-F413ZH`    | `USART3`  | `PD8`    | `PD9`    | *`DMA1`, `Stream 1`, `Channel 4`*  | *`DMA1`, `Stream 3`, `Channel 4`* |
| STM32G0xx    | `NUCLEO-G071RB`    | `USART2`  | `PA2`    | `PA3`    | *`DMA1`, `Channel 1`*              |                                   |
| STM32G4xx    | `NUCLEO-G474RE`    | `LPUART1` | `PA2`    | `PA3`    | *`DMA1`, `Channel 1`*              |                                   |
| STM32L4xx    | `NUCLEO-L432KC`    | `USART2`  | `PA2`    | `PA15`   | *`DMA1`, `Channel 6`, `Request 2`* |                                   |
| STM32H7xx    | `NUCLEO-H743ZI2*`  | `USART3`  | `PD8`    | `PD9`    | *`DMA1`, `Stream 0`*               | *`DMA1`, `Stream 1`*              |
| STM32U5xx    | `NUCLEO-U575ZI-Q*` | `USART1`  | `PA9`    | `PA10`   | *`GPDMA1`, `Channel 0`*            | *`GPDMA1`, `Channel 1`*           |
| STM32C5xx    | `NUCLEO-C562RE`    | `USART2`  | `PA2`    | `PA3`    | *`LPDMA1`, `Channel 0`*            | *`LPDMA1`, `Channel 1`*           |
| STM32C5xx    | `NUCLEO-C542RC`    | `USART2`  | `PA2`    | `PA3`    | *`LPDMA1`, `Channel 0`*            | *`LPDMA1`, `Channel 1`*           |
| STM32C5xx    | `NUCLEO-C5A3ZG`    | `USART2`  | `PA2`    | `PA3`    | *`LPDMA1`, `Channel 0`*            | *`LPDMA1`, `Channel 1`*           |

> * It is possible to run H743 (single-core) examples on dual-core STM32H7 Nucleo boards, NUCLEO-H745 or NUCLEO-H755.
> Special care needs to be taken, as dual-core H7 Nucleo boards use DCDC for MCU power; hence
> the application must check the clock configuration in the main file and uncomment the code to enable SMPS.

Examples demonstrate different use cases for RX only or combined RX & TX.

> The demos in this repository are all based on Low-Level (LL) drivers to maximize user understanding of how to convert theory into practice.
> Some STM32Cube firmware packages include the same example using HAL drivers too. Some of them are listed below, with a link to the example (the list is not exhaustive).
> All examples are identified as *UART_ReceptionToIdle_CircularDMA* - you can search for it in your local Cube firmware repository.
> * [STM32U5 UART_ReceptionToIdle_CircularDMA](https://github.com/STMicroelectronics/STM32CubeU5/tree/main/Projects/NUCLEO-U575ZI-Q/Examples/UART/UART_ReceptionToIdle_CircularDMA)
> * [STM32L5 UART_ReceptionToIdle_CircularDMA](https://github.com/STMicroelectronics/STM32CubeL5/tree/master/Projects/NUCLEO-L552ZE-Q/Examples/UART/UART_ReceptionToIdle_CircularDMA)
> * [STM32G4 UART_ReceptionToIdle_CircularDMA](https://github.com/STMicroelectronics/STM32CubeG4/tree/master/Projects/NUCLEO-G474RE/Examples/UART/UART_ReceptionToIdle_CircularDMA)
> * [STM32F4 UART_ReceptionToIdle_CircularDMA](https://github.com/STMicroelectronics/STM32CubeF4/tree/master/Projects/STM32446E-Nucleo/Examples/UART/UART_ReceptionToIdle_CircularDMA)
> * [STM32G0 UART_ReceptionToIdle_CircularDMA](https://github.com/STMicroelectronics/STM32CubeG0/tree/master/Projects/STM32G0C1E-EV/Examples/UART/UART_ReceptionToIdle_CircularDMA)
> * [STM32L4 UART_ReceptionToIdle_CircularDMA](https://github.com/STMicroelectronics/STM32CubeL4/tree/master/Projects/NUCLEO-L476RG/Examples/UART/UART_ReceptionToIdle_CircularDMA)
> * [STM32WB UART_ReceptionToIdle_CircularDMA](https://github.com/STMicroelectronics/STM32CubeWB/tree/master/Projects/P-NUCLEO-WB55.Nucleo/Examples/UART/UART_ReceptionToIdle_CircularDMA)
> * [STM32WL UART_ReceptionToIdle_CircularDMA](https://github.com/STMicroelectronics/STM32CubeWL/tree/main/Projects/NUCLEO-WL55JC/Examples/UART/UART_ReceptionToIdle_CircularDMA)

## Examples for UART + DMA RX

### Polling for changes

- DMA hardware takes care of transferring received data to memory
- The application must constantly poll for changes in the DMA registers and read the received data quickly enough to make sure DMA does not overwrite data in the buffer
- Processing of received data happens in thread mode (not in an interrupt)
- Advantages:
    - Easy to implement
    - No interrupts, so no consideration of priority or race conditions
    - Suits devices without *UART IDLE* line detection
- Disadvantages:
    - The application must handle data periodically
    - Not possible to put the application into low-power (sleep) mode

### Polling for changes with an operating system

- Same as polling for changes, but with a dedicated thread in the operating system to process data
- Advantages:
    - Easy to implement on RTOS systems; uses a single thread without additional RTOS features (no mutexes, semaphores, or memory queues)
    - No interrupts, so no consideration of priority or race conditions
    - Data processing is always *on time*, with a maximum delay given by the thread delay, giving a known maximum latency between the received character and processing time
        - Unless the system has higher-priority threads
    - Suits devices without *UART IDLE* line detection
- Disadvantages:
    - The application must handle data periodically
    - Uses memory resources dedicated to a separate thread for data processing
    - Not possible to put the application into low-power (sleep) mode

### UART IDLE line detection + DMA HT & TC interrupts

- The application receives a notification via IDLE line detection or DMA TC/HT events
- The application only needs to process data when it receives one of these `3` interrupts
- Advantages:
    - The application does not need to poll for changes
    - The application receives interrupts on events
    - The application may enter low-power modes to save power
- Disadvantages:
    - Data is read (processed) in the interrupt; we strive to execute the interrupt routine as fast as possible
    - Long interrupt execution may hurt responsiveness elsewhere in the application

*Processing of incoming data happens from 2 interrupt vectors, so it is important that they do not preempt each other. Set both to the same preemption priority!*

### UART IDLE line detection + DMA HT & TC interrupts with RTOS

- The application receives a notification via IDLE line detection or DMA TC/HT events
- The application uses a separate thread to process the data only when notified by one of these interrupts
- Advantages:
    - Processing happens not in the interrupt, but in a separate thread
    - The interrupt only informs the processing thread that it needs to run (or wake up)
    - The operating system may put the processing thread into a blocked state while it waits for the event
- Disadvantages:
    - Memory usage for a separate thread plus a message queue (or semaphore)

> This is the preferred way to receive and process UART characters.

## Examples for UART DMA for TX (and optionally included RX)

- The application uses DMA in normal mode to transfer data
- The application always uses a ring buffer between the high-level write and the low-level transmit operation
- The DMA TC interrupt is triggered when a transfer has finished, after which the application can send more data

### Demo application for debug messages

This is a demo application available in the `projects` folder.
Its purpose is to show how the application can output debug messages without significantly affecting CPU performance.
It uses DMA to transfer data (so the CPU does not need to wait on UART flags) and can achieve very high or very low data rates.

- All debug messages from the application are written to an intermediate ring buffer
- The application tries to start and configure DMA after every write to the ring buffer
- If a transfer is ongoing, the next start is configured from the DMA TC interrupt

As a result of running this demo application on an STM32F413-Nucleo board, the following observations were made:
- The demo code sends `1581` bytes every second at `115200` baud, which takes approx `142ms`
- With DMA disabled, CPU load was `14%`, in line with the time needed to transmit the data
- With DMA enabled, CPU load was `0%`
- DMA can be enabled/disabled via the `USE_DMA_TX` macro configuration in `main.c`
