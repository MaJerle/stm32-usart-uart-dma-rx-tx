cmake_minimum_required(VERSION 3.19)

project(usart_tx_debug_rtos_F4_${BOARD_NAME})
enable_language(C CXX ASM)

# Board-specific linker script and startup file
if(${BOARD_NAME} STREQUAL "NUCLEO_F413ZH")
    message(STATUS "Build configuration for board: NUCLEO_F413ZH")
    set(linker_script_SRC ${CMAKE_CURRENT_LIST_DIR}/linker/STM32F413ZH_FLASH.ld)
    set(startup_SRC ${CMAKE_CURRENT_LIST_DIR}/startup/startup_stm32f413xx.s)
else()
    message(FATAL_ERROR "Error, invalid board")
endif()

message(STATUS "Linker script: " ${linker_script_SRC})

# Start by scanning all sources in the root of this project
file(GLOB_RECURSE files_SRCS
    ${CMAKE_CURRENT_LIST_DIR}/src/*.c
)

# Add startup file and additional driver/middleware sources
set(files_SRCS ${files_SRCS}
    ${startup_SRC}

    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_dma.c
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_exti.c
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_gpio.c
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_rcc.c
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_usart.c
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_utils.c
    ${CMAKE_CURRENT_LIST_DIR}/../../middlewares/FreeRTOS/Source/CMSIS_RTOS/cmsis_os2.c
    ${CMAKE_CURRENT_LIST_DIR}/../../middlewares/FreeRTOS/Source/CMSIS_RTOS/cpu_utils.c
    ${CMAKE_CURRENT_LIST_DIR}/../../middlewares/FreeRTOS/Source/croutine.c
    ${CMAKE_CURRENT_LIST_DIR}/../../middlewares/FreeRTOS/Source/event_groups.c
    ${CMAKE_CURRENT_LIST_DIR}/../../middlewares/FreeRTOS/Source/portable/MemMang/heap_4.c
    ${CMAKE_CURRENT_LIST_DIR}/../../middlewares/FreeRTOS/Source/list.c
    ${CMAKE_CURRENT_LIST_DIR}/../../middlewares/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
    ${CMAKE_CURRENT_LIST_DIR}/../../middlewares/FreeRTOS/Source/queue.c
    ${CMAKE_CURRENT_LIST_DIR}/../../middlewares/FreeRTOS/Source/tasks.c
    ${CMAKE_CURRENT_LIST_DIR}/../../middlewares/FreeRTOS/Source/timers.c
)

# Include directories
set(include_DIRS ${include_DIRS}
    ${CMAKE_CURRENT_LIST_DIR}/inc
    ${CMAKE_CURRENT_LIST_DIR}/../../middlewares/FreeRTOS/Source/include
    ${CMAKE_CURRENT_LIST_DIR}/../../middlewares/FreeRTOS/Source/portable/GCC/ARM_CM4F
    ${CMAKE_CURRENT_LIST_DIR}/../../middlewares/FreeRTOS/Source/CMSIS_RTOS
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/STM32F4xx_HAL_Driver/Inc
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/CMSIS/Device/ST/STM32F4xx/Include
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/CMSIS/Include
)

# Symbols for compilation
set(symbols_SYMB ${symbols_SYMB}
    "USE_FULL_LL_DRIVER"
    "STM32F413xx"
    "${BOARD_NAME}"
)

# Core MCU flags, CPU, instruction set and FPU setup
# Set the global compilation flags for all units
include(${CMAKE_CURRENT_LIST_DIR}/../../cmake/compiler-gcc-cortex-m4.cmake)

# Add executable
add_executable(${CMAKE_PROJECT_NAME})
target_compile_definitions(${CMAKE_PROJECT_NAME} PUBLIC ${symbols_SYMB})
target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC ${include_DIRS})
target_sources(${CMAKE_PROJECT_NAME} PUBLIC ${files_SRCS})

# Add lwrb library
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../../middlewares/lwrb lwrb)
target_link_libraries(${CMAKE_PROJECT_NAME} PUBLIC lwrb)

# Setup linker parameters
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -T${linker_script_SRC}
    -Wl,-Map=${CMAKE_PROJECT_NAME}.map
    -u _printf_float # STDIO float formatting support (remove if not used)
    --specs=nosys.specs
    -Wl,--start-group
    -lc
    -lm
    -lstdc++
    -lsupc++
    -Wl,--end-group
    -Wl,--print-memory-usage
)

# Output files in the build folder
set(BUILD_OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR}/../../_build_outputs)
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> $<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>/${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> $<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>/${CMAKE_PROJECT_NAME}.bin
    COMMAND ${CMAKE_OBJDUMP} -S $<TARGET_FILE:${CMAKE_PROJECT_NAME}> > $<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>/${CMAKE_PROJECT_NAME}_objdump.txt
    COMMAND ${CMAKE_COMMAND} -E make_directory ${BUILD_OUTPUT_DIR}
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${BUILD_OUTPUT_DIR}/${CMAKE_PROJECT_NAME}.elf
)
