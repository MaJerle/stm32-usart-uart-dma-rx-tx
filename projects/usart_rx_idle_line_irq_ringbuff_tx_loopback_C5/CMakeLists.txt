cmake_minimum_required(VERSION 3.19)

project(usart_rx_idle_line_irq_ringbuff_tx_loopback_C5_${BOARD_NAME})
enable_language(C CXX ASM)

# Source files from src directory
if(${BOARD_NAME} STREQUAL "NUCLEO_C562RE")
    message(STATUS "Build configuration for board: NUCLEO_C562RE")
    set(linker_script_SRC ${CMAKE_CURRENT_LIST_DIR}/linker/stm32c562xe_flash.ld)
    set(startup_SRC ${CMAKE_CURRENT_LIST_DIR}/startup/startup_stm32c562xx.c)
    set(CPU_DEFINE "STM32C562xx")
elseif(${BOARD_NAME} STREQUAL "NUCLEO_C542RC")
    message(STATUS "Build configuration for board: NUCLEO_C542RC")
    set(linker_script_SRC ${CMAKE_CURRENT_LIST_DIR}/linker/stm32c542xc_flash.ld)
    set(startup_SRC ${CMAKE_CURRENT_LIST_DIR}/startup/startup_stm32c542xx.c)
    set(CPU_DEFINE "STM32C542xx")
elseif(${BOARD_NAME} STREQUAL "NUCLEO_C5A3ZG")
    message(STATUS "Build configuration for board: NUCLEO_C5A3ZG")
    set(linker_script_SRC ${CMAKE_CURRENT_LIST_DIR}/linker/stm32c5a3xg_flash.ld)
    set(startup_SRC ${CMAKE_CURRENT_LIST_DIR}/startup/startup_stm32c5a3xx.c)
    set(CPU_DEFINE "STM32C5A3xx")
else()
    message(FATAL_ERROR "Error, invalid board")
endif()

message(STATUS "Linker script: " ${linker_script_SRC})

# Start by scanning all sources in the root of this project
file(GLOB_RECURSE files_SRCS
    ${CMAKE_CURRENT_LIST_DIR}/src/*.c
)

# Add drivers files
set(files_SRCS ${files_SRCS}
    ${startup_SRC}

    # Add startup files
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/STM32C5xx_drivers/ll/
)

# Include directories
set(include_DIRS ${include_DIRS}
    ${CMAKE_CURRENT_LIST_DIR}/inc
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/CMSIS/include
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/STM32C5xx_drivers/ll/
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/CMSIS/Device/ST/STM32C5xx/Include
    ${CMAKE_CURRENT_LIST_DIR}/../../middlewares/lwrb/src/include
)

# Symbols for compilation
set(symbols_SYMB ${symbols_SYMB}
    "STM32C5xx"
    "${CPU_DEFINE}" # STM32C562xx or equilvalent
    "${BOARD_NAME}"
)

# Core MCU flags, CPU, instruction set and FPU setup
# Set the global compilation flags for all units
include(${CMAKE_CURRENT_LIST_DIR}/../../cmake/compiler-flags-cortex-m33.cmake)

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
    ${cpu_PARAMS}
    -Wl,-Map=${CMAKE_PROJECT_NAME}.map

    -Wl,--start-group
    -lc
    -lm

    -lstdc++
    -lsupc++
    -Wl,--end-group
    -Wl,-z,max-page-size=8 # Allow good software remapping across address space (with proper GCC section making)
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
