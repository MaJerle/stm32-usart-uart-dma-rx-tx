cmake_minimum_required(VERSION 3.22)

#
# Core project settings
#
set(PROJ_PATH                       ${CMAKE_CURRENT_SOURCE_DIR})
project(usart_rx_idle_line_irq_ringbuff_tx_loopback_U5_${BOARD_NAME})
enable_language(C CXX ASM)
message("Build type: "              ${CMAKE_BUILD_TYPE})

# Setup compiler settings
set(CMAKE_C_STANDARD                11)
set(CMAKE_C_STANDARD_REQUIRED       ON)
set(CMAKE_C_EXTENSIONS              ON)
set(CMAKE_CXX_STANDARD              20)
set(CMAKE_CXX_STANDARD_REQUIRED     ON)
set(CMAKE_CXX_EXTENSIONS            ON)

#
# Core MCU flags, CPU, instruction set and FPU setup
#
set(CPU_PARAMETERS
    -mthumb
    # Other parameters
    # -mcpu, -mfloat, -mfloat-abi, ...
    -mcpu=cortex-m33
    -mfpu=fpv5-sp-d16
    -mfloat-abi=hard
)

# Set linker script
set(linker_script_SRC               ${PROJ_PATH}/linker/STM32U575ZITXQ_FLASH.ld)

#
# Source files
#
set(src_stm32u5xx_hal_driver_SRCS 
    ${PROJ_PATH}/../../drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_ll_dma.c
    ${PROJ_PATH}/../../drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_ll_exti.c
    ${PROJ_PATH}/../../drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_ll_gpio.c
    ${PROJ_PATH}/../../drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_ll_icache.c
    ${PROJ_PATH}/../../drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_ll_lpgpio.c
    ${PROJ_PATH}/../../drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_ll_pwr.c
    ${PROJ_PATH}/../../drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_ll_rcc.c
    ${PROJ_PATH}/../../drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_ll_usart.c
    ${PROJ_PATH}/../../drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_ll_utils.c)

set(src_lwrb_SRCS 
    ${PROJ_PATH}/../../middlewares/lwrb/src/lwrb/lwrb.c)

set(src_src_SRCS 
    ${PROJ_PATH}/Src/main.c
    ${PROJ_PATH}/Src/stm32u5xx_it.c
    ${PROJ_PATH}/Src/syscalls.c
    ${PROJ_PATH}/Src/sysmem.c
    ${PROJ_PATH}/Src/system_stm32u5xx.c)

set(src_startup_SRCS 
    ${PROJ_PATH}/Startup/startup_stm32u575zitxq.s)

#
# Include directories
#
set(include_c_DIRS
    ${PROJ_PATH}/Inc
    ${PROJ_PATH}/../../drivers/STM32U5xx_HAL_Driver/Inc
    ${PROJ_PATH}/../../drivers/CMSIS/Device/ST/STM32U5xx/Include
    ${PROJ_PATH}/../../drivers/CMSIS/Include
    ${PROJ_PATH}/../../middlewares/lwrb/src/include
)
set(include_cxx_DIRS
    
)
set(include_asm_DIRS
    
)

#
# Symbols definition
#
set(symbols_c_SYMB
    "DEBUG"
    "STM32U575xx"
    "USE_FULL_LL_DRIVER"
    "HSE_VALUE=16000000"
    "HSE_STARTUP_TIMEOUT=100"
    "LSE_STARTUP_TIMEOUT=5000"
    "LSE_VALUE=32768"
    "MSI_VALUE=48000000"
    "EXTERNALSAI1_CLOCK_VALUE=48000"
    "HSI_VALUE=16000000"
    "LSI_VALUE=32000"
    "VDD_VALUE=3300"
)
set(symbols_cxx_SYMB
    
)
set(symbols_asm_SYMB
    "DEBUG"
)

#
# Link directories setup
# Must be before executable is added
#
set(link_DIRS
    
)
link_directories(${CMAKE_PROJECT_NAME} ${link_DIRS})

#
# Executable files
#
add_executable(${CMAKE_PROJECT_NAME} 
    ${src_stm32u5xx_hal_driver_SRCS}
    ${src_lwrb_SRCS}
    ${src_src_SRCS}
    ${src_startup_SRCS})

#
# Add linked libraries for linker
#
set(link_LIBS
    
)
target_link_libraries(${CMAKE_PROJECT_NAME} ${link_LIBS})

#
# Project symbols
#
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Language specific only
    $<$<COMPILE_LANGUAGE:C>: ${symbols_c_SYMB}>
    $<$<COMPILE_LANGUAGE:CXX>: ${symbols_cxx_SYMB}>
    $<$<COMPILE_LANGUAGE:ASM>: ${symbols_asm_SYMB}>

    # Configuration specific
    $<$<CONFIG:Debug>:
        DEBUG
    >
    $<$<CONFIG:Release>: >
)

#
# Add include paths for each of the compiler
#
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Language specific only
    $<$<COMPILE_LANGUAGE:C>: ${include_c_DIRS}>
    $<$<COMPILE_LANGUAGE:CXX>: ${include_cxx_DIRS}>
    $<$<COMPILE_LANGUAGE:ASM>: ${include_asm_DIRS}>

    # Configuration specific
    $<$<CONFIG:Debug>: >
    $<$<CONFIG:Release>: >
)

# Compiler and linker options
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    ${CPU_PARAMETERS}
    -Wall
    -Wextra
    -Wpedantic
    -Wno-unused-parameter
    $<$<COMPILE_LANGUAGE:C>:

    >
    $<$<COMPILE_LANGUAGE:CXX>:
        #-Wno-volatile
        #-Wold-style-cast
        #-Wuseless-cast
        #-Wsuggest-override
    >
    $<$<COMPILE_LANGUAGE:ASM>:
        -x assembler-with-cpp
        -MMD
        -MP
    >
    $<$<CONFIG:Debug>:
        -Og -g3 -ggdb
    >
    $<$<CONFIG:Release>:
        -Og -g0
    >
)

# Setup linker parameters
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -T${linker_script_SRC}
    ${CPU_PARAMETERS}
    -Wl,-Map=${CMAKE_PROJECT_NAME}.map
    -u _printf_float              # STDIO float formatting support (remove if not used)
    --specs=nosys.specs
    -Wl,--start-group
    -lc
    -lm
    -lstdc++
    -lsupc++
    -Wl,--end-group
    -Wl,--print-memory-usage
)

# Output generated files
set(FILE_ELF $<TARGET_FILE:${CMAKE_PROJECT_NAME}>)
set(FILE_HEX $<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>/${CMAKE_PROJECT_NAME}.hex)
set(FILE_BIN $<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>/${CMAKE_PROJECT_NAME}.bin)
set(FILE_OBJDUMPED $<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>/${CMAKE_PROJECT_NAME}_objdump.txt)

# Output files in the build folder
set(BUILD_OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR}/../../_build_outputs)
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
    COMMAND ${CMAKE_OBJCOPY} -O ihex ${FILE_ELF} ${FILE_HEX}
    COMMAND ${CMAKE_OBJCOPY} -O binary ${FILE_ELF} ${FILE_BIN}
    COMMAND ${CMAKE_OBJDUMP} -S ${FILE_ELF} > ${FILE_OBJDUMPED}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${BUILD_OUTPUT_DIR}
    COMMAND ${CMAKE_COMMAND} -E copy ${FILE_ELF} ${BUILD_OUTPUT_DIR}/${CMAKE_PROJECT_NAME}.elf
)
