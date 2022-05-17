cmake_minimum_required(VERSION 3.22)

message("Entering ${CMAKE_CURRENT_LIST_DIR}/CMakeLists.txt")

#
# Core project settings
#
set(PROJ_PATH                       ${CMAKE_CURRENT_SOURCE_DIR})
project(usart_dma_rx_idle_line_irq_ringbuff_tx_loopback_U5)
enable_language(C CXX ASM)
message("Build type: "              ${CMAKE_BUILD_TYPE})

# Setup compiler settings
set(CMAKE_C_STANDARD                11)
set(CMAKE_C_STANDARD_REQUIRED       ON)
set(CMAKE_C_EXTENSIONS              ON)
set(CMAKE_CXX_STANDARD              20)
set(CMAKE_CXX_STANDARD_REQUIRED     ON)
set(CMAKE_CXX_EXTENSIONS            ON)

message("Config value ${CONFIG}")

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
set(linker_script_SRC               ${PROJ_PATH}/STM32U575ZITXQ_FLASH.ld)
set(EXECUTABLE                      ${CMAKE_PROJECT_NAME})

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
)

#
# Link directories setup
# Must be before executable is added
#
set(link_DIRS
    
)
link_directories(${EXECUTABLE} ${link_DIRS})

#
# Executable files
#
add_executable(${EXECUTABLE} 
	${src_stm32u5xx_hal_driver_SRCS}
	${src_lwrb_SRCS}
	${src_src_SRCS}
	${src_startup_SRCS})

#
# Add linked libraries for linker
#
set(link_LIBS
    
)
target_link_libraries(${EXECUTABLE} ${link_LIBS})

#
# Project symbols
#
target_compile_definitions(${EXECUTABLE} PRIVATE
    # Language specific only
    $<$<COMPILE_LANGUAGE:C>: ${symbols_c_SYMB}>
    $<$<COMPILE_LANGUAGE:CXX>: ${symbols_cxx_SYMB}>
    $<$<COMPILE_LANGUAGE:ASM>: ${symbols_asm_SYMB}>

    # Configuration specific
    $<$<CONFIG:Debug>:DEBUG >
    $<$<CONFIG:Release>: >
)

#
# Add include paths for each of the compiler
#
target_include_directories(${EXECUTABLE} PRIVATE
    # Language specific only
    $<$<COMPILE_LANGUAGE:C>: ${include_c_DIRS}>
    $<$<COMPILE_LANGUAGE:CXX>: ${include_cxx_DIRS}>
    $<$<COMPILE_LANGUAGE:ASM>: ${include_asm_DIRS}>

    # Configuration specific
    $<$<CONFIG:Debug>: >
    $<$<CONFIG:Release>: >
)

# Compiler and linker options
target_compile_options(${EXECUTABLE} PRIVATE
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
)

# Setup linker parameters
target_link_options(${EXECUTABLE} PRIVATE
    -T${MCU_LINKER_SCRIPT}${linker_script_SRC}
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

# Execute post-build to print size
add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${EXECUTABLE}>
)

# Convert output to hex and binary
add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${EXECUTABLE}> ${EXECUTABLE}.hex
)

# Convert to bin file -> add conditional check?
add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${EXECUTABLE}> ${EXECUTABLE}.bin
)

message("Exiting ${CMAKE_CURRENT_LIST_DIR}/CMakeLists.txt")
