cmake_minimum_required(VERSION 3.22)

message("Entering ${CMAKE_CURRENT_LIST_DIR}/CMakeLists.txt")

#
# Core project settings
#
set(PROJ_PATH                       ${CMAKE_CURRENT_SOURCE_DIR})
project(usart_dma_rx_idle_line_irq_rtos_multi_instance_L4)
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
    -mcpu=cortex-m4
	-mfpu=fpv4-sp-d16
	-mfloat-abi=hard
)

# Set linker script
set(linker_script_SRC               ${PROJ_PATH}/STM32L432KC_FLASH.ld)
set(EXECUTABLE                      ${CMAKE_PROJECT_NAME})

#
# Source files
#
set(linked_SRCS
    ${PROJ_PATH}/../../drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_dma.c
	${PROJ_PATH}/../../drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_exti.c
	${PROJ_PATH}/../../drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_gpio.c
	${PROJ_PATH}/../../drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_rcc.c
	${PROJ_PATH}/../../drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_usart.c
	${PROJ_PATH}/../../drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_utils.c
	${PROJ_PATH}/../../middlewares/FreeRTOS/Source/CMSIS_RTOS/cmsis_os2.c
	${PROJ_PATH}/../../middlewares/FreeRTOS/Source/croutine.c
	${PROJ_PATH}/../../middlewares/FreeRTOS/Source/event_groups.c
	${PROJ_PATH}/../../middlewares/FreeRTOS/Source/portable/MemMang/heap_4.c
	${PROJ_PATH}/../../middlewares/FreeRTOS/Source/list.c
	${PROJ_PATH}/../../middlewares/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
	${PROJ_PATH}/../../middlewares/FreeRTOS/Source/queue.c
	${PROJ_PATH}/../../middlewares/FreeRTOS/Source/stream_buffer.c
	${PROJ_PATH}/../../middlewares/FreeRTOS/Source/tasks.c
	${PROJ_PATH}/../../middlewares/FreeRTOS/Source/timers.c
)
set(source_folder_Drivers_SRCS 
	)

set(source_folder_Drivers_STM32L4xx_HAL_Driver_SRCS 
	)

set(source_folder_Inc_SRCS 
	)

set(source_folder_Middleware_SRCS 
	)

set(source_folder_Middleware_FreeRTOS_SRCS 
	)

set(source_folder_Src_SRCS 
	${PROJ_PATH}/Src/main.c
	${PROJ_PATH}/Src/stm32l4xx_it.c
	${PROJ_PATH}/Src/system_stm32l4xx.c)

set(source_folder_startup_SRCS 
	${PROJ_PATH}/startup/startup_stm32l432xx.s)

# Source files in the same path as ".cproject/.project" but nowhere included in file descriptions
set(all_project_dir_SRCS
    ${PROJ_PATH}/syscalls.c
)

#
# Include directories
#
set(include_c_DIRS
    ${PROJ_PATH}/Inc
	${PROJ_PATH}/../../middlewares/FreeRTOS/Source/Include
	${PROJ_PATH}/../../middlewares/FreeRTOS/Source/portable/GCC/ARM_CM4F
	${PROJ_PATH}/../../middlewares/FreeRTOS/Source/CMSIS_RTOS
	${PROJ_PATH}/../../drivers/STM32L4xx_HAL_Driver/Inc
	${PROJ_PATH}/../../drivers/CMSIS/Device/ST/STM32L4xx/Include
	${PROJ_PATH}/../../drivers/CMSIS/Include
)
set(include_cxx_DIRS
    
)
set(include_asm_DIRS
    
)

#
# Symbols definition
#
set(symbols_c_SYMB
    "USE_FULL_LL_DRIVER"
	"__weak=__attribute__((weak))"
	"__packed=__attribute__((__packed__))"
	"STM32L432xx"
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
    ${linked_SRCS}
    ${all_project_dir_SRCS} 
	${source_folder_Drivers_SRCS}
	${source_folder_Drivers_STM32L4xx_HAL_Driver_SRCS}
	${source_folder_Inc_SRCS}
	${source_folder_Middleware_SRCS}
	${source_folder_Middleware_FreeRTOS_SRCS}
	${source_folder_Src_SRCS}
	${source_folder_startup_SRCS})

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
    $<$<CONFIG:Debug>:
        DEBUG
    >
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

    >
    $<$<CONFIG:Debug>:
        -Og -g3 -ggdb
    >
    $<$<CONFIG:Release>:
        -Og -g0
    >
)

# Setup linker parameters
target_link_options(${EXECUTABLE} PRIVATE
    -T${MCU_LINKER_SCRIPT}${linker_script_SRC}
    ${CPU_PARAMETERS}
    -Wl,-Map=${CMAKE_PROJECT_NAME}.map
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

# Conver to bin file -> add conditional check?
add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${EXECUTABLE}> ${EXECUTABLE}.bin
)

message("Exiting ${CMAKE_CURRENT_LIST_DIR}/CMakeLists.txt")