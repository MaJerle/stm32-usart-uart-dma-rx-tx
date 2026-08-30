# CPU parameters include
#
# - Cortex-M33
# - Thumb instructions
# - Single precision floating point unit
# - Hardware floating point unit config
set(cpu_PARAMS -mthumb -mcpu=cortex-m33 -mfpu=fpv5-sp-d16 -mfloat-abi=hard)
include(${CMAKE_CURRENT_LIST_DIR}/compiler-gcc-cortex-common.cmake)
