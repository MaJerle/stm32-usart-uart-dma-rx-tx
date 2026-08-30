# CPU parameters include
#
# - Cortex-M7
# - Thumb instructions
# - Double precision floating point unit
# - Hardware floating point unit config
set(cpu_PARAMS -mthumb -mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard)
include(${CMAKE_CURRENT_LIST_DIR}/compiler-gcc-cortex-common.cmake)
