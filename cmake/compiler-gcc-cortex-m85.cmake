# CPU parameters include
#
# - Cortex-M85
# - Thumb instructions
# - Helium (MVE) and floating point unit, auto-negotiated from the CPU
# - Hardware floating point unit config
set(cpu_PARAMS -mthumb -mcpu=cortex-m85 -mfpu=auto -mfloat-abi=hard)
include(${CMAKE_CURRENT_LIST_DIR}/compiler-gcc-cortex-common.cmake)
