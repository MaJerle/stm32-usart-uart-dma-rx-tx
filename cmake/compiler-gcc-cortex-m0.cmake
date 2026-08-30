# CPU parameters include
#
# - Cortex-M0
# - Thumb instructions
# - No floating point unit
# - Software floating point config
set(cpu_PARAMS -mthumb -mcpu=cortex-m0 -mfloat-abi=soft)
include(${CMAKE_CURRENT_LIST_DIR}/compiler-gcc-cortex-common.cmake)
