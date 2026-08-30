# CPU parameters include
#
# - Cortex-M23
# - Thumb instructions
# - No floating point unit
# - Software floating point config
# - TrustZone (CMSE) support enabled, for secure/non-secure split builds
set(cpu_PARAMS -mthumb -mcpu=cortex-m23 -mfloat-abi=soft -mcmse)
include(${CMAKE_CURRENT_LIST_DIR}/compiler-gcc-cortex-common.cmake)
