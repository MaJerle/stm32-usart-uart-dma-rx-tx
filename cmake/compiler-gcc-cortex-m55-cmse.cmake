# CPU parameters include
#
# - Cortex-M55
# - Thumb instructions
# - Helium (MVE) and floating point unit, auto-negotiated from the CPU
# - Hardware floating point unit config
# - TrustZone (CMSE) support enabled, for secure/non-secure split builds
set(cpu_PARAMS -mthumb -mcpu=cortex-m55 -mfpu=auto -mfloat-abi=hard -mcmse)
include(${CMAKE_CURRENT_LIST_DIR}/compiler-gcc-cortex-common.cmake)
