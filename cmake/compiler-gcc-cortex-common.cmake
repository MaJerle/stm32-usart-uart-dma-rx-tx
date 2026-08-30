# Common GCC compiler flags shared across all Cortex-M targets.
# The cortex-*.cmake file including this must set cpu_PARAMS before including it.

string(REPLACE ";" " " CPU_FLAGS "${cpu_PARAMS}")

set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} ${CPU_FLAGS} -Wall -Wextra -Wpedantic -Wno-unused-parameter")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CPU_FLAGS} -Wall -Wextra -Wpedantic -Wno-unused-parameter -Wno-deprecated -Wno-volatile -Wno-deprecated-enum-float-conversion -Wno-missing-field-initializers")
set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} ${CPU_FLAGS} -Wall -Wextra -Wpedantic -Wno-unused-parameter -x assembler-with-cpp -MMD -MP")

set(CMAKE_C_FLAGS_DEBUG     "-Og -g3 -ggdb")
set(CMAKE_CXX_FLAGS_DEBUG   "-Og -g3 -ggdb")
set(CMAKE_ASM_FLAGS_DEBUG   "-Og -g3 -ggdb")

set(CMAKE_C_FLAGS_RELEASE   "-Og -O3 -g0")
set(CMAKE_CXX_FLAGS_RELEASE "-Og -O3 -g0")
set(CMAKE_ASM_FLAGS_RELEASE "-Og -O3 -g0")
