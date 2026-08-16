# Common compiler flags shared across all Cortex-M targets.
# The cortex-*.cmake file including this must set cpu_PARAMS before including it.
set(cxx_flags -Wno-deprecated -Wno-volatile -Wno-deprecated-enum-float-conversion -Wno-missing-field-initializers)
set(asm_flags -x assembler-with-cpp -MMD -MP)
set(debug_flags -Og -g3 -ggdb)
set(release_flags -Og -O3 -g0)

add_compile_options(
    ${cpu_PARAMS}
    -Wall -Wextra -Wpedantic -Wno-unused-parameter
    "$<$<COMPILE_LANGUAGE:CXX>:${cxx_flags}>"
    "$<$<COMPILE_LANGUAGE:ASM>:${asm_flags}>"
    "$<$<CONFIG:Debug>:${debug_flags}>"
    "$<$<CONFIG:Release>:${release_flags}>"
)
