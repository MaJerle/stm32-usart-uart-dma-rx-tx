#
# LIB_PREFIX: LWRB
#
# This file provides set of variables for end user
# and also generates one (or more) libraries, that can be added to the project using target_link_libraries(...)
#
# Before this file is included to the root CMakeLists file (using include() function), user can set some variables:
#
# LWRB_COMPILE_OPTIONS: If defined, it provide compiler options for generated library.
# LWRB_COMPILE_DEFINITIONS: If defined, it provides "-D" definitions to the library build
#

# Custom include directory
set(LWRB_CUSTOM_INC_DIR ${CMAKE_CURRENT_BINARY_DIR}/lib_inc)

# Library core sources
set(lwrb_core_SRCS
    ${CMAKE_CURRENT_LIST_DIR}/src/lwrb/lwrb.c
)

# Library extended sources
set(lwrb_ex_SRCS
    ${CMAKE_CURRENT_LIST_DIR}/src/lwrb/lwrb_ex.c
)

# Setup include directories
set(lwrb_include_DIRS
    ${CMAKE_CURRENT_LIST_DIR}/src/include
    ${LWRB_CUSTOM_INC_DIR}
)

# Register library to the system
add_library(lwrb)
target_sources(lwrb PRIVATE ${lwrb_core_SRCS})
target_include_directories(lwrb PUBLIC ${lwrb_include_DIRS})
target_compile_options(lwrb PRIVATE ${LWRB_COMPILE_OPTIONS})
target_compile_definitions(lwrb PRIVATE ${LWRB_COMPILE_DEFINITIONS})

# Register extended part
add_library(lwrb_ex)
target_sources(lwrb_ex PRIVATE ${lwrb_ex_SRCS})
target_include_directories(lwrb_ex PUBLIC ${lwrb_include_DIRS})
target_compile_options(lwrb_ex PRIVATE ${LWRB_COMPILE_OPTIONS})
target_compile_definitions(lwrb_ex PRIVATE ${LWRB_COMPILE_DEFINITIONS} LWRB_EXTENDED)
target_link_libraries(lwrb_ex PUBLIC lwrb)
