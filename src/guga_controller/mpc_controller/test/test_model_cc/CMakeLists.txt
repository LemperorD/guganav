cmake_minimum_required(VERSION 3.10)
project(test_unicycle_solver CXX C)

# ---- acados paths ----
set(ACADOS_DIR /home/ld/3rdparty/acados)
set(ACADOS_INCLUDE_DIR ${ACADOS_DIR}/include)
set(ACADOS_LIB_DIR ${ACADOS_DIR}/lib)

# ---- generated solver source directory ----
set(GENERATED_DIR
    ${CMAKE_SOURCE_DIR}/../../generated/unicycle_ocp)

# ---- gather generated C sources (model + cost + solver) ----
file(GLOB MODEL_SRC   ${GENERATED_DIR}/unicycle_model/*.c)
file(GLOB COST_SRC    ${GENERATED_DIR}/unicycle_cost/*.c)
file(GLOB SOLVER_SRC  ${GENERATED_DIR}/acados_solver_unicycle.c)

set(GENERATED_SRC
    ${MODEL_SRC}
    ${COST_SRC}
    ${SOLVER_SRC}
)

# ---- main executable ----
add_executable(test_unicycle_solver
    test_unicycle_solver.cc
    ${GENERATED_SRC}
)

# ---- include directories ----
target_include_directories(test_unicycle_solver PRIVATE
    ${ACADOS_INCLUDE_DIR}
    ${ACADOS_INCLUDE_DIR}/acados
    ${ACADOS_INCLUDE_DIR}/blasfeo/include
    ${ACADOS_INCLUDE_DIR}/hpipm/include
    ${GENERATED_DIR}
)

# ---- compile flags ----
# C files from acados are C99
set_source_files_properties(${GENERATED_SRC} PROPERTIES
    COMPILE_FLAGS "-std=c99 -O2"
)

target_compile_options(test_unicycle_solver PRIVATE
    -std=c++17 -O2 -Wall
)

target_compile_definitions(test_unicycle_solver PRIVATE
    EIGEN_NO_AUTOMATIC_RESIZING
)

# ---- link libraries ----
target_link_directories(test_unicycle_solver PRIVATE
    ${ACADOS_LIB_DIR}
)

target_link_libraries(test_unicycle_solver
    acados
    hpipm
    blasfeo
    m
)
