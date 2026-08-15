cmake_minimum_required(VERSION 3.10)
project(test_model_cc CXX C)

# ---- acados paths ----
set(ACADOS_INCLUDE_DIR $ENV{ACADOS_SOURCE_DIR}/include)
set(ACADOS_LIB_DIR $ENV{ACADOS_SOURCE_DIR}/lib)

# ---- generated solver source directories ----
set(GENERATED_ROOT ${CMAKE_SOURCE_DIR}/../../generated)
set(UNICYCLE_OCP_DIR ${GENERATED_ROOT}/unicycle/unicycle_ocp)
set(UNICYCLE_SIM_DIR ${GENERATED_ROOT}/unicycle/unicycle_sim)
set(OMNI_OCP_DIR     ${GENERATED_ROOT}/omni/omni_ocp)
set(OMNI_SIM_DIR     ${GENERATED_ROOT}/omni/omni_sim)

# ---- common acados include dirs ----
set(ACADOS_INCLUDE_DIRS
    ${ACADOS_INCLUDE_DIR}
    ${ACADOS_INCLUDE_DIR}/acados
    ${ACADOS_INCLUDE_DIR}/blasfeo/include
    ${ACADOS_INCLUDE_DIR}/hpipm/include
)

# =============================================================================
# test_unicycle_solver
# =============================================================================
file(GLOB UNICYCLE_MODEL_SRC  ${UNICYCLE_OCP_DIR}/unicycle_model/*.c)
file(GLOB UNICYCLE_COST_SRC   ${UNICYCLE_OCP_DIR}/unicycle_cost/*.c)
file(GLOB UNICYCLE_SOLVER_SRC ${UNICYCLE_OCP_DIR}/acados_solver_unicycle.c)
set(UNICYCLE_GENERATED_SRC
    ${UNICYCLE_MODEL_SRC}
    ${UNICYCLE_COST_SRC}
    ${UNICYCLE_SOLVER_SRC}
)

add_executable(test_unicycle_solver
    test_unicycle_solver.cc
    ${UNICYCLE_GENERATED_SRC}
)

target_include_directories(test_unicycle_solver PRIVATE
    ${ACADOS_INCLUDE_DIRS}
    ${UNICYCLE_OCP_DIR}
    ${UNICYCLE_SIM_DIR}
)

set_source_files_properties(${UNICYCLE_GENERATED_SRC} PROPERTIES
    COMPILE_FLAGS "-std=c99 -O2"
)

target_compile_options(test_unicycle_solver PRIVATE
    -std=c++17 -O2 -Wall
)

target_compile_definitions(test_unicycle_solver PRIVATE
    EIGEN_NO_AUTOMATIC_RESIZING
)

target_link_directories(test_unicycle_solver PRIVATE
    ${ACADOS_LIB_DIR}
)

target_link_libraries(test_unicycle_solver
    acados
    hpipm
    blasfeo
    m
)

# =============================================================================
# test_omni_solver
# =============================================================================
file(GLOB OMNI_MODEL_SRC  ${OMNI_OCP_DIR}/omni_model/*.c)
file(GLOB OMNI_COST_SRC   ${OMNI_OCP_DIR}/omni_cost/*.c)
file(GLOB OMNI_SOLVER_SRC ${OMNI_OCP_DIR}/acados_solver_omni.c)
set(OMNI_GENERATED_SRC
    ${OMNI_MODEL_SRC}
    ${OMNI_COST_SRC}
    ${OMNI_SOLVER_SRC}
)

add_executable(test_omni_solver
    test_omni_solver.cc
    ${OMNI_GENERATED_SRC}
)

target_include_directories(test_omni_solver PRIVATE
    ${ACADOS_INCLUDE_DIRS}
    ${OMNI_OCP_DIR}
    ${OMNI_SIM_DIR}
)

set_source_files_properties(${OMNI_GENERATED_SRC} PROPERTIES
    COMPILE_FLAGS "-std=c99 -O2"
)

target_compile_options(test_omni_solver PRIVATE
    -std=c++17 -O2 -Wall
)

target_compile_definitions(test_omni_solver PRIVATE
    EIGEN_NO_AUTOMATIC_RESIZING
)

target_link_directories(test_omni_solver PRIVATE
    ${ACADOS_LIB_DIR}
)

target_link_libraries(test_omni_solver
    acados
    hpipm
    blasfeo
    m
)
