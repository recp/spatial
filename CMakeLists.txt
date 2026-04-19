cmake_minimum_required(VERSION 3.13)
project(spatial
  VERSION   0.1.0
  LANGUAGES C
  DESCRIPTION "Shared spatial kernel (hierarchy, pose, SoA runtime)")

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(SPATIAL_BUILD_TESTS     "Build unit tests"   ON)
option(SPATIAL_BUILD_BENCH     "Build benchmarks"   ON)
option(SPATIAL_ENABLE_ASAN     "AddressSanitizer"   OFF)
option(SPATIAL_ENABLE_TSAN     "ThreadSanitizer"    OFF)
option(SPATIAL_ENABLE_UBSAN    "UBSanitizer"        OFF)

find_path(CGLM_INCLUDE_DIR cglm/cglm.h
  HINTS
    /opt/homebrew/include
    /usr/local/include
    /usr/include
    ENV CGLM_INCLUDE)
if(NOT CGLM_INCLUDE_DIR)
  # Try pkg-config as last resort
  find_package(cglm CONFIG QUIET)
  if(NOT cglm_FOUND)
    message(FATAL_ERROR "cglm headers not found. Install https://github.com/recp/cglm")
  endif()
endif()
message(STATUS "cglm include: ${CGLM_INCLUDE_DIR}")

# Source discovery
set(SPATIAL_SRC
  spatial/src/spatial.c
)

if(WIN32)
  list(APPEND SPATIAL_SRC spatial/src/win/thread.c)
else()
  list(APPEND SPATIAL_SRC spatial/src/posix/thread.c)
endif()

add_library(spatial ${SPATIAL_SRC})
target_include_directories(spatial
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/spatial/include>
    $<INSTALL_INTERFACE:include>)

target_include_directories(spatial PUBLIC
  $<BUILD_INTERFACE:${CGLM_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>)
if(TARGET cglm::cglm)
  target_link_libraries(spatial PUBLIC cglm::cglm)
endif()

if(NOT WIN32)
  find_package(Threads REQUIRED)
  target_link_libraries(spatial PUBLIC Threads::Threads m)
endif()

if(SPATIAL_ENABLE_ASAN)
  target_compile_options(spatial PUBLIC -fsanitize=address -fno-omit-frame-pointer)
  target_link_options(spatial    PUBLIC -fsanitize=address)
endif()
if(SPATIAL_ENABLE_TSAN)
  target_compile_options(spatial PUBLIC -fsanitize=thread -fno-omit-frame-pointer)
  target_link_options(spatial    PUBLIC -fsanitize=thread)
endif()
if(SPATIAL_ENABLE_UBSAN)
  target_compile_options(spatial PUBLIC -fsanitize=undefined -fno-omit-frame-pointer)
  target_link_options(spatial    PUBLIC -fsanitize=undefined)
endif()

target_compile_options(spatial PRIVATE
  -Wall -Wextra -Wpedantic -Wdeclaration-after-statement)

if(SPATIAL_BUILD_TESTS)
  enable_testing()
  add_executable(test_spatial tests/test_spatial.c)
  target_link_libraries(test_spatial PRIVATE spatial)
  add_test(NAME test_spatial COMMAND test_spatial)

  add_executable(stress_spatial tests/stress_spatial.c)
  target_link_libraries(stress_spatial PRIVATE spatial)
  add_test(NAME stress_spatial COMMAND stress_spatial)
endif()

if(SPATIAL_BUILD_BENCH)
  add_executable(bench_spatial bench/bench_spatial.c)
  target_link_libraries(bench_spatial PRIVATE spatial)
endif()

# install
include(GNUInstallDirs)
install(TARGETS spatial
  EXPORT  spatialTargets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY spatial/include/spatial
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT spatialTargets
  FILE        spatialConfig.cmake
  NAMESPACE   spatial::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/spatial)

# pkg-config
configure_file(cmake/spatial.pc.in
  ${CMAKE_CURRENT_BINARY_DIR}/spatial.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/spatial.pc
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig)
