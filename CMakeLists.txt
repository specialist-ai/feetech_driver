cmake_minimum_required(VERSION 3.16)
project(feetech_driver VERSION 0.2.0 LANGUAGES CXX)

string(COMPARE EQUAL "${CMAKE_SOURCE_DIR}" "${PROJECT_SOURCE_DIR}" FEETECH_IS_TOP_LEVEL)
if(FEETECH_IS_TOP_LEVEL AND NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(FEETECH_BUILD_TESTING "Build the unit tests" ${FEETECH_IS_TOP_LEVEL})
option(FEETECH_BUILD_PYTHON "Build the Python extension module" OFF)
option(FEETECH_INSTALL "Generate the install and CMake export rules" ON)

find_package(Threads REQUIRED)

add_library(feetech src/bus.cpp)
add_library(feetech::feetech ALIAS feetech)
target_include_directories(feetech PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(feetech PUBLIC cxx_std_17)
target_link_libraries(feetech PUBLIC Threads::Threads)
set_target_properties(feetech PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
  POSITION_INDEPENDENT_CODE ON
)

if(FEETECH_INSTALL)
  include(GNUInstallDirs)
  include(CMakePackageConfigHelpers)
  set(_feetech_cmake_dir ${CMAKE_INSTALL_LIBDIR}/cmake/feetech_driver)
  install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
  install(TARGETS feetech EXPORT feetech_driverTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  )
  install(EXPORT feetech_driverTargets NAMESPACE feetech:: DESTINATION ${_feetech_cmake_dir})
  write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/feetech_driverConfigVersion.cmake"
    COMPATIBILITY SameMinorVersion
  )
  configure_package_config_file(cmake/feetech_driverConfig.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/feetech_driverConfig.cmake"
    INSTALL_DESTINATION ${_feetech_cmake_dir}
  )
  install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/feetech_driverConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/feetech_driverConfigVersion.cmake"
    DESTINATION ${_feetech_cmake_dir}
  )
endif()

if(FEETECH_BUILD_PYTHON)
  find_package(Python 3.10 COMPONENTS Interpreter Development.Module REQUIRED)
  find_package(nanobind CONFIG REQUIRED)
  nanobind_add_module(_core NB_STATIC python/bindings.cpp)
  target_link_libraries(_core PRIVATE feetech)
  target_compile_definitions(_core PRIVATE FEETECH_VERSION="${PROJECT_VERSION}")
  install(TARGETS _core LIBRARY DESTINATION feetech_driver)
  nanobind_add_stub(_core_stub INSTALL_TIME MODULE feetech_driver._core
    OUTPUT feetech_driver/_core.pyi MARKER_FILE feetech_driver/py.typed PYTHON_PATH .)
endif()

if(FEETECH_BUILD_TESTING)
  enable_testing()
  find_package(GTest REQUIRED)
  include(GoogleTest)
  add_executable(feetech_tests test/test_protocol.cpp)
  target_link_libraries(feetech_tests PRIVATE feetech GTest::gtest_main)
  gtest_discover_tests(feetech_tests DISCOVERY_TIMEOUT 30)
endif()
