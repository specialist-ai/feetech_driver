cmake_minimum_required(VERSION 3.16)
project(feetech_driver VERSION 0.1.0 LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(FEETECH_BUILD_TESTING "Build the unit tests" ON)
option(FEETECH_BUILD_EXAMPLES "Build the example programs" ON)
option(FEETECH_BUILD_PYTHON "Build the Python extension module" OFF)
# The wheel needs only the extension module, not the C++ install rules.
option(FEETECH_INSTALL "Generate the install and CMake export rules" ON)

find_package(Threads REQUIRED)

add_library(feetech src/bus.cpp)
add_library(feetech::feetech ALIAS feetech)
target_include_directories(feetech PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(feetech PUBLIC Threads::Threads)
set_target_properties(feetech PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR}
  POSITION_INDEPENDENT_CODE ON
)

if(FEETECH_INSTALL)
  include(GNUInstallDirs)
  install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
  install(TARGETS feetech EXPORT feetech_driverTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  )
  install(EXPORT feetech_driverTargets
    FILE feetech_driverTargets.cmake
    NAMESPACE feetech::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/feetech_driver
  )

  include(CMakePackageConfigHelpers)
  write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/feetech_driverConfigVersion.cmake"
    COMPATIBILITY SameMajorVersion
  )
  configure_package_config_file(
    cmake/feetech_driverConfig.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/feetech_driverConfig.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/feetech_driver
  )
  install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/feetech_driverConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/feetech_driverConfigVersion.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/feetech_driver
  )
endif()

if(FEETECH_BUILD_PYTHON)
  find_package(Python 3.10 COMPONENTS Interpreter Development.Module REQUIRED)
  find_package(nanobind CONFIG REQUIRED)
  nanobind_add_module(_core STABLE_ABI NB_STATIC python/bindings.cpp)
  target_link_libraries(_core PRIVATE feetech)
  install(TARGETS _core LIBRARY DESTINATION feetech_driver)
endif()

if(FEETECH_BUILD_EXAMPLES)
  add_executable(feetech_scan_example examples/scan.cpp)
  target_link_libraries(feetech_scan_example PRIVATE feetech)
endif()

if(FEETECH_BUILD_TESTING)
  enable_testing()
  find_package(GTest REQUIRED)
  include(GoogleTest)

  add_executable(test_protocol test/test_protocol.cpp)
  target_link_libraries(test_protocol PRIVATE feetech GTest::gtest_main)
  gtest_discover_tests(test_protocol)

  # util: openpty
  add_executable(test_bus_sharing test/test_bus_sharing.cpp)
  target_link_libraries(test_bus_sharing PRIVATE feetech GTest::gtest_main util)
  gtest_discover_tests(test_bus_sharing)
endif()
