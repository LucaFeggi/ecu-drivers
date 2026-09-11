cmake_minimum_required(VERSION 3.25)

project(ecu-drivers LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

set(ECU_DRIVERS_THIRD_PARTY
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party"
    CACHE PATH "Third-party target support dependencies")

function(ecu_drivers_enable_strict_warnings target)
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4 /WX /GR-)
    elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(${target} PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion
            -Werror -fno-exceptions -fno-rtti)
        if(ECU_DRIVERS_ENABLE_SANITIZERS)
            target_compile_options(${target} PRIVATE
                -fsanitize=address,undefined -fno-omit-frame-pointer)
            target_link_options(${target} PRIVATE
                -fsanitize=address,undefined -fno-omit-frame-pointer)
        endif()
    endif()
endfunction()

add_library(hal-core INTERFACE)
add_library(hal::core ALIAS hal-core)
set_target_properties(hal-core PROPERTIES EXPORT_NAME core)
target_compile_features(hal-core INTERFACE cxx_std_20)
target_include_directories(hal-core INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

add_library(hal-devices INTERFACE)
add_library(hal::devices ALIAS hal-devices)
set_target_properties(hal-devices PROPERTIES EXPORT_NAME devices)
target_compile_features(hal-devices INTERFACE cxx_std_20)
target_link_libraries(hal-devices INTERFACE hal::core)

option(ECU_DRIVERS_BUILD_TESTS "Build ecu-drivers tests" ${PROJECT_IS_TOP_LEVEL})
option(ECU_DRIVERS_ENABLE_SANITIZERS
       "Enable AddressSanitizer and UndefinedBehaviorSanitizer" OFF)
set(HAL_CORE_ENABLE_SANITIZERS ${ECU_DRIVERS_ENABLE_SANITIZERS}
    CACHE BOOL "Compatibility setting for the core test suite")

if(ECU_DRIVERS_BUILD_TESTS)
    include(CTest)
endif()

add_subdirectory(platforms/linux)
add_subdirectory(platforms/stm32/f4)
add_subdirectory(platforms/stm32/h5)
add_subdirectory(platforms/stm32/h7)
add_subdirectory(platforms/stm32/g4)
add_subdirectory(platforms/esp32/s3)
add_subdirectory(devices)

if(ECU_DRIVERS_BUILD_TESTS AND BUILD_TESTING)
    add_subdirectory(tests/core)
endif()

install(TARGETS
    hal-target-linux
    hal-target-stm32f446vet6
    hal-target-stm32h563vit6
    hal-target-stm32h723vgt6
    hal-target-stm32g484ce
    hal-target-esp32s3-wroom-1-n16r8
    EXPORT ecuDriversTargets)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(DIRECTORY devices/include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(DIRECTORY mcu/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
        FILES_MATCHING PATTERN "*.hpp")
foreach(_platform
        platforms/linux
        platforms/esp32/s3
        platforms/stm32/f4
        platforms/stm32/g4
        platforms/stm32/h5
        platforms/stm32/h7)
    install(DIRECTORY ${_platform}/include/
            DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
endforeach()
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/platforms/esp32/s3/generated")
    install(DIRECTORY platforms/esp32/s3/generated/
            DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
endif()
install(FILES LICENSE README.md
        DESTINATION ${CMAKE_INSTALL_DOCDIR})
install(FILES docs/System_Architecture.md
        DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/doc/ecu-drivers)
install(EXPORT ecuDriversTargets
        FILE ecu-driversTargets.cmake
        NAMESPACE hal::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ecu-drivers)

configure_package_config_file(
    cmake/ecu-driversConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/ecu-driversConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ecu-drivers)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/ecu-driversConfig.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ecu-drivers)
