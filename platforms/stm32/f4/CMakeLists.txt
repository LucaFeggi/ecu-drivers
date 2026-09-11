add_library(hal-target-stm32f4 INTERFACE)
target_compile_features(hal-target-stm32f4 INTERFACE cxx_std_20)
target_include_directories(hal-target-stm32f4 INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../../include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../../mcu/stm32f446vet6/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

add_library(hal-target-stm32f446vet6 INTERFACE)
add_library(hal::stm32f446vet6 ALIAS hal-target-stm32f446vet6)
set_target_properties(hal-target-stm32f446vet6
                      PROPERTIES EXPORT_NAME stm32f446vet6)
target_include_directories(hal-target-stm32f446vet6 INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../../include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../../mcu/stm32f446vet6/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_compile_definitions(hal-target-stm32f446vet6 INTERFACE STM32F446xx=1)
target_include_directories(hal-target-stm32f446vet6 SYSTEM INTERFACE
    $<BUILD_INTERFACE:${ECU_DRIVERS_THIRD_PARTY}/cmsis-device-f4/Include>
    $<BUILD_INTERFACE:${ECU_DRIVERS_THIRD_PARTY}/cmsis-core/CMSIS/Core/Include>)

if(ECU_DRIVERS_BUILD_TESTS AND BUILD_TESTING)
    add_executable(hal_stm32f4_device_profile_tests tests/device_profile.cpp)
    target_link_libraries(hal_stm32f4_device_profile_tests
                          PRIVATE hal::stm32f446vet6)
    ecu_drivers_enable_strict_warnings(hal_stm32f4_device_profile_tests)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(hal_stm32f4_device_profile_tests PRIVATE
                               -Wno-error=conversion -Wno-error=sign-conversion)
    endif()
    add_test(NAME hal_stm32f4_device_profile
             COMMAND hal_stm32f4_device_profile_tests)
    set_tests_properties(hal_stm32f4_device_profile
                         PROPERTIES LABELS "compile;register_model")
endif()
