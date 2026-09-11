add_library(hal-target-stm32g4 INTERFACE)
target_compile_features(hal-target-stm32g4 INTERFACE cxx_std_20)
target_include_directories(hal-target-stm32g4 INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../../include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../../mcu/stm32g484ce/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

add_library(hal-target-stm32g484ce INTERFACE)
add_library(hal::stm32g484ce ALIAS hal-target-stm32g484ce)
set_target_properties(hal-target-stm32g484ce
                      PROPERTIES EXPORT_NAME stm32g484ce)
target_include_directories(hal-target-stm32g484ce INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../../include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../../mcu/stm32g484ce/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_compile_definitions(hal-target-stm32g484ce INTERFACE STM32G484xx=1)
target_include_directories(hal-target-stm32g484ce SYSTEM INTERFACE
    $<BUILD_INTERFACE:${ECU_DRIVERS_THIRD_PARTY}/cmsis-device-g4/Include>
    $<BUILD_INTERFACE:${ECU_DRIVERS_THIRD_PARTY}/cmsis-core/CMSIS/Core/Include>)

if(ECU_DRIVERS_BUILD_TESTS AND BUILD_TESTING)
    add_executable(hal_stm32g4_device_profile_tests tests/device_profile.cpp)
    target_link_libraries(hal_stm32g4_device_profile_tests
                          PRIVATE hal::stm32g484ce)
    ecu_drivers_enable_strict_warnings(hal_stm32g4_device_profile_tests)
    add_test(NAME hal_stm32g4_device_profile
             COMMAND hal_stm32g4_device_profile_tests)
    set_tests_properties(hal_stm32g4_device_profile
                         PROPERTIES LABELS "compile;register_model")

    if(CMAKE_CROSSCOMPILING)
        add_library(hal_stm32g4_device_profile_exact_tests OBJECT
                    tests/device_profile_exact.cpp)
        target_link_libraries(hal_stm32g4_device_profile_exact_tests
                              PRIVATE hal::stm32g484ce)
        ecu_drivers_enable_strict_warnings(hal_stm32g4_device_profile_exact_tests)
        # This source contains CMSIS MMIO addresses and is deliberately a
        # compile-only target. Do not let native CTest try to execute it.
    endif()
endif()
