add_library(hal-target-stm32h723vgt6 INTERFACE)
add_library(hal::stm32h723vgt6 ALIAS hal-target-stm32h723vgt6)
set_target_properties(hal-target-stm32h723vgt6
                      PROPERTIES EXPORT_NAME stm32h723vgt6)
target_link_libraries(hal-target-stm32h723vgt6 INTERFACE hal::devices)
target_include_directories(hal-target-stm32h723vgt6 INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../../mcu/stm32h723vgt6/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/ecu-drivers/targets/stm32h7>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/ecu-drivers/profiles/stm32h723vgt6>)
target_compile_definitions(hal-target-stm32h723vgt6 INTERFACE STM32H723xx=1)
target_include_directories(hal-target-stm32h723vgt6 SYSTEM INTERFACE
    $<BUILD_INTERFACE:${ECU_DRIVERS_THIRD_PARTY}/cmsis-device-h7/Include>
    $<BUILD_INTERFACE:${ECU_DRIVERS_THIRD_PARTY}/cmsis-core/CMSIS/Core/Include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/ecu-drivers/third_party/cmsis-device-h7>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/ecu-drivers/third_party/cmsis-core>)

install(DIRECTORY include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ecu-drivers/targets/stm32h7)
install(DIRECTORY ../../../mcu/stm32h723vgt6/include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ecu-drivers/profiles/stm32h723vgt6)
install(DIRECTORY ${ECU_DRIVERS_THIRD_PARTY}/cmsis-device-h7/Include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ecu-drivers/third_party/cmsis-device-h7)
install(DIRECTORY ${ECU_DRIVERS_THIRD_PARTY}/cmsis-core/CMSIS/Core/Include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ecu-drivers/third_party/cmsis-core)

if(ECU_DRIVERS_BUILD_TESTS AND BUILD_TESTING)
    add_executable(hal_stm32h7_device_profile_tests tests/device_profile.cpp)
    target_link_libraries(hal_stm32h7_device_profile_tests
                          PRIVATE hal::stm32h723vgt6)
    ecu_drivers_enable_strict_warnings(hal_stm32h7_device_profile_tests)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(hal_stm32h7_device_profile_tests PRIVATE
                               -Wno-error=conversion -Wno-error=sign-conversion)
    endif()
    add_test(NAME hal_stm32h7_device_profile
             COMMAND hal_stm32h7_device_profile_tests)
    set_tests_properties(hal_stm32h7_device_profile
                         PROPERTIES LABELS "compile_validated;register_model_validated")

    # The exact template-instantiation test touches CMSIS cache helpers that
    # intentionally use 32-bit pointer casts. Build it with the target
    # toolchain, where those casts are valid, rather than weakening a native
    # host build with -fpermissive.
    if(CMAKE_CROSSCOMPILING)
        add_library(hal_stm32h7_device_profile_exact_tests OBJECT
                    tests/device_profile_exact.cpp)
        target_link_libraries(hal_stm32h7_device_profile_exact_tests
                              PRIVATE hal::stm32h723vgt6)
        ecu_drivers_enable_strict_warnings(hal_stm32h7_device_profile_exact_tests)
        if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
            target_compile_options(hal_stm32h7_device_profile_exact_tests PRIVATE
                                   -Wno-error=conversion -Wno-error=sign-conversion)
        endif()
        # This source contains CMSIS MMIO addresses and is deliberately a
        # compile-only target. Do not let native CTest try to execute it.
    endif()
endif()
