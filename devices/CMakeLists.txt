# External-device drivers are header-only. The public aggregate target is
# created by the repository root so all package targets share one definition.

target_include_directories(hal-devices INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/ecu-drivers/devices>)

if(ECU_DRIVERS_BUILD_TESTS AND BUILD_TESTING)
    add_executable(hal_devices_st7735_test
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/st7735.cpp")
    target_link_libraries(hal_devices_st7735_test PRIVATE hal::devices)
    set_target_properties(hal_devices_st7735_test PROPERTIES CXX_EXTENSIONS OFF)
    ecu_drivers_enable_strict_warnings(hal_devices_st7735_test)
    add_test(NAME hal_devices_st7735 COMMAND hal_devices_st7735_test)
    set_tests_properties(hal_devices_st7735 PROPERTIES LABELS "host_validated")
endif()
