function(hal_core_add_test test_name source_file)
    add_executable(${test_name} ${source_file})
    target_link_libraries(${test_name} PRIVATE hal::core)
    set_target_properties(${test_name} PROPERTIES CXX_EXTENSIONS OFF)

    if(MSVC)
        target_compile_options(${test_name} PRIVATE /W4 /WX /GR-)
    else()
        target_compile_options(
            ${test_name}
            PRIVATE
                -Wall
                -Wextra
                -Wpedantic
                -Wconversion
                -Wsign-conversion
                -Werror
                -fno-exceptions
                -fno-rtti)

        if(HAL_CORE_ENABLE_SANITIZERS)
            target_compile_options(
                ${test_name}
                PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
            target_link_options(
                ${test_name}
                PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
        endif()
    endif()

    add_test(NAME ${test_name} COMMAND ${test_name})
    set_tests_properties(${test_name} PROPERTIES LABELS "host_validated")
endfunction()

hal_core_add_test(hal_core_contract_adc ../contracts/adc.cpp)
hal_core_add_test(hal_core_contract_block ../contracts/block.cpp)
hal_core_add_test(hal_core_contract_can ../contracts/can.cpp)
hal_core_add_test(hal_core_contract_ethernet ../contracts/ethernet.cpp)
hal_core_add_test(hal_core_contract_gpio ../contracts/gpio.cpp)
hal_core_add_test(hal_core_active_low_gpio ../adapters/inverted_gpio.cpp)
hal_core_add_test(hal_core_contract_i2c ../contracts/i2c.cpp)
hal_core_add_test(hal_core_contract_nv ../contracts/nv.cpp)
hal_core_add_test(hal_core_journal_memory ../adapters/journal_memory.cpp)
hal_core_add_test(hal_core_contract_pwm ../contracts/pwm.cpp)
hal_core_add_test(hal_core_pwm_duty_cycle ../adapters/pwm_duty_cycle.cpp)
hal_core_add_test(hal_core_contract_rtc ../contracts/rtc.cpp)
hal_core_add_test(hal_core_contract_serial ../contracts/serial.cpp)
hal_core_add_test(hal_core_text_writer ../adapters/text_writer.cpp)
hal_core_add_test(hal_core_contract_spi ../contracts/spi.cpp)
hal_core_add_test(hal_core_static_spi_device ../adapters/static_spi_device.cpp)
hal_core_add_test(hal_core_contract_time ../contracts/time.cpp)
hal_core_add_test(hal_core_contract_watchdog ../contracts/watchdog.cpp)
hal_core_add_test(hal_core_primitives core_tests.cpp)
