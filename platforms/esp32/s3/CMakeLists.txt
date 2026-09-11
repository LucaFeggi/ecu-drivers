set(ECU_ESP32S3_IDF_ROOT
    "${ECU_DRIVERS_THIRD_PARTY}/esp-idf"
    CACHE PATH "ESP-IDF checkout used for ESP32-S3 register definitions")

if(NOT EXISTS "${ECU_ESP32S3_IDF_ROOT}/components/soc/esp32s3/register/soc")
    message(FATAL_ERROR
        "ESP32-S3 target requires an ESP-IDF 6.2 development checkout at "
        "${ECU_ESP32S3_IDF_ROOT}")
endif()

file(READ "${ECU_ESP32S3_IDF_ROOT}/components/esp_common/include/esp_idf_version.h"
     ECU_ESP32S3_IDF_VERSION_HEADER)
if(NOT ECU_ESP32S3_IDF_VERSION_HEADER MATCHES
   "ESP_IDF_VERSION_MAJOR[ \t]+6" OR
   NOT ECU_ESP32S3_IDF_VERSION_HEADER MATCHES
   "ESP_IDF_VERSION_MINOR[ \t]+2")
    message(FATAL_ERROR
        "ESP32-S3 target requires the ESP-IDF 6.2 development register checkout")
endif()

add_library(hal-target-esp32s3_wroom_1_n16r8 INTERFACE)
add_library(hal::esp32s3_wroom_1_n16r8 ALIAS
            hal-target-esp32s3_wroom_1_n16r8)
set_target_properties(hal-target-esp32s3_wroom_1_n16r8
                      PROPERTIES EXPORT_NAME esp32s3_wroom_1_n16r8)
target_compile_features(hal-target-esp32s3_wroom_1_n16r8 INTERFACE cxx_std_20)
target_link_libraries(hal-target-esp32s3_wroom_1_n16r8 INTERFACE hal::devices)
target_compile_definitions(hal-target-esp32s3_wroom_1_n16r8 INTERFACE
    ECU_HAL_ESP32S3_WROOM_1_N16R8=1)
target_include_directories(hal-target-esp32s3_wroom_1_n16r8 INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../../mcu/esp32s3_wroom_1_n16r8/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/ecu-drivers/targets/esp32s3>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/ecu-drivers/profiles/esp32s3_wroom_1_n16r8>)

# Only raw SoC register descriptions are exposed. No ESP-IDF peripheral
# driver, LL layer, FreeRTOS header, or IDF handle type is part of this target.
target_include_directories(hal-target-esp32s3_wroom_1_n16r8 SYSTEM INTERFACE
    $<BUILD_INTERFACE:${ECU_ESP32S3_IDF_ROOT}/components/soc/esp32s3/include>
    $<BUILD_INTERFACE:${ECU_ESP32S3_IDF_ROOT}/components/soc/esp32s3/register>
    $<BUILD_INTERFACE:${ECU_ESP32S3_IDF_ROOT}/components/esp_common/include>
    $<BUILD_INTERFACE:${ECU_ESP32S3_IDF_ROOT}/components/hal/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/ecu-drivers/third_party/esp-idf/soc/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/ecu-drivers/third_party/esp-idf/soc/register>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/ecu-drivers/third_party/esp-idf/esp_common/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/ecu-drivers/third_party/esp-idf/hal/include>)

install(DIRECTORY include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ecu-drivers/targets/esp32s3)
install(DIRECTORY ../../../mcu/esp32s3_wroom_1_n16r8/include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ecu-drivers/profiles/esp32s3_wroom_1_n16r8)
install(DIRECTORY ${ECU_ESP32S3_IDF_ROOT}/components/soc/esp32s3/include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ecu-drivers/third_party/esp-idf/soc/include)
install(DIRECTORY ${ECU_ESP32S3_IDF_ROOT}/components/soc/esp32s3/register/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ecu-drivers/third_party/esp-idf/soc/register)
install(DIRECTORY ${ECU_ESP32S3_IDF_ROOT}/components/esp_common/include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ecu-drivers/third_party/esp-idf/esp_common/include)
install(DIRECTORY ${ECU_ESP32S3_IDF_ROOT}/components/hal/include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ecu-drivers/third_party/esp-idf/hal/include)

if(ECU_DRIVERS_BUILD_TESTS AND BUILD_TESTING)
    add_executable(hal_esp32s3_device_profile_tests tests/device_profile.cpp)
    target_link_libraries(hal_esp32s3_device_profile_tests PRIVATE
                          hal::esp32s3_wroom_1_n16r8)
    ecu_drivers_enable_strict_warnings(hal_esp32s3_device_profile_tests)
    add_test(NAME hal_esp32s3_device_profile
             COMMAND hal_esp32s3_device_profile_tests)
    set_tests_properties(hal_esp32s3_device_profile
                         PROPERTIES LABELS "compile_validated;register_model_validated")
endif()
