# Install script for directory: C:/Users/lr/.platformio/packages/framework-espidf

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "C:/Program Files (x86)/mimirOpenV5")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "TRUE")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/xtensa/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/mbedtls/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/efuse/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/bootloader_support/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/partition_table/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/app_update/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/esp_ipc/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/spi_flash/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/esp_system/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/esp_rom/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/soc/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/vfs/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/esp_eth/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/tcpip_adapter/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/esp_netif/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/esp_event/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/wpa_supplicant/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/nvs_flash/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/esp_wifi/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/lwip/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/log/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/heap/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/esp_ringbuf/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/driver/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/pthread/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/espcoredump/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/esp32s2/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/esp_common/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/esp_timer/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/freertos/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/newlib/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/cxx/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/app_trace/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/asio/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/bootloader/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/bt/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/cbor/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/coap/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/console/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/nghttp/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/esp-tls/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/esp_adc_cal/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/esp_gdbstub/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/esp_hid/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/tcp_transport/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/esp_http_client/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/esp_http_server/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/esp_https_ota/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/esp_https_server/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/protobuf-c/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/protocomm/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/mdns/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/esp_local_ctrl/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/sdmmc/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/esp_serial_slave_link/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/esp_websocket_client/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/esptool_py/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/expat/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/wear_levelling/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/fatfs/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/freemodbus/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/idf_test/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/jsmn/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/json/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/libsodium/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/mqtt/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/openssl/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/perfmon/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/spiffs/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/tinyusb/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/ulp/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/unity/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/wifi_provisioning/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/__pio_env/cmake_install.cmake")
  include("C:/Users/lr/Documents/GitHub/mimirOpen/mimirOpenV5/.pio/build/esp32-s2-saola-1/esp-idf/src/cmake_install.cmake")

endif()

