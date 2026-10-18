set(srcs "main.c" "asha_web.c" "asha_protocol.c" "asha_test_audio.cpp" "g722/g722_encode.cc")

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES usb_device_uac esp_wifi esp_event esp_netif esp_http_server nvs_flash bt)
