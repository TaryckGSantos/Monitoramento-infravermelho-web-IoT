idf_component_register(SRCS "station_example_main.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_wifi esp_event esp_netif nvs_flash esp_http_server esp_driver_uart driver)
