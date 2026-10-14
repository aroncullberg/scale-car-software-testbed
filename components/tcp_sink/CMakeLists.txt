idf_component_register(
    SRCS "tcp_sink.cpp"
    INCLUDE_DIRS "include"
    REQUIRES logger freertos
    PRIV_REQUIRES esp_wifi esp_netif esp_event nvs_flash lwip esp_hw_support
)
