set(page_gzip "${CMAKE_CURRENT_BINARY_DIR}/car_page.html.gz")
execute_process(COMMAND "${PYTHON}" "${CMAKE_CURRENT_LIST_DIR}/compress_page.py"
    "${CMAKE_CURRENT_LIST_DIR}/index.html" "${page_gzip}" COMMAND_ERROR_IS_FATAL ANY)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${CMAKE_CURRENT_LIST_DIR}/index.html")
idf_component_register(
    SRCS "server.cpp"
    INCLUDE_DIRS "include"
    REQUIRES vehicle recording esp_http_server
    PRIV_REQUIRES esp_wifi esp_event esp_netif espressif__cjson esp_timer nvs_flash
    EMBED_FILES "${page_gzip}"
)
target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_17)
