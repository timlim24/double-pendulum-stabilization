idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "." "${CMAKE_CURRENT_SOURCE_DIR}/../../common"
                       REQUIRES esp_driver_pcnt esp_driver_ledc esp_driver_gpio esp_timer)
