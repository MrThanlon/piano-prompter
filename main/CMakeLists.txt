set(CMAKE_COMPILE_COMMANDS On)

idf_component_register(SRCS "hello_world_main.c"
                       PRIV_REQUIRES spi_flash esp_driver_spi esp_driver_gpio
                       INCLUDE_DIRS "")
