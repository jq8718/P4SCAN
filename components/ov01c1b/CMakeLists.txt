idf_component_register(
    SRCS "ov01c1b.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "private_include"
    REQUIRES esp_cam_sensor esp_sccb_intf esp_driver_gpio
)

target_link_libraries(${COMPONENT_LIB} INTERFACE "-u ov01c1b_detect")
