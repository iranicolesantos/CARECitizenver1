idf_component_register(SRCS "main.c"
                            "nmea_parser.c"
                            "sen5x_i2c.c" 
                            "sensirion_common.c" 
                            "sensirion_i2c.c" 
                            "sensirion_i2c_hal.c"
                            "cjson.c"
                            "esp_qrcode_main.c" 
                            "esp_qrcode_wrapper.c" 
                            "qrcodegen.c"
                            "mhz19b.c"
                    INCLUDE_DIRS ".")
set(CERT_FILE "mqtt_eclipseprojects_io.pem")
set(CERT_FILE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/${CERT_FILE}")

if(EXISTS "${CERT_FILE_PATH}")
    message(STATUS "Including certificate file: ${CERT_FILE_PATH}")
    target_add_binary_data(${COMPONENT_LIB} "mqtt_eclipseprojects_io.pem" TEXT)
else()
    message(FATAL_ERROR "Certificate file not found: ${CERT_FILE_PATH}")
endif()