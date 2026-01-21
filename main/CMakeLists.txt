idf_component_register(SRCS "oneshot_read_main.c"
                    PRIV_REQUIRES esp_adc
                    REQUIRES driver
                    INCLUDE_DIRS ".")
