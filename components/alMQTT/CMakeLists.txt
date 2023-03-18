idf_component_register(SRCS "pal_prop_set_api.c" "pal_prop_post_api.c" "alMQTT.c"
                    INCLUDE_DIRS "include"
                    REQUIRES "LinkSDK" "esp_wifi" "nvs_flash" "cJSON")
