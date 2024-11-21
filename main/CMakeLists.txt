set(srcs "main.c"
         "ble_m3.c"
         "esp_hid_gap.c")
set(include_dirs ".")

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "${include_dirs}"
                       REQUIRES esp_hid led_strip
                       PRIV_REQUIRES nvs_flash)