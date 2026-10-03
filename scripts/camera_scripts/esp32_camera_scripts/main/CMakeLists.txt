# No REQUIRES on purpose: the "main" component then sees every component, including the
# managed ones (esp32-camera, esp-tflite-micro, esp-nn) under their registry names.
idf_component_register(SRCS "main.c" "camera.c" "fc_uart.c" "proto.c" "tracker.c"
                            "policy.c" "situations.c" "infer.cpp" "model_data.c" "wifi.c"
                            "collect.c" "scan.c" "thermal.c"
                       INCLUDE_DIRS ".")
target_compile_options(${COMPONENT_LIB} PRIVATE -O2)
