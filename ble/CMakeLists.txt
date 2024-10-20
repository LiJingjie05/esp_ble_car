file(GLOB SRCS "src/*.c")

idf_component_register(SRCS ${SRCS}
                        INCLUDE_DIRS "inc"
                        INCLUDE_DIRS "."
                        REQUIRES  bt nvs_flash)
