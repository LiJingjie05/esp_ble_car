# 查找 src 目录中的所有 .c 文件
file(GLOB SRCS "src/*.c")

idf_component_register(SRCS ${SRCS}
                       INCLUDE_DIRS "inc"
                       REQUIRES driver)
