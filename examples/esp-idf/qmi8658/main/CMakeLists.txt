set(APPS_DIR ./)
file(GLOB_RECURSE APPS_C_SRCS ${APPS_DIR}/*.c)

idf_component_register(SRCS ${APPS_C_SRCS}
                    INCLUDE_DIRS "."
                    REQUIRES bsp fatfs
                    )

# spiffs_create_partition_image(storage ../spiffs FLASH_IN_PROJECT)