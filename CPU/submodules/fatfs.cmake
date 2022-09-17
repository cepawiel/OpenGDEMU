project(fatfs)

add_library(${PROJECT_NAME} 
    ${PROJECT_SOURCE_DIR}/fatfs/source/diskio_sam3u.c
    ${PROJECT_SOURCE_DIR}/fatfs/source/ff.c
    ${PROJECT_SOURCE_DIR}/fatfs/source/ffsystem.c
    ${PROJECT_SOURCE_DIR}/fatfs/source/ffunicode.c
)

target_link_libraries(${PROJECT_NAME} 
    openGDEMU_Drivers
)

target_include_directories(${PROJECT_NAME}
    PUBLIC ${PROJECT_SOURCE_DIR}/fatfs/source/
)

target_link_libraries(${PROJECT_NAME} 
    asf
)
