project(cmsis)

add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME} 
    INTERFACE ${PROJECT_SOURCE_DIR}/common/include
    INTERFACE ${PROJECT_SOURCE_DIR}/sam3u/include
)

target_compile_definitions(${PROJECT_NAME} INTERFACE
    __SAM3U4E__
)