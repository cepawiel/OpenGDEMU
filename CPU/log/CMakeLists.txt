project(log)

add_library(${PROJECT_NAME} 
    src/log.cpp
    src/trice.c
    )

target_link_libraries(${PROJECT_NAME} 
    openGDEMU_Drivers
    argon
)

target_include_directories(${PROJECT_NAME}
    PUBLIC ${PROJECT_SOURCE_DIR}/include
    PRIVATE ${PROJECT_SOURCE_DIR}/src
)

target_compile_definitions(${PROJECT_NAME} PUBLIC
    TRICE_UART
)

add_custom_target(
    generateTrice
    COMMAND touch til.json && /home/coltonp/go/bin/trice update -src ${PROJECT_SOURCE_DIR}/../
)
add_dependencies(log generateTrice)