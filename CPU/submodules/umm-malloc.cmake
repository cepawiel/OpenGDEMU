project(umm)

add_library(${PROJECT_NAME} STATIC
    # UMM Malloc
    ${PROJECT_SOURCE_DIR}/umm_malloc/src/umm_info.c
    ${PROJECT_SOURCE_DIR}/umm_malloc/src/umm_integrity.c
    ${PROJECT_SOURCE_DIR}/umm_malloc/src/umm_malloc.c
    ${PROJECT_SOURCE_DIR}/umm_malloc/src/umm_poison.c
)
    
target_link_libraries(${PROJECT_NAME} 

) 

target_include_directories(${PROJECT_NAME}
    PUBLIC ${PROJECT_SOURCE_DIR}/umm_malloc/src/
)

