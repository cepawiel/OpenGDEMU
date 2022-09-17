project(argon)
enable_language(ASM)

add_library(argon
    # Boot
    ${PROJECT_SOURCE_DIR}/system_sam3u.c
    ${PROJECT_SOURCE_DIR}/startup_sam3u.c
    ${PROJECT_SOURCE_DIR}/stdlib.cpp
    ${PROJECT_SOURCE_DIR}/init.c

    # RTOS
    ${PROJECT_SOURCE_DIR}/argon-rtos/src/ar_channel.cpp
    ${PROJECT_SOURCE_DIR}/argon-rtos/src/ar_kernel.cpp
    ${PROJECT_SOURCE_DIR}/argon-rtos/src/ar_mutex.cpp
    ${PROJECT_SOURCE_DIR}/argon-rtos/src/ar_queue.cpp
    ${PROJECT_SOURCE_DIR}/argon-rtos/src/ar_runloop.cpp
    ${PROJECT_SOURCE_DIR}/argon-rtos/src/ar_sem.cpp
    ${PROJECT_SOURCE_DIR}/argon-rtos/src/ar_thread.cpp
    ${PROJECT_SOURCE_DIR}/argon-rtos/src/ar_timer.cpp
    ${PROJECT_SOURCE_DIR}/argon-rtos/src/cortex_m/ar_main_thread.cpp
    ${PROJECT_SOURCE_DIR}/argon-rtos/src/cortex_m/ar_port.cpp
    ${PROJECT_SOURCE_DIR}/argon-rtos/src/cortex_m/ar_atomics_cm4.S
    ${PROJECT_SOURCE_DIR}/argon-rtos/src/cortex_m/ar_handlers_cm4_nofpu.S
)

target_link_libraries(argon
    umm
    cmsis
    log
)

target_include_directories(argon
    PUBLIC ${PROJECT_SOURCE_DIR}/argon-rtos/include/
    PUBLIC ${PROJECT_SOURCE_DIR}/argon-rtos/src/  
    PUBLIC ${PROJECT_SOURCE_DIR}/argon-rtos/src/cortex_m/
)

target_compile_definitions(argon PUBLIC
    __SAM3U4E__
    UMM_ENABLE
    UMM_BEST_FIT
)

target_link_options(argon PUBLIC
    -Wl,-u Reset_Handler
)