add_executable(motion motion.cc)
target_link_libraries(motion
        ${PROJECT_NAME}.common
        ${PROJECT_NAME}.tools
        )

add_executable(motion_fx_parabola motion_fx_parabola.cc)
target_link_libraries(motion_fx_parabola
        ${PROJECT_NAME}.common
        ${PROJECT_NAME}.tools
        )
