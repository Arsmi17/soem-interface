
set(SOURCES multi_motor_keyboard.c)
add_executable(multi_motor_keyboard ${SOURCES})
target_link_libraries(multi_motor_keyboard soem)
install(TARGETS multi_motor_keyboard DESTINATION bin)
