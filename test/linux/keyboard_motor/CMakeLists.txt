
set(SOURCES keyboard_motor.c)
add_executable(keyboard_motor ${SOURCES})
target_link_libraries(keyboard_motor soem)
install(TARGETS keyboard_motor DESTINATION bin)
