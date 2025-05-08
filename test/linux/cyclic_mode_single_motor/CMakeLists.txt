
set(SOURCES cyclic_mode_single_motor.c)
add_executable(cyclic_mode_single_motor ${SOURCES})
target_link_libraries(cyclic_mode_single_motor soem)
install(TARGETS cyclic_mode_single_motor DESTINATION bin)
