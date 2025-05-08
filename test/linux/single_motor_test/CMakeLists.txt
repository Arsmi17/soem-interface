
set(SOURCES single_motor_test.c)
add_executable(single_motor_test ${SOURCES})
target_link_libraries(single_motor_test soem)
install(TARGETS single_motor_test DESTINATION bin)
