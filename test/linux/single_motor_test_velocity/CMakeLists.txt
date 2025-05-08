
set(SOURCES single_motor_test_velocity.c)
add_executable(single_motor_test_velocity ${SOURCES})
target_link_libraries(single_motor_test_velocity soem)
install(TARGETS single_motor_test_velocity DESTINATION bin)
