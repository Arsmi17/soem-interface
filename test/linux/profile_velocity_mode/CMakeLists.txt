
set(SOURCES profile_velocity_mode.c)
add_executable(profile_velocity_mode ${SOURCES})
target_link_libraries(profile_velocity_mode soem)
install(TARGETS profile_velocity_mode DESTINATION bin)
