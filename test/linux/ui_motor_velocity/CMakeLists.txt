
find_package(PkgConfig REQUIRED)
pkg_check_modules(GTK3 REQUIRED gtk+-3.0)

link_directories(/opt/homebrew/lib)
set(SOURCES ui_motor_velocity.c)
add_executable(motor_control ${SOURCES})

# Link with SOEM and GTK+ 3.0 libraries
target_link_libraries(motor_control PRIVATE soem ${GTK3_LIBRARIES})

# Include directories for GTK+ 3.0
target_include_directories(motor_control PRIVATE ${GTK3_INCLUDE_DIRS})

# Add GTK+ 3.0 compiler flags
target_compile_options(motor_control PRIVATE ${GTK3_CFLAGS_OTHER})

# Install the executable
install(TARGETS motor_control DESTINATION bin)