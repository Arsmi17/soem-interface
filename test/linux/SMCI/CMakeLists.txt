cmake_minimum_required(VERSION 3.10)
project(SMCI C)

# Set C standard
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Set source files
set(SOURCES
    shared/app_context.c
    shared/ethercat_shared.c
    shared/slave_selector.c 
    shared/ethercat_comm.c 
    
    src/main.c
    src/templates.c
    src/position_test.c
    src/velocity_test.c
)

# Add executable target
add_executable(SMCI ${SOURCES})

# Include directories
target_include_directories(SMCI
    PRIVATE
    ${LIBLO_INCLUDE_DIRS}
    ${GTK3_INCLUDE_DIRS}
    ${SOEM_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/shared
    ${JSON_GLIB_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(SMCI
    PRIVATE
    soem
    ${LIBLO_LIBRARIES}
    ${GTK3_LIBRARIES}
    ${JSON_GLIB_LIBRARIES}
    shlwapi
)

# Copy resource folders to build directory for runtime discovery
set(RESOURCE_OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR})
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/builders DESTINATION ${RESOURCE_OUTPUT_DIR})
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/css DESTINATION ${RESOURCE_OUTPUT_DIR})

# Install the binary
install(TARGETS SMCI DESTINATION bin)


find_package(PkgConfig REQUIRED)
pkg_check_modules(JSON_GLIB REQUIRED json-glib-1.0)

# Install UI files
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/builders DESTINATION share/smci)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/css DESTINATION share/smci)
