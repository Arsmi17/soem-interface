# Declare this directory as part of a subproject
include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/..
)

# Define source and headers
set(SOURCES
    slave_detector_api.c
)

set(HEADERS
    slave_detector_api.h
)

# Build shared library (you can use STATIC instead)
add_library(slave_detector_api SHARED ${SOURCES} ${HEADERS})

# Link SOEM and dependencies
target_link_libraries(slave_detector_api PRIVATE soem pthread)

# Optional: Install headers and lib
install(TARGETS slave_detector_api
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(FILES ${HEADERS}
    DESTINATION include/slave_detector_api
)