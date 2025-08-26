set(SOURCES movement_dynamic.c)
add_executable(dell_hmrs ${SOURCES})
target_include_directories(dell_hmrs PRIVATE ${LIBLO_INCLUDE_DIRS} ${GTK3_INCLUDE_DIRS})
target_link_libraries(dell_hmrs PRIVATE soem ${LIBLO_LIBRARIES} ${GTK3_LIBRARIES})

# Install binary
install(TARGETS dell_hmrs DESTINATION bin)

# Install UI file
install(FILES interface.ui DESTINATION dell_hmrs)
