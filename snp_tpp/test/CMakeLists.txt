# Test compilation of library components to check for undefined symbols, etc.
add_executable(test_compile test_compile.cpp)
target_link_libraries(test_compile ${PROJECT_NAME})
