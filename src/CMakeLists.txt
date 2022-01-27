# Library
add_library(${LIB_NAME} STATIC ICM20649.cpp)

target_include_directories(${LIB_NAME} PUBLIC ${${LIB_NAME}_INCLUDE_DIR})
target_link_libraries(${LIB_NAME} i2c)

# Function to add new executables
function(add_new_executable ARG)
  add_executable(${ARG} ${ARG}.cpp)
  target_link_libraries(${ARG} ${LIB_NAME})
endfunction()

# List of executables
add_new_executable(display_measurements)
