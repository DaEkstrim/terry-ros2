# MPU9250 sensor library
add_library(mpu9250 src/mpu9250.cpp)
target_include_directories(mpu9250 PUBLIC include/mpu9250)
target_link_libraries(mpu9250 PRIVATE i2c_comm_interface)