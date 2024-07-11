cmake_minimum_required(VERSION 3.0.2)
project(ImageProcessor)

# 设置C++标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 查找OpenCV包
find_package(OpenCV REQUIRED)

# 添加可执行文件
add_executable(image_processor pic_process.cpp)
add_executable(getHeightMapImage getHeightMapImage.cpp)
add_executable(crop_image crop_image.cpp)

# 链接OpenCV库
target_link_libraries(image_processor ${OpenCV_LIBS})
target_link_libraries(getHeightMapImage ${OpenCV_LIBS})
target_link_libraries(crop_image ${OpenCV_LIBS})
