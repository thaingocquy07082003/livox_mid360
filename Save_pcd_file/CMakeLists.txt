cmake_minimum_required(VERSION 3.0) # Nâng phiên bản để tương thích tốt hơn
project(save_livox_to_pcd)

# Tìm PCL
find_package(PCL REQUIRED COMPONENTS common io) # Chỉ định các thành phần cần thiết

# Tìm Livox SDK2
find_library(LIVOX_LIDAR_SDK_LIBRARY 
             NAMES livox_lidar_sdk_static 
             PATHS /usr/local/lib
             NO_DEFAULT_PATH)
if(NOT LIVOX_LIDAR_SDK_LIBRARY)
  message(FATAL_ERROR "Không tìm thấy Livox SDK2 library. Vui lòng cài đặt Livox SDK2.")
endif()

# Tìm Boost (Livox SDK2 yêu cầu)
find_package(Boost REQUIRED COMPONENTS system thread)

# Thêm thư mục include
include_directories(
  ${PCL_INCLUDE_DIRS}
  /usr/local/include # Đường dẫn chính xác đến Livox SDK2
)

# Thêm thư mục link
link_directories(
  ${PCL_LIBRARY_DIRS}
  /usr/local/lib # Đảm bảo liên kết với thư viện Livox
)

# Thêm definitions của PCL
add_definitions(${PCL_DEFINITIONS})

# Tạo executable
add_executable(save_livox_to_pcd save_data_pcd.cpp)

# Liên kết thư viện
target_link_libraries(save_livox_to_pcd
  ${PCL_LIBRARIES}
  ${LIVOX_LIDAR_SDK_LIBRARY}
  Boost::system
  Boost::thread
)