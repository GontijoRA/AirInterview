cmake_minimum_required (VERSION 2.6)
project (airsquire)

find_package (OpenCV REQUIRED) 
find_package (PCL 1.7.2 REQUIRED)

include_directories (${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable ( airsquire main.cpp functions.cpp )
target_link_libraries ( airsquire
   ${OpenCV_LIBS}
   ${PCL_COMMON_LIBRARIES} 
   ${PCL_IO_LIBRARIES}	 
   ${PCL_LIBRARIES}
)
