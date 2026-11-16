find_package(OpenCV 4 REQUIRED)
find_package(Sophus CONFIG REQUIRED)

add_executable(dense_mapping dense_mapping.cpp)
target_compile_features(dense_mapping PRIVATE cxx_std_14)
target_compile_options(dense_mapping PRIVATE -march=native -O3)
target_include_directories(dense_mapping PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(dense_mapping PRIVATE ${OpenCV_LIBS} Sophus::Sophus)
