project(traj_block_format)

add_subdirectory(googletest)

include_directories(${gtest_SOURCE_DIR}/include
        ${gtest_SOURCE_DIR}
        ${CMAKE_SOURCE_DIR}/../include
        ${CMAKE_SOURCE_DIR}/../lib
        )

add_executable(mytest mytest.cpp)
target_link_libraries(mytest gtest gtest_main)
target_link_libraries(mytest ${CMAKE_SOURCE_DIR}/cmake-build-debug/libtraj_block_format.a)


add_executable(traj_block_format_test traj_block_format_test.cpp)
target_link_libraries(traj_block_format_test gtest gtest_main)
target_link_libraries(traj_block_format_test ${CMAKE_SOURCE_DIR}/cmake-build-debug/libtraj_block_format.a)

add_executable(porto_dataset_read_test porto_dataset_read_test.cpp)
target_link_libraries(porto_dataset_read_test gtest gtest_main)
target_link_libraries(porto_dataset_read_test ${CMAKE_SOURCE_DIR}/cmake-build-debug/libtraj_block_format.a)

add_executable(traj_processing_test traj_processing_test.cpp)
target_link_libraries(traj_processing_test gtest gtest_main)
target_link_libraries(traj_processing_test ${CMAKE_SOURCE_DIR}/cmake-build-debug/libtraj_block_format.a)

add_executable(hashset_int_test  hashset_int_test.cpp)
target_link_libraries(hashset_int_test gtest gtest_main)
target_link_libraries(hashset_int_test ${CMAKE_SOURCE_DIR}/cmake-build-debug/libtraj_block_format.a)

add_executable(simple_storage_test simple_storage_test.cpp)
target_link_libraries(simple_storage_test gtest gtest_main)
target_link_libraries(simple_storage_test ${CMAKE_SOURCE_DIR}/cmake-build-debug/libtraj_block_format.a)


add_executable(simple_index_test simple_index_test.cpp)
target_link_libraries(simple_index_test gtest gtest_main)
target_link_libraries(simple_index_test ${CMAKE_SOURCE_DIR}/cmake-build-debug/libtraj_block_format.a)


add_executable(simple_query_engine_test simple_query_engine_test.cpp)
target_link_libraries(simple_query_engine_test gtest gtest_main)
target_link_libraries(simple_query_engine_test ${CMAKE_SOURCE_DIR}/cmake-build-debug/libtraj_block_format.a)

add_executable(query_workload_reader_test query_workload_reader_test.cpp)
target_link_libraries(query_workload_reader_test gtest gtest_main)
target_link_libraries(query_workload_reader_test ${CMAKE_SOURCE_DIR}/cmake-build-debug/libtraj_block_format.a)

add_executable(isp_descriptor_test isp_descriptor_test.cpp)
target_link_libraries(isp_descriptor_test gtest gtest_main)
target_link_libraries(isp_descriptor_test ${CMAKE_SOURCE_DIR}/cmake-build-debug/libtraj_block_format.a)

add_executable(seg_meta_store_test seg_meta_store_test.cpp)
target_link_libraries(seg_meta_store_test gtest gtest_main)
target_link_libraries(seg_meta_store_test ${CMAKE_SOURCE_DIR}/cmake-build-debug/libtraj_block_format.a)

add_executable(fs_api_test fs_api_test.cpp)
target_link_libraries(fs_api_test gtest gtest_main)

add_executable(isp_output_buffer isp_output_buffer_test.cpp
        ${CMAKE_SOURCE_DIR}/src/groundhog/isp_output_buffer.c
        ${CMAKE_SOURCE_DIR}/include/groundhog/isp_output_buffer.h)
target_link_libraries(isp_output_buffer gtest gtest_main ${CMAKE_SOURCE_DIR}/cmake-build-debug/libtraj_block_format.a)
