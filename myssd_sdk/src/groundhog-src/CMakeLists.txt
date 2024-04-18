cmake_minimum_required(VERSION 3.23)
project(traj_block_format C)

set(CMAKE_C_STANDARD 99)

include_directories("${PROJECT_BINARY_DIR}"
        "${PROJECT_SOURCE_DIR}/include"
        "${PROJECT_SOURCE_DIR}/lib"
        "${PROJECT_SOURCE_DIR}/lib/myssd_sdk"
        "${PROJECT_SOURCE_DIR}/lib/r5poll"
        )

# testing section
enable_testing()
add_subdirectory(test)

add_library(traj_block_format
        src/groundhog/library.c
        src/groundhog/normalization_util.c
        src/groundhog/traj_block_format.c
        src/groundhog/porto_dataset_reader.c
        src/groundhog/traj_processing.c
        src/groundhog/simple_index.c
        src/groundhog/simple_storage.c
        src/groundhog/simple_query_engine.c
        src/groundhog/hashset_int.c
        src/groundhog/query_verification_util.c
        src/groundhog/query_workload_reader.c
        src/groundhog/isp_descriptor.c
        src/groundhog/seg_meta_store.c
        src/groundhog/persistence_manager.c
        src/groundhog/common_fs_layer.c
        src/groundhog/isp_output_format.c
        src/groundhog/bloom/bloom.c
        src/groundhog/bloom/bitutil.c
        src/groundhog/bloom/hashf.c
        include/groundhog/normalization_util.h
        include/groundhog/traj_block_format.h
        include/groundhog/porto_dataset_reader.h
        include/groundhog/traj_processing.h
        include/groundhog/simple_index.h
        include/groundhog/simple_storage.h
        include/groundhog/simple_query_engine.h
        include/groundhog/hashset_int.h
        include/groundhog/query_verification_util.h
        include/groundhog/query_workload_reader.h
        include/groundhog/isp_descriptor.h
        include/groundhog/seg_meta_store.h
        include/groundhog/persistence_manager.h
        include/groundhog/common_fs_layer.h
        include/groundhog/isp_output_format.h
        include/groundhog/common_util.h
        include/groundhog/log.h
        include/groundhog/bloom/bloom.h
        include/groundhog/bloom/bitutil.h
        include/groundhog/bloom/hashf.h
        )


add_executable(mymain src/groundhog/mymain.c)
target_link_libraries(mymain ${PROJECT_SOURCE_DIR}/cmake-build-debug/libtraj_block_format.a)

add_executable(simple_verification src/groundhog/simple_verification.c)
target_link_libraries(simple_verification ${PROJECT_SOURCE_DIR}/cmake-build-debug/libtraj_block_format.a)
