cmake_minimum_required(VERSION 3.23)
project(traj_block_format C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_FLAGS_RELEASE "-O3")

include_directories("${PROJECT_BINARY_DIR}"
        "${PROJECT_SOURCE_DIR}/include"
        "${PROJECT_SOURCE_DIR}/lib"
        "${PROJECT_SOURCE_DIR}/lib/myssd_sdk"
        "${PROJECT_SOURCE_DIR}/lib/r5poll"
        )

link_directories("/home/yangguo/Codes/spdk/isa-l/.libs")
link_directories("${PROJECT_SOURCE_DIR}/lib/spdk")
link_directories("${PROJECT_SOURCE_DIR}/lib/dpdk")
link_directories("/home/yangguo/CLionProjects/spdk-test/cmake-build-debug")

# testing section
enable_testing()
add_subdirectory(test)

set(BEGIN_WHOLE_ARCHIVE -Wl,--whole-archive)
set(END_WHOLE_ARCHIVE -Wl,--no-whole-archive)
set(STATIC -Wl,-Bstatic)
set(DYNAMIC -Wl,-Bdynamic)
set(SPDKNVME_LIBRARIES
        -lspdk_nvme -lspdk_sock -lspdk_sock_posix -lspdk_trace -lspdk_rpc -lspdk_jsonrpc -lspdk_json -lspdk_util -lspdk_log -lspdk_vmd -lspdk_env_dpdk)
set(SPDKENVDPDK_LIBRARIES
        -lrte_eal -lrte_mempool -lrte_ring -lrte_mbuf -lrte_bus_pci -lrte_pci -lrte_mempool_ring -lrte_telemetry -lrte_kvargs -lrte_power -lrte_ethdev -lrte_net -lrte_vhost -lrte_dmadev -lrte_cryptodev -lrte_hash -lrte_rcu)
set(SPKDSYSLIBS_LIBRARIES
        -lisal -pthread -lrt -luuid -lssl -lcrypto -lm -laio -lnuma -ldl)
set(SPDK_LIBS
        ${BEGIN_WHOLE_ARCHIVE} ${STATIC} ${SPDKNVME_LIBRARIES} ${SPDKENVDPDK_LIBRARIES} ${END_WHOLE_ARCHIVE} ${DYNAMIC} ${SPKDSYSLIBS_LIBRARIES})


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
        src/groundhog/static_spdk_fs_layer.c
        src/groundhog/bloom/bitutil.c
        src/groundhog/bloom/bloom.c
        src/groundhog/bloom/hashf.c
        src/groundhog/nyc_dataset_reader.c
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
        include/groundhog/static_spdk_fs_layer.h
        include/groundhog/common_util.h
        include/groundhog/log.h
        include/groundhog/bloom/bitutil.h
        include/groundhog/bloom/bloom.h
        include/groundhog/bloom/hashf.h
        include/groundhog/nyc_dataset_reader.h
        include/groundhog/knn_util.h src/groundhog/knn_util.c
        include/groundhog/osm_dataset_reader.h src/groundhog/osm_dataset_reader.c src/common_util.c)


add_executable(mymain src/groundhog/mymain.c)
target_link_libraries(mymain ${PROJECT_SOURCE_DIR}/cmake-build-debug/libtraj_block_format.a)

add_executable(simple_verification src/groundhog/simple_verification.c)
target_link_libraries(simple_verification ${SPDK_LIBS} ${PROJECT_SOURCE_DIR}/cmake-build-debug/libtraj_block_format.a)

add_executable(exp_native_host_and_device src/groundhog/exp/exp_native_host_and_device.c)
target_link_libraries(exp_native_host_and_device ${SPDK_LIBS} ${PROJECT_SOURCE_DIR}/cmake-build-debug/libtraj_block_format.a)

add_executable(exp_spdk_exe_host src/groundhog/exp/exp_spdk_exe_host.c)
target_link_libraries(exp_spdk_exe_host ${SPDK_LIBS} ${PROJECT_SOURCE_DIR}/cmake-build-debug/libtraj_block_format.a)

add_executable(pre_spdk_write src/groundhog/exp/pre_spdk_write.c)
target_link_libraries(pre_spdk_write ${SPDK_LIBS} ${PROJECT_SOURCE_DIR}/cmake-build-debug/libtraj_block_format.a)

add_executable(exp_spdk_exe_isp src/groundhog/exp/exp_spdk_exe_isp.c)
target_link_libraries(exp_spdk_exe_isp ${SPDK_LIBS} ${PROJECT_SOURCE_DIR}/cmake-build-debug/libtraj_block_format.a)

add_executable(exp_spdk_exe_isp_multi src/groundhog/exp/exp_spdk_exe_isp_multi.c)
target_link_libraries(exp_spdk_exe_isp_multi ${SPDK_LIBS} ${PROJECT_SOURCE_DIR}/cmake-build-debug/libtraj_block_format.a)

add_executable(exp_spdk_exe_isp_fpga src/groundhog/exp/exp_spdk_exe_isp_fpga.c)
target_link_libraries(exp_spdk_exe_isp_fpga ${SPDK_LIBS} ${PROJECT_SOURCE_DIR}/cmake-build-debug/libtraj_block_format.a)

add_executable(exp_spdk_exe_isp_fpga_multithread src/groundhog/exp/exp_spdk_exe_isp_fpga_multithread.c)
target_link_libraries(exp_spdk_exe_isp_fpga_multithread ${SPDK_LIBS} ${PROJECT_SOURCE_DIR}/cmake-build-debug/libtraj_block_format.a)

add_executable(pre_spdk_flush src/groundhog/exp/pre_spdk_flush.c)
target_link_libraries(pre_spdk_flush ${SPDK_LIBS} ${PROJECT_SOURCE_DIR}/cmake-build-debug/libtraj_block_format.a)

add_executable(pre_spdk_read src/groundhog/exp/pre_spdk_read.c)
target_link_libraries(pre_spdk_read ${SPDK_LIBS} ${PROJECT_SOURCE_DIR}/cmake-build-debug/libtraj_block_format.a)

add_executable(pre_spdk_read_multithread src/groundhog/exp/pre_spdk_read_multithread.c)
target_link_libraries(pre_spdk_read_multithread ${SPDK_LIBS} ${PROJECT_SOURCE_DIR}/cmake-build-debug/libtraj_block_format.a)

add_executable(exp_cost_model src/groundhog/exp/exp_cost_model.c src/common_util.c)
target_link_libraries(exp_cost_model ${SPDK_LIBS} ${PROJECT_SOURCE_DIR}/cmake-build-debug/libtraj_block_format.a)

add_executable(exp_native_host_and_device_large src/groundhog/exp/exp_native_host_and_device_large.c)
target_link_libraries(exp_native_host_and_device_large ${SPDK_LIBS} ${PROJECT_SOURCE_DIR}/cmake-build-debug/libtraj_block_format.a)

add_executable(exp_traj_format src/groundhog/exp/exp_traj_format.c)
target_link_libraries(exp_traj_format ${SPDK_LIBS} ${PROJECT_SOURCE_DIR}/cmake-build-debug/libtraj_block_format.a)

add_executable(exp_spdk_exe_isp_count_query src/groundhog/exp/exp_spdk_exe_isp_count_query.c)
target_link_libraries(exp_spdk_exe_isp_count_query ${SPDK_LIBS} ${PROJECT_SOURCE_DIR}/cmake-build-debug/libtraj_block_format.a)

