//
// Created by yangguo on 23-1-11.
//

#include "gtest/gtest.h"

extern "C" {
#include "groundhog/persistence_manager.h"
#include "groundhog/traj_block_format.h"
#include "groundhog/porto_dataset_reader.h"
}

extern struct spdk_static_fs_desc spdk_static_fs_layer_for_traj;

TEST(persistence_test, init) {
    init_and_mk_fs_for_traj(false);
    struct my_file *my_fp = my_fopen(DATA_FILENAME, "w", SPDK_FS_MODE);

    int block_num = 5;
    int traj_block_size = 4096;
    int split_segment_num = 10;
    int points_num = calculate_points_num_via_block_size(traj_block_size, split_segment_num);
    printf("point num per block: %d\n", points_num);
    FILE *fp = fopen("/home/yangguo/Dataset/trajectory/porto_data_v2.csv", "r");
    char my_buffer[traj_block_size];
    struct traj_point **points = allocate_points_memory(points_num);

    for (int i = 0; i < block_num; i++) {
        read_points_from_csv(fp, points, i*points_num, points_num);
        //print_traj_points(points, points_num);
        do_self_contained_traj_block(points, points_num, my_buffer, traj_block_size);
        int write_size = my_fwrite(my_buffer, traj_block_size, 1, my_fp, SPDK_FS_MODE);
        printf("write id: %d; write size: %d\n", i, write_size);
    }

    spdk_flush_static_fs_meta(&spdk_static_fs_layer_for_traj);
}

TEST(persistence_test, rebuild) {
    init_and_mk_fs_for_traj(true);
    print_spdk_static_fs_meta(&spdk_static_fs_layer_for_traj);

    struct my_file *my_fp = my_fopen(DATA_FILENAME, "r", SPDK_FS_MODE);

    int block_num = 5;
    int traj_block_size = 4096;
    char my_buffer[traj_block_size];

    for (int i = 0; i < block_num; i++) {
        int read_size = my_fread(my_buffer, traj_block_size, 1, my_fp, SPDK_FS_MODE);
        printf("read id: %d; read size: %d\n", i, read_size);
    }
    print_spdk_static_fs_meta(&spdk_static_fs_layer_for_traj);

}
