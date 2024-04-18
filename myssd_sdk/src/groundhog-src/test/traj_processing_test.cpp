//
// Created by yangguo on 11/14/22.
//

#include "gtest/gtest.h"

extern "C" {
#include "groundhog/traj_processing.h"
#include "groundhog/porto_dataset_reader.h"
}

TEST(traj_processing, sort) {

    struct traj_point **points = allocate_points_memory(4);
    FILE *fp = fopen("/home/yangguo/Data/DataSet/Trajectory/TaxiPorto/archive/porto_data_v2.csv", "r");

    read_points_from_csv(fp, points, 0, 4);
    print_traj_points(points, 4);
    sort_traj_points(points, 4);
    printf("\n\n");
    print_traj_points(points, 4);

}

TEST(traj_processing, split) {
    int points_num = 4;
    struct traj_point **points = allocate_points_memory(points_num);
    FILE *fp = fopen("/home/yangguo/Data/DataSet/Trajectory/TaxiPorto/archive/porto_data_v2.csv", "r");

    read_points_from_csv(fp, points, 0, points_num);
    struct seg_meta_pair_itr meta_pair_array;
    struct seg_meta_pair meta_pairs[2];
    init_seg_meta_pair_array(&meta_pair_array, meta_pairs, 2);

    split_traj_points_via_point_num(points, points_num, 2, &meta_pair_array, 2);
    print_seg_meta_pair_itr(&meta_pair_array);
}

TEST(traj_processing, split_new) {
    int points_num = 400;
    struct traj_point **points = allocate_points_memory(points_num);
    FILE *fp = fopen("/home/yangguo/Data/DataSet/Trajectory/TaxiPorto/archive/porto_data_v2.csv", "r");

    read_points_from_csv(fp, points, 0, points_num);
    int array_size = 3;
    struct seg_meta_pair_itr meta_pair_array;
    struct seg_meta_pair meta_pairs[array_size];
    init_seg_meta_pair_array(&meta_pair_array, meta_pairs, array_size);

    split_traj_points_via_point_num_and_seg_num(points, points_num, &meta_pair_array, array_size);
    print_seg_meta_pair_itr(&meta_pair_array);
}
