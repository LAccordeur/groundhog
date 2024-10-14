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
    FILE *fp = fopen("/home/yangguo/Dataset/trajectory/porto_data_v2.csv", "r");

    read_points_from_csv(fp, points, 0, 4);
    print_traj_points(points, 4);
    sort_traj_points(points, 4);
    printf("\n\n");
    print_traj_points(points, 4);

}

TEST(traj_processing, zcurve_sort) {
    int array_size = 6;
    struct traj_point **points = allocate_points_memory(array_size);
    FILE *fp = fopen("/home/yangguo/Dataset/trajectory/porto_data_v2.csv", "r");

    read_points_from_csv(fp, points, 0, array_size);
    print_traj_points(points, array_size);
    sort_traj_points_zcurve_with_option(points, array_size, 2);
    printf("\n\n");
    print_traj_points(points, array_size);
}


TEST(traj_processing, zcurve_sort_option) {
    int array_size = 6;
    struct traj_point **points = allocate_points_memory(array_size);
    points[0]->timestamp_sec = 6;
    points[0]->normalized_longitude = 6;
    points[0]->normalized_latitude = 6;

    points[1]->timestamp_sec = 6;
    points[1]->normalized_longitude = 8;
    points[1]->normalized_latitude = 8;

    points[2]->timestamp_sec = 6;
    points[2]->normalized_longitude = 2;
    points[2]->normalized_latitude = 2;

    points[3]->timestamp_sec = 3;
    points[3]->normalized_longitude = 6;
    points[3]->normalized_latitude = 6;

    points[4]->timestamp_sec = 3;
    points[4]->normalized_longitude = 9;
    points[4]->normalized_latitude = 9;

    points[5]->timestamp_sec = 3;
    points[5]->normalized_longitude = 1;
    points[5]->normalized_latitude = 1;



    print_traj_points(points, array_size);
    sort_traj_points_zcurve_with_option(points, array_size, 2);
    printf("\n\n");
    print_traj_points(points, array_size);
}

TEST(traj_processing, split) {
    int points_num = 4;
    struct traj_point **points = allocate_points_memory(points_num);
    FILE *fp = fopen("/home/yangguo/Dataset/trajectory/porto_data_v2.csv", "r");

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
