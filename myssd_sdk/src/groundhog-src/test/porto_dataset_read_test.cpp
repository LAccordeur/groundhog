//
// Created by yangguo on 11/14/22.
//
#include "gtest/gtest.h"

extern "C" {
#include "groundhog/porto_dataset_reader.h"
}

TEST(readtest, readblock) {

    FILE *fp = fopen("/home/yangguo/Data/DataSet/Trajectory/TaxiPorto/archive/porto_data_v2.csv", "r");
    char buffer[4096];

    int row_offset = 0;
    int row_count = 4;
    int result_count = read_one_buffer_block_from_csv(fp, buffer, row_offset, row_count);
    printf("read count: %d\n", result_count);

    int values[4] = {1, 2, 3, 4};
    int *value = values;
    value[2] = 123;
    printf("%d\n", values[2]);

}

TEST(readtest, readpoints) {

    FILE *fp = fopen("/home/yangguo/Data/DataSet/Trajectory/TaxiPorto/archive/porto_data_v2.csv", "r");

    struct traj_point **points = allocate_points_memory(4);

    int row_offset = 0;
    int row_count = 4;
    int result_count = read_points_from_csv(fp, points, row_offset, row_count);
    printf("read count: %d\n", result_count);
    print_traj_points(points, row_count);

    char buffer[4096];
    convert_struct_to_serialized_traj(points, buffer, row_count);

    struct traj_point **points2 = allocate_points_memory(4);
    convert_serialized_to_struct_traj(buffer, points2, row_count);
    printf("\n\n serialized:\n");
    print_traj_points(points2, row_count);

    free_points_memory(points, 4);
    free_points_memory(points2, 4);


}