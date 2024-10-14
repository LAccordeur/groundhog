//
// Created by yangguo on 24-6-6.
//

#include "gtest/gtest.h"

extern "C" {
#include "groundhog/geolife_dataset_reader.h"
#include <stdio.h>
}

TEST(geolifereadtest, readpoints) {

    FILE *fp = fopen("/home/yangguo/Dataset/geolife/Geolife_v1_format.csv", "r");

    struct traj_point **points = allocate_points_memory(16);

    int row_offset = 4;
    int row_count = 16;
    int result_count = read_points_from_csv_geolife(fp, points, row_offset, row_count);
    printf("read count: %d\n", result_count);
    print_traj_points(points, row_count);

    free_points_memory(points, 16);


}