//
// Created by yangguo on 24-4-9.
//

#include "gtest/gtest.h"

extern "C" {
#include "groundhog/osm_dataset_reader.h"
#include <stdio.h>
}

TEST(osmreadtest, readpoints) {

    FILE *fp = fopen("/home/yangguo/Dataset/osm/osm_points_v1.csv", "r");

    struct traj_point **points = allocate_points_memory(16);

    int row_offset = 4;
    int row_count = 16;
    int result_count = read_points_from_csv_osm(fp, points, row_offset, row_count);
    printf("read count: %d\n", result_count);
    print_traj_points(points, row_count);

    free_points_memory(points, 16);


}