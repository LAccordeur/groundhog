//
// Created by yangguo on 11/14/22.
//
#include "gtest/gtest.h"

extern "C" {
#include "groundhog/traj_block_format.h"
#include "groundhog/log.h"
#include "groundhog/porto_dataset_reader.h"
#include "groundhog/config.h"
}

static void parse_seg_meta_section(void *metadata, struct seg_meta *meta_array, int array_size) {
    char* m = (char*)metadata;
    int seg_meta_size = get_seg_meta_size();
    for (int i= 0; i < array_size; i++) {
        char* offset = m + i * seg_meta_size;
        deserialize_seg_meta(offset, meta_array + i);
    }
}

static void print_meta_array(struct seg_meta *meta_array, int array_size) {
    for (int i = 0; i < array_size; i++) {
        struct seg_meta meta_item = meta_array[i];
        print_seg_meta(&meta_item);
    }
}

TEST(formattest, create) {

    int points_num = 400;
    struct traj_point **points = allocate_points_memory(points_num);
    FILE *fp = fopen("/home/yangguo/Data/DataSet/Trajectory/TaxiPorto/archive/porto_data_v2.csv", "r");

    read_points_from_csv(fp, points, 0, points_num);
    print_traj_points(points, points_num);

    int block_size = calculate_block_size_via_points_num(400, SPLIT_SEGMENT_NUM);
    char block[block_size];
    printf("block size: %d\n", block_size);
    do_self_contained_traj_block(points, points_num, block, block_size);

    struct traj_block_header header;
    parse_traj_block_for_header(block, &header);
    printf("seg count: %d\n", header.seg_count);

    struct seg_meta meta_arr[SPLIT_SEGMENT_NUM];
    parse_traj_block_for_seg_meta_section(block, meta_arr, SPLIT_SEGMENT_NUM);
    print_seg_meta(meta_arr);
    print_seg_meta(meta_arr+1);

    struct traj_point **deserialized = allocate_points_memory(40);
    parse_traj_block_for_seg_data(block, 324, deserialized, 40);
    print_traj_points(deserialized, 40);

}

TEST(formarttest, seg_meta) {
    int points_num = 400;
    struct traj_point **points = allocate_points_memory(points_num);
    FILE *fp = fopen("/home/yangguo/Data/DataSet/Trajectory/TaxiPorto/archive/porto_data_v2.csv", "r");

    read_points_from_csv(fp, points, 0, points_num);
    //print_traj_points(points, points_num);

    int block_size = calculate_block_size_via_points_num(400, SPLIT_SEGMENT_NUM);
    char block[block_size];
    printf("block size: %d\n", block_size);
    do_self_contained_traj_block(points, points_num, block, block_size);

    struct seg_meta raw_meta_array[SPLIT_SEGMENT_NUM];
    parse_traj_block_for_seg_meta_section(block, raw_meta_array, SPLIT_SEGMENT_NUM);
    print_meta_array(raw_meta_array, SPLIT_SEGMENT_NUM);
    printf("\n\n");

    int meta_section_size = get_seg_meta_section_size(block);
    printf("meta item size: %d\n", get_seg_meta_size());
    printf("meta section size: %d\n", meta_section_size);

    char meta_section[meta_section_size];
    extract_seg_meta_section(block, meta_section);

    struct seg_meta meta_array[SPLIT_SEGMENT_NUM];
    parse_seg_meta_section(meta_section, meta_array, SPLIT_SEGMENT_NUM);
    print_meta_array(meta_array, SPLIT_SEGMENT_NUM);

}

