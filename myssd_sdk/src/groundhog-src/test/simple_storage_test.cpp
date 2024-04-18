//
// Created by yangguo on 11/22/22.
//

#include "gtest/gtest.h"

extern "C" {
#include "groundhog/simple_storage.h"
#include "groundhog/simple_query_engine.h"
#include "groundhog/config.h"
#include "groundhog/porto_dataset_reader.h"
#include "groundhog/persistence_manager.h"
}

static void print_index_entry(struct index_entry *entry) {
    printf("oid_array_size: %d\n", entry->oid_array_size);
    printf("oid_array: %p\n", entry->oid_array);
    printf("lon_min: %d\n", entry->lon_min);
    printf("lon_max: %d\n", entry->lon_max);
    printf("lat_min: %d\n", entry->lat_min);
    printf("lat_max: %d\n", entry->lat_max);
    printf("time_min: %d\n", entry->time_min);
    printf("time_max: %d\n", entry->time_max);
    printf("block_logical_adr: %d\n", entry->block_logical_adr);
    printf("block_physical_ptr: %p\n\n", entry->block_physical_ptr);
}

static void print_specific_seg_points(void* data_block) {
    struct traj_block_header block_header;
    parse_traj_block_for_header(data_block, &block_header);
    struct seg_meta meta_array[block_header.seg_count];
    parse_traj_block_for_seg_meta_section(data_block, meta_array, block_header.seg_count);

    struct seg_meta meta_item = meta_array[block_header.seg_count-1];

    int data_seg_points_num = meta_item.seg_size / get_traj_point_size();
    struct traj_point **points = allocate_points_memory(data_seg_points_num);
    parse_traj_block_for_seg_data(data_block, meta_item.seg_offset, points, data_seg_points_num);
    print_traj_points(points, data_seg_points_num);

    free_points_memory(points, data_seg_points_num);

}

TEST(storage, test) {

    struct traj_storage storage;
    init_traj_storage(&storage);

    for (int i = 0; i < 8002; i++) {
        void * ptr = NULL;
        struct address_pair result = append_traj_block_to_storage(&storage, ptr);
        printf("physical: %p, logical: %d\n", result.physical_ptr, result.logical_adr);
    }


    free_traj_storage(&storage);
}


TEST(storage, flush) {
    char filename[] = "/home/yangguo/CLionProjects/traj-block-format/datafile/traj.data";
    struct traj_storage data_storage;
    init_traj_storage_with_persistence(&data_storage, filename, "w", COMMON_FS_MODE);

    FILE *fp = fopen("/home/yangguo/Data/DataSet/Trajectory/TaxiPorto/archive/porto_data_v2.csv", "r");

    // trajectory block info
    int points_num = calculate_points_num_via_block_size(TRAJ_BLOCK_SIZE, SPLIT_SEGMENT_NUM);
    int block_num = 3;

    for (int i = 0; i < block_num; i++) {
        struct traj_point **points = allocate_points_memory(points_num);
        read_points_from_csv(fp,points, i*points_num, points_num);
        // convert and put this data to traj storage
        void *data = malloc(TRAJ_BLOCK_SIZE);
        do_self_contained_traj_block(points, points_num, data, TRAJ_BLOCK_SIZE);
        struct address_pair data_addresses = append_traj_block_to_storage(&data_storage, data);


        free_points_memory(points, points_num);
    }

    // print the points of last seg in the last block
    void* data_block = data_storage.traj_blocks_base[block_num-1];
    print_specific_seg_points(data_block);


    // flush and load

    flush_traj_storage(&data_storage);

    char* rebuild_block[TRAJ_BLOCK_SIZE];
    struct my_file *rebuild_fp = my_fopen(filename, "r", COMMON_FS_MODE);
    my_fseek(rebuild_fp, TRAJ_BLOCK_SIZE * (block_num - 1), COMMON_FS_MODE);
    my_fread(rebuild_block, TRAJ_BLOCK_SIZE, 1, rebuild_fp, COMMON_FS_MODE);
    printf("\n\nrebuild:\n");
    print_specific_seg_points(rebuild_block);

    void* rebuild_block_1[TRAJ_BLOCK_SIZE];
    struct traj_storage disk_storage;
    init_traj_storage_with_persistence(&disk_storage, filename, "r", COMMON_FS_MODE);
    fetch_traj_data_via_logical_pointer(&disk_storage, block_num-1, rebuild_block_1);
    printf("\n\nrebuild 1:\n");
    print_specific_seg_points(rebuild_block_1);


    free_traj_storage(&data_storage);
}

