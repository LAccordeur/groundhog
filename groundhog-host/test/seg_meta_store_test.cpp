//
// Created by yangguo on 11/29/22.
//

#include "gtest/gtest.h"

extern "C" {
#include "groundhog/seg_meta_store.h"
#include "groundhog/config.h"
#include "groundhog/traj_block_format.h"
#include "groundhog/porto_dataset_reader.h"
#include "groundhog/simple_query_engine.h"
#include "groundhog/persistence_manager.h"
}

static void parse_seg_meta_section(void *metadata, struct seg_meta *meta_array, int array_size) {
    char* m = (char*)metadata;
    int seg_meta_size = get_seg_meta_size();
    for (int i= 0; i < array_size; i++) {
        char* offset = m + i * seg_meta_size;
        deserialize_seg_meta(offset, meta_array + i);
    }
}

static void print_seg_meta_entry_storage(struct seg_meta_section_entry_storage *storage) {
    printf("current entry count: %d\n", storage->current_index + 1);
    for (int i = 0; i <= storage->current_index; i++) {
        struct seg_meta_section_entry *entry = storage->base[i];
        struct seg_meta meta_array[entry->seg_meta_count];
        parse_seg_meta_section(entry->seg_meta_section, meta_array, entry->seg_meta_count);
        for (int j = 0; j < entry->seg_meta_count; j++) {
            struct seg_meta meta_item = meta_array[j];
            print_seg_meta(&meta_item);
        }
        printf("\n");
    }
}

static void print_meta_array(struct seg_meta *meta_array, int array_size) {
    printf("print meta array\n");
    for (int i = 0; i < array_size; i++) {
        struct seg_meta meta_item = meta_array[i];
        print_seg_meta(&meta_item);
    }
}

static void print_seg_meta_section_entry(struct seg_meta_section_entry *entry) {
    printf("seg meta count: %d\n", entry->seg_meta_count);
    printf("seg meta section: %p\n", entry->seg_meta_section);
    printf("logical adr: %d\n", entry->block_logical_adr);
    struct seg_meta meta_array[entry->seg_meta_count];
    parse_seg_meta_section(entry->seg_meta_section, meta_array, entry->seg_meta_count);
    for (int j = 0; j < entry->seg_meta_count; j++) {
        struct seg_meta meta_item = meta_array[j];
        print_seg_meta(&meta_item);
    }
    printf("\n");
}

TEST(seg_meta_store_test, init) {
    struct seg_meta_section_entry_storage storage;
    init_seg_meta_entry_storage(&storage);
    free_seg_meta_entry_storage(&storage);
}

TEST(seg_meta_store_test, create) {
    struct seg_meta_section_entry_storage storage;
    init_seg_meta_entry_storage(&storage);

    FILE *data_fp = fopen("/home/yangguo/Data/DataSet/Trajectory/TaxiPorto/archive/porto_data_v2.csv", "r");

    // ingest data
    struct simple_query_engine query_engine;
    init_query_engine(&query_engine);

    struct traj_storage *data_storage = &query_engine.data_storage;
    struct index_entry_storage *index_storage = &query_engine.index_storage;
    // trajectory block info
    int points_num = calculate_points_num_via_block_size(TRAJ_BLOCK_SIZE, SPLIT_SEGMENT_NUM);

    for (int i = 0; i < 2; i++) {
        struct traj_point **points = allocate_points_memory(points_num);
        read_points_from_csv(data_fp,points, i*points_num, points_num);
        // convert and put this data to traj storage
        void *data = malloc(TRAJ_BLOCK_SIZE);
        do_self_contained_traj_block(points, points_num, data, TRAJ_BLOCK_SIZE);
        struct address_pair data_addresses = append_traj_block_to_storage(data_storage, data);

        /*struct seg_meta raw_meta_array[SPLIT_SEGMENT_NUM];
        parse_traj_block_for_seg_meta_section(data, raw_meta_array, SPLIT_SEGMENT_NUM);
        print_meta_array(raw_meta_array, SPLIT_SEGMENT_NUM);
        printf("\n\n");*/

        // update seg_meta
        struct seg_meta_section_entry *entry = (struct seg_meta_section_entry *)malloc(sizeof(struct seg_meta_section_entry));
        struct traj_block_header header;
        parse_traj_block_for_header(data, &header);
        int meta_section_size = get_seg_meta_section_size(data);
        void* meta_section = malloc(meta_section_size);
        extract_seg_meta_section(data, meta_section);
        entry->seg_meta_count = header.seg_count;
        entry->seg_meta_section = meta_section;
        entry->block_logical_adr = data_addresses.logical_adr;

        /*printf("meta section ptr: %p\n", meta_section);
        struct seg_meta meta_array[SPLIT_SEGMENT_NUM];
        parse_seg_meta_section(meta_section, meta_array, SPLIT_SEGMENT_NUM);
        print_meta_array(meta_array, SPLIT_SEGMENT_NUM);*/

        append_to_seg_meta_entry_storage(&storage, entry);

        //print_seg_meta_entry_storage(&storage);

        free_points_memory(points, points_num);
    }

    print_seg_meta_entry_storage(&storage);

    struct id_temporal_predicate predicate;
    predicate.oid = 20000367;
    predicate.time_min = 1372637312;
    predicate.time_max = 1372640912;
    int result_size = estimate_id_temporal_result_size(&storage, &predicate);
    printf("id temporal result size: %d\n", result_size);

    struct spatio_temporal_range_predicate st_predicate;
    st_predicate.time_min = 1372637312;
    st_predicate.time_max = 1372640912;
    st_predicate.lon_min = 7988543;
    st_predicate.lon_max = 7988999;
    st_predicate.lat_min = 12223167;
    st_predicate.lat_max = 12223393;

    int st_result_size = estimate_spatio_temporal_result_size(&storage, &st_predicate);
    printf("st temporal result size: %d\n", st_result_size);


    free_seg_meta_entry_storage(&storage);
}

TEST(seg_meta_store, serialize_entry) {
    struct seg_meta_section_entry_storage storage;
    init_seg_meta_entry_storage(&storage);

    FILE *data_fp = fopen("/home/yangguo/Data/DataSet/Trajectory/TaxiPorto/archive/porto_data_v2.csv", "r");

    // ingest data
    struct simple_query_engine query_engine;
    init_query_engine(&query_engine);

    struct traj_storage *data_storage = &query_engine.data_storage;
    struct index_entry_storage *index_storage = &query_engine.index_storage;
    // trajectory block info
    int points_num = calculate_points_num_via_block_size(TRAJ_BLOCK_SIZE, SPLIT_SEGMENT_NUM);

    void* meta_section;
    for (int i = 0; i < 1; i++) {
        struct traj_point **points = allocate_points_memory(points_num);
        read_points_from_csv(data_fp,points, i*points_num, points_num);
        // convert and put this data to traj storage
        void *data = malloc(TRAJ_BLOCK_SIZE);
        do_self_contained_traj_block(points, points_num, data, TRAJ_BLOCK_SIZE);
        struct address_pair data_addresses = append_traj_block_to_storage(data_storage, data);


        struct seg_meta_section_entry *entry = (struct seg_meta_section_entry *)malloc(sizeof(struct seg_meta_section_entry));
        struct traj_block_header header;
        parse_traj_block_for_header(data, &header);
        int meta_section_size = get_seg_meta_section_size(data);
        meta_section = malloc(meta_section_size);
        extract_seg_meta_section(data, meta_section);

        free_points_memory(points, points_num);
    }


    struct seg_meta_section_entry entry;
    entry.seg_meta_count = 10;
    entry.seg_meta_section = meta_section;
    entry.block_logical_adr = 12;
    print_seg_meta_section_entry(&entry);
    char block[4096];

    serialize_seg_meta_section_entry(&entry, block);

    printf("\n");
    struct seg_meta_section_entry re;
    deserialize_seg_meta_section_entry(block, &re);
    print_seg_meta_section_entry(&re);
}


TEST(seg_meta_store, serialize_serialized_store) {
    struct serialized_seg_meta_section_entry_storage serialized_storage;
    init_serialized_seg_meta_section_entry_storage(&serialized_storage);


    struct seg_meta_section_entry_storage storage;
    init_seg_meta_entry_storage(&storage);

    FILE *data_fp = fopen("/home/yangguo/Data/DataSet/Trajectory/TaxiPorto/archive/porto_data_v2.csv", "r");

    // ingest data
    struct simple_query_engine query_engine;
    init_query_engine(&query_engine);

    struct traj_storage *data_storage = &query_engine.data_storage;
    struct index_entry_storage *index_storage = &query_engine.index_storage;
    // trajectory block info
    int points_num = calculate_points_num_via_block_size(TRAJ_BLOCK_SIZE, SPLIT_SEGMENT_NUM);

    for (int i = 0; i < 200; i++) {
        struct traj_point **points = allocate_points_memory(points_num);
        read_points_from_csv(data_fp,points, i*points_num, points_num);
        // convert and put this data to traj storage
        void *data = malloc(TRAJ_BLOCK_SIZE);
        do_self_contained_traj_block(points, points_num, data, TRAJ_BLOCK_SIZE);
        struct address_pair data_addresses = append_traj_block_to_storage(data_storage, data);

        // update seg_meta
        struct seg_meta_section_entry *entry = (struct seg_meta_section_entry *)malloc(sizeof(struct seg_meta_section_entry));
        struct traj_block_header header;
        parse_traj_block_for_header(data, &header);
        int meta_section_size = get_seg_meta_section_size(data);
        void* meta_section = malloc(meta_section_size);
        extract_seg_meta_section(data, meta_section);
        entry->seg_meta_count = header.seg_count;
        entry->seg_meta_section = meta_section;
        entry->block_logical_adr = data_addresses.logical_adr;
        append_to_seg_meta_entry_storage(&storage, entry);

        free_points_memory(points, points_num);
    }

    print_seg_meta_entry_storage(&storage);


    serialize_seg_meta_section_entry_storage(&storage, &serialized_storage);
    struct seg_meta_section_entry_storage re_storage;
    init_seg_meta_entry_storage(&re_storage);
    deserialize_seg_meta_section_entry_storage(&serialized_storage, &re_storage);
    printf("\nresult\n");
    print_seg_meta_entry_storage(&re_storage);

    free_seg_meta_entry_storage(&re_storage);
    free_seg_meta_entry_storage(&storage);
    free_serialized_seg_meta_section_entry_storage(&serialized_storage);
}

TEST(seg_meta_store, flush) {
    struct serialized_seg_meta_section_entry_storage serialized_storage;
    init_serialized_seg_meta_section_entry_storage(&serialized_storage);


    struct seg_meta_section_entry_storage storage;
    init_seg_meta_entry_storage(&storage);

    FILE *data_fp = fopen("/home/yangguo/Data/DataSet/Trajectory/TaxiPorto/archive/porto_data_v2.csv", "r");

    // ingest data
    struct simple_query_engine query_engine;
    init_query_engine(&query_engine);

    struct traj_storage *data_storage = &query_engine.data_storage;
    struct index_entry_storage *index_storage = &query_engine.index_storage;
    // trajectory block info
    int points_num = calculate_points_num_via_block_size(TRAJ_BLOCK_SIZE, SPLIT_SEGMENT_NUM);

    for (int i = 0; i < 200; i++) {
        struct traj_point **points = allocate_points_memory(points_num);
        read_points_from_csv(data_fp,points, i*points_num, points_num);
        // convert and put this data to traj storage
        void *data = malloc(TRAJ_BLOCK_SIZE);
        do_self_contained_traj_block(points, points_num, data, TRAJ_BLOCK_SIZE);
        struct address_pair data_addresses = append_traj_block_to_storage(data_storage, data);

        // update seg_meta
        struct seg_meta_section_entry *entry = (struct seg_meta_section_entry *)malloc(sizeof(struct seg_meta_section_entry));
        struct traj_block_header header;
        parse_traj_block_for_header(data, &header);
        int meta_section_size = get_seg_meta_section_size(data);
        void* meta_section = malloc(meta_section_size);
        extract_seg_meta_section(data, meta_section);
        entry->seg_meta_count = header.seg_count;
        entry->seg_meta_section = meta_section;
        entry->block_logical_adr = data_addresses.logical_adr;
        append_to_seg_meta_entry_storage(&storage, entry);

        free_points_memory(points, points_num);
    }

    print_seg_meta_entry_storage(&storage);


    serialize_seg_meta_section_entry_storage(&storage, &serialized_storage);

    // test flush and rebuild
    char* filename = "/home/yangguo/CLionProjects/traj-block-format/datafile/traj.segmeta";
    flush_serialized_seg_meta_storage(&serialized_storage, filename, COMMON_FS_MODE);

    struct seg_meta_section_entry_storage rebuild;
    init_seg_meta_entry_storage(&rebuild);
    rebuild_seg_meta_storage(filename, COMMON_FS_MODE, &rebuild);
    printf("rebuild:\n");
    print_seg_meta_entry_storage(&rebuild);

}
