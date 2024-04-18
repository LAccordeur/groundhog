//
// Created by yangguo on 11/22/22.
//

#include "gtest/gtest.h"

extern "C" {
#include "groundhog/simple_index.h"
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

static void print_entry_storage(struct index_entry_storage *storage) {
    printf("index_entry_base: %p\n", storage->index_entry_base);
    printf("current index: %d\n", storage->current_index);
    printf("total_size: %d\n\n", storage->total_size);
}



TEST(testindex, init_entry) {
    struct index_entry entry;

    init_index_entry(&entry);
    print_index_entry(&entry);
    free_index_entry(&entry);
}

TEST(testindex, init_entry_storage) {
    struct index_entry_storage entry_storage;
    init_index_entry_storage(&entry_storage);
    print_entry_storage(&entry_storage);
    free_index_entry_storage(&entry_storage);
}

TEST(testindex, create_entry) {
    struct index_entry_storage entry_storage;
    init_index_entry_storage(&entry_storage);
    print_entry_storage(&entry_storage);

    struct traj_point **points = allocate_points_memory(1);
    points[0]->oid = 12;
    points[0]->normalized_longitude = 111;
    points[0]->normalized_latitude = 111;
    points[0]->timestamp_sec = 111;

    for (int i = 0; i < 1002; i++) {
        struct index_entry *entry = (struct index_entry *)malloc(sizeof(struct index_entry));
        init_index_entry(entry);

        fill_index_entry(entry, points, 1, NULL, 1111);
        append_index_entry_to_storage(&entry_storage, entry);
        if (i == 1001) {
            entry->block_logical_adr = 21231;
            print_index_entry(entry);
        }

    }
    print_entry_storage(&entry_storage);
    print_index_entry(entry_storage.index_entry_base[entry_storage.current_index]);
    free_points_memory(points, 1);
    free_index_entry_storage(&entry_storage);
}

TEST(testindex, serialize_entry) {
    struct traj_point **points = allocate_points_memory(1);
    points[0]->oid = 12;
    points[0]->normalized_longitude = 111;
    points[0]->normalized_latitude = 111;
    points[0]->timestamp_sec = 111;

    struct index_entry *entry = (struct index_entry *)malloc(sizeof(struct index_entry));
    init_index_entry(entry);

    fill_index_entry(entry, points, 1, NULL, 1111);

    void *index_block = malloc(4096);
    serialize_index_entry(entry, index_block);

    struct index_entry *result = (struct index_entry *)malloc(sizeof(struct index_entry));
    deserialize_index_entry(index_block, result);
    free_index_entry(entry);
    free_index_entry(result);
    free(index_block);
}


TEST(testindex, serialization) {
    struct serialized_index_storage serialized_storage;
    init_serialized_index_storage(&serialized_storage);



    struct index_entry_storage entry_storage;
    init_index_entry_storage(&entry_storage);
    print_entry_storage(&entry_storage);

    struct traj_point **points = allocate_points_memory(1);
    points[0]->oid = 12;
    points[0]->normalized_longitude = 111;
    points[0]->normalized_latitude = 111;
    points[0]->timestamp_sec = 111;

    for (int i = 0; i < 1002; i++) {
        struct index_entry *entry = (struct index_entry *)malloc(sizeof(struct index_entry));
        init_index_entry(entry);

        fill_index_entry(entry, points, 1, NULL, 1111);
        append_index_entry_to_storage(&entry_storage, entry);
        if (i == 1001) {
            entry->block_logical_adr = 21231;
            print_index_entry(entry);
        }

    }
    print_entry_storage(&entry_storage);
    print_index_entry(entry_storage.index_entry_base[entry_storage.current_index]);

    serialize_index_entry_storage(&entry_storage, &serialized_storage);

    struct index_entry_storage result;
    init_index_entry_storage(&result);
    deserialize_index_entry_storage(&serialized_storage, &result);

    print_entry_storage(&result);
    print_index_entry(result.index_entry_base[result.current_index]);
    free_index_entry_storage(&result);
    free_points_memory(points, 1);
    free_index_entry_storage(&entry_storage);
    free_serialized_index_storage(&serialized_storage);
}

TEST(testindex, flush) {
    struct serialized_index_storage serialized_storage;
    init_serialized_index_storage(&serialized_storage);



    struct index_entry_storage entry_storage;
    init_index_entry_storage(&entry_storage);
    print_entry_storage(&entry_storage);

    struct traj_point **points = allocate_points_memory(1);
    points[0]->oid = 12;
    points[0]->normalized_longitude = 111;
    points[0]->normalized_latitude = 111;
    points[0]->timestamp_sec = 111;

    for (int i = 0; i < 1002; i++) {
        struct index_entry *entry = (struct index_entry *)malloc(sizeof(struct index_entry));
        init_index_entry(entry);

        fill_index_entry(entry, points, 1, NULL, 1111);
        append_index_entry_to_storage(&entry_storage, entry);
        if (i == 1001) {
            entry->block_logical_adr = 21231;
            print_index_entry(entry);
        }

    }
    print_entry_storage(&entry_storage);
    print_index_entry(entry_storage.index_entry_base[entry_storage.current_index]);

    serialize_index_entry_storage(&entry_storage, &serialized_storage);


    // test flush and rebuild
    char* filename = "/home/yangguo/CLionProjects/traj-block-format/datafile/traj.index";
    flush_serialized_index_storage(&serialized_storage, filename, COMMON_FS_MODE);

    struct index_entry_storage rebuild_storage;
    init_index_entry_storage(&rebuild_storage);
    rebuild_index_storage(filename, COMMON_FS_MODE, &rebuild_storage);
    printf("rebuild one:\n");
    print_entry_storage(&rebuild_storage);
    print_index_entry(rebuild_storage.index_entry_base[rebuild_storage.current_index]);

}

