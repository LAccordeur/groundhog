//
// Created by yangguo on 11/16/22.
//

#ifndef TRAJ_BLOCK_FORMAT_SIMPLE_INDEX_H
#define TRAJ_BLOCK_FORMAT_SIMPLE_INDEX_H
#include "traj_block_format.h"

struct index_entry {
    int oid_array_size;
    int *oid_array;
    int lon_min;
    int lon_max;
    int lat_min;
    int lat_max;
    int time_min;
    int time_max;
    int block_logical_adr;
    void *block_physical_ptr;
};

struct index_entry_storage {
    struct index_entry **index_entry_base;
    int current_index;
    int total_size;
    struct my_file *my_fp;  // the file used to flush and rebuild
};

struct serialized_index_storage {
    void **index_block_base;
    int current_index;
    int total_size;
};

void serialize_index_entry(struct index_entry *source, void *destination);

void deserialize_index_entry(void* source, struct index_entry *destination);

void init_serialized_index_storage(struct serialized_index_storage *storage);

void free_serialized_index_storage(struct serialized_index_storage *storage);

void append_serialized_index_block_to_storage(struct serialized_index_storage *storage, void *block);

void serialize_index_entry_storage(struct index_entry_storage *entry_storage, struct serialized_index_storage *serialized_storage);

void deserialize_index_entry_storage(struct serialized_index_storage *serialized_storage, struct index_entry_storage *entry_storage);

/**
 * flush serialized index blocks to a file in a specific fs
 * @param storage
 * @param filename
 * @param fs_mode 0 -> common fs (linux fs), 1 -> spdk fs, 2 -> spdk in-storage-computing fs
 */
void flush_serialized_index_storage(struct serialized_index_storage *storage, char* filename, int fs_mode);

/**
 * rebuild deserialized index block storage from a file
 * @param filename
 * @param fs_mode
 * @param storage
 */
void rebuild_index_storage(char* filename, int fs_mode, struct index_entry_storage *storage);

void init_index_entry_storage(struct index_entry_storage *storage);

void init_index_entry_storage_with_persistence(struct index_entry_storage *storage, char* filename, char* file_operation_mode, int fs_mode);

void append_index_entry_to_storage(struct index_entry_storage *storage, struct index_entry *entry);

void free_index_entry_storage(struct index_entry_storage *storage);

void init_index_entry(struct index_entry *entry);

void free_index_entry(struct index_entry *entry);

void fill_index_entry(struct index_entry *entry, struct traj_point **points, int points_num, void* block_physical_ptr, int block_logical_adr);

#endif //TRAJ_BLOCK_FORMAT_SIMPLE_INDEX_H
