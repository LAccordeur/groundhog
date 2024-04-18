//
// Created by yangguo on 11/16/22.
//

#ifndef TRAJ_BLOCK_FORMAT_SIMPLE_STORAGE_H
#define TRAJ_BLOCK_FORMAT_SIMPLE_STORAGE_H

#include "persistence_manager.h"

struct traj_storage {
    void **traj_blocks_base;
    int current_index;
    int total_size;
    struct my_file *my_fp;
};

struct address_pair{
    int logical_adr;
    void* physical_ptr;
};

void init_traj_storage(struct traj_storage *storage);

/**
 * for disk-based version
 * @param storage
 * @param filename
 */
void init_traj_storage_with_persistence(struct traj_storage *storage, char* filename, char* file_operation_mode, int fs_mode);

struct address_pair append_traj_block_to_storage(struct traj_storage *storage, void *block_ptr);

void free_traj_storage(struct traj_storage *storage);

int calculate_total_num_of_points_in_storage(struct traj_storage *storage);

void fetch_traj_data_via_logical_pointer(struct traj_storage *storage, int logical_pointer, void* destination);

void flush_traj_storage(struct traj_storage *storage);

#endif //TRAJ_BLOCK_FORMAT_SIMPLE_STORAGE_H
