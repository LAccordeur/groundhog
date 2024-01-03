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

/**
 * fetch multiple continuous trajectory data block starting from @param block_logical_pointer_start
 * @param storage
 * @param block_logical_pointer_start
 * @param block_num
 * @param destination
 */
void fetch_continuous_traj_data_block(struct traj_storage *storage, int block_logical_pointer_start, int block_num, void* destination);

void fetch_continuous_traj_data_block_spdk_batch(int batch_size, struct traj_storage *storage, int *block_logical_pointer_start, int *block_num, void **destination);

void do_isp_for_trajectory_data_without_comp(struct traj_storage *storage, void* result_buffer, size_t result_size, struct isp_descriptor *isp_desc);

void do_isp_for_trajectory_data_without_comp_batch(int batch_size, struct traj_storage *storage, void** result_buffer, size_t *result_size, struct isp_descriptor **isp_desc);

/**
 * some isp in the batch is in-storage-computing, some isp (without computation) in the batch is host computation
 * @param batch_size
 * @param storage
 * @param result_buffer
 * @param result_size
 * @param isp_desc
 */
void do_isp_for_trajectory_data_hybrid_comp_batch(int batch_size, struct traj_storage *storage, void** result_buffer, size_t *result_size, struct isp_descriptor **isp_desc);

void do_isp_for_trajectory_data(struct traj_storage *storage, void* result_buffer, size_t estimated_result_size, struct isp_descriptor *isp_desc, int accelerator_type);

void do_isp_for_trajectory_data_batch(int batch_size, struct traj_storage *storage, void** result_buffer, size_t *estimated_result_size, struct isp_descriptor **isp_desc, int accelerator_type);

void flush_traj_storage(struct traj_storage *storage);

#endif //TRAJ_BLOCK_FORMAT_SIMPLE_STORAGE_H
