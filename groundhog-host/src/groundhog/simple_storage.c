//
// Created by yangguo on 11/16/22.
//
#include "groundhog/simple_storage.h"
#include "groundhog/log.h"
#include <stdlib.h>
#include "groundhog/traj_block_format.h"
#include "groundhog/persistence_manager.h"
#include "groundhog/config.h"

void init_traj_storage(struct traj_storage *storage) {
    storage->current_index = -1;
    storage->total_size = 1000; // initial capacity is 1000
    storage->traj_blocks_base = malloc(1000 * sizeof(void*));
}

void init_traj_storage_with_persistence(struct traj_storage *storage, char* filename, char* file_operation_mode, int fs_mode) {
    storage->current_index = -1;
    storage->total_size = 1000; // initial capacity is 1000
    storage->traj_blocks_base = malloc(1000 * sizeof(void*));
    storage->my_fp = my_fopen(filename, file_operation_mode, fs_mode);
}

struct address_pair append_traj_block_to_storage(struct traj_storage *storage, void *block_ptr) {
    storage->current_index++;

    struct address_pair addresses;
    addresses.physical_ptr = block_ptr;
    addresses.logical_adr = storage->current_index;
    if (storage->current_index < storage->total_size) {
        (storage->traj_blocks_base)[storage->current_index] = block_ptr;
        //storage->current_index++;
    } else {
        // we need to extend array
        int new_total_size = storage->total_size * 2;
        debug_print("extend data storage from size %d to size %d\n", storage->total_size, new_total_size);
        void **tmp_base = malloc(new_total_size * sizeof(void*));
        for (int i = 0; i < storage->total_size; i++) {
            tmp_base[i] = (storage->traj_blocks_base)[i];
        }
        free(storage->traj_blocks_base);
        storage->traj_blocks_base = tmp_base;

        (storage->traj_blocks_base)[storage->current_index] = block_ptr;
        //storage->current_index++;
        storage->total_size = new_total_size;
    }
    return addresses;
}

int calculate_total_num_of_points_in_storage(struct traj_storage *storage) {
    int total_num = 0;
    for (int i = 0; i < storage->current_index; i++) {
        void* data_block = storage->traj_blocks_base[i];
        struct traj_block_header block_header;
        parse_traj_block_for_header(data_block, &block_header);
        struct seg_meta meta_array[block_header.seg_count];
        parse_traj_block_for_seg_meta_section(data_block, meta_array, block_header.seg_count);
        for (int j = 0; j < block_header.seg_count; j++) {
            struct seg_meta meta_item = meta_array[j];
            int data_seg_points_num = meta_item.seg_size / get_traj_point_size();
            total_num += data_seg_points_num;
        }
    }
    return total_num;
}

void fetch_traj_data_via_logical_pointer(struct traj_storage *storage, int logical_pointer, void* destination) {
    int fs_mode = storage->my_fp->fs_mode;
    struct my_file *my_fp = storage->my_fp;
    my_fseek(my_fp, TRAJ_BLOCK_SIZE * logical_pointer, fs_mode);
    //void* data_block = malloc(TRAJ_BLOCK_SIZE);
    my_fread(destination, 1, TRAJ_BLOCK_SIZE, my_fp, fs_mode);
    //return data_block;
    //debug_print("[fetch_traj_data_via_logical_pointer] fetch block [%d] from file [%s] to memory [%p]\n", logical_pointer, my_fp->filename, destination);
}

void fetch_continuous_traj_data_block(struct traj_storage *storage, int block_logical_pointer_start, int block_num, void* destination) {
    int fs_mode = storage->my_fp->fs_mode;
    struct my_file *my_fp = storage->my_fp;


    if (fs_mode == COMMON_FS_MODE) {
        my_fseek(my_fp, TRAJ_BLOCK_SIZE * block_logical_pointer_start, fs_mode);
        my_fread(destination, block_num, TRAJ_BLOCK_SIZE, my_fp, fs_mode);
    }
    if (fs_mode == SPDK_FS_MODE) {
        // check if the sector count exceed the limit in FTL (256)
        int block_num_limit = 256 * SECTOR_SIZE / TRAJ_BLOCK_SIZE;
        if (block_num <= block_num_limit) {
            long long my_offset = TRAJ_BLOCK_SIZE * (long long) block_logical_pointer_start;
            my_fseek(my_fp, my_offset, fs_mode);
            my_fread(destination, block_num, TRAJ_BLOCK_SIZE, my_fp, fs_mode);
        } else {
            int remaining_block_num = block_num;
            int i = 0;
            do {
                long long my_offset = TRAJ_BLOCK_SIZE * (long long) block_logical_pointer_start + i * block_num_limit * TRAJ_BLOCK_SIZE;
                my_fseek(my_fp, my_offset, fs_mode);

                //my_fseek(my_fp, TRAJ_BLOCK_SIZE * block_logical_pointer_start + i * block_num_limit * TRAJ_BLOCK_SIZE, fs_mode);
                int read_block_num = remaining_block_num > block_num_limit ? block_num_limit : remaining_block_num;
                my_fread(destination + i * block_num_limit * TRAJ_BLOCK_SIZE, read_block_num, TRAJ_BLOCK_SIZE, my_fp, fs_mode);

                remaining_block_num -= block_num_limit;
                i++;
            } while (remaining_block_num > 0);
        }
    }

}

void fetch_continuous_traj_data_block_spdk_batch(int batch_size, struct traj_storage *storage, int *block_logical_pointer_start, int *block_num, void **destination) {
    int fs_mode = storage->my_fp->fs_mode;
    struct my_file *my_fp = storage->my_fp;


    if (fs_mode == SPDK_FS_MODE) {

        int logical_sector_start[batch_size];
        size_t size_vec[batch_size];
        for (int i = 0; i < batch_size; i++) {
            logical_sector_start[i] = block_logical_pointer_start[i] * TRAJ_BLOCK_SIZE / SECTOR_SIZE;
            size_vec[i] = block_num[i] * TRAJ_BLOCK_SIZE;
        }
        long long my_offset = TRAJ_BLOCK_SIZE * (long long) block_logical_pointer_start[0];
        my_fseek(my_fp, my_offset, fs_mode);
        my_fread_spdk_batch(batch_size, destination, logical_sector_start, size_vec, my_fp);

    } else {
        printf("only support for SPDK mode\n");
    }

}

void do_isp_for_trajectory_data_without_comp(struct traj_storage *storage, void* result_buffer, size_t result_size, struct isp_descriptor *isp_desc) {
    int fs_mode = storage->my_fp->fs_mode;
    if (fs_mode == SPDK_FS_MODE) {
        my_fread_spdk_multi_addr(result_buffer, result_size, storage->my_fp, isp_desc);
    } else {
        printf("only support for SPDK mode\n");
    }
}

void do_isp_for_trajectory_data_without_comp_batch(int batch_size, struct traj_storage *storage, void** result_buffer, size_t *result_size, struct isp_descriptor **isp_desc) {
    int fs_mode = storage->my_fp->fs_mode;
    if (fs_mode == SPDK_FS_MODE) {
        my_fread_spdk_multi_addr_batch(batch_size, result_buffer, result_size, storage->my_fp, isp_desc);
    } else {
        printf("only support for SPDK mode\n");
    }
}


void do_isp_for_trajectory_data_hybrid_comp_batch(int batch_size, struct traj_storage *storage, void** result_buffer, size_t *result_size, struct isp_descriptor **isp_desc) {
    int fs_mode = storage->my_fp->fs_mode;
    if (fs_mode == SPDK_FS_MODE) {
        my_fread_spdk_multi_addr_batch(batch_size, result_buffer, result_size, storage->my_fp, isp_desc);
    } else {
        printf("only support for SPDK mode\n");
    }
}

void do_isp_for_trajectory_data(struct traj_storage *storage, void* result_buffer, size_t estimated_result_size, struct isp_descriptor *isp_desc, int accelerator_type) {
    struct my_file *my_fp = storage->my_fp;
    my_fread_isp(result_buffer, estimated_result_size, my_fp, isp_desc, accelerator_type);
}

void do_isp_for_trajectory_data_batch(int batch_size, struct traj_storage *storage, void** result_buffer, size_t *estimated_result_size, struct isp_descriptor **isp_desc, int accelerator_type) {
    struct my_file *my_fp = storage->my_fp;
    my_fread_isp_batch(batch_size, result_buffer, estimated_result_size, my_fp, isp_desc, accelerator_type);
}

void free_traj_storage(struct traj_storage *storage) {
    for (int i = 0; i <= storage->current_index; i++) {
        void* block_ptr = (storage->traj_blocks_base)[i];
        free(block_ptr);
        (storage->traj_blocks_base)[i] = NULL;
    }
    free(storage->traj_blocks_base);
}

void flush_traj_storage(struct traj_storage *storage) {
    struct my_file *fp = storage->my_fp;
    int fs_mode = storage->my_fp->fs_mode;
    int count = 0;
    /*for (int i = 0; i <= storage->current_index; i++) {
        void *block_ptr = storage->traj_blocks_base[i];
        my_fseek(fp, i * TRAJ_BLOCK_SIZE, fs_mode);
        int result_size = my_fwrite(block_ptr, 1, TRAJ_BLOCK_SIZE, fp, fs_mode);
        if (result_size > 0) {
            count++;
        }
    }*/


    int batch_block_size = 1024;
    void *buffer = malloc(batch_block_size * TRAJ_BLOCK_SIZE);
    long long i;
    for (i = 0; i <= storage->current_index; i++) {
        void *block_ptr = storage->traj_blocks_base[i];
        int buffer_offset = (int)i % batch_block_size;
        memcpy(buffer + buffer_offset * TRAJ_BLOCK_SIZE, block_ptr, TRAJ_BLOCK_SIZE);

        if ((i+1) % batch_block_size == 0) {
            my_fseek(fp, i * TRAJ_BLOCK_SIZE, fs_mode);
            int result_size = my_fwrite(buffer, batch_block_size, TRAJ_BLOCK_SIZE, fp, fs_mode);
            if (result_size > 0) {
                count+=batch_block_size;
            }
            memset(buffer, 0, batch_block_size * TRAJ_BLOCK_SIZE);
        }
    }


    if ((storage->current_index+1) % batch_block_size != 0) {
        int file_offset = storage->current_index - ((storage->current_index + 1) % batch_block_size) + 1;
        int block_num = (storage->current_index + 1) % batch_block_size;
        my_fseek(fp, file_offset * TRAJ_BLOCK_SIZE, fs_mode);
        my_fwrite(buffer, block_num, TRAJ_BLOCK_SIZE, fp, fs_mode);
        count + block_num;
    }
    free(buffer);

    debug_print("[flush_traj_storage] expected flushed data block count: %d\n", storage->current_index+1);
    debug_print("[flush_traj_storage] total flushed data block count: %d\n", count);
}
