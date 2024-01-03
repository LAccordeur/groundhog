//
// Created by yangguo on 11/30/22.
//

#ifndef TRAJ_BLOCK_FORMAT_PERSISTENCE_MANAGER_H
#define TRAJ_BLOCK_FORMAT_PERSISTENCE_MANAGER_H

#define COMMON_FS_MODE 0
#define SPDK_FS_MODE 1

#include <stdio.h>
#include "groundhog/static_spdk_fs_layer.h"

struct my_file {
    FILE *common_file;
    char* filename;
    char* file_operation_mode;
    int fs_mode;
    struct spdk_static_file_desc *spdk_file;
    //struct isp_descriptor *isp_desc;
};

struct my_file *my_fopen(const char *filename, const char *file_operation_mode, int fs_mode);

size_t my_fwrite(const void *ptr, size_t size, size_t nmemb, struct my_file *stream, int fs_mode);

size_t my_fread(void *ptr, size_t size, size_t nmemb, struct my_file *stream, int fs_mode);

/**
 * only used for our csd (accessed via spdk)
 * @param batch_size
 * @param data_ptr_vec
 * @param logical_sector_start
 * @param size_vec unit is byte
 * @param stream
 * @return
 */
size_t my_fread_spdk_batch(int batch_size, void **data_ptr_vec, const int *logical_sector_start, const size_t *size_vec, struct my_file *stream);

size_t my_fread_spdk_multi_addr(void *ptr, size_t result_byte_size, struct my_file *stream, struct isp_descriptor *isp_desc);

size_t my_fread_spdk_multi_addr_batch(int batch_size, void **ptr, size_t *result_byte_size, struct my_file *stream, struct isp_descriptor **isp_desc);

size_t my_fread_isp(void *ptr, size_t estimated_result_size, struct my_file *stream, struct isp_descriptor *isp_desc, int accelerator_type);

size_t my_fread_isp_batch(int batch_size, void **ptr, size_t *estimated_result_size, struct my_file *stream, struct isp_descriptor **isp_desc, int accelerator_type);

size_t my_fseek(struct my_file *stream, long long offset, int fs_mode);

#endif //TRAJ_BLOCK_FORMAT_PERSISTENCE_MANAGER_H
