//
// Created by yangguo on 11/30/22.
//

#include <malloc.h>
#include "groundhog/persistence_manager.h"
#include "groundhog/common_fs_layer.h"

extern struct spdk_static_fs_desc spdk_static_fs_layer_for_traj;

struct my_file *my_fopen(const char *filename, const char *file_operation_mode, int fs_mode) {
    struct my_file *my_fp = malloc(sizeof(struct my_file));
    if (fs_mode == COMMON_FS_MODE) {
        FILE *fp = common_fs_open(filename, file_operation_mode);
        my_fp->common_file = fp;
        my_fp->filename = filename;
        my_fp->file_operation_mode = file_operation_mode;
        my_fp->fs_mode = fs_mode;
    }
    if (fs_mode == SPDK_FS_MODE) {
        struct spdk_static_file_desc *spdk_fp = spdk_static_fs_fopen(filename, &spdk_static_fs_layer_for_traj);
        my_fp->spdk_file = spdk_fp;
        my_fp->filename = filename;
        my_fp->file_operation_mode = file_operation_mode;
        my_fp->fs_mode = fs_mode;
    }
    return my_fp;
}

size_t my_fwrite(const void *ptr, size_t size, size_t nmemb, struct my_file *stream, int fs_mode) {
    if (fs_mode == COMMON_FS_MODE) {
        return common_fs_write(ptr, size, nmemb, stream->common_file);
    }
    if (fs_mode == SPDK_FS_MODE) {
        return spdk_static_fs_fwrite(ptr, size*nmemb, stream->spdk_file);
    }
}

size_t my_fread(void *ptr, size_t size, size_t nmemb, struct my_file *stream, int fs_mode) {
    if (fs_mode == COMMON_FS_MODE) {
        return common_fs_read(ptr, size, nmemb, stream->common_file);
    }
    if (fs_mode == SPDK_FS_MODE) {
        return spdk_static_fs_fread(ptr, size*nmemb, stream->spdk_file);
    }
    /*if (fs_mode == SPDK_FS_ISP_MODE) {
        return spdk_static_fs_fread_isp(ptr, size*nmemb, stream->spdk_file, stream->isp_desc);
    }*/
}

size_t my_fread_spdk_batch(int batch_size, void **data_ptr_vec, const int *logical_sector_start, const size_t *size_vec, struct my_file *stream) {
    return spdk_static_fs_fread_batch(batch_size, data_ptr_vec, logical_sector_start, size_vec, stream->spdk_file);
}

size_t my_fread_spdk_multi_addr(void *ptr, size_t result_byte_size, struct my_file *stream, struct isp_descriptor *isp_desc) {
    return spdk_static_fs_fread_multi_addr(ptr, result_byte_size, stream->spdk_file, isp_desc);
}

size_t my_fread_spdk_multi_addr_batch(int batch_size, void **ptr, size_t *result_byte_size, struct my_file *stream, struct isp_descriptor **isp_desc) {
    return spdk_static_fs_fread_multi_addr_batch(batch_size, ptr, result_byte_size, stream->spdk_file, isp_desc);
}

size_t my_fread_isp(void *ptr, size_t estimated_result_size, struct my_file *stream, struct isp_descriptor *isp_desc, int accelerator_type) {
    if (accelerator_type == 0) {
        // arm cpu
        spdk_static_fs_fread_isp(ptr, estimated_result_size, stream->spdk_file, isp_desc);
    } else if (accelerator_type == 1) {
        // fpga
        spdk_static_fs_fread_isp_fpga(ptr, estimated_result_size, stream->spdk_file, isp_desc);
    }
}

size_t my_fread_isp_batch(int batch_size, void **ptr, size_t *estimated_result_size, struct my_file *stream, struct isp_descriptor **isp_desc, int accelerator_type) {
    if (accelerator_type == 0) {
        // arm cpu
        spdk_static_fs_fread_isp_batch(batch_size, ptr, estimated_result_size, stream->spdk_file, isp_desc);
    } else if (accelerator_type == 1) {
        // fpga
        spdk_static_fs_fread_isp_fpga_batch(batch_size, ptr, estimated_result_size, stream->spdk_file, isp_desc);
    }
}


size_t my_fseek(struct my_file *stream, long long offset, int fs_mode) {
    if (fs_mode == COMMON_FS_MODE) {
        if (offset > 2147483647) {
            printf("too large offset");
        }
        return common_fs_seek(stream->common_file, offset, SEEK_SET);
    }
    if (fs_mode == SPDK_FS_MODE) {
        if (offset > 9223372036854775807) {
            printf("too large offset");
        }
        return spdk_static_fs_fseek(stream->spdk_file, offset);
    }
}
