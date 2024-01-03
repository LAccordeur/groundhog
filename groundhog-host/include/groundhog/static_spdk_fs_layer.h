//
// Created by yangguo on 22-12-30.
//

#ifndef SPDK_TEST_STATIC_SPDK_FS_LAYER_H
#define SPDK_TEST_STATIC_SPDK_FS_LAYER_H

#include <stddef.h>
#include "spdk/queue.h"
#include "spdk/env.h"
#include "spdk/nvme.h"
#include "spdk/vmd.h"
#include "groundhog/isp_descriptor.h"

#define SECTOR_SIZE 0x1000
#define SUPER_BLOCK_OFFSET 0

/**
 * a simple file system
 * - only support append-only write
 * - write and read should be sector-based
 * - the maximum size of data file is 16 GB and the maximum size of index file and meta file is 4 GB
 *
 *  every time when reading data, you should reset the read offset via fseek()
 *  write_offset is controlled by the program itself
 */
#define DATA_FILENAME "trajectory.data"
#define DATA_FILE_OFFSET 1
#define DATA_FILE_LENGTH 4194304
#define INDEX_FILENAME "trajectory.index"
#define INDEX_FILE_OFFSET 4194305
#define INDEX_FILE_LENGTH 1048576
#define SEG_META_FILENAME "trajectory_seg.meta"
#define SEG_META_FILE_OFFSET 5242881
#define SEG_META_FILE_LENGTH 1048576


struct spdk_static_fs_desc {
    struct spdk_nvme_driver_desc *driver_desc;
    struct spdk_static_file_desc *file_desc_vec;
    int file_desc_vec_num;
};

struct spdk_static_file_desc {
    char filename[128];
    unsigned long int start_lba;    // the start lba of this file
    unsigned int sector_count;  // the length of this file
    unsigned int current_write_offset;    // the current unused offset of this file, starting from 0 to sector_count; only support append-only way
    unsigned int current_read_offset;
    struct spdk_static_fs_desc *fs_desc;
};

struct spdk_nvme_driver_desc {
    struct ns_entry *ns_entry;
};

struct ctrlr_entry {
    struct spdk_nvme_ctrlr		*ctrlr;
    TAILQ_ENTRY(ctrlr_entry)	link;
    char				name[1024];
};

struct ns_entry {
    struct spdk_nvme_ctrlr	*ctrlr;
    struct spdk_nvme_ns	*ns;
    TAILQ_ENTRY(ns_entry)	link;
    struct spdk_nvme_qpair	*qpair;
};

void init_and_mk_fs_for_traj(bool is_flushed);

void spdk_flush_static_fs_meta_for_traj();

void print_spdk_static_fs_meta_for_traj();
/**
 * initialize spdk nvme driver and store driver structure in @driver_desc
 * @param driver_desc
 */
int init_spdk_nvme_driver(struct spdk_nvme_driver_desc *driver_desc);

int cleanup_spdk_nvme_driver(struct spdk_nvme_driver_desc *driver_desc);

void spdk_mk_static_fs(struct spdk_static_fs_desc *fs_desc, struct spdk_static_file_desc *file_desc, int file_desc_num, struct spdk_nvme_driver_desc *driver_desc, bool is_flushed);

void spdk_flush_static_fs_meta(struct spdk_static_fs_desc *fs_desc);

void print_spdk_static_fs_meta(struct spdk_static_fs_desc *fs_desc);

struct spdk_static_file_desc *spdk_static_fs_fopen(const char *filename, struct spdk_static_fs_desc *fs_desc);

size_t spdk_static_fs_fwrite(const void *data_ptr, size_t size, struct spdk_static_file_desc *file_desc);

size_t spdk_static_fs_fread(const void *data_ptr, size_t size, struct spdk_static_file_desc *file_desc);

size_t spdk_static_fs_fread_batch(int batch_size, const void **data_ptr_vec, const int *logical_sector_start, const size_t *size_vec, struct spdk_static_file_desc *file_desc);

size_t spdk_static_fs_fread_multi_addr(const void *data_ptr, size_t size, struct spdk_static_file_desc *file_desc, struct isp_descriptor *isp_desc);

/**
 *
 * @param data_ptr
 * @param size estimated result size
 * @param file_desc
 * @param isp_desc there is a limit (i.e., 256 sector) of total sector number in each operation. The program should make sure that it obeys this condition
 * @return
 */
size_t spdk_static_fs_fread_isp(const void *data_ptr, size_t size, struct spdk_static_file_desc *file_desc, struct isp_descriptor *isp_desc);

size_t spdk_static_fs_fread_isp_fpga(const void *data_ptr, size_t size, struct spdk_static_file_desc *file_desc, struct isp_descriptor *isp_desc);

size_t spdk_static_fs_fread_multi_addr_batch(int batch_size, const void **data_ptr_vec, const size_t *size_vec, struct spdk_static_file_desc *file_desc, struct isp_descriptor **isp_desc_vec);

size_t spdk_static_fs_fread_isp_batch(int batch_size, const void **data_ptr_vec, const size_t *size_vec, struct spdk_static_file_desc *file_desc, struct isp_descriptor **isp_desc_vec);

size_t spdk_static_fs_fread_isp_fpga_batch(int batch_size, const void **data_ptr_vec, const size_t *size_vec, struct spdk_static_file_desc *file_desc, struct isp_descriptor **isp_desc_vec);

size_t spdk_static_fs_fseek(struct spdk_static_file_desc *file_desc, long long offset);


#endif //SPDK_TEST_STATIC_SPDK_FS_LAYER_H


