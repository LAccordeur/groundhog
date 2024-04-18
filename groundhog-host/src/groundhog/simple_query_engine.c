//
// Created by yangguo on 11/16/22.
//
#define _GNU_SOURCE
#include "groundhog/simple_query_engine.h"
#include "groundhog/porto_dataset_reader.h"
#include "groundhog/traj_block_format.h"
#include "groundhog/config.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sched.h>
#include "groundhog/log.h"
#include "groundhog/isp_output_format.h"
#include "time.h"
#include "groundhog/bloom/bloom.h"
#include "groundhog/normalization_util.h"
#include "groundhog/traj_processing.h"
#include "groundhog/nyc_dataset_reader.h"
#include "groundhog/knn_util.h"
#include "groundhog/osm_dataset_reader.h"

static bool enable_estimated_result_size = false;
static int REQUEST_BATCH_SIZE = 1;  // when using a large batch size, the performance is not good. This needs more investigation

struct host_result_buffer {
    void *buffer_ptr;
    int buffer_size;
    int point_count;
};


int
run_spatio_temporal_query_in_host(struct spatio_temporal_range_predicate *predicate, struct traj_storage *data_storage,
                                  int block_logical_addr_count, int *block_logical_addr_vec);

int
run_spatio_temporal_query_device(struct spatio_temporal_range_predicate *predicate, struct traj_storage *data_storage,
                                 struct seg_meta_section_entry_storage *meta_storage, int block_logical_addr_count,
                                 int *block_logical_addr_vec, bool enable_result_estimation);

int
run_spatio_temporal_query_device_fpga(struct spatio_temporal_range_predicate *predicate, struct traj_storage *data_storage,
                                      struct seg_meta_section_entry_storage *meta_storage, int block_logical_addr_count,
                                      int *block_logical_addr_vec, bool enable_result_estimation);

int
run_id_temporal_query_device(struct id_temporal_predicate *predicate, struct traj_storage *data_storage,
                                 struct seg_meta_section_entry_storage *meta_storage, int block_logical_addr_count,
                                 int *block_logical_addr_vec, bool enable_result_estimation);

int
run_id_temporal_query_device_fpga(struct id_temporal_predicate *predicate, struct traj_storage *data_storage,
                                  struct seg_meta_section_entry_storage *meta_storage, int block_logical_addr_count,
                                  int *block_logical_addr_vec, bool enable_result_estimation);

int
run_id_temporal_query_in_host(struct id_temporal_predicate *predicate, struct traj_storage *data_storage,
                              int block_logical_addr_count, int *block_logical_addr_vec);

int run_spatio_temporal_count_query_device_batch_naive(struct spatio_temporal_range_predicate *predicate, struct traj_storage *data_storage,
                                                       int block_logical_addr_count, int *block_logical_addr_vec);

int run_spatio_temporal_count_query_device_batch(struct spatio_temporal_range_predicate *predicate, struct traj_storage *data_storage,
                                                 int block_logical_addr_count, int *block_logical_addr_vec);

int
run_spatio_temporal_query_device_batch_naive(struct spatio_temporal_range_predicate *predicate, struct traj_storage *data_storage,
                                             struct seg_meta_section_entry_storage *meta_storage, int block_logical_addr_count,
                                             int *block_logical_addr_vec, bool enable_result_estimation);

int
run_spatio_temporal_query_device_batch(struct spatio_temporal_range_predicate *predicate, struct traj_storage *data_storage,
                                            struct seg_meta_section_entry_storage *meta_storage, int block_logical_addr_count,
                                            int *block_logical_addr_vec, bool enable_result_estimation);

int
run_spatio_temporal_query_device_fpga_batch(struct spatio_temporal_range_predicate *predicate, struct traj_storage *data_storage,
                                            struct seg_meta_section_entry_storage *meta_storage, int block_logical_addr_count,
                                            int *block_logical_addr_vec, bool enable_result_estimation);

int
run_id_temporal_query_device_batch_naive(struct id_temporal_predicate *predicate, struct traj_storage *data_storage,
                                         struct seg_meta_section_entry_storage *meta_storage, int block_logical_addr_count,
                                         int *block_logical_addr_vec, bool enable_result_estimation);

int
run_id_temporal_query_device_batch(struct id_temporal_predicate *predicate, struct traj_storage *data_storage,
                             struct seg_meta_section_entry_storage *meta_storage, int block_logical_addr_count,
                             int *block_logical_addr_vec, bool enable_result_estimation);

int
run_id_temporal_query_device_fpga_batch(struct id_temporal_predicate *predicate, struct traj_storage *data_storage,
                                  struct seg_meta_section_entry_storage *meta_storage, int block_logical_addr_count,
                                  int *block_logical_addr_vec, bool enable_result_estimation);

int
run_spatio_temporal_count_query_in_host_batch(struct spatio_temporal_range_predicate *predicate, struct traj_storage *data_storage,
                                              int block_logical_addr_count, int *block_logical_addr_vec);

int
run_knn_query_in_host_batch(struct spatio_temporal_knn_predicate *predicate, struct traj_storage *data_storage,
                            int block_logical_addr_count, int *block_logical_addr_vec);

int
run_spatio_temporal_knn_query_host_multi_addr_batch(struct spatio_temporal_knn_predicate *predicate, struct traj_storage *data_storage,
                                                    int block_logical_addr_count,
                                                    int *block_logical_addr_vec);

int
run_spatio_temporal_query_in_host_batch(struct spatio_temporal_range_predicate *predicate, struct traj_storage *data_storage,
                                        int block_logical_addr_count, int *block_logical_addr_vec);

int
run_id_temporal_query_in_host_batch(struct id_temporal_predicate *predicate, struct traj_storage *data_storage,
                                    int block_logical_addr_count, int *block_logical_addr_vec);

int
run_spatio_temporal_query_host_multi_addr(struct spatio_temporal_range_predicate *predicate, struct traj_storage *data_storage,int block_logical_addr_count,
                                          int *block_logical_addr_vec);

int
run_spatio_temporal_query_host_multi_addr_batch(struct spatio_temporal_range_predicate *predicate, struct traj_storage *data_storage,
                                                int block_logical_addr_count,
                                                int *block_logical_addr_vec);


int
run_spatio_temporal_query_hybrid_batch(struct spatio_temporal_range_predicate *predicate, struct traj_storage *data_storage, struct seg_meta_section_entry_storage *meta_storage,
                                       int host_block_logical_addr_count,
                                       int *host_block_logical_addr_vec,
                                       int pushdown_block_logical_addr_count,
                                       int *pushdown_block_logical_addr_vec,
                                       bool enable_result_estimation);

int
run_id_temporal_query_host_multi_addr_batch(struct id_temporal_predicate *predicate, struct traj_storage *data_storage,
                                            int block_logical_addr_count,
                                            int *block_logical_addr_vec);

int
run_spatio_temporal_knn_query_device_batch(struct spatio_temporal_knn_predicate *predicate, struct traj_storage *data_storage,
                                           int block_logical_addr_count, int *block_logical_addr_vec, int option) ;

int
run_knn_join_query_in_host_batch(struct spatio_temporal_knn_join_predicate *predicate, struct traj_storage *data_storage,
                                 int block_logical_addr_count, int *block_logical_addr_vec, struct knnjoin_result_buffer *knnjoin_buffer);

int
run_spatio_temporal_knn_join_query_device_batch(struct spatio_temporal_knn_join_predicate *predicate, struct traj_storage *data_storage,
                                                int block_logical_addr_count, int *block_logical_addr_vec, struct knnjoin_result_buffer *knnjoin_buffer, int option);

int
run_knn_join_query_in_host_fileapi(struct spatio_temporal_knn_join_predicate *predicate, struct traj_storage *data_storage,
                                   int block_logical_addr_count, int *block_logical_addr_vec, struct knnjoin_result_buffer *knnjoin_buffer);

static int check_oid_exist(const int *oids, int array_size, int checked_oid) {
    for (int i = 0; i < array_size; i++) {
        if (oids[i] == checked_oid) {
            return 1;
        }
    }
    return -1;
}

static void parse_seg_meta_section(void *metadata, struct seg_meta *meta_array, int array_size) {
    char* m = metadata;
    int seg_meta_size = get_seg_meta_size();
    for (int i= 0; i < array_size; i++) {
        char* offset = m + i * seg_meta_size;
        deserialize_seg_meta(offset, meta_array + i);
    }
}

void init_query_engine(struct simple_query_engine *engine) {
    init_index_entry_storage(&(engine->index_storage));
    init_traj_storage(&(engine->data_storage));
    init_seg_meta_entry_storage(&(engine->seg_meta_storage));
}

void init_query_engine_with_persistence(struct simple_query_engine *engine, struct my_file *data_file, struct my_file *index_file, struct my_file *meta_file) {
    init_index_entry_storage_with_persistence(&(engine->index_storage), index_file->filename, index_file->file_operation_mode, index_file->fs_mode);
    init_traj_storage_with_persistence(&(engine->data_storage), data_file->filename, data_file->file_operation_mode, data_file->fs_mode);
    init_seg_meta_entry_storage_with_persistence(&(engine->seg_meta_storage), meta_file->filename, meta_file->file_operation_mode, meta_file->fs_mode);
}

void free_query_engine(struct simple_query_engine *engine) {
    free_index_entry_storage(&(engine->index_storage));
    free_traj_storage(&(engine->data_storage));
    free_seg_meta_entry_storage(&(engine->seg_meta_storage));
}

void ingest_data_via_time_partition(struct simple_query_engine *engine, FILE *fp, int block_num) {
    struct traj_storage *data_storage = &engine->data_storage;
    struct index_entry_storage *index_storage = &engine->index_storage;
    struct seg_meta_section_entry_storage *meta_storage = &engine->seg_meta_storage;
    // trajectory block info
    int points_num = calculate_points_num_via_block_size(TRAJ_BLOCK_SIZE, SPLIT_SEGMENT_NUM);

    int buffer_size = 1024 * 8; // the number of block
    int i;
    for (i = 0; i < block_num; i+= buffer_size) {

        int used_buffer_size = block_num - i < buffer_size ? block_num - i : buffer_size;
        int total_points_num = points_num * used_buffer_size;

        struct traj_point **points_buffer = allocate_points_memory(total_points_num);
        int read_line_num = read_points_from_csv(fp, points_buffer, i * points_num, total_points_num);
        if (read_line_num == 0) {
            break;
        }
        sort_traj_points(points_buffer, total_points_num);

        for (int j = 0; j < used_buffer_size; j++) {
            struct traj_point **points = points_buffer + j * points_num;

            // convert and put this data to traj storage
            void *data = malloc(TRAJ_BLOCK_SIZE);
            do_self_contained_traj_block(points, points_num, data, TRAJ_BLOCK_SIZE);
            struct address_pair data_addresses = append_traj_block_to_storage(data_storage, data);

            // update index
            struct index_entry *entry = malloc(sizeof(struct index_entry));
            init_index_entry(entry);
            fill_index_entry(entry, points, points_num, data_addresses.physical_ptr, data_addresses.logical_adr);
            append_index_entry_to_storage(index_storage, entry);

            // update seg_meta store
            struct seg_meta_section_entry *seg_entry = (struct seg_meta_section_entry *)malloc(sizeof(struct seg_meta_section_entry));
            struct traj_block_header header;
            parse_traj_block_for_header(data, &header);
            int meta_section_size = get_seg_meta_section_size(data);
            void* meta_section = malloc(meta_section_size);
            extract_seg_meta_section(data, meta_section);
            seg_entry->seg_meta_count = header.seg_count;
            seg_entry->seg_meta_section = meta_section;
            seg_entry->block_logical_adr = data_addresses.logical_adr;
            append_to_seg_meta_entry_storage(meta_storage, seg_entry);
        }

        free_points_memory(points_buffer, total_points_num);
    }

    debug_print("[ingest_data_via_time_partition] num of ingesting data points: %d\n", points_num * (block_num - 1));

}

void ingest_nyc_data_via_time_partition(struct simple_query_engine *engine, FILE *fp, int block_num) {
    struct traj_storage *data_storage = &engine->data_storage;
    struct index_entry_storage *index_storage = &engine->index_storage;
    struct seg_meta_section_entry_storage *meta_storage = &engine->seg_meta_storage;
    // trajectory block info
    int points_num = calculate_points_num_via_block_size(TRAJ_BLOCK_SIZE, SPLIT_SEGMENT_NUM);

    for (int i = 0; i < block_num; i++) {
        struct traj_point **points = allocate_points_memory(points_num);
        read_points_from_csv_nyc(fp,points, i*points_num, points_num);
        // convert and put this data to traj storage
        void *data = malloc(TRAJ_BLOCK_SIZE);
        sort_traj_points(points, points_num);
        do_self_contained_traj_block(points, points_num, data, TRAJ_BLOCK_SIZE);
        struct address_pair data_addresses = append_traj_block_to_storage(data_storage, data);

        // update index
        struct index_entry *entry = malloc(sizeof(struct index_entry));
        init_index_entry(entry);
        fill_index_entry(entry, points, points_num, data_addresses.physical_ptr, data_addresses.logical_adr);
        append_index_entry_to_storage(index_storage, entry);

        // update seg_meta store
        struct seg_meta_section_entry *seg_entry = (struct seg_meta_section_entry *)malloc(sizeof(struct seg_meta_section_entry));
        struct traj_block_header header;
        parse_traj_block_for_header(data, &header);
        int meta_section_size = get_seg_meta_section_size(data);
        void* meta_section = malloc(meta_section_size);
        extract_seg_meta_section(data, meta_section);
        seg_entry->seg_meta_count = header.seg_count;
        seg_entry->seg_meta_section = meta_section;
        seg_entry->block_logical_adr = data_addresses.logical_adr;
        append_to_seg_meta_entry_storage(meta_storage, seg_entry);

        free_points_memory(points, points_num);
    }

    debug_print("[ingest_data_via_time_partition] num of ingesting data points: %d\n", points_num * (block_num - 1));

}

/**
 * block index specifies from where we read the data in the file
 * @param engine
 * @param fp
 * @param block_num
 */
void ingest_data_via_time_partition_with_block_index(struct simple_query_engine *engine, FILE *fp, int block_index, int block_num) {
    struct traj_storage *data_storage = &engine->data_storage;
    struct index_entry_storage *index_storage = &engine->index_storage;
    struct seg_meta_section_entry_storage *meta_storage = &engine->seg_meta_storage;
    // trajectory block info
    int points_num = calculate_points_num_via_block_size(TRAJ_BLOCK_SIZE, SPLIT_SEGMENT_NUM);

    for (int i = 0; i < block_num; i++) {
        struct traj_point **points = allocate_points_memory(points_num);
        read_points_from_csv(fp,points, (i+block_index)*points_num, points_num);
        // convert and put this data to traj storage
        void *data = malloc(TRAJ_BLOCK_SIZE);
        sort_traj_points(points, points_num);
        do_self_contained_traj_block(points, points_num, data, TRAJ_BLOCK_SIZE);
        struct address_pair data_addresses = append_traj_block_to_storage(data_storage, data);

        // update index
        struct index_entry *entry = malloc(sizeof(struct index_entry));
        init_index_entry(entry);
        fill_index_entry(entry, points, points_num, data_addresses.physical_ptr, data_addresses.logical_adr + block_index);
        append_index_entry_to_storage(index_storage, entry);

        // update seg_meta store
        struct seg_meta_section_entry *seg_entry = (struct seg_meta_section_entry *)malloc(sizeof(struct seg_meta_section_entry));
        struct traj_block_header header;
        parse_traj_block_for_header(data, &header);
        int meta_section_size = get_seg_meta_section_size(data);
        void* meta_section = malloc(meta_section_size);
        extract_seg_meta_section(data, meta_section);
        seg_entry->seg_meta_count = header.seg_count;
        seg_entry->seg_meta_section = meta_section;
        seg_entry->block_logical_adr = data_addresses.logical_adr + block_index;
        append_to_seg_meta_entry_storage(meta_storage, seg_entry);

        free_points_memory(points, points_num);
    }

    debug_print("[ingest_data_via_time_partition_via_block_index] num of ingesting data points: %d\n", points_num * (block_num - 1));

}


void ingest_data_via_zcurve_partition(struct simple_query_engine *engine, FILE *fp, int block_num) {
    struct traj_storage *data_storage = &engine->data_storage;
    struct index_entry_storage *index_storage = &engine->index_storage;
    struct seg_meta_section_entry_storage *meta_storage = &engine->seg_meta_storage;
    // trajectory block info
    int points_num = calculate_points_num_via_block_size(TRAJ_BLOCK_SIZE, SPLIT_SEGMENT_NUM);

    int buffer_size = 1024 * 8; // the number of block
    int i;
    for (i = 0; i < block_num; i+= buffer_size) {

        int used_buffer_size = block_num - i < buffer_size ? block_num - i : buffer_size;
        int total_points_num = points_num * used_buffer_size;

        struct traj_point **points_buffer = allocate_points_memory(total_points_num);
        int read_line_num = read_points_from_csv(fp, points_buffer, i * points_num, total_points_num);
        if (read_line_num == 0) {
            break;
        }
        sort_traj_points_zcurve(points_buffer, total_points_num);

        for (int j = 0; j < used_buffer_size; j++) {
            struct traj_point **points = points_buffer + j * points_num;

            // convert and put this data to traj storage
            void *data = malloc(TRAJ_BLOCK_SIZE);
            do_self_contained_traj_block(points, points_num, data, TRAJ_BLOCK_SIZE);
            struct address_pair data_addresses = append_traj_block_to_storage(data_storage, data);

            // update index
            struct index_entry *entry = malloc(sizeof(struct index_entry));
            init_index_entry(entry);
            fill_index_entry(entry, points, points_num, data_addresses.physical_ptr, data_addresses.logical_adr);
            append_index_entry_to_storage(index_storage, entry);

            // update seg_meta store
            struct seg_meta_section_entry *seg_entry = (struct seg_meta_section_entry *)malloc(sizeof(struct seg_meta_section_entry));
            struct traj_block_header header;
            parse_traj_block_for_header(data, &header);
            int meta_section_size = get_seg_meta_section_size(data);
            void* meta_section = malloc(meta_section_size);
            extract_seg_meta_section(data, meta_section);
            seg_entry->seg_meta_count = header.seg_count;
            seg_entry->seg_meta_section = meta_section;
            seg_entry->block_logical_adr = data_addresses.logical_adr;
            append_to_seg_meta_entry_storage(meta_storage, seg_entry);
        }

        free_points_memory(points_buffer, total_points_num);
    }


    debug_print("[ingest_data_via_zcurve_partition] num of ingesting data points (not accurate): %d\n", points_num * (i - 1));

}

void ingest_nyc_data_via_zcurve_partition(struct simple_query_engine *engine, FILE *fp, int block_num) {
    struct traj_storage *data_storage = &engine->data_storage;
    struct index_entry_storage *index_storage = &engine->index_storage;
    struct seg_meta_section_entry_storage *meta_storage = &engine->seg_meta_storage;
    // trajectory block info
    int points_num = calculate_points_num_via_block_size(TRAJ_BLOCK_SIZE, SPLIT_SEGMENT_NUM);

    int buffer_size = 1024 * 8; // the number of block
    int i;
    for (i = 0; i < block_num; i+= buffer_size) {

        int used_buffer_size = block_num - i < buffer_size ? block_num - i : buffer_size;
        int total_points_num = points_num * used_buffer_size;

        struct traj_point **points_buffer = allocate_points_memory(total_points_num);
        int read_line_num = read_points_from_csv_nyc(fp, points_buffer, i * points_num, total_points_num);
        if (read_line_num == 0) {
            break;
        }
        sort_traj_points_zcurve(points_buffer, total_points_num);

        for (int j = 0; j < used_buffer_size; j++) {
            struct traj_point **points = points_buffer + j * points_num;

            // convert and put this data to traj storage
            void *data = malloc(TRAJ_BLOCK_SIZE);
            do_self_contained_traj_block(points, points_num, data, TRAJ_BLOCK_SIZE);
            struct address_pair data_addresses = append_traj_block_to_storage(data_storage, data);

            // update index
            struct index_entry *entry = malloc(sizeof(struct index_entry));
            init_index_entry(entry);
            fill_index_entry(entry, points, points_num, data_addresses.physical_ptr, data_addresses.logical_adr);
            append_index_entry_to_storage(index_storage, entry);

            // update seg_meta store
            struct seg_meta_section_entry *seg_entry = (struct seg_meta_section_entry *)malloc(sizeof(struct seg_meta_section_entry));
            struct traj_block_header header;
            parse_traj_block_for_header(data, &header);
            int meta_section_size = get_seg_meta_section_size(data);
            void* meta_section = malloc(meta_section_size);
            extract_seg_meta_section(data, meta_section);
            seg_entry->seg_meta_count = header.seg_count;
            seg_entry->seg_meta_section = meta_section;
            seg_entry->block_logical_adr = data_addresses.logical_adr;
            append_to_seg_meta_entry_storage(meta_storage, seg_entry);
        }

        free_points_memory(points_buffer, total_points_num);
    }


    debug_print("[ingest_nyc_data_via_zcurve_partition] num of ingesting data points (not accurate): %d\n", points_num * (i - 1));

}

void ingest_data_via_zcurve_partition_with_block_index(struct simple_query_engine *engine, FILE *fp, int block_index, int block_num) {
    struct traj_storage *data_storage = &engine->data_storage;
    struct index_entry_storage *index_storage = &engine->index_storage;
    struct seg_meta_section_entry_storage *meta_storage = &engine->seg_meta_storage;
    // trajectory block info
    int points_num = calculate_points_num_via_block_size(TRAJ_BLOCK_SIZE, SPLIT_SEGMENT_NUM);

    int buffer_size = 1024 * 8; // the number of block
    for (int i = 0; i < block_num; i+= buffer_size) {

        int used_buffer_size = block_num - i < buffer_size ? block_num - i : buffer_size;
        int total_points_num = points_num * used_buffer_size;

        struct traj_point **points_buffer = allocate_points_memory(total_points_num);
        read_points_from_csv(fp, points_buffer, (i + block_index) * points_num, total_points_num);
        sort_traj_points_zcurve(points_buffer, total_points_num);

        for (int j = 0; j < used_buffer_size; j++) {
            struct traj_point **points = points_buffer + j * points_num;

            // convert and put this data to traj storage
            void *data = malloc(TRAJ_BLOCK_SIZE);
            do_self_contained_traj_block(points, points_num, data, TRAJ_BLOCK_SIZE);
            struct address_pair data_addresses = append_traj_block_to_storage(data_storage, data);

            // update index
            struct index_entry *entry = malloc(sizeof(struct index_entry));
            init_index_entry(entry);
            fill_index_entry(entry, points, points_num, data_addresses.physical_ptr, data_addresses.logical_adr + block_index);
            append_index_entry_to_storage(index_storage, entry);

            // update seg_meta store
            struct seg_meta_section_entry *seg_entry = (struct seg_meta_section_entry *)malloc(sizeof(struct seg_meta_section_entry));
            struct traj_block_header header;
            parse_traj_block_for_header(data, &header);
            int meta_section_size = get_seg_meta_section_size(data);
            void* meta_section = malloc(meta_section_size);
            extract_seg_meta_section(data, meta_section);
            seg_entry->seg_meta_count = header.seg_count;
            seg_entry->seg_meta_section = meta_section;
            seg_entry->block_logical_adr = data_addresses.logical_adr + block_index;
            append_to_seg_meta_entry_storage(meta_storage, seg_entry);
        }

        free_points_memory(points_buffer, total_points_num);
    }


    debug_print("[ingest_data_via_zcurve_partition_via_block_index] num of ingesting data points: %d\n", points_num * (block_num - 1));

}


void ingest_osm_data_via_zcurve_partition_with_block_index(struct simple_query_engine *engine, FILE *fp, int block_index, int block_num) {
    struct traj_storage *data_storage = &engine->data_storage;
    struct index_entry_storage *index_storage = &engine->index_storage;
    struct seg_meta_section_entry_storage *meta_storage = &engine->seg_meta_storage;
    // trajectory block info
    int points_num = calculate_points_num_via_block_size(TRAJ_BLOCK_SIZE, SPLIT_SEGMENT_NUM);

    int buffer_size = 1024 * 8; // the number of block
    for (int i = 0; i < block_num; i+= buffer_size) {

        int used_buffer_size = block_num - i < buffer_size ? block_num - i : buffer_size;
        int total_points_num = points_num * used_buffer_size;

        struct traj_point **points_buffer = allocate_points_memory(total_points_num);
        read_points_from_csv_osm(fp, points_buffer, (i + block_index) * points_num, total_points_num);
        sort_traj_points_zcurve(points_buffer, total_points_num);

        for (int j = 0; j < used_buffer_size; j++) {
            struct traj_point **points = points_buffer + j * points_num;

            // convert and put this data to traj storage
            void *data = malloc(TRAJ_BLOCK_SIZE);
            do_self_contained_traj_block(points, points_num, data, TRAJ_BLOCK_SIZE);
            struct address_pair data_addresses = append_traj_block_to_storage(data_storage, data);

            // update index
            struct index_entry *entry = malloc(sizeof(struct index_entry));
            init_index_entry(entry);
            fill_index_entry(entry, points, points_num, data_addresses.physical_ptr, data_addresses.logical_adr + block_index);
            append_index_entry_to_storage(index_storage, entry);

            // update seg_meta store
            struct seg_meta_section_entry *seg_entry = (struct seg_meta_section_entry *)malloc(sizeof(struct seg_meta_section_entry));
            struct traj_block_header header;
            parse_traj_block_for_header(data, &header);
            int meta_section_size = get_seg_meta_section_size(data);
            void* meta_section = malloc(meta_section_size);
            extract_seg_meta_section(data, meta_section);
            seg_entry->seg_meta_count = header.seg_count;
            seg_entry->seg_meta_section = meta_section;
            seg_entry->block_logical_adr = data_addresses.logical_adr + block_index;
            append_to_seg_meta_entry_storage(meta_storage, seg_entry);
        }

        free_points_memory(points_buffer, total_points_num);
    }


    debug_print("[ingest_osm_data_via_zcurve_partition_with_block_index] num of ingesting data points: %d\n", points_num * (block_num - 1));

}

void ingest_osm_data_via_time_partition_with_block_index(struct simple_query_engine *engine, FILE *fp, int block_index, int block_num) {
    struct traj_storage *data_storage = &engine->data_storage;
    struct index_entry_storage *index_storage = &engine->index_storage;
    struct seg_meta_section_entry_storage *meta_storage = &engine->seg_meta_storage;
    // trajectory block info
    int points_num = calculate_points_num_via_block_size(TRAJ_BLOCK_SIZE, SPLIT_SEGMENT_NUM);

    for (int i = 0; i < block_num; i++) {
        struct traj_point **points = allocate_points_memory(points_num);
        read_points_from_csv_osm(fp,points, (i+block_index)*points_num, points_num);
        // convert and put this data to traj storage
        void *data = malloc(TRAJ_BLOCK_SIZE);
        sort_traj_points(points, points_num);
        do_self_contained_traj_block(points, points_num, data, TRAJ_BLOCK_SIZE);
        struct address_pair data_addresses = append_traj_block_to_storage(data_storage, data);

        // update index
        struct index_entry *entry = malloc(sizeof(struct index_entry));
        init_index_entry(entry);
        fill_index_entry(entry, points, points_num, data_addresses.physical_ptr, data_addresses.logical_adr + block_index);
        append_index_entry_to_storage(index_storage, entry);

        // update seg_meta store
        struct seg_meta_section_entry *seg_entry = (struct seg_meta_section_entry *)malloc(sizeof(struct seg_meta_section_entry));
        struct traj_block_header header;
        parse_traj_block_for_header(data, &header);
        int meta_section_size = get_seg_meta_section_size(data);
        void* meta_section = malloc(meta_section_size);
        extract_seg_meta_section(data, meta_section);
        seg_entry->seg_meta_count = header.seg_count;
        seg_entry->seg_meta_section = meta_section;
        seg_entry->block_logical_adr = data_addresses.logical_adr + block_index;
        append_to_seg_meta_entry_storage(meta_storage, seg_entry);

        free_points_memory(points, points_num);
    }

    debug_print("[ingest_osm_data_via_time_partition_with_block_index] num of ingesting data points: %d\n", points_num * (block_num - 1));

}


void ingest_synthetic_data_via_time_partition(struct simple_query_engine *engine, int block_num) {

    struct traj_storage *data_storage = &engine->data_storage;
    struct index_entry_storage *index_storage = &engine->index_storage;
    struct seg_meta_section_entry_storage *meta_storage = &engine->seg_meta_storage;
    // trajectory block info
    int points_num = calculate_points_num_via_block_size(TRAJ_BLOCK_SIZE, SPLIT_SEGMENT_NUM);

    for (int i = 0; i < block_num; i++) {
        struct traj_point **points = allocate_points_memory(points_num);
        generate_synthetic_points(points, i*points_num, points_num);
        //generate_synthetic_random_points(points, i*points_num, points_num, 31850495);
        // convert and put this data to traj storage
        void *data = malloc(TRAJ_BLOCK_SIZE);
        sort_traj_points(points, points_num);
        do_self_contained_traj_block(points, points_num, data, TRAJ_BLOCK_SIZE);
        struct address_pair data_addresses = append_traj_block_to_storage(data_storage, data);

        // update index
        struct index_entry *entry = malloc(sizeof(struct index_entry));
        init_index_entry(entry);
        fill_index_entry(entry, points, points_num, data_addresses.physical_ptr, data_addresses.logical_adr);
        append_index_entry_to_storage(index_storage, entry);

        // update seg_meta store
        struct seg_meta_section_entry *seg_entry = (struct seg_meta_section_entry *)malloc(sizeof(struct seg_meta_section_entry));
        struct traj_block_header header;
        parse_traj_block_for_header(data, &header);
        int meta_section_size = get_seg_meta_section_size(data);
        void* meta_section = malloc(meta_section_size);
        extract_seg_meta_section(data, meta_section);
        seg_entry->seg_meta_count = header.seg_count;
        seg_entry->seg_meta_section = meta_section;
        seg_entry->block_logical_adr = data_addresses.logical_adr;
        append_to_seg_meta_entry_storage(meta_storage, seg_entry);

        free_points_memory(points, points_num);

        /*if (i*points_num>25559039) {
            break;
        }*/
    }
    debug_print("[ingest_synthetic_data_via_time_partition] num of ingesting data points: %d\n", points_num * (block_num - 1));
}

void ingest_synthetic_data_via_time_partition_with_block_index(struct simple_query_engine *engine, int block_index, int block_num) {

    struct traj_storage *data_storage = &engine->data_storage;
    struct index_entry_storage *index_storage = &engine->index_storage;
    struct seg_meta_section_entry_storage *meta_storage = &engine->seg_meta_storage;
    // trajectory block info
    int points_num = calculate_points_num_via_block_size(TRAJ_BLOCK_SIZE, SPLIT_SEGMENT_NUM);

    for (int i = 0; i < block_num; i++) {
        struct traj_point **points = allocate_points_memory(points_num);
        generate_synthetic_points(points, (i + block_index)*points_num, points_num);
        //generate_synthetic_random_points(points, (i+block_index)*points_num, points_num, 29491199);
        // convert and put this data to traj storage
        void *data = malloc(TRAJ_BLOCK_SIZE);
        sort_traj_points(points, points_num);
        do_self_contained_traj_block(points, points_num, data, TRAJ_BLOCK_SIZE);
        struct address_pair data_addresses = append_traj_block_to_storage(data_storage, data);

        // update index
        struct index_entry *entry = malloc(sizeof(struct index_entry));
        init_index_entry(entry);
        fill_index_entry(entry, points, points_num, data_addresses.physical_ptr, data_addresses.logical_adr + block_index);
        append_index_entry_to_storage(index_storage, entry);

        // update seg_meta store
        struct seg_meta_section_entry *seg_entry = (struct seg_meta_section_entry *)malloc(sizeof(struct seg_meta_section_entry));
        struct traj_block_header header;
        parse_traj_block_for_header(data, &header);
        int meta_section_size = get_seg_meta_section_size(data);
        void* meta_section = malloc(meta_section_size);
        extract_seg_meta_section(data, meta_section);
        seg_entry->seg_meta_count = header.seg_count;
        seg_entry->seg_meta_section = meta_section;
        seg_entry->block_logical_adr = data_addresses.logical_adr + block_index;
        append_to_seg_meta_entry_storage(meta_storage, seg_entry);

        free_points_memory(points, points_num);
    }
    debug_print("[ingest_synthetic_data_via_time_partition] num of ingesting data points: %d\n", points_num * (block_num - 1));
}

void ingest_and_flush_data_via_time_partition(struct simple_query_engine *engine, FILE *fp, int block_num) {
    // ingest to memory
    ingest_data_via_time_partition(engine, fp, block_num);

    // flush memory to disk
    struct traj_storage *data_storage = &engine->data_storage;
    struct index_entry_storage *index_storage = &engine->index_storage;
    struct seg_meta_section_entry_storage *meta_storage = &engine->seg_meta_storage;

    // flush data storage
    flush_traj_storage(data_storage);
    // serialized and flush index storage
    struct serialized_index_storage serialized_index;
    init_serialized_index_storage(&serialized_index);
    serialize_index_entry_storage(index_storage, &serialized_index);
    flush_serialized_index_storage(&serialized_index, index_storage->my_fp->filename, index_storage->my_fp->fs_mode);
    free_serialized_index_storage(&serialized_index);
    // serialize and flush seg meta storage
    struct serialized_seg_meta_section_entry_storage serialized_seg_meta;
    init_serialized_seg_meta_section_entry_storage(&serialized_seg_meta);
    serialize_seg_meta_section_entry_storage(meta_storage, &serialized_seg_meta);
    flush_serialized_seg_meta_storage(&serialized_seg_meta, meta_storage->my_fp->filename, meta_storage->my_fp->fs_mode);
    free_serialized_seg_meta_section_entry_storage(&serialized_seg_meta);

}

void ingest_and_flush_nyc_data_via_time_partition(struct simple_query_engine *engine, FILE *fp, int block_num) {
    // ingest to memory
    ingest_nyc_data_via_time_partition(engine, fp, block_num);

    // flush memory to disk
    struct traj_storage *data_storage = &engine->data_storage;
    struct index_entry_storage *index_storage = &engine->index_storage;
    struct seg_meta_section_entry_storage *meta_storage = &engine->seg_meta_storage;

    // flush data storage
    flush_traj_storage(data_storage);
    // serialized and flush index storage
    struct serialized_index_storage serialized_index;
    init_serialized_index_storage(&serialized_index);
    serialize_index_entry_storage(index_storage, &serialized_index);
    flush_serialized_index_storage(&serialized_index, index_storage->my_fp->filename, index_storage->my_fp->fs_mode);
    free_serialized_index_storage(&serialized_index);
    // serialize and flush seg meta storage
    struct serialized_seg_meta_section_entry_storage serialized_seg_meta;
    init_serialized_seg_meta_section_entry_storage(&serialized_seg_meta);
    serialize_seg_meta_section_entry_storage(meta_storage, &serialized_seg_meta);
    flush_serialized_seg_meta_storage(&serialized_seg_meta, meta_storage->my_fp->filename, meta_storage->my_fp->fs_mode);
    free_serialized_seg_meta_section_entry_storage(&serialized_seg_meta);

}


void ingest_and_flush_data_via_time_partition_with_block_index(struct simple_query_engine *engine, FILE *fp, int block_index, int block_num) {
    // ingest to memory
    ingest_data_via_time_partition_with_block_index(engine, fp, block_index, block_num);

    // flush memory to disk
    struct traj_storage *data_storage = &engine->data_storage;
    struct index_entry_storage *index_storage = &engine->index_storage;
    struct seg_meta_section_entry_storage *meta_storage = &engine->seg_meta_storage;

    // flush data storage
    flush_traj_storage(data_storage);
    // serialized and flush index storage
    struct serialized_index_storage serialized_index;
    init_serialized_index_storage(&serialized_index);
    serialize_index_entry_storage(index_storage, &serialized_index);
    flush_serialized_index_storage(&serialized_index, index_storage->my_fp->filename, index_storage->my_fp->fs_mode);
    free_serialized_index_storage(&serialized_index);
    // serialize and flush seg meta storage
    struct serialized_seg_meta_section_entry_storage serialized_seg_meta;
    init_serialized_seg_meta_section_entry_storage(&serialized_seg_meta);
    serialize_seg_meta_section_entry_storage(meta_storage, &serialized_seg_meta);
    flush_serialized_seg_meta_storage(&serialized_seg_meta, meta_storage->my_fp->filename, meta_storage->my_fp->fs_mode);
    free_serialized_seg_meta_section_entry_storage(&serialized_seg_meta);

}

void ingest_and_flush_data_via_zcurve_partition(struct simple_query_engine *engine, FILE *fp, int block_num) {
    // ingest to memory
    ingest_data_via_zcurve_partition(engine, fp, block_num);

    // flush memory to disk
    struct traj_storage *data_storage = &engine->data_storage;
    struct index_entry_storage *index_storage = &engine->index_storage;
    struct seg_meta_section_entry_storage *meta_storage = &engine->seg_meta_storage;

    // flush data storage
    flush_traj_storage(data_storage);
    // serialized and flush index storage
    struct serialized_index_storage serialized_index;
    init_serialized_index_storage(&serialized_index);
    serialize_index_entry_storage(index_storage, &serialized_index);
    flush_serialized_index_storage(&serialized_index, index_storage->my_fp->filename, index_storage->my_fp->fs_mode);
    free_serialized_index_storage(&serialized_index);
    // serialize and flush seg meta storage
    struct serialized_seg_meta_section_entry_storage serialized_seg_meta;
    init_serialized_seg_meta_section_entry_storage(&serialized_seg_meta);
    serialize_seg_meta_section_entry_storage(meta_storage, &serialized_seg_meta);
    flush_serialized_seg_meta_storage(&serialized_seg_meta, meta_storage->my_fp->filename, meta_storage->my_fp->fs_mode);
    free_serialized_seg_meta_section_entry_storage(&serialized_seg_meta);

}

void ingest_and_flush_nyc_data_via_zcurve_partition(struct simple_query_engine *engine, FILE *fp, int block_num) {
    // ingest to memory
    ingest_nyc_data_via_zcurve_partition(engine, fp, block_num);

    // flush memory to disk
    struct traj_storage *data_storage = &engine->data_storage;
    struct index_entry_storage *index_storage = &engine->index_storage;
    struct seg_meta_section_entry_storage *meta_storage = &engine->seg_meta_storage;

    // flush data storage
    flush_traj_storage(data_storage);
    // serialized and flush index storage
    struct serialized_index_storage serialized_index;
    init_serialized_index_storage(&serialized_index);
    serialize_index_entry_storage(index_storage, &serialized_index);
    flush_serialized_index_storage(&serialized_index, index_storage->my_fp->filename, index_storage->my_fp->fs_mode);
    free_serialized_index_storage(&serialized_index);
    // serialize and flush seg meta storage
    struct serialized_seg_meta_section_entry_storage serialized_seg_meta;
    init_serialized_seg_meta_section_entry_storage(&serialized_seg_meta);
    serialize_seg_meta_section_entry_storage(meta_storage, &serialized_seg_meta);
    flush_serialized_seg_meta_storage(&serialized_seg_meta, meta_storage->my_fp->filename, meta_storage->my_fp->fs_mode);
    free_serialized_seg_meta_section_entry_storage(&serialized_seg_meta);

}

void ingest_and_flush_data_via_zcurve_partition_with_block_index(struct simple_query_engine *engine, FILE *fp, int block_index, int block_num) {
    // ingest to memory
    ingest_data_via_zcurve_partition_with_block_index(engine, fp, block_index, block_num);

    // flush memory to disk
    struct traj_storage *data_storage = &engine->data_storage;
    struct index_entry_storage *index_storage = &engine->index_storage;
    struct seg_meta_section_entry_storage *meta_storage = &engine->seg_meta_storage;

    // flush data storage
    flush_traj_storage(data_storage);
    // serialized and flush index storage
    struct serialized_index_storage serialized_index;
    init_serialized_index_storage(&serialized_index);
    serialize_index_entry_storage(index_storage, &serialized_index);
    flush_serialized_index_storage(&serialized_index, index_storage->my_fp->filename, index_storage->my_fp->fs_mode);
    free_serialized_index_storage(&serialized_index);
    // serialize and flush seg meta storage
    struct serialized_seg_meta_section_entry_storage serialized_seg_meta;
    init_serialized_seg_meta_section_entry_storage(&serialized_seg_meta);
    serialize_seg_meta_section_entry_storage(meta_storage, &serialized_seg_meta);
    flush_serialized_seg_meta_storage(&serialized_seg_meta, meta_storage->my_fp->filename, meta_storage->my_fp->fs_mode);
    free_serialized_seg_meta_section_entry_storage(&serialized_seg_meta);

}

void ingest_and_flush_synthetic_data_via_time_partition(struct simple_query_engine *engine, int block_num) {
    // ingest to memory
    ingest_synthetic_data_via_time_partition(engine, block_num);

    // flush memory to disk
    struct traj_storage *data_storage = &engine->data_storage;
    struct index_entry_storage *index_storage = &engine->index_storage;
    struct seg_meta_section_entry_storage *meta_storage = &engine->seg_meta_storage;

    // flush data storage
    flush_traj_storage(data_storage);
    // serialized and flush index storage
    struct serialized_index_storage serialized_index;
    init_serialized_index_storage(&serialized_index);
    serialize_index_entry_storage(index_storage, &serialized_index);
    flush_serialized_index_storage(&serialized_index, index_storage->my_fp->filename, index_storage->my_fp->fs_mode);
    free_serialized_index_storage(&serialized_index);
    // serialize and flush seg meta storage
    struct serialized_seg_meta_section_entry_storage serialized_seg_meta;
    init_serialized_seg_meta_section_entry_storage(&serialized_seg_meta);
    serialize_seg_meta_section_entry_storage(meta_storage, &serialized_seg_meta);
    flush_serialized_seg_meta_storage(&serialized_seg_meta, meta_storage->my_fp->filename, meta_storage->my_fp->fs_mode);
    free_serialized_seg_meta_section_entry_storage(&serialized_seg_meta);
}

void ingest_and_flush_synthetic_data_via_time_partition_with_block_index(struct simple_query_engine *engine, int block_index, int block_num) {
    // ingest to memory
    ingest_synthetic_data_via_time_partition_with_block_index(engine, block_index, block_num);

    // flush memory to disk
    struct traj_storage *data_storage = &engine->data_storage;
    struct index_entry_storage *index_storage = &engine->index_storage;
    struct seg_meta_section_entry_storage *meta_storage = &engine->seg_meta_storage;

    // flush data storage
    flush_traj_storage(data_storage);
    // serialized and flush index storage
    struct serialized_index_storage serialized_index;
    init_serialized_index_storage(&serialized_index);
    serialize_index_entry_storage(index_storage, &serialized_index);
    flush_serialized_index_storage(&serialized_index, index_storage->my_fp->filename, index_storage->my_fp->fs_mode);
    free_serialized_index_storage(&serialized_index);
    // serialize and flush seg meta storage
    struct serialized_seg_meta_section_entry_storage serialized_seg_meta;
    init_serialized_seg_meta_section_entry_storage(&serialized_seg_meta);
    serialize_seg_meta_section_entry_storage(meta_storage, &serialized_seg_meta);
    flush_serialized_seg_meta_storage(&serialized_seg_meta, meta_storage->my_fp->filename, meta_storage->my_fp->fs_mode);
    free_serialized_seg_meta_section_entry_storage(&serialized_seg_meta);
}

void ingest_and_flush_osm_data_via_zcurve_partition_with_block_index(struct simple_query_engine *engine, FILE *fp, int block_index, int block_num) {
    // ingest to memory
    ingest_osm_data_via_zcurve_partition_with_block_index(engine, fp, block_index, block_num);

    // flush memory to disk
    struct traj_storage *data_storage = &engine->data_storage;
    struct index_entry_storage *index_storage = &engine->index_storage;
    struct seg_meta_section_entry_storage *meta_storage = &engine->seg_meta_storage;

    // flush data storage
    flush_traj_storage(data_storage);

    // serialized and flush index storage
    struct serialized_index_storage serialized_index;
    init_serialized_index_storage(&serialized_index);
    serialize_index_entry_storage(index_storage, &serialized_index);
    flush_serialized_index_storage(&serialized_index, index_storage->my_fp->filename, index_storage->my_fp->fs_mode);
    free_serialized_index_storage(&serialized_index);
    // serialize and flush seg meta storage
    struct serialized_seg_meta_section_entry_storage serialized_seg_meta;
    init_serialized_seg_meta_section_entry_storage(&serialized_seg_meta);
    serialize_seg_meta_section_entry_storage(meta_storage, &serialized_seg_meta);
    flush_serialized_seg_meta_storage(&serialized_seg_meta, meta_storage->my_fp->filename, meta_storage->my_fp->fs_mode);
    free_serialized_seg_meta_section_entry_storage(&serialized_seg_meta);
}

void ingest_and_flush_osm_data_via_time_partition_with_block_index(struct simple_query_engine *engine, FILE *fp, int block_index, int block_num) {
    // ingest to memory
    ingest_osm_data_via_time_partition_with_block_index(engine, fp, block_index, block_num);

    // flush memory to disk
    struct traj_storage *data_storage = &engine->data_storage;
    struct index_entry_storage *index_storage = &engine->index_storage;
    struct seg_meta_section_entry_storage *meta_storage = &engine->seg_meta_storage;

    // flush data storage
    flush_traj_storage(data_storage);
    // serialized and flush index storage
    struct serialized_index_storage serialized_index;
    init_serialized_index_storage(&serialized_index);
    serialize_index_entry_storage(index_storage, &serialized_index);
    flush_serialized_index_storage(&serialized_index, index_storage->my_fp->filename, index_storage->my_fp->fs_mode);
    free_serialized_index_storage(&serialized_index);
    // serialize and flush seg meta storage
    struct serialized_seg_meta_section_entry_storage serialized_seg_meta;
    init_serialized_seg_meta_section_entry_storage(&serialized_seg_meta);
    serialize_seg_meta_section_entry_storage(meta_storage, &serialized_seg_meta);
    flush_serialized_seg_meta_storage(&serialized_seg_meta, meta_storage->my_fp->filename, meta_storage->my_fp->fs_mode);
    free_serialized_seg_meta_section_entry_storage(&serialized_seg_meta);
}

void rebuild_query_engine_from_file(struct simple_query_engine *engine) {
    struct traj_storage *data_storage = &engine->data_storage;
    struct index_entry_storage *index_storage = &engine->index_storage;
    struct seg_meta_section_entry_storage *meta_storage = &engine->seg_meta_storage;

    rebuild_index_storage(index_storage->my_fp->filename, index_storage->my_fp->fs_mode, index_storage);
    rebuild_seg_meta_storage(meta_storage->my_fp->filename, meta_storage->my_fp->fs_mode, meta_storage);

}

/**
 * not used
 * @param engine
 * @param predicate
 * @return
 */
int id_temporal_query(struct simple_query_engine *engine, struct id_temporal_predicate *predicate) {
    int result_count = 0;
    // check the index
    struct index_entry_storage *index_storage = &engine->index_storage;
    struct traj_storage *data_storage = &engine->data_storage;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        if (check_oid_exist(entry->oid_array, entry->oid_array_size, predicate->oid) > 0
        && predicate->time_min <= entry->time_max
        && predicate->time_max >= entry->time_min) {
            void *data_block = NULL;
            if (entry->block_physical_ptr != NULL) {
                data_block = entry->block_physical_ptr;
            } else {
                data_block = malloc(TRAJ_BLOCK_SIZE);
                fetch_traj_data_via_logical_pointer(data_storage, entry->block_logical_adr, data_block);
            }

            // parse data block
            struct traj_block_header block_header;
            parse_traj_block_for_header(data_block, &block_header);
            struct seg_meta meta_array[block_header.seg_count];
            parse_traj_block_for_seg_meta_section(data_block, meta_array, block_header.seg_count);
            for (int j = 0; j < block_header.seg_count; j++) {
                struct seg_meta meta_item = meta_array[j];
                if (predicate->time_min <= meta_item.time_max && predicate->time_max >= meta_item.time_min) {
                    int data_seg_points_num = meta_item.seg_size / get_traj_point_size();
                    struct traj_point **points = allocate_points_memory(data_seg_points_num);
                    parse_traj_block_for_seg_data(data_block, meta_item.seg_offset, points, data_seg_points_num);
                    for (int k = 0; k < data_seg_points_num; k++) {
                        struct traj_point *point = points[k];
                        if (point->oid == predicate->oid
                        && predicate->time_min <= point->timestamp_sec
                        && predicate->time_max >= point->timestamp_sec) {
                            result_count++;
                        }
                    }
                    free_points_memory(points, data_seg_points_num);
                }
            }

        }
    }

    return result_count;

}



/**
 * not used
 * @param engine
 * @param predicate
 * @return
 */
int spatio_temporal_query(struct simple_query_engine *engine, struct spatio_temporal_range_predicate *predicate) {
    int result_count = 0;
    // check the index
    struct index_entry_storage *index_storage = &engine->index_storage;
    struct traj_storage *data_storage = &engine->data_storage;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        if (predicate->lon_min <= entry->lon_max
            && predicate->lon_max >= entry->lon_min
            && predicate->lat_min <= entry->lat_max
            && predicate->lat_max >= entry->lat_min
            && predicate->time_min <= entry->time_max
            && predicate->time_max >= entry->time_min) {
            void *data_block = NULL;
            if (entry->block_physical_ptr != NULL) {
                data_block = entry->block_physical_ptr;
            } else {
                data_block = malloc(TRAJ_BLOCK_SIZE);
                fetch_traj_data_via_logical_pointer(data_storage, entry->block_logical_adr, data_block);
                printf("block pointer: %d\n", entry->block_logical_adr);
            }

            // parse data block
            struct traj_block_header block_header;
            parse_traj_block_for_header(data_block, &block_header);
            struct seg_meta meta_array[block_header.seg_count];
            parse_traj_block_for_seg_meta_section(data_block, meta_array, block_header.seg_count);
            for (int j = 0; j < block_header.seg_count; j++) {
                struct seg_meta meta_item = meta_array[j];
                if (predicate->time_min <= meta_item.time_max && predicate->time_max >= meta_item.time_min
                && predicate->lon_min <= meta_item.lon_max && predicate->lon_max >= meta_item.lon_min
                && predicate->lat_min <= meta_item.lat_max && predicate->lat_max >= meta_item.lat_min) {
                    int data_seg_points_num = meta_item.seg_size / get_traj_point_size();
                    struct traj_point **points = allocate_points_memory(data_seg_points_num);
                    parse_traj_block_for_seg_data(data_block, meta_item.seg_offset, points, data_seg_points_num);
                    for (int k = 0; k < data_seg_points_num; k++) {
                        struct traj_point *point = points[k];
                        if (predicate->lon_min <= point->normalized_longitude
                            && predicate->lon_max >= point->normalized_longitude
                            && predicate->lat_min <= point->normalized_latitude
                            && predicate->lat_max >= point->normalized_latitude
                            && predicate->time_min <= point->timestamp_sec
                            && predicate->time_max >= point->timestamp_sec) {
                            result_count++;
                        }
                    }
                    free_points_memory(points, data_seg_points_num);
                }
            }

        }
    }
    return result_count;
}

static void assemble_isp_desc_for_id_temporal_naive(struct isp_descriptor *isp_desc, struct id_temporal_predicate *predicate, int estimated_result_block_num, struct lba *lba_vec, int lba_vec_size) {
    isp_desc->isp_type = 3;
    isp_desc->estimated_result_page_num = estimated_result_block_num;
    isp_desc->oid = predicate->oid;
    isp_desc->lba_array = lba_vec;
    isp_desc->lba_count = lba_vec_size;
    isp_desc->time_min = predicate->time_min;
    isp_desc->time_max = predicate->time_max;
    isp_desc->lon_min = 0;
    isp_desc->lon_max = 0;
    isp_desc->lat_min = 0;
    isp_desc->lat_max = 0;
}

static void assemble_isp_desc_for_id_temporal(struct isp_descriptor *isp_desc, struct id_temporal_predicate *predicate, int estimated_result_block_num, struct lba *lba_vec, int lba_vec_size) {
    isp_desc->isp_type = 0;
    isp_desc->estimated_result_page_num = estimated_result_block_num;
    isp_desc->oid = predicate->oid;
    isp_desc->lba_array = lba_vec;
    isp_desc->lba_count = lba_vec_size;
    isp_desc->time_min = predicate->time_min;
    isp_desc->time_max = predicate->time_max;
    isp_desc->lon_min = 0;
    isp_desc->lon_max = 0;
    isp_desc->lat_min = 0;
    isp_desc->lat_max = 0;
}

static void assemble_isp_desc_for_spatial_temporal(struct isp_descriptor *isp_desc, struct spatio_temporal_range_predicate *predicate, int estimated_result_block_num, struct lba *lba_vec, int lba_vec_size) {
    isp_desc->isp_type = 1;
    isp_desc->estimated_result_page_num = estimated_result_block_num;
    isp_desc->oid = 0;
    isp_desc->lba_array = lba_vec;
    isp_desc->lba_count = lba_vec_size;
    isp_desc->time_min = predicate->time_min;
    isp_desc->time_max = predicate->time_max;
    isp_desc->lon_min = predicate->lon_min;
    isp_desc->lon_max = predicate->lon_max;
    isp_desc->lat_min = predicate->lat_min;
    isp_desc->lat_max = predicate->lat_max;
}

static void assemble_isp_desc_for_spatial_temporal_naive(struct isp_descriptor *isp_desc, struct spatio_temporal_range_predicate *predicate, int estimated_result_block_num, struct lba *lba_vec, int lba_vec_size) {
    isp_desc->isp_type = 4;
    isp_desc->estimated_result_page_num = estimated_result_block_num;
    isp_desc->oid = 0;
    isp_desc->lba_array = lba_vec;
    isp_desc->lba_count = lba_vec_size;
    isp_desc->time_min = predicate->time_min;
    isp_desc->time_max = predicate->time_max;
    isp_desc->lon_min = predicate->lon_min;
    isp_desc->lon_max = predicate->lon_max;
    isp_desc->lat_min = predicate->lat_min;
    isp_desc->lat_max = predicate->lat_max;
}

static void assemble_isp_desc_for_spatial_temporal_count_naive(struct isp_descriptor *isp_desc, struct spatio_temporal_range_predicate *predicate, int estimated_result_block_num, struct lba *lba_vec, int lba_vec_size) {
    isp_desc->isp_type = 5;
    isp_desc->estimated_result_page_num = estimated_result_block_num;
    isp_desc->oid = 0;
    isp_desc->lba_array = lba_vec;
    isp_desc->lba_count = lba_vec_size;
    isp_desc->time_min = predicate->time_min;
    isp_desc->time_max = predicate->time_max;
    isp_desc->lon_min = predicate->lon_min;
    isp_desc->lon_max = predicate->lon_max;
    isp_desc->lat_min = predicate->lat_min;
    isp_desc->lat_max = predicate->lat_max;
}

static void assemble_isp_desc_for_spatial_temporal_count(struct isp_descriptor *isp_desc, struct spatio_temporal_range_predicate *predicate, int estimated_result_block_num, struct lba *lba_vec, int lba_vec_size) {
    isp_desc->isp_type = 2;
    isp_desc->estimated_result_page_num = estimated_result_block_num;
    isp_desc->oid = 0;
    isp_desc->lba_array = lba_vec;
    isp_desc->lba_count = lba_vec_size;
    isp_desc->time_min = predicate->time_min;
    isp_desc->time_max = predicate->time_max;
    isp_desc->lon_min = predicate->lon_min;
    isp_desc->lon_max = predicate->lon_max;
    isp_desc->lat_min = predicate->lat_min;
    isp_desc->lat_max = predicate->lat_max;
}

static void assemble_isp_desc_for_spatial_temporal_knn(struct isp_descriptor *isp_desc, struct spatio_temporal_knn_predicate *predicate, int estimated_result_block_num, struct lba *lba_vec, int lba_vec_size) {
    isp_desc->isp_type = 6;
    isp_desc->estimated_result_page_num = estimated_result_block_num;
    isp_desc->oid = predicate->k;
    isp_desc->lba_array = lba_vec;
    isp_desc->lba_count = lba_vec_size;
    isp_desc->time_min = predicate->query_point.timestamp_sec;
    isp_desc->time_max = 0;
    isp_desc->lon_min = predicate->query_point.normalized_longitude;
    isp_desc->lon_max = 0;
    isp_desc->lat_min = predicate->query_point.normalized_latitude;
    isp_desc->lat_max = 0;
}

static void assemble_isp_desc_for_spatial_temporal_knn_naive(struct isp_descriptor *isp_desc, struct spatio_temporal_knn_predicate *predicate, int estimated_result_block_num, struct lba *lba_vec, int lba_vec_size) {
    isp_desc->isp_type = 61;
    isp_desc->estimated_result_page_num = estimated_result_block_num;
    isp_desc->oid = predicate->k;
    isp_desc->lba_array = lba_vec;
    isp_desc->lba_count = lba_vec_size;
    isp_desc->time_min = predicate->query_point.timestamp_sec;
    isp_desc->time_max = 0;
    isp_desc->lon_min = predicate->query_point.normalized_longitude;
    isp_desc->lon_max = 0;
    isp_desc->lat_min = predicate->query_point.normalized_latitude;
    isp_desc->lat_max = 0;
}

static void assemble_isp_desc_for_spatial_temporal_knn_naive_add_mbr_pruning(struct isp_descriptor *isp_desc, struct spatio_temporal_knn_predicate *predicate, int estimated_result_block_num, struct lba *lba_vec, int lba_vec_size) {
    isp_desc->isp_type = 62;
    isp_desc->estimated_result_page_num = estimated_result_block_num;
    isp_desc->oid = predicate->k;
    isp_desc->lba_array = lba_vec;
    isp_desc->lba_count = lba_vec_size;
    isp_desc->time_min = predicate->query_point.timestamp_sec;
    isp_desc->time_max = 0;
    isp_desc->lon_min = predicate->query_point.normalized_longitude;
    isp_desc->lon_max = 0;
    isp_desc->lat_min = predicate->query_point.normalized_latitude;
    isp_desc->lat_max = 0;
}

static void assemble_isp_desc_for_spatial_temporal_knn_join(struct isp_descriptor *isp_desc, struct spatio_temporal_knn_join_predicate *predicate, int estimated_result_block_num, struct lba *lba_vec, int lba_vec_size) {
    isp_desc->isp_type = 7;
    isp_desc->estimated_result_page_num = estimated_result_block_num;
    isp_desc->oid = predicate->k;
    isp_desc->lba_array = lba_vec;
    isp_desc->lba_count = lba_vec_size;
    isp_desc->time_min = 0;
    isp_desc->time_max = 0;
    isp_desc->lon_min = 0;
    isp_desc->lon_max = 0;
    isp_desc->lat_min = 0;
    isp_desc->lat_max = 0;
}

static void assemble_isp_desc_for_spatial_temporal_knn_join_naive(struct isp_descriptor *isp_desc, struct spatio_temporal_knn_join_predicate *predicate, int estimated_result_block_num, struct lba *lba_vec, int lba_vec_size) {
    isp_desc->isp_type = 71;
    isp_desc->estimated_result_page_num = estimated_result_block_num;
    isp_desc->oid = predicate->k;
    isp_desc->lba_array = lba_vec;
    isp_desc->lba_count = lba_vec_size;
    isp_desc->time_min = 0;
    isp_desc->time_max = 0;
    isp_desc->lon_min = 0;
    isp_desc->lon_max = 0;
    isp_desc->lat_min = 0;
    isp_desc->lat_max = 0;
}

static void assemble_isp_desc_for_spatial_temporal_knn_join_naive_add_mbr_pruning(struct isp_descriptor *isp_desc, struct spatio_temporal_knn_join_predicate *predicate, int estimated_result_block_num, struct lba *lba_vec, int lba_vec_size) {
    isp_desc->isp_type = 72;
    isp_desc->estimated_result_page_num = estimated_result_block_num;
    isp_desc->oid = predicate->k;
    isp_desc->lba_array = lba_vec;
    isp_desc->lba_count = lba_vec_size;
    isp_desc->time_min = 0;
    isp_desc->time_max = 0;
    isp_desc->lon_min = 0;
    isp_desc->lon_max = 0;
    isp_desc->lat_min = 0;
    isp_desc->lat_max = 0;
}

static void assemble_isp_desc_without_comp(struct isp_descriptor *isp_desc, int estimated_result_block_num, struct lba *lba_vec, int lba_vec_size) {
    isp_desc->isp_type = 9;
    isp_desc->estimated_result_page_num = estimated_result_block_num;
    isp_desc->oid = 0;
    isp_desc->lba_array = lba_vec;
    isp_desc->lba_count = lba_vec_size;
    isp_desc->time_min = 0;
    isp_desc->time_max = 0;
    isp_desc->lon_min = 0;
    isp_desc->lon_max = 0;
    isp_desc->lat_min = 0;
    isp_desc->lat_max = 0;
}


int estimate_id_temporal_result_size(struct seg_meta_section_entry_storage *storage, struct id_temporal_predicate *predicate) {
    // TODO fix bug: over-estimated size since there is no oid info
    int result_size = 0;
    for (int i = 0; i <= storage->current_index; i++) {
        struct seg_meta_section_entry *entry = storage->base[i];
        struct seg_meta meta_array[entry->seg_meta_count];
        parse_seg_meta_section(entry->seg_meta_section, meta_array, entry->seg_meta_count);
        for (int j = 0; j < entry->seg_meta_count; j++) {
            struct seg_meta meta_item = meta_array[j];
            if (predicate->time_min <= meta_item.time_max && predicate->time_max >= meta_item.time_min) {
                result_size += meta_item.seg_size;
            }
        }
    }
    return result_size;

}


int estimate_spatio_temporal_result_size(struct seg_meta_section_entry_storage *storage, struct spatio_temporal_range_predicate *predicate) {
    int result_size = 0;
    for (int i = 0; i <= storage->current_index; i++) {
        struct seg_meta_section_entry *entry = storage->base[i];
        struct seg_meta meta_array[entry->seg_meta_count];
        parse_seg_meta_section(entry->seg_meta_section, meta_array, entry->seg_meta_count);
        for (int j = 0; j < entry->seg_meta_count; j++) {
            struct seg_meta meta_item = meta_array[j];

            if (predicate->time_min <= meta_item.time_max && predicate->time_max >= meta_item.time_min
                && predicate->lon_min <= meta_item.lon_max && predicate->lon_max >= meta_item.lon_min
                && predicate->lat_min <= meta_item.lat_max && predicate->lat_max >= meta_item.lat_min) {
                result_size += meta_item.seg_size;
            }
        }
    }
    return result_size;
}

int estimate_id_temporal_result_size_for_blocks(struct seg_meta_section_entry_storage *storage, struct id_temporal_predicate *predicate, int *block_addr_vec, int block_addr_vec_size) {
    int result_size = 0;
    /*for (int i = 0; i <= storage->current_index; i++) {

        struct seg_meta_section_entry *entry = storage->base[i];
        for (int j = 0; j < block_addr_vec_size; j++) {
            if (entry->block_logical_adr == block_addr_vec[j]) {
                struct seg_meta meta_array[entry->seg_meta_count];
                parse_seg_meta_section(entry->seg_meta_section, meta_array, entry->seg_meta_count);
                for (int k = 0; k < entry->seg_meta_count; k++) {
                    struct seg_meta meta_item = meta_array[k];

                    if (predicate->time_min <= meta_item.time_max && predicate->time_max >= meta_item.time_min
                        && predicate->lon_min <= meta_item.lon_max && predicate->lon_max >= meta_item.lon_min
                        && predicate->lat_min <= meta_item.lat_max && predicate->lat_max >= meta_item.lat_min) {
                        result_size += meta_item.seg_size;
                    }
                }
            }
        }
    }*/

    for (int i = 0; i < block_addr_vec_size; i++) {
        int block_addr = block_addr_vec[i];
        //printf("block addr: %d\n", block_addr);
        struct seg_meta_section_entry * entry = storage->base[block_addr];
        struct seg_meta meta_array[entry->seg_meta_count];
        parse_seg_meta_section(entry->seg_meta_section, meta_array, entry->seg_meta_count);
        for (int k = 0; k < entry->seg_meta_count; k++) {
            struct seg_meta meta_item = meta_array[k];

            bloom_filter rebuild_filter;
            bit_vect rebuild_bit_vect;
            void* bit_mem = meta_item.oid_filter;
            bloom_filter_rebuild_default(&rebuild_filter, &rebuild_bit_vect, bit_mem, MY_OID_FILTER_SIZE * 8);
            bool oid_contained = bloom_filter_test(&rebuild_filter, &predicate->oid, 4);

            if (oid_contained && predicate->time_min <= meta_item.time_max && predicate->time_max >= meta_item.time_min) {
                result_size += meta_item.seg_size;
            }
        }
    }

    return result_size + 16 * (result_size / 0x1000);    // extra space for metadata in isp output block
}

int estimate_spatio_temporal_result_size_for_blocks(struct seg_meta_section_entry_storage *storage, struct spatio_temporal_range_predicate *predicate, int *block_addr_vec, int block_addr_vec_size) {
    int result_size = 0;

    for (int i = 0; i < block_addr_vec_size; i++) {
        int block_addr = block_addr_vec[i];
        struct seg_meta_section_entry * entry = storage->base[block_addr];
        struct seg_meta meta_array[entry->seg_meta_count];
        parse_seg_meta_section(entry->seg_meta_section, meta_array, entry->seg_meta_count);
        for (int k = 0; k < entry->seg_meta_count; k++) {
            struct seg_meta meta_item = meta_array[k];

            if (predicate->time_min <= meta_item.time_max && predicate->time_max >= meta_item.time_min
                && predicate->lon_min <= meta_item.lon_max && predicate->lon_max >= meta_item.lon_min
                && predicate->lat_min <= meta_item.lat_max && predicate->lat_max >= meta_item.lat_min) {
                result_size += meta_item.seg_size;
            }
        }
    }

    return result_size + 16 * (result_size / 0x1000);    // extra space for metadata in isp output block
}

int assemble_lba_vec(struct lba *lba_vec, int base_lba, int *block_logical_adr_vec, int block_logical_adr_vec_size) {
    for (int i = 0; i < block_logical_adr_vec_size; i++) {
        lba_vec[i].start_lba = base_lba + block_logical_adr_vec[i];
        lba_vec[i].sector_count = 1;
    }
    return block_logical_adr_vec_size;
}

int assemble_lba_vec_via_block_meta(struct lba *lba_vec, int base_lba, struct continuous_block_meta *block_meta_vec, int block_logical_adr_vec_size) {
    int factor = TRAJ_BLOCK_SIZE / SECTOR_SIZE;
    for (int i = 0; i < block_logical_adr_vec_size; i++) {
        lba_vec[i].start_lba = base_lba + block_meta_vec[i].block_start * factor;
        lba_vec[i].sector_count = block_meta_vec[i].block_count * factor;
    }
    return block_logical_adr_vec_size;
}

/**
 * not used
 * @param engine
 * @param predicate
 * @return
 */
int id_temporal_query_isp(struct simple_query_engine *engine, struct id_temporal_predicate *predicate) {
    int result_count = 0;
    // check the index
    struct index_entry_storage *index_storage = &engine->index_storage;
    struct traj_storage *data_storage = &engine->data_storage;
    struct seg_meta_section_entry_storage *meta_storage = &engine->seg_meta_storage;

    int block_logical_adr_vec[256];
    int block_logical_adr_count = 0;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        if (check_oid_exist(entry->oid_array, entry->oid_array_size, predicate->oid) > 0
            && predicate->time_min <= entry->time_max
            && predicate->time_max >= entry->time_min) {

            if (block_logical_adr_count > 255) {
                printf("Too many blocks in one isp. Not supported now\n");
                return -1;
            }

            block_logical_adr_vec[block_logical_adr_count] = entry->block_logical_adr;
            block_logical_adr_count++;

        }
    }

    //int estimated_result_size = estimate_id_temporal_result_size(meta_storage, predicate);
    int estimated_result_size = block_logical_adr_count * 0x1000;
    if (estimated_result_size == 0) {
        return 0;
    }
    if (estimated_result_size % 0x1000 != 0) {
        estimated_result_size = ((estimated_result_size / 0x1000) + 1) * 0x1000;
    }
    int estimated_result_block_num = estimated_result_size / 0x1000;
    void* result_buffer = malloc(estimated_result_size);

    struct isp_descriptor isp_desc;
    struct lba lba_vec[256];
    int lba_vec_size = assemble_lba_vec(lba_vec, DATA_FILE_OFFSET, block_logical_adr_vec, block_logical_adr_count);
    assemble_isp_desc_for_id_temporal(&isp_desc, predicate, estimated_result_block_num, lba_vec, lba_vec_size);
    do_isp_for_trajectory_data(data_storage, result_buffer, estimated_result_size, &isp_desc, 0);

    for (int block_count = 0; block_count < estimated_result_block_num; block_count++, result_buffer+=4096) {
        int points_num = parse_points_num_from_output_buffer_page(result_buffer);
        //printf("block points num: %d\n", points_num);
        if (points_num > 0) {
            struct traj_point **points = allocate_points_memory(points_num);
            deserialize_output_buffer_page(result_buffer, points, points_num);
            for (int k = 0; k < points_num; k++) {
                struct traj_point *point = points[k];
                //printf("oid: %d, time: %d, lon: %d, lat: %d\n", point->oid, point->timestamp_sec, point->normalized_longitude, point->normalized_latitude);
                if (point->oid == predicate->oid
                    && predicate->time_min <= point->timestamp_sec
                    && predicate->time_max >= point->timestamp_sec) {
                    result_count++;
                }
            }
            free_points_memory(points, points_num);
        }
    }

    return result_count;
}

/**
 * not used
 * @param engine
 * @param predicate
 * @return
 */
int spatio_temporal_query_isp(struct simple_query_engine *engine, struct spatio_temporal_range_predicate *predicate) {
    int result_count = 0;
    // check the index
    struct index_entry_storage *index_storage = &engine->index_storage;
    struct traj_storage *data_storage = &engine->data_storage;
    struct seg_meta_section_entry_storage *meta_storage = &engine->seg_meta_storage;

    int block_logical_adr_vec[256];
    int block_logical_adr_count = 0;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        if (predicate->lon_min <= entry->lon_max
            && predicate->lon_max >= entry->lon_min
            && predicate->lat_min <= entry->lat_max
            && predicate->lat_max >= entry->lat_min
            && predicate->time_min <= entry->time_max
            && predicate->time_max >= entry->time_min) {

            if (block_logical_adr_count > 255) {
                printf("Too many blocks in one isp. Not supported now\n");
                return -1;
            }

            block_logical_adr_vec[block_logical_adr_count] = entry->block_logical_adr;
            block_logical_adr_count++;

        }
    }

    //int estimated_result_size = estimate_id_temporal_result_size(meta_storage, predicate);
    int estimated_result_size = block_logical_adr_count * 0x1000;
    if (estimated_result_size == 0) {
        return 0;
    }
    if (estimated_result_size % 0x1000 != 0) {
        estimated_result_size = ((estimated_result_size / 0x1000) + 1) * 0x1000;
    }
    int estimated_result_block_num = estimated_result_size / 0x1000;
    void* result_buffer = malloc(estimated_result_size);

    struct isp_descriptor isp_desc;
    struct lba lba_vec[256];
    int lba_vec_size = assemble_lba_vec(lba_vec, DATA_FILE_OFFSET, block_logical_adr_vec, block_logical_adr_count);
    assemble_isp_desc_for_spatial_temporal(&isp_desc, predicate, estimated_result_block_num, lba_vec, lba_vec_size);
    do_isp_for_trajectory_data(data_storage, result_buffer, estimated_result_size, &isp_desc, 0);

    for (int block_count = 0; block_count < estimated_result_block_num; block_count++, result_buffer+=4096) {
        int points_num = parse_points_num_from_output_buffer_page(result_buffer);
        //printf("block points num: %d\n", points_num);
        if (points_num > 0) {
            struct traj_point **points = allocate_points_memory(points_num);
            deserialize_output_buffer_page(result_buffer, points, points_num);
            for (int k = 0; k < points_num; k++) {
                struct traj_point *point = points[k];
                //printf("oid: %d, time: %d, lon: %d, lat: %d\n", point->oid, point->timestamp_sec, point->normalized_longitude, point->normalized_latitude);
                if (predicate->lon_min <= point->normalized_longitude
                    && predicate->lon_max >= point->normalized_longitude
                    && predicate->lat_min <= point->normalized_latitude
                    && predicate->lat_max >= point->normalized_latitude
                    && predicate->time_min <= point->timestamp_sec
                    && predicate->time_max >= point->timestamp_sec) {
                    result_count++;
                }
            }
            free_points_memory(points, points_num);
        }
    }

    return result_count;
}

int calculate_aggregate_block_vec_size(int *block_addr_vec, int block_addr_vec_size) {
    if (block_addr_vec_size == 1) {
        return 1;
    }

    int aggregate_block_size = 0;
    for (int i = 0; i < block_addr_vec_size - 1; i++) {
        if (block_addr_vec[i] + 1 != block_addr_vec[i + 1]) {
            aggregate_block_size++;
        }
    }
    aggregate_block_size++;
    return aggregate_block_size;

}

void aggregate_blocks(int *block_addr_vec, int block_addr_vec_size, struct continuous_block_meta *block_meta_vec, int block_meta_vec_size) {

    if (calculate_aggregate_block_vec_size(block_addr_vec, block_addr_vec_size) > block_meta_vec_size) {
        fprintf(stderr, "[simple query engine] not matched aggregate block num\n");
        return;
    }

    if (block_addr_vec_size == 1) {
        block_meta_vec[0].block_start = block_addr_vec[0];
        block_meta_vec[0].block_count = 1;
        return;
    }

    int base = 0;
    int cursor = 0;
    int current_block_meta_vec_index = 0;
    for (int i = 0; i < block_addr_vec_size - 1; i++) {
        if (block_addr_vec[i] + 1 == block_addr_vec[i + 1]) {
            // move cursor
            cursor++;
        } else {
//            printf("base: %d; base element: %d\n", base, block_addr_vec[base]);
//            printf("cursor: %d\n", cursor);
//            printf("count: %d\n\n", cursor - base + 1);
            block_meta_vec[current_block_meta_vec_index].block_start = block_addr_vec[base];
            block_meta_vec[current_block_meta_vec_index].block_count = cursor - base + 1;
            current_block_meta_vec_index++;
            // start a new continuous block region
            base = i+1;
            cursor = i+1;
        }
    }

    // handle the last one
//    printf("base: %d; base element: %d\n", base, block_addr_vec[base]);
//    printf("cursor: %d\n", cursor);
//    printf("count: %d\n\n", cursor - base + 1);
    block_meta_vec[current_block_meta_vec_index].block_start = block_addr_vec[base];
    block_meta_vec[current_block_meta_vec_index].block_count = cursor - base + 1;
}

void print_aggregated_blocks_meta(struct continuous_block_meta *block_meta_vec, int block_meta_vec_size) {
    for (int i = 0; i < block_meta_vec_size; i++) {
        printf("block start: %d\n", block_meta_vec[i].block_start);
        printf("block count: %d\n\n", block_meta_vec[i].block_count);
    }
}

static struct traj_point **points_buffer;
static int points_buffer_size = 400;
static void fill_points_buffer(int points_num) {
    points_buffer = allocate_points_memory(points_num);
}

static void free_points_buffer(int points_num) {
    free_points_memory(points_buffer, points_num);
}



static int spatio_temporal_query_raw_trajectory_block(void* data_block, struct spatio_temporal_range_predicate *predicate) {
    int result_count = 0;
    struct traj_block_header block_header;
    parse_traj_block_for_header(data_block, &block_header);
    struct traj_point *result_buffer = (struct traj_point *)malloc(TRAJ_BLOCK_SIZE);

    int traj_point_size = get_traj_point_size();
    /*if (block_header.seg_count == -1) {
        //printf("bad block\n");
        return 0; // bad block
    }*/
    struct seg_meta meta_array[block_header.seg_count];
    parse_traj_block_for_seg_meta_section(data_block, meta_array, block_header.seg_count);
    for (int j = 0; j < block_header.seg_count; j++) {
        struct seg_meta meta_item = meta_array[j];
        if (predicate->time_min <= meta_item.time_max && predicate->time_max >= meta_item.time_min
            && predicate->lon_min <= meta_item.lon_max && predicate->lon_max >= meta_item.lon_min
            && predicate->lat_min <= meta_item.lat_max && predicate->lat_max >= meta_item.lat_min) {
            int data_seg_points_num = meta_item.seg_size / traj_point_size;


            struct traj_point *point_ptr = (struct traj_point *)((char *) data_block + meta_item.seg_offset);

            for (int k = 0; k < data_seg_points_num; k++) {

                struct traj_point *point = &point_ptr[k];
                if (predicate->lon_min <= point->normalized_longitude
                    && predicate->lon_max >= point->normalized_longitude
                    && predicate->lat_min <= point->normalized_latitude
                    && predicate->lat_max >= point->normalized_latitude
                    && predicate->time_min <= point->timestamp_sec
                    && predicate->time_max >= point->timestamp_sec) {
                    result_buffer[result_count] = *point;
                    result_count++;

                }
            }

        }
    }

    free(result_buffer);
    return result_count;
}

static int id_temporal_query_raw_trajectory_block(void* data_block, struct id_temporal_predicate *predicate) {
    int result_count = 0;
    struct traj_block_header block_header;
    struct traj_point *result_buffer = (struct traj_point *)malloc(TRAJ_BLOCK_SIZE);
    int traj_point_size = get_traj_point_size();

    parse_traj_block_for_header(data_block, &block_header);
    struct seg_meta meta_array[block_header.seg_count];
    parse_traj_block_for_seg_meta_section(data_block, meta_array, block_header.seg_count);
    for (int j = 0; j < block_header.seg_count; j++) {
        struct seg_meta meta_item = meta_array[j];
        bloom_filter  rebuild_filter;
        bit_vect rebuilt_bit_vec;
        void* bit_mem = meta_item.oid_filter;
        bloom_filter_rebuild_default(&rebuild_filter, &rebuilt_bit_vec, bit_mem, MY_OID_FILTER_SIZE * 8);
        bool oid_contained = bloom_filter_test(&rebuild_filter, &predicate->oid, 4);

        if (oid_contained && predicate->time_min <= meta_item.time_max && predicate->time_max >= meta_item.time_min) {
            int data_seg_points_num = meta_item.seg_size / traj_point_size;


            struct traj_point *point_ptr = (struct traj_point *)((char *) data_block + meta_item.seg_offset);

            for (int k = 0; k < data_seg_points_num; k++) {
                struct traj_point *point = &point_ptr[k];
                if (predicate->oid == point->oid
                    && predicate->time_min <= point->timestamp_sec
                    && predicate->time_max >= point->timestamp_sec) {
                    result_buffer[result_count] = *point;
                    result_count++;

                }
            }

        }
    }
    free(result_buffer);
    return result_count;
}

static int spatio_temporal_query_raw_trajectory_block_without_meta_filtering(void* data_block, struct spatio_temporal_range_predicate *predicate) {
    struct traj_point result;
    int result_count = 0;
    struct traj_block_header block_header;
    parse_traj_block_for_header(data_block, &block_header);

    struct traj_point tmp_point;

    struct seg_meta meta_array[block_header.seg_count];
    parse_traj_block_for_seg_meta_section(data_block, meta_array, block_header.seg_count);
    for (int j = 0; j < block_header.seg_count; j++) {
        struct seg_meta meta_item = meta_array[j];

        int data_seg_points_num = meta_item.seg_size / get_traj_point_size();

        struct traj_point **points = points_buffer;
        if (points_buffer_size < data_seg_points_num) {
            printf("the points buffer is not big enough\n");
        }
        //parse_traj_block_for_seg_data(data_block, meta_item.seg_offset, points, data_seg_points_num);

        struct traj_point *point_ptr = (struct traj_point *)((char *) data_block + meta_item.seg_offset);
        //printf("oid: %d, time: %d, lon: %d, lat: %d\n", points[0]->oid, points[0]->timestamp_sec, points[0]->normalized_longitude, points[0]->normalized_latitude);
        for (int k = 0; k < data_seg_points_num; k++) {
            //struct traj_point *point = points[k];
            struct traj_point *point = &point_ptr[k];
            if (predicate->lon_min <= point->normalized_longitude
                    && predicate->lon_max >= point->normalized_longitude
                    && predicate->lat_min <= point->normalized_latitude
                    && predicate->lat_max >= point->normalized_latitude
                    && predicate->time_min <= point->timestamp_sec
                    && predicate->time_max >= point->timestamp_sec) {

                result_count++;
                tmp_point.oid = point->oid; // simulate the operation to write result to output buffer
                tmp_point.normalized_longitude = point->normalized_longitude;
                tmp_point.normalized_latitude = point->normalized_latitude;
                tmp_point.timestamp_sec = point->timestamp_sec;
            }

        }

    }

    return result_count;
}

static int spatio_temporal_query_raw_trajectory_block_without_meta_filtering_test(void* data_block, struct spatio_temporal_range_predicate *predicate) {
    struct traj_point result;
    int result_count = 0;
    struct traj_block_header block_header;
    parse_traj_block_for_header(data_block, &block_header);

    struct traj_point tmp_point;


    char* meta_ptr = ((char*) data_block + get_header_size());
    for (int j = 0; j < block_header.seg_count; j++) {
        struct seg_meta *meta_item = (struct seg_meta *)(meta_ptr + j * get_seg_meta_size());
        int data_seg_points_num = meta_item->seg_size / get_traj_point_size();

        struct traj_point *point_ptr = (struct traj_point *)((char *) data_block + meta_item->seg_offset);
        for (int k = 0; k < data_seg_points_num; k++) {

            struct traj_point *point = &point_ptr[k];
            if (predicate->lon_min <= point->normalized_longitude
                && predicate->lon_max >= point->normalized_longitude
                && predicate->lat_min <= point->normalized_latitude
                && predicate->lat_max >= point->normalized_latitude
                && predicate->time_min <= point->timestamp_sec
                && predicate->time_max >= point->timestamp_sec) {

                result_count++;
                tmp_point.oid = point->oid; // simulate the operation to write result to output buffer
                tmp_point.normalized_longitude = point->normalized_longitude;
                tmp_point.normalized_latitude = point->normalized_latitude;
                tmp_point.timestamp_sec = point->timestamp_sec;
            }

        }

    }

    return result_count;
}

static void init_result_buffer(struct host_result_buffer *buffer, int buffer_size) {
    buffer->point_count = 0;
    buffer->buffer_size = buffer_size;
    buffer->buffer_ptr = malloc(buffer_size);
}

static void free_result_buffer(struct host_result_buffer *buffer) {
    free(buffer->buffer_ptr);
}

static void put_result_to_buffer(struct host_result_buffer *result_buffer, void* data, int size) {
    if (result_buffer->point_count * size + size > result_buffer->buffer_size) {
        printf("no space left in buffer\n");
        return;
    }
    struct traj_point *dst = (struct traj_point*) result_buffer->buffer_ptr;
    struct traj_point *src = (struct traj_point*) data;
    dst[result_buffer->point_count] = *src;
    result_buffer->point_count++;
}

static int spatio_temporal_query_raw_trajectory_block_with_result_buffer_without_meta_filtering(void* data_block, struct spatio_temporal_range_predicate *predicate, struct host_result_buffer* result_buffer) {
    struct traj_point result;
    int result_count = 0;
    struct traj_block_header block_header;
    parse_traj_block_for_header(data_block, &block_header);


    struct seg_meta meta_array[block_header.seg_count];
    parse_traj_block_for_seg_meta_section(data_block, meta_array, block_header.seg_count);
    for (int j = 0; j < block_header.seg_count; j++) {
        struct seg_meta meta_item = meta_array[j];

        int data_seg_points_num = meta_item.seg_size / get_traj_point_size();

        struct traj_point **points = points_buffer;
        if (points_buffer_size < data_seg_points_num) {
            printf("the points buffer is not big enough\n");
        }
        //parse_traj_block_for_seg_data(data_block, meta_item.seg_offset, points, data_seg_points_num);

        struct traj_point *point_ptr = (struct traj_point *)((char *) data_block + meta_item.seg_offset);
        //printf("oid: %d, time: %d, lon: %d, lat: %d\n", points[0]->oid, points[0]->timestamp_sec, points[0]->normalized_longitude, points[0]->normalized_latitude);
        for (int k = 0; k < data_seg_points_num; k++) {
            //struct traj_point *point = points[k];
            struct traj_point *point = &point_ptr[k];
            if (predicate->lon_min <= point->normalized_longitude
                && predicate->lon_max >= point->normalized_longitude
                && predicate->lat_min <= point->normalized_latitude
                && predicate->lat_max >= point->normalized_latitude
                && predicate->time_min <= point->timestamp_sec
                && predicate->time_max >= point->timestamp_sec) {

                result_count++;
                put_result_to_buffer(result_buffer, point, 16);
            }

        }

    }

    return result_count;
}

static int id_temporal_query_raw_trajectory_block_without_meta_filtering(void* data_block, struct id_temporal_predicate *predicate) {
    int result_count = 0;
    struct traj_block_header block_header;
    parse_traj_block_for_header(data_block, &block_header);

    struct traj_point tmp_point;

    struct seg_meta meta_array[block_header.seg_count];
    parse_traj_block_for_seg_meta_section(data_block, meta_array, block_header.seg_count);
    for (int j = 0; j < block_header.seg_count; j++) {
        struct seg_meta meta_item = meta_array[j];

        int data_seg_points_num = meta_item.seg_size / get_traj_point_size();
        //struct traj_point **points = allocate_points_memory(data_seg_points_num);
        struct traj_point **points = points_buffer;
        if (points_buffer_size < data_seg_points_num) {
            printf("the points buffer is not big enough\n");
        }
        //print_traj_points(points, data_seg_points_num);

        //parse_traj_block_for_seg_data(data_block, meta_item.seg_offset, points, data_seg_points_num);
        struct traj_point *point_ptr = (struct traj_point *)((char *) data_block + meta_item.seg_offset);
        //printf("oid: %d, time: %d, lon: %d, lat: %d\n", points[0]->oid, points[0]->timestamp_sec, points[0]->normalized_longitude, points[0]->normalized_latitude);
        for (int k = 0; k < data_seg_points_num; k++) {
            //struct traj_point *point = points[k];
            struct traj_point *point = &point_ptr[k];
            if (predicate->oid == point->oid
                && predicate->time_min <= point->timestamp_sec
                && predicate->time_max >= point->timestamp_sec) {
                result_count++;
                tmp_point.oid = point->oid; // simulate the operation to write result to output buffer
                tmp_point.normalized_longitude = point->normalized_longitude;
                tmp_point.normalized_latitude = point->normalized_latitude;
                tmp_point.timestamp_sec = point->timestamp_sec;

            }
        }
        //free_points_memory(points, data_seg_points_num);
        //}
    }
    return result_count;
}

// not used
static int spatio_temporal_query_isp_output_block(void* data_block, struct spatio_temporal_range_predicate *predicate) {
    int result_count = 0;
    int points_num = parse_points_num_from_output_buffer_page(data_block);
    //printf("block points num: %d\n", points_num);
    if (points_num > 0) {
        //struct traj_point **points = allocate_points_memory(points_num);
        struct traj_point **points = points_buffer;
        deserialize_output_buffer_page(data_block, points, points_num);
        //struct traj_point *point_ptr = (struct traj_point *) ((char*)data_block + 4);
        for (int k = 0; k < points_num; k++) {
            struct traj_point *point = points[k];
            //printf("oid: %d, time: %d, lon: %d, lat: %d\n", point->oid, point->timestamp_sec, point->normalized_longitude, point->normalized_latitude);
            /*if (predicate->lon_min <= point->normalized_longitude
                && predicate->lon_max >= point->normalized_longitude
                && predicate->lat_min <= point->normalized_latitude
                && predicate->lat_max >= point->normalized_latitude
                && predicate->time_min <= point->timestamp_sec
                && predicate->time_max >= point->timestamp_sec) {
                result_count++;
            }*/
            result_count++;
        }
        //free_points_memory(points, points_num);
    }
    return result_count;
}

// not used
static int id_temporal_query_isp_output_block(void* data_block, struct id_temporal_predicate *predicate) {
    int result_count = 0;
    int points_num = parse_points_num_from_output_buffer_page(data_block);
    //printf("block points num: %d\n", points_num);
    if (points_num > 0) {
        //struct traj_point **points = allocate_points_memory(points_num);
        struct traj_point **points = points_buffer;
        deserialize_output_buffer_page(data_block, points, points_num);
        for (int k = 0; k < points_num; k++) {
            struct traj_point *point = points[k];

            result_count++;
        }
        //free_points_memory(points, points_num);
    }
    return result_count;
}

static int spatio_temporal_query_isp_new_format_output_blocks(void* data_blocks, struct spatio_temporal_range_predicate *predicate) {
    int result_count = 0;
    memcpy(&result_count, data_blocks, 4);

    struct traj_point *points_base = (struct traj_point *)((char *)data_blocks + 4);   // output format is good, so do not need deserialization

    for (int i = 0; i < result_count; i++) {
        struct traj_point tmp = points_base[i];
        //printf("oid: %d, lon: %d, lat: %d, time: %d\n", tmp.oid, tmp.normalized_longitude, tmp.normalized_latitude, tmp.timestamp_sec);
    }
    return result_count;
}

static int spatio_temporal_count_query_block_host_processing(void* data_block, void* result_buf, struct spatio_temporal_range_predicate *predicate) {
    int total_count = 0;
    int header_size = get_header_size();
    struct traj_block_header block_header;
    parse_traj_block_for_header(data_block, &block_header);
    int data_offset = header_size + block_header.seg_count * get_seg_meta_size();

    struct traj_point *point_ptr = (struct traj_point *)((char *) data_block + data_offset);

    int traj_point_size = get_traj_point_size();
    int point_num = (4096 - data_offset) / traj_point_size;

    int *buffer_dst = (int *)result_buf;

    for (int i = 0; i < point_num; i++) {
        int lon_offset = (point_ptr[i].normalized_longitude - predicate->lon_min) / predicate->time_min;    // predicate time_min is cell longitude width (we reuse this field)
        int lat_offset = (point_ptr[i].normalized_latitude - predicate->lat_min) / predicate->time_max;
        int result_index = lon_offset * 30 + lat_offset;
        if (result_index < 0 || result_index > 1023) {
            continue;
        }
        buffer_dst[result_index] = buffer_dst[result_index] + 1;
        total_count +=1;
    }


    return total_count;
}

static int spatio_temporal_count_query_block_isp_processing(void* data_block, void* result_buf, struct spatio_temporal_range_predicate *predicate) {

    int total_count = 0;
    int *count_base = (int *)((char *)data_block);   // output format is good, so do not need deserialization
    int *result_base = (int *)result_base;

    for (int i = 0; i < 1024; i++) {
        //result_base[i] += count_base[i];
        //printf("%d\n", count_base[i]);
        total_count += count_base[i];
    }
    return total_count;
}

static int id_temporal_query_isp_new_format_output_blocks(void* data_blocks, struct id_temporal_predicate *predicate) {
    int result_count = 0;
    memcpy(&result_count, data_blocks, 4);

    struct traj_point *points_base = (struct traj_point *)((char *)data_blocks + 4);
    for (int i = 0; i < result_count; i++) {
        struct traj_point tmp = points_base[i];
        //printf("oid: %d, lon: %d, lat: %d, time: %d\n", tmp.oid, tmp.normalized_longitude, tmp.normalized_latitude, tmp.timestamp_sec);
    }
    return result_count;
}

static int spatio_temporal_query_isp_fpga_output_blocks(void* data_blocks, struct spatio_temporal_range_predicate *predicate) {
    int result_count = 0;
    memcpy(&result_count, data_blocks, 4);

    struct traj_point *points_base = (struct traj_point *)((char *)data_blocks + 16);   // output format is good, so do not need deserialization

    for (int i = 0; i < result_count; i++) {
        struct traj_point tmp = points_base[i];
        //printf("oid: %d, lon: %d, lat: %d, time: %d\n", tmp.oid, tmp.normalized_longitude, tmp.normalized_latitude, tmp.timestamp_sec);
    }
    return result_count;
}

static int id_temporal_query_isp_fpga_output_blocks(void* data_blocks, struct id_temporal_predicate *predicate) {
    int result_count = 0;
    memcpy(&result_count, data_blocks, 4);

    struct traj_point *points_base = (struct traj_point *)((char *)data_blocks + 16);
    for (int i = 0; i < result_count; i++) {
        struct traj_point tmp = points_base[i];
        //printf("oid: %d, lon: %d, lat: %d, time: %d\n", tmp.oid, tmp.normalized_longitude, tmp.normalized_latitude, tmp.timestamp_sec);
    }
    return result_count;
}

static double calculate_goodness_for_spatio_temporal(struct seg_meta_section_entry_storage *storage, int block_logical_pointer, struct spatio_temporal_range_predicate *predicate) {
    int result_size = 0;
    int total_size = 0;

    //for (int i = 0; i <= storage->current_index; i++) {
        struct seg_meta_section_entry *entry = storage->base[block_logical_pointer];
        if (entry->block_logical_adr == block_logical_pointer) {
            //printf("index: %d, real value: %d\n", i, block_logical_pointer);
            struct seg_meta meta_array[entry->seg_meta_count];
            parse_seg_meta_section(entry->seg_meta_section, meta_array, entry->seg_meta_count);
            for (int j = 0; j < entry->seg_meta_count; j++) {
                struct seg_meta meta_item = meta_array[j];
                total_size += meta_item.seg_size;
                if (predicate->time_min <= meta_item.time_max && predicate->time_max >= meta_item.time_min
                    && predicate->lon_min <= meta_item.lon_max && predicate->lon_max >= meta_item.lon_min
                    && predicate->lat_min <= meta_item.lat_max && predicate->lat_max >= meta_item.lat_min) {
                    result_size += meta_item.seg_size;
                }
            }
        } else {
            printf("please use the iteration way\n");
        }
    //}
    return 1.0 * result_size / total_size;
}

static double calculate_goodness_for_id_temporal(struct seg_meta_section_entry_storage *storage, int block_logical_pointer, struct id_temporal_predicate *predicate) {
    int result_size = 0;
    int total_size = 0;

    //for (int i = 0; i <= storage->current_index; i++) {
    struct seg_meta_section_entry *entry = storage->base[block_logical_pointer];
    if (entry->block_logical_adr == block_logical_pointer) {
        //printf("index: %d, real value: %d\n", i, block_logical_pointer);
        struct seg_meta meta_array[entry->seg_meta_count];
        parse_seg_meta_section(entry->seg_meta_section, meta_array, entry->seg_meta_count);
        for (int j = 0; j < entry->seg_meta_count; j++) {
            struct seg_meta meta_item = meta_array[j];
            bloom_filter rebuild_filter;
            bit_vect rebuild_bit_vec;
            void* bit_mem = meta_item.oid_filter;
            bloom_filter_rebuild_default(&rebuild_filter, &rebuild_bit_vec, bit_mem, MY_OID_FILTER_SIZE * 8);
            bool oid_contained = bloom_filter_test(&rebuild_filter, &predicate->oid, 4);

            total_size += meta_item.seg_size;
            if (predicate->time_min <= meta_item.time_max && predicate->time_max >= meta_item.time_min
                && oid_contained) {
                result_size += meta_item.seg_size;
            }
        }
    } else {
        printf("please use the iteration way\n");
    }
    //}
    return 1.0 * result_size / total_size;
}

int spatio_temporal_query_without_pushdown(struct simple_query_engine *engine, struct spatio_temporal_range_predicate *predicate, bool enable_host_index) {

    struct index_entry_storage *index_storage = &engine->index_storage;
    struct traj_storage *data_storage = &engine->data_storage;
    struct seg_meta_section_entry_storage *meta_storage = &engine->seg_meta_storage;

    clock_t  index_start, index_end;
    index_start = clock();
    // calculate the matched block num
    int block_logical_addr_count = 0;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        if (enable_host_index) {
            if (predicate->lon_min <= entry->lon_max
                && predicate->lon_max >= entry->lon_min
                && predicate->lat_min <= entry->lat_max
                && predicate->lat_max >= entry->lat_min
                && predicate->time_min <= entry->time_max
                && predicate->time_max >= entry->time_min) {
                block_logical_addr_count++;
            }
        } else {
            block_logical_addr_count++;
        }
    }

    if (block_logical_addr_count == 0) {
        return 0;
    }

    // get the match block id vector
    // int block_logical_addr_vec[block_logical_addr_count];
    int *block_logical_addr_vec = malloc(block_logical_addr_count * sizeof(int));

    int addr_vec_index = 0;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        if (enable_host_index) {
            if (predicate->lon_min <= entry->lon_max
                && predicate->lon_max >= entry->lon_min
                && predicate->lat_min <= entry->lat_max
                && predicate->lat_max >= entry->lat_min
                && predicate->time_min <= entry->time_max
                && predicate->time_max >= entry->time_min) {
                block_logical_addr_vec[addr_vec_index] = entry->block_logical_adr;
                addr_vec_index++;
            }
        } else {
            block_logical_addr_vec[addr_vec_index] = entry->block_logical_adr;
            addr_vec_index++;
        }
    }
    index_end = clock();
    printf("[host] index lookup time: %f\n", (double)(index_end - index_start));


    printf("[host] read block num: %d\n", block_logical_addr_count);
    // run query
    int result_count = run_spatio_temporal_query_in_host(predicate, data_storage, block_logical_addr_count,
                                                         block_logical_addr_vec);

    free(block_logical_addr_vec);
    return result_count;
}


int
run_spatio_temporal_query_in_host(struct spatio_temporal_range_predicate *predicate, struct traj_storage *data_storage,
                                  int block_logical_addr_count, int *block_logical_addr_vec) {
    clock_t start, start_config, end;
    clock_t start_read, end_read, pure_read;
    clock_t start_computation, end_computation, pure_computation;
    pure_read = 0;
    pure_computation = 0;
    start_config = clock();
    int result_count = 0;
    int aggregated_block_vec_size = calculate_aggregate_block_vec_size(block_logical_addr_vec, block_logical_addr_count);
    struct continuous_block_meta aggregated_block_vec[aggregated_block_vec_size];
    aggregate_blocks(block_logical_addr_vec, block_logical_addr_count, aggregated_block_vec, aggregated_block_vec_size);

    fill_points_buffer(points_buffer_size);


    start = clock();

    // split aggregated block if its size is larger than 256
    for (int i = 0; i < aggregated_block_vec_size; i++) {
        struct continuous_block_meta block_meta = aggregated_block_vec[i];
        int block_count = block_meta.block_count;
        int block_start = block_meta.block_start;
        //printf("block start: %d, block count: %d\n", block_start, block_count);
        for (int j = 0; j < block_count; j+=256) {
            int current_block_count = block_count - j > 256 ? 256 : block_count - j;
            int current_block_start = block_start + j;
            char *buffer = malloc(current_block_count * TRAJ_BLOCK_SIZE);
            memset(buffer, 0, current_block_count * TRAJ_BLOCK_SIZE);

            struct host_result_buffer result_buffer;
            init_result_buffer(&result_buffer, current_block_count * TRAJ_BLOCK_SIZE);

            start_read = clock();
            fetch_continuous_traj_data_block(data_storage, current_block_start, current_block_count, buffer);
            end_read = clock();
            pure_read += (end_read - start_read);
            for (int index = 0; index < current_block_count; index++) {
                start_computation = clock();
                //int count = spatio_temporal_query_raw_trajectory_block_with_result_buffer_without_meta_filtering(buffer + index * TRAJ_BLOCK_SIZE, predicate, &result_buffer);
                int count = spatio_temporal_query_raw_trajectory_block_without_meta_filtering(buffer + index * TRAJ_BLOCK_SIZE, predicate);
                //int count = spatio_temporal_query_raw_trajectory_block(buffer + index * TRAJ_BLOCK_SIZE, predicate);
                result_count += count;
                end_computation = clock();
                pure_computation += (end_computation - start_computation);
            }
            free(buffer);
            free_result_buffer(&result_buffer);
        }

    }



    end = clock();
    free_points_buffer(points_buffer_size);
    printf("[host] pure read time: %f\n",(double)(pure_read));
    printf("[host] pure computation time: %f\n",(double)(pure_computation));
    printf("[host] pure read and computation time: %f\n",(double)(pure_read + pure_computation));
    printf("[host] query time: %f\n",(double)(end-start_config));
    return result_count;
}

int spatio_temporal_query_without_pushdown_multi_addr(struct simple_query_engine *engine, struct spatio_temporal_range_predicate *predicate, bool enable_host_index) {

    struct index_entry_storage *index_storage = &engine->index_storage;
    struct traj_storage *data_storage = &engine->data_storage;
    struct seg_meta_section_entry_storage *meta_storage = &engine->seg_meta_storage;

    // calculate the matched block num
    int block_logical_addr_count = 0;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        if (enable_host_index) {
            if (predicate->lon_min <= entry->lon_max
                && predicate->lon_max >= entry->lon_min
                && predicate->lat_min <= entry->lat_max
                && predicate->lat_max >= entry->lat_min
                && predicate->time_min <= entry->time_max
                && predicate->time_max >= entry->time_min) {
                block_logical_addr_count++;
            }
        } else {
            block_logical_addr_count++;
        }
    }

    if (block_logical_addr_count == 0) {
        return 0;
    }

    // get the match block id vector
    // int block_logical_addr_vec[block_logical_addr_count];
    int *block_logical_addr_vec = malloc(block_logical_addr_count * sizeof(int));
    int addr_vec_index = 0;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        if (enable_host_index) {
            if (predicate->lon_min <= entry->lon_max
                && predicate->lon_max >= entry->lon_min
                && predicate->lat_min <= entry->lat_max
                && predicate->lat_max >= entry->lat_min
                && predicate->time_min <= entry->time_max
                && predicate->time_max >= entry->time_min) {
                block_logical_addr_vec[addr_vec_index] = entry->block_logical_adr;
                addr_vec_index++;
            }
        } else {
            block_logical_addr_vec[addr_vec_index] = entry->block_logical_adr;
            addr_vec_index++;
        }
    }


    printf("[host multi] read block num: %d\n", block_logical_addr_count);
    // run query
    int result_count = run_spatio_temporal_query_host_multi_addr(predicate, data_storage, block_logical_addr_count,
                                                         block_logical_addr_vec);
    free(block_logical_addr_vec);
    return result_count;
}

int spatio_temporal_query_without_pushdown_multi_addr_batch(struct simple_query_engine *engine, struct spatio_temporal_range_predicate *predicate, bool enable_host_index) {

    struct index_entry_storage *index_storage = &engine->index_storage;
    struct traj_storage *data_storage = &engine->data_storage;
    struct seg_meta_section_entry_storage *meta_storage = &engine->seg_meta_storage;

    // calculate the matched block num
    int block_logical_addr_count = 0;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        if (enable_host_index) {
            if (predicate->lon_min <= entry->lon_max
                && predicate->lon_max >= entry->lon_min
                && predicate->lat_min <= entry->lat_max
                && predicate->lat_max >= entry->lat_min
                && predicate->time_min <= entry->time_max
                && predicate->time_max >= entry->time_min) {
                block_logical_addr_count++;
            }
        } else {
            block_logical_addr_count++;
        }
    }

    if (block_logical_addr_count == 0) {
        return 0;
    }

    // get the match block id vector
    // int block_logical_addr_vec[block_logical_addr_count];
    int *block_logical_addr_vec = malloc(block_logical_addr_count * sizeof(int));
    int addr_vec_index = 0;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        if (enable_host_index) {
            if (predicate->lon_min <= entry->lon_max
                && predicate->lon_max >= entry->lon_min
                && predicate->lat_min <= entry->lat_max
                && predicate->lat_max >= entry->lat_min
                && predicate->time_min <= entry->time_max
                && predicate->time_max >= entry->time_min) {
                block_logical_addr_vec[addr_vec_index] = entry->block_logical_adr;
                addr_vec_index++;
            }
        } else {
            block_logical_addr_vec[addr_vec_index] = entry->block_logical_adr;
            addr_vec_index++;
        }
    }


    printf("[host multi batch] read block num: %d\n", block_logical_addr_count);
    // run query
    int result_count = run_spatio_temporal_query_host_multi_addr_batch(predicate, data_storage, block_logical_addr_count,
                                                                 block_logical_addr_vec);
    free(block_logical_addr_vec);
    return result_count;
}

int
run_spatio_temporal_query_host_multi_addr(struct spatio_temporal_range_predicate *predicate, struct traj_storage *data_storage,int block_logical_addr_count,
                                      int *block_logical_addr_vec) {
    int result_count = 0;

    clock_t start, end, pure_read;
    pure_read = 0;
    clock_t start_all, end_all;
    int total_result_block_num = 0;
    start_all = clock();
    fill_points_buffer(points_buffer_size);
    for (int i = 0; i < block_logical_addr_count; i += 256) {
        int block_addr_vec_size = block_logical_addr_count - i > 256 ? 256 : block_logical_addr_count - i;
        int result_size = block_addr_vec_size * 0x1000;

        if (result_size % 0x1000 != 0) {
            result_size = ((result_size / 0x1000) + 1) * 0x1000;
        }
        int result_block_num = result_size / 0x1000;
        //int estimated_result_block_num = 1;
        total_result_block_num += result_block_num;


        void* result_buffer = malloc(result_size);

        int block_meta_vec_size = calculate_aggregate_block_vec_size(&block_logical_addr_vec[i], block_addr_vec_size);
        struct continuous_block_meta block_meta_vec[block_meta_vec_size];
        aggregate_blocks(&block_logical_addr_vec[i], block_addr_vec_size, block_meta_vec, block_meta_vec_size);

        struct isp_descriptor isp_desc;
        struct lba lba_vec[block_meta_vec_size];
        assemble_lba_vec_via_block_meta(lba_vec, DATA_FILE_OFFSET, block_meta_vec, block_meta_vec_size);
        assemble_isp_desc_without_comp(&isp_desc, result_block_num, lba_vec, block_meta_vec_size);

        start = clock();
        do_isp_for_trajectory_data_without_comp(data_storage, result_buffer, result_size, &isp_desc);
        end = clock();
        pure_read += (end - start);
        for (int k = 0; k < block_addr_vec_size; k++) {
            result_count += spatio_temporal_query_raw_trajectory_block_without_meta_filtering(result_buffer + k * TRAJ_BLOCK_SIZE, predicate);
        }

        free(result_buffer);


    }
    free_points_buffer(points_buffer_size);
    end_all = clock();
    printf("[host multi] estimated result block number: %d\n", total_result_block_num);
    printf("[host multi] pure read time: %f\n",(double)pure_read);
    printf("[host multi] query time (including computation): %f\n", (double)(end_all - start_all));

    return result_count;
}

int
run_spatio_temporal_query_host_multi_addr_batch(struct spatio_temporal_range_predicate *predicate, struct traj_storage *data_storage,
                                       int block_logical_addr_count,
                                       int *block_logical_addr_vec) {
    int result_count = 0;

    clock_t start, end, pure_read, comp_start, comp_end, pure_comp;
    pure_read = 0;
    pure_comp = 0;
    clock_t start_all, end_all;
    int total_result_block_num = 0;
    start_all = clock();
    fill_points_buffer(points_buffer_size);

    // prepare batch parameters
    int batch_size = REQUEST_BATCH_SIZE; //  fix bug here -> fixed: caused by the same request used in the batch using the same lba_array (before, we do not use malloc for lba_array)
    void* result_buffer_vec[batch_size];
    size_t result_buffer_size_vec[batch_size];
    struct isp_descriptor *isp_desc_vec[batch_size];
    struct lba *lba_vec_ptr[batch_size];
    int batch_count = 0;

    int batch_function_call_num = 0;
    for (int i = 0; i < block_logical_addr_count; i += 256) {

        int block_addr_vec_size = block_logical_addr_count - i > 256 ? 256 : block_logical_addr_count - i;
        int result_size = block_addr_vec_size * 0x1000;


        if (result_size % 0x1000 != 0) {
            result_size = ((result_size / 0x1000) + 1) * 0x1000;
        }
        int estimated_result_block_num = result_size / 0x1000;
        //int estimated_result_block_num = 1;
        total_result_block_num += estimated_result_block_num;


        void* result_buffer = malloc(result_size);

        int block_meta_vec_size = calculate_aggregate_block_vec_size(&block_logical_addr_vec[i], block_addr_vec_size);
        struct continuous_block_meta block_meta_vec[block_meta_vec_size];
        aggregate_blocks(&block_logical_addr_vec[i], block_addr_vec_size, block_meta_vec, block_meta_vec_size);

        struct isp_descriptor *isp_desc = malloc(sizeof(struct isp_descriptor));
        struct lba *lba_vec = malloc(sizeof(struct lba) * block_meta_vec_size);
        assemble_lba_vec_via_block_meta(lba_vec, DATA_FILE_OFFSET, block_meta_vec, block_meta_vec_size);
        assemble_isp_desc_without_comp(isp_desc, estimated_result_block_num, lba_vec, block_meta_vec_size);

        //start = clock();
        result_buffer_vec[batch_count] = result_buffer;
        result_buffer_size_vec[batch_count] = result_size;
        isp_desc_vec[batch_count] = isp_desc;
        lba_vec_ptr[batch_count] = lba_vec;


        batch_count++;
        if (batch_count % batch_size == 0) {
            start = clock();
            do_isp_for_trajectory_data_without_comp_batch(batch_size, data_storage, result_buffer_vec, result_buffer_size_vec, isp_desc_vec);
            batch_function_call_num++;
            end = clock();

            for (int k = 0; k < batch_size; k++) {
                void* batch_base = result_buffer_vec[k];
                size_t batch_buffer_count = result_buffer_size_vec[k] / 0x1000;
                for (int j = 0; j < batch_buffer_count; j++) {
                    void* block_ptr = batch_base + j * TRAJ_BLOCK_SIZE;
                    comp_start = clock();
                    result_count += spatio_temporal_query_raw_trajectory_block(block_ptr, predicate);
                    comp_end = clock();
                    pure_comp += (comp_end - comp_start);
                }
                //print_isp_descriptor(isp_desc_vec[k]);
            }
            batch_count = 0;

            pure_read += (end - start);
            for (int m = 0; m < batch_size; m++) {
                free(result_buffer_vec[m]);
                free(isp_desc_vec[m]);
                free(lba_vec_ptr[m]);
            }

        }

    }
    // handle the last batch (less than batch size)
    start = clock();
    do_isp_for_trajectory_data_without_comp_batch(batch_count, data_storage, result_buffer_vec, result_buffer_size_vec, isp_desc_vec);
    batch_function_call_num++;
    end = clock();

    for (int k = 0; k < batch_count; k++) {
        void* batch_base = result_buffer_vec[k];
        size_t batch_buffer_count = result_buffer_size_vec[k] / 0x1000;
        for (int j = 0; j < batch_buffer_count; j++) {
            void* block_ptr = batch_base + j * TRAJ_BLOCK_SIZE;
            comp_start = clock();
            result_count += spatio_temporal_query_raw_trajectory_block(block_ptr, predicate);
            comp_end = clock();
            pure_comp += (comp_end - comp_start);
            //print_isp_descriptor(isp_desc_vec[k]);
        }
    }

    pure_read += (end - start);
    for (int m = 0; m < batch_count; m++) {
        free(result_buffer_vec[m]);
        free(isp_desc_vec[m]);
        free(lba_vec_ptr[m]);
    }


    free_points_buffer(points_buffer_size);
    end_all = clock();
    printf("batch function call num: %d\n", batch_function_call_num);
    printf("[host multi batch] estimated result block number: %d\n", total_result_block_num);
    printf("[host multi batch] pure read time: %f\n",(double)pure_read);
    printf("[host multi batch] pure computation time: %f\n",(double)pure_comp);
    printf("[host multi batch] query time (including computation): %f\n", (double)(end_all - start_all));

    return result_count;
}





struct st_computation_data {
    struct spatio_temporal_range_predicate *predicate;
    int batch_count;
    void** result_buffer;
    size_t *result_size;
    int result_count;
    int cpu_id;
};

struct isp_st_computation_data {
    struct spatio_temporal_range_predicate *predicate;
    struct traj_storage *data_storage;
    struct seg_meta_section_entry_storage *meta_storage;
    int block_logical_addr_count;
    int *block_logical_addr_vec;
    bool enable_result_estimation;
    int result_count;
    int cpu_id;
};

int set_cpu(int i) {
    cpu_set_t mask;
    CPU_ZERO(&mask);
    CPU_SET(i, &mask);
    printf("thread %u, i = %d\n", pthread_self(), i);
    if (-1 == pthread_setaffinity_np(pthread_self(), sizeof(mask), &mask)) {
        return -1;
    }
    return 0;

}

void *spatio_temporal_computation(void *ptr) {
    struct st_computation_data * computation_data = (struct st_computation_data *) ptr;
    if (set_cpu(computation_data->cpu_id)) {
        printf("set cpu error\n");
    }

    int batch_count = computation_data->batch_count;
    void** result_buffer_vec = computation_data->result_buffer;
    size_t *result_buffer_size_vec = computation_data->result_size;

    struct spatio_temporal_range_predicate st_predicate = *computation_data->predicate;
    int result_count = 0;
    for (int k = 0; k < batch_count; k++) {
        void* batch_base = result_buffer_vec[k];
        size_t batch_buffer_count = result_buffer_size_vec[k] / 0x1000;
        for (int j = 0; j < batch_buffer_count; j++) {
            void* block_ptr = batch_base + j * TRAJ_BLOCK_SIZE;
            result_count += spatio_temporal_query_raw_trajectory_block(block_ptr, &st_predicate);
            //print_isp_descriptor(isp_desc_vec[k]);
        }
    }

    computation_data->result_count = result_count;
    pthread_exit(NULL);
}

void *isp_spatio_temporal_computation(void *ptr) {

    struct isp_st_computation_data * params = (struct isp_st_computation_data *) ptr;
    if (set_cpu(params->cpu_id)) {
        printf("set cpu error\n");
    }

    int result_count = run_spatio_temporal_query_device_batch(params->predicate, params->data_storage, params->meta_storage, params->block_logical_addr_count, params->block_logical_addr_vec, params->enable_result_estimation);
    params->result_count = result_count;
    pthread_exit(NULL);
}


/*
 * in a batch, the first set is host isp and the remaining set is isp cpu
 */
int
run_spatio_temporal_query_hybrid_batch(struct spatio_temporal_range_predicate *predicate, struct traj_storage *data_storage, struct seg_meta_section_entry_storage *meta_storage,
                                       int host_block_logical_addr_count,
                                       int *host_block_logical_addr_vec,
                                       int pushdown_block_logical_addr_count,
                                       int *pushdown_block_logical_addr_vec,
                                       bool enable_result_estimation) {
    int result_count = 0;

    clock_t start, end, pure_read;
    pure_read = 0;
    clock_t start_all, end_all;
    int total_result_block_num = 0;
    start_all = clock();

    // prepare batch parameters
    int batch_size = REQUEST_BATCH_SIZE; //  fix bug here -> fixed: caused by the same request used in the batch using the same lba_array (before, we do not use malloc for lba_array)
    int host_isp_request_num = host_block_logical_addr_count / 256 + 1;
    int host_request_count = 0;
    if (host_block_logical_addr_count % 256 == 0) {
        host_isp_request_num = host_block_logical_addr_count / 256;
    }
    void* host_result_buffer_vec[host_isp_request_num];
    size_t host_result_buffer_size_vec[host_isp_request_num];
    struct isp_descriptor *host_isp_desc_vec[batch_size];
    struct lba *host_lba_vec_ptr[batch_size];

    void* pushdown_result_buffer_vec[batch_size];
    size_t pushdown_result_buffer_size_vec[batch_size];
    struct isp_descriptor *pushdwon_isp_desc_vec[batch_size];
    struct lba *pushdown_lba_vec_ptr[batch_size];

    // first read data for host computation
    int batch_count = 0;
    int host_loop_index, pushdown_loop_index;
    pushdown_loop_index = 0;
    for (host_loop_index = 0; host_loop_index < host_block_logical_addr_count; host_loop_index+= 256) {
        int host_block_addr_vec_size = host_block_logical_addr_count - host_loop_index > 256 ? 256 : host_block_logical_addr_count - host_loop_index;
        int host_result_size = host_block_addr_vec_size * 0x1000;


        if (host_result_size % 0x1000 != 0) {
            host_result_size = ((host_result_size / 0x1000) + 1) * 0x1000;
        }
        int host_result_block_num = host_result_size / 0x1000;
        //int estimated_result_block_num = 1;
        total_result_block_num += host_result_block_num;


        void* host_result_buffer = malloc(host_result_size);

        int host_block_meta_vec_size = calculate_aggregate_block_vec_size(&host_block_logical_addr_vec[host_loop_index], host_block_addr_vec_size);
        struct continuous_block_meta host_block_meta_vec[host_block_meta_vec_size];
        aggregate_blocks(&host_block_logical_addr_vec[host_loop_index], host_block_addr_vec_size, host_block_meta_vec, host_block_meta_vec_size);

        struct isp_descriptor *host_isp_desc = malloc(sizeof(struct isp_descriptor));
        struct lba *host_lba_vec = malloc(sizeof(struct lba) * host_block_meta_vec_size);
        assemble_lba_vec_via_block_meta(host_lba_vec, DATA_FILE_OFFSET, host_block_meta_vec, host_block_meta_vec_size);
        assemble_isp_desc_without_comp(host_isp_desc, host_result_block_num, host_lba_vec, host_block_meta_vec_size);

        //start = clock();
        host_result_buffer_vec[host_request_count] = host_result_buffer;
        host_result_buffer_size_vec[host_request_count] = host_result_size;
        host_isp_desc_vec[batch_count] = host_isp_desc;
        host_lba_vec_ptr[batch_count] = host_lba_vec;


        batch_count++;
        host_request_count++;
        if ((batch_count % batch_size == 0) || (host_loop_index + 256 >= host_block_logical_addr_count)) {
            // submit host computation
            start = clock();
            //do_isp_for_trajectory_data_without_comp_batch(batch_count, data_storage, host_result_buffer_vec+host_request_count-batch_count, host_result_buffer_size_vec+host_request_count-batch_count, host_isp_desc_vec);
            do_isp_for_trajectory_data_without_comp_batch(batch_count, data_storage, &host_result_buffer_vec[host_request_count-batch_count], &host_result_buffer_size_vec[host_request_count-batch_count], host_isp_desc_vec);
            //do_isp_for_trajectory_data_without_comp_batch(batch_count, data_storage, host_result_buffer_vec, host_result_buffer_size_vec, host_isp_desc_vec);

            end = clock();
            pure_read += (end - start);


            for (int m = 0; m < batch_count; m++) {
                //print_isp_descriptor(host_isp_desc_vec[m]);
                free(host_isp_desc_vec[m]);
                free(host_lba_vec_ptr[m]);
            }

            batch_count = 0;
        }
    }

    // lanuch a thread for host computation and submit the computation to the device
    pthread_t thread_comp, thread_isp;
    int iret1, iret2;
    struct st_computation_data computation_data = {predicate, host_request_count, host_result_buffer_vec, host_result_buffer_size_vec, 0};
    computation_data.cpu_id = 6;
    iret1 = pthread_create(&thread_comp, NULL, spatio_temporal_computation, &computation_data);

    // submit device computation
    struct isp_st_computation_data isp_data = {predicate, data_storage, meta_storage, pushdown_block_logical_addr_count, pushdown_block_logical_addr_vec, enable_result_estimation};
    isp_data.cpu_id = 7;
    iret2 = pthread_create(&thread_isp, NULL, isp_spatio_temporal_computation, &isp_data);

    //result_count+= run_spatio_temporal_query_device_batch(predicate, data_storage, meta_storage, pushdown_block_logical_addr_count, pushdown_block_logical_addr_vec, enable_result_estimation);

    pthread_join(thread_comp, NULL);
    pthread_join(thread_isp, NULL);
    result_count += computation_data.result_count;
    result_count += isp_data.result_count;
    for (int m = 0; m < host_request_count; m++) {
        free(host_result_buffer_vec[m]);
    }


    end_all = clock();


    printf("[host & device parallel batch] total estimated result block number: %d\n", total_result_block_num);
    printf("[host & device parallel batch] host pure read time: %f\n",(double)pure_read);
    printf("[host & device parallel batch] query time (including computation): %f\n", (double)(end_all - start_all));

    return result_count;
}


/*
 * in a batch, the first set is host isp and the remaining set is isp cpu
 */
int
run_spatio_temporal_query_hybrid_batch_poor_perf(struct spatio_temporal_range_predicate *predicate, struct traj_storage *data_storage, struct seg_meta_section_entry_storage *meta_storage,
                                       int host_block_logical_addr_count,
                                       int *host_block_logical_addr_vec,
                                       int pushdown_block_logical_addr_count,
                                       int *pushdown_block_logical_addr_vec,
                                       bool enable_result_estimation) {
    int result_count = 0;

    clock_t start, end, pure_read;
    pure_read = 0;
    clock_t start_all, end_all;
    int total_result_block_num = 0;
    start_all = clock();

    // prepare batch parameters
    int batch_size = REQUEST_BATCH_SIZE; //  fix bug here -> fixed: caused by the same request used in the batch using the same lba_array (before, we do not use malloc for lba_array)
    void* host_result_buffer_vec[batch_size];
    size_t host_result_buffer_size_vec[batch_size];
    struct isp_descriptor *host_isp_desc_vec[batch_size];
    struct lba *host_lba_vec_ptr[batch_size];

    void* pushdown_result_buffer_vec[batch_size];
    size_t pushdown_result_buffer_size_vec[batch_size];
    struct isp_descriptor *pushdwon_isp_desc_vec[batch_size];
    struct lba *pushdown_lba_vec_ptr[batch_size];

    int batch_count = 0;
    pthread_t thread_comp;
    int host_loop_index, pushdown_loop_index;
    pushdown_loop_index = 0;
    for (host_loop_index = 0; host_loop_index < host_block_logical_addr_count; host_loop_index+= 256) {
        int host_block_addr_vec_size = host_block_logical_addr_count - host_loop_index > 256 ? 256 : host_block_logical_addr_count - host_loop_index;
        int host_result_size = host_block_addr_vec_size * 0x1000;


        if (host_result_size % 0x1000 != 0) {
            host_result_size = ((host_result_size / 0x1000) + 1) * 0x1000;
        }
        int host_result_block_num = host_result_size / 0x1000;
        //int estimated_result_block_num = 1;
        total_result_block_num += host_result_block_num;


        void* host_result_buffer = malloc(host_result_size);

        int host_block_meta_vec_size = calculate_aggregate_block_vec_size(&host_block_logical_addr_vec[host_loop_index], host_block_addr_vec_size);
        struct continuous_block_meta host_block_meta_vec[host_block_meta_vec_size];
        aggregate_blocks(&host_block_logical_addr_vec[host_loop_index], host_block_addr_vec_size, host_block_meta_vec, host_block_meta_vec_size);

        struct isp_descriptor *host_isp_desc = malloc(sizeof(struct isp_descriptor));
        struct lba *host_lba_vec = malloc(sizeof(struct lba) * host_block_meta_vec_size);
        assemble_lba_vec_via_block_meta(host_lba_vec, DATA_FILE_OFFSET, host_block_meta_vec, host_block_meta_vec_size);
        assemble_isp_desc_without_comp(host_isp_desc, host_result_block_num, host_lba_vec, host_block_meta_vec_size);

        //start = clock();
        host_result_buffer_vec[batch_count] = host_result_buffer;
        host_result_buffer_size_vec[batch_count] = host_result_size;
        host_isp_desc_vec[batch_count] = host_isp_desc;
        host_lba_vec_ptr[batch_count] = host_lba_vec;


        batch_count++;
        if ((batch_count % batch_size == 0) || (host_loop_index + 256 >= host_block_logical_addr_count)) {
            // submit host computation
            start = clock();
            do_isp_for_trajectory_data_hybrid_comp_batch(batch_count, data_storage, host_result_buffer_vec, host_result_buffer_size_vec, host_isp_desc_vec);
            end = clock();
            pure_read += (end - start);

            int iret1;
            struct st_computation_data computation_data = {predicate, batch_count, host_result_buffer_vec, host_result_buffer_size_vec, 0};
            iret1 = pthread_create(&thread_comp, NULL, spatio_temporal_computation, &computation_data);

            int pushdown_block_addr_vec_size = pushdown_block_logical_addr_count - pushdown_loop_index > 256 ? 256 : pushdown_block_logical_addr_count - pushdown_loop_index;
            int pushdown_result_count = run_spatio_temporal_query_device_batch(predicate, data_storage, meta_storage, pushdown_block_addr_vec_size, &pushdown_block_logical_addr_vec[pushdown_loop_index], enable_result_estimation);
            result_count += pushdown_result_count;
            pushdown_loop_index += 256;

            pthread_join(thread_comp, NULL);
            result_count += computation_data.result_count;




            for (int m = 0; m < batch_count; m++) {
                free(host_result_buffer_vec[m]);
                free(host_isp_desc_vec[m]);
                free(host_lba_vec_ptr[m]);
            }

            batch_count = 0;
            if (pushdown_loop_index >= pushdown_block_logical_addr_count) {
                break;
            }

        }
    }

    // handle the remaining ones
    if (host_loop_index < host_block_logical_addr_count) {
        int remain_host_block_count = host_block_logical_addr_count - host_loop_index;
        int remain_host_result_count = run_spatio_temporal_query_host_multi_addr_batch(predicate, data_storage, remain_host_block_count, &host_block_logical_addr_vec[host_loop_index]);
        result_count += remain_host_result_count;
    }

    if (pushdown_loop_index < pushdown_block_logical_addr_count) {
        int remain_pushdown_block_count = pushdown_block_logical_addr_count - pushdown_loop_index;
        int remain_pushdown_result_count = run_spatio_temporal_query_device_batch(predicate, data_storage, meta_storage, remain_pushdown_block_count, &pushdown_block_logical_addr_vec[pushdown_loop_index], enable_result_estimation);
        result_count += remain_pushdown_result_count;
    }
    end_all = clock();


    printf("[host & device parallel batch] total estimated result block number: %d\n", total_result_block_num);
    //printf("[host & device parallel batch] read time: %f\n",(double)pure_read);
    printf("[host & device parallel batch] query time (including computation): %f\n", (double)(end_all - start_all));

    return result_count;
}


/*
 * not used (the performance of this version is not good, why?)
 * in a batch, the first set is host isp and the remaining set is isp cpu
 */
int
run_spatio_temporal_query_hybrid_batch_old(struct spatio_temporal_range_predicate *predicate, struct traj_storage *data_storage, struct seg_meta_section_entry_storage *meta_storage,
                                                int host_block_logical_addr_count,
                                                int *host_block_logical_addr_vec,
                                                int pushdown_block_logical_addr_count,
                                                int *pushdown_block_logical_addr_vec,
                                                bool enable_result_estimation) {
    int result_count = 0;

    clock_t start, end, pure_read;
    pure_read = 0;
    clock_t start_all, end_all;
    int total_result_block_num = 0;
    start_all = clock();
    //fill_points_buffer(points_buffer_size);

    int host_isp_request_num = host_block_logical_addr_count / 256 + 1;
    if (host_block_logical_addr_count % 256 == 0) {
        host_isp_request_num = host_block_logical_addr_count / 256;
    }
    int pushdown_isp_request_num = pushdown_block_logical_addr_count / 256 + 1;
    if (pushdown_block_logical_addr_count % 256 == 0) {
        pushdown_isp_request_num = pushdown_block_logical_addr_count / 256;
    }

    // prepare isp descriptor for host
    void* host_result_buffer_vec[host_isp_request_num];
    size_t host_result_buffer_size_vec[host_isp_request_num];
    struct isp_descriptor *host_isp_desc_vec[host_isp_request_num];
    struct lba *host_lba_vec_ptr[host_isp_request_num];

    int host_isp_count = 0;
    for (int i = 0; i < host_block_logical_addr_count; i += 256) {

        int block_addr_vec_size = host_block_logical_addr_count - i > 256 ? 256 : host_block_logical_addr_count - i;
        int result_size = block_addr_vec_size * 0x1000;


        if (result_size % 0x1000 != 0) {
            result_size = ((result_size / 0x1000) + 1) * 0x1000;
        }
        int estimated_result_block_num = result_size / 0x1000;
        //int estimated_result_block_num = 1;
        total_result_block_num += estimated_result_block_num;


        void* result_buffer = malloc(result_size);

        int block_meta_vec_size = calculate_aggregate_block_vec_size(&host_block_logical_addr_vec[i], block_addr_vec_size);
        struct continuous_block_meta block_meta_vec[block_meta_vec_size];
        aggregate_blocks(&host_block_logical_addr_vec[i], block_addr_vec_size, block_meta_vec, block_meta_vec_size);

        struct isp_descriptor *isp_desc = malloc(sizeof(struct isp_descriptor));
        struct lba *lba_vec = malloc(sizeof(struct lba) * block_meta_vec_size);
        assemble_lba_vec_via_block_meta(lba_vec, DATA_FILE_OFFSET, block_meta_vec, block_meta_vec_size);
        assemble_isp_desc_without_comp(isp_desc, estimated_result_block_num, lba_vec, block_meta_vec_size);

        //start = clock();
        host_result_buffer_vec[host_isp_count] = result_buffer;
        host_result_buffer_size_vec[host_isp_count] = result_size;
        host_isp_desc_vec[host_isp_count] = isp_desc;
        host_lba_vec_ptr[host_isp_count] = lba_vec;
        host_isp_count++;
    }


    // prepare isp descriptor for device
    void* pushdown_result_buffer_vec[pushdown_isp_request_num];
    size_t pushdown_result_buffer_size_vec[pushdown_isp_request_num];
    struct isp_descriptor *pushdown_isp_desc_vec[pushdown_isp_request_num];
    struct lba *pushdown_lba_vec_ptr[pushdown_isp_request_num];

    int pushdown_isp_count = 0;
    for (int i = 0; i < pushdown_block_logical_addr_count; i += 256) {

        int block_addr_vec_size = pushdown_block_logical_addr_count - i > 256 ? 256 : pushdown_block_logical_addr_count - i;
        int estimated_result_size = block_addr_vec_size * 0x1000;
        if (enable_result_estimation) {
            estimated_result_size = estimate_spatio_temporal_result_size_for_blocks(meta_storage, predicate,
                                                                                    &pushdown_block_logical_addr_vec[i],
                                                                                    block_addr_vec_size);
        }
        //printf("estimated result size: %d\n", estimated_result_size);
        if (estimated_result_size == 0) {
            continue;
        }
        if (estimated_result_size % 0x1000 != 0) {
            estimated_result_size = ((estimated_result_size / 0x1000) + 1) * 0x1000;
        }
        int estimated_result_block_num = estimated_result_size / 0x1000;
        //int estimated_result_block_num = 1;
        total_result_block_num += estimated_result_block_num;


        void* result_buffer = malloc(estimated_result_size);

        int block_meta_vec_size = calculate_aggregate_block_vec_size(&pushdown_block_logical_addr_vec[i], block_addr_vec_size);
        struct continuous_block_meta block_meta_vec[block_meta_vec_size];
        aggregate_blocks(&pushdown_block_logical_addr_vec[i], block_addr_vec_size, block_meta_vec, block_meta_vec_size);

        struct isp_descriptor *isp_desc = malloc(sizeof(struct isp_descriptor));
        struct lba *lba_vec = malloc(sizeof(struct lba) * block_meta_vec_size);
        assemble_lba_vec_via_block_meta(lba_vec, DATA_FILE_OFFSET, block_meta_vec, block_meta_vec_size);
        assemble_isp_desc_for_spatial_temporal(isp_desc, predicate, estimated_result_block_num, lba_vec, block_meta_vec_size);

        //start = clock();
        pushdown_result_buffer_vec[pushdown_isp_count] = result_buffer;
        pushdown_result_buffer_size_vec[pushdown_isp_count] = estimated_result_size;
        pushdown_isp_desc_vec[pushdown_isp_count] = isp_desc;
        pushdown_lba_vec_ptr[pushdown_isp_count] = lba_vec;
        pushdown_isp_count++;
    }
    pushdown_isp_request_num = pushdown_isp_count;

    // prepare batch parameters
    int batch_size = REQUEST_BATCH_SIZE; //  fix bug here -> fixed: caused by the same request used in the batch using the same lba_array (before, we do not use malloc for lba_array)
    int host_batch_count = 0;
    int pushdown_batch_count = 0;
    void* host_batch_result_buffer_vec[batch_size];
    size_t host_batch_result_buffer_size_vec[batch_size];
    struct isp_descriptor *host_batch_isp_desc_vec[batch_size];
    struct lba *host_batch_lba_vec_ptr[batch_size];
    void* pushdown_batch_result_buffer_vec[batch_size];
    size_t pushdown_batch_result_buffer_size_vec[batch_size];
    struct isp_descriptor *pushdown_batch_isp_desc_vec[batch_size];
    struct lba *pushdown_batch_lba_vec_ptr[batch_size];

    int host_batch_func_call_num = host_isp_request_num / batch_size + 1;
    if (host_isp_request_num % batch_size == 0) {
        host_batch_func_call_num = host_isp_request_num / batch_size;
    }
    int pushdown_batch_func_call_num = pushdown_isp_request_num / batch_size + 1;
    if (pushdown_isp_request_num % batch_size == 0) {
        pushdown_batch_func_call_num = pushdown_isp_request_num / batch_size;
    }

    printf("host batch function call num: %d, pushdown batch function call num: %d\n", host_batch_func_call_num, pushdown_batch_func_call_num);
    // begin computation
    clock_t host_start, host_end, pure_host, pushdown_start, pushdown_end, pure_pushdown;
    pure_host = 0;
    pure_pushdown = 0;
    start = clock();
    pthread_t thread_comp;
    if ((host_batch_func_call_num == 1 && pushdown_batch_func_call_num == 1)) {
        // submit host batch
        host_start = clock();
        do_isp_for_trajectory_data_hybrid_comp_batch(host_isp_request_num, data_storage, host_result_buffer_vec, host_result_buffer_size_vec, host_isp_desc_vec);
        host_end = clock();
        pure_host += host_end - host_start;
        // lanuch another thread to do the host computation
        int iret1;
        struct st_computation_data computation_data = {predicate, host_isp_request_num, host_result_buffer_vec, host_result_buffer_size_vec, 0};
        iret1 = pthread_create(&thread_comp, NULL, spatio_temporal_computation, &computation_data);

        // submit device batch
        pushdown_start = clock();
        do_isp_for_trajectory_data_hybrid_comp_batch(pushdown_isp_request_num, data_storage, pushdown_result_buffer_vec, pushdown_result_buffer_size_vec, pushdown_isp_desc_vec);
        pushdown_end = clock();
        pure_pushdown += pushdown_end - pushdown_start;
        for (int k = 0; k < pushdown_isp_request_num; k++) {
            void* batch_base = pushdown_result_buffer_vec[k];
            result_count += spatio_temporal_query_isp_new_format_output_blocks(batch_base, predicate);
        }

        pthread_join(thread_comp, NULL);
        result_count += computation_data.result_count;

    } else {
        if (host_batch_func_call_num >= pushdown_batch_func_call_num) {
            for (int i = 0; i < host_batch_func_call_num; i++) {
                if (i < pushdown_batch_func_call_num) {
                    // have both host and device computation
                    int host_vec_index = i * batch_size;
                    int pushdown_vec_index = i * batch_size;
                    int host_batch_size = host_isp_request_num - host_vec_index > batch_size ? batch_size : (host_isp_request_num - host_vec_index);
                    int pushdown_batch_size = pushdown_isp_request_num - pushdown_vec_index > batch_size ? batch_size : (pushdown_isp_request_num - pushdown_vec_index);

                    // submit host batch
                    host_start = clock();
                    do_isp_for_trajectory_data_hybrid_comp_batch(host_batch_size, data_storage, &host_result_buffer_vec[host_vec_index], &host_result_buffer_size_vec[host_vec_index], &host_isp_desc_vec[host_vec_index]);
                    host_end = clock();
                    pure_host += host_end - host_start;
                    // lanuch another thread to do the host computation
                    int iret1;
                    struct st_computation_data computation_data = {predicate, host_batch_size, &host_result_buffer_vec[host_vec_index], &host_result_buffer_size_vec[host_vec_index], 0};
                    iret1 = pthread_create(&thread_comp, NULL, spatio_temporal_computation, &computation_data);

                    // submit device batch
                    pushdown_start = clock();
                    do_isp_for_trajectory_data_hybrid_comp_batch(pushdown_batch_size, data_storage, &pushdown_result_buffer_vec[pushdown_vec_index], &pushdown_result_buffer_size_vec[pushdown_vec_index], &pushdown_isp_desc_vec[pushdown_vec_index]);
                    pushdown_end = clock();
                    pure_pushdown += pushdown_end - pushdown_start;
                    for (int k = 0; k < pushdown_batch_size; k++) {
                        void* batch_base = pushdown_result_buffer_vec[pushdown_vec_index + k];
                        result_count += spatio_temporal_query_isp_new_format_output_blocks(batch_base, predicate);
                    }
                    pthread_join(thread_comp, NULL);
                    result_count += computation_data.result_count;
                } else {
                    // only have host computation left
                    int host_vec_index = i * batch_size;
                    int host_batch_size = host_isp_request_num - host_vec_index > batch_size ? batch_size : (host_isp_request_num - host_vec_index);

                    // submit host batch
                    host_start = clock();
                    do_isp_for_trajectory_data_hybrid_comp_batch(host_batch_size, data_storage, &host_result_buffer_vec[host_vec_index], &host_result_buffer_size_vec[host_vec_index], &host_isp_desc_vec[host_vec_index]);
                    host_end = clock();
                    pure_host += host_end - host_start;
                    for (int k = 0; k < host_batch_size; k++) {
                        void* batch_base = host_result_buffer_vec[host_vec_index + k];
                        size_t batch_buffer_count = host_result_buffer_size_vec[host_vec_index + k] / 0x1000;
                        for (int j = 0; j < batch_buffer_count; j++) {
                            void* block_ptr = batch_base + j * TRAJ_BLOCK_SIZE;
                            result_count += spatio_temporal_query_raw_trajectory_block_without_meta_filtering(block_ptr, predicate);
                            // print_isp_descriptor(isp_desc_vec[k]);
                        }
                    }
                }

            }
        } else {
            for (int i = 0; i < pushdown_batch_func_call_num; i++) {
                if (i < host_batch_func_call_num) {
                    // have both host and device computation
                    int host_vec_index = i * batch_size;
                    int pushdown_vec_index = i * batch_size;
                    int host_batch_size = host_isp_request_num - host_vec_index > batch_size ? batch_size : (host_isp_request_num - host_vec_index);
                    int pushdown_batch_size = pushdown_isp_request_num - pushdown_vec_index > batch_size ? batch_size : (pushdown_isp_request_num - pushdown_vec_index);

                    // submit host batch
                    host_start = clock();
                    do_isp_for_trajectory_data_hybrid_comp_batch(host_batch_size, data_storage, &host_result_buffer_vec[host_vec_index], &host_result_buffer_size_vec[host_vec_index], &host_isp_desc_vec[host_vec_index]);
                    host_end = clock();
                    pure_host += host_end - host_start;
                    // lanuch another thread to do the host computation
                    int iret1;
                    struct st_computation_data computation_data = {predicate, host_batch_size, &host_result_buffer_vec[host_vec_index], &host_result_buffer_size_vec[host_vec_index], 0};
                    iret1 = pthread_create(&thread_comp, NULL, spatio_temporal_computation, (void*)&computation_data);

                    // submit device batch
                    pushdown_start = clock();
                    do_isp_for_trajectory_data_hybrid_comp_batch(pushdown_batch_size, data_storage, &pushdown_result_buffer_vec[pushdown_vec_index], &pushdown_result_buffer_size_vec[pushdown_vec_index], &pushdown_isp_desc_vec[pushdown_vec_index]);
                    pushdown_end = clock();
                    pure_pushdown += pushdown_end - pushdown_start;
                    for (int k = 0; k < pushdown_batch_size; k++) {
                        void* batch_base = pushdown_result_buffer_vec[pushdown_vec_index + k];
                        result_count += spatio_temporal_query_isp_new_format_output_blocks(batch_base, predicate);
                    }
                    pthread_join(thread_comp, NULL);
                    result_count += computation_data.result_count;
                } else {
                    // only have device computation left
                    int host_vec_index = i * batch_size;
                    int pushdown_vec_index = i * batch_size;
                    int host_batch_size = host_isp_request_num - host_vec_index > batch_size ? batch_size : (host_isp_request_num - host_vec_index);
                    int pushdown_batch_size = pushdown_isp_request_num - pushdown_vec_index > batch_size ? batch_size : (pushdown_isp_request_num - pushdown_vec_index);

                    // submit device batch
                    pushdown_start = clock();
                    do_isp_for_trajectory_data_hybrid_comp_batch(pushdown_batch_size, data_storage, &pushdown_result_buffer_vec[pushdown_vec_index], &pushdown_result_buffer_size_vec[pushdown_vec_index], &pushdown_isp_desc_vec[pushdown_vec_index]);
                    pushdown_end = clock();
                    pure_pushdown += pushdown_end - pushdown_start;
                    for (int k = 0; k < pushdown_batch_size; k++) {
                        void* batch_base = pushdown_result_buffer_vec[pushdown_vec_index + k];
                        result_count += spatio_temporal_query_isp_new_format_output_blocks(batch_base, predicate);
                    }
                }

            }
        }
    }
    end = clock();
    pure_read = end - start;

    //free_points_buffer(points_buffer_size);

    for (int i = 0; i < host_isp_count; i++) {
        free(host_result_buffer_vec[i]);
        free(host_isp_desc_vec[i]);
        free(host_lba_vec_ptr[i]);
    }
    for (int i = 0; i < pushdown_isp_count; i++) {
        free(pushdown_result_buffer_vec[i]);
        free(pushdown_isp_desc_vec[i]);
        free(pushdown_lba_vec_ptr[i]);
    }


    end_all = clock();
    printf("[host & device parallel batch] total estimated result block number: %d\n", total_result_block_num);
    printf("[host & device parallel batch] pure host read time: %f\n",(double)(pure_host));
    printf("[host & device parallel batch] pure device read time: %f\n",(double)(pure_pushdown));
    printf("[host & device parallel batch] pure read time: %f\n",(double)(pure_host + pure_pushdown));
    printf("[host & device parallel batch] read time: %f\n",(double)pure_read);
    printf("[host & device parallel batch] query time (including computation): %f\n", (double)(end_all - start_all));

    return result_count;
}

int spatio_temporal_count_query_without_pushdown_batch(struct simple_query_engine *engine, struct spatio_temporal_range_predicate *predicate) {

    struct index_entry_storage *index_storage = &engine->index_storage;
    struct traj_storage *data_storage = &engine->data_storage;
    struct seg_meta_section_entry_storage *meta_storage = &engine->seg_meta_storage;

    // calculate the matched block num
    int block_logical_addr_count = 0;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];

        block_logical_addr_count++;

    }

    if (block_logical_addr_count == 0) {
        return 0;
    }

    // get the match block id vector
    //int block_logical_addr_vec[block_logical_addr_count];   // TODO use malloc for large vector
    int *block_logical_addr_vec = malloc(block_logical_addr_count * sizeof(int));

    int addr_vec_index = 0;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];

        block_logical_addr_vec[addr_vec_index] = entry->block_logical_adr;
        addr_vec_index++;

    }


    printf("[host batch] read block num: %d\n", block_logical_addr_count);
    // run query
    int result_count = run_spatio_temporal_count_query_in_host_batch(predicate, data_storage, block_logical_addr_count,
                                                               block_logical_addr_vec);

    printf("selectivity: %f\n", (1.0 / 256.0));
    free(block_logical_addr_vec);
    return result_count;
}

int
run_spatio_temporal_count_query_in_host_batch(struct spatio_temporal_range_predicate *predicate, struct traj_storage *data_storage,
                                              int block_logical_addr_count, int *block_logical_addr_vec) {
    clock_t start, start_config, end;
    clock_t start_read, end_read, pure_read;
    clock_t start_computation, end_computation, pure_computation;
    pure_read = 0;
    pure_computation = 0;
    start_config = clock();
    int result_count = 0;
    int aggregated_block_vec_size = calculate_aggregate_block_vec_size(block_logical_addr_vec, block_logical_addr_count);
    struct continuous_block_meta aggregated_block_vec[aggregated_block_vec_size];
    aggregate_blocks(block_logical_addr_vec, block_logical_addr_count, aggregated_block_vec, aggregated_block_vec_size);

    fill_points_buffer(points_buffer_size);

    int batch_size = REQUEST_BATCH_SIZE;
    int batch_count  = 0;
    int current_block_start_vec[batch_size];
    int current_block_count_vec[batch_size];
    void* result_buffer_vec[batch_size];
    start = clock();

    void *count_result_buffer = malloc(4096);

    // split aggregated block if its size is larger than 256
    for (int i = 0; i < aggregated_block_vec_size; i++) {
        struct continuous_block_meta block_meta = aggregated_block_vec[i];
        int block_count = block_meta.block_count;
        int block_start = block_meta.block_start;
        //printf("block start: %d, block count: %d\n", block_start, block_count);
        for (int j = 0; j < block_count; j+=256) {
            int current_block_count = block_count - j > 256 ? 256 : block_count - j;
            int current_block_start = block_start + j;
            char *buffer = malloc(current_block_count * TRAJ_BLOCK_SIZE);
            memset(buffer, 0, current_block_count * TRAJ_BLOCK_SIZE);


            current_block_count_vec[batch_count] = current_block_count;
            current_block_start_vec[batch_count] = current_block_start;
            result_buffer_vec[batch_count] = buffer;

            batch_count++;
            if (batch_count % batch_size == 0) {
                start_read = clock();
                fetch_continuous_traj_data_block_spdk_batch(batch_size, data_storage, current_block_start_vec, current_block_count_vec, result_buffer_vec);
                end_read = clock();
                batch_count = 0;
                pure_read += (end_read - start_read);

                for (int m = 0; m < batch_size; m++) {
                    void* buffer_ptr = result_buffer_vec[m];
                    int block_count_value = current_block_count_vec[m];
                    for (int index = 0; index < block_count_value; index++) {
                        start_computation = clock();
                        int count = spatio_temporal_count_query_block_host_processing(
                                buffer_ptr + index * TRAJ_BLOCK_SIZE, count_result_buffer, predicate);
                        result_count += count;
                        end_computation = clock();
                        pure_computation += (end_computation - start_computation);
                    }
                }

                for (int k = 0; k < batch_size; k++) {
                    free(result_buffer_vec[k]);
                }

            }

        }

    }

    // handle the last batch
    start_read = clock();
    fetch_continuous_traj_data_block_spdk_batch(batch_count, data_storage, current_block_start_vec, current_block_count_vec, result_buffer_vec);
    end_read = clock();
    pure_read += (end_read - start_read);

    for (int m = 0; m < batch_count; m++) {
        void* buffer_ptr = result_buffer_vec[m];
        int block_count_value = current_block_count_vec[m];
        for (int index = 0; index < block_count_value; index++) {
            start_computation = clock();
            int count = spatio_temporal_count_query_block_host_processing(
                    buffer_ptr + index * TRAJ_BLOCK_SIZE, count_result_buffer, predicate);
            result_count += count;
            end_computation = clock();
            pure_computation += (end_computation - start_computation);
        }
    }

    for (int k = 0; k < batch_count; k++) {
        free(result_buffer_vec[k]);
    }

    end = clock();
    free_points_buffer(points_buffer_size);
    printf("[host batch] pure read time: %f\n",(double)(pure_read));
    printf("[host batch] pure computation time: %f\n",(double)(pure_computation));
    printf("[host batch] pure read and computation time: %f\n",(double)(pure_read + pure_computation));
    printf("[host batch] query time: %f\n",(double)(end-start_config));
    free(count_result_buffer);
    return result_count;
}



int spatio_temporal_query_without_pushdown_batch(struct simple_query_engine *engine, struct spatio_temporal_range_predicate *predicate, bool enable_host_index) {

    struct index_entry_storage *index_storage = &engine->index_storage;
    struct traj_storage *data_storage = &engine->data_storage;
    struct seg_meta_section_entry_storage *meta_storage = &engine->seg_meta_storage;

    // calculate the matched block num
    int block_logical_addr_count = 0;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        if (enable_host_index) {
            if (predicate->lon_min <= entry->lon_max
                && predicate->lon_max >= entry->lon_min
                && predicate->lat_min <= entry->lat_max
                && predicate->lat_max >= entry->lat_min
                && predicate->time_min <= entry->time_max
                && predicate->time_max >= entry->time_min) {
                block_logical_addr_count++;
            }
        } else {
            block_logical_addr_count++;
        }
    }

    if (block_logical_addr_count == 0) {
        return 0;
    }

    // get the match block id vector
    //int block_logical_addr_vec[block_logical_addr_count];   // TODO use malloc for large vector
    int *block_logical_addr_vec = malloc(block_logical_addr_count * sizeof(int));

    int addr_vec_index = 0;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        if (enable_host_index) {
            if (predicate->lon_min <= entry->lon_max
                && predicate->lon_max >= entry->lon_min
                && predicate->lat_min <= entry->lat_max
                && predicate->lat_max >= entry->lat_min
                && predicate->time_min <= entry->time_max
                && predicate->time_max >= entry->time_min) {
                block_logical_addr_vec[addr_vec_index] = entry->block_logical_adr;
                addr_vec_index++;
            }
        } else {
            block_logical_addr_vec[addr_vec_index] = entry->block_logical_adr;
            addr_vec_index++;
        }
    }


    printf("[host batch] read block num: %d\n", block_logical_addr_count);
    // run query
    int result_count = run_spatio_temporal_query_in_host_batch(predicate, data_storage, block_logical_addr_count,
                                                         block_logical_addr_vec);

    printf("selectivity: %f\n", (1.0 * result_count / (block_logical_addr_count * 243)));
    free(block_logical_addr_vec);
    return result_count;
}





int
run_spatio_temporal_query_in_host_batch(struct spatio_temporal_range_predicate *predicate, struct traj_storage *data_storage,
                                  int block_logical_addr_count, int *block_logical_addr_vec) {
    clock_t start, start_config, end;
    clock_t start_read, end_read, pure_read;
    clock_t start_computation, end_computation, pure_computation;
    pure_read = 0;
    pure_computation = 0;
    start_config = clock();
    int result_count = 0;
    int aggregated_block_vec_size = calculate_aggregate_block_vec_size(block_logical_addr_vec, block_logical_addr_count);
    struct continuous_block_meta aggregated_block_vec[aggregated_block_vec_size];
    aggregate_blocks(block_logical_addr_vec, block_logical_addr_count, aggregated_block_vec, aggregated_block_vec_size);

    fill_points_buffer(points_buffer_size);

    int batch_size = REQUEST_BATCH_SIZE;
    int batch_count  = 0;
    int current_block_start_vec[batch_size];
    int current_block_count_vec[batch_size];
    void* result_buffer_vec[batch_size];
    start = clock();

    // split aggregated block if its size is larger than 256
    for (int i = 0; i < aggregated_block_vec_size; i++) {
        struct continuous_block_meta block_meta = aggregated_block_vec[i];
        int block_count = block_meta.block_count;
        int block_start = block_meta.block_start;
        //printf("block start: %d, block count: %d\n", block_start, block_count);
        for (int j = 0; j < block_count; j+=256) {
            int current_block_count = block_count - j > 256 ? 256 : block_count - j;
            int current_block_start = block_start + j;
            char *buffer = malloc(current_block_count * TRAJ_BLOCK_SIZE);
            memset(buffer, 0, current_block_count * TRAJ_BLOCK_SIZE);


            current_block_count_vec[batch_count] = current_block_count;
            current_block_start_vec[batch_count] = current_block_start;
            result_buffer_vec[batch_count] = buffer;

            batch_count++;
            if (batch_count % batch_size == 0) {
                start_read = clock();
                fetch_continuous_traj_data_block_spdk_batch(batch_size, data_storage, current_block_start_vec, current_block_count_vec, result_buffer_vec);
                end_read = clock();
                batch_count = 0;
                pure_read += (end_read - start_read);

                for (int m = 0; m < batch_size; m++) {
                    void* buffer_ptr = result_buffer_vec[m];
                    int block_count_value = current_block_count_vec[m];
                    for (int index = 0; index < block_count_value; index++) {
                        start_computation = clock();
                        int count = spatio_temporal_query_raw_trajectory_block(
                                buffer_ptr + index * TRAJ_BLOCK_SIZE, predicate);
                        result_count += count;
                        end_computation = clock();
                        pure_computation += (end_computation - start_computation);
                    }
                }

                for (int k = 0; k < batch_size; k++) {
                    free(result_buffer_vec[k]);
                }

            }

        }

    }

    // handle the last batch
    start_read = clock();
    fetch_continuous_traj_data_block_spdk_batch(batch_count, data_storage, current_block_start_vec, current_block_count_vec, result_buffer_vec);
    end_read = clock();
    pure_read += (end_read - start_read);

    for (int m = 0; m < batch_count; m++) {
        void* buffer_ptr = result_buffer_vec[m];
        int block_count_value = current_block_count_vec[m];
        for (int index = 0; index < block_count_value; index++) {
            start_computation = clock();
            int count = spatio_temporal_query_raw_trajectory_block(
                    buffer_ptr + index * TRAJ_BLOCK_SIZE, predicate);
            result_count += count;
            end_computation = clock();
            pure_computation += (end_computation - start_computation);
        }
    }

    for (int k = 0; k < batch_count; k++) {
        free(result_buffer_vec[k]);
    }

    end = clock();
    free_points_buffer(points_buffer_size);
    printf("[host batch] pure read time: %f\n",(double)(pure_read));
    printf("[host batch] pure computation time: %f\n",(double)(pure_computation));
    printf("[host batch] pure read and computation time: %f\n",(double)(pure_read + pure_computation));
    printf("[host batch] query time: %f\n",(double)(end-start_config));
    return result_count;
}

int spatio_temporal_query_with_full_pushdown(struct simple_query_engine *engine, struct spatio_temporal_range_predicate *predicate, bool enable_host_index) {

    struct index_entry_storage *index_storage = &engine->index_storage;
    struct traj_storage *data_storage = &engine->data_storage;
    struct seg_meta_section_entry_storage *meta_storage = &engine->seg_meta_storage;

    clock_t index_start, index_end;

    // calculate the matched block num
    index_start = clock();
    int block_logical_addr_count = 0;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        if (enable_host_index) {
            if (predicate->lon_min <= entry->lon_max
                && predicate->lon_max >= entry->lon_min
                && predicate->lat_min <= entry->lat_max
                && predicate->lat_max >= entry->lat_min
                && predicate->time_min <= entry->time_max
                && predicate->time_max >= entry->time_min) {
                block_logical_addr_count++;
            }
        } else {
            block_logical_addr_count++;
        }
    }
    if (block_logical_addr_count == 0) {
        return 0;
    }

    // get the match block id vector
    // int block_logical_addr_vec[block_logical_addr_count];
    int *block_logical_addr_vec = malloc(block_logical_addr_count * sizeof(int));
    int addr_vec_index = 0;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        if (enable_host_index) {
            if (predicate->lon_min <= entry->lon_max
                && predicate->lon_max >= entry->lon_min
                && predicate->lat_min <= entry->lat_max
                && predicate->lat_max >= entry->lat_min
                && predicate->time_min <= entry->time_max
                && predicate->time_max >= entry->time_min) {
                block_logical_addr_vec[addr_vec_index] = entry->block_logical_adr;
                addr_vec_index++;
            }
        } else {
            block_logical_addr_vec[addr_vec_index] = entry->block_logical_adr;
            addr_vec_index++;
        }
    }
    index_end = clock();
    printf("[isp cpu] index lookup time: %f\n", (double)(index_end - index_start));

    printf("[isp cpu] read block num: %d\n", block_logical_addr_count);
    // run query
    int result_count = run_spatio_temporal_query_device(predicate, data_storage, meta_storage, block_logical_addr_count,
                                                        block_logical_addr_vec, enable_estimated_result_size);
    free(block_logical_addr_vec);
    return result_count;
}

int spatio_temporal_count_query_with_pushdown_batch_naive(struct simple_query_engine *engine, struct spatio_temporal_range_predicate *predicate) {

    struct index_entry_storage *index_storage = &engine->index_storage;
    struct traj_storage *data_storage = &engine->data_storage;
    struct seg_meta_section_entry_storage *meta_storage = &engine->seg_meta_storage;

    // calculate the matched block num
    int block_logical_addr_count = 0;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        block_logical_addr_count++;

    }
    if (block_logical_addr_count == 0) {
        return 0;
    }

    // get the match block id vector
    // int block_logical_addr_vec[block_logical_addr_count];
    int *block_logical_addr_vec = malloc(block_logical_addr_count * sizeof(int));
    int addr_vec_index = 0;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        block_logical_addr_vec[addr_vec_index] = entry->block_logical_adr;
        addr_vec_index++;

    }
    printf("[isp cpu batch naive] read block num: %d\n", block_logical_addr_count);
    // run query
    int result_count = run_spatio_temporal_count_query_device_batch_naive(predicate, data_storage, block_logical_addr_count,
                                                                    block_logical_addr_vec);
    free(block_logical_addr_vec);
    return result_count;
}

int spatio_temporal_count_query_with_pushdown_batch(struct simple_query_engine *engine, struct spatio_temporal_range_predicate *predicate) {

    struct index_entry_storage *index_storage = &engine->index_storage;
    struct traj_storage *data_storage = &engine->data_storage;
    struct seg_meta_section_entry_storage *meta_storage = &engine->seg_meta_storage;

    // calculate the matched block num
    int block_logical_addr_count = 0;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        block_logical_addr_count++;

    }
    if (block_logical_addr_count == 0) {
        return 0;
    }

    // get the match block id vector
    // int block_logical_addr_vec[block_logical_addr_count];
    int *block_logical_addr_vec = malloc(block_logical_addr_count * sizeof(int));
    int addr_vec_index = 0;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        block_logical_addr_vec[addr_vec_index] = entry->block_logical_adr;
        addr_vec_index++;

    }
    printf("[isp cpu batch] read block num: %d\n", block_logical_addr_count);
    // run query
    int result_count = run_spatio_temporal_count_query_device_batch(predicate, data_storage, block_logical_addr_count,
                                                              block_logical_addr_vec);
    free(block_logical_addr_vec);
    return result_count;
}

int spatio_temporal_query_with_full_pushdown_batch_naive(struct simple_query_engine *engine, struct spatio_temporal_range_predicate *predicate, bool enable_host_index) {

    struct index_entry_storage *index_storage = &engine->index_storage;
    struct traj_storage *data_storage = &engine->data_storage;
    struct seg_meta_section_entry_storage *meta_storage = &engine->seg_meta_storage;

    // calculate the matched block num
    int block_logical_addr_count = 0;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        if (enable_host_index) {
            if (predicate->lon_min <= entry->lon_max
                && predicate->lon_max >= entry->lon_min
                && predicate->lat_min <= entry->lat_max
                && predicate->lat_max >= entry->lat_min
                && predicate->time_min <= entry->time_max
                && predicate->time_max >= entry->time_min) {
                block_logical_addr_count++;
            }
        } else {
            block_logical_addr_count++;
        }
    }
    if (block_logical_addr_count == 0) {
        return 0;
    }

    // get the match block id vector
    // int block_logical_addr_vec[block_logical_addr_count];
    int *block_logical_addr_vec = malloc(block_logical_addr_count * sizeof(int));
    int addr_vec_index = 0;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        if (enable_host_index) {
            if (predicate->lon_min <= entry->lon_max
                && predicate->lon_max >= entry->lon_min
                && predicate->lat_min <= entry->lat_max
                && predicate->lat_max >= entry->lat_min
                && predicate->time_min <= entry->time_max
                && predicate->time_max >= entry->time_min) {
                block_logical_addr_vec[addr_vec_index] = entry->block_logical_adr;
                addr_vec_index++;
            }
        } else {
            block_logical_addr_vec[addr_vec_index] = entry->block_logical_adr;
            addr_vec_index++;
        }
    }
    printf("[isp cpu batch naive] read block num: %d\n", block_logical_addr_count);
    // run query
    int result_count = run_spatio_temporal_query_device_batch_naive(predicate, data_storage, meta_storage, block_logical_addr_count,
                                                              block_logical_addr_vec, enable_estimated_result_size);
    free(block_logical_addr_vec);
    return result_count;
}

int spatio_temporal_query_with_full_pushdown_batch(struct simple_query_engine *engine, struct spatio_temporal_range_predicate *predicate, bool enable_host_index) {

    struct index_entry_storage *index_storage = &engine->index_storage;
    struct traj_storage *data_storage = &engine->data_storage;
    struct seg_meta_section_entry_storage *meta_storage = &engine->seg_meta_storage;

    // calculate the matched block num
    int block_logical_addr_count = 0;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        if (enable_host_index) {
            if (predicate->lon_min <= entry->lon_max
                && predicate->lon_max >= entry->lon_min
                && predicate->lat_min <= entry->lat_max
                && predicate->lat_max >= entry->lat_min
                && predicate->time_min <= entry->time_max
                && predicate->time_max >= entry->time_min) {
                block_logical_addr_count++;
            }
        } else {
            block_logical_addr_count++;
        }
    }
    if (block_logical_addr_count == 0) {
        return 0;
    }

    // get the match block id vector
    // int block_logical_addr_vec[block_logical_addr_count];
    int *block_logical_addr_vec = malloc(block_logical_addr_count * sizeof(int));
    int addr_vec_index = 0;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        if (enable_host_index) {
            if (predicate->lon_min <= entry->lon_max
                && predicate->lon_max >= entry->lon_min
                && predicate->lat_min <= entry->lat_max
                && predicate->lat_max >= entry->lat_min
                && predicate->time_min <= entry->time_max
                && predicate->time_max >= entry->time_min) {
                block_logical_addr_vec[addr_vec_index] = entry->block_logical_adr;
                addr_vec_index++;
            }
        } else {
            block_logical_addr_vec[addr_vec_index] = entry->block_logical_adr;
            addr_vec_index++;
        }
    }
    printf("[isp cpu batch] read block num: %d\n", block_logical_addr_count);
    // run query
    int result_count = run_spatio_temporal_query_device_batch(predicate, data_storage, meta_storage, block_logical_addr_count,
                                                        block_logical_addr_vec, enable_estimated_result_size);
    free(block_logical_addr_vec);
    return result_count;
}

int spatio_temporal_query_with_full_pushdown_fpga(struct simple_query_engine *engine, struct spatio_temporal_range_predicate *predicate, bool enable_host_index) {

    struct index_entry_storage *index_storage = &engine->index_storage;
    struct traj_storage *data_storage = &engine->data_storage;
    struct seg_meta_section_entry_storage *meta_storage = &engine->seg_meta_storage;

    clock_t index_start, index_end;
    // calculate the matched block num
    index_start = clock();
    int block_logical_addr_count = 0;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        if (enable_host_index) {
            if (predicate->lon_min <= entry->lon_max
                && predicate->lon_max >= entry->lon_min
                && predicate->lat_min <= entry->lat_max
                && predicate->lat_max >= entry->lat_min
                && predicate->time_min <= entry->time_max
                && predicate->time_max >= entry->time_min) {
                block_logical_addr_count++;
            }
        } else {
            block_logical_addr_count++;
        }
    }
    if (block_logical_addr_count == 0) {
        return 0;
    }

    // get the match block id vector
    // int block_logical_addr_vec[block_logical_addr_count];
    int *block_logical_addr_vec = malloc(block_logical_addr_count * sizeof(int));
    int addr_vec_index = 0;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        if (enable_host_index) {
            if (predicate->lon_min <= entry->lon_max
                && predicate->lon_max >= entry->lon_min
                && predicate->lat_min <= entry->lat_max
                && predicate->lat_max >= entry->lat_min
                && predicate->time_min <= entry->time_max
                && predicate->time_max >= entry->time_min) {
                block_logical_addr_vec[addr_vec_index] = entry->block_logical_adr;
                addr_vec_index++;
            }
        } else {
            block_logical_addr_vec[addr_vec_index] = entry->block_logical_adr;
            addr_vec_index++;
        }
    }
    index_end = clock();

    printf("[isp fpga] index lookup time: %f\n", (double)(index_end - index_start));
    printf("[isp fpga] read block num: %d\n", block_logical_addr_count);
    // run query
    int result_count = run_spatio_temporal_query_device_fpga(predicate, data_storage, meta_storage, block_logical_addr_count,
                                                        block_logical_addr_vec, enable_estimated_result_size);
    free(block_logical_addr_vec);
    return result_count;
}

int spatio_temporal_query_with_full_pushdown_fpga_batch(struct simple_query_engine *engine, struct spatio_temporal_range_predicate *predicate, bool enable_host_index) {

    struct index_entry_storage *index_storage = &engine->index_storage;
    struct traj_storage *data_storage = &engine->data_storage;
    struct seg_meta_section_entry_storage *meta_storage = &engine->seg_meta_storage;

    // calculate the matched block num
    int block_logical_addr_count = 0;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        if (enable_host_index) {
            if (predicate->lon_min <= entry->lon_max
                && predicate->lon_max >= entry->lon_min
                && predicate->lat_min <= entry->lat_max
                && predicate->lat_max >= entry->lat_min
                && predicate->time_min <= entry->time_max
                && predicate->time_max >= entry->time_min) {
                block_logical_addr_count++;
            }
        } else {
            block_logical_addr_count++;
        }
    }
    if (block_logical_addr_count == 0) {
        return 0;
    }

    // get the match block id vector
    // int block_logical_addr_vec[block_logical_addr_count];
    int *block_logical_addr_vec = malloc(block_logical_addr_count * sizeof(int));
    int addr_vec_index = 0;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        if (enable_host_index) {
            if (predicate->lon_min <= entry->lon_max
                && predicate->lon_max >= entry->lon_min
                && predicate->lat_min <= entry->lat_max
                && predicate->lat_max >= entry->lat_min
                && predicate->time_min <= entry->time_max
                && predicate->time_max >= entry->time_min) {
                block_logical_addr_vec[addr_vec_index] = entry->block_logical_adr;
                addr_vec_index++;
            }
        } else {
            block_logical_addr_vec[addr_vec_index] = entry->block_logical_adr;
            addr_vec_index++;
        }
    }

    printf("[isp fpga] read block num: %d\n", block_logical_addr_count);
    // run query
    int result_count = run_spatio_temporal_query_device_fpga_batch(predicate, data_storage, meta_storage, block_logical_addr_count,
                                                             block_logical_addr_vec, enable_estimated_result_size);
    free(block_logical_addr_vec);
    return result_count;
}

int
run_spatio_temporal_query_device(struct spatio_temporal_range_predicate *predicate, struct traj_storage *data_storage,
                                 struct seg_meta_section_entry_storage *meta_storage, int block_logical_addr_count,
                                 int *block_logical_addr_vec, bool enable_result_estimation) {
    int result_count = 0;

    clock_t start, end, pure_read;
    pure_read = 0;
    clock_t start_all, end_all;
    clock_t estimation_start, estimation_end, pure_estimation;
    pure_estimation = 0;

    int total_result_block_num = 0;
    start_all = clock();
    fill_points_buffer(points_buffer_size);
    for (int i = 0; i < block_logical_addr_count; i += 256) {
        int block_addr_vec_size = block_logical_addr_count - i > 256 ? 256 : block_logical_addr_count - i;
        int estimated_result_size = block_addr_vec_size * 0x1000;
        if (enable_result_estimation) {
            estimation_start = clock();
            estimated_result_size = estimate_spatio_temporal_result_size_for_blocks(meta_storage, predicate,
                                                                                    &block_logical_addr_vec[i],
                                                                                    block_addr_vec_size);
            estimation_end = clock();
            pure_estimation += estimation_end - estimation_start;
        }
        //printf("estimated result size: %d\n", estimated_result_size);
        if (estimated_result_size == 0) {
            continue;
        }
        if (estimated_result_size % 0x1000 != 0) {
            estimated_result_size = ((estimated_result_size / 0x1000) + 1) * 0x1000;
        }
        int estimated_result_block_num = estimated_result_size / 0x1000;
        //int estimated_result_block_num = 1;
        total_result_block_num += estimated_result_block_num;

        void* result_buffer = malloc(estimated_result_size);

        int block_meta_vec_size = calculate_aggregate_block_vec_size(&block_logical_addr_vec[i], block_addr_vec_size);
        struct continuous_block_meta block_meta_vec[block_meta_vec_size];
        aggregate_blocks(&block_logical_addr_vec[i], block_addr_vec_size, block_meta_vec, block_meta_vec_size);

        struct isp_descriptor isp_desc;
        struct lba lba_vec[block_meta_vec_size];
        assemble_lba_vec_via_block_meta(lba_vec, DATA_FILE_OFFSET, block_meta_vec, block_meta_vec_size);
        assemble_isp_desc_for_spatial_temporal(&isp_desc, predicate, estimated_result_block_num, lba_vec, block_meta_vec_size);

        start = clock();
        do_isp_for_trajectory_data(data_storage, result_buffer, estimated_result_size, &isp_desc, 0);
        end = clock();
        pure_read += (end - start);

        /*for (int block_count = 0; block_count < estimated_result_block_num; block_count++) {
            int count = spatio_temporal_query_isp_output_block(result_buffer + block_count * 4096, predicate);
            result_count += count;
        }*/
        result_count += spatio_temporal_query_isp_new_format_output_blocks(result_buffer, predicate);

        free(result_buffer);

        //printf("pure read time [isp]: %f\n",(double)(end-start));
    }
    free_points_buffer(points_buffer_size);
    end_all = clock();
    printf("[isp cpu] estimated result block number: %d\n", total_result_block_num);
    printf("[isp cpu] pure read time: %f\n",(double)pure_read);
    printf("[isp cpu] pure estimation time: %f\n", (double)pure_estimation);
    printf("[isp cpu] pure read time and estimation: %f\n",(double)(pure_read + pure_estimation));
    printf("[isp cpu] query time: %f\n", (double)(end_all - start_all));


    return result_count;
}

int
run_spatio_temporal_query_device_fpga(struct spatio_temporal_range_predicate *predicate, struct traj_storage *data_storage,
                                 struct seg_meta_section_entry_storage *meta_storage, int block_logical_addr_count,
                                 int *block_logical_addr_vec, bool enable_result_estimation) {
    int result_count = 0;

    clock_t start, end, pure_read, estimation_start, estimation_end, pure_estimation;
    pure_read = 0;
    pure_estimation = 0;
    clock_t start_all, end_all;
    int total_result_block_num = 0;
    start_all = clock();
    fill_points_buffer(points_buffer_size);
    for (int i = 0; i < block_logical_addr_count; i += 256) {
        int block_addr_vec_size = block_logical_addr_count - i > 256 ? 256 : block_logical_addr_count - i;
        int estimated_result_size = block_addr_vec_size * 0x1000;
        if (enable_result_estimation) {
            estimation_start = clock();
            estimated_result_size = estimate_spatio_temporal_result_size_for_blocks(meta_storage, predicate,
                                                                                    &block_logical_addr_vec[i],
                                                                                    block_addr_vec_size);
            estimation_end = clock();
            pure_estimation += estimation_end - estimation_start;
        }
        //printf("estimated result size: %d\n", estimated_result_size);
        if (estimated_result_size == 0) {
            continue;
        }
        if (estimated_result_size % 0x1000 != 0) {
            estimated_result_size = ((estimated_result_size / 0x1000) + 1) * 0x1000;
        }
        int estimated_result_block_num = estimated_result_size / 0x1000;
        //int estimated_result_block_num = 1;
        total_result_block_num += estimated_result_block_num;


        void* result_buffer = malloc(estimated_result_size);

        int block_meta_vec_size = calculate_aggregate_block_vec_size(&block_logical_addr_vec[i], block_addr_vec_size);
        struct continuous_block_meta block_meta_vec[block_meta_vec_size];
        aggregate_blocks(&block_logical_addr_vec[i], block_addr_vec_size, block_meta_vec, block_meta_vec_size);

        struct isp_descriptor isp_desc;
        struct lba lba_vec[block_meta_vec_size];
        assemble_lba_vec_via_block_meta(lba_vec, DATA_FILE_OFFSET, block_meta_vec, block_meta_vec_size);
        assemble_isp_desc_for_spatial_temporal(&isp_desc, predicate, estimated_result_block_num, lba_vec, block_meta_vec_size);

        start = clock();
        do_isp_for_trajectory_data(data_storage, result_buffer, estimated_result_size, &isp_desc, 1);
        end = clock();
        pure_read += (end - start);
        result_count += spatio_temporal_query_isp_fpga_output_blocks(result_buffer, predicate);

        free(result_buffer);


    }
    free_points_buffer(points_buffer_size);
    end_all = clock();
    printf("[isp fpga] estimated result block number: %d\n", total_result_block_num);
    printf("[isp fpga] pure read time: %f\n",(double)pure_read);
    printf("[isp fpga] pure estimation time: %f\n",(double)pure_estimation);
    printf("[isp fpga] pure estimation + pure read: %f\n", (double)(pure_estimation + pure_read));
    printf("[isp fpga] query time: %f\n", (double)(end_all - start_all));

    return result_count;
}

int run_spatio_temporal_count_query_device_batch_naive(struct spatio_temporal_range_predicate *predicate, struct traj_storage *data_storage,
                                                 int block_logical_addr_count, int *block_logical_addr_vec) {
    int result_count = 0;
    int total_batch_func_num = 0;

    clock_t start, end, pure_read;
    pure_read = 0;
    clock_t start_all, end_all;
    int total_result_block_num = 0;
    start_all = clock();
    fill_points_buffer(points_buffer_size);

    // prepare batch parameters
    int batch_size = REQUEST_BATCH_SIZE; //  fix bug here -> fixed: caused by the same request used in the batch using the same lba_array (before, we do not use malloc for lba_array)
    void* result_buffer_vec[batch_size];
    size_t result_buffer_size_vec[batch_size];
    struct isp_descriptor *isp_desc_vec[batch_size];
    struct lba *lba_vec_ptr[batch_size];
    int batch_count = 0;

    void *count_result_buf = malloc(4096);

    for (int i = 0; i < block_logical_addr_count; i += 256) {

        int block_addr_vec_size = block_logical_addr_count - i > 256 ? 256 : block_logical_addr_count - i;
        int estimated_result_size =  0x1000;
        int estimated_result_block_num = 1;

        total_result_block_num += 1;

        void* result_buffer = malloc(estimated_result_size);

        int block_meta_vec_size = calculate_aggregate_block_vec_size(&block_logical_addr_vec[i], block_addr_vec_size);
        struct continuous_block_meta block_meta_vec[block_meta_vec_size];
        aggregate_blocks(&block_logical_addr_vec[i], block_addr_vec_size, block_meta_vec, block_meta_vec_size);

        struct isp_descriptor *isp_desc = malloc(sizeof(struct isp_descriptor));
        struct lba *lba_vec = malloc(sizeof(struct lba) * block_meta_vec_size);
        assemble_lba_vec_via_block_meta(lba_vec, DATA_FILE_OFFSET, block_meta_vec, block_meta_vec_size);
        assemble_isp_desc_for_spatial_temporal_count_naive(isp_desc, predicate, estimated_result_block_num, lba_vec, block_meta_vec_size);

        //start = clock();
        result_buffer_vec[batch_count] = result_buffer;
        result_buffer_size_vec[batch_count] = estimated_result_size;
        isp_desc_vec[batch_count] = isp_desc;
        lba_vec_ptr[batch_count] = lba_vec;


        batch_count++;
        if (batch_count % batch_size == 0) {
            start = clock();
            do_isp_for_trajectory_data_batch(batch_size, data_storage, result_buffer_vec, result_buffer_size_vec, isp_desc_vec, 0);
            end = clock();
            /*for (int o = 0; o < batch_size; o++) {
                print_isp_descriptor(isp_desc_vec[o]);
            }
            printf("each function call time:%d\n",(end - start));*/
            total_batch_func_num++;
            for (int k = 0; k < batch_size; k++) {
                void* batch_base = result_buffer_vec[k];

                result_count += spatio_temporal_count_query_block_isp_processing(batch_base, count_result_buf, predicate);
            }
            batch_count = 0;

            pure_read += (end - start);
            for (int m = 0; m < batch_size; m++) {
                free(result_buffer_vec[m]);
                free(isp_desc_vec[m]);
                free(lba_vec_ptr[m]);
            }

        }

    }
    // handle the last batch (less than batch size)
    start = clock();
    do_isp_for_trajectory_data_batch(batch_count, data_storage, result_buffer_vec, result_buffer_size_vec, isp_desc_vec, 0);
    end = clock();
    total_batch_func_num++;
    for (int k = 0; k < batch_count; k++) {
        void* batch_base = result_buffer_vec[k];

        result_count += spatio_temporal_count_query_block_isp_processing(batch_base, count_result_buf, predicate);
    }

    pure_read += (end - start);
    for (int m = 0; m < batch_count; m++) {
        free(result_buffer_vec[m]);
        free(isp_desc_vec[m]);
        free(lba_vec_ptr[m]);
    }


    free_points_buffer(points_buffer_size);
    end_all = clock();
    printf("batch function call num: %d\n", total_batch_func_num);
    printf("[isp cpu batch naive] estimated result block number: %d\n", total_result_block_num);
    printf("[isp cpu batch naive] pure read time: %f\n",(double)pure_read);
    printf("[isp cpu batch naive] query time (including estimation): %f\n", (double)(end_all - start_all));
    free(count_result_buf);

    return result_count;

}

int run_spatio_temporal_count_query_device_batch(struct spatio_temporal_range_predicate *predicate, struct traj_storage *data_storage,
                                                 int block_logical_addr_count, int *block_logical_addr_vec) {
    int result_count = 0;
    int total_batch_func_num = 0;

    clock_t start, end, pure_read;
    pure_read = 0;
    clock_t start_all, end_all;
    int total_result_block_num = 0;
    start_all = clock();
    fill_points_buffer(points_buffer_size);

    // prepare batch parameters
    int batch_size = REQUEST_BATCH_SIZE; //  fix bug here -> fixed: caused by the same request used in the batch using the same lba_array (before, we do not use malloc for lba_array)
    void* result_buffer_vec[batch_size];
    size_t result_buffer_size_vec[batch_size];
    struct isp_descriptor *isp_desc_vec[batch_size];
    struct lba *lba_vec_ptr[batch_size];
    int batch_count = 0;

    void *count_result_buf = malloc(4096);

    for (int i = 0; i < block_logical_addr_count; i += 256) {

        int block_addr_vec_size = block_logical_addr_count - i > 256 ? 256 : block_logical_addr_count - i;
        int estimated_result_size =  0x1000;
        int estimated_result_block_num = 1;

        total_result_block_num += 1;

        void* result_buffer = malloc(estimated_result_size);

        int block_meta_vec_size = calculate_aggregate_block_vec_size(&block_logical_addr_vec[i], block_addr_vec_size);
        struct continuous_block_meta block_meta_vec[block_meta_vec_size];
        aggregate_blocks(&block_logical_addr_vec[i], block_addr_vec_size, block_meta_vec, block_meta_vec_size);

        struct isp_descriptor *isp_desc = malloc(sizeof(struct isp_descriptor));
        struct lba *lba_vec = malloc(sizeof(struct lba) * block_meta_vec_size);
        assemble_lba_vec_via_block_meta(lba_vec, DATA_FILE_OFFSET, block_meta_vec, block_meta_vec_size);
        assemble_isp_desc_for_spatial_temporal_count(isp_desc, predicate, estimated_result_block_num, lba_vec, block_meta_vec_size);

        //start = clock();
        result_buffer_vec[batch_count] = result_buffer;
        result_buffer_size_vec[batch_count] = estimated_result_size;
        isp_desc_vec[batch_count] = isp_desc;
        lba_vec_ptr[batch_count] = lba_vec;


        batch_count++;
        if (batch_count % batch_size == 0) {
            start = clock();
            do_isp_for_trajectory_data_batch(batch_size, data_storage, result_buffer_vec, result_buffer_size_vec, isp_desc_vec, 0);
            end = clock();
            /*for (int o = 0; o < batch_size; o++) {
                print_isp_descriptor(isp_desc_vec[o]);
            }
            printf("each function call time:%d\n",(end - start));*/
            total_batch_func_num++;
            for (int k = 0; k < batch_size; k++) {
                void* batch_base = result_buffer_vec[k];

                result_count += spatio_temporal_count_query_block_isp_processing(batch_base, count_result_buf, predicate);
            }
            batch_count = 0;

            pure_read += (end - start);
            for (int m = 0; m < batch_size; m++) {
                free(result_buffer_vec[m]);
                free(isp_desc_vec[m]);
                free(lba_vec_ptr[m]);
            }

        }

    }
    // handle the last batch (less than batch size)
    start = clock();
    do_isp_for_trajectory_data_batch(batch_count, data_storage, result_buffer_vec, result_buffer_size_vec, isp_desc_vec, 0);
    end = clock();
    total_batch_func_num++;
    for (int k = 0; k < batch_count; k++) {
        void* batch_base = result_buffer_vec[k];

        result_count += spatio_temporal_count_query_block_isp_processing(batch_base, count_result_buf, predicate);
    }

    pure_read += (end - start);
    for (int m = 0; m < batch_count; m++) {
        free(result_buffer_vec[m]);
        free(isp_desc_vec[m]);
        free(lba_vec_ptr[m]);
    }


    free_points_buffer(points_buffer_size);
    end_all = clock();
    printf("batch function call num: %d\n", total_batch_func_num);
    printf("[isp cpu batch] estimated result block number: %d\n", total_result_block_num);
    printf("[isp cpu batch] pure read time: %f\n",(double)pure_read);
    printf("[isp cpu batch] query time (including estimation): %f\n", (double)(end_all - start_all));
    free(count_result_buf);

    return result_count;

}

int
run_spatio_temporal_query_device_batch_naive(struct spatio_temporal_range_predicate *predicate, struct traj_storage *data_storage,
                                       struct seg_meta_section_entry_storage *meta_storage, int block_logical_addr_count,
                                       int *block_logical_addr_vec, bool enable_result_estimation) {
    int result_count = 0;
    int total_batch_func_num = 0;

    clock_t start, end, pure_read;
    pure_read = 0;
    clock_t start_all, end_all;
    int total_result_block_num = 0;
    start_all = clock();
    fill_points_buffer(points_buffer_size);

    // prepare batch parameters
    int batch_size = REQUEST_BATCH_SIZE; //  fix bug here -> fixed: caused by the same request used in the batch using the same lba_array (before, we do not use malloc for lba_array)
    void* result_buffer_vec[batch_size];
    size_t result_buffer_size_vec[batch_size];
    struct isp_descriptor *isp_desc_vec[batch_size];
    struct lba *lba_vec_ptr[batch_size];
    int batch_count = 0;

    for (int i = 0; i < block_logical_addr_count; i += 256) {

        int block_addr_vec_size = block_logical_addr_count - i > 256 ? 256 : block_logical_addr_count - i;
        int estimated_result_size = block_addr_vec_size * 0x1000;
        if (enable_result_estimation) {
            estimated_result_size = estimate_spatio_temporal_result_size_for_blocks(meta_storage, predicate,
                                                                                    &block_logical_addr_vec[i],
                                                                                    block_addr_vec_size);
        }
        //printf("estimated result size: %d\n", estimated_result_size);
        if (estimated_result_size == 0) {
            continue;
        }
        if (estimated_result_size % 0x1000 != 0) {
            estimated_result_size = ((estimated_result_size / 0x1000) + 1) * 0x1000;
        }
        int estimated_result_block_num = estimated_result_size / 0x1000;
        //int estimated_result_block_num = 1;
        total_result_block_num += estimated_result_block_num;


        void* result_buffer = malloc(estimated_result_size);

        int block_meta_vec_size = calculate_aggregate_block_vec_size(&block_logical_addr_vec[i], block_addr_vec_size);
        struct continuous_block_meta block_meta_vec[block_meta_vec_size];
        aggregate_blocks(&block_logical_addr_vec[i], block_addr_vec_size, block_meta_vec, block_meta_vec_size);

        struct isp_descriptor *isp_desc = malloc(sizeof(struct isp_descriptor));
        struct lba *lba_vec = malloc(sizeof(struct lba) * block_meta_vec_size);
        assemble_lba_vec_via_block_meta(lba_vec, DATA_FILE_OFFSET, block_meta_vec, block_meta_vec_size);
        assemble_isp_desc_for_spatial_temporal_naive(isp_desc, predicate, estimated_result_block_num, lba_vec, block_meta_vec_size);

        //start = clock();
        result_buffer_vec[batch_count] = result_buffer;
        result_buffer_size_vec[batch_count] = estimated_result_size;
        isp_desc_vec[batch_count] = isp_desc;
        lba_vec_ptr[batch_count] = lba_vec;


        batch_count++;
        if (batch_count % batch_size == 0) {
            start = clock();
            do_isp_for_trajectory_data_batch(batch_size, data_storage, result_buffer_vec, result_buffer_size_vec, isp_desc_vec, 0);
            end = clock();
            /*for (int o = 0; o < batch_size; o++) {
                print_isp_descriptor(isp_desc_vec[o]);
            }
            printf("each function call time:%d\n",(end - start));*/
            total_batch_func_num++;
            for (int k = 0; k < batch_size; k++) {
                void* batch_base = result_buffer_vec[k];

                result_count += spatio_temporal_query_isp_new_format_output_blocks(batch_base, predicate);
            }
            batch_count = 0;

            pure_read += (end - start);
            for (int m = 0; m < batch_size; m++) {
                free(result_buffer_vec[m]);
                free(isp_desc_vec[m]);
                free(lba_vec_ptr[m]);
            }

        }

    }
    // handle the last batch (less than batch size)
    start = clock();
    do_isp_for_trajectory_data_batch(batch_count, data_storage, result_buffer_vec, result_buffer_size_vec, isp_desc_vec, 0);
    end = clock();
    total_batch_func_num++;
    for (int k = 0; k < batch_count; k++) {
        void* batch_base = result_buffer_vec[k];
        /*size_t batch_buffer_count = result_buffer_size_vec[k] / 0x1000;
        for (int j = 0; j < batch_buffer_count; j++) {
            void* block_ptr = batch_base + j * TRAJ_BLOCK_SIZE;
            result_count += spatio_temporal_query_isp_output_block(block_ptr, predicate);
            //print_isp_descriptor(isp_desc_vec[k]);
        }*/
        result_count += spatio_temporal_query_isp_new_format_output_blocks(batch_base, predicate);
    }

    pure_read += (end - start);
    for (int m = 0; m < batch_count; m++) {
        free(result_buffer_vec[m]);
        free(isp_desc_vec[m]);
        free(lba_vec_ptr[m]);
    }


    free_points_buffer(points_buffer_size);
    end_all = clock();
    printf("batch function call num: %d\n", total_batch_func_num);
    printf("[isp cpu batch naive] estimated result block number: %d\n", total_result_block_num);
    printf("[isp cpu batch naive] pure read time: %f\n",(double)pure_read);
    printf("[isp cpu batch naive] query time (including estimation): %f\n", (double)(end_all - start_all));

    return result_count;
}


int
run_spatio_temporal_query_device_batch(struct spatio_temporal_range_predicate *predicate, struct traj_storage *data_storage,
                                            struct seg_meta_section_entry_storage *meta_storage, int block_logical_addr_count,
                                            int *block_logical_addr_vec, bool enable_result_estimation) {
    int result_count = 0;
    int total_batch_func_num = 0;

    clock_t start, end, pure_read;
    pure_read = 0;
    clock_t start_all, end_all;
    int total_result_block_num = 0;
    start_all = clock();
    fill_points_buffer(points_buffer_size);

    // prepare batch parameters
    int batch_size = REQUEST_BATCH_SIZE; //  fix bug here -> fixed: caused by the same request used in the batch using the same lba_array (before, we do not use malloc for lba_array)
    void* result_buffer_vec[batch_size];
    size_t result_buffer_size_vec[batch_size];
    struct isp_descriptor *isp_desc_vec[batch_size];
    struct lba *lba_vec_ptr[batch_size];
    int batch_count = 0;

    for (int i = 0; i < block_logical_addr_count; i += 256) {

        int block_addr_vec_size = block_logical_addr_count - i > 256 ? 256 : block_logical_addr_count - i;
        int estimated_result_size = block_addr_vec_size * 0x1000;
        if (enable_result_estimation) {
            estimated_result_size = estimate_spatio_temporal_result_size_for_blocks(meta_storage, predicate,
                                                                                    &block_logical_addr_vec[i],
                                                                                    block_addr_vec_size);
        }
        //printf("estimated result size: %d\n", estimated_result_size);
        if (estimated_result_size == 0) {
            continue;
        }
        if (estimated_result_size % 0x1000 != 0) {
            estimated_result_size = ((estimated_result_size / 0x1000) + 1) * 0x1000;
        }
        int estimated_result_block_num = estimated_result_size / 0x1000;
        //int estimated_result_block_num = 1;
        total_result_block_num += estimated_result_block_num;


        void* result_buffer = malloc(estimated_result_size);

        int block_meta_vec_size = calculate_aggregate_block_vec_size(&block_logical_addr_vec[i], block_addr_vec_size);
        struct continuous_block_meta block_meta_vec[block_meta_vec_size];
        aggregate_blocks(&block_logical_addr_vec[i], block_addr_vec_size, block_meta_vec, block_meta_vec_size);

        struct isp_descriptor *isp_desc = malloc(sizeof(struct isp_descriptor));
        struct lba *lba_vec = malloc(sizeof(struct lba) * block_meta_vec_size);
        assemble_lba_vec_via_block_meta(lba_vec, DATA_FILE_OFFSET, block_meta_vec, block_meta_vec_size);
        assemble_isp_desc_for_spatial_temporal(isp_desc, predicate, estimated_result_block_num, lba_vec, block_meta_vec_size);

        //start = clock();
        result_buffer_vec[batch_count] = result_buffer;
        result_buffer_size_vec[batch_count] = estimated_result_size;
        isp_desc_vec[batch_count] = isp_desc;
        lba_vec_ptr[batch_count] = lba_vec;


        batch_count++;
        if (batch_count % batch_size == 0) {
            start = clock();
            do_isp_for_trajectory_data_batch(batch_size, data_storage, result_buffer_vec, result_buffer_size_vec, isp_desc_vec, 0);
            end = clock();
            /*for (int o = 0; o < batch_size; o++) {
                print_isp_descriptor(isp_desc_vec[o]);
            }
            printf("each function call time:%d\n",(end - start));*/
            total_batch_func_num++;
            for (int k = 0; k < batch_size; k++) {
                void* batch_base = result_buffer_vec[k];

                result_count += spatio_temporal_query_isp_new_format_output_blocks(batch_base, predicate);
            }
            batch_count = 0;

            pure_read += (end - start);
            for (int m = 0; m < batch_size; m++) {
                free(result_buffer_vec[m]);
                free(isp_desc_vec[m]);
                free(lba_vec_ptr[m]);
            }

        }

    }
    // handle the last batch (less than batch size)
    start = clock();
    do_isp_for_trajectory_data_batch(batch_count, data_storage, result_buffer_vec, result_buffer_size_vec, isp_desc_vec, 0);
    end = clock();
    total_batch_func_num++;
    for (int k = 0; k < batch_count; k++) {
        void* batch_base = result_buffer_vec[k];
        /*size_t batch_buffer_count = result_buffer_size_vec[k] / 0x1000;
        for (int j = 0; j < batch_buffer_count; j++) {
            void* block_ptr = batch_base + j * TRAJ_BLOCK_SIZE;
            result_count += spatio_temporal_query_isp_output_block(block_ptr, predicate);
            //print_isp_descriptor(isp_desc_vec[k]);
        }*/
        result_count += spatio_temporal_query_isp_new_format_output_blocks(batch_base, predicate);
    }

    pure_read += (end - start);
    for (int m = 0; m < batch_count; m++) {
        free(result_buffer_vec[m]);
        free(isp_desc_vec[m]);
        free(lba_vec_ptr[m]);
    }


    free_points_buffer(points_buffer_size);
    end_all = clock();
    printf("batch function call num: %d\n", total_batch_func_num);
    printf("[isp cpu batch] estimated result block number: %d\n", total_result_block_num);
    printf("[isp cpu batch] pure read time: %f\n",(double)pure_read);
    printf("[isp cpu batch] query time (including estimation): %f\n", (double)(end_all - start_all));

    return result_count;
}


int
run_spatio_temporal_query_device_fpga_batch(struct spatio_temporal_range_predicate *predicate, struct traj_storage *data_storage,
                                      struct seg_meta_section_entry_storage *meta_storage, int block_logical_addr_count,
                                      int *block_logical_addr_vec, bool enable_result_estimation) {
    int result_count = 0;

    clock_t start, end, pure_read;
    pure_read = 0;
    clock_t start_all, end_all;
    int total_result_block_num = 0;
    start_all = clock();
    fill_points_buffer(points_buffer_size);

    // prepare batch parameters
    int batch_size = REQUEST_BATCH_SIZE; //  fix bug here -> fixed: caused by the same request used in the batch using the same lba_array (before, we do not use malloc for lba_array)
    void* result_buffer_vec[batch_size];
    size_t result_buffer_size_vec[batch_size];
    struct isp_descriptor *isp_desc_vec[batch_size];
    struct lba *lba_vec_ptr[batch_size];
    int batch_count = 0;

    for (int i = 0; i < block_logical_addr_count; i += 256) {

        int block_addr_vec_size = block_logical_addr_count - i > 256 ? 256 : block_logical_addr_count - i;
        int estimated_result_size = block_addr_vec_size * 0x1000;
        if (enable_result_estimation) {
            estimated_result_size = estimate_spatio_temporal_result_size_for_blocks(meta_storage, predicate,
                                                                                    &block_logical_addr_vec[i],
                                                                                    block_addr_vec_size);
        }
        //printf("estimated result size: %d\n", estimated_result_size);
        if (estimated_result_size == 0) {
            continue;
        }
        if (estimated_result_size % 0x1000 != 0) {
            estimated_result_size = ((estimated_result_size / 0x1000) + 1) * 0x1000;
        }
        int estimated_result_block_num = estimated_result_size / 0x1000;
        //int estimated_result_block_num = 1;
        total_result_block_num += estimated_result_block_num;


        void* result_buffer = malloc(estimated_result_size);

        int block_meta_vec_size = calculate_aggregate_block_vec_size(&block_logical_addr_vec[i], block_addr_vec_size);
        struct continuous_block_meta block_meta_vec[block_meta_vec_size];
        aggregate_blocks(&block_logical_addr_vec[i], block_addr_vec_size, block_meta_vec, block_meta_vec_size);

        struct isp_descriptor *isp_desc = malloc(sizeof(struct isp_descriptor));
        struct lba *lba_vec = malloc(sizeof(struct lba) * block_meta_vec_size);
        assemble_lba_vec_via_block_meta(lba_vec, DATA_FILE_OFFSET, block_meta_vec, block_meta_vec_size);
        assemble_isp_desc_for_spatial_temporal(isp_desc, predicate, estimated_result_block_num, lba_vec, block_meta_vec_size);

        //start = clock();
        result_buffer_vec[batch_count] = result_buffer;
        result_buffer_size_vec[batch_count] = estimated_result_size;
        isp_desc_vec[batch_count] = isp_desc;
        lba_vec_ptr[batch_count] = lba_vec;


        batch_count++;
        if (batch_count % batch_size == 0) {
            start = clock();
            do_isp_for_trajectory_data_batch(batch_size, data_storage, result_buffer_vec, result_buffer_size_vec, isp_desc_vec, 1);
            end = clock();
            batch_count = 0;


            for (int k = 0; k < batch_size; k++) {
                result_count += spatio_temporal_query_isp_fpga_output_blocks(result_buffer_vec[k], predicate);
                //print_isp_descriptor(isp_desc_vec[k]);
            }

            pure_read += (end - start);
            for (int m = 0; m < batch_size; m++) {
                free(result_buffer_vec[m]);
                free(isp_desc_vec[m]);
                free(lba_vec_ptr[m]);
            }

        }

    }
    // handle the last batch (less than batch size)
    start = clock();
    do_isp_for_trajectory_data_batch(batch_count, data_storage, result_buffer_vec, result_buffer_size_vec, isp_desc_vec, 1);
    end = clock();

    for (int k = 0; k < batch_count; k++) {
        result_count += spatio_temporal_query_isp_fpga_output_blocks(result_buffer_vec[k], predicate);
    }

    pure_read += (end - start);
    for (int m = 0; m < batch_count; m++) {
        free(result_buffer_vec[m]);
        free(isp_desc_vec[m]);
        free(lba_vec_ptr[m]);
    }


    free_points_buffer(points_buffer_size);
    end_all = clock();
    printf("[isp fpga batch] estimated result block number: %d\n", total_result_block_num);
    printf("[isp fpga batch] pure read time: %f\n",(double)pure_read);
    printf("[isp fpga batch] query time (including estimation): %f\n", (double)(end_all - start_all));

    return result_count;
}


int spatio_temporal_query_with_adaptive_pushdown(struct simple_query_engine *engine, struct spatio_temporal_range_predicate *predicate, bool enable_host_index) {

    struct index_entry_storage *index_storage = &engine->index_storage;
    struct traj_storage *data_storage = &engine->data_storage;
    struct seg_meta_section_entry_storage *meta_storage = &engine->seg_meta_storage;

    // calculate the matched block num
    int block_logical_addr_count = 0;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        if (enable_host_index) {
            if (predicate->lon_min <= entry->lon_max
                && predicate->lon_max >= entry->lon_min
                && predicate->lat_min <= entry->lat_max
                && predicate->lat_max >= entry->lat_min
                && predicate->time_min <= entry->time_max
                && predicate->time_max >= entry->time_min) {
                block_logical_addr_count++;
            }
        } else {
            block_logical_addr_count++;
        }
    }
    if (block_logical_addr_count == 0) {
        return 0;
    }

    // get the match block id vector
    // int block_logical_addr_vec[block_logical_addr_count];
    int *block_logical_addr_vec = malloc(block_logical_addr_count * sizeof(int));
    int addr_vec_index = 0;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        if (enable_host_index) {
            if (predicate->lon_min <= entry->lon_max
                && predicate->lon_max >= entry->lon_min
                && predicate->lat_min <= entry->lat_max
                && predicate->lat_max >= entry->lat_min
                && predicate->time_min <= entry->time_max
                && predicate->time_max >= entry->time_min) {
                block_logical_addr_vec[addr_vec_index] = entry->block_logical_adr;
                addr_vec_index++;
            }
        } else {
            block_logical_addr_vec[addr_vec_index] = entry->block_logical_adr;
            addr_vec_index++;
        }
    }

    // separate block id vector according to the goodness
    bool flag_for_pushdown[block_logical_addr_count];
    int pushdown_block_num = 0;
    for (int i = 0; i < block_logical_addr_count; i++) {
        if (calculate_goodness_for_spatio_temporal(meta_storage, block_logical_addr_vec[i], predicate) <= 0.3) {
            flag_for_pushdown[i] = true;
            pushdown_block_num++;
        } else {
            flag_for_pushdown[i] = false;
        }
    }

    int *blocks_for_pushdown = malloc(pushdown_block_num * sizeof(int));
    int pushdown_index = 0;
    int host_block_num = block_logical_addr_count - pushdown_block_num;
    int *blocks_for_host = malloc(host_block_num * sizeof(int));
    int host_index = 0;
    for (int i = 0; i < block_logical_addr_count; i++) {
        if (flag_for_pushdown[i]) {
            blocks_for_pushdown[pushdown_index] = block_logical_addr_vec[i];
            pushdown_index++;
        } else {
            blocks_for_host[host_index] = block_logical_addr_vec[i];
            host_index++;
        }
    }

    // run query
    printf("[isp adaptive] host block num: %d, device block num: %d\n", host_block_num, pushdown_block_num);
    int result_count1 = run_spatio_temporal_query_host_multi_addr(predicate, data_storage, host_block_num, blocks_for_host);
    int result_count2 = run_spatio_temporal_query_device_fpga(predicate, data_storage, meta_storage, pushdown_block_num, blocks_for_pushdown, enable_estimated_result_size);

    free(blocks_for_pushdown);
    free(blocks_for_host);
    free(block_logical_addr_vec);
    return result_count1 + result_count2;
}

int spatio_temporal_query_with_adaptive_pushdown_batch(struct simple_query_engine *engine, struct spatio_temporal_range_predicate *predicate, bool enable_host_index) {

    struct index_entry_storage *index_storage = &engine->index_storage;
    struct traj_storage *data_storage = &engine->data_storage;
    struct seg_meta_section_entry_storage *meta_storage = &engine->seg_meta_storage;

    // calculate the matched block num
    int block_logical_addr_count = 0;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        if (enable_host_index) {
            if (predicate->lon_min <= entry->lon_max
                && predicate->lon_max >= entry->lon_min
                && predicate->lat_min <= entry->lat_max
                && predicate->lat_max >= entry->lat_min
                && predicate->time_min <= entry->time_max
                && predicate->time_max >= entry->time_min) {
                block_logical_addr_count++;
            }
        } else {
            block_logical_addr_count++;
        }
    }
    if (block_logical_addr_count == 0) {
        return 0;
    }

    // get the match block id vector
    // int block_logical_addr_vec[block_logical_addr_count];
    int *block_logical_addr_vec = malloc(block_logical_addr_count * sizeof(int));
    int addr_vec_index = 0;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        if (enable_host_index) {
            if (predicate->lon_min <= entry->lon_max
                && predicate->lon_max >= entry->lon_min
                && predicate->lat_min <= entry->lat_max
                && predicate->lat_max >= entry->lat_min
                && predicate->time_min <= entry->time_max
                && predicate->time_max >= entry->time_min) {
                block_logical_addr_vec[addr_vec_index] = entry->block_logical_adr;
                addr_vec_index++;
            }
        } else {
            block_logical_addr_vec[addr_vec_index] = entry->block_logical_adr;
            addr_vec_index++;
        }
    }

    // separate block id vector according to the goodness
    bool flag_for_pushdown[block_logical_addr_count];
    int pushdown_block_num = 0;
    for (int i = 0; i < block_logical_addr_count; i++) {
        if (calculate_goodness_for_spatio_temporal(meta_storage, block_logical_addr_vec[i], predicate) < 0.3) {
            flag_for_pushdown[i] = true;
            pushdown_block_num++;
        } else {
            flag_for_pushdown[i] = false;
        }
    }

    int *blocks_for_pushdown = malloc(pushdown_block_num * sizeof(int));
    int pushdown_index = 0;
    int host_block_num = block_logical_addr_count - pushdown_block_num;
    int *blocks_for_host = malloc(host_block_num * sizeof(int));
    int host_index = 0;
    for (int i = 0; i < block_logical_addr_count; i++) {
        if (flag_for_pushdown[i]) {
            blocks_for_pushdown[pushdown_index] = block_logical_addr_vec[i];
            pushdown_index++;
        } else {
            blocks_for_host[host_index] = block_logical_addr_vec[i];
            host_index++;
        }
    }

    // run query
    printf("[isp adaptive batch] host block num: %d, device block num: %d\n", host_block_num, pushdown_block_num);
    int result_count1 = run_spatio_temporal_query_host_multi_addr_batch(predicate, data_storage, host_block_num, blocks_for_host);
    int result_count2 = run_spatio_temporal_query_device_batch(predicate, data_storage, meta_storage, pushdown_block_num, blocks_for_pushdown, enable_estimated_result_size);
    printf("[isp adaptive batch] host result num: %d, device result num: %d\n", result_count1, result_count2);
    free(blocks_for_pushdown);
    free(blocks_for_host);
    free(block_logical_addr_vec);
    return result_count1 + result_count2;
}


int spatio_temporal_query_with_host_device_parallel_batch(struct simple_query_engine *engine, struct spatio_temporal_range_predicate *predicate, bool enable_host_index) {

    struct index_entry_storage *index_storage = &engine->index_storage;
    struct traj_storage *data_storage = &engine->data_storage;
    struct seg_meta_section_entry_storage *meta_storage = &engine->seg_meta_storage;

    // calculate the matched block num
    int block_logical_addr_count = 0;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        if (enable_host_index) {
            if (predicate->lon_min <= entry->lon_max
                && predicate->lon_max >= entry->lon_min
                && predicate->lat_min <= entry->lat_max
                && predicate->lat_max >= entry->lat_min
                && predicate->time_min <= entry->time_max
                && predicate->time_max >= entry->time_min) {
                block_logical_addr_count++;
            }
        } else {
            block_logical_addr_count++;
        }
    }
    if (block_logical_addr_count == 0) {
        return 0;
    }

    // get the match block id vector
    // int block_logical_addr_vec[block_logical_addr_count];
    int *block_logical_addr_vec = malloc(block_logical_addr_count * sizeof(int));
    int addr_vec_index = 0;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        if (enable_host_index) {
            if (predicate->lon_min <= entry->lon_max
                && predicate->lon_max >= entry->lon_min
                && predicate->lat_min <= entry->lat_max
                && predicate->lat_max >= entry->lat_min
                && predicate->time_min <= entry->time_max
                && predicate->time_max >= entry->time_min) {
                block_logical_addr_vec[addr_vec_index] = entry->block_logical_adr;
                addr_vec_index++;
            }
        } else {
            block_logical_addr_vec[addr_vec_index] = entry->block_logical_adr;
            addr_vec_index++;
        }
    }

    // separate block id vector according to the goodness
    bool flag_for_pushdown[block_logical_addr_count];
    int pushdown_block_num = 0;
    for (int i = 0; i < block_logical_addr_count; i++) {
        if (calculate_goodness_for_spatio_temporal(meta_storage, block_logical_addr_vec[i], predicate) < 0.3) {
            flag_for_pushdown[i] = true;
            pushdown_block_num++;
        } else {
            flag_for_pushdown[i] = false;
        }
    }

    int *blocks_for_pushdown = malloc(pushdown_block_num * sizeof(int));
    int pushdown_index = 0;
    int host_block_num = block_logical_addr_count - pushdown_block_num;
    int *blocks_for_host = malloc(host_block_num * sizeof(int));
    int host_index = 0;
    for (int i = 0; i < block_logical_addr_count; i++) {
        if (flag_for_pushdown[i]) {
            blocks_for_pushdown[pushdown_index] = block_logical_addr_vec[i];
            pushdown_index++;
        } else {
            blocks_for_host[host_index] = block_logical_addr_vec[i];
            host_index++;
        }
    }

    // run query
    printf("[isp host & device batch] host block num: %d, device block num: %d\n", host_block_num, pushdown_block_num);
    //int result_count1 = run_spatio_temporal_query_host_multi_addr_batch(predicate, data_storage, host_block_num, blocks_for_host);
    //int result_count2 = run_spatio_temporal_query_device_batch(predicate, data_storage, meta_storage, pushdown_block_num, blocks_for_pushdown, enable_estimated_result_size);
    int result_count = run_spatio_temporal_query_hybrid_batch(predicate, data_storage, meta_storage, host_block_num, blocks_for_host, pushdown_block_num, blocks_for_pushdown, enable_estimated_result_size);

    free(blocks_for_pushdown);
    free(blocks_for_host);
    free(block_logical_addr_vec);
    return result_count;
}


int id_temporal_query_with_full_pushdown(struct simple_query_engine *engine, struct id_temporal_predicate *predicate, bool enable_host_index) {
    struct index_entry_storage *index_storage = &engine->index_storage;
    struct traj_storage *data_storage = &engine->data_storage;
    struct seg_meta_section_entry_storage *meta_storage = &engine->seg_meta_storage;

    // calculate the matched block num
    int block_logical_addr_count = 0;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        if (enable_host_index) {
            if (check_oid_exist(entry->oid_array, entry->oid_array_size, predicate->oid) > 0
                && predicate->time_min <= entry->time_max
                && predicate->time_max >= entry->time_min) {
                block_logical_addr_count++;
            }
        } else {
            block_logical_addr_count++;
        }
    }
    if (block_logical_addr_count == 0) {
        return 0;
    }

    // get the match block id vector
    //int block_logical_addr_vec[block_logical_addr_count];
    int *block_logical_addr_vec = malloc(block_logical_addr_count * sizeof(int));
    int addr_vec_index = 0;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        if (enable_host_index) {
            if (check_oid_exist(entry->oid_array, entry->oid_array_size, predicate->oid) > 0
                && predicate->time_min <= entry->time_max
                && predicate->time_max >= entry->time_min) {
                block_logical_addr_vec[addr_vec_index] = entry->block_logical_adr;
                addr_vec_index++;
            }
        } else {
            block_logical_addr_vec[addr_vec_index] = entry->block_logical_adr;
            addr_vec_index++;
        }
    }
    printf("[isp cpu] id temporal read block num: %d\n", block_logical_addr_count);
    // run query
    int result_count = run_id_temporal_query_device(predicate, data_storage, meta_storage, block_logical_addr_count,
                                                        block_logical_addr_vec, enable_estimated_result_size);
    free(block_logical_addr_vec);
    return result_count;
}

int run_id_temporal_query_device(struct id_temporal_predicate *predicate, struct traj_storage *data_storage,
                                 struct seg_meta_section_entry_storage *meta_storage, int block_logical_addr_count,
                                 int *block_logical_addr_vec, bool enable_result_estimation) {
    int result_count = 0;

    clock_t start, end, pure_read;
    pure_read = 0;
    clock_t start_all, end_all;

    int total_result_block_num = 0;
    start_all = clock();
    fill_points_buffer(points_buffer_size);
    for (int i = 0; i < block_logical_addr_count; i += 256) {
        int block_addr_vec_size = block_logical_addr_count - i > 256 ? 256 : block_logical_addr_count - i;
        int estimated_result_size = block_addr_vec_size * 0x1000;
        if (enable_result_estimation) {
            estimated_result_size = estimate_id_temporal_result_size_for_blocks(meta_storage, predicate,
                                                                                    &block_logical_addr_vec[i],
                                                                                    block_addr_vec_size);
        }
        //printf("estimated result size: %d\n", estimated_result_size);
        if (estimated_result_size == 0) {
            continue;
        }
        if (estimated_result_size % 0x1000 != 0) {
            estimated_result_size = ((estimated_result_size / 0x1000) + 1) * 0x1000;
        }
        int estimated_result_block_num = estimated_result_size / 0x1000;
        //int estimated_result_block_num = 1;
        total_result_block_num += estimated_result_block_num;

        void* result_buffer = malloc(estimated_result_size);

        int block_meta_vec_size = calculate_aggregate_block_vec_size(&block_logical_addr_vec[i], block_addr_vec_size);
        struct continuous_block_meta block_meta_vec[block_meta_vec_size];
        aggregate_blocks(&block_logical_addr_vec[i], block_addr_vec_size, block_meta_vec, block_meta_vec_size);

        struct isp_descriptor isp_desc;
        struct lba lba_vec[block_meta_vec_size];
        assemble_lba_vec_via_block_meta(lba_vec, DATA_FILE_OFFSET, block_meta_vec, block_meta_vec_size);
        assemble_isp_desc_for_id_temporal(&isp_desc, predicate, estimated_result_block_num, lba_vec, block_meta_vec_size);

        start = clock();
        do_isp_for_trajectory_data(data_storage, result_buffer, estimated_result_size, &isp_desc, 0);
        end = clock();
        pure_read += (end - start);

        /*for (int block_count = 0; block_count < estimated_result_block_num; block_count++) {
            int count = id_temporal_query_isp_output_block(result_buffer + block_count * 4096, predicate);
            result_count += count;
        }*/
        result_count += id_temporal_query_isp_new_format_output_blocks(result_buffer, predicate);

        free(result_buffer);

        //printf("pure read time [isp]: %f\n",(double)(end-start));
    }
    free_points_buffer(points_buffer_size);
    end_all = clock();
    printf("[isp cpu] id temporal estimated result block number: %d\n", total_result_block_num);
    printf("[isp cpu] id temporal pure read time: %f\n",(double)pure_read);
    printf("[isp cpu] id temporal query time (including estimation): %f\n", (double)(end_all - start_all));


    return result_count;
}

int id_temporal_query_with_full_pushdown_fpga(struct simple_query_engine *engine, struct id_temporal_predicate *predicate, bool enable_host_index) {

    struct index_entry_storage *index_storage = &engine->index_storage;
    struct traj_storage *data_storage = &engine->data_storage;
    struct seg_meta_section_entry_storage *meta_storage = &engine->seg_meta_storage;

    // calculate the matched block num
    int block_logical_addr_count = 0;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        if (enable_host_index) {
            if (check_oid_exist(entry->oid_array, entry->oid_array_size, predicate->oid) > 0
                && predicate->time_min <= entry->time_max
                && predicate->time_max >= entry->time_min) {
                block_logical_addr_count++;
            }
        } else {
            block_logical_addr_count++;
        }
    }
    if (block_logical_addr_count == 0) {
        return 0;
    }

    // get the match block id vector
    // int block_logical_addr_vec[block_logical_addr_count];
    int *block_logical_addr_vec = malloc(block_logical_addr_count * sizeof(int));
    int addr_vec_index = 0;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        if (enable_host_index) {
            if (check_oid_exist(entry->oid_array, entry->oid_array_size, predicate->oid) > 0
                && predicate->time_min <= entry->time_max
                && predicate->time_max >= entry->time_min) {
                block_logical_addr_vec[addr_vec_index] = entry->block_logical_adr;
                addr_vec_index++;
            }
        } else {
            block_logical_addr_vec[addr_vec_index] = entry->block_logical_adr;
            addr_vec_index++;
        }
    }

    printf("[isp fpga] id temporal read block num: %d\n", block_logical_addr_count);
    // run query
    int result_count = run_id_temporal_query_device_fpga(predicate, data_storage, meta_storage, block_logical_addr_count,
                                                             block_logical_addr_vec, enable_estimated_result_size);
    free(block_logical_addr_vec);
    return result_count;
}

int
run_id_temporal_query_device_fpga(struct id_temporal_predicate *predicate, struct traj_storage *data_storage,
                                      struct seg_meta_section_entry_storage *meta_storage, int block_logical_addr_count,
                                      int *block_logical_addr_vec, bool enable_result_estimation) {
    int result_count = 0;

    clock_t start, end, pure_read;
    pure_read = 0;
    clock_t start_all, end_all;
    int total_result_block_num = 0;
    start_all = clock();
    fill_points_buffer(points_buffer_size);
    for (int i = 0; i < block_logical_addr_count; i += 256) {
        int block_addr_vec_size = block_logical_addr_count - i > 256 ? 256 : block_logical_addr_count - i;
        int estimated_result_size = block_addr_vec_size * 0x1000;
        if (enable_result_estimation) {
            estimated_result_size = estimate_id_temporal_result_size_for_blocks(meta_storage, predicate,
                                                                                    &block_logical_addr_vec[i],
                                                                                    block_addr_vec_size);
        }
        //printf("estimated result size: %d\n", estimated_result_size);
        if (estimated_result_size == 0) {
            continue;
        }
        if (estimated_result_size % 0x1000 != 0) {
            estimated_result_size = ((estimated_result_size / 0x1000) + 1) * 0x1000;
        }
        int estimated_result_block_num = estimated_result_size / 0x1000;
        //int estimated_result_block_num = 1;
        total_result_block_num += estimated_result_block_num;


        void* result_buffer = malloc(estimated_result_size);

        int block_meta_vec_size = calculate_aggregate_block_vec_size(&block_logical_addr_vec[i], block_addr_vec_size);
        struct continuous_block_meta block_meta_vec[block_meta_vec_size];
        aggregate_blocks(&block_logical_addr_vec[i], block_addr_vec_size, block_meta_vec, block_meta_vec_size);

        struct isp_descriptor isp_desc;
        struct lba lba_vec[block_meta_vec_size];
        assemble_lba_vec_via_block_meta(lba_vec, DATA_FILE_OFFSET, block_meta_vec, block_meta_vec_size);
        assemble_isp_desc_for_id_temporal(&isp_desc, predicate, estimated_result_block_num, lba_vec, block_meta_vec_size);

        start = clock();
        do_isp_for_trajectory_data(data_storage, result_buffer, estimated_result_size, &isp_desc, 1);
        end = clock();
        pure_read += (end - start);
        result_count += id_temporal_query_isp_fpga_output_blocks(result_buffer, predicate);

        free(result_buffer);


    }
    free_points_buffer(points_buffer_size);
    end_all = clock();
    printf("[isp fpga] id temporal estimated result block number: %d\n", total_result_block_num);
    printf("[isp fpga] id temporal pure read time: %f\n",(double)pure_read);
    printf("[isp fpga] id temporal query time (including estimation): %f\n", (double)(end_all - start_all));

    return result_count;
}

int id_temporal_query_without_pushdown(struct simple_query_engine *engine, struct id_temporal_predicate *predicate, bool enable_host_index) {

    struct index_entry_storage *index_storage = &engine->index_storage;
    struct traj_storage *data_storage = &engine->data_storage;
    struct seg_meta_section_entry_storage *meta_storage = &engine->seg_meta_storage;

    // calculate the matched block num
    // calculate the matched block num
    int block_logical_addr_count = 0;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        if (enable_host_index) {
            if (check_oid_exist(entry->oid_array, entry->oid_array_size, predicate->oid) > 0
                && predicate->time_min <= entry->time_max
                && predicate->time_max >= entry->time_min) {
                block_logical_addr_count++;
            }
        } else {
            block_logical_addr_count++;
        }
    }
    if (block_logical_addr_count == 0) {
        return 0;
    }

    // get the match block id vector
    // int block_logical_addr_vec[block_logical_addr_count];
    int *block_logical_addr_vec = malloc(block_logical_addr_count * sizeof(int));
    int addr_vec_index = 0;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        if (enable_host_index) {
            if (check_oid_exist(entry->oid_array, entry->oid_array_size, predicate->oid) > 0
                && predicate->time_min <= entry->time_max
                && predicate->time_max >= entry->time_min) {
                block_logical_addr_vec[addr_vec_index] = entry->block_logical_adr;
                addr_vec_index++;
            }
        } else {
            block_logical_addr_vec[addr_vec_index] = entry->block_logical_adr;
            addr_vec_index++;
        }
    }
    printf("[host] id temporal read block num: %d\n", block_logical_addr_count);
    // run query
    int result_count = run_id_temporal_query_in_host(predicate, data_storage, block_logical_addr_count,
                                                     block_logical_addr_vec);

    free(block_logical_addr_vec);
    return result_count;
}

int
run_id_temporal_query_in_host(struct id_temporal_predicate *predicate, struct traj_storage *data_storage,
                                  int block_logical_addr_count, int *block_logical_addr_vec) {
    clock_t start, start_config, end;
    clock_t start_read, end_read, pure_read;
    clock_t start_computation, end_computation, pure_computation;
    pure_computation = 0;
    pure_read = 0;
    start_config = clock();
    int result_count = 0;
    int aggregated_block_vec_size = calculate_aggregate_block_vec_size(block_logical_addr_vec, block_logical_addr_count);
    struct continuous_block_meta aggregated_block_vec[aggregated_block_vec_size];
    aggregate_blocks(block_logical_addr_vec, block_logical_addr_count, aggregated_block_vec, aggregated_block_vec_size);

    fill_points_buffer(points_buffer_size);

    start = clock();


    // split aggregated block if its size is larger than 256
    for (int i = 0; i < aggregated_block_vec_size; i++) {
        struct continuous_block_meta block_meta = aggregated_block_vec[i];
        int block_count = block_meta.block_count;
        int block_start = block_meta.block_start;
        for (int j = 0; j < block_count; j+=256) {
            int current_block_count = block_count - j > 256 ? 256 : block_count - j;
            int current_block_start = block_start + j;
            char *buffer = malloc(current_block_count * TRAJ_BLOCK_SIZE);
            start_read = clock();
            fetch_continuous_traj_data_block(data_storage, current_block_start, current_block_count, buffer);
            end_read = clock();
            pure_read += (end_read - start_read);
            for (int index = 0; index < current_block_count; index++) {
                start_computation = clock();
                int count = id_temporal_query_raw_trajectory_block_without_meta_filtering(buffer + index * TRAJ_BLOCK_SIZE, predicate);
                //int count = id_temporal_query_raw_trajectory_block(buffer + index * TRAJ_BLOCK_SIZE, predicate);
                result_count += count;
                end_computation = clock();
                pure_computation += (end_computation - start_computation);
            }
            free(buffer);
        }

    }



    end = clock();
    free_points_buffer(points_buffer_size);
    printf("[host] pure read time: %f\n",(double)(pure_read));
    printf("[host] pure computation time: %f\n",(double)(pure_computation));
    printf("[host] pure read and computation time: %f\n",(double)(end-start));
    printf("[host] query time: %f\n",(double)(end-start_config));
    return result_count;
}


int id_temporal_query_with_full_pushdown_fpga_batch(struct simple_query_engine *engine, struct id_temporal_predicate *predicate, bool enable_host_index) {

    struct index_entry_storage *index_storage = &engine->index_storage;
    struct traj_storage *data_storage = &engine->data_storage;
    struct seg_meta_section_entry_storage *meta_storage = &engine->seg_meta_storage;

    // calculate the matched block num
    int block_logical_addr_count = 0;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        if (enable_host_index) {
            if (check_oid_exist(entry->oid_array, entry->oid_array_size, predicate->oid) > 0
                && predicate->time_min <= entry->time_max
                && predicate->time_max >= entry->time_min) {
                block_logical_addr_count++;
            }
        } else {
            block_logical_addr_count++;
        }
    }
    if (block_logical_addr_count == 0) {
        return 0;
    }

    // get the match block id vector
    // int block_logical_addr_vec[block_logical_addr_count];
    int *block_logical_addr_vec = malloc(block_logical_addr_count * sizeof(int));
    int addr_vec_index = 0;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        if (enable_host_index) {
            if (check_oid_exist(entry->oid_array, entry->oid_array_size, predicate->oid) > 0
                && predicate->time_min <= entry->time_max
                && predicate->time_max >= entry->time_min) {
                block_logical_addr_vec[addr_vec_index] = entry->block_logical_adr;
                addr_vec_index++;
            }
        } else {
            block_logical_addr_vec[addr_vec_index] = entry->block_logical_adr;
            addr_vec_index++;
        }
    }

    printf("[isp fpga batch] id temporal read block num: %d\n", block_logical_addr_count);
    // run query
    int result_count = run_id_temporal_query_device_fpga_batch(predicate, data_storage, meta_storage, block_logical_addr_count,
                                                         block_logical_addr_vec, enable_estimated_result_size);
    free(block_logical_addr_vec);
    return result_count;
}


int
run_id_temporal_query_device_fpga_batch(struct id_temporal_predicate *predicate, struct traj_storage *data_storage,
                                        struct seg_meta_section_entry_storage *meta_storage, int block_logical_addr_count,
                                        int *block_logical_addr_vec, bool enable_result_estimation) {
    int result_count = 0;

    clock_t start, end, pure_read;
    pure_read = 0;
    clock_t start_all, end_all;
    int total_result_block_num = 0;
    start_all = clock();
    fill_points_buffer(points_buffer_size);

    // prepare batch parameters
    int batch_size = REQUEST_BATCH_SIZE; //  fix bug here -> fixed: caused by the same request used in the batch using the same lba_array (before, we do not use malloc for lba_array)
    void* result_buffer_vec[batch_size];
    size_t result_buffer_size_vec[batch_size];
    struct isp_descriptor *isp_desc_vec[batch_size];
    struct lba *lba_vec_ptr[batch_size];
    int batch_count = 0;

    for (int i = 0; i < block_logical_addr_count; i += 256) {

        int block_addr_vec_size = block_logical_addr_count - i > 256 ? 256 : block_logical_addr_count - i;
        int estimated_result_size = block_addr_vec_size * 0x1000;
        if (enable_result_estimation) {
            estimated_result_size = estimate_id_temporal_result_size_for_blocks(meta_storage, predicate,
                                                                                    &block_logical_addr_vec[i],
                                                                                    block_addr_vec_size);
        }
        //printf("estimated result size: %d\n", estimated_result_size);
        if (estimated_result_size == 0) {
            continue;
        }
        if (estimated_result_size % 0x1000 != 0) {
            estimated_result_size = ((estimated_result_size / 0x1000) + 1) * 0x1000;
        }
        int estimated_result_block_num = estimated_result_size / 0x1000;
        //int estimated_result_block_num = 1;
        total_result_block_num += estimated_result_block_num;


        void* result_buffer = malloc(estimated_result_size);

        int block_meta_vec_size = calculate_aggregate_block_vec_size(&block_logical_addr_vec[i], block_addr_vec_size);
        struct continuous_block_meta block_meta_vec[block_meta_vec_size];
        aggregate_blocks(&block_logical_addr_vec[i], block_addr_vec_size, block_meta_vec, block_meta_vec_size);

        struct isp_descriptor *isp_desc = malloc(sizeof(struct isp_descriptor));
        struct lba *lba_vec = malloc(sizeof(struct lba) * block_meta_vec_size);
        assemble_lba_vec_via_block_meta(lba_vec, DATA_FILE_OFFSET, block_meta_vec, block_meta_vec_size);
        assemble_isp_desc_for_id_temporal(isp_desc, predicate, estimated_result_block_num, lba_vec, block_meta_vec_size);

        //start = clock();
        result_buffer_vec[batch_count] = result_buffer;
        result_buffer_size_vec[batch_count] = estimated_result_size;
        isp_desc_vec[batch_count] = isp_desc;
        lba_vec_ptr[batch_count] = lba_vec;


        batch_count++;
        if (batch_count % batch_size == 0) {
            start = clock();
            do_isp_for_trajectory_data_batch(batch_size, data_storage, result_buffer_vec, result_buffer_size_vec, isp_desc_vec, 1);
            end = clock();
            batch_count = 0;


            for (int k = 0; k < batch_size; k++) {
                result_count += id_temporal_query_isp_fpga_output_blocks(result_buffer_vec[k], predicate);
                //print_isp_descriptor(isp_desc_vec[k]);
            }

            pure_read += (end - start);
            for (int m = 0; m < batch_size; m++) {
                free(result_buffer_vec[m]);
                free(isp_desc_vec[m]);
                free(lba_vec_ptr[m]);
            }

        }

    }
    // handle the last batch (less than batch size)
    start = clock();
    do_isp_for_trajectory_data_batch(batch_count, data_storage, result_buffer_vec, result_buffer_size_vec, isp_desc_vec, 1);
    end = clock();

    for (int k = 0; k < batch_count; k++) {
        result_count += id_temporal_query_isp_fpga_output_blocks(result_buffer_vec[k], predicate);
    }

    pure_read += (end - start);
    for (int m = 0; m < batch_count; m++) {
        free(result_buffer_vec[m]);
        free(isp_desc_vec[m]);
        free(lba_vec_ptr[m]);
    }


    free_points_buffer(points_buffer_size);
    end_all = clock();
    printf("[isp fpga batch] estimated result block number: %d\n", total_result_block_num);
    printf("[isp fpga batch] pure read time: %f\n",(double)pure_read);
    printf("[isp fpga batch] query time (including estimation): %f\n", (double)(end_all - start_all));

    return result_count;
}

int id_temporal_query_with_full_pushdown_batch_naive(struct simple_query_engine *engine, struct id_temporal_predicate *predicate, bool enable_host_index) {
    struct index_entry_storage *index_storage = &engine->index_storage;
    struct traj_storage *data_storage = &engine->data_storage;
    struct seg_meta_section_entry_storage *meta_storage = &engine->seg_meta_storage;

    // calculate the matched block num
    int block_logical_addr_count = 0;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        if (enable_host_index) {
            if (check_oid_exist(entry->oid_array, entry->oid_array_size, predicate->oid) > 0
                && predicate->time_min <= entry->time_max
                && predicate->time_max >= entry->time_min) {
                block_logical_addr_count++;
            }
        } else {
            block_logical_addr_count++;
        }
    }
    if (block_logical_addr_count == 0) {
        return 0;
    }

    // get the match block id vector
    // int block_logical_addr_vec[block_logical_addr_count];
    int *block_logical_addr_vec = malloc(block_logical_addr_count * sizeof(int));
    int addr_vec_index = 0;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        if (enable_host_index) {
            if (check_oid_exist(entry->oid_array, entry->oid_array_size, predicate->oid) > 0
                && predicate->time_min <= entry->time_max
                && predicate->time_max >= entry->time_min) {
                block_logical_addr_vec[addr_vec_index] = entry->block_logical_adr;
                addr_vec_index++;
            }
        } else {
            block_logical_addr_vec[addr_vec_index] = entry->block_logical_adr;
            addr_vec_index++;
        }
    }
    printf("[isp cpu batch naive] id temporal read block num: %d\n", block_logical_addr_count);
    // run query
    int result_count = run_id_temporal_query_device_batch_naive(predicate, data_storage, meta_storage, block_logical_addr_count,
                                                          block_logical_addr_vec, enable_estimated_result_size);
    free(block_logical_addr_vec);
    return result_count;
}


int id_temporal_query_with_full_pushdown_batch(struct simple_query_engine *engine, struct id_temporal_predicate *predicate, bool enable_host_index) {
    struct index_entry_storage *index_storage = &engine->index_storage;
    struct traj_storage *data_storage = &engine->data_storage;
    struct seg_meta_section_entry_storage *meta_storage = &engine->seg_meta_storage;

    // calculate the matched block num
    int block_logical_addr_count = 0;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        if (enable_host_index) {
            if (check_oid_exist(entry->oid_array, entry->oid_array_size, predicate->oid) > 0
                && predicate->time_min <= entry->time_max
                && predicate->time_max >= entry->time_min) {
                block_logical_addr_count++;
            }
        } else {
            block_logical_addr_count++;
        }
    }
    if (block_logical_addr_count == 0) {
        return 0;
    }

    // get the match block id vector
    // int block_logical_addr_vec[block_logical_addr_count];
    int *block_logical_addr_vec = malloc(block_logical_addr_count * sizeof(int));
    int addr_vec_index = 0;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        if (enable_host_index) {
            if (check_oid_exist(entry->oid_array, entry->oid_array_size, predicate->oid) > 0
                && predicate->time_min <= entry->time_max
                && predicate->time_max >= entry->time_min) {
                block_logical_addr_vec[addr_vec_index] = entry->block_logical_adr;
                addr_vec_index++;
            }
        } else {
            block_logical_addr_vec[addr_vec_index] = entry->block_logical_adr;
            addr_vec_index++;
        }
    }
    printf("[isp cpu batch] id temporal read block num: %d\n", block_logical_addr_count);
    // run query
    int result_count = run_id_temporal_query_device_batch(predicate, data_storage, meta_storage, block_logical_addr_count,
                                                    block_logical_addr_vec, enable_estimated_result_size);
    free(block_logical_addr_vec);
    return result_count;
}


int
run_id_temporal_query_device_batch_naive(struct id_temporal_predicate *predicate, struct traj_storage *data_storage,
                                   struct seg_meta_section_entry_storage *meta_storage, int block_logical_addr_count,
                                   int *block_logical_addr_vec, bool enable_result_estimation) {
    int result_count = 0;

    clock_t start, end, pure_read;
    pure_read = 0;
    clock_t start_all, end_all;
    int total_result_block_num = 0;
    start_all = clock();
    fill_points_buffer(points_buffer_size);

    // prepare batch parameters
    int batch_size = REQUEST_BATCH_SIZE; //  fix bug here -> fixed: caused by the same request used in the batch using the same lba_array (before, we do not use malloc for lba_array)
    void* result_buffer_vec[batch_size];
    size_t result_buffer_size_vec[batch_size];
    struct isp_descriptor *isp_desc_vec[batch_size];
    struct lba *lba_vec_ptr[batch_size];
    int batch_count = 0;

    for (int i = 0; i < block_logical_addr_count; i += 256) {

        int block_addr_vec_size = block_logical_addr_count - i > 256 ? 256 : block_logical_addr_count - i;
        int estimated_result_size = block_addr_vec_size * 0x1000;
        if (enable_result_estimation) {
            estimated_result_size = estimate_id_temporal_result_size_for_blocks(meta_storage, predicate,
                                                                                &block_logical_addr_vec[i],
                                                                                block_addr_vec_size);
        }
        //printf("estimated result size: %d\n", estimated_result_size);
        if (estimated_result_size == 0) {
            continue;
        }
        if (estimated_result_size % 0x1000 != 0) {
            estimated_result_size = ((estimated_result_size / 0x1000) + 1) * 0x1000;
        }
        int estimated_result_block_num = estimated_result_size / 0x1000;
        //int estimated_result_block_num = 1;
        total_result_block_num += estimated_result_block_num;


        void* result_buffer = malloc(estimated_result_size);

        int block_meta_vec_size = calculate_aggregate_block_vec_size(&block_logical_addr_vec[i], block_addr_vec_size);
        struct continuous_block_meta block_meta_vec[block_meta_vec_size];
        aggregate_blocks(&block_logical_addr_vec[i], block_addr_vec_size, block_meta_vec, block_meta_vec_size);

        struct isp_descriptor *isp_desc = malloc(sizeof(struct isp_descriptor));
        struct lba *lba_vec = malloc(sizeof(struct lba) * block_meta_vec_size);
        assemble_lba_vec_via_block_meta(lba_vec, DATA_FILE_OFFSET, block_meta_vec, block_meta_vec_size);
        assemble_isp_desc_for_id_temporal_naive(isp_desc, predicate, estimated_result_block_num, lba_vec, block_meta_vec_size);

        //start = clock();
        result_buffer_vec[batch_count] = result_buffer;
        result_buffer_size_vec[batch_count] = estimated_result_size;
        isp_desc_vec[batch_count] = isp_desc;
        lba_vec_ptr[batch_count] = lba_vec;


        batch_count++;
        if (batch_count % batch_size == 0) {
            start = clock();
            do_isp_for_trajectory_data_batch(batch_size, data_storage, result_buffer_vec, result_buffer_size_vec, isp_desc_vec, 0);
            end = clock();

            for (int k = 0; k < batch_size; k++) {
                void* batch_base = result_buffer_vec[k];
                /*size_t batch_buffer_count = result_buffer_size_vec[k] / 0x1000;
                for (int j = 0; j < batch_buffer_count; j++) {
                    void* block_ptr = batch_base + j * TRAJ_BLOCK_SIZE;
                    result_count += id_temporal_query_isp_output_block(block_ptr, predicate);

                }*/
                result_count += id_temporal_query_isp_new_format_output_blocks(batch_base, predicate);
            }
            batch_count = 0;

            pure_read += (end - start);
            for (int m = 0; m < batch_size; m++) {
                free(result_buffer_vec[m]);
                free(isp_desc_vec[m]);
                free(lba_vec_ptr[m]);
            }

        }

    }
    // handle the last batch (less than batch size)
    start = clock();
    do_isp_for_trajectory_data_batch(batch_count, data_storage, result_buffer_vec, result_buffer_size_vec, isp_desc_vec, 0);
    end = clock();

    for (int k = 0; k < batch_count; k++) {
        void* batch_base = result_buffer_vec[k];
        /*size_t batch_buffer_count = result_buffer_size_vec[k] / 0x1000;
        for (int j = 0; j < batch_buffer_count; j++) {
            void* block_ptr = batch_base + j * TRAJ_BLOCK_SIZE;
            result_count += id_temporal_query_isp_output_block(block_ptr, predicate);

        }*/
        result_count += id_temporal_query_isp_new_format_output_blocks(batch_base, predicate);
    }

    pure_read += (end - start);
    for (int m = 0; m < batch_count; m++) {
        free(result_buffer_vec[m]);
        free(isp_desc_vec[m]);
        free(lba_vec_ptr[m]);
    }


    free_points_buffer(points_buffer_size);
    end_all = clock();
    printf("[isp cpu batch naive] estimated result block number: %d\n", total_result_block_num);
    printf("[isp cpu batch naive] pure read time: %f\n",(double)pure_read);
    printf("[isp cpu batch naive] query time (including estimation): %f\n", (double)(end_all - start_all));

    return result_count;
}


int
run_id_temporal_query_device_batch(struct id_temporal_predicate *predicate, struct traj_storage *data_storage,
                                        struct seg_meta_section_entry_storage *meta_storage, int block_logical_addr_count,
                                        int *block_logical_addr_vec, bool enable_result_estimation) {
    int result_count = 0;

    clock_t start, end, pure_read;
    pure_read = 0;
    clock_t start_all, end_all;
    int total_result_block_num = 0;
    start_all = clock();
    fill_points_buffer(points_buffer_size);

    // prepare batch parameters
    int batch_size = REQUEST_BATCH_SIZE; //  fix bug here -> fixed: caused by the same request used in the batch using the same lba_array (before, we do not use malloc for lba_array)
    void* result_buffer_vec[batch_size];
    size_t result_buffer_size_vec[batch_size];
    struct isp_descriptor *isp_desc_vec[batch_size];
    struct lba *lba_vec_ptr[batch_size];
    int batch_count = 0;

    for (int i = 0; i < block_logical_addr_count; i += 256) {

        int block_addr_vec_size = block_logical_addr_count - i > 256 ? 256 : block_logical_addr_count - i;
        int estimated_result_size = block_addr_vec_size * 0x1000;
        if (enable_result_estimation) {
            estimated_result_size = estimate_id_temporal_result_size_for_blocks(meta_storage, predicate,
                                                                                &block_logical_addr_vec[i],
                                                                                block_addr_vec_size);
        }
        //printf("estimated result size: %d\n", estimated_result_size);
        if (estimated_result_size == 0) {
            continue;
        }
        if (estimated_result_size % 0x1000 != 0) {
            estimated_result_size = ((estimated_result_size / 0x1000) + 1) * 0x1000;
        }
        int estimated_result_block_num = estimated_result_size / 0x1000;
        //int estimated_result_block_num = 1;
        total_result_block_num += estimated_result_block_num;


        void* result_buffer = malloc(estimated_result_size);

        int block_meta_vec_size = calculate_aggregate_block_vec_size(&block_logical_addr_vec[i], block_addr_vec_size);
        struct continuous_block_meta block_meta_vec[block_meta_vec_size];
        aggregate_blocks(&block_logical_addr_vec[i], block_addr_vec_size, block_meta_vec, block_meta_vec_size);

        struct isp_descriptor *isp_desc = malloc(sizeof(struct isp_descriptor));
        struct lba *lba_vec = malloc(sizeof(struct lba) * block_meta_vec_size);
        assemble_lba_vec_via_block_meta(lba_vec, DATA_FILE_OFFSET, block_meta_vec, block_meta_vec_size);
        assemble_isp_desc_for_id_temporal(isp_desc, predicate, estimated_result_block_num, lba_vec, block_meta_vec_size);

        //start = clock();
        result_buffer_vec[batch_count] = result_buffer;
        result_buffer_size_vec[batch_count] = estimated_result_size;
        isp_desc_vec[batch_count] = isp_desc;
        lba_vec_ptr[batch_count] = lba_vec;


        batch_count++;
        if (batch_count % batch_size == 0) {
            start = clock();
            do_isp_for_trajectory_data_batch(batch_size, data_storage, result_buffer_vec, result_buffer_size_vec, isp_desc_vec, 0);
            end = clock();

            for (int k = 0; k < batch_size; k++) {
                void* batch_base = result_buffer_vec[k];
                /*size_t batch_buffer_count = result_buffer_size_vec[k] / 0x1000;
                for (int j = 0; j < batch_buffer_count; j++) {
                    void* block_ptr = batch_base + j * TRAJ_BLOCK_SIZE;
                    result_count += id_temporal_query_isp_output_block(block_ptr, predicate);

                }*/
                result_count += id_temporal_query_isp_new_format_output_blocks(batch_base, predicate);
            }
            batch_count = 0;

            pure_read += (end - start);
            for (int m = 0; m < batch_size; m++) {
                free(result_buffer_vec[m]);
                free(isp_desc_vec[m]);
                free(lba_vec_ptr[m]);
            }

        }

    }
    // handle the last batch (less than batch size)
    start = clock();
    do_isp_for_trajectory_data_batch(batch_count, data_storage, result_buffer_vec, result_buffer_size_vec, isp_desc_vec, 0);
    end = clock();

    for (int k = 0; k < batch_count; k++) {
        void* batch_base = result_buffer_vec[k];
        /*size_t batch_buffer_count = result_buffer_size_vec[k] / 0x1000;
        for (int j = 0; j < batch_buffer_count; j++) {
            void* block_ptr = batch_base + j * TRAJ_BLOCK_SIZE;
            result_count += id_temporal_query_isp_output_block(block_ptr, predicate);

        }*/
        result_count += id_temporal_query_isp_new_format_output_blocks(batch_base, predicate);
    }

    pure_read += (end - start);
    for (int m = 0; m < batch_count; m++) {
        free(result_buffer_vec[m]);
        free(isp_desc_vec[m]);
        free(lba_vec_ptr[m]);
    }


    free_points_buffer(points_buffer_size);
    end_all = clock();
    printf("[isp cpu batch] estimated result block number: %d\n", total_result_block_num);
    printf("[isp cpu batch] pure read time: %f\n",(double)pure_read);
    printf("[isp cpu batch] query time (including estimation): %f\n", (double)(end_all - start_all));

    return result_count;
}

int
run_id_temporal_query_in_host_batch(struct id_temporal_predicate *predicate, struct traj_storage *data_storage,
                                        int block_logical_addr_count, int *block_logical_addr_vec) {
    clock_t start, start_config, end;
    clock_t start_read, end_read, pure_read;
    clock_t start_computation, end_computation, pure_computation;
    pure_read = 0;
    pure_computation = 0;
    start_config = clock();
    int result_count = 0;
    int aggregated_block_vec_size = calculate_aggregate_block_vec_size(block_logical_addr_vec, block_logical_addr_count);
    struct continuous_block_meta aggregated_block_vec[aggregated_block_vec_size];
    aggregate_blocks(block_logical_addr_vec, block_logical_addr_count, aggregated_block_vec, aggregated_block_vec_size);

    fill_points_buffer(points_buffer_size);

    int batch_size = REQUEST_BATCH_SIZE;
    int batch_count  = 0;
    int current_block_start_vec[batch_size];
    int current_block_count_vec[batch_size];
    void* result_buffer_vec[batch_size];
    start = clock();

    printf("continous memory region size: %d\n", aggregated_block_vec_size);
    // split aggregated block if its size is larger than 256
    for (int i = 0; i < aggregated_block_vec_size; i++) {
        struct continuous_block_meta block_meta = aggregated_block_vec[i];
        int block_count = block_meta.block_count;
        int block_start = block_meta.block_start;
        //printf("block start: %d, block count: %d\n", block_start, block_count);
        for (int j = 0; j < block_count; j+=256) {
            int current_block_count = block_count - j > 256 ? 256 : block_count - j;
            int current_block_start = block_start + j;
            char *buffer = malloc(current_block_count * TRAJ_BLOCK_SIZE);
            memset(buffer, 0, current_block_count * TRAJ_BLOCK_SIZE);


            current_block_count_vec[batch_count] = current_block_count;
            current_block_start_vec[batch_count] = current_block_start;
            result_buffer_vec[batch_count] = buffer;

            batch_count++;
            if (batch_count % batch_size == 0) {
                start_read = clock();
                fetch_continuous_traj_data_block_spdk_batch(batch_size, data_storage, current_block_start_vec, current_block_count_vec, result_buffer_vec);
                end_read = clock();
                batch_count = 0;
                pure_read += (end_read - start_read);

                for (int m = 0; m < batch_size; m++) {
                    void* buffer_ptr = result_buffer_vec[m];
                    int block_count_value = current_block_count_vec[m];
                    for (int index = 0; index < block_count_value; index++) {
                        start_computation = clock();
                        int count = id_temporal_query_raw_trajectory_block(
                                buffer_ptr + index * TRAJ_BLOCK_SIZE, predicate);
                        result_count += count;
                        end_computation = clock();
                        pure_computation += (end_computation - start_computation);
                    }
                }

                for (int k = 0; k < batch_size; k++) {
                    free(result_buffer_vec[k]);
                }

            }

        }

    }

    // handle the last batch
    start_read = clock();
    fetch_continuous_traj_data_block_spdk_batch(batch_count, data_storage, current_block_start_vec, current_block_count_vec, result_buffer_vec);
    end_read = clock();
    pure_read += (end_read - start_read);

    for (int m = 0; m < batch_count; m++) {
        void* buffer_ptr = result_buffer_vec[m];
        int block_count_value = current_block_count_vec[m];
        for (int index = 0; index < block_count_value; index++) {
            start_computation = clock();
            int count = id_temporal_query_raw_trajectory_block(
                    buffer_ptr + index * TRAJ_BLOCK_SIZE, predicate);
            result_count += count;
            end_computation = clock();
            pure_computation += (end_computation - start_computation);
        }
    }

    for (int k = 0; k < batch_count; k++) {
        free(result_buffer_vec[k]);
    }

    end = clock();
    free_points_buffer(points_buffer_size);
    printf("[host batch] pure read time: %f\n",(double)(pure_read));
    printf("[host batch] pure computation time: %f\n",(double)(pure_computation));
    printf("[host batch] pure read and computation time: %f\n",(double)(end-start));
    printf("[host batch] query time: %f\n",(double)(end-start_config));
    return result_count;
}


int id_temporal_query_without_pushdown_multi_addr_batch(struct simple_query_engine *engine, struct id_temporal_predicate *predicate, bool enable_host_index) {

    struct index_entry_storage *index_storage = &engine->index_storage;
    struct traj_storage *data_storage = &engine->data_storage;
    struct seg_meta_section_entry_storage *meta_storage = &engine->seg_meta_storage;

    // calculate the matched block num
    // calculate the matched block num
    int block_logical_addr_count = 0;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        if (enable_host_index) {
            if (check_oid_exist(entry->oid_array, entry->oid_array_size, predicate->oid) > 0
                && predicate->time_min <= entry->time_max
                && predicate->time_max >= entry->time_min) {
                block_logical_addr_count++;
            }
        } else {
            block_logical_addr_count++;
        }
    }
    if (block_logical_addr_count == 0) {
        return 0;
    }

    // get the match block id vector
    //int block_logical_addr_vec[block_logical_addr_count];
    int *block_logical_addr_vec = malloc(block_logical_addr_count * sizeof(int));
    int addr_vec_index = 0;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        if (enable_host_index) {
            if (check_oid_exist(entry->oid_array, entry->oid_array_size, predicate->oid) > 0
                && predicate->time_min <= entry->time_max
                && predicate->time_max >= entry->time_min) {
                block_logical_addr_vec[addr_vec_index] = entry->block_logical_adr;
                addr_vec_index++;
            }
        } else {
            block_logical_addr_vec[addr_vec_index] = entry->block_logical_adr;
            addr_vec_index++;
        }
    }
    printf("[host batch] id temporal read block num: %d\n", block_logical_addr_count);
    // run query

    int result_count = run_id_temporal_query_host_multi_addr_batch(predicate, data_storage, block_logical_addr_count, block_logical_addr_vec);
    printf("selectivity: %f\n", (1.0 * result_count / (block_logical_addr_count * 243)));
    free(block_logical_addr_vec);
    return result_count;
}


int id_temporal_query_without_pushdown_batch(struct simple_query_engine *engine, struct id_temporal_predicate *predicate, bool enable_host_index) {

    struct index_entry_storage *index_storage = &engine->index_storage;
    struct traj_storage *data_storage = &engine->data_storage;
    struct seg_meta_section_entry_storage *meta_storage = &engine->seg_meta_storage;

    // calculate the matched block num
    // calculate the matched block num
    int block_logical_addr_count = 0;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        if (enable_host_index) {
            if (check_oid_exist(entry->oid_array, entry->oid_array_size, predicate->oid) > 0
                && predicate->time_min <= entry->time_max
                && predicate->time_max >= entry->time_min) {
                block_logical_addr_count++;
            }
        } else {
            block_logical_addr_count++;
        }
    }
    if (block_logical_addr_count == 0) {
        return 0;
    }

    // get the match block id vector
    //int block_logical_addr_vec[block_logical_addr_count];
    int *block_logical_addr_vec = malloc(block_logical_addr_count * sizeof(int));
    int addr_vec_index = 0;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        if (enable_host_index) {
            if (check_oid_exist(entry->oid_array, entry->oid_array_size, predicate->oid) > 0
                && predicate->time_min <= entry->time_max
                && predicate->time_max >= entry->time_min) {
                block_logical_addr_vec[addr_vec_index] = entry->block_logical_adr;
                addr_vec_index++;
            }
        } else {
            block_logical_addr_vec[addr_vec_index] = entry->block_logical_adr;
            addr_vec_index++;
        }
    }
    printf("[host batch] id temporal read block num: %d\n", block_logical_addr_count);
    // run query
    int result_count = run_id_temporal_query_in_host_batch(predicate, data_storage, block_logical_addr_count, block_logical_addr_vec);
    //int result_count = run_id_temporal_query_host_multi_addr_batch(predicate, data_storage, block_logical_addr_count, block_logical_addr_vec);
    printf("selectivity: %f\n", (1.0 * result_count / (block_logical_addr_count * 243)));
    free(block_logical_addr_vec);
    return result_count;
}


int
run_id_temporal_query_host_multi_addr_batch(struct id_temporal_predicate *predicate, struct traj_storage *data_storage,
                                                int block_logical_addr_count,
                                                int *block_logical_addr_vec) {
    int result_count = 0;

    clock_t start, end, pure_read, comp_start, comp_end, pure_comp;
    pure_read = 0;
    pure_comp = 0;
    clock_t start_all, end_all;
    int total_result_block_num = 0;
    start_all = clock();
    fill_points_buffer(points_buffer_size);

    // prepare batch parameters
    int batch_size = REQUEST_BATCH_SIZE; //  fix bug here -> fixed: caused by the same request used in the batch using the same lba_array (before, we do not use malloc for lba_array)
    void* result_buffer_vec[batch_size];
    size_t result_buffer_size_vec[batch_size];
    struct isp_descriptor *isp_desc_vec[batch_size];
    struct lba *lba_vec_ptr[batch_size];
    int batch_count = 0;

    int batch_function_call_num = 0;
    for (int i = 0; i < block_logical_addr_count; i += 256) {

        int block_addr_vec_size = block_logical_addr_count - i > 256 ? 256 : block_logical_addr_count - i;
        int result_size = block_addr_vec_size * 0x1000;


        if (result_size % 0x1000 != 0) {
            result_size = ((result_size / 0x1000) + 1) * 0x1000;
        }
        int estimated_result_block_num = result_size / 0x1000;
        //int estimated_result_block_num = 1;
        total_result_block_num += estimated_result_block_num;


        void* result_buffer = malloc(result_size);

        int block_meta_vec_size = calculate_aggregate_block_vec_size(&block_logical_addr_vec[i], block_addr_vec_size);
        struct continuous_block_meta block_meta_vec[block_meta_vec_size];
        aggregate_blocks(&block_logical_addr_vec[i], block_addr_vec_size, block_meta_vec, block_meta_vec_size);

        struct isp_descriptor *isp_desc = malloc(sizeof(struct isp_descriptor));
        struct lba *lba_vec = malloc(sizeof(struct lba) * block_meta_vec_size);
        assemble_lba_vec_via_block_meta(lba_vec, DATA_FILE_OFFSET, block_meta_vec, block_meta_vec_size);
        assemble_isp_desc_without_comp(isp_desc, estimated_result_block_num, lba_vec, block_meta_vec_size);

        //start = clock();
        result_buffer_vec[batch_count] = result_buffer;
        result_buffer_size_vec[batch_count] = result_size;
        isp_desc_vec[batch_count] = isp_desc;
        lba_vec_ptr[batch_count] = lba_vec;


        batch_count++;
        if (batch_count % batch_size == 0) {
            start = clock();
            do_isp_for_trajectory_data_without_comp_batch(batch_size, data_storage, result_buffer_vec, result_buffer_size_vec, isp_desc_vec);
            batch_function_call_num++;
            end = clock();

            for (int k = 0; k < batch_size; k++) {
                void* batch_base = result_buffer_vec[k];
                size_t batch_buffer_count = result_buffer_size_vec[k] / 0x1000;
                for (int j = 0; j < batch_buffer_count; j++) {
                    void* block_ptr = batch_base + j * TRAJ_BLOCK_SIZE;
                    comp_start = clock();
                    result_count += id_temporal_query_raw_trajectory_block(block_ptr, predicate);
                    comp_end = clock();
                    pure_comp += (comp_end - comp_start);
                    //print_isp_descriptor(isp_desc_vec[k]);
                }
            }
            batch_count = 0;

            pure_read += (end - start);

            for (int m = 0; m < batch_size; m++) {
                free(result_buffer_vec[m]);
                free(isp_desc_vec[m]);
                free(lba_vec_ptr[m]);
            }

        }

    }
    // handle the last batch (less than batch size)
    start = clock();
    do_isp_for_trajectory_data_without_comp_batch(batch_count, data_storage, result_buffer_vec, result_buffer_size_vec, isp_desc_vec);
    batch_function_call_num++;
    end = clock();

    for (int k = 0; k < batch_count; k++) {
        void* batch_base = result_buffer_vec[k];
        size_t batch_buffer_count = result_buffer_size_vec[k] / 0x1000;
        for (int j = 0; j < batch_buffer_count; j++) {
            void* block_ptr = batch_base + j * TRAJ_BLOCK_SIZE;
            comp_start = clock();
            result_count += id_temporal_query_raw_trajectory_block(block_ptr, predicate);
            comp_end = clock();
            pure_comp += (comp_end - comp_start);
            //print_isp_descriptor(isp_desc_vec[k]);
        }
    }

    pure_read += (end - start);
    for (int m = 0; m < batch_count; m++) {
        free(result_buffer_vec[m]);
        free(isp_desc_vec[m]);
        free(lba_vec_ptr[m]);
    }


    free_points_buffer(points_buffer_size);
    end_all = clock();
    printf("batch function call num: %d\n", batch_function_call_num);
    printf("[host multi batch] estimated result block number: %d\n", total_result_block_num);
    printf("[host multi batch] pure read time: %f\n",(double)pure_read);
    printf("[host multi batch] pure computation time: %f\n",(double)pure_comp);
    printf("[host multi batch] query time (including computation): %f\n", (double)(end_all - start_all));

    return result_count;
}

int id_temporal_query_with_adaptive_pushdown_batch(struct simple_query_engine *engine, struct id_temporal_predicate *predicate, bool enable_host_index) {

    struct index_entry_storage *index_storage = &engine->index_storage;
    struct traj_storage *data_storage = &engine->data_storage;
    struct seg_meta_section_entry_storage *meta_storage = &engine->seg_meta_storage;

    // calculate the matched block num
    int block_logical_addr_count = 0;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        if (enable_host_index) {
            if (check_oid_exist(entry->oid_array, entry->oid_array_size, predicate->oid) > 0
                && predicate->time_min <= entry->time_max
                && predicate->time_max >= entry->time_min) {
                block_logical_addr_count++;
            }
        } else {
            block_logical_addr_count++;
        }
    }
    if (block_logical_addr_count == 0) {
        return 0;
    }

    // get the match block id vector
    // int block_logical_addr_vec[block_logical_addr_count];
    int *block_logical_addr_vec = malloc(block_logical_addr_count * sizeof(int));
    int addr_vec_index = 0;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        if (enable_host_index) {
            if (check_oid_exist(entry->oid_array, entry->oid_array_size, predicate->oid) > 0
                && predicate->time_min <= entry->time_max
                && predicate->time_max >= entry->time_min) {
                block_logical_addr_vec[addr_vec_index] = entry->block_logical_adr;
                addr_vec_index++;
            }
        } else {
            block_logical_addr_vec[addr_vec_index] = entry->block_logical_adr;
            addr_vec_index++;
        }
    }

    // separate block id vector according to the goodness
    bool flag_for_pushdown[block_logical_addr_count];
    int pushdown_block_num = 0;
    for (int i = 0; i < block_logical_addr_count; i++) {
        if (calculate_goodness_for_id_temporal(meta_storage, block_logical_addr_vec[i], predicate) < 0.3) {
            flag_for_pushdown[i] = true;
            pushdown_block_num++;
        } else {
            flag_for_pushdown[i] = false;
        }
    }

    int *blocks_for_pushdown = malloc(pushdown_block_num * sizeof(int));
    int pushdown_index = 0;
    int host_block_num = block_logical_addr_count - pushdown_block_num;
    int *blocks_for_host = malloc(host_block_num * sizeof(int));
    int host_index = 0;
    for (int i = 0; i < block_logical_addr_count; i++) {
        if (flag_for_pushdown[i]) {
            blocks_for_pushdown[pushdown_index] = block_logical_addr_vec[i];
            pushdown_index++;
        } else {
            blocks_for_host[host_index] = block_logical_addr_vec[i];
            host_index++;
        }
    }

    // run query
    printf("[isp adaptive batch] host block num: %d, device block num: %d\n", host_block_num, pushdown_block_num);
    int result_count1 = run_id_temporal_query_host_multi_addr_batch(predicate, data_storage, host_block_num, blocks_for_host);
    int result_count2 = run_id_temporal_query_device_batch(predicate, data_storage, meta_storage, pushdown_block_num, blocks_for_pushdown, enable_estimated_result_size);
    printf("[isp adaptive batch] host result num: %d, device result num: %d\n", result_count1, result_count2);
    free(blocks_for_pushdown);
    free(blocks_for_host);
    free(block_logical_addr_vec);
    return result_count1 + result_count2;
}




/**
 *
 *
 * knn queries
 *
 *
 *
 *
 *
 */

int spatio_temporal_knn_query_without_pushdown_multi_addr_batch(struct simple_query_engine *engine, struct spatio_temporal_knn_predicate *predicate, bool enable_host_index) {

    struct index_entry_storage *index_storage = &engine->index_storage;
    struct traj_storage *data_storage = &engine->data_storage;
    struct seg_meta_section_entry_storage *meta_storage = &engine->seg_meta_storage;

    // calculate the matched block num
    int block_logical_addr_count = 0;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        if (enable_host_index) {
            if (predicate->query_point.normalized_longitude <= entry->lon_max
                && predicate->query_point.normalized_longitude >= entry->lon_min
                && predicate->query_point.normalized_latitude <= entry->lat_max
                && predicate->query_point.normalized_latitude >= entry->lat_min
                    ) {
                block_logical_addr_count++;
            }
        } else {
            block_logical_addr_count++;
        }
    }

    if (block_logical_addr_count == 0) {
        return 0;
    }

    // get the match block id vector
    //int block_logical_addr_vec[block_logical_addr_count];   // TODO use malloc for large vector
    int *block_logical_addr_vec = malloc(block_logical_addr_count * sizeof(int));

    int addr_vec_index = 0;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        if (enable_host_index) {
            if (predicate->query_point.normalized_longitude <= entry->lon_max
                && predicate->query_point.normalized_longitude >= entry->lon_min
                && predicate->query_point.normalized_latitude <= entry->lat_max
                && predicate->query_point.normalized_latitude >= entry->lat_min
                    ) {
                block_logical_addr_vec[addr_vec_index] = entry->block_logical_adr;
                addr_vec_index++;
            }

        } else {
            block_logical_addr_vec[addr_vec_index] = entry->block_logical_adr;
            addr_vec_index++;
        }
    }


    printf("[host batch] read block num: %d\n", block_logical_addr_count);
    // run query
    int result_count = run_spatio_temporal_knn_query_host_multi_addr_batch(predicate, data_storage, block_logical_addr_count,
                                                   block_logical_addr_vec);

    free(block_logical_addr_vec);
    return result_count;
}


int spatio_temporal_knn_query_without_pushdown_batch(struct simple_query_engine *engine, struct spatio_temporal_knn_predicate *predicate, bool enable_host_index) {

    struct index_entry_storage *index_storage = &engine->index_storage;
    struct traj_storage *data_storage = &engine->data_storage;
    struct seg_meta_section_entry_storage *meta_storage = &engine->seg_meta_storage;

    // calculate the matched block num
    int block_logical_addr_count = 0;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        if (enable_host_index) {
            if (predicate->query_point.normalized_longitude <= entry->lon_max
                && predicate->query_point.normalized_longitude >= entry->lon_min
                && predicate->query_point.normalized_latitude <= entry->lat_max
                && predicate->query_point.normalized_latitude >= entry->lat_min
                ) {
                block_logical_addr_count++;
            }
        } else {
            block_logical_addr_count++;
        }
    }

    if (block_logical_addr_count == 0) {
        return 0;
    }

    // get the match block id vector
    //int block_logical_addr_vec[block_logical_addr_count];   // TODO use malloc for large vector
    int *block_logical_addr_vec = malloc(block_logical_addr_count * sizeof(int));

    int addr_vec_index = 0;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        if (enable_host_index) {
            if (predicate->query_point.normalized_longitude <= entry->lon_max
                && predicate->query_point.normalized_longitude >= entry->lon_min
                && predicate->query_point.normalized_latitude <= entry->lat_max
                && predicate->query_point.normalized_latitude >= entry->lat_min
                ) {
                block_logical_addr_vec[addr_vec_index] = entry->block_logical_adr;
                addr_vec_index++;
            }

        } else {
            block_logical_addr_vec[addr_vec_index] = entry->block_logical_adr;
            addr_vec_index++;
        }
    }


    printf("[host batch] read block num: %d\n", block_logical_addr_count);
    // run query
    int result_count = run_knn_query_in_host_batch(predicate, data_storage, block_logical_addr_count,
                                                   block_logical_addr_vec);

    free(block_logical_addr_vec);
    return result_count;
}

/**
 *
 * @param data_block
 * @param predicate
 * @param result_buffer
 * @return
 */
static int spatio_temporal_knn_query_raw_trajectory_block(void* data_block, struct spatio_temporal_knn_predicate *predicate, struct knn_result_buffer *result_buffer) {
    int result_count = 0;
    struct traj_block_header block_header;
    parse_traj_block_for_header(data_block, &block_header);

    int traj_point_size = get_traj_point_size();
    long current_max_dist = result_buffer->max_distance;

    struct seg_meta meta_array[block_header.seg_count];
    parse_traj_block_for_seg_meta_section(data_block, meta_array, block_header.seg_count);

    // pruning based on mbr
    long minmaxdist_min = LONG_MAX;
    for (int j = 0; j < block_header.seg_count; j++) {
        struct seg_meta meta_item = meta_array[j];
        long minmaxdist = cal_minmax_distance(&(predicate->query_point), &meta_item);
        if (minmaxdist < minmaxdist_min) {
            minmaxdist_min = minmaxdist;
        }
    }

    for (int j = 0; j < block_header.seg_count; j++) {
        struct seg_meta meta_item = meta_array[j];
        long min_dist = cal_min_distance(&(predicate->query_point), &meta_item);
        if (min_dist <= current_max_dist && min_dist <= minmaxdist_min) {
        //if (min_dist <= current_max_dist) {
                result_buffer->statistics.checked_segment_num++;
                int data_seg_points_num = meta_item.seg_size / traj_point_size;
                struct traj_point *point_ptr = (struct traj_point *)((char *) data_block + meta_item.seg_offset);

                for (int k = 0; k < data_seg_points_num; k++) {

                    struct traj_point *point = &point_ptr[k];
                    long distance = cal_points_distance(&(predicate->query_point), point);
                    if (distance < result_buffer->max_distance) {
                        struct result_item item = {*point, distance};
                        add_item_to_buffer(result_buffer, &item);
                        result_count++;

                    }
                }
        }


    }

    return result_count;
}

int
run_knn_query_in_host_batch(struct spatio_temporal_knn_predicate *predicate, struct traj_storage *data_storage,
                            int block_logical_addr_count, int *block_logical_addr_vec) {
    clock_t start, start_config, end;
    clock_t start_read, end_read, pure_read;
    clock_t start_computation, end_computation, pure_computation;
    struct knn_result_buffer knn_result_buffer;
    init_knn_result_buffer(predicate->k, &knn_result_buffer);

    pure_read = 0;
    pure_computation = 0;
    start_config = clock();
    int result_count = 0;
    int aggregated_block_vec_size = calculate_aggregate_block_vec_size(block_logical_addr_vec, block_logical_addr_count);
    struct continuous_block_meta aggregated_block_vec[aggregated_block_vec_size];
    aggregate_blocks(block_logical_addr_vec, block_logical_addr_count, aggregated_block_vec, aggregated_block_vec_size);

    fill_points_buffer(points_buffer_size);

    int batch_size = REQUEST_BATCH_SIZE;
    int batch_count  = 0;
    int current_block_start_vec[batch_size];
    int current_block_count_vec[batch_size];
    void* result_buffer_vec[batch_size];
    start = clock();

    // split aggregated block if its size is larger than 256
    for (int i = 0; i < aggregated_block_vec_size; i++) {
        struct continuous_block_meta block_meta = aggregated_block_vec[i];
        int block_count = block_meta.block_count;
        int block_start = block_meta.block_start;
        //printf("block start: %d, block count: %d\n", block_start, block_count);
        for (int j = 0; j < block_count; j+=256) {
            int current_block_count = block_count - j > 256 ? 256 : block_count - j;
            int current_block_start = block_start + j;
            char *buffer = malloc(current_block_count * TRAJ_BLOCK_SIZE);
            memset(buffer, 0, current_block_count * TRAJ_BLOCK_SIZE);


            current_block_count_vec[batch_count] = current_block_count;
            current_block_start_vec[batch_count] = current_block_start;
            result_buffer_vec[batch_count] = buffer;

            batch_count++;
            if (batch_count % batch_size == 0) {
                start_read = clock();
                fetch_continuous_traj_data_block_spdk_batch(batch_size, data_storage, current_block_start_vec, current_block_count_vec, result_buffer_vec);
                end_read = clock();
                batch_count = 0;
                pure_read += (end_read - start_read);

                for (int m = 0; m < batch_size; m++) {
                    void* buffer_ptr = result_buffer_vec[m];
                    int block_count_value = current_block_count_vec[m];
                    for (int index = 0; index < block_count_value; index++) {
                        start_computation = clock();
                        int count = spatio_temporal_knn_query_raw_trajectory_block(
                                buffer_ptr + index * TRAJ_BLOCK_SIZE, predicate, &knn_result_buffer);
                        result_count += count;
                        end_computation = clock();
                        pure_computation += (end_computation - start_computation);
                    }
                }

                for (int k = 0; k < batch_size; k++) {
                    free(result_buffer_vec[k]);
                }

            }

        }

    }

    // handle the last batch
    start_read = clock();
    fetch_continuous_traj_data_block_spdk_batch(batch_count, data_storage, current_block_start_vec, current_block_count_vec, result_buffer_vec);
    end_read = clock();
    pure_read += (end_read - start_read);

    for (int m = 0; m < batch_count; m++) {
        void* buffer_ptr = result_buffer_vec[m];
        int block_count_value = current_block_count_vec[m];
        for (int index = 0; index < block_count_value; index++) {
            start_computation = clock();
            int count = spatio_temporal_knn_query_raw_trajectory_block(
                    buffer_ptr + index * TRAJ_BLOCK_SIZE, predicate, &knn_result_buffer);
            result_count += count;
            end_computation = clock();
            pure_computation += (end_computation - start_computation);
        }
    }

    for (int k = 0; k < batch_count; k++) {
        free(result_buffer_vec[k]);
    }

    combine_and_sort(&knn_result_buffer);
    end = clock();
    //print_result_buffer(&knn_result_buffer);
    print_runtime_statistics(&knn_result_buffer.statistics);
    free_points_buffer(points_buffer_size);
    free_knn_result_buffer(&knn_result_buffer);
    printf("[host batch] pure read time: %f\n",(double)(pure_read));
    printf("[host batch] pure computation time: %f\n",(double)(pure_computation));
    printf("[host batch] pure read and computation time: %f\n",(double)(pure_read + pure_computation));
    printf("[host batch] query time: %f\n",(double)(end-start_config));
    return (end-start_config);
}

int
run_spatio_temporal_knn_query_host_multi_addr_batch(struct spatio_temporal_knn_predicate *predicate, struct traj_storage *data_storage,
                                                    int block_logical_addr_count,
                                                    int *block_logical_addr_vec) {
    int result_count = 0;

    clock_t start, end, pure_read, comp_start, comp_end, pure_comp;
    pure_read = 0;
    pure_comp = 0;
    clock_t start_all, end_all;
    int total_result_block_num = 0;
    start_all = clock();
    struct knn_result_buffer knn_result_buffer;
    init_knn_result_buffer(predicate->k, &knn_result_buffer);
    fill_points_buffer(points_buffer_size);

    // prepare batch parameters
    int batch_size = REQUEST_BATCH_SIZE; //  fix bug here -> fixed: caused by the same request used in the batch using the same lba_array (before, we do not use malloc for lba_array)
    void* result_buffer_vec[batch_size];
    size_t result_buffer_size_vec[batch_size];
    struct isp_descriptor *isp_desc_vec[batch_size];
    struct lba *lba_vec_ptr[batch_size];
    int batch_count = 0;

    int batch_function_call_num = 0;
    for (int i = 0; i < block_logical_addr_count; i += 256) {

        int block_addr_vec_size = block_logical_addr_count - i > 256 ? 256 : block_logical_addr_count - i;
        int result_size = block_addr_vec_size * 0x1000;


        if (result_size % 0x1000 != 0) {
            result_size = ((result_size / 0x1000) + 1) * 0x1000;
        }
        int estimated_result_block_num = result_size / 0x1000;
        //int estimated_result_block_num = 1;
        total_result_block_num += estimated_result_block_num;


        void* result_buffer = malloc(result_size);

        int block_meta_vec_size = calculate_aggregate_block_vec_size(&block_logical_addr_vec[i], block_addr_vec_size);
        struct continuous_block_meta block_meta_vec[block_meta_vec_size];
        aggregate_blocks(&block_logical_addr_vec[i], block_addr_vec_size, block_meta_vec, block_meta_vec_size);

        struct isp_descriptor *isp_desc = malloc(sizeof(struct isp_descriptor));
        struct lba *lba_vec = malloc(sizeof(struct lba) * block_meta_vec_size);
        assemble_lba_vec_via_block_meta(lba_vec, DATA_FILE_OFFSET, block_meta_vec, block_meta_vec_size);
        assemble_isp_desc_without_comp(isp_desc, estimated_result_block_num, lba_vec, block_meta_vec_size);

        //start = clock();
        result_buffer_vec[batch_count] = result_buffer;
        result_buffer_size_vec[batch_count] = result_size;
        isp_desc_vec[batch_count] = isp_desc;
        lba_vec_ptr[batch_count] = lba_vec;


        batch_count++;
        if (batch_count % batch_size == 0) {
            start = clock();
            do_isp_for_trajectory_data_without_comp_batch(batch_size, data_storage, result_buffer_vec, result_buffer_size_vec, isp_desc_vec);
            batch_function_call_num++;
            end = clock();

            for (int k = 0; k < batch_size; k++) {
                void* batch_base = result_buffer_vec[k];
                size_t batch_buffer_count = result_buffer_size_vec[k] / 0x1000;
                for (int j = 0; j < batch_buffer_count; j++) {
                    void* block_ptr = batch_base + j * TRAJ_BLOCK_SIZE;
                    comp_start = clock();
                    result_count += spatio_temporal_knn_query_raw_trajectory_block(block_ptr, predicate, &knn_result_buffer);
                    comp_end = clock();
                    pure_comp += (comp_end - comp_start);
                }
                //print_isp_descriptor(isp_desc_vec[k]);
            }
            batch_count = 0;

            pure_read += (end - start);
            for (int m = 0; m < batch_size; m++) {
                free(result_buffer_vec[m]);
                free(isp_desc_vec[m]);
                free(lba_vec_ptr[m]);
            }

        }

    }
    // handle the last batch (less than batch size)
    start = clock();
    do_isp_for_trajectory_data_without_comp_batch(batch_count, data_storage, result_buffer_vec, result_buffer_size_vec, isp_desc_vec);
    batch_function_call_num++;
    end = clock();

    for (int k = 0; k < batch_count; k++) {
        void* batch_base = result_buffer_vec[k];
        size_t batch_buffer_count = result_buffer_size_vec[k] / 0x1000;
        for (int j = 0; j < batch_buffer_count; j++) {
            void* block_ptr = batch_base + j * TRAJ_BLOCK_SIZE;
            comp_start = clock();
            result_count += spatio_temporal_knn_query_raw_trajectory_block(block_ptr, predicate, &knn_result_buffer);
            comp_end = clock();
            pure_comp += (comp_end - comp_start);
            //print_isp_descriptor(isp_desc_vec[k]);
        }
    }


    pure_read += (end - start);
    for (int m = 0; m < batch_count; m++) {
        free(result_buffer_vec[m]);
        free(isp_desc_vec[m]);
        free(lba_vec_ptr[m]);
    }

    combine_and_sort(&knn_result_buffer);

    end_all = clock();
    free_knn_result_buffer(&knn_result_buffer);
    free_points_buffer(points_buffer_size);
    printf("batch function call num: %d\n", batch_function_call_num);
    printf("[host multi batch] estimated result block number: %d\n", total_result_block_num);
    printf("[host multi batch] pure read time: %f\n",(double)pure_read);
    printf("[host multi batch] pure computation time: %f\n",(double)pure_comp);
    printf("[host multi batch] query time (including computation): %f\n", (double)(end_all - start_all));

    return (end_all - start_all);
}


int spatio_temporal_knn_query_with_pushdown_batch(struct simple_query_engine *engine, struct spatio_temporal_knn_predicate *predicate, int option, bool enable_host_index) {
    struct index_entry_storage *index_storage = &engine->index_storage;
    struct traj_storage *data_storage = &engine->data_storage;
    struct seg_meta_section_entry_storage *meta_storage = &engine->seg_meta_storage;

    // calculate the matched block num
    int block_logical_addr_count = 0;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        if (enable_host_index) {
            if (predicate->query_point.normalized_longitude <= entry->lon_max
                && predicate->query_point.normalized_longitude >= entry->lon_min
                && predicate->query_point.normalized_latitude <= entry->lat_max
                && predicate->query_point.normalized_latitude >= entry->lat_min
                ) {
                block_logical_addr_count++;
            }
        } else {
            block_logical_addr_count++;
        }
    }

    if (block_logical_addr_count == 0) {
        return 0;
    }

    // get the match block id vector
    //int block_logical_addr_vec[block_logical_addr_count];   // TODO use malloc for large vector
    int *block_logical_addr_vec = malloc(block_logical_addr_count * sizeof(int));

    int addr_vec_index = 0;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        if (enable_host_index) {
            if (predicate->query_point.normalized_longitude <= entry->lon_max
                && predicate->query_point.normalized_longitude >= entry->lon_min
                && predicate->query_point.normalized_latitude <= entry->lat_max
                && predicate->query_point.normalized_latitude >= entry->lat_min
                ) {
                block_logical_addr_vec[addr_vec_index] = entry->block_logical_adr;
                addr_vec_index++;
            }
        } else {
            block_logical_addr_vec[addr_vec_index] = entry->block_logical_adr;
            addr_vec_index++;
        }
    }


    printf("[isp cpu batch] read block num: %d\n", block_logical_addr_count);
    // run query
    int result_count = run_spatio_temporal_knn_query_device_batch(predicate, data_storage, block_logical_addr_count,
                                                   block_logical_addr_vec, option);

    free(block_logical_addr_vec);
    return result_count;
}

static int spatio_temporal_knn_query_isp_blocks(void* data_blocks, struct spatio_temporal_knn_predicate *predicate, struct knn_result_buffer *result_buffer) {
    int result_count = 0;
    memcpy(&result_count, data_blocks, 4);

    struct result_item *items_base = (struct result_item *)((char *)data_blocks + 4);   // output format is good, so do not need deserialization

    for (int i = 0; i < result_count; i++) {
        struct result_item tmp = items_base[i];
        //printf("oid: %d, lon: %d, lat: %d, time: %d\n", tmp.oid, tmp.normalized_longitude, tmp.normalized_latitude, tmp.timestamp_sec);
        add_item_to_buffer(result_buffer, &tmp);

    }
    return result_count;
}

int
run_spatio_temporal_knn_query_device_batch(struct spatio_temporal_knn_predicate *predicate, struct traj_storage *data_storage,
                                       int block_logical_addr_count, int *block_logical_addr_vec, int option) {
    int result_count = 0;
    int total_batch_func_num = 0;

    clock_t start, end, pure_read;
    pure_read = 0;
    clock_t start_all, end_all;
    int total_result_block_num = 0;
    start_all = clock();
    fill_points_buffer(points_buffer_size);

    struct knn_result_buffer knn_buffer;
    init_knn_result_buffer(predicate->k, &knn_buffer);

    // prepare batch parameters
    int batch_size = REQUEST_BATCH_SIZE; //  fix bug here -> fixed: caused by the same request used in the batch using the same lba_array (before, we do not use malloc for lba_array)
    void* result_buffer_vec[batch_size];
    size_t result_buffer_size_vec[batch_size];
    struct isp_descriptor *isp_desc_vec[batch_size];
    struct lba *lba_vec_ptr[batch_size];
    int batch_count = 0;

    for (int i = 0; i < block_logical_addr_count; i += 256) {

        int block_addr_vec_size = block_logical_addr_count - i > 256 ? 256 : block_logical_addr_count - i;
        int estimated_result_size = predicate->k * sizeof(struct result_item) + 4;


        if (estimated_result_size % 0x1000 != 0) {
            estimated_result_size = ((estimated_result_size / 0x1000) + 1) * 0x1000;
        }
        int estimated_result_block_num = estimated_result_size / 0x1000;
        //int estimated_result_block_num = 1;
        total_result_block_num += estimated_result_block_num;


        void* result_buffer = malloc(estimated_result_size);

        int block_meta_vec_size = calculate_aggregate_block_vec_size(&block_logical_addr_vec[i], block_addr_vec_size);
        struct continuous_block_meta block_meta_vec[block_meta_vec_size];
        aggregate_blocks(&block_logical_addr_vec[i], block_addr_vec_size, block_meta_vec, block_meta_vec_size);

        struct isp_descriptor *isp_desc = malloc(sizeof(struct isp_descriptor));
        struct lba *lba_vec = malloc(sizeof(struct lba) * block_meta_vec_size);
        assemble_lba_vec_via_block_meta(lba_vec, DATA_FILE_OFFSET, block_meta_vec, block_meta_vec_size);
        if (option == 1) {
            assemble_isp_desc_for_spatial_temporal_knn_naive(isp_desc, predicate, estimated_result_block_num, lba_vec,
                                                       block_meta_vec_size);
        } else if (option == 2) {
            assemble_isp_desc_for_spatial_temporal_knn_naive_add_mbr_pruning(isp_desc, predicate, estimated_result_block_num, lba_vec,
                                                             block_meta_vec_size);
        } else {
            assemble_isp_desc_for_spatial_temporal_knn(isp_desc, predicate, estimated_result_block_num, lba_vec,
                                                       block_meta_vec_size);
        }


        //start = clock();
        result_buffer_vec[batch_count] = result_buffer;
        result_buffer_size_vec[batch_count] = estimated_result_size;
        isp_desc_vec[batch_count] = isp_desc;
        lba_vec_ptr[batch_count] = lba_vec;


        batch_count++;
        if (batch_count % batch_size == 0) {
            start = clock();
            do_isp_for_trajectory_data_batch(batch_size, data_storage, result_buffer_vec, result_buffer_size_vec, isp_desc_vec, 0);
            end = clock();
            total_batch_func_num++;
            for (int k = 0; k < batch_size; k++) {
                void* batch_base = result_buffer_vec[k];

                result_count += spatio_temporal_knn_query_isp_blocks(batch_base, predicate, &knn_buffer);
            }
            batch_count = 0;

            pure_read += (end - start);
            for (int m = 0; m < batch_size; m++) {
                free(result_buffer_vec[m]);
                free(isp_desc_vec[m]);
                free(lba_vec_ptr[m]);
            }

        }

    }
    // handle the last batch (less than batch size)
    start = clock();
    do_isp_for_trajectory_data_batch(batch_count, data_storage, result_buffer_vec, result_buffer_size_vec, isp_desc_vec, 0);
    end = clock();
    total_batch_func_num++;
    for (int k = 0; k < batch_count; k++) {
        void* batch_base = result_buffer_vec[k];

        result_count += spatio_temporal_knn_query_isp_blocks(batch_base, predicate, &knn_buffer);
    }

    pure_read += (end - start);
    for (int m = 0; m < batch_count; m++) {
        free(result_buffer_vec[m]);
        free(isp_desc_vec[m]);
        free(lba_vec_ptr[m]);
    }


    combine_and_sort(&knn_buffer);

    /*for (int i = 0; i < knn_buffer.current_buffer_size; i++) {
        struct result_item item = knn_buffer.result_buffer_k[i];
        printf("oid: %d, dist: %d\n", item.point.oid, item.distance);
    }*/
    printf("result k value: %d\n", knn_buffer.current_buffer_size);
    print_runtime_statistics(&knn_buffer.statistics);
    free_knn_result_buffer(&knn_buffer);
    free_points_buffer(points_buffer_size);
    end_all = clock();
    printf("batch function call num: %d\n", total_batch_func_num);
    printf("[isp cpu batch] estimated result block number: %d\n", total_result_block_num);
    printf("[isp cpu batch] pure read time: %f\n",(double)pure_read);
    printf("[isp cpu batch] query time (including estimation): %f\n", (double)(end_all - start_all));

    return (end_all - start_all);
}

/**
 *  knn join queries
 */


int spatio_temporal_knn_join_query_without_pushdown_batch(struct simple_query_engine *engine, struct spatio_temporal_knn_join_predicate *predicate) {

    struct index_entry_storage *index_storage = &engine->index_storage;
    struct traj_storage *data_storage = &engine->data_storage;
    struct seg_meta_section_entry_storage *meta_storage = &engine->seg_meta_storage;

    int result_count = 0;

    int block_logical_addr_count = 0;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        block_logical_addr_count++;
    }

    // get the block id vector
    int *block_logical_addr_vec = malloc(block_logical_addr_count * sizeof(int) * 2);

    struct knnjoin_result_buffer knnjoin_buffer;
    init_knnjoin_result_buffer(predicate->k, &knnjoin_buffer);

    int addr_vec_index = 0;
    for (int i = 0; i <= 4; i++) {
        printf("data load i: %d\n", i);
        struct index_entry *entry1 = index_storage->index_entry_base[i];
        int block_logical_addr1 = entry1->block_logical_adr;
        for (int j = (index_storage->current_index / 4); j < index_storage->current_index; j++) {
            struct index_entry *entry2 = index_storage->index_entry_base[j];
            int block_logical_addr2 = entry2->block_logical_adr;
            block_logical_addr_vec[addr_vec_index] = block_logical_addr1;
            addr_vec_index++;
            block_logical_addr_vec[addr_vec_index] = block_logical_addr2;
            addr_vec_index++;
        }
        //run_knn_join_query_in_host_batch(predicate, data_storage, addr_vec_index, block_logical_addr_vec, &knnjoin_buffer);
        result_count += run_knn_join_query_in_host_fileapi(predicate, data_storage, addr_vec_index, block_logical_addr_vec, &knnjoin_buffer);

        addr_vec_index = 0;
    }


    combine_and_sort_knnjoin(&knnjoin_buffer);

    /*for (int i = 0; i < knnjoin_buffer.current_buffer_size; i++) {
        struct knnjoin_result_item item = knnjoin_buffer.knnjoin_result_buffer_k[i];
        printf("oid: %d, dist: %ld\n", item.point1.oid, item.distance);
    }*/
    printf("result k value: %d\n", knnjoin_buffer.current_buffer_size);
    print_runtime_statistics(&knnjoin_buffer.statistics);
    free_knnjoin_result_buffer(&knnjoin_buffer);



    free(block_logical_addr_vec);
    return result_count;
}



/**
 * we perform knn(r, data_block2) for every r belongs to data_block1
 * @param data_block1
 * @param data_block2
 * @param predicate
 * @param result_buffer
 * @return
 */

static int spatio_temporal_knn_join_query_raw_trajectory_block(void* data_block1, void* data_block2, struct spatio_temporal_knn_join_predicate *predicate, struct knnjoin_result_buffer *result_buffer) {
    int result_count = 0;

    int traj_point_size = get_traj_point_size();
    long current_max_dist = result_buffer->max_distance;

    struct traj_block_header block_header1, block_header2;
    parse_traj_block_for_header(data_block1, &block_header1);
    parse_traj_block_for_header(data_block2, &block_header2);

    struct seg_meta meta_array1[block_header1.seg_count];
    parse_traj_block_for_seg_meta_section(data_block1, meta_array1, block_header1.seg_count);
    struct seg_meta meta_array2[block_header2.seg_count];
    parse_traj_block_for_seg_meta_section(data_block2, meta_array2, block_header2.seg_count);

    int data_offset_in_block1 = get_header_size() + block_header1.seg_count * get_seg_meta_size();
    int points_num = 0;
    for (int i = 0; i < block_header1.seg_count; i++) {
        points_num += meta_array1[i].seg_size / traj_point_size;
    }
    struct traj_point *block1_points_ptr = (struct traj_point *)((char*) data_block1 + data_offset_in_block1);

    for (int i = 0; i < points_num; i++) {
        struct traj_point *query_point = &block1_points_ptr[i];
        //printf("query point: %d\n", query_point->oid);
        // pruning based on mbr
        long minmaxdist_min = LONG_MAX;
        /*for (int j = 0; j < block_header2.seg_count; j++) {
            struct seg_meta metaitem = meta_array2[j];
            long minmaxdist = cal_minmax_distance(query_point, &metaitem);
            if (minmaxdist < minmaxdist_min) {
                minmaxdist_min = minmaxdist;
            }
        }*/

        // for each point (block1_points_ptr[i]) in block1, we do the knn query
        for (int j = 0; j < block_header2.seg_count; j++) {
            struct seg_meta meta_item2 = meta_array2[j];
            long min_dist = cal_min_distance(query_point, &meta_item2);
            //if (min_dist <= current_max_dist && min_dist <= minmaxdist_min) {
            if (min_dist <= current_max_dist) {
                int data_seg_points_num = meta_item2.seg_size / traj_point_size;
                struct traj_point *point_ptr = (struct traj_point *)((char *) data_block2 + meta_item2.seg_offset);

                for (int k = 0; k < data_seg_points_num; k++) {
                    struct traj_point *point = &point_ptr[k];
                    long distance = cal_points_distance(query_point, point);
                    if (distance < result_buffer->max_distance) {
                        struct knnjoin_result_item item = {*query_point, *point, distance};
                        add_item_to_knnjoin_buffer(result_buffer, &item);
                        result_count++;
                    }
                }
            }
        }
    }
    return result_count;
}

/**
 * use the isp descriptor to send i/o address
 * for the blocks in the @param block_logical_addr_vec, they are organzaied for knn join, i.e., [0, 1], [2, 3], ..., [n, n+1], ... are block pairs where n is the vector index
 * @param predicate
 * @param data_storage
 * @param block_logical_addr_count
 * @param block_logical_addr_vec
 * @return
 */
int
run_knn_join_query_in_host_batch(struct spatio_temporal_knn_join_predicate *predicate, struct traj_storage *data_storage,
                            int block_logical_addr_count, int *block_logical_addr_vec, struct knnjoin_result_buffer *knnjoin_buffer) {

    int result_count = 0;

    clock_t start, end, pure_read, comp_start, comp_end, pure_comp;
    pure_read = 0;
    pure_comp = 0;
    clock_t start_all, end_all;
    int total_result_block_num = 0;
    start_all = clock();
    //fill_points_buffer(points_buffer_size);

    /*struct knnjoin_result_buffer knnjoin_buffer;
    init_knnjoin_result_buffer(predicate->k, &knnjoin_buffer);*/

    // prepare batch parameters
    int batch_size = REQUEST_BATCH_SIZE; //  fix bug here -> fixed: caused by the same request used in the batch using the same lba_array (before, we do not use malloc for lba_array)
    void* result_buffer_vec[batch_size];
    size_t result_buffer_size_vec[batch_size];
    struct isp_descriptor *isp_desc_vec[batch_size];
    struct lba *lba_vec_ptr[batch_size];
    int batch_count = 0;

    int batch_function_call_num = 0;
    for (int i = 0; i < block_logical_addr_count; i += 240) {   // because we set isp desc size to 4 kb, if we consider 256 blocks, in the worst case, the lba vec size is larger than 4096

        int block_addr_vec_size = block_logical_addr_count - i > 240 ? 240 : block_logical_addr_count - i;
        int result_size = block_addr_vec_size * 0x1000;


        if (result_size % 0x1000 != 0) {
            result_size = ((result_size / 0x1000) + 1) * 0x1000;
        }
        int estimated_result_block_num = result_size / 0x1000;
        //int estimated_result_block_num = 1;
        total_result_block_num += estimated_result_block_num;


        void* result_buffer = malloc(result_size);

        int block_meta_vec_size = calculate_aggregate_block_vec_size(&block_logical_addr_vec[i], block_addr_vec_size);
        struct continuous_block_meta block_meta_vec[block_meta_vec_size];
        aggregate_blocks(&block_logical_addr_vec[i], block_addr_vec_size, block_meta_vec, block_meta_vec_size);

        struct isp_descriptor *isp_desc = malloc(sizeof(struct isp_descriptor));
        struct lba *lba_vec = malloc(sizeof(struct lba) * block_meta_vec_size);
        assemble_lba_vec_via_block_meta(lba_vec, DATA_FILE_OFFSET, block_meta_vec, block_meta_vec_size);
        assemble_isp_desc_without_comp(isp_desc, estimated_result_block_num, lba_vec, block_meta_vec_size);

        int isp_descriptor_size = calculate_isp_descriptor_space(isp_desc);
        //printf("[run_knn_join_query_in_host_batch] isp descriptor size: %d\n", isp_descriptor_size);

        //start = clock();
        result_buffer_vec[batch_count] = result_buffer;
        result_buffer_size_vec[batch_count] = result_size;
        isp_desc_vec[batch_count] = isp_desc;
        lba_vec_ptr[batch_count] = lba_vec;


        batch_count++;
        if (batch_count % batch_size == 0) {
            start = clock();
            do_isp_for_trajectory_data_without_comp_batch(batch_size, data_storage, result_buffer_vec, result_buffer_size_vec, isp_desc_vec);
            batch_function_call_num++;
            end = clock();

            for (int k = 0; k < batch_size; k++) {
                void* batch_base = result_buffer_vec[k];
                size_t batch_buffer_count = result_buffer_size_vec[k] / 0x1000;
                for (int j = 0; j < batch_buffer_count; j+=2) {
                    void* block1_ptr = batch_base + j * TRAJ_BLOCK_SIZE;
                    void* block2_ptr = batch_base + (j+1) * TRAJ_BLOCK_SIZE;
                    comp_start = clock();
                    result_count += spatio_temporal_knn_join_query_raw_trajectory_block(block1_ptr, block2_ptr, predicate, knnjoin_buffer);
                    comp_end = clock();
                    pure_comp += (comp_end - comp_start);
                }
                //print_isp_descriptor(isp_desc_vec[k]);
            }
            batch_count = 0;

            pure_read += (end - start);
            for (int m = 0; m < batch_size; m++) {
                free(result_buffer_vec[m]);
                free(isp_desc_vec[m]);
                free(lba_vec_ptr[m]);
            }

        }

    }
    // handle the last batch (less than batch size)
    start = clock();
    do_isp_for_trajectory_data_without_comp_batch(batch_count, data_storage, result_buffer_vec, result_buffer_size_vec, isp_desc_vec);
    batch_function_call_num++;
    end = clock();

    for (int k = 0; k < batch_count; k++) {
        void* batch_base = result_buffer_vec[k];
        size_t batch_buffer_count = result_buffer_size_vec[k] / 0x1000;
        for (int j = 0; j < batch_buffer_count; j+=2) {
            void* block1_ptr = batch_base + j * TRAJ_BLOCK_SIZE;
            void* block2_ptr = batch_base + (j+1) * TRAJ_BLOCK_SIZE;
            comp_start = clock();
            result_count += spatio_temporal_knn_join_query_raw_trajectory_block(block1_ptr, block2_ptr, predicate, knnjoin_buffer);
            comp_end = clock();
            pure_comp += (comp_end - comp_start);
            //print_isp_descriptor(isp_desc_vec[k]);
        }
    }

    pure_read += (end - start);
    for (int m = 0; m < batch_count; m++) {
        free(result_buffer_vec[m]);
        free(isp_desc_vec[m]);
        free(lba_vec_ptr[m]);
    }


    //free_knnjoin_result_buffer(&knnjoin_buffer);
    //free_points_buffer(points_buffer_size);
    end_all = clock();
    printf("batch function call num: %d\n", batch_function_call_num);
    printf("[host batch] estimated result block number: %d\n", total_result_block_num);
    printf("[host batch] pure read time: %f\n",(double)pure_read);
    printf("[host batch] pure computation time: %f\n",(double)pure_comp);
    printf("[host batch] query time (including computation): %f\n", (double)(end_all - start_all));

    return result_count;

}


/**
 * use the original file api to send i/o address
 * for the blocks in the @param block_logical_addr_vec, they are organzaied for knn join, i.e., [0, 1], [2, 3], ..., [n, n+1], ... are block pairs where n is the vector index
 * @param predicate
 * @param data_storage
 * @param block_logical_addr_count
 * @param block_logical_addr_vec
 * @return
 */
int
run_knn_join_query_in_host_fileapi(struct spatio_temporal_knn_join_predicate *predicate, struct traj_storage *data_storage,
                                 int block_logical_addr_count, int *block_logical_addr_vec, struct knnjoin_result_buffer *knnjoin_buffer) {

    int result_count = 0;

    clock_t start, end, pure_read, comp_start, comp_end, pure_comp;
    pure_read = 0;
    pure_comp = 0;
    clock_t start_all, end_all;
    start_all = clock();

    char *buffer1 = malloc(TRAJ_BLOCK_SIZE);
    char *buffer2 = malloc(TRAJ_BLOCK_SIZE);
    int batch_function_call_num = 0;
    for (int i = 0; i < block_logical_addr_count; i += 2) {

        int block_logical_addr1 = block_logical_addr_vec[i];
        int block_logical_addr2 = block_logical_addr_vec[i+1];
        start = clock();
        fetch_continuous_traj_data_block(data_storage, block_logical_addr1, 1, buffer1);
        fetch_continuous_traj_data_block(data_storage, block_logical_addr2, 1, buffer2);
        end = clock();
        pure_read += (end - start);

        comp_start = clock();
        result_count += spatio_temporal_knn_join_query_raw_trajectory_block(buffer1, buffer2, predicate, knnjoin_buffer);
        comp_end = clock();
        pure_comp += (comp_end - comp_start);
    }
    free(buffer1);
    free(buffer2);

    end_all = clock();
    printf("[host batch] pure read time: %f\n",(double)pure_read);
    printf("[host batch] pure computation time: %f\n",(double)pure_comp);
    printf("[host batch] query time (including computation): %f\n", (double)(end_all - start_all));

    return result_count;

}

int spatio_temporal_knn_join_query_with_pushdown_batch(struct simple_query_engine *engine, struct spatio_temporal_knn_join_predicate *predicate, int option) {
    struct index_entry_storage *index_storage = &engine->index_storage;
    struct traj_storage *data_storage = &engine->data_storage;
    struct seg_meta_section_entry_storage *meta_storage = &engine->seg_meta_storage;

    int result_count = 0;

    int block_logical_addr_count = 0;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        block_logical_addr_count++;
    }

    // get the block id vector
    int *block_logical_addr_vec = malloc(block_logical_addr_count * sizeof(int) * 2);

    struct knnjoin_result_buffer knnjoin_buffer;
    init_knnjoin_result_buffer(predicate->k, &knnjoin_buffer);

    int addr_vec_index = 0;
    for (int i = 0; i <= 4; i++) {
        printf("data load i: %d\n", i);
        struct index_entry *entry1 = index_storage->index_entry_base[i];
        int block_logical_addr1 = entry1->block_logical_adr;
        for (int j = (index_storage->current_index / 4); j < index_storage->current_index; j++) {
            struct index_entry *entry2 = index_storage->index_entry_base[j];
            int block_logical_addr2 = entry2->block_logical_adr;
            block_logical_addr_vec[addr_vec_index] = block_logical_addr1;
            addr_vec_index++;
            block_logical_addr_vec[addr_vec_index] = block_logical_addr2;
            addr_vec_index++;
        }
        result_count += run_spatio_temporal_knn_join_query_device_batch(predicate, data_storage, addr_vec_index,
                                         block_logical_addr_vec, &knnjoin_buffer, option);

        addr_vec_index = 0;
    }

    combine_and_sort_knnjoin(&knnjoin_buffer);

    /*for (int i = 0; i < knnjoin_buffer.current_buffer_size; i++) {
        struct knnjoin_result_item item = knnjoin_buffer.knnjoin_result_buffer_k[i];
        printf("oid: %d, dist: %ld\n", item.point1.oid, item.distance);
    }*/
    printf("result k value: %d\n", knnjoin_buffer.current_buffer_size);

    free_knnjoin_result_buffer(&knnjoin_buffer);



    free(block_logical_addr_vec);
    return 0;
}

static int spatio_temporal_knn_join_query_isp_blocks(void* data_blocks, struct spatio_temporal_knn_join_predicate *predicate, struct knnjoin_result_buffer *buffer) {
    int result_count = 0;
    memcpy(&result_count, data_blocks, 4);

    struct knnjoin_result_item *items_base = (struct knnjoin_result_item *)((char *)data_blocks + 4);   // output format is good, so do not need deserialization

    for (int i = 0; i < result_count; i++) {
        struct knnjoin_result_item tmp = items_base[i];
        add_item_to_knnjoin_buffer(buffer, &tmp);
    }

    return result_count;
}

/**
 * for the blocks in the @param block_logical_addr_vec, they are organzaied for knn join, i.e., [0, 1], [2, 3], ..., [n, n+1], ... are block pairs where n is the vector index
 * @param predicate
 * @param data_storage
 * @param block_logical_addr_count
 * @param block_logical_addr_vec
 * @return
 */
int
run_spatio_temporal_knn_join_query_device_batch(struct spatio_temporal_knn_join_predicate *predicate, struct traj_storage *data_storage,
                                                int block_logical_addr_count, int *block_logical_addr_vec, struct knnjoin_result_buffer *knnjoin_buffer, int option) {
    int result_count = 0;

    clock_t start, end, pure_read, comp_start, comp_end, pure_comp;
    pure_read = 0;
    pure_comp = 0;
    clock_t start_all, end_all;
    int total_result_block_num = 0;
    start_all = clock();

    // prepare batch parameters
    int batch_size = REQUEST_BATCH_SIZE; //  fix bug here -> fixed: caused by the same request used in the batch using the same lba_array (before, we do not use malloc for lba_array)
    void* result_buffer_vec[batch_size];
    size_t result_buffer_size_vec[batch_size];
    struct isp_descriptor *isp_desc_vec[batch_size];
    struct lba *lba_vec_ptr[batch_size];
    int batch_count = 0;

    int batch_function_call_num = 0;
    for (int i = 0; i < block_logical_addr_count; i += 240) {   // because we set isp desc size to 4 kb, if we consider 256 blocks, in the worst case, the lba vec size is larger than 4096

        int block_addr_vec_size = block_logical_addr_count - i > 240 ? 240 : block_logical_addr_count - i;
        int estimated_result_size = predicate->k * sizeof(struct knnjoin_result_item) + 4;


        if (estimated_result_size % 0x1000 != 0) {
            estimated_result_size = ((estimated_result_size / 0x1000) + 1) * 0x1000;
        }
        int estimated_result_block_num = estimated_result_size / 0x1000;
        //int estimated_result_block_num = 1;
        total_result_block_num += estimated_result_block_num;


        void* result_buffer = malloc(estimated_result_size);

        int block_meta_vec_size = calculate_aggregate_block_vec_size(&block_logical_addr_vec[i], block_addr_vec_size);
        struct continuous_block_meta block_meta_vec[block_meta_vec_size];
        aggregate_blocks(&block_logical_addr_vec[i], block_addr_vec_size, block_meta_vec, block_meta_vec_size);

        struct isp_descriptor *isp_desc = malloc(sizeof(struct isp_descriptor));
        struct lba *lba_vec = malloc(sizeof(struct lba) * block_meta_vec_size);
        assemble_lba_vec_via_block_meta(lba_vec, DATA_FILE_OFFSET, block_meta_vec, block_meta_vec_size);

        if (option == 1) {
            assemble_isp_desc_for_spatial_temporal_knn_join_naive(isp_desc, predicate, estimated_result_block_num, lba_vec,
                                                            block_meta_vec_size);
        } else if (option == 2) {
            assemble_isp_desc_for_spatial_temporal_knn_join_naive_add_mbr_pruning(isp_desc, predicate, estimated_result_block_num, lba_vec,
                                                            block_meta_vec_size);
        } else {
            assemble_isp_desc_for_spatial_temporal_knn_join(isp_desc, predicate, estimated_result_block_num, lba_vec,
                                                            block_meta_vec_size);
        }
        //int isp_descriptor_size = calculate_isp_descriptor_space(isp_desc);
        //printf("[run_knn_join_query_in_host_batch] isp descriptor size: %d\n", isp_descriptor_size);

        //start = clock();
        result_buffer_vec[batch_count] = result_buffer;
        result_buffer_size_vec[batch_count] = estimated_result_size;
        isp_desc_vec[batch_count] = isp_desc;
        lba_vec_ptr[batch_count] = lba_vec;


        batch_count++;
        if (batch_count % batch_size == 0) {
            start = clock();
            do_isp_for_trajectory_data_batch(batch_size, data_storage, result_buffer_vec, result_buffer_size_vec, isp_desc_vec, 0);
            batch_function_call_num++;
            end = clock();

            for (int k = 0; k < batch_size; k++) {
                void* batch_base = result_buffer_vec[k];
                result_count += spatio_temporal_knn_join_query_isp_blocks(batch_base, predicate, knnjoin_buffer);

            }
            batch_count = 0;

            pure_read += (end - start);
            for (int m = 0; m < batch_size; m++) {
                free(result_buffer_vec[m]);
                free(isp_desc_vec[m]);
                free(lba_vec_ptr[m]);
            }

        }

    }
    // handle the last batch (less than batch size)
    start = clock();
    do_isp_for_trajectory_data_batch(batch_count, data_storage, result_buffer_vec, result_buffer_size_vec, isp_desc_vec, 0);
    batch_function_call_num++;
    end = clock();

    for (int k = 0; k < batch_count; k++) {
        void* batch_base = result_buffer_vec[k];
        result_count += spatio_temporal_knn_join_query_isp_blocks(batch_base, predicate, knnjoin_buffer);

    }

    pure_read += (end - start);
    for (int m = 0; m < batch_count; m++) {
        free(result_buffer_vec[m]);
        free(isp_desc_vec[m]);
        free(lba_vec_ptr[m]);
    }

    end_all = clock();
    printf("batch function call num: %d\n", batch_function_call_num);
    printf("[isp cpu batch] estimated result block number: %d\n", total_result_block_num);
    printf("[isp cpu batch] pure read time: %f\n",(double)pure_read);
    printf("[isp cpu batch] query time (including computation): %f\n", (double)(end_all - start_all));

    return result_count;
}
