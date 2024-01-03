//
// Created by yangguo on 23-5-4.
//

#include <stdio.h>
#include "groundhog/simple_query_engine.h"
#include "groundhog/query_workload_reader.h"
#include "time.h"
#include <stdlib.h>
#include "groundhog/bloom/bloom.h"
#include "groundhog/config.h"
#include "groundhog/porto_dataset_reader.h"
#include "groundhog/traj_processing.h"
#include "groundhog/normalization_util.h"

int
loop(int result_count, const struct traj_point *points, const struct spatio_temporal_range_predicate *predicate, int i);

static void ingest_and_flush_synthetic_data(int data_block_num) {
    init_and_mk_fs_for_traj(false);


    // ingest data
    struct simple_query_engine query_engine;
    struct my_file data_file = {NULL, DATA_FILENAME, "w", SPDK_FS_MODE};
    struct my_file index_file = {NULL, INDEX_FILENAME, "w", SPDK_FS_MODE};
    struct my_file meta_file = {NULL, SEG_META_FILENAME, "w", SPDK_FS_MODE};
    init_query_engine_with_persistence(&query_engine, &data_file, &index_file, &meta_file);
    ingest_and_flush_synthetic_data_via_time_partition(&query_engine, data_block_num);

    // print temporal meta information for each block
    struct index_entry_storage *index_storage = &query_engine.index_storage;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        printf("block pointer: [%d], time min: %d, time max: %d\n", entry->block_logical_adr, entry->time_min, entry->time_max);
    }

    // save filesystem meta
    spdk_flush_static_fs_meta_for_traj();

}

static void exp_read_data_host(struct spatio_temporal_range_predicate *predicate) {
    init_and_mk_fs_for_traj(true);
    print_spdk_static_fs_meta_for_traj();

    struct simple_query_engine rebuild_engine;
    struct my_file data_file_rebuild = {NULL, DATA_FILENAME, "r", SPDK_FS_MODE};
    struct my_file index_file_rebuild = {NULL, INDEX_FILENAME, "r", SPDK_FS_MODE};
    struct my_file meta_file_rebuild = {NULL, SEG_META_FILENAME, "r", SPDK_FS_MODE};
    init_query_engine_with_persistence(&rebuild_engine, &data_file_rebuild, &index_file_rebuild, &meta_file_rebuild);


    clock_t start, end;

    start = clock();
    //fetch_continuous_traj_data_block(rebuild_engine.data_storage, 0, 256, );

    end = clock();
    printf("[host] read time: %f second\n", (double) (end - start));


    free_query_engine(&rebuild_engine);
}

void exp_computation_mem_rw() {

    srand(time(NULL));   // Initialization, should only be called once.


    int record_num = 29490975; // the point num of 512 MB data
    int result_count = 0;
    struct traj_point *points = malloc(sizeof(struct traj_point) * record_num);
    printf("result size (MB): %d\n", sizeof(struct traj_point) * record_num / 1024 / 1024);
    clock_t write_start = clock();
    for (int i = 0; i < record_num; i++) {
        //int random_value = rand() % record_num;
        int random_value = i;
        points[i].normalized_latitude = random_value;
        points[i].normalized_longitude = random_value;
        points[i].timestamp_sec = random_value;
        points[i].oid = random_value;
        //printf("value: %d\n", random_value);
    }
    clock_t write_end = clock();
    struct spatio_temporal_range_predicate *predicate = malloc(sizeof(*predicate));

    int predicate_value = 2949119*5;
    predicate->lat_max = predicate_value;
    predicate->lat_min = 0;
    predicate->lon_max = predicate_value;
    predicate->lon_min = 0;
    predicate->time_max = predicate_value;
    predicate->time_min = 0;

    clock_t start, stop;

    void* output_buffer = malloc(sizeof(struct traj_point) * record_num);
    struct traj_point* traj_output = output_buffer;
    //struct traj_point *point = &points[0];


    start = clock();
    for (int k = 0; k < record_num; k++) {

        struct traj_point *point = &points[k];

        if (predicate->lon_min <= point->normalized_longitude
            && predicate->lon_max >= point->normalized_longitude
            && predicate->lat_min <= point->normalized_latitude
            && predicate->lat_max >= point->normalized_latitude
            && predicate->time_min <= point->timestamp_sec
            && predicate->time_max >= point->timestamp_sec) {

            /*traj_output[result_count].oid = point->oid;
            traj_output[result_count].normalized_longitude = point->normalized_longitude;
            traj_output[result_count].normalized_latitude = point->normalized_latitude;
            traj_output[result_count].timestamp_sec = point->timestamp_sec;*/
            //result_count++;

        }

    }
    stop = clock();

    clock_t read_start = clock();
    for (int k = 0; k < record_num; k++) {

        struct traj_point *point = &points[k];
        point->normalized_longitude;
        point->normalized_longitude;
        point->normalized_latitude;
        point->normalized_latitude;
        point->timestamp_sec;
        point->timestamp_sec;

    }
    clock_t read_stop = clock();

    printf("write time: %d\n", (write_end - write_start));
    printf("read time: %d\n", (read_stop - read_start));
    printf("result count: %d, computation + read time: %f\n", result_count, (double)(stop - start));
}

void exp_computation() {

    srand(time(NULL));   // Initialization, should only be called once.


    int record_num = 29491190;
    int result_count = 0;
    struct traj_point *points = malloc(sizeof(struct traj_point) * record_num);
    for (int i = 0; i < record_num; i++) {
        //int random_value = rand() % record_num;
        int random_value = i;
        points[i].normalized_latitude = random_value;
        points[i].normalized_longitude = random_value;
        points[i].timestamp_sec = random_value;
        points[i].oid = random_value;
        //printf("value: %d\n", random_value);
    }
    struct spatio_temporal_range_predicate *predicate = malloc(sizeof(*predicate));
    int predicate_value = 2949119*10;
    predicate->lat_max = predicate_value;
    predicate->lat_min = 0;
    predicate->lon_max = predicate_value;
    predicate->lon_min = 0;
    predicate->time_max = predicate_value;
    predicate->time_min = 0;

    clock_t start, stop;

    void* output_buffer = malloc(sizeof(struct traj_point) * record_num);
    struct traj_point* traj_output = output_buffer;
    //struct traj_point *point = &points[0];
    start = clock();
    for (int k = 0; k < record_num; k++) {
        struct traj_point *point = &points[k];
        if (predicate->lon_min <= point->normalized_longitude
            && predicate->lon_max >= point->normalized_longitude
            && predicate->lat_min <= point->normalized_latitude
            && predicate->lat_max >= point->normalized_latitude
            && predicate->time_min <= point->timestamp_sec
            && predicate->time_max >= point->timestamp_sec) {

            traj_output[result_count].oid = point->oid;
            traj_output[result_count].normalized_longitude = point->normalized_longitude;
            traj_output[result_count].normalized_latitude = point->normalized_latitude;
            traj_output[result_count].timestamp_sec = point->timestamp_sec;
            result_count++;

        }

    }
    stop = clock();

    printf("result count: %d, total computation time: %f\n", result_count, (double)(stop - start));
}

void exp_computation_motivation() {

    srand(time(NULL));   // Initialization, should only be called once.


    int record_num = 29491190;
    int result_count = 0;
    struct traj_point *points = malloc(sizeof(struct traj_point) * record_num);
    for (int i = 0; i < record_num; i++) {
        //int random_value = rand() % record_num;
        int random_value = i;
        points[i].normalized_latitude = random_value;
        points[i].normalized_longitude = random_value;
        points[i].timestamp_sec = random_value;
        points[i].oid = random_value;
        //printf("value: %d\n", random_value);
    }
    struct spatio_temporal_range_predicate *predicate = malloc(sizeof(*predicate));
    int predicate_value = 2949119*10;
    predicate->lat_max = predicate_value;
    predicate->lat_min = 0;
    predicate->lon_max = predicate_value;
    predicate->lon_min = 0;
    predicate->time_max = predicate_value;
    predicate->time_min = 0;

    clock_t start, stop;

    void* output_buffer = malloc(sizeof(struct traj_point) * record_num);
    struct traj_point* traj_output = output_buffer;
    //struct traj_point *point = &points[0];
    start = clock();
    for (int k = 0; k < record_num; k++) {
        struct traj_point *point = &points[k];
        if (predicate->lon_min <= point->normalized_longitude
            && predicate->lon_max >= point->normalized_longitude
            && predicate->lat_min <= point->normalized_latitude
            && predicate->lat_max >= point->normalized_latitude
            && predicate->time_min <= point->timestamp_sec
            && predicate->time_max >= point->timestamp_sec) {

            traj_output[result_count].oid = point->oid;
            traj_output[result_count].normalized_longitude = point->normalized_longitude;
            traj_output[result_count].normalized_latitude = point->normalized_latitude;
            traj_output[result_count].timestamp_sec = point->timestamp_sec;
            result_count++;

        }

    }
    stop = clock();

    printf("result count: %d, total computation time: %f\n", result_count, (double)(stop - start));
}

void exp_computation_host_cpu() {

    srand(123);   // Initialization, should only be called once.


    int record_num = 29491199;
    int result_count = 0;
    struct traj_point *points = malloc(sizeof(struct traj_point) * record_num);
    for (int i = 0; i < record_num; i++) {
        int random_value = rand() % record_num;
        //int random_value = i;
        points[i].normalized_latitude = random_value;
        points[i].normalized_longitude = random_value;
        points[i].timestamp_sec = random_value;
        points[i].oid = random_value;
        //printf("value: %d\n", random_value);
    }
    struct spatio_temporal_range_predicate *predicate = malloc(sizeof(*predicate));
    int predicate_value = 2949119 * 10;
    predicate->lat_max = predicate_value;
    predicate->lat_min = 0;
    predicate->lon_max = predicate_value;
    predicate->lon_min = 0;
    predicate->time_max = predicate_value;
    predicate->time_min = 0;

    clock_t start, stop;

    void* output_buffer = malloc(sizeof(struct traj_point) * record_num);
    struct traj_point* traj_output = output_buffer;
    //struct traj_point *point = &points[0];
    struct traj_point tmp_point;
    start = clock();
    for (int k = 0; k < record_num; k++) {
        struct traj_point *point = &points[k];
        if (predicate->lon_min <= point->normalized_longitude
            && predicate->lon_max >= point->normalized_longitude
            && predicate->lat_min <= point->normalized_latitude
            && predicate->lat_max >= point->normalized_latitude
            && predicate->time_min <= point->timestamp_sec
            && predicate->time_max >= point->timestamp_sec) {

            traj_output[result_count] = *point;
            /*tmp_point.oid = point->oid; // simulate the operation to write result to output buffer
            tmp_point.normalized_longitude = point->normalized_longitude;
            tmp_point.normalized_latitude = point->normalized_latitude;
            tmp_point.timestamp_sec = point->timestamp_sec;*/
            result_count++;

        }

    }
    stop = clock();

    printf("result count: %d, total computation time: %f\n", result_count, (double)(stop - start));
}

static void prepare_data(int block_num, void** block_buf_ptr_vec) {
    int points_num = calculate_points_num_via_block_size(TRAJ_BLOCK_SIZE, SPLIT_SEGMENT_NUM);

    for (int i = 0; i < block_num; i++) {
        struct traj_point **points = allocate_points_memory(points_num);
        //generate_synthetic_points(points, i*points_num, points_num);

        generate_synthetic_random_points(points, i*points_num, points_num, 29491199);
        // convert and put this data to traj storage
        void *data = malloc(TRAJ_BLOCK_SIZE);
        sort_traj_points(points, points_num);
        do_self_contained_traj_block(points, points_num, data, TRAJ_BLOCK_SIZE);

        block_buf_ptr_vec[i] = data;
        free_points_memory(points, points_num);
    }
}

static void prepare_data_porto_zcurve(int block_num, void** block_buf_ptr_vec) {
    int points_num = calculate_points_num_via_block_size(TRAJ_BLOCK_SIZE, SPLIT_SEGMENT_NUM);
    FILE *data_fp = fopen("/home/yangguo/Dataset/trajectory/porto_data_v2.csv", "r");
    for (int i = 0; i < block_num; i++) {
        struct traj_point **points = allocate_points_memory(points_num);

        read_points_from_csv(data_fp, points, i*points_num, points_num);
        //generate_synthetic_random_points(points, i*points_num, points_num, 29491199);
        // convert and put this data to traj storage
        void *data = malloc(TRAJ_BLOCK_SIZE);
        sort_traj_points_zcurve(points, points_num);
        do_self_contained_traj_block(points, points_num, data, TRAJ_BLOCK_SIZE);

        block_buf_ptr_vec[i] = data;
        free_points_memory(points, points_num);
    }
}

static int spatio_temporal_query_raw_trajectory_block(void* data_block, struct spatio_temporal_range_predicate *predicate) {
    int result_count = 0;
    struct traj_block_header block_header;
    parse_traj_block_for_header(data_block, &block_header);
    struct traj_point *result_buffer = (struct traj_point *)malloc(TRAJ_BLOCK_SIZE);

    int traj_point_size = get_traj_point_size();
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

static int spatio_temporal_query_raw_trajectory_block_new(void* data_block, struct spatio_temporal_range_predicate *predicate) {
    int result_count = 0;

    int seg_count = *((int*)data_block);

    struct traj_point *result_buffer = (struct traj_point *)malloc(TRAJ_BLOCK_SIZE);

    int traj_point_size = get_traj_point_size();

    void* meta_array_base = (char*)data_block + get_header_size();
    struct seg_meta *meta_array = (struct seg_meta *)meta_array_base;

    for (int j = 0; j < seg_count; j++) {

        struct seg_meta *meta_item = &meta_array[j];
        if (predicate->time_min <= meta_item->time_max && predicate->time_max >= meta_item->time_min
            && predicate->lon_min <= meta_item->lon_max && predicate->lon_max >= meta_item->lon_min
            && predicate->lat_min <= meta_item->lat_max && predicate->lat_max >= meta_item->lat_min) {
            int data_seg_points_num = meta_item->seg_size / traj_point_size;


            struct traj_point *point_ptr = (struct traj_point *)((char *) data_block + meta_item->seg_offset);

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

static int spatio_temporal_query_raw_trajectory_block_new_opt(void* data_block, struct spatio_temporal_range_predicate *predicate) {
    int result_count = 0;

    int seg_count = *((int*)data_block);

    struct traj_point *result_buffer = (struct traj_point *)malloc(TRAJ_BLOCK_SIZE);

    int traj_point_size = get_traj_point_size();

    void* meta_array_base = (char*)data_block + get_header_size();
    struct seg_meta *meta_array = (struct seg_meta *)meta_array_base;

    for (int j = 0; j < seg_count; j++) {

        struct seg_meta *meta_item = &meta_array[j];
        if (predicate->time_min <= meta_item->time_max && predicate->time_max >= meta_item->time_min
            && predicate->lon_min <= meta_item->lon_max && predicate->lon_max >= meta_item->lon_min
            && predicate->lat_min <= meta_item->lat_max && predicate->lat_max >= meta_item->lat_min) {
            int data_seg_points_num = meta_item->seg_size / traj_point_size;
            struct traj_point *point_ptr = (struct traj_point *)((char *) data_block + meta_item->seg_offset);

            if (meta_item->time_min >= predicate->time_min && meta_item->time_max <= predicate->time_max
                && meta_item->lon_min >= predicate->lon_min && meta_item->lon_max <= predicate->lon_max
                && meta_item->lat_min >= predicate->lat_min && meta_item->lat_max <= predicate->lat_max) {
                //fully contained
                memcpy(&result_buffer[result_count], point_ptr, data_seg_points_num * traj_point_size);
                //printf("i am fully contained\n");
            } else {

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
    }

    free(result_buffer);
    return result_count;
}


static int spatio_temporal_query_raw_trajectory_block_empty(void* data_block, struct spatio_temporal_range_predicate *predicate) {
    int result_count = 0;
    /*struct traj_block_header block_header;
    parse_traj_block_for_header(data_block, &block_header);
    struct traj_point *result_buffer = (struct traj_point *)malloc(TRAJ_BLOCK_SIZE);

    int traj_point_size = get_traj_point_size();
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

    free(result_buffer);*/
    return result_count;
}

static int spatio_temporal_query_raw_trajectory_block_s1(void* data_block, struct spatio_temporal_range_predicate *predicate) {
    int result_count = 0;
    struct traj_block_header block_header;
    parse_traj_block_for_header(data_block, &block_header);
    struct traj_point *result_buffer = (struct traj_point *)malloc(TRAJ_BLOCK_SIZE);

    int traj_point_size = get_traj_point_size();
    struct seg_meta meta_array[block_header.seg_count];
    parse_traj_block_for_seg_meta_section(data_block, meta_array, block_header.seg_count);
    /*for (int j = 0; j < block_header.seg_count; j++) {
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

    free(result_buffer);*/
    return result_count;

}


static int spatio_temporal_query_raw_trajectory_block_s1_s2(void* data_block, struct spatio_temporal_range_predicate *predicate) {
    int result_count = 0;
    struct traj_block_header block_header;
    parse_traj_block_for_header(data_block, &block_header);
    struct traj_point *result_buffer = (struct traj_point *)malloc(TRAJ_BLOCK_SIZE);

    int traj_point_size = get_traj_point_size();
    struct seg_meta meta_array[block_header.seg_count];
    parse_traj_block_for_seg_meta_section(data_block, meta_array, block_header.seg_count);
    for (int j = 0; j < block_header.seg_count; j++) {
        struct seg_meta meta_item = meta_array[j];
        if (predicate->time_min <= meta_item.time_max && predicate->time_max >= meta_item.time_min
            && predicate->lon_min <= meta_item.lon_max && predicate->lon_max >= meta_item.lon_min
            && predicate->lat_min <= meta_item.lat_max && predicate->lat_max >= meta_item.lat_min) {
            /*int data_seg_points_num = meta_item.seg_size / traj_point_size;


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
            }*/

        }
    }

    free(result_buffer);
    return result_count;
}

static int spatio_temporal_query_raw_trajectory_block_s1_s2_s3(void* data_block, struct spatio_temporal_range_predicate *predicate) {
    int result_count = 0;
    struct traj_block_header block_header;
    parse_traj_block_for_header(data_block, &block_header);
    struct traj_point *result_buffer = (struct traj_point *)malloc(TRAJ_BLOCK_SIZE);

    int traj_point_size = get_traj_point_size();
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
                    /*result_buffer[result_count] = *point;
                    result_count++;*/

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
                if (predicate->oid <= point->oid
                    && predicate->time_min <= point->timestamp_sec
                    && predicate->time_max >= point->timestamp_sec) {
                    result_buffer[result_count] = *point;
                    result_count++;

                }
            }

        }
    }
    return result_count;
}


static int id_temporal_query_raw_trajectory_block_new(void* data_block, struct id_temporal_predicate *predicate) {
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
                if (predicate->oid <= point->oid
                    && predicate->time_min <= point->timestamp_sec
                    && predicate->time_max >= point->timestamp_sec) {
                    result_buffer[result_count] = *point;
                    result_count++;

                }
            }

        }
    }
    return result_count;
}

static int id_temporal_query_raw_trajectory_block_empty(void* data_block, struct id_temporal_predicate *predicate) {
    int result_count = 0;
    /*struct traj_block_header block_header;
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
                if (predicate->oid <= point->oid
                    && predicate->time_min <= point->timestamp_sec
                    && predicate->time_max >= point->timestamp_sec) {
                    result_buffer[result_count] = *point;
                    result_count++;

                }
            }

        }
    }*/
    return result_count;
}

static int id_temporal_query_raw_trajectory_block_s1(void* data_block, struct id_temporal_predicate *predicate) {
    int result_count = 0;
    struct traj_block_header block_header;
    struct traj_point *result_buffer = (struct traj_point *)malloc(TRAJ_BLOCK_SIZE);
    int traj_point_size = get_traj_point_size();

    parse_traj_block_for_header(data_block, &block_header);
    struct seg_meta meta_array[block_header.seg_count];
    parse_traj_block_for_seg_meta_section(data_block, meta_array, block_header.seg_count);
    /*for (int j = 0; j < block_header.seg_count; j++) {
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
                if (predicate->oid <= point->oid
                    && predicate->time_min <= point->timestamp_sec
                    && predicate->time_max >= point->timestamp_sec) {
                    result_buffer[result_count] = *point;
                    result_count++;

                }
            }

        }
    }
     */
    free(result_buffer);
    return result_count;
}

static int id_temporal_query_raw_trajectory_block_s1_s2(void* data_block, struct id_temporal_predicate *predicate) {
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
            /*int data_seg_points_num = meta_item.seg_size / traj_point_size;


            struct traj_point *point_ptr = (struct traj_point *)((char *) data_block + meta_item.seg_offset);

            for (int k = 0; k < data_seg_points_num; k++) {
                struct traj_point *point = &point_ptr[k];
                if (predicate->oid <= point->oid
                    && predicate->time_min <= point->timestamp_sec
                    && predicate->time_max >= point->timestamp_sec) {
                    result_buffer[result_count] = *point;
                    result_count++;

                }
            }*/

        }
    }
    free(result_buffer);
    return result_count;
}

static int id_temporal_query_raw_trajectory_block_s1_s2_s3(void* data_block, struct id_temporal_predicate *predicate) {
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
                if (predicate->oid <= point->oid
                    && predicate->time_min <= point->timestamp_sec
                    && predicate->time_max >= point->timestamp_sec) {
                    /*result_buffer[result_count] = *point;
                    result_count++;*/

                }
            }

        }
    }
    free(result_buffer);
    return result_count;
}

void do_cost_model_test() {
    /*int seg_count = 10;
    void* data_buffer = malloc(4096);
    memset(data_buffer, 0, 4096);
    memcpy(data_buffer, &seg_count, sizeof(int));*/
    int block_num = 1;
    void* buf_ptr_vec[block_num];
    prepare_data(block_num, buf_ptr_vec);

    struct spatio_temporal_range_predicate predicate = {0 ,1000, 0, 1000, 0, 1000};
    int result_count = spatio_temporal_query_raw_trajectory_block_empty(buf_ptr_vec[0], &predicate);
    printf("result count: %d\n", result_count);

    //free(data_buffer);
}

void verify_cost_model() {
    int result_count = 0;
    int block_num = 131072;
    void* buf_ptr_vec[block_num];
    prepare_data(block_num, buf_ptr_vec);

    struct spatio_temporal_range_predicate predicate = {0 ,2949119 * 1, 0, 29491199, 0, 29491199};
    /*struct spatio_temporal_range_predicate predicate = {normalize_longitude(-8.612973), normalize_longitude(-8.512973) ,
                                                        normalize_latitude(41.150043), normalize_latitude(41.250043), 1372670836, 1373275636};
*/
    clock_t start = clock();
    for (int i = 0; i < block_num; i++) {
         result_count += spatio_temporal_query_raw_trajectory_block(buf_ptr_vec[i], &predicate);
    }
    clock_t end = clock();
    printf("result count: %d, time: %f s\n", result_count, (double )(end - start)/CLOCKS_PER_SEC);
}


void verify_cost_model_id() {
    int result_count = 0;
    int block_num = 131072;
    void* buf_ptr_vec[block_num];
    prepare_data(block_num, buf_ptr_vec);

    struct id_temporal_predicate predicate = {12, 0 ,2949119 * 1};

    clock_t start = clock();
    for (int i = 0; i < block_num; i++) {
        result_count += id_temporal_query_raw_trajectory_block(buf_ptr_vec[i], &predicate);
    }
    clock_t end = clock();
    printf("result count: %d, time: %f s\n", result_count, (double )(end - start)/CLOCKS_PER_SEC);
}

double cost_model_host_st(int block_number, double selectivity) {
    long data_size = block_number * 4096;    // unit is kb
    double bandwidth = 767.42;  // MB/s
    double transfer_time = data_size / (1024.0 * 1024) / bandwidth;

    int meta_num = block_number * 10;
    int tuple_num = block_number * 225;
    double computation_time = (block_number * 3196.0 + meta_num * 64.0 + tuple_num * 40.0 * selectivity + tuple_num * 19.0 * selectivity) / (4*3600000000.0);
    return transfer_time + computation_time;
}

double cost_model_device_st(int block_number, double selectivity) {
    long data_size = block_number * 4096;    // unit is kb
    double bandwidth = 767.42;  // MB/s
    double transfer_time = (data_size * selectivity) / (1024.0 * 1024) / bandwidth;

    int meta_num = block_number * 10;
    int tuple_num = block_number * 225;
    double computation_time = (block_number * 301.0 + tuple_num * ((5996-301)/225) * selectivity)  / (1200000000.0);
    double ftl_time = 0.230631;
    return transfer_time + computation_time + ftl_time;
}

double cost_model_host_id(int block_number, double selectivity) {
    long data_size = block_number * 4096;    // unit is kb
    double bandwidth = 767.42;  // MB/s
    double transfer_time = data_size / (1024.0 * 1024) / bandwidth;

    int meta_num = block_number * 10;
    int tuple_num = block_number * 225;
    double computation_time = (block_number * 3312.0 + meta_num * 401.0 + tuple_num * 22.0 * selectivity + tuple_num * 18.0 * selectivity) / (4*3600000000.0);
    return transfer_time + computation_time;
}

double cost_model_device_id(int block_number, double selectivity) {
    long data_size = block_number * 4096;    // unit is kb
    double bandwidth = 767.42;  // MB/s
    double transfer_time = (data_size * selectivity) / (1024.0 * 1024) / bandwidth;

    int meta_num = block_number * 10;
    int tuple_num = block_number * 225;
    double computation_time = (block_number * 301.0 + tuple_num * ((7314-301)/225) * selectivity)  / (1200000000.0);
    double ftl_time = 0.230631;
    return transfer_time + computation_time + ftl_time;
}

void get_estimated_cost_host_st() {
    for (double s = 0; s <=1; s+=0.1) {
        double cost = cost_model_host_st(131072, s);
        printf("selectivity: %f, cost:%f\n", s, cost);
    }
}

void get_estimated_cost_host_id() {
    for (double s = 0; s <=1; s+=0.1) {
        double cost = cost_model_host_id(131072, s);
        printf("selectivity: %f, cost:%f\n", s, cost);
    }
}

void get_estimated_cost_device_st() {
    for (double s = 0; s <=1; s+=0.1) {
        double cost = cost_model_device_st(131072, s);
        printf("selectivity: %f, cost:%f\n", s, cost);
    }
}

void get_estimated_cost_device_id() {
    for (double s = 0; s <=1; s+=0.1) {
        double cost = cost_model_device_id(131072, s);
        printf("selectivity: %f, cost:%f\n", s, cost);
    }
}

int main(void) {
    verify_cost_model();
    return 0;
}