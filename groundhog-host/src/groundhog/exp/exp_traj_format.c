//
// Created by yangguo on 23-6-29.
//
#include <stdio.h>
#include <stdlib.h>
#include "groundhog/simple_query_engine.h"
#include "groundhog/traj_block_format.h"
#include "groundhog/traj_processing.h"
#include "groundhog/config.h"
#include "groundhog/normalization_util.h"
#include "groundhog/bloom/bloom.h"
#include "groundhog/porto_dataset_reader.h"

static void generate_sequential_points(struct traj_point **points, int points_num) {
    for (int i = 0; i < points_num; i++) {
        points[i]->oid = 12;
        points[i]->normalized_longitude = i;
        points[i]->normalized_latitude = i;
        points[i]->timestamp_sec = i;

    }
}


static int prepare_synthetic_data_random(int block_num, void** block_buf_ptr_vec, int total_point_num) {
    int points_num = calculate_points_num_via_block_size(4096, SPLIT_SEGMENT_NUM);
    int point_count = 0;
    int i;
    for (i = 0; i < block_num; i++) {
        struct traj_point **points = allocate_points_memory(points_num);
        generate_synthetic_random_points(points, i*points_num, points_num, total_point_num);
        // convert and put this data to traj storage
        void *data = malloc(TRAJ_BLOCK_SIZE);
        sort_traj_points(points, points_num);
        do_self_contained_traj_block(points, points_num, data, TRAJ_BLOCK_SIZE);

        block_buf_ptr_vec[i] = data;
        free_points_memory(points, points_num);
        point_count += points_num;
        if (point_count > total_point_num) {
            break;
        }
    }
    return i;
}

/*static int prepare_synthetic_data_random(int block_num, void** block_buf_ptr_vec) {
    int points_num = calculate_points_num_via_block_size(4096, SPLIT_SEGMENT_NUM);
    int point_count = 0;
    int i;
    for (i = 0; i < block_num; i++) {
        struct traj_point **points = allocate_points_memory(points_num);
        generate_synthetic_random_points(points, i*points_num, points_num, 29491199);
        // convert and put this data to traj storage
        void *data = malloc(TRAJ_BLOCK_SIZE);
        sort_traj_points(points, points_num);
        do_self_contained_traj_block(points, points_num, data, TRAJ_BLOCK_SIZE);

        block_buf_ptr_vec[i] = data;
        free_points_memory(points, points_num);
        point_count += points_num;

    }
    return i;
}*/


static int prepare_synthetic_data(int block_num, void** block_buf_ptr_vec, int total_point_num) {
    int points_num = calculate_points_num_via_block_size(4096, SPLIT_SEGMENT_NUM);
    int point_count = 0;
    int i;
    for (i = 0; i < block_num; i++) {
        struct traj_point **points = allocate_points_memory(points_num);
        generate_sequential_points(points, points_num);
        // convert and put this data to traj storage
        void *data = malloc(TRAJ_BLOCK_SIZE);

        do_self_contained_traj_block(points, points_num, data, TRAJ_BLOCK_SIZE);

        block_buf_ptr_vec[i] = data;
        free_points_memory(points, points_num);
        point_count += points_num;
        if (point_count > total_point_num) {
            break;
        }
    }
    return i;
}

/*
static int spatio_temporal_query_raw_trajectory_block_new(void* data_block, struct spatio_temporal_range_predicate *predicate) {
    int result_count = 0;

    int seg_count = *((int*)data_block);

    struct traj_point *result_buffer = (struct traj_point *)malloc(TRAJ_BLOCK_SIZE);

    int traj_point_size = get_traj_point_size();

    void* meta_array_base = (char*)data_block + get_header_size();
    struct seg_meta *meta_array = (struct seg_meta *)meta_array_base;

    for (int j = 0; j < seg_count; j++) {

        struct seg_meta *meta_item = &meta_array[j];
        //print_seg_meta(meta_item);
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
*/

static int spatio_temporal_query_raw_trajectory_block_new_without_meta_filter(void* data_block, struct spatio_temporal_range_predicate *predicate) {
    int result_count = 0;

    int seg_count = *((int*)data_block);

    struct traj_point *result_buffer = (struct traj_point *)malloc(TRAJ_BLOCK_SIZE);
    int traj_point_size = get_traj_point_size();

    void* data_base = (char*)data_block + get_header_size() + seg_count * get_seg_meta_size();
    int point_num = calculate_points_num_via_block_size(TRAJ_BLOCK_SIZE, SPLIT_SEGMENT_NUM);
    struct traj_point *point_ptr = (struct traj_point *)(data_base);

    for (int j = 0; j < point_num; j++) {

            struct traj_point *point = &point_ptr[j];
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

    //printf("count: %d\n", result_count);
    free(result_buffer);
    return result_count;
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

void exp_segment_num_st() {
    int result_count = 0;
    int block_num = 131072;
    void* buf_ptr_vec[block_num];
    int prepared_block_num = prepare_synthetic_data_random(block_num, buf_ptr_vec, 27131904);
    printf("prepared block num: %d\n", prepared_block_num);
    printf("point num: %d\n", calculate_points_num_via_block_size(4096, 16));

    struct spatio_temporal_range_predicate predicate = {0 ,2713190 * 1, 0, 29491199, 0, 29491199};
    //struct spatio_temporal_range_predicate predicate = {0, 40, 0, 340, 0, 340};

    clock_t start = clock();
    for (int i = 0; i < prepared_block_num; i++) {
        result_count += spatio_temporal_query_raw_trajectory_block(buf_ptr_vec[i], &predicate);
    }
    clock_t end = clock();
    printf("result count: %d, time: %f s\n", result_count, (double )(end - start)/CLOCKS_PER_SEC);

    /*int result_count = 0;
    int block_num = 131072;
    void* buf_ptr_vec[block_num];
    prepare_synthetic_data_random(block_num, buf_ptr_vec);

    struct spatio_temporal_range_predicate predicate = {0 ,2949119 * 1, 0, 29491199, 0, 29491199};
    *//*struct spatio_temporal_range_predicate predicate = {normalize_longitude(-8.612973), normalize_longitude(-8.512973) ,
                                                        normalize_latitude(41.150043), normalize_latitude(41.250043), 1372670836, 1373275636};*//*

    clock_t start = clock();
    for (int i = 0; i < block_num; i++) {
        result_count += spatio_temporal_query_raw_trajectory_block(buf_ptr_vec[i], &predicate);
    }
    clock_t end = clock();
    printf("result count: %d, time: %f s\n", result_count, (double )(end - start)/CLOCKS_PER_SEC);*/
}

void exp_segment_num_id() {
    int result_count = 0;
    int block_num = 131072;
    void* buf_ptr_vec[block_num];
    int prepared_block_num = prepare_synthetic_data_random(block_num, buf_ptr_vec, 27131904);
    printf("prepared block num: %d\n", prepared_block_num);
    printf("point num: %d\n", calculate_points_num_via_block_size(4096, 16));

    struct id_temporal_predicate predicate = {12, 0 ,2713190 * 1};

    clock_t start = clock();
    for (int i = 0; i < prepared_block_num; i++) {
        result_count += id_temporal_query_raw_trajectory_block(buf_ptr_vec[i], &predicate);
    }
    clock_t end = clock();
    printf("result count: %d, time: %f s\n", result_count, (double )(end - start)/CLOCKS_PER_SEC);

}

void exp_segment_filtering() {
    int result_count = 0;
    int block_num = 131072;
    void* buf_ptr_vec[block_num];
    int prepared_block_num = prepare_synthetic_data(block_num, buf_ptr_vec, 27131904);
    printf("prepared block num: %d\n", prepared_block_num);
    printf("point num: %d\n", calculate_points_num_via_block_size(4096, 4));

    struct spatio_temporal_range_predicate predicate = {0, 24*2, 0, 340, 0, 340};

    clock_t start = clock();
    for (int i = 0; i < prepared_block_num; i++) {
        //result_count += spatio_temporal_query_raw_trajectory_block(buf_ptr_vec[i], &predicate);
        result_count += spatio_temporal_query_raw_trajectory_block_new_without_meta_filter(buf_ptr_vec[i], &predicate);
    }
    clock_t end = clock();
    printf("result count: %d, time: %f s\n", result_count, (double )(end - start)/CLOCKS_PER_SEC);

}


int main(void) {
    exp_segment_filtering();
    return 0;
}