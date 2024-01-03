//
// Created by yangguo on 11/22/22.
//
#include "groundhog/query_verification_util.h"
#include "groundhog/porto_dataset_reader.h"
#include "groundhog/config.h"

int verify_id_temporal_query(FILE *fp, struct id_temporal_predicate *predicate, int block_num) {
    int result_count = 0;
    int points_num = calculate_points_num_via_block_size(TRAJ_BLOCK_SIZE, SPLIT_SEGMENT_NUM);
    struct traj_point **points = NULL;
    for (int i = 0; i < block_num; i++) {
        points = allocate_points_memory(points_num);
        read_points_from_csv(fp, points, i*points_num, points_num);

        for (int j = 0; j < points_num; j++) {
            struct traj_point *point = points[j];
            if (point->oid == predicate->oid
                && predicate->time_min <= point->timestamp_sec
                && predicate->time_max >= point->timestamp_sec) {
                result_count++;
            }
        }
        free_points_memory(points, points_num);
    }

    return result_count;
}

int verify_spatio_temporal_query(FILE *fp, struct spatio_temporal_range_predicate *predicate, int block_num) {
    int result_count = 0;
    int points_num = calculate_points_num_via_block_size(TRAJ_BLOCK_SIZE, SPLIT_SEGMENT_NUM);
    struct traj_point **points = NULL;
    for (int i = 0; i < block_num; i++) {
        points = allocate_points_memory(points_num);
        read_points_from_csv(fp, points, i*points_num, points_num);

        for (int j = 0; j < points_num; j++) {
            struct traj_point *point = points[j];
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
    return result_count;
}
