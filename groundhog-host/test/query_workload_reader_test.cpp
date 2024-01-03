//
// Created by yangguo on 11/28/22.
//

#include "gtest/gtest.h"

extern "C" {
#include "groundhog/query_workload_reader.h"
#include "groundhog/config.h"
}

static void print_id_temporal_predicates(struct id_temporal_predicate **predicates, int array_size) {
    for (int i = 0; i < array_size; i++) {
        struct id_temporal_predicate *predicate = predicates[i];
        printf("id: %d, time_min: %d, time_max: %d\n", predicate->oid, predicate->time_min, predicate->time_max);
    }
    printf("\n");
}

static void print_spatio_temporal_predicates(struct spatio_temporal_range_predicate **predicates, int array_size) {
    for (int i = 0; i < array_size; i++) {
        struct spatio_temporal_range_predicate *predicate = predicates[i];
        printf("time_min: %d, time_max: %d, lon_min: %d, lon_max: %d, lat_min: %d, lat_max: %d\n", predicate->time_min, predicate->time_max, predicate->lon_min, predicate->lon_max, predicate->lat_min, predicate->lat_max);
    }
    printf("\n");
}

TEST(query_reader, idtemporal) {
    FILE *fp = fopen("/home/yangguo/Data/DataSet/Trajectory/TaxiPorto/archive/query-on-10w/porto_10w_id_24h.query", "r");
    struct id_temporal_predicate **predicates = allocate_id_temporal_predicate_mem(500);
    int result_count = read_id_temporal_queries_from_csv(fp, predicates, 500);
    printf("result count: %d\n", result_count);
    print_id_temporal_predicates(predicates, 500);
    free_id_temporal_predicate_mem(predicates, 500);
}

TEST(query_reader, spatiotemporal) {
    FILE *fp = fopen("/home/yangguo/Data/DataSet/Trajectory/TaxiPorto/archive/query-on-10w/porto_10w_1h_01.query", "r");
    struct spatio_temporal_range_predicate **predicates = allocate_spatio_temporal_predicate_mem(500);
    int result_count = read_spatio_temporal_queries_from_csv(fp, predicates, 500);
    printf("result count: %d\n", result_count);
    print_spatio_temporal_predicates(predicates, 500);
    free_spatio_temporal_predicate_mem(predicates, 500);

    int points_num = calculate_points_num_via_block_size(TRAJ_BLOCK_SIZE, SPLIT_SEGMENT_NUM);
    printf("points_num: %d\n", points_num);

}
