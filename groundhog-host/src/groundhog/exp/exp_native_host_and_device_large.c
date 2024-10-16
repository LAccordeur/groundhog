//
// Created by yangguo on 23-6-6.
//

#include <stdio.h>
#include "groundhog/simple_query_engine.h"
#include "groundhog/query_workload_reader.h"
#include "time.h"
#include "groundhog/config.h"
#include "groundhog/normalization_util.h"
#include "groundhog/common_util.h"

static bool enable_host_index = true;

static void ingest_and_flush_synthetic_data_large(int data_block_num_each_run, int run_num) {
    init_and_mk_fs_for_traj(false);

    // ingest data
    struct simple_query_engine query_engine;
    struct my_file data_file = {NULL, DATA_FILENAME, "w", SPDK_FS_MODE};
    struct my_file index_file = {NULL, INDEX_FILENAME, "w", SPDK_FS_MODE};
    struct my_file meta_file = {NULL, SEG_META_FILENAME, "w", SPDK_FS_MODE};
    init_query_engine_with_persistence(&query_engine, &data_file, &index_file, &meta_file);

    for (int i = 0; i < run_num; i++) {
        printf("the %d run of data ingestion\n", i);
        init_query_engine_with_persistence(&query_engine, &data_file, &index_file, &meta_file);
        ingest_and_flush_synthetic_data_via_time_partition_with_block_index(&query_engine, i * data_block_num_each_run, data_block_num_each_run);
        free_query_engine(&query_engine);
    }


    print_spdk_static_fs_meta_for_traj();
    // save filesystem meta
    spdk_flush_static_fs_meta_for_traj();
}

static void ingest_and_flush_osm_data_zcurve_large(int data_block_num_each_run, int run_num) {
    init_and_mk_fs_for_traj(false);

    FILE *data_fp = fopen("/home/yangguo/Dataset/osm/osm_points_v1.csv", "r");
    // ingest data
    struct simple_query_engine query_engine;
    struct my_file data_file = {NULL, DATA_FILENAME, "w", SPDK_FS_MODE};
    struct my_file index_file = {NULL, INDEX_FILENAME, "w", SPDK_FS_MODE};
    struct my_file meta_file = {NULL, SEG_META_FILENAME, "w", SPDK_FS_MODE};
    init_query_engine_with_persistence(&query_engine, &data_file, &index_file, &meta_file);

    for (int i = 0; i < run_num; i++) {
        printf("the %d run of data ingestion\n", i);
        init_query_engine_with_persistence(&query_engine, &data_file, &index_file, &meta_file);
        ingest_and_flush_osm_data_via_zcurve_partition_with_block_index(&query_engine, data_fp, i * data_block_num_each_run, data_block_num_each_run);
        free_query_engine(&query_engine);
    }


    print_spdk_static_fs_meta_for_traj();
    // save filesystem meta
    spdk_flush_static_fs_meta_for_traj();
}

static void ingest_and_flush_osm_data_zcurve_large_with_sort_option(int data_block_num_each_run, int run_num, int sort_option) {
    init_and_mk_fs_for_traj(false);

    FILE *data_fp = fopen("/home/yangguo/Dataset/osm/osm_points_with_time_v1.csv", "r");
    // ingest data
    struct simple_query_engine query_engine;
    struct my_file data_file = {NULL, DATA_FILENAME, "w", SPDK_FS_MODE};
    struct my_file index_file = {NULL, INDEX_FILENAME, "w", SPDK_FS_MODE};
    struct my_file meta_file = {NULL, SEG_META_FILENAME, "w", SPDK_FS_MODE};
    init_query_engine_with_persistence(&query_engine, &data_file, &index_file, &meta_file);

    for (int i = 0; i < run_num; i++) {
        printf("the %d run of data ingestion\n", i);
        init_query_engine_with_persistence(&query_engine, &data_file, &index_file, &meta_file);
        ingest_and_flush_osm_data_via_zcurve_partition_with_block_index_with_sort_option(&query_engine, data_fp, i * data_block_num_each_run, data_block_num_each_run, sort_option);

        // print temporal meta information for each block
        double total_time_width = 0;
        double total_lon_width = 0;
        double total_lat_width = 0;
        // print temporal meta information for each block
        struct index_entry_storage *index_storage = &query_engine.index_storage;
        for (int n = 0; n <= index_storage->current_index; n++) {
            struct index_entry *entry = index_storage->index_entry_base[n];
            printf("block pointer: [%d], time min: %d, time max: %d, lon min: %d, lon max: %d, lat min: %d, lat max: %d\n", entry->block_logical_adr, entry->time_min, entry->time_max, entry->lon_min, entry->lon_max, entry->lat_min, entry->lat_max);
            total_time_width += (entry->time_max - entry->time_min);
            total_lon_width += (entry->lon_max - entry->lon_min);
            total_lat_width += (entry->lat_max - entry->lat_min);
        }
        printf("mbr shape: time width: %f, lon width: %f, lat width: %f\n", total_time_width / index_storage->current_index, total_lon_width / index_storage->current_index,
               total_lat_width / index_storage->current_index);
        printf("block num: %d\n", index_storage->current_index);

        free_query_engine(&query_engine);
    }


    print_spdk_static_fs_meta_for_traj();
    // save filesystem meta
    spdk_flush_static_fs_meta_for_traj();
}


static void ingest_and_flush_osm_data_time_oid_large(int data_block_num_each_run, int run_num) {
    init_and_mk_fs_for_traj(false);

    FILE *data_fp = fopen("/home/yangguo/Dataset/osm/osm_points_v1.csv", "r");
    // ingest data
    struct simple_query_engine query_engine;
    struct my_file data_file = {NULL, DATA_FILENAME, "w", SPDK_FS_MODE};
    struct my_file index_file = {NULL, INDEX_FILENAME, "w", SPDK_FS_MODE};
    struct my_file meta_file = {NULL, SEG_META_FILENAME, "w", SPDK_FS_MODE};
    init_query_engine_with_persistence(&query_engine, &data_file, &index_file, &meta_file);

    for (int i = 0; i < run_num; i++) {
        printf("the %d run of data ingestion\n", i);
        init_query_engine_with_persistence(&query_engine, &data_file, &index_file, &meta_file);
        ingest_and_flush_osm_data_via_time_partition_with_block_index(&query_engine, data_fp, i * data_block_num_each_run, data_block_num_each_run);
        free_query_engine(&query_engine);
    }


    print_spdk_static_fs_meta_for_traj();
    // save filesystem meta
    spdk_flush_static_fs_meta_for_traj();
}

static void ingest_and_flush_osm_data_zcurve_full() {
    ingest_and_flush_osm_data_zcurve_large(1779499, 1);
}

static void ingest_and_flush_osm_data_time_oid_full() {
    ingest_and_flush_osm_data_time_oid_large(1779499, 1);
}

static void print_large_file_info() {
    init_and_mk_fs_for_traj(true);
    print_spdk_static_fs_meta_for_traj();

    struct simple_query_engine rebuild_engine;
    struct my_file data_file_rebuild = {NULL, DATA_FILENAME, "r", SPDK_FS_MODE};
    struct my_file index_file_rebuild = {NULL, INDEX_FILENAME, "r", SPDK_FS_MODE};
    struct my_file meta_file_rebuild = {NULL, SEG_META_FILENAME, "r", SPDK_FS_MODE};
    init_query_engine_with_persistence(&rebuild_engine, &data_file_rebuild, &index_file_rebuild, &meta_file_rebuild);


    rebuild_query_engine_from_file(&rebuild_engine);

    // print temporal meta information for each block
    struct index_entry_storage *index_storage = &rebuild_engine.index_storage;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        printf("block pointer: [%d], time min: %d, time max: %d\n", entry->block_logical_adr, entry->time_min, entry->time_max);
    }
    free_query_engine(&rebuild_engine);
}

static int exp_native_spatio_temporal_knn_host_multi_addr_batch_v1(struct spatio_temporal_knn_predicate *predicate, struct simple_query_engine *rebuild_engine) {

    clock_t start, end;
    start = clock();
    int engine_result = spatio_temporal_knn_query_without_pushdown_multi_addr_batch(rebuild_engine, predicate, enable_host_index);
    end = clock();
    printf("[host] query time (total time, including all): %f\n", (double )(end - start));
    printf("engine result: %d\n", engine_result);

    return engine_result;
}

static int exp_native_spatio_temporal_knn_host_batch_v1(struct spatio_temporal_knn_predicate *predicate, struct simple_query_engine *rebuild_engine) {

    clock_t start, end;
    start = clock();
    int engine_result = spatio_temporal_knn_query_without_pushdown_batch(rebuild_engine, predicate, enable_host_index);
    end = clock();
    printf("[host] query time (total time, including all): %f\n", (double )(end - start));
    printf("engine result: %d\n", engine_result);

    return engine_result;
}

static int exp_native_spatio_temporal_knn_armcpu_pushdown_batch_v1(struct spatio_temporal_knn_predicate *predicate, struct simple_query_engine *rebuild_engine, int option) {

    clock_t start, end;
    start = clock();
    int engine_result = spatio_temporal_knn_query_with_pushdown_batch(rebuild_engine, predicate, option, enable_host_index);
    end = clock();
    printf("[isp cpu] query time (total time, including all): %f\n", (double )(end - start));
    printf("engine result: %d\n", engine_result);

    return engine_result;
}

static int exp_native_spatio_temporal_knn_do_nothing_batch_v1(struct spatio_temporal_knn_predicate *predicate, struct simple_query_engine *rebuild_engine) {

    clock_t start, end;
    start = clock();
    int engine_result = spatio_temporal_knn_query_do_nothing_batch(rebuild_engine, predicate, enable_host_index);
    end = clock();
    printf("[host] query time (total time, including all): %f\n", (double )(end - start));
    printf("engine result: %d\n", engine_result);

    return engine_result;
}

void exp_spatio_temporal_knn_query_osm_scan_subset() {
    init_and_mk_fs_for_traj(true);
    print_spdk_static_fs_meta_for_traj();

    struct simple_query_engine rebuild_engine;
    struct my_file data_file_rebuild = {NULL, DATA_FILENAME, "r", SPDK_FS_MODE};
    struct my_file index_file_rebuild = {NULL, INDEX_FILENAME, "r", SPDK_FS_MODE};
    struct my_file meta_file_rebuild = {NULL, SEG_META_FILENAME, "r", SPDK_FS_MODE};
    init_query_engine_with_persistence(&rebuild_engine, &data_file_rebuild, &index_file_rebuild, &meta_file_rebuild);

    rebuild_query_engine_from_file(&rebuild_engine);

    clock_t start, end, query_start, query_end;
    start = clock();

    struct spatio_temporal_knn_predicate *predicate_ptr = NULL;

    FILE *query_fp_k10 = fopen("/home/yangguo/Codes/groundhog/query-workload/osm_knn_k10.query", "r");
    FILE *query_fp_k30 = fopen("/home/yangguo/Codes/groundhog/query-workload/osm_knn_k30.query", "r");
    FILE *query_fp_k50 = fopen("/home/yangguo/Codes/groundhog/query-workload/osm_knn_k50.query", "r");
    FILE *query_fp_k70 = fopen("/home/yangguo/Codes/groundhog/query-workload/osm_knn_k70.query", "r");
    FILE *query_fp_k90 = fopen("/home/yangguo/Codes/groundhog/query-workload/osm_knn_k90.query", "r");
    // read queries
    //int query_num = 100;
    //int query_num = 20;
    int query_num = 100;
    struct spatio_temporal_knn_predicate **predicates_k10 = allocate_spatio_temporal_knn_predicate_mem(query_num);
    read_spatio_temporal_knn_queries_from_csv(query_fp_k10, predicates_k10, query_num);
    struct spatio_temporal_knn_predicate **predicates_k30 = allocate_spatio_temporal_knn_predicate_mem(query_num);
    read_spatio_temporal_knn_queries_from_csv(query_fp_k30, predicates_k30, query_num);
    struct spatio_temporal_knn_predicate **predicates_k50 = allocate_spatio_temporal_knn_predicate_mem(query_num);
    read_spatio_temporal_knn_queries_from_csv(query_fp_k50, predicates_k50, query_num);
    struct spatio_temporal_knn_predicate **predicates_k70 = allocate_spatio_temporal_knn_predicate_mem(query_num);
    read_spatio_temporal_knn_queries_from_csv(query_fp_k70, predicates_k70, query_num);
    struct spatio_temporal_knn_predicate **predicates_k90 = allocate_spatio_temporal_knn_predicate_mem(query_num);
    read_spatio_temporal_knn_queries_from_csv(query_fp_k90, predicates_k90, query_num);


    struct spatio_temporal_knn_predicate **predicates = allocate_spatio_temporal_knn_predicate_mem(query_num);
    int offset = 2;
    /*predicates[0] = predicates_k10[offset];
    predicates[1] = predicates_k30[offset];
    predicates[2] = predicates_k50[offset];
    predicates[3] = predicates_k70[offset];
    predicates[4] = predicates_k90[offset];*/

    read_spatio_temporal_knn_queries_from_csv(query_fp_k90, predicates, query_num);


    long host_ioopt_time[query_num];
    long host_ioopt_time_pure[query_num];
    long host_time[query_num];
    long host_time_pure[query_num];
    long device_time_naive[query_num];
    long device_time_naive_pure[query_num];
    long device_time_mbr_pruning[query_num];
    long device_time_mbr_pruning_pure[query_num];
    long device_time[query_num];
    long device_time_pure[query_num];

    int running_time;
    long block_num[query_num];

    for (int i = 0; i < query_num; i++) {
        printf("\n\ni: %d\n", i);
        predicate_ptr = predicates[i];
        predicate_ptr->k = 700;

        query_start = clock();
        running_time = exp_native_spatio_temporal_knn_host_multi_addr_batch_v1(predicate_ptr, &rebuild_engine);
        query_end = clock();
        host_ioopt_time[i] = query_end - query_start;
        host_ioopt_time_pure[i] = running_time;


        query_start = clock();
        running_time = exp_native_spatio_temporal_knn_host_batch_v1(predicate_ptr, &rebuild_engine);
        query_end = clock();
        host_time[i] = query_end - query_start;
        host_time_pure[i] = running_time;

        // naive
        query_start = clock();
        running_time = exp_native_spatio_temporal_knn_armcpu_pushdown_batch_v1(predicate_ptr, &rebuild_engine, 1);
        query_end = clock();
        device_time_naive[i] = query_end - query_start;
        device_time_naive_pure[i] = running_time;

        // add pruning
        query_start = clock();
        running_time = exp_native_spatio_temporal_knn_armcpu_pushdown_batch_v1(predicate_ptr, &rebuild_engine, 2);
        query_end = clock();
        device_time_mbr_pruning[i] = query_end - query_start;
        device_time_mbr_pruning_pure[i] = running_time;

        // add pruning and sorting optimizations
        query_start = clock();
        running_time = exp_native_spatio_temporal_knn_armcpu_pushdown_batch_v1(predicate_ptr, &rebuild_engine, 3);
        query_end = clock();
        device_time[i] = query_end - query_start;
        device_time_pure[i] = running_time;

        running_time = exp_native_spatio_temporal_knn_do_nothing_batch_v1(predicate_ptr, &rebuild_engine);
        block_num[i] = running_time;
    }
    end = clock();
    printf("total time: %f\n",(double)(end-start));

    printf("\n\n[host io opt] average time: %f, average pure time: %f\n", average_values(host_ioopt_time, query_num), average_values(host_ioopt_time_pure, query_num));
    printf("[host] average time: %f, average pure time: %f\n", average_values(host_time, query_num), average_values(host_time_pure, query_num));
    printf("[device naive] average time: %f, average pure time: %f\n", average_values(device_time_naive, query_num),
           average_values(device_time_naive_pure, query_num));
    printf("[device add mbr pruning] average time: %f, average pure time: %f\n", average_values(device_time_mbr_pruning, query_num),
           average_values(device_time_mbr_pruning_pure, query_num));
    printf("[device] average time: %f, average pure time: %f\n", average_values(device_time, query_num), average_values(device_time_pure, query_num));
    printf("[average block num] %f\n", average_values(block_num, query_num));

    free_spatio_temporal_knn_predicate_mem(predicates, query_num);
    free_query_engine(&rebuild_engine);
}

void exp_spatio_temporal_knn_query_osm_scan() {
    init_and_mk_fs_for_traj(true);
    print_spdk_static_fs_meta_for_traj();

    struct simple_query_engine rebuild_engine;
    struct my_file data_file_rebuild = {NULL, DATA_FILENAME, "r", SPDK_FS_MODE};
    struct my_file index_file_rebuild = {NULL, INDEX_FILENAME, "r", SPDK_FS_MODE};
    struct my_file meta_file_rebuild = {NULL, SEG_META_FILENAME, "r", SPDK_FS_MODE};
    init_query_engine_with_persistence(&rebuild_engine, &data_file_rebuild, &index_file_rebuild, &meta_file_rebuild);

    rebuild_query_engine_from_file(&rebuild_engine);

    clock_t start, end, query_start, query_end;
    start = clock();

    struct spatio_temporal_knn_predicate *predicate_ptr = NULL;

    FILE *query_fp_k10 = fopen("/home/yangguo/Codes/groundhog/query-workload/osm_knn_k10.query", "r");
    FILE *query_fp_k30 = fopen("/home/yangguo/Codes/groundhog/query-workload/osm_knn_k30.query", "r");
    FILE *query_fp_k50 = fopen("/home/yangguo/Codes/groundhog/query-workload/osm_knn_k50.query", "r");
    FILE *query_fp_k70 = fopen("/home/yangguo/Codes/groundhog/query-workload/osm_knn_k70.query", "r");
    FILE *query_fp_k90 = fopen("/home/yangguo/Codes/groundhog/query-workload/osm_knn_k90.query", "r");
    // read queries
    //int query_num = 100;

    int query_num = 20;
    struct spatio_temporal_knn_predicate **predicates_k10 = allocate_spatio_temporal_knn_predicate_mem(query_num);
    read_spatio_temporal_knn_queries_from_csv(query_fp_k10, predicates_k10, query_num);
    struct spatio_temporal_knn_predicate **predicates_k30 = allocate_spatio_temporal_knn_predicate_mem(query_num);
    read_spatio_temporal_knn_queries_from_csv(query_fp_k30, predicates_k30, query_num);
    struct spatio_temporal_knn_predicate **predicates_k50 = allocate_spatio_temporal_knn_predicate_mem(query_num);
    read_spatio_temporal_knn_queries_from_csv(query_fp_k50, predicates_k50, query_num);
    struct spatio_temporal_knn_predicate **predicates_k70 = allocate_spatio_temporal_knn_predicate_mem(query_num);
    read_spatio_temporal_knn_queries_from_csv(query_fp_k70, predicates_k70, query_num);
    struct spatio_temporal_knn_predicate **predicates_k90 = allocate_spatio_temporal_knn_predicate_mem(query_num);
    read_spatio_temporal_knn_queries_from_csv(query_fp_k90, predicates_k90, query_num);


    struct spatio_temporal_knn_predicate **predicates = allocate_spatio_temporal_knn_predicate_mem(query_num);
    /*int offset = 4;
    predicates[0] = predicates_k10[offset];
    predicates[1] = predicates_k30[offset];
    predicates[2] = predicates_k50[offset];
    predicates[3] = predicates_k70[offset];
    predicates[4] = predicates_k90[offset];*/

    read_spatio_temporal_knn_queries_from_csv(query_fp_k90, predicates, query_num);


    int running_time;
    int loop_num = query_num;
    long host_time[loop_num];
    long host_time_pure[loop_num];

    long device_time[loop_num];
    long device_time_pure[loop_num];
    for (int i = 0; i < loop_num; i++) {
        printf("\n\ni: %d\n", i);
        predicate_ptr = predicates[i];
        printf("query point: (%d, %d), predicate k: %d\n", predicate_ptr->query_point.normalized_longitude, predicate_ptr->query_point.normalized_latitude, predicate_ptr->k);

        query_start = clock();
        running_time = exp_native_spatio_temporal_knn_host_batch_v1(predicate_ptr, &rebuild_engine);
        query_end = clock();
        host_time[i] = query_end - query_start;
        host_time_pure[i] = running_time;

        /*// naive
        query_start = clock();
        running_time = exp_native_spatio_temporal_knn_armcpu_pushdown_batch_v1(predicate_ptr, &rebuild_engine, 1);
        query_end = clock();
        device_time_naive[i] = query_end - query_start;
        device_time_naive_pure[i] = running_time;

        // add pruning
        query_start = clock();
        running_time = exp_native_spatio_temporal_knn_armcpu_pushdown_batch_v1(predicate_ptr, &rebuild_engine, 2);
        query_end = clock();
        device_time_mbr_pruning[i] = query_end - query_start;
        device_time_mbr_pruning_pure[i] = running_time;*/

        // add pruning and sorting optimizations
        /*query_start = clock();
        running_time = exp_native_spatio_temporal_knn_armcpu_pushdown_batch_v1(predicate_ptr, &rebuild_engine, 3);
        query_end = clock();
        device_time[i] = query_end - query_start;
        device_time_pure[i] = running_time;*/
    }
    end = clock();
    printf("total time: %f\n",(double)(end-start));

    printf("\n\n[host] average time: %f, average pure time: %f\n", average_values(host_time, loop_num), average_values(host_time_pure, loop_num));
    printf("[device] average time: %f, average pure time: %f\n", average_values(device_time, loop_num), average_values(device_time_pure, loop_num));

    free_spatio_temporal_knn_predicate_mem(predicates, query_num);
    free_query_engine(&rebuild_engine);
}

void ingest_and_flush_osm_data_zcurve_time_preferred() {
    ingest_and_flush_osm_data_zcurve_large_with_sort_option(197949, 1, 1);
}

void ingest_and_flush_osm_data_zcurve_space_preferred() {
    ingest_and_flush_osm_data_zcurve_large_with_sort_option(197949, 1, 2);
}

void ingest_and_flush_osm_data_zcurve_no_preferred() {
    ingest_and_flush_osm_data_zcurve_large_with_sort_option(197949, 1, 3);
}


static int exp_native_spatio_temporal_armcpu_full_pushdown_batch_v1(struct spatio_temporal_range_predicate *predicate, struct simple_query_engine *rebuild_engine) {

    clock_t start, end;
    start = clock();
    int engine_result = spatio_temporal_query_with_full_pushdown_batch(rebuild_engine, predicate, enable_host_index);
    end = clock();
    printf("[isp cpu batch] query time (total time, including all): %f\n", (double )(end - start));
    printf("[isp cpu batch] engine result: %d\n", engine_result);
    return engine_result;
}


void exp_spatio_temporal_query_porto_index_scan_with_query_file_device_only(FILE *query_fp) {
    init_and_mk_fs_for_traj(true);
    print_spdk_static_fs_meta_for_traj();

    struct simple_query_engine rebuild_engine;
    struct my_file data_file_rebuild = {NULL, DATA_FILENAME, "r", SPDK_FS_MODE};
    struct my_file index_file_rebuild = {NULL, INDEX_FILENAME, "r", SPDK_FS_MODE};
    struct my_file meta_file_rebuild = {NULL, SEG_META_FILENAME, "r", SPDK_FS_MODE};
    init_query_engine_with_persistence(&rebuild_engine, &data_file_rebuild, &index_file_rebuild, &meta_file_rebuild);


    rebuild_query_engine_from_file(&rebuild_engine);

    //FILE *query_fp = fopen("/home/yangguo/Codes/groundhog/query-workload/porto_st_005.query", "r");
    // read queries
    int query_num = 500;
    struct spatio_temporal_range_predicate **predicates = allocate_spatio_temporal_predicate_mem(query_num);
    read_spatio_temporal_queries_from_csv(query_fp, predicates, query_num);


    long device_time_mbr_pruning[query_num];
    long device_time_mbr_pruning_pure[query_num];


    int running_time;
    double selectivity;

    uint64_t prev_time_value, time_value;

    prev_time_value = get_posix_clock_time();
    for (int i = 0; i < query_num; i++) {

        printf("i: %d\n", i);
        printf("time min: %d, time max: %d, lon min: %d, lon max: %d, lat min: %d, lat max: %d\n",
               predicates[i]->time_min,
               predicates[i]->time_max, predicates[i]->lon_min, predicates[i]->lon_max, predicates[i]->lat_min,
               predicates[i]->lat_max);



        printf("\n");

        prev_time_value = get_posix_clock_time();
        running_time = exp_native_spatio_temporal_armcpu_full_pushdown_batch_v1(predicates[i], &rebuild_engine);
        time_value = get_posix_clock_time();
        device_time_mbr_pruning[i] = time_value - prev_time_value;
        device_time_mbr_pruning_pure[i] = running_time;
        printf("\n");




    }

    printf("\n\n[device add mbr pruning] average time: %f, average pure time: %f\n", average_values(device_time_mbr_pruning, query_num),
           average_values(device_time_mbr_pruning_pure, query_num));


    free_spatio_temporal_predicate_mem(predicates, query_num);
    free_query_engine(&rebuild_engine);
}

static int exp_native_spatio_temporal_host_io_opt_batch_v1(struct spatio_temporal_range_predicate *predicate, struct simple_query_engine *rebuild_engine) {

    clock_t start, end;
    start = clock();
    int engine_result = spatio_temporal_query_without_pushdown_multi_addr_batch(rebuild_engine, predicate, enable_host_index);
    end = clock();
    printf("[host] query time (total time, including all): %f\n", (double )(end - start));
    printf("engine result: %d\n", engine_result);

    return engine_result;
}

void exp_spatio_temporal_query_porto_index_scan_with_query_file_host_ioopt_only(FILE *query_fp) {
    init_and_mk_fs_for_traj(true);
    print_spdk_static_fs_meta_for_traj();

    struct simple_query_engine rebuild_engine;
    struct my_file data_file_rebuild = {NULL, DATA_FILENAME, "r", SPDK_FS_MODE};
    struct my_file index_file_rebuild = {NULL, INDEX_FILENAME, "r", SPDK_FS_MODE};
    struct my_file meta_file_rebuild = {NULL, SEG_META_FILENAME, "r", SPDK_FS_MODE};
    init_query_engine_with_persistence(&rebuild_engine, &data_file_rebuild, &index_file_rebuild, &meta_file_rebuild);


    rebuild_query_engine_from_file(&rebuild_engine);

    //FILE *query_fp = fopen("/home/yangguo/Codes/groundhog/query-workload/porto_st_005.query", "r");
    // read queries
    int query_num = 500;
    struct spatio_temporal_range_predicate **predicates = allocate_spatio_temporal_predicate_mem(query_num);
    read_spatio_temporal_queries_from_csv(query_fp, predicates, query_num);


    long host_time[query_num];
    long host_time_pure[query_num];



    int running_time;
    double selectivity;

    uint64_t prev_time_value, time_value;

    prev_time_value = get_posix_clock_time();
    for (int i = 0; i < query_num; i++) {

        printf("i: %d\n", i);
        printf("time min: %d, time max: %d, lon min: %d, lon max: %d, lat min: %d, lat max: %d\n",
               predicates[i]->time_min,
               predicates[i]->time_max, predicates[i]->lon_min, predicates[i]->lon_max, predicates[i]->lat_min,
               predicates[i]->lat_max);

        prev_time_value = get_posix_clock_time();
        running_time = exp_native_spatio_temporal_host_io_opt_batch_v1(predicates[i], &rebuild_engine);
        time_value = get_posix_clock_time();
        host_time[i] = time_value - prev_time_value;
        host_time_pure[i] = running_time;

        printf("\n");



    }


    printf("\n\n[host io opt] average time: %f, average pure time: %f\n", average_values(host_time, query_num), average_values(host_time_pure, query_num));


    free_spatio_temporal_predicate_mem(predicates, query_num);
    free_query_engine(&rebuild_engine);
}

static int exp_native_spatio_temporal_host_batch_v1(struct spatio_temporal_range_predicate *predicate, struct simple_query_engine *rebuild_engine) {

    clock_t start, end;
    start = clock();
    int engine_result = spatio_temporal_query_without_pushdown_batch(rebuild_engine, predicate, enable_host_index);
    end = clock();
    printf("[host] query time (total time, including all): %f\n", (double )(end - start));
    printf("engine result: %d\n", engine_result);

    return engine_result;
}

void exp_spatio_temporal_query_porto_index_scan_with_query_file_host_only(FILE *query_fp) {
    init_and_mk_fs_for_traj(true);
    print_spdk_static_fs_meta_for_traj();

    struct simple_query_engine rebuild_engine;
    struct my_file data_file_rebuild = {NULL, DATA_FILENAME, "r", SPDK_FS_MODE};
    struct my_file index_file_rebuild = {NULL, INDEX_FILENAME, "r", SPDK_FS_MODE};
    struct my_file meta_file_rebuild = {NULL, SEG_META_FILENAME, "r", SPDK_FS_MODE};
    init_query_engine_with_persistence(&rebuild_engine, &data_file_rebuild, &index_file_rebuild, &meta_file_rebuild);


    rebuild_query_engine_from_file(&rebuild_engine);

    //FILE *query_fp = fopen("/home/yangguo/Codes/groundhog/query-workload/porto_st_005.query", "r");
    // read queries
    int query_num = 500;
    struct spatio_temporal_range_predicate **predicates = allocate_spatio_temporal_predicate_mem(query_num);
    read_spatio_temporal_queries_from_csv(query_fp, predicates, query_num);


    long host_time[query_num];
    long host_time_pure[query_num];



    int running_time;
    double selectivity;

    uint64_t prev_time_value, time_value;

    prev_time_value = get_posix_clock_time();
    for (int i = 0; i < query_num; i++) {

        printf("i: %d\n", i);
        printf("time min: %d, time max: %d, lon min: %d, lon max: %d, lat min: %d, lat max: %d\n",
               predicates[i]->time_min,
               predicates[i]->time_max, predicates[i]->lon_min, predicates[i]->lon_max, predicates[i]->lat_min,
               predicates[i]->lat_max);

        prev_time_value = get_posix_clock_time();
        running_time = exp_native_spatio_temporal_host_batch_v1(predicates[i], &rebuild_engine);
        time_value = get_posix_clock_time();
        host_time[i] = time_value - prev_time_value;
        host_time_pure[i] = running_time;

        printf("\n");



    }


    printf("\n\n[host] average time: %f, average pure time: %f\n", average_values(host_time, query_num), average_values(host_time_pure, query_num));


    free_spatio_temporal_predicate_mem(predicates, query_num);
    free_query_engine(&rebuild_engine);
}


int main(void)  {
    //ingest_and_flush_synthetic_data_large(2, 4);
    //print_large_file_info();


    //ingest_and_flush_osm_data_zcurve_large(197949, 1);
    //ingest_and_flush_osm_data_time_oid_large(197949, 1);
    //exp_spatio_temporal_knn_query_osm_scan_subset();


    //ingest_and_flush_osm_data_time_oid_large(197949, 1);
    //ingest_and_flush_osm_data_time_oid_full();
    //exp_spatio_temporal_knn_query_osm_scan();

    // mbr shape: mbr shape: time width: 13.237264, lon width: 1491549.035231, lat width: 1686129.582926
    //ingest_and_flush_osm_data_zcurve_time_preferred();
    // mbr shape: mbr shape: time width: 46242.305252, lon width: 24425.988224, lat width: 18894.890936
    //ingest_and_flush_osm_data_zcurve_space_preferred();
    // mbr shape: time width: 11448.553837, lon width: 203355.715557, lat width: 283639.642376
    //ingest_and_flush_osm_data_zcurve_no_preferred();

    FILE *query_fp = fopen("/home/yangguo/Codes/groundhog/query-workload/porto_st_time_10min_workload2_500queries.query", "r");

    //exp_spatio_temporal_query_porto_index_scan_with_query_file_device_only(query_fp);
    exp_spatio_temporal_query_porto_index_scan_with_query_file_host_only(query_fp);
    //exp_spatio_temporal_query_porto_index_scan_with_query_file_host_ioopt_only(query_fp);
}
