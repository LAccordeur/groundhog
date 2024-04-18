//
// Created by yangguo on 23-2-7.
//
// compare performance between host computation and in-storage-computing
// to get the cross-point which tell us when in-storage-computing is better

#include <stdio.h>
#include "groundhog/simple_query_engine.h"
#include "groundhog/query_workload_reader.h"
#include "time.h"
#include "groundhog/config.h"
#include "groundhog/normalization_util.h"
#include "groundhog/common_util.h"

static bool enable_host_index = true;

static void ingest_and_flush_porto_data(int data_block_num) {
    init_and_mk_fs_for_traj(false);

    //int data_block_num = 1024;

    FILE *data_fp = fopen("/home/yangguo/Dataset/trajectory/porto_data_v2.csv", "r");
    // ingest data
    struct simple_query_engine query_engine;
    struct my_file data_file = {NULL, DATA_FILENAME, "w", SPDK_FS_MODE};
    struct my_file index_file = {NULL, INDEX_FILENAME, "w", SPDK_FS_MODE};
    struct my_file meta_file = {NULL, SEG_META_FILENAME, "w", SPDK_FS_MODE};
    init_query_engine_with_persistence(&query_engine, &data_file, &index_file, &meta_file);
    ingest_and_flush_data_via_time_partition(&query_engine, data_fp, data_block_num);

    // print temporal meta information for each block
    struct index_entry_storage *index_storage = &query_engine.index_storage;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        //printf("block pointer: [%d], time min: %d, time max: %d, lon min: %d, lon max: %d, lat min: %d, lat max: %d\n", entry->block_logical_adr, entry->time_min, entry->time_max, entry->lon_min, entry->lon_max, entry->lat_min, entry->lat_max);
    }

    // save filesystem meta
    spdk_flush_static_fs_meta_for_traj();

}

static void ingest_and_flush_porto_data_10x(int data_block_num) {
    init_and_mk_fs_for_traj(false);

    //int data_block_num = 1024;

    FILE *data_fp = fopen("/home/yangguo/Codes/groundhog/query-workload/porto_data_v2_10x.csv", "r");
    // ingest data
    struct simple_query_engine query_engine;
    struct my_file data_file = {NULL, DATA_FILENAME, "w", SPDK_FS_MODE};
    struct my_file index_file = {NULL, INDEX_FILENAME, "w", SPDK_FS_MODE};
    struct my_file meta_file = {NULL, SEG_META_FILENAME, "w", SPDK_FS_MODE};
    init_query_engine_with_persistence(&query_engine, &data_file, &index_file, &meta_file);
    ingest_and_flush_data_via_time_partition(&query_engine, data_fp, data_block_num);

    // print temporal meta information for each block
    struct index_entry_storage *index_storage = &query_engine.index_storage;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        //printf("block pointer: [%d], time min: %d, time max: %d, lon min: %d, lon max: %d, lat min: %d, lat max: %d\n", entry->block_logical_adr, entry->time_min, entry->time_max, entry->lon_min, entry->lon_max, entry->lat_min, entry->lat_max);
    }

    // save filesystem meta
    spdk_flush_static_fs_meta_for_traj();

}

static void ingest_and_flush_porto_data_zcurve(int data_block_num) {
    init_and_mk_fs_for_traj(false);

    //int data_block_num = 1024;

    FILE *data_fp = fopen("/home/yangguo/Dataset/trajectory/porto_data_v2.csv", "r");
    // ingest data
    struct simple_query_engine query_engine;
    struct my_file data_file = {NULL, DATA_FILENAME, "w", SPDK_FS_MODE};
    struct my_file index_file = {NULL, INDEX_FILENAME, "w", SPDK_FS_MODE};
    struct my_file meta_file = {NULL, SEG_META_FILENAME, "w", SPDK_FS_MODE};
    init_query_engine_with_persistence(&query_engine, &data_file, &index_file, &meta_file);
    ingest_and_flush_data_via_zcurve_partition(&query_engine, data_fp, data_block_num);

    // print temporal meta information for each block
    struct index_entry_storage *index_storage = &query_engine.index_storage;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        printf("block pointer: [%d], time min: %d, time max: %d, lon min: %d, lon max: %d, lat min: %d, lat max: %d\n", entry->block_logical_adr, entry->time_min, entry->time_max, entry->lon_min, entry->lon_max, entry->lat_min, entry->lat_max);
    }

    printf("block num: %d\n", index_storage->current_index);
    // save filesystem meta
    spdk_flush_static_fs_meta_for_traj();

}

static void ingest_and_flush_porto_data_zcurve_segment_num(int data_block_num) {
    init_and_mk_fs_for_traj(false);

    //int data_block_num = 1024;

    FILE *data_fp = fopen("/home/yangguo/Dataset/trajectory/porto_data_v2_24million.csv", "r");
    // ingest data
    struct simple_query_engine query_engine;
    struct my_file data_file = {NULL, DATA_FILENAME, "w", SPDK_FS_MODE};
    struct my_file index_file = {NULL, INDEX_FILENAME, "w", SPDK_FS_MODE};
    struct my_file meta_file = {NULL, SEG_META_FILENAME, "w", SPDK_FS_MODE};
    init_query_engine_with_persistence(&query_engine, &data_file, &index_file, &meta_file);
    ingest_and_flush_data_via_zcurve_partition(&query_engine, data_fp, data_block_num);

    // print temporal meta information for each block
    struct index_entry_storage *index_storage = &query_engine.index_storage;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        printf("block pointer: [%d], time min: %d, time max: %d, lon min: %d, lon max: %d, lat min: %d, lat max: %d\n", entry->block_logical_adr, entry->time_min, entry->time_max, entry->lon_min, entry->lon_max, entry->lat_min, entry->lat_max);
    }

    printf("block num: %d\n", index_storage->current_index);
    // save filesystem meta
    spdk_flush_static_fs_meta_for_traj();

}

static void ingest_and_flush_porto_data_10x_zcurve(int data_block_num) {
    init_and_mk_fs_for_traj(false);

    //int data_block_num = 1024;

    FILE *data_fp = fopen("/home/yangguo/Codes/groundhog/query-workload/porto_data_v2_10x.csv", "r");
    // ingest data
    struct simple_query_engine query_engine;
    struct my_file data_file = {NULL, DATA_FILENAME, "w", SPDK_FS_MODE};
    struct my_file index_file = {NULL, INDEX_FILENAME, "w", SPDK_FS_MODE};
    struct my_file meta_file = {NULL, SEG_META_FILENAME, "w", SPDK_FS_MODE};
    init_query_engine_with_persistence(&query_engine, &data_file, &index_file, &meta_file);
    ingest_and_flush_data_via_zcurve_partition(&query_engine, data_fp, data_block_num);

    // print temporal meta information for each block
    struct index_entry_storage *index_storage = &query_engine.index_storage;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        printf("block pointer: [%d], time min: %d, time max: %d, lon min: %d, lon max: %d, lat min: %d, lat max: %d\n", entry->block_logical_adr, entry->time_min, entry->time_max, entry->lon_min, entry->lon_max, entry->lat_min, entry->lat_max);
    }

    // save filesystem meta
    spdk_flush_static_fs_meta_for_traj();

}

static void ingest_and_flush_nyc_data(int data_block_num) {
    init_and_mk_fs_for_traj(false);

    //int data_block_num = 1024;

    FILE *data_fp = fopen("/home/yangguo/Dataset/nyctaxi/pickup_trip_data_2010_2013_full_v1.csv", "r");
    // ingest data
    struct simple_query_engine query_engine;
    struct my_file data_file = {NULL, DATA_FILENAME, "w", SPDK_FS_MODE};
    struct my_file index_file = {NULL, INDEX_FILENAME, "w", SPDK_FS_MODE};
    struct my_file meta_file = {NULL, SEG_META_FILENAME, "w", SPDK_FS_MODE};
    init_query_engine_with_persistence(&query_engine, &data_file, &index_file, &meta_file);
    ingest_and_flush_nyc_data_via_time_partition(&query_engine, data_fp, data_block_num);

    // print temporal meta information for each block
    struct index_entry_storage *index_storage = &query_engine.index_storage;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        printf("block pointer: [%d], time min: %d, time max: %d, lon min: %d, lon max: %d, lat min: %d, lat max: %d\n", entry->block_logical_adr, entry->time_min, entry->time_max, entry->lon_min, entry->lon_max, entry->lat_min, entry->lat_max);
    }

    // save filesystem meta
    spdk_flush_static_fs_meta_for_traj();

}

static void ingest_and_flush_nyc_data_zcurve(int data_block_num) {
    init_and_mk_fs_for_traj(false);

    //int data_block_num = 1024;

    FILE *data_fp = fopen("/home/yangguo/Dataset/nyctaxi/pickup_trip_data_2010_2013_full_v1.csv", "r");
    // ingest data
    struct simple_query_engine query_engine;
    struct my_file data_file = {NULL, DATA_FILENAME, "w", SPDK_FS_MODE};
    struct my_file index_file = {NULL, INDEX_FILENAME, "w", SPDK_FS_MODE};
    struct my_file meta_file = {NULL, SEG_META_FILENAME, "w", SPDK_FS_MODE};
    init_query_engine_with_persistence(&query_engine, &data_file, &index_file, &meta_file);
    ingest_and_flush_nyc_data_via_zcurve_partition(&query_engine, data_fp, data_block_num);

    // print temporal meta information for each block
    struct index_entry_storage *index_storage = &query_engine.index_storage;
    for (int i = 0; i <= index_storage->current_index; i++) {
        struct index_entry *entry = index_storage->index_entry_base[i];
        printf("block pointer: [%d], time min: %d, time max: %d, lon min: %d, lon max: %d, lat min: %d, lat max: %d\n", entry->block_logical_adr, entry->time_min, entry->time_max, entry->lon_min, entry->lon_max, entry->lat_min, entry->lat_max);
    }

    // save filesystem meta
    spdk_flush_static_fs_meta_for_traj();

}

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

static void exp_native_spatio_temporal_host(struct spatio_temporal_range_predicate *predicate) {
    init_and_mk_fs_for_traj(true);
    print_spdk_static_fs_meta_for_traj();

    struct simple_query_engine rebuild_engine;
    struct my_file data_file_rebuild = {NULL, DATA_FILENAME, "r", SPDK_FS_MODE};
    struct my_file index_file_rebuild = {NULL, INDEX_FILENAME, "r", SPDK_FS_MODE};
    struct my_file meta_file_rebuild = {NULL, SEG_META_FILENAME, "r", SPDK_FS_MODE};
    init_query_engine_with_persistence(&rebuild_engine, &data_file_rebuild, &index_file_rebuild, &meta_file_rebuild);


    rebuild_query_engine_from_file(&rebuild_engine);
    clock_t start, end;
    start = clock();
    int engine_result = spatio_temporal_query_without_pushdown(&rebuild_engine, predicate, enable_host_index);
    end = clock();
    printf("[host] query time (total): %f\n", (double )(end - start));
    printf("engine result: %d\n", engine_result);

    free_query_engine(&rebuild_engine);
}

static void exp_native_id_temporal_host(struct id_temporal_predicate *predicate) {
    init_and_mk_fs_for_traj(true);
    print_spdk_static_fs_meta_for_traj();

    struct simple_query_engine rebuild_engine;
    struct my_file data_file_rebuild = {NULL, DATA_FILENAME, "r", SPDK_FS_MODE};
    struct my_file index_file_rebuild = {NULL, INDEX_FILENAME, "r", SPDK_FS_MODE};
    struct my_file meta_file_rebuild = {NULL, SEG_META_FILENAME, "r", SPDK_FS_MODE};
    init_query_engine_with_persistence(&rebuild_engine, &data_file_rebuild, &index_file_rebuild, &meta_file_rebuild);


    rebuild_query_engine_from_file(&rebuild_engine);
    clock_t start, end;
    start = clock();
    int engine_result = id_temporal_query_without_pushdown(&rebuild_engine, predicate, false);
    end = clock();
    printf("[host] id temporal query time (includin index lookup): %f\n", (double )(end - start));
    printf("engine result: %d\n", engine_result);

    free_query_engine(&rebuild_engine);
}

static void exp_native_id_temporal_host_batch(struct id_temporal_predicate *predicate) {
    init_and_mk_fs_for_traj(true);
    print_spdk_static_fs_meta_for_traj();

    struct simple_query_engine rebuild_engine;
    struct my_file data_file_rebuild = {NULL, DATA_FILENAME, "r", SPDK_FS_MODE};
    struct my_file index_file_rebuild = {NULL, INDEX_FILENAME, "r", SPDK_FS_MODE};
    struct my_file meta_file_rebuild = {NULL, SEG_META_FILENAME, "r", SPDK_FS_MODE};
    init_query_engine_with_persistence(&rebuild_engine, &data_file_rebuild, &index_file_rebuild, &meta_file_rebuild);


    rebuild_query_engine_from_file(&rebuild_engine);
    clock_t start, end;
    start = clock();
    int engine_result = id_temporal_query_without_pushdown_batch(&rebuild_engine, predicate, false);
    end = clock();
    printf("[host batch] id temporal query time (includin index lookup): %f\n", (double )(end - start));
    printf("engine result: %d\n", engine_result);

    free_query_engine(&rebuild_engine);
}


static void exp_native_spatio_temporal_armcpu_full_pushdown(struct spatio_temporal_range_predicate *predicate) {
    init_and_mk_fs_for_traj(true);
    print_spdk_static_fs_meta_for_traj();

    struct simple_query_engine rebuild_engine;
    struct my_file data_file_rebuild = {NULL, DATA_FILENAME, "r", SPDK_FS_MODE};
    struct my_file index_file_rebuild = {NULL, INDEX_FILENAME, "r", SPDK_FS_MODE};
    struct my_file meta_file_rebuild = {NULL, SEG_META_FILENAME, "r", SPDK_FS_MODE};
    init_query_engine_with_persistence(&rebuild_engine, &data_file_rebuild, &index_file_rebuild, &meta_file_rebuild);


    rebuild_query_engine_from_file(&rebuild_engine);
    clock_t start, end;
    start = clock();
    int engine_result = spatio_temporal_query_with_full_pushdown(&rebuild_engine, predicate, enable_host_index);
    end = clock();
    printf("[isp cpu] query time (total): %f\n", (double )(end - start));
    printf("[isp cpu] engine result: %d\n", engine_result);
    free_query_engine(&rebuild_engine);

}

static void exp_native_id_temporal_fpga_full_pushdown(struct id_temporal_predicate *predicate) {

    init_and_mk_fs_for_traj(true);
    print_spdk_static_fs_meta_for_traj();

    struct simple_query_engine rebuild_engine;
    struct my_file data_file_rebuild = {NULL, DATA_FILENAME, "r", SPDK_FS_MODE};
    struct my_file index_file_rebuild = {NULL, INDEX_FILENAME, "r", SPDK_FS_MODE};
    struct my_file meta_file_rebuild = {NULL, SEG_META_FILENAME, "r", SPDK_FS_MODE};
    init_query_engine_with_persistence(&rebuild_engine, &data_file_rebuild, &index_file_rebuild, &meta_file_rebuild);


    rebuild_query_engine_from_file(&rebuild_engine);
    clock_t start, end;
    start = clock();
    int engine_result = id_temporal_query_with_full_pushdown_fpga(&rebuild_engine, predicate, enable_host_index);
    end = clock();
    printf("[isp fpga] id temporal query time (includin index lookup): %f\n", (double )(end - start));
    printf("[isp fpga] id temporal engine result: %d\n", engine_result);
    free_query_engine(&rebuild_engine);
}

static void exp_native_id_temporal_armcpu_full_pushdown(struct id_temporal_predicate *predicate) {
    init_and_mk_fs_for_traj(true);
    print_spdk_static_fs_meta_for_traj();

    struct simple_query_engine rebuild_engine;
    struct my_file data_file_rebuild = {NULL, DATA_FILENAME, "r", SPDK_FS_MODE};
    struct my_file index_file_rebuild = {NULL, INDEX_FILENAME, "r", SPDK_FS_MODE};
    struct my_file meta_file_rebuild = {NULL, SEG_META_FILENAME, "r", SPDK_FS_MODE};
    init_query_engine_with_persistence(&rebuild_engine, &data_file_rebuild, &index_file_rebuild, &meta_file_rebuild);


    rebuild_query_engine_from_file(&rebuild_engine);
    clock_t start, end;
    start = clock();
    int engine_result = id_temporal_query_with_full_pushdown(&rebuild_engine, predicate, enable_host_index);
    end = clock();
    printf("[isp cpu] query time (including index lookup): %f\n", (double )(end - start));
    printf("[isp cpu] engine result: %d\n", engine_result);
    free_query_engine(&rebuild_engine);

}

static void exp_native_spatio_temporal_fpga_full_pushdown(struct spatio_temporal_range_predicate *predicate) {

    init_and_mk_fs_for_traj(true);
    print_spdk_static_fs_meta_for_traj();

    struct simple_query_engine rebuild_engine;
    struct my_file data_file_rebuild = {NULL, DATA_FILENAME, "r", SPDK_FS_MODE};
    struct my_file index_file_rebuild = {NULL, INDEX_FILENAME, "r", SPDK_FS_MODE};
    struct my_file meta_file_rebuild = {NULL, SEG_META_FILENAME, "r", SPDK_FS_MODE};
    init_query_engine_with_persistence(&rebuild_engine, &data_file_rebuild, &index_file_rebuild, &meta_file_rebuild);


    rebuild_query_engine_from_file(&rebuild_engine);
    clock_t start, end;
    start = clock();
    int engine_result = spatio_temporal_query_with_full_pushdown_fpga(&rebuild_engine, predicate, enable_host_index);
    end = clock();
    printf("[isp fpga] query time (total): %f\n", (double )(end - start));
    printf("[isp fpga] engine result: %d\n", engine_result);
    free_query_engine(&rebuild_engine);
}


static int exp_native_spatio_temporal_host_batch(struct spatio_temporal_range_predicate *predicate) {
    init_and_mk_fs_for_traj(true);
    print_spdk_static_fs_meta_for_traj();

    struct simple_query_engine rebuild_engine;
    struct my_file data_file_rebuild = {NULL, DATA_FILENAME, "r", SPDK_FS_MODE};
    struct my_file index_file_rebuild = {NULL, INDEX_FILENAME, "r", SPDK_FS_MODE};
    struct my_file meta_file_rebuild = {NULL, SEG_META_FILENAME, "r", SPDK_FS_MODE};
    init_query_engine_with_persistence(&rebuild_engine, &data_file_rebuild, &index_file_rebuild, &meta_file_rebuild);


    rebuild_query_engine_from_file(&rebuild_engine);

    clock_t start, end;
    start = clock();

    int engine_result = spatio_temporal_query_without_pushdown_batch(&rebuild_engine, predicate, enable_host_index);
    end = clock();
    printf("[host] query time (including index lookup): %f\n", (double )(end - start));
    printf("engine result: %d\n", engine_result);

    free_query_engine(&rebuild_engine);
    return engine_result;
}

static int exp_native_spatio_temporal_count_query_host_batch_v1(struct spatio_temporal_range_predicate *predicate, struct simple_query_engine *rebuild_engine) {

    clock_t start, end;
    start = clock();
    int engine_result = spatio_temporal_count_query_without_pushdown_batch(rebuild_engine, predicate);
    end = clock();
    printf("[host] query time (total time, including all): %f\n", (double )(end - start));
    printf("engine result: %d\n", engine_result);

    return engine_result;
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

static int exp_native_spatio_temporal_host_batch_v1(struct spatio_temporal_range_predicate *predicate, struct simple_query_engine *rebuild_engine) {

    clock_t start, end;
    start = clock();
    int engine_result = spatio_temporal_query_without_pushdown_batch(rebuild_engine, predicate, enable_host_index);
    end = clock();
    printf("[host] query time (total time, including all): %f\n", (double )(end - start));
    printf("engine result: %d\n", engine_result);

    return engine_result;
}


static int exp_native_id_temporal_host_io_opt_batch_v1(struct id_temporal_predicate *predicate, struct simple_query_engine *rebuild_engine) {

    clock_t start, end;
    start = clock();
    int engine_result = id_temporal_query_without_pushdown_multi_addr_batch(rebuild_engine, predicate, enable_host_index);
    end = clock();
    printf("[host] query time (total time, including all): %f\n", (double )(end - start));
    printf("engine result: %d\n", engine_result);

    return engine_result;
}

static int exp_native_id_temporal_host_batch_v1(struct id_temporal_predicate *predicate, struct simple_query_engine *rebuild_engine) {

    clock_t start, end;
    start = clock();
    int engine_result = id_temporal_query_without_pushdown_batch(rebuild_engine, predicate, enable_host_index);
    end = clock();
    printf("[host] query time (total time, including all): %f\n", (double )(end - start));
    printf("engine result: %d\n", engine_result);

    return engine_result;
}


static void exp_native_spatio_temporal_fpga_full_pushdown_batch(struct spatio_temporal_range_predicate *predicate) {

    init_and_mk_fs_for_traj(true);
    print_spdk_static_fs_meta_for_traj();

    struct simple_query_engine rebuild_engine;
    struct my_file data_file_rebuild = {NULL, DATA_FILENAME, "r", SPDK_FS_MODE};
    struct my_file index_file_rebuild = {NULL, INDEX_FILENAME, "r", SPDK_FS_MODE};
    struct my_file meta_file_rebuild = {NULL, SEG_META_FILENAME, "r", SPDK_FS_MODE};
    init_query_engine_with_persistence(&rebuild_engine, &data_file_rebuild, &index_file_rebuild, &meta_file_rebuild);


    rebuild_query_engine_from_file(&rebuild_engine);
    clock_t start, end;
    start = clock();
    int engine_result = spatio_temporal_query_with_full_pushdown_fpga_batch(&rebuild_engine, predicate, enable_host_index);
    end = clock();
    printf("[isp fpga] query time (includin index lookup): %f\n", (double )(end - start));
    printf("[isp fpga] engine result: %d\n", engine_result);
    free_query_engine(&rebuild_engine);
}

static void exp_native_spatio_temporal_armcpu_full_pushdown_batch(struct spatio_temporal_range_predicate *predicate) {

    init_and_mk_fs_for_traj(true);
    print_spdk_static_fs_meta_for_traj();

    struct simple_query_engine rebuild_engine;
    struct my_file data_file_rebuild = {NULL, DATA_FILENAME, "r", SPDK_FS_MODE};
    struct my_file index_file_rebuild = {NULL, INDEX_FILENAME, "r", SPDK_FS_MODE};
    struct my_file meta_file_rebuild = {NULL, SEG_META_FILENAME, "r", SPDK_FS_MODE};
    init_query_engine_with_persistence(&rebuild_engine, &data_file_rebuild, &index_file_rebuild, &meta_file_rebuild);


    rebuild_query_engine_from_file(&rebuild_engine);
    clock_t start, end;
    start = clock();
    int engine_result = spatio_temporal_query_with_full_pushdown_batch(&rebuild_engine, predicate, enable_host_index);
    end = clock();
    printf("[isp cpu batch] query time (total): %f\n", (double )(end - start));
    printf("[isp cpu batch] engine result: %d\n", engine_result);
    free_query_engine(&rebuild_engine);
}

static void exp_native_spatio_temporal_armcpu_full_pushdown_batch_naive(struct spatio_temporal_range_predicate *predicate) {

    init_and_mk_fs_for_traj(true);
    print_spdk_static_fs_meta_for_traj();

    struct simple_query_engine rebuild_engine;
    struct my_file data_file_rebuild = {NULL, DATA_FILENAME, "r", SPDK_FS_MODE};
    struct my_file index_file_rebuild = {NULL, INDEX_FILENAME, "r", SPDK_FS_MODE};
    struct my_file meta_file_rebuild = {NULL, SEG_META_FILENAME, "r", SPDK_FS_MODE};
    init_query_engine_with_persistence(&rebuild_engine, &data_file_rebuild, &index_file_rebuild, &meta_file_rebuild);


    rebuild_query_engine_from_file(&rebuild_engine);
    clock_t start, end;
    start = clock();
    int engine_result = spatio_temporal_query_with_full_pushdown_batch_naive(&rebuild_engine, predicate, enable_host_index);
    end = clock();
    printf("[isp cpu batch] query time (total): %f\n", (double )(end - start));
    printf("[isp cpu batch] engine result: %d\n", engine_result);
    free_query_engine(&rebuild_engine);
}

static int exp_native_spatio_temporal_count_query_armcpu_full_pushdown_batch_naive_v1(struct spatio_temporal_range_predicate *predicate, struct simple_query_engine *rebuild_engine) {

    clock_t start, end;
    start = clock();
    int engine_result = spatio_temporal_count_query_with_pushdown_batch_naive(rebuild_engine, predicate);
    end = clock();
    printf("[isp cpu batch naive] query time (total time, including all): %f\n", (double )(end - start));
    printf("[isp cpu batch naive] engine result: %d\n", engine_result);
    return engine_result;
}

static int exp_native_spatio_temporal_count_query_armcpu_full_pushdown_batch_v1(struct spatio_temporal_range_predicate *predicate, struct simple_query_engine *rebuild_engine) {

    clock_t start, end;
    start = clock();
    int engine_result = spatio_temporal_count_query_with_pushdown_batch(rebuild_engine, predicate);
    end = clock();
    printf("[isp cpu batch] query time (total time, including all): %f\n", (double )(end - start));
    printf("[isp cpu batch] engine result: %d\n", engine_result);
    return engine_result;
}

static int exp_native_spatio_temporal_armcpu_full_pushdown_batch_naive_v1(struct spatio_temporal_range_predicate *predicate, struct simple_query_engine *rebuild_engine) {

    clock_t start, end;
    start = clock();
    int engine_result = spatio_temporal_query_with_full_pushdown_batch_naive(rebuild_engine, predicate, enable_host_index);
    end = clock();
    printf("[isp cpu batch naive] query time (total time, including all): %f\n", (double )(end - start));
    printf("[isp cpu batch naive] engine result: %d\n", engine_result);
    return engine_result;
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

static int exp_native_id_temporal_armcpu_full_pushdown_batch_naive_v1(struct id_temporal_predicate *predicate, struct simple_query_engine *rebuild_engine) {

    clock_t start, end;
    start = clock();
    int engine_result = id_temporal_query_with_full_pushdown_batch_naive(rebuild_engine, predicate, enable_host_index);
    end = clock();
    printf("[isp cpu batch naive] query time (total time, including all): %f\n", (double )(end - start));
    printf("[isp cpu batch naive] engine result: %d\n", engine_result);
    return engine_result;
}

static int exp_native_id_temporal_armcpu_full_pushdown_batch_v1(struct id_temporal_predicate *predicate, struct simple_query_engine *rebuild_engine) {

    clock_t start, end;
    start = clock();
    int engine_result = id_temporal_query_with_full_pushdown_batch(rebuild_engine, predicate, enable_host_index);
    end = clock();
    printf("[isp cpu batch] query time (total time, including all): %f\n", (double )(end - start));
    printf("[isp cpu batch] engine result: %d\n", engine_result);
    return engine_result;
}

static void exp_native_id_temporal_fpga_full_pushdown_batch(struct id_temporal_predicate *predicate) {

    init_and_mk_fs_for_traj(true);
    print_spdk_static_fs_meta_for_traj();

    struct simple_query_engine rebuild_engine;
    struct my_file data_file_rebuild = {NULL, DATA_FILENAME, "r", SPDK_FS_MODE};
    struct my_file index_file_rebuild = {NULL, INDEX_FILENAME, "r", SPDK_FS_MODE};
    struct my_file meta_file_rebuild = {NULL, SEG_META_FILENAME, "r", SPDK_FS_MODE};
    init_query_engine_with_persistence(&rebuild_engine, &data_file_rebuild, &index_file_rebuild, &meta_file_rebuild);


    rebuild_query_engine_from_file(&rebuild_engine);
    clock_t start, end;
    start = clock();
    int engine_result = id_temporal_query_with_full_pushdown_fpga_batch(&rebuild_engine, predicate, enable_host_index);
    end = clock();
    printf("[isp fpga batch] id temporal query time (includin index lookup): %f\n", (double )(end - start));
    printf("[isp fpga batch] id temporal engine result: %d\n", engine_result);
    free_query_engine(&rebuild_engine);
}

static void exp_native_id_temporal_armcpu_full_pushdown_batch(struct id_temporal_predicate *predicate) {
    init_and_mk_fs_for_traj(true);
    print_spdk_static_fs_meta_for_traj();

    struct simple_query_engine rebuild_engine;
    struct my_file data_file_rebuild = {NULL, DATA_FILENAME, "r", SPDK_FS_MODE};
    struct my_file index_file_rebuild = {NULL, INDEX_FILENAME, "r", SPDK_FS_MODE};
    struct my_file meta_file_rebuild = {NULL, SEG_META_FILENAME, "r", SPDK_FS_MODE};
    init_query_engine_with_persistence(&rebuild_engine, &data_file_rebuild, &index_file_rebuild, &meta_file_rebuild);


    rebuild_query_engine_from_file(&rebuild_engine);
    clock_t start, end;
    start = clock();
    int engine_result = id_temporal_query_with_full_pushdown_batch(&rebuild_engine, predicate, enable_host_index);
    end = clock();
    printf("[isp cpu batch] query time (total): %f\n", (double )(end - start));
    printf("[isp cpu batch] engine result: %d\n", engine_result);
    free_query_engine(&rebuild_engine);

}


static void exp_native_spatio_temporal_adaptive_pushdown(struct spatio_temporal_range_predicate *predicate) {

    init_and_mk_fs_for_traj(true);
    print_spdk_static_fs_meta_for_traj();

    struct simple_query_engine rebuild_engine;
    struct my_file data_file_rebuild = {NULL, DATA_FILENAME, "r", SPDK_FS_MODE};
    struct my_file index_file_rebuild = {NULL, INDEX_FILENAME, "r", SPDK_FS_MODE};
    struct my_file meta_file_rebuild = {NULL, SEG_META_FILENAME, "r", SPDK_FS_MODE};
    init_query_engine_with_persistence(&rebuild_engine, &data_file_rebuild, &index_file_rebuild, &meta_file_rebuild);


    rebuild_query_engine_from_file(&rebuild_engine);
    clock_t start, end;
    start = clock();
    int engine_result = spatio_temporal_query_with_adaptive_pushdown(&rebuild_engine, predicate, enable_host_index);
    end = clock();
    printf("[isp adaptive fpga] query time (total): %f\n", (double )(end - start));
    printf("[isp adaptive fpga] engine result: %d\n", engine_result);
    free_query_engine(&rebuild_engine);
}

static void exp_native_spatio_temporal_adaptive_pushdown_batch(struct spatio_temporal_range_predicate *predicate) {

    init_and_mk_fs_for_traj(true);
    print_spdk_static_fs_meta_for_traj();

    struct simple_query_engine rebuild_engine;
    struct my_file data_file_rebuild = {NULL, DATA_FILENAME, "r", SPDK_FS_MODE};
    struct my_file index_file_rebuild = {NULL, INDEX_FILENAME, "r", SPDK_FS_MODE};
    struct my_file meta_file_rebuild = {NULL, SEG_META_FILENAME, "r", SPDK_FS_MODE};
    init_query_engine_with_persistence(&rebuild_engine, &data_file_rebuild, &index_file_rebuild, &meta_file_rebuild);


    rebuild_query_engine_from_file(&rebuild_engine);
    clock_t start, end;
    start = clock();
    int engine_result = spatio_temporal_query_with_adaptive_pushdown_batch(&rebuild_engine, predicate, enable_host_index);
    end = clock();
    printf("[isp adaptive fpga] query time (includin index lookup): %f\n", (double )(end - start));
    printf("[isp adaptive fpga] engine result: %d\n", engine_result);
    free_query_engine(&rebuild_engine);
}

static void exp_native_spatio_temporal_adaptive_pushdown_batch_v1(struct spatio_temporal_range_predicate *predicate, struct simple_query_engine *rebuild_engine) {


    clock_t start, end;
    start = clock();
    int engine_result = spatio_temporal_query_with_adaptive_pushdown_batch(rebuild_engine, predicate, enable_host_index);
    end = clock();
    printf("[isp adaptive cpu] query time (total, including all): %f\n", (double )(end - start));
    printf("[isp adaptive cpu] engine result: %d\n", engine_result);

}

static void exp_native_id_temporal_adaptive_pushdown_batch_v1(struct id_temporal_predicate *predicate, struct simple_query_engine *rebuild_engine) {


    clock_t start, end;
    start = clock();
    int engine_result = id_temporal_query_with_adaptive_pushdown_batch(rebuild_engine, predicate, enable_host_index);
    end = clock();
    printf("[isp adaptive fpga] query time (total, including all): %f\n", (double )(end - start));
    printf("[isp adaptive fpga] engine result: %d\n", engine_result);

}


static void exp_native_spatio_temporal_host_device_parallel_batch(struct spatio_temporal_range_predicate *predicate) {

    init_and_mk_fs_for_traj(true);
    print_spdk_static_fs_meta_for_traj();

    struct simple_query_engine rebuild_engine;
    struct my_file data_file_rebuild = {NULL, DATA_FILENAME, "r", SPDK_FS_MODE};
    struct my_file index_file_rebuild = {NULL, INDEX_FILENAME, "r", SPDK_FS_MODE};
    struct my_file meta_file_rebuild = {NULL, SEG_META_FILENAME, "r", SPDK_FS_MODE};
    init_query_engine_with_persistence(&rebuild_engine, &data_file_rebuild, &index_file_rebuild, &meta_file_rebuild);


    rebuild_query_engine_from_file(&rebuild_engine);
    clock_t start, end;
    start = clock();
    int engine_result = spatio_temporal_query_with_host_device_parallel_batch(&rebuild_engine, predicate, enable_host_index);
    end = clock();
    printf("[isp host & device parallel] query time (includin index lookup): %f\n", (double )(end - start));
    printf("[isp host & device parallel] engine result: %d\n", engine_result);
    free_query_engine(&rebuild_engine);
}

static int exp_native_spatio_temporal_host_device_parallel_batch_v1(struct spatio_temporal_range_predicate *predicate, struct simple_query_engine *rebuild_engine) {


    clock_t start, end;
    start = clock();
    int engine_result = spatio_temporal_query_with_host_device_parallel_batch(rebuild_engine, predicate, enable_host_index);
    end = clock();
    printf("[isp host & device parallel] query time (total, including all): %f\n", (double )(end - start));
    printf("[isp host & device parallel] engine result: %d\n", engine_result);
    return engine_result;
}

static void exp_native_spatio_temporal_host_v1(struct spatio_temporal_range_predicate *predicate, struct simple_query_engine *rebuild_engine) {

    clock_t start, end;
    start = clock();
    int engine_result = spatio_temporal_query_without_pushdown(rebuild_engine, predicate, enable_host_index);
    end = clock();
    printf("[host] query time (includin index lookup): %f\n", (double )(end - start));
    printf("engine result: %d\n", engine_result);


}

static void exp_native_spatio_temporal_armcpu_full_pushdown_v1(struct spatio_temporal_range_predicate *predicate, struct simple_query_engine *rebuild_engine) {

    clock_t start, end;
    start = clock();
    int engine_result = spatio_temporal_query_with_full_pushdown(rebuild_engine, predicate, enable_host_index);
    end = clock();
    printf("[isp cpu] query time (including index lookup): %f\n", (double )(end - start));
    printf("[isp cpu] engine result: %d\n", engine_result);


}

static void exp_native_spatio_temporal_fpga_full_pushdown_v1(struct spatio_temporal_range_predicate *predicate, struct simple_query_engine *rebuild_engine) {


    clock_t start, end;
    start = clock();
    int engine_result = spatio_temporal_query_with_full_pushdown_fpga(rebuild_engine, predicate, enable_host_index);
    end = clock();
    printf("[isp fpga] query time (includin index lookup): %f\n", (double )(end - start));
    printf("[isp fpga] engine result: %d\n", engine_result);

}


static void exp_native_id_temporal_fpga_full_pushdown_v1(struct id_temporal_predicate *predicate, struct simple_query_engine *rebuild_engine) {


    clock_t start, end;
    start = clock();
    int engine_result = id_temporal_query_with_full_pushdown_fpga(rebuild_engine, predicate, enable_host_index);
    end = clock();
    printf("[isp fpga] id temporal query time (includin index lookup): %f\n", (double )(end - start));
    printf("[isp fpga] id temporal engine result: %d\n", engine_result);

}

static void exp_native_id_temporal_armcpu_full_pushdown_v1(struct id_temporal_predicate *predicate, struct simple_query_engine *rebuild_engine) {

    clock_t start, end;
    start = clock();
    int engine_result = id_temporal_query_with_full_pushdown(rebuild_engine, predicate, enable_host_index);
    end = clock();
    printf("[isp cpu] query time (including index lookup): %f\n", (double )(end - start));
    printf("[isp cpu] engine result: %d\n", engine_result);


}

static void exp_native_id_temporal_host_v1(struct id_temporal_predicate *predicate, struct simple_query_engine *rebuild_engine) {

    clock_t start, end;
    start = clock();
    int engine_result = id_temporal_query_without_pushdown(rebuild_engine, predicate, enable_host_index);
    end = clock();
    printf("[host] id temporal query time (includin index lookup): %f\n", (double )(end - start));
    printf("engine result: %d\n", engine_result);

}

static void run_on_porto_data() {
    //ingest_and_flush_data();
    //struct spatio_temporal_range_predicate predicate = {0, 2147483647, 0, 2147483647, 1372636853, 1372757330};
    struct spatio_temporal_range_predicate predicate = {7983124, 8100193, 12214847, 12220845, 0, 1386524941};

    clock_t start, end;
    start = clock();
    //exp_native_spatio_temporal_host(&predicate);
    //exp_native_spatio_temporal_armcpu_full_pushdown(&predicate);
    exp_native_spatio_temporal_fpga_full_pushdown(&predicate);
    //exp_native_spatio_temporal_adaptive_pushdown(&predicate);
    end = clock();
    printf("total time: %f\n",(double)(end-start));
}

static void run_on_porto_data_multithread() {
    //ingest_and_flush_data();
    //struct spatio_temporal_range_predicate predicate = {0, 2147483647, 0, 2147483647, 1372636853, 1372757330};
    //struct spatio_temporal_range_predicate predicate = {7983124, 8100193, 12214847, 12220845, 0, 1386524941};
    struct spatio_temporal_range_predicate predicate = {7983124, 8000193, 12214847, 12220845, 0, 1386524941};
    struct spatio_temporal_range_predicate breakdown_query = {7988275, 8011577, 12225298, 12271901, 1372772085, 1375364085};


    clock_t start, end;
    start = clock();

    exp_native_spatio_temporal_host_batch(&predicate);
    //exp_native_spatio_temporal_armcpu_full_pushdown_batch(&breakdown_query);
    //exp_native_spatio_temporal_fpga_full_pushdown_batch(&predicate);
    //exp_native_spatio_temporal_adaptive_pushdown_batch(&breakdown_query);
    //exp_native_spatio_temporal_host_device_parallel_batch(&breakdown_query);

    end = clock();
    printf("total time: %f\n",(double)(end-start));

}

static void run_on_porto_data_multithread_batch() {

    init_and_mk_fs_for_traj(true);
    print_spdk_static_fs_meta_for_traj();

    struct simple_query_engine rebuild_engine;
    struct my_file data_file_rebuild = {NULL, DATA_FILENAME, "r", SPDK_FS_MODE};
    struct my_file index_file_rebuild = {NULL, INDEX_FILENAME, "r", SPDK_FS_MODE};
    struct my_file meta_file_rebuild = {NULL, SEG_META_FILENAME, "r", SPDK_FS_MODE};
    init_query_engine_with_persistence(&rebuild_engine, &data_file_rebuild, &index_file_rebuild, &meta_file_rebuild);


    rebuild_query_engine_from_file(&rebuild_engine);

    FILE *query_fp = fopen("/home/yangguo/Codes/groundhog/traj-block-format/queryfile/groundhog_3000w_st_7d_1.query", "r");
    // read queries
    int query_num = 10;
    struct spatio_temporal_range_predicate **predicates = allocate_spatio_temporal_predicate_mem(query_num);
    read_spatio_temporal_queries_from_csv(query_fp, predicates, query_num);

    clock_t start, end;
    start = clock();
    for (int i = 0; i < query_num; i++) {
        printf("time min: %d, time max: %d, lon min: %d, lon max: %d, lat min: %d, lat max: %d\n", predicates[i]->time_min,
               predicates[i]->time_max, predicates[i]->lon_min, predicates[i]->lon_max, predicates[i]->lat_min, predicates[i]->lat_max);
        exp_native_spatio_temporal_host_batch_v1(predicates[i], &rebuild_engine);
        printf("\n");

        exp_native_spatio_temporal_armcpu_full_pushdown_batch_v1(predicates[i], &rebuild_engine);
        printf("\n");

        exp_native_spatio_temporal_adaptive_pushdown_batch_v1(predicates[i], &rebuild_engine);
        printf("\n");

        //exp_native_spatio_temporal_host_device_parallel_batch_v1(predicates[i], &rebuild_engine);

        //printf("selectivity: %f\n", (1.0 * result_count / 30000000.0));
        printf("\n\n\n");
    }
    end = clock();
    printf("total time: %f\n",(double)(end-start));

    free_spatio_temporal_predicate_mem(predicates, query_num);
    free_query_engine(&rebuild_engine);
}


static void run_on_porto_data_id_temporal_multithread_batch() {

    init_and_mk_fs_for_traj(true);
    print_spdk_static_fs_meta_for_traj();

    struct simple_query_engine rebuild_engine;
    struct my_file data_file_rebuild = {NULL, DATA_FILENAME, "r", SPDK_FS_MODE};
    struct my_file index_file_rebuild = {NULL, INDEX_FILENAME, "r", SPDK_FS_MODE};
    struct my_file meta_file_rebuild = {NULL, SEG_META_FILENAME, "r", SPDK_FS_MODE};
    init_query_engine_with_persistence(&rebuild_engine, &data_file_rebuild, &index_file_rebuild, &meta_file_rebuild);


    rebuild_query_engine_from_file(&rebuild_engine);

    FILE *query_fp = fopen("/home/yangguo/Codes/groundhog/traj-block-format/queryfile/groundhog_3000w_id_30d.query", "r");
    // read queries
    int query_num = 50;
    struct id_temporal_predicate **predicates = allocate_id_temporal_predicate_mem(query_num);
    read_id_temporal_queries_from_csv(query_fp, predicates, query_num);

    clock_t start, end;
    start = clock();
    for (int i = 0; i < query_num; i++) {
        printf("oid: %d, time min: %d, time max: %d\n", predicates[i]->oid, predicates[i]->time_min,
               predicates[i]->time_max);
        exp_native_id_temporal_host_batch_v1(predicates[i], &rebuild_engine);
        printf("\n");

        exp_native_id_temporal_armcpu_full_pushdown_batch_v1(predicates[i], &rebuild_engine);
        printf("\n");

        exp_native_id_temporal_adaptive_pushdown_batch_v1(predicates[i], &rebuild_engine);
        printf("\n");

        //exp_native_spatio_temporal_host_device_parallel_batch_v1(predicates[i], &rebuild_engine);

        //printf("selectivity: %f\n", (1.0 * result_count / 30000000.0));
        printf("\n\n\n");
    }
    end = clock();
    printf("total time: %f\n",(double)(end-start));

    free_id_temporal_predicate_mem(predicates, query_num);
    free_query_engine(&rebuild_engine);
}


static void run_on_synthetic_data() {
    //ingest_and_flush_synthetic_data(4096);

    // predicate for 512 MB data
    // 1 blocks
    //struct spatio_temporal_range_predicate predicate = {30801685, 30801919, 30801685, 30801919, 30801685, 30801919};

    // 13 blocks
    //struct spatio_temporal_range_predicate predicate = {30798877, 30801919, 30798877, 30801919, 30798877, 30801919};

    // 131 blocks
    //struct spatio_temporal_range_predicate predicate = {30771134, 30801919, 30771134, 30801919, 30771134, 30801919};

    // 1310 blocks
    //struct spatio_temporal_range_predicate predicate = {30494069, 30801919, 30494069, 30801919, 30494069, 30801919};

    // 13107 blocks
    //struct spatio_temporal_range_predicate predicate = {27751774, 30801919, 27751774, 30801919, 27751774, 30801919};

    // 26214 blocks
    //struct spatio_temporal_range_predicate predicate = {24641629, 30801919, 24641629, 30801919, 24641629, 30801919};

    // 52428 blocks
    //struct spatio_temporal_range_predicate predicate = {18481339, 30801919, 18481339, 30801919, 18481339, 30801919};

    // 78643 blocks
    //struct spatio_temporal_range_predicate predicate = {12320814, 30801919, 12320814, 30801919, 12320814, 30801919};

    // 104857 blocks
    //struct spatio_temporal_range_predicate predicate = {6160524, 30801919, 6160524, 30801919, 6160524, 30801919};

    // 131072 blocks
    //struct spatio_temporal_range_predicate predicate = {0, 30801919, 0, 30801919, 0, 30801919};


    // predicate for 4GB data
    // 1 blocks
    struct spatio_temporal_range_predicate predicate_4g_0 = {246415125, 246415359, 246415125, 246415359, 246415125, 246415359};

    // 104 blocks
    struct spatio_temporal_range_predicate predicate_4g_1 = {246390919, 246415359, 246390919, 246415359, 246390919, 246415359};

    // 1048 blocks
    struct spatio_temporal_range_predicate predicate_4g_2 = {246169079, 246415359, 246169079, 246415359, 246169079, 246415359};

    // 10485 blocks
    struct spatio_temporal_range_predicate predicate_4g_3 = {243951384, 246415359, 243951384, 246415359, 243951384, 246415359};

    // 104857 blocks
    struct spatio_temporal_range_predicate predicate_4g_4 = {221773964, 246415359, 221773964, 246415359, 221773964, 246415359};

    // 209715 blocks
    struct spatio_temporal_range_predicate predicate_4g_5 = {197132334, 246415359, 197132334, 246415359, 197132334, 246415359};

    // 419430 blocks
    struct spatio_temporal_range_predicate predicate_4g_6 = {147849309, 246415359, 147849309, 246415359, 147849309, 246415359};

    // 629145 blocks
    struct spatio_temporal_range_predicate predicate_4g_7 = {98566284, 246415359, 98566284, 246415359, 98566284, 246415359};

    // 838860 blocks
    struct spatio_temporal_range_predicate predicate_4g_8 = {49283259, 246415359, 49283259, 246415359, 49283259, 246415359};

    // 1048576 blocks
    struct spatio_temporal_range_predicate predicate_4g_9 = {0, 246415359, 0, 246415359, 0, 246415359};

    struct spatio_temporal_range_predicate predicate = {0, 2949119*0, 0, 29491199, 0, 29491199};

    clock_t start, end;
    start = clock();
    //exp_native_spatio_temporal_host(&predicate);
    //exp_native_spatio_temporal_adaptive_pushdown(&predicate);
    exp_native_spatio_temporal_armcpu_full_pushdown(&predicate);
    //exp_native_spatio_temporal_fpga_full_pushdown(&predicate);
    end = clock();
    printf("total time: %f\n",(double)(end-start));
    printf("---------------------------------\n");
}

static void run_on_synthetic_data_multithread() {

    struct spatio_temporal_range_predicate predicate = {0, 2949119 * 3, 0, 29491199, 0, 29491199};

    struct id_temporal_predicate predicate_id = {12, 0,  2949119*1};

    clock_t start, end;
    start = clock();

    exp_native_spatio_temporal_host_batch(&predicate);
    exp_native_spatio_temporal_armcpu_full_pushdown_batch_naive(&predicate);
    exp_native_spatio_temporal_armcpu_full_pushdown_batch(&predicate);
    exp_native_spatio_temporal_adaptive_pushdown_batch(&predicate);
    //exp_native_spatio_temporal_fpga_full_pushdown_batch(&predicate);
    //exp_native_spatio_temporal_host_device_parallel_batch(&predicate);

    //exp_native_id_temporal_host_batch(&predicate_id);
    //exp_native_id_temporal_armcpu_full_pushdown_batch(&predicate_id);
    //exp_native_id_temporal_fpga_full_pushdown_batch(&predicate_id);

    end = clock();
    printf("total time: %f\n",(double)(end-start));
    printf("---------------------------------\n");
}

void exp_spatio_temporal_query_synthetic_scan() {
    init_and_mk_fs_for_traj(true);
    print_spdk_static_fs_meta_for_traj();

    struct simple_query_engine rebuild_engine;
    struct my_file data_file_rebuild = {NULL, DATA_FILENAME, "r", SPDK_FS_MODE};
    struct my_file index_file_rebuild = {NULL, INDEX_FILENAME, "r", SPDK_FS_MODE};
    struct my_file meta_file_rebuild = {NULL, SEG_META_FILENAME, "r", SPDK_FS_MODE};
    init_query_engine_with_persistence(&rebuild_engine, &data_file_rebuild, &index_file_rebuild, &meta_file_rebuild);


    rebuild_query_engine_from_file(&rebuild_engine);

    int query_num = 10;
    struct spatio_temporal_range_predicate **predicates = allocate_spatio_temporal_predicate_mem(query_num+1);
    for (int i = 0; i <= query_num; i++) {
        //{0, 2949119 * 0, 0, 29491199, 0, 29491199};
        predicates[i]->time_min = 0;
        predicates[i]->time_max = 3185049 * i;
        predicates[i]->lon_min = 0;
        predicates[i]->lon_max = 3185049 * i;
        predicates[i]->lat_min = 0;
        predicates[i]->lat_max = 3185049 * i;
    }

    clock_t start, end;
    start = clock();
    /*struct spatio_temporal_range_predicate predicate = {0, 2949119 * 0, 0, 29491199, 0, 29491199};
    exp_native_spatio_temporal_armcpu_full_pushdown_batch_naive_v1(predicates[0], &rebuild_engine);*/
    printf("\n");
    for (int i = 0; i <= 10; i++) {
        printf("time min: %d, time max: %d, lon min: %d, lon max: %d, lat min: %d, lat max: %d\n", predicates[i]->time_min,
               predicates[i]->time_max, predicates[i]->lon_min, predicates[i]->lon_max, predicates[i]->lat_min, predicates[i]->lat_max);
        exp_native_spatio_temporal_host_batch_v1(predicates[i], &rebuild_engine);
        printf("\n");

        exp_native_spatio_temporal_armcpu_full_pushdown_batch_naive_v1(predicates[i], &rebuild_engine);
        printf("\n");

        exp_native_spatio_temporal_armcpu_full_pushdown_batch_v1(predicates[i], &rebuild_engine);
        printf("\n");

        exp_native_spatio_temporal_adaptive_pushdown_batch_v1(predicates[i], &rebuild_engine);
        printf("\n");

        //exp_native_spatio_temporal_host_device_parallel_batch_v1(predicates[i], &rebuild_engine);

        //printf("selectivity: %f\n", (1.0 * result_count / 30000000.0));
        printf("\n\n\n");
    }
    end = clock();
    printf("total time: %f\n",(double)(end-start));

    free_spatio_temporal_predicate_mem(predicates, query_num);
    free_query_engine(&rebuild_engine);
}

static void run_on_synthetic_data_batch() {
    init_and_mk_fs_for_traj(true);
    print_spdk_static_fs_meta_for_traj();

    struct simple_query_engine rebuild_engine;
    struct my_file data_file_rebuild = {NULL, DATA_FILENAME, "r", SPDK_FS_MODE};
    struct my_file index_file_rebuild = {NULL, INDEX_FILENAME, "r", SPDK_FS_MODE};
    struct my_file meta_file_rebuild = {NULL, SEG_META_FILENAME, "r", SPDK_FS_MODE};
    init_query_engine_with_persistence(&rebuild_engine, &data_file_rebuild, &index_file_rebuild, &meta_file_rebuild);


    rebuild_query_engine_from_file(&rebuild_engine);

    //ingest_and_flush_synthetic_data(4096);

    // predicate for 512 MB data



    // 1 blocks
    struct spatio_temporal_range_predicate predicate_512mb_0 = {30801685, 30801919, 30801685, 30801919, 30801685, 30801919};

    // 13 blocks
    struct spatio_temporal_range_predicate predicate_512mb_1 = {30798877, 30801919, 30798877, 30801919, 30798877, 30801919};

    // 131 blocks
    struct spatio_temporal_range_predicate predicate_512mb_2 = {30771134, 30801919, 30771134, 30801919, 30771134, 30801919};

    // 1310 blocks
    struct spatio_temporal_range_predicate predicate_512mb_3 = {30494069, 30801919, 30494069, 30801919, 30494069, 30801919};

    // 13107 blocks
    struct spatio_temporal_range_predicate predicate_512mb_4 = {27751774, 30801919, 27751774, 30801919, 27751774, 30801919};

    // 26214 blocks
    struct spatio_temporal_range_predicate predicate_512mb_5 = {24641629, 30801919, 24641629, 30801919, 24641629, 30801919};

    // 52428 blocks
    struct spatio_temporal_range_predicate predicate_512mb_6 = {18481339, 30801919, 18481339, 30801919, 18481339, 30801919};

    // 78643 blocks
    struct spatio_temporal_range_predicate predicate_512mb_7 = {12320814, 30801919, 12320814, 30801919, 12320814, 30801919};

    // 104857 blocks
    struct spatio_temporal_range_predicate predicate_512mb_8 = {6160524, 30801919, 6160524, 30801919, 6160524, 30801919};

    // 131072 blocks
    struct spatio_temporal_range_predicate predicate_512mb_9 = {0, 30801919, 0, 30801919, 0, 30801919};


    // predicate for 4GB data
    // 1 blocks
    struct spatio_temporal_range_predicate predicate_4g_0 = {246415125, 246415359, 246415125, 246415359, 246415125, 246415359};

    // 104 blocks
    struct spatio_temporal_range_predicate predicate_4g_1 = {246390919, 246415359, 246390919, 246415359, 246390919, 246415359};

    // 1048 blocks
    struct spatio_temporal_range_predicate predicate_4g_2 = {246169079, 246415359, 246169079, 246415359, 246169079, 246415359};

    // 10485 blocks
    struct spatio_temporal_range_predicate predicate_4g_3 = {243951384, 246415359, 243951384, 246415359, 243951384, 246415359};

    // 104857 blocks
    struct spatio_temporal_range_predicate predicate_4g_4 = {221773964, 246415359, 221773964, 246415359, 221773964, 246415359};

    // 209715 blocks
    struct spatio_temporal_range_predicate predicate_4g_5 = {197132334, 246415359, 197132334, 246415359, 197132334, 246415359};

    // 419430 blocks
    struct spatio_temporal_range_predicate predicate_4g_6 = {147849309, 246415359, 147849309, 246415359, 147849309, 246415359};

    // 629145 blocks
    struct spatio_temporal_range_predicate predicate_4g_7 = {98566284, 246415359, 98566284, 246415359, 98566284, 246415359};

    // 838860 blocks
    struct spatio_temporal_range_predicate predicate_4g_8 = {49283259, 246415359, 49283259, 246415359, 49283259, 246415359};

    // 1048576 blocks
    struct spatio_temporal_range_predicate predicate_4g_9 = {0, 246415359, 0, 246415359, 0, 246415359};


    clock_t start, end;
    start = clock();
    /*exp_native_spatio_temporal_host_v1(&predicate_4g_0, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_spatio_temporal_armcpu_full_pushdown_v1(&predicate_4g_0, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_spatio_temporal_fpga_full_pushdown_v1(&predicate_4g_0, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_spatio_temporal_host_v1(&predicate_4g_1, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_spatio_temporal_armcpu_full_pushdown_v1(&predicate_4g_1, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_spatio_temporal_fpga_full_pushdown_v1(&predicate_4g_1, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_spatio_temporal_host_v1(&predicate_4g_2, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_spatio_temporal_armcpu_full_pushdown_v1(&predicate_4g_2, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_spatio_temporal_fpga_full_pushdown_v1(&predicate_4g_2, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_spatio_temporal_host_v1(&predicate_4g_3, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_spatio_temporal_armcpu_full_pushdown_v1(&predicate_4g_3, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_spatio_temporal_fpga_full_pushdown_v1(&predicate_4g_3, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_spatio_temporal_host_v1(&predicate_4g_4, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_spatio_temporal_armcpu_full_pushdown_v1(&predicate_4g_4, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_spatio_temporal_fpga_full_pushdown_v1(&predicate_4g_4, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_spatio_temporal_host_v1(&predicate_4g_5, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_spatio_temporal_armcpu_full_pushdown_v1(&predicate_4g_5, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_spatio_temporal_fpga_full_pushdown_v1(&predicate_4g_5, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_spatio_temporal_host_v1(&predicate_4g_6, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_spatio_temporal_armcpu_full_pushdown_v1(&predicate_4g_6, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_spatio_temporal_fpga_full_pushdown_v1(&predicate_4g_6, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_spatio_temporal_host_v1(&predicate_4g_7, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_spatio_temporal_armcpu_full_pushdown_v1(&predicate_4g_7, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_spatio_temporal_fpga_full_pushdown_v1(&predicate_4g_7, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_spatio_temporal_host_v1(&predicate_4g_8, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_spatio_temporal_armcpu_full_pushdown_v1(&predicate_4g_8, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_spatio_temporal_fpga_full_pushdown_v1(&predicate_4g_8, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_spatio_temporal_host_v1(&predicate_4g_9, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_spatio_temporal_armcpu_full_pushdown_v1(&predicate_4g_9, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_spatio_temporal_fpga_full_pushdown_v1(&predicate_4g_9, &rebuild_engine);*/

    exp_native_spatio_temporal_host_v1(&predicate_512mb_0, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_spatio_temporal_armcpu_full_pushdown_v1(&predicate_512mb_0, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_spatio_temporal_fpga_full_pushdown_v1(&predicate_512mb_0, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_spatio_temporal_host_v1(&predicate_512mb_1, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_spatio_temporal_armcpu_full_pushdown_v1(&predicate_512mb_1, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_spatio_temporal_fpga_full_pushdown_v1(&predicate_512mb_1, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_spatio_temporal_host_v1(&predicate_512mb_2, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_spatio_temporal_armcpu_full_pushdown_v1(&predicate_512mb_2, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_spatio_temporal_fpga_full_pushdown_v1(&predicate_512mb_2, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_spatio_temporal_host_v1(&predicate_512mb_3, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_spatio_temporal_armcpu_full_pushdown_v1(&predicate_512mb_3, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_spatio_temporal_fpga_full_pushdown_v1(&predicate_512mb_3, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_spatio_temporal_host_v1(&predicate_512mb_4, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_spatio_temporal_armcpu_full_pushdown_v1(&predicate_512mb_4, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_spatio_temporal_fpga_full_pushdown_v1(&predicate_512mb_4, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_spatio_temporal_host_v1(&predicate_512mb_5, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_spatio_temporal_armcpu_full_pushdown_v1(&predicate_512mb_5, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_spatio_temporal_fpga_full_pushdown_v1(&predicate_512mb_5, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_spatio_temporal_host_v1(&predicate_512mb_6, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_spatio_temporal_armcpu_full_pushdown_v1(&predicate_512mb_6, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_spatio_temporal_fpga_full_pushdown_v1(&predicate_512mb_6, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_spatio_temporal_host_v1(&predicate_512mb_7, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_spatio_temporal_armcpu_full_pushdown_v1(&predicate_512mb_7, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_spatio_temporal_fpga_full_pushdown_v1(&predicate_512mb_7, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_spatio_temporal_host_v1(&predicate_512mb_8, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_spatio_temporal_armcpu_full_pushdown_v1(&predicate_512mb_8, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_spatio_temporal_fpga_full_pushdown_v1(&predicate_512mb_8, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_spatio_temporal_host_v1(&predicate_512mb_9, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_spatio_temporal_armcpu_full_pushdown_v1(&predicate_512mb_9, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_spatio_temporal_fpga_full_pushdown_v1(&predicate_512mb_9, &rebuild_engine);
    end = clock();
    printf("total time: %f\n",(double)(end-start));

    free_query_engine(&rebuild_engine);
}

static void run_on_synthetic_data_id() {
    struct id_temporal_predicate predicate = {12, 0, 2949119*1};


    clock_t start, end;
    start = clock();
    //exp_native_id_temporal_host(&predicate);
    exp_native_id_temporal_fpga_full_pushdown(&predicate);
    //exp_native_id_temporal_armcpu_full_pushdown(&predicate);
    end = clock();
    printf("total time: %f\n",(double)(end-start));
    printf("---------------------------------\n");
}

static void run_on_synthetic_data_id_batch() {
    init_and_mk_fs_for_traj(true);
    print_spdk_static_fs_meta_for_traj();

    struct simple_query_engine rebuild_engine;
    struct my_file data_file_rebuild = {NULL, DATA_FILENAME, "r", SPDK_FS_MODE};
    struct my_file index_file_rebuild = {NULL, INDEX_FILENAME, "r", SPDK_FS_MODE};
    struct my_file meta_file_rebuild = {NULL, SEG_META_FILENAME, "r", SPDK_FS_MODE};
    init_query_engine_with_persistence(&rebuild_engine, &data_file_rebuild, &index_file_rebuild, &meta_file_rebuild);


    rebuild_query_engine_from_file(&rebuild_engine);

    //ingest_and_flush_synthetic_data(4096);

    // predicate for 512 MB data
    // 1 blocks
    struct id_temporal_predicate predicate_512mb_0 = {12, 30801685, 30801919};

    // 13 blocks
    struct id_temporal_predicate predicate_512mb_1 = {12, 30798877, 30801919};

    // 131 blocks
    struct id_temporal_predicate predicate_512mb_2 = {12, 30771134, 30801919};

    // 1310 blocks
    struct id_temporal_predicate predicate_512mb_3 = {12, 30494069, 30801919};

    // 13107 blocks
    struct id_temporal_predicate predicate_512mb_4 = {12, 27751774, 30801919};

    // 26214 blocks
    struct id_temporal_predicate predicate_512mb_5 = {12, 24641629, 30801919};

    // 52428 blocks
    struct id_temporal_predicate predicate_512mb_6 = {12, 18481339, 30801919};

    // 78643 blocks
    struct id_temporal_predicate predicate_512mb_7 = {12, 12320814, 30801919};

    // 104857 blocks
    struct id_temporal_predicate predicate_512mb_8 = {12, 6160524, 30801919};

    // 131072 blocks
    struct id_temporal_predicate predicate_512mb_9 = {12, 0, 30801919};



    clock_t start, end;
    start = clock();

    exp_native_id_temporal_host_v1(&predicate_512mb_0, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_id_temporal_armcpu_full_pushdown_v1(&predicate_512mb_0, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_id_temporal_fpga_full_pushdown_v1(&predicate_512mb_0, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_id_temporal_host_v1(&predicate_512mb_1, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_id_temporal_armcpu_full_pushdown_v1(&predicate_512mb_1, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_id_temporal_fpga_full_pushdown_v1(&predicate_512mb_1, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_id_temporal_host_v1(&predicate_512mb_2, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_id_temporal_armcpu_full_pushdown_v1(&predicate_512mb_2, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_id_temporal_fpga_full_pushdown_v1(&predicate_512mb_2, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_id_temporal_host_v1(&predicate_512mb_3, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_id_temporal_armcpu_full_pushdown_v1(&predicate_512mb_3, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_id_temporal_fpga_full_pushdown_v1(&predicate_512mb_3, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_id_temporal_host_v1(&predicate_512mb_4, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_id_temporal_armcpu_full_pushdown_v1(&predicate_512mb_4, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_id_temporal_fpga_full_pushdown_v1(&predicate_512mb_4, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_id_temporal_host_v1(&predicate_512mb_5, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_id_temporal_armcpu_full_pushdown_v1(&predicate_512mb_5, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_id_temporal_fpga_full_pushdown_v1(&predicate_512mb_5, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_id_temporal_host_v1(&predicate_512mb_6, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_id_temporal_armcpu_full_pushdown_v1(&predicate_512mb_6, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_id_temporal_fpga_full_pushdown_v1(&predicate_512mb_6, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_id_temporal_host_v1(&predicate_512mb_7, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_id_temporal_armcpu_full_pushdown_v1(&predicate_512mb_7, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_id_temporal_fpga_full_pushdown_v1(&predicate_512mb_7, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_id_temporal_host_v1(&predicate_512mb_8, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_id_temporal_armcpu_full_pushdown_v1(&predicate_512mb_8, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_id_temporal_fpga_full_pushdown_v1(&predicate_512mb_8, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_id_temporal_host_v1(&predicate_512mb_9, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_id_temporal_armcpu_full_pushdown_v1(&predicate_512mb_9, &rebuild_engine);
    printf("---------------------------------\n");
    exp_native_id_temporal_fpga_full_pushdown_v1(&predicate_512mb_9, &rebuild_engine);
    end = clock();
    printf("total time: %f\n",(double)(end-start));

    free_query_engine(&rebuild_engine);
}

// from here, it is the code for end-to-end-eva

/*void ingest_and_flush_porto_data_zcurve_mem() {
    ingest_and_flush_porto_data_zcurve(131072);
}

void ingest_and_flush_porto_data_time_oid_mem() {
    ingest_and_flush_porto_data(131072);
}*/

void ingest_and_flush_porto_data_zcurve_full() {
    //ingest_and_flush_porto_data_zcurve(277949);
    ingest_and_flush_porto_data_zcurve(197949);

}


void ingest_and_flush_porto_data_zcurve_for_segment_test() {
    // read all points data util to the end of the file because the number of blocks for different segment numbers is different
    ingest_and_flush_porto_data_zcurve_segment_num(INT32_MAX);

}

void ingest_and_flush_porto_data_zcurve_10x_full() {
    //ingest_and_flush_porto_data_zcurve(277949);
    ingest_and_flush_porto_data_10x_zcurve(1979490);
}

void ingest_and_flush_porto_data_time_oid_full() {
    //ingest_and_flush_porto_data(277949);
    ingest_and_flush_porto_data(197949);
}

void ingest_and_flush_porto_data_time_oid_10x_full() {
    //ingest_and_flush_porto_data(277949);
    ingest_and_flush_porto_data_10x(1979490);
}

/*
void ingest_and_flush_nyc_data_zcurve_mem() {
    ingest_and_flush_nyc_data_zcurve(131072);
}

void ingest_and_flush_nyc_data_time_oid_mem() {
    ingest_and_flush_nyc_data(131072);
}
*/

void ingest_and_flush_nyc_data_time_oid_full() {
    ingest_and_flush_nyc_data(1779499);
}

void ingest_and_flush_nyc_data_zcurve_full() {
    //ingest_and_flush_nyc_data_zcurve(2571096);
    ingest_and_flush_nyc_data_zcurve(1779499);
}

void exp_spatio_temporal_query_porto_scan() {
    init_and_mk_fs_for_traj(true);
    print_spdk_static_fs_meta_for_traj();

    struct simple_query_engine rebuild_engine;
    struct my_file data_file_rebuild = {NULL, DATA_FILENAME, "r", SPDK_FS_MODE};
    struct my_file index_file_rebuild = {NULL, INDEX_FILENAME, "r", SPDK_FS_MODE};
    struct my_file meta_file_rebuild = {NULL, SEG_META_FILENAME, "r", SPDK_FS_MODE};
    init_query_engine_with_persistence(&rebuild_engine, &data_file_rebuild, &index_file_rebuild, &meta_file_rebuild);


    rebuild_query_engine_from_file(&rebuild_engine);

    FILE *query_fp = fopen("/home/yangguo/Codes/groundhog/query-workload/porto_st_1.query", "r");
    // read queries
    int query_num = 4;
    struct spatio_temporal_range_predicate **predicates = allocate_spatio_temporal_predicate_mem(query_num);
    read_spatio_temporal_queries_from_csv(query_fp, predicates, query_num);

    clock_t start, end;
    start = clock();
    for (int i = 0; i < 4; i++) {
        printf("time min: %d, time max: %d, lon min: %d, lon max: %d, lat min: %d, lat max: %d\n", predicates[i]->time_min,
               predicates[i]->time_max, predicates[i]->lon_min, predicates[i]->lon_max, predicates[i]->lat_min, predicates[i]->lat_max);
        exp_native_spatio_temporal_host_batch_v1(predicates[i], &rebuild_engine);
        printf("\n");

        exp_native_spatio_temporal_armcpu_full_pushdown_batch_naive_v1(predicates[i], &rebuild_engine);
        printf("\n");

        exp_native_spatio_temporal_armcpu_full_pushdown_batch_v1(predicates[i], &rebuild_engine);
        printf("\n");

        exp_native_spatio_temporal_adaptive_pushdown_batch_v1(predicates[i], &rebuild_engine);
        printf("\n");

        //exp_native_spatio_temporal_host_device_parallel_batch_v1(predicates[i], &rebuild_engine);

        //printf("selectivity: %f\n", (1.0 * result_count / 30000000.0));
        printf("\n\n\n");
    }
    end = clock();
    printf("total time: %f\n",(double)(end-start));

    free_spatio_temporal_predicate_mem(predicates, query_num);
    free_query_engine(&rebuild_engine);
}

void exp_spatio_temporal_query_porto_scan_segment_num() {
    init_and_mk_fs_for_traj(true);
    print_spdk_static_fs_meta_for_traj();

    struct simple_query_engine rebuild_engine;
    struct my_file data_file_rebuild = {NULL, DATA_FILENAME, "r", SPDK_FS_MODE};
    struct my_file index_file_rebuild = {NULL, INDEX_FILENAME, "r", SPDK_FS_MODE};
    struct my_file meta_file_rebuild = {NULL, SEG_META_FILENAME, "r", SPDK_FS_MODE};
    init_query_engine_with_persistence(&rebuild_engine, &data_file_rebuild, &index_file_rebuild, &meta_file_rebuild);


    rebuild_query_engine_from_file(&rebuild_engine);

    FILE *query_fp = fopen("/home/yangguo/Codes/groundhog/query-workload/porto_st_1.query", "r");
    // read queries
    int query_num = 20;
    struct spatio_temporal_range_predicate **predicates = allocate_spatio_temporal_predicate_mem(query_num);
    read_spatio_temporal_queries_from_csv(query_fp, predicates, query_num);

    clock_t start, end;
    start = clock();
    for (int i = 0; i < query_num; i++) {
        if (i == 2 || i == 4 || i == 8 || i == 9 || i == 11 || i == 13 ) {
            printf("i: %d\n", i);
            printf("time min: %d, time max: %d, lon min: %d, lon max: %d, lat min: %d, lat max: %d\n",
                   predicates[i]->time_min,
                   predicates[i]->time_max, predicates[i]->lon_min, predicates[i]->lon_max, predicates[i]->lat_min,
                   predicates[i]->lat_max);
            exp_native_spatio_temporal_host_batch_v1(predicates[i], &rebuild_engine);
            printf("\n");

            /*exp_native_spatio_temporal_armcpu_full_pushdown_batch_naive_v1(predicates[i], &rebuild_engine);
            printf("\n");*/

            exp_native_spatio_temporal_armcpu_full_pushdown_batch_v1(predicates[i], &rebuild_engine);
            printf("\n");

            exp_native_spatio_temporal_adaptive_pushdown_batch_v1(predicates[i], &rebuild_engine);
            printf("\n");

            //exp_native_spatio_temporal_host_device_parallel_batch_v1(predicates[i], &rebuild_engine);

            //printf("selectivity: %f\n", (1.0 * result_count / 30000000.0));
            printf("\n\n\n");
        }
    }
    end = clock();
    printf("total time: %f\n",(double)(end-start));

    free_spatio_temporal_predicate_mem(predicates, query_num);
    free_query_engine(&rebuild_engine);
}

void exp_spatio_temporal_query_porto_index_scan() {
    init_and_mk_fs_for_traj(true);
    print_spdk_static_fs_meta_for_traj();

    struct simple_query_engine rebuild_engine;
    struct my_file data_file_rebuild = {NULL, DATA_FILENAME, "r", SPDK_FS_MODE};
    struct my_file index_file_rebuild = {NULL, INDEX_FILENAME, "r", SPDK_FS_MODE};
    struct my_file meta_file_rebuild = {NULL, SEG_META_FILENAME, "r", SPDK_FS_MODE};
    init_query_engine_with_persistence(&rebuild_engine, &data_file_rebuild, &index_file_rebuild, &meta_file_rebuild);


    rebuild_query_engine_from_file(&rebuild_engine);

    FILE *query_fp = fopen("/home/yangguo/Codes/groundhog/query-workload/porto_st_005.query", "r");
    // read queries
    int query_num = 4;
    struct spatio_temporal_range_predicate **predicates = allocate_spatio_temporal_predicate_mem(query_num);
    read_spatio_temporal_queries_from_csv(query_fp, predicates, query_num);

    clock_t start, end;
    start = clock();
    for (int i = 0; i < 4; i++) {
        printf("time min: %d, time max: %d, lon min: %d, lon max: %d, lat min: %d, lat max: %d\n", predicates[i]->time_min,
               predicates[i]->time_max, predicates[i]->lon_min, predicates[i]->lon_max, predicates[i]->lat_min, predicates[i]->lat_max);
        exp_native_spatio_temporal_host_batch_v1(predicates[i], &rebuild_engine);
        printf("\n");

        //exp_native_spatio_temporal_armcpu_full_pushdown_batch_naive_v1(predicates[i], &rebuild_engine);
        printf("\n");

        exp_native_spatio_temporal_armcpu_full_pushdown_batch_v1(predicates[i], &rebuild_engine);
        printf("\n");

        //exp_native_spatio_temporal_adaptive_pushdown_batch_v1(predicates[i], &rebuild_engine);
        printf("\n");

        //exp_native_spatio_temporal_host_device_parallel_batch_v1(predicates[i], &rebuild_engine);

        //printf("selectivity: %f\n", (1.0 * result_count / 30000000.0));
        printf("\n\n\n");
    }
    end = clock();
    printf("total time: %f\n",(double)(end-start));

    free_spatio_temporal_predicate_mem(predicates, query_num);
    free_query_engine(&rebuild_engine);
}

void exp_spatio_temporal_query_porto_index_scan_low_selectivity_add_host_io_opt() {
    init_and_mk_fs_for_traj(true);
    print_spdk_static_fs_meta_for_traj();

    struct simple_query_engine rebuild_engine;
    struct my_file data_file_rebuild = {NULL, DATA_FILENAME, "r", SPDK_FS_MODE};
    struct my_file index_file_rebuild = {NULL, INDEX_FILENAME, "r", SPDK_FS_MODE};
    struct my_file meta_file_rebuild = {NULL, SEG_META_FILENAME, "r", SPDK_FS_MODE};
    init_query_engine_with_persistence(&rebuild_engine, &data_file_rebuild, &index_file_rebuild, &meta_file_rebuild);


    rebuild_query_engine_from_file(&rebuild_engine);

    FILE *query_fp = fopen("/home/yangguo/Codes/groundhog/query-workload/porto_st_005.query", "r");
    // read queries
    int query_num = 20;
    struct spatio_temporal_range_predicate **predicates = allocate_spatio_temporal_predicate_mem(query_num);
    read_spatio_temporal_queries_from_csv(query_fp, predicates, query_num);

    clock_t start, end;
    start = clock();
    for (int i = 0; i < query_num; i++) {
        //if (i == 14 || i == 15 || i == 17) {
        if (i == 0 || i == 1 || i == 2 || i == 15) {
            printf("i: %d\n", i);
            printf("time min: %d, time max: %d, lon min: %d, lon max: %d, lat min: %d, lat max: %d\n",
                   predicates[i]->time_min,
                   predicates[i]->time_max, predicates[i]->lon_min, predicates[i]->lon_max, predicates[i]->lat_min,
                   predicates[i]->lat_max);
            exp_native_spatio_temporal_host_io_opt_batch_v1(predicates[i], &rebuild_engine);
            printf("\n");

            exp_native_spatio_temporal_host_batch_v1(predicates[i], &rebuild_engine);
            printf("\n");

            exp_native_spatio_temporal_armcpu_full_pushdown_batch_naive_v1(predicates[i], &rebuild_engine);
            printf("\n");

            exp_native_spatio_temporal_armcpu_full_pushdown_batch_v1(predicates[i], &rebuild_engine);
            printf("\n");

            exp_native_spatio_temporal_adaptive_pushdown_batch_v1(predicates[i], &rebuild_engine);
            printf("\n");

            //exp_native_spatio_temporal_host_device_parallel_batch_v1(predicates[i], &rebuild_engine);

            //printf("selectivity: %f\n", (1.0 * result_count / 30000000.0));
            printf("\n\n\n");
        }
    }
    end = clock();
    printf("total time: %f\n",(double)(end-start));

    free_spatio_temporal_predicate_mem(predicates, query_num);
    free_query_engine(&rebuild_engine);
}


void exp_spatio_temporal_query_porto_index_scan_segment_num() {
    init_and_mk_fs_for_traj(true);
    print_spdk_static_fs_meta_for_traj();

    struct simple_query_engine rebuild_engine;
    struct my_file data_file_rebuild = {NULL, DATA_FILENAME, "r", SPDK_FS_MODE};
    struct my_file index_file_rebuild = {NULL, INDEX_FILENAME, "r", SPDK_FS_MODE};
    struct my_file meta_file_rebuild = {NULL, SEG_META_FILENAME, "r", SPDK_FS_MODE};
    init_query_engine_with_persistence(&rebuild_engine, &data_file_rebuild, &index_file_rebuild, &meta_file_rebuild);


    rebuild_query_engine_from_file(&rebuild_engine);

    FILE *query_fp = fopen("/home/yangguo/Codes/groundhog/query-workload/porto_st_005.query", "r");
    // read queries
    int query_num = 20;
    struct spatio_temporal_range_predicate **predicates = allocate_spatio_temporal_predicate_mem(query_num);
    read_spatio_temporal_queries_from_csv(query_fp, predicates, query_num);

    clock_t start, end;
    start = clock();
    for (int i = 0; i < query_num; i++) {
        if (i == 14 || i == 15 || i == 17) {
            printf("i: %d\n", i);
            printf("time min: %d, time max: %d, lon min: %d, lon max: %d, lat min: %d, lat max: %d\n",
                   predicates[i]->time_min,
                   predicates[i]->time_max, predicates[i]->lon_min, predicates[i]->lon_max, predicates[i]->lat_min,
                   predicates[i]->lat_max);
            exp_native_spatio_temporal_host_batch_v1(predicates[i], &rebuild_engine);
            printf("\n");

            //exp_native_spatio_temporal_armcpu_full_pushdown_batch_naive_v1(predicates[i], &rebuild_engine);
            printf("\n");

            exp_native_spatio_temporal_armcpu_full_pushdown_batch_v1(predicates[i], &rebuild_engine);
            printf("\n");

            exp_native_spatio_temporal_adaptive_pushdown_batch_v1(predicates[i], &rebuild_engine);
            printf("\n");

            //exp_native_spatio_temporal_host_device_parallel_batch_v1(predicates[i], &rebuild_engine);

            //printf("selectivity: %f\n", (1.0 * result_count / 30000000.0));
            printf("\n\n\n");
        }
    }
    end = clock();
    printf("total time: %f\n",(double)(end-start));

    free_spatio_temporal_predicate_mem(predicates, query_num);
    free_query_engine(&rebuild_engine);
}


void exp_spatio_temporal_query_porto_index_scan_query_type() {
    init_and_mk_fs_for_traj(true);
    print_spdk_static_fs_meta_for_traj();

    struct simple_query_engine rebuild_engine;
    struct my_file data_file_rebuild = {NULL, DATA_FILENAME, "r", SPDK_FS_MODE};
    struct my_file index_file_rebuild = {NULL, INDEX_FILENAME, "r", SPDK_FS_MODE};
    struct my_file meta_file_rebuild = {NULL, SEG_META_FILENAME, "r", SPDK_FS_MODE};
    init_query_engine_with_persistence(&rebuild_engine, &data_file_rebuild, &index_file_rebuild, &meta_file_rebuild);


    rebuild_query_engine_from_file(&rebuild_engine);

    FILE *query_fp = fopen("/home/yangguo/Codes/groundhog/query-workload/porto_st_1.query", "r");
    // read queries
    int query_num = 6;
    struct spatio_temporal_range_predicate **predicates = allocate_spatio_temporal_predicate_mem(query_num);
    read_spatio_temporal_queries_from_csv(query_fp, predicates, query_num);

    clock_t start, end;
    start = clock();
    for (int i = 0; i < query_num; i++) {
        printf("time min: %d, time max: %d, lon min: %d, lon max: %d, lat min: %d, lat max: %d\n", predicates[i]->time_min,
               predicates[i]->time_max, predicates[i]->lon_min, predicates[i]->lon_max, predicates[i]->lat_min, predicates[i]->lat_max);
        exp_native_spatio_temporal_host_batch_v1(predicates[i], &rebuild_engine);
        exp_native_spatio_temporal_host_batch_v1(predicates[i], &rebuild_engine);
        printf("\n");

        exp_native_spatio_temporal_armcpu_full_pushdown_batch_v1(predicates[i], &rebuild_engine);
        exp_native_spatio_temporal_armcpu_full_pushdown_batch_v1(predicates[i], &rebuild_engine);
        printf("\n");

        exp_native_spatio_temporal_adaptive_pushdown_batch_v1(predicates[i], &rebuild_engine);
        exp_native_spatio_temporal_adaptive_pushdown_batch_v1(predicates[i], &rebuild_engine);
        printf("\n");

        //exp_native_spatio_temporal_host_device_parallel_batch_v1(predicates[i], &rebuild_engine);

        //printf("selectivity: %f\n", (1.0 * result_count / 30000000.0));
        printf("\n\n\n");
    }
    end = clock();
    printf("total time: %f\n",(double)(end-start));

    free_spatio_temporal_predicate_mem(predicates, query_num);
    free_query_engine(&rebuild_engine);
}

void exp_spatio_temporal_query_nyc_scan() {
    init_and_mk_fs_for_traj(true);
    print_spdk_static_fs_meta_for_traj();

    struct simple_query_engine rebuild_engine;
    struct my_file data_file_rebuild = {NULL, DATA_FILENAME, "r", SPDK_FS_MODE};
    struct my_file index_file_rebuild = {NULL, INDEX_FILENAME, "r", SPDK_FS_MODE};
    struct my_file meta_file_rebuild = {NULL, SEG_META_FILENAME, "r", SPDK_FS_MODE};
    init_query_engine_with_persistence(&rebuild_engine, &data_file_rebuild, &index_file_rebuild, &meta_file_rebuild);


    rebuild_query_engine_from_file(&rebuild_engine);

    //FILE *query_fp = fopen("/home/yangguo/Codes/groundhog/query-workload/nyc_st_1.query", "r");
    FILE *query_fp = fopen("/home/yangguo/Codes/groundhog/query-workload/nyc_st_05.query", "r");
    // read queries
    int query_num = 10;
    struct spatio_temporal_range_predicate **predicates = allocate_spatio_temporal_predicate_mem(query_num);
    read_spatio_temporal_queries_from_csv_nyc(query_fp, predicates, query_num);

    clock_t start, end;
    start = clock();
    for (int i = 0; i < query_num; i++) {

        if (i == 0 || i == 1 || i == 2 || i == 9 || i == 7 || i == 5) {
            printf("time min: %d, time max: %d, lon min: %d, lon max: %d, lat min: %d, lat max: %d\n",
                   predicates[i]->time_min,
                   predicates[i]->time_max, predicates[i]->lon_min, predicates[i]->lon_max, predicates[i]->lat_min,
                   predicates[i]->lat_max);

            exp_native_spatio_temporal_host_batch_v1(predicates[i], &rebuild_engine);
            //exp_native_spatio_temporal_host_batch_v1(predicates[i], &rebuild_engine);
            printf("\n");

            exp_native_spatio_temporal_armcpu_full_pushdown_batch_naive_v1(predicates[i], &rebuild_engine);
            printf("\n");

            exp_native_spatio_temporal_armcpu_full_pushdown_batch_v1(predicates[i], &rebuild_engine);
            //exp_native_spatio_temporal_armcpu_full_pushdown_batch_v1(predicates[i], &rebuild_engine);
            printf("\n");

            exp_native_spatio_temporal_adaptive_pushdown_batch_v1(predicates[i], &rebuild_engine);
            printf("\n");

            //exp_native_spatio_temporal_host_device_parallel_batch_v1(predicates[i], &rebuild_engine);
            //printf("selectivity: %f\n", (1.0 * result_count / 30000000.0));
            printf("\n\n\n");
        }

    }
    end = clock();
    printf("total time: %f\n",(double)(end-start));

    free_spatio_temporal_predicate_mem(predicates, query_num);
    free_query_engine(&rebuild_engine);
}

void exp_spatio_temporal_query_nyc_index_scan() {
    init_and_mk_fs_for_traj(true);
    print_spdk_static_fs_meta_for_traj();

    struct simple_query_engine rebuild_engine;
    struct my_file data_file_rebuild = {NULL, DATA_FILENAME, "r", SPDK_FS_MODE};
    struct my_file index_file_rebuild = {NULL, INDEX_FILENAME, "r", SPDK_FS_MODE};
    struct my_file meta_file_rebuild = {NULL, SEG_META_FILENAME, "r", SPDK_FS_MODE};
    init_query_engine_with_persistence(&rebuild_engine, &data_file_rebuild, &index_file_rebuild, &meta_file_rebuild);


    rebuild_query_engine_from_file(&rebuild_engine);

    FILE *query_fp = fopen("/home/yangguo/Codes/groundhog/query-workload/nyc_st_005.query", "r");
    // read queries
    int query_num = 6;
    struct spatio_temporal_range_predicate **predicates = allocate_spatio_temporal_predicate_mem(query_num);
    read_spatio_temporal_queries_from_csv_nyc(query_fp, predicates, query_num);

    clock_t start, end;
    start = clock();
    for (int i = 0; i < query_num; i++) {
        printf("time min: %d, time max: %d, lon min: %d, lon max: %d, lat min: %d, lat max: %d\n", predicates[i]->time_min,
               predicates[i]->time_max, predicates[i]->lon_min, predicates[i]->lon_max, predicates[i]->lat_min, predicates[i]->lat_max);
        exp_native_spatio_temporal_host_batch_v1(predicates[i], &rebuild_engine);
        printf("\n");

        exp_native_spatio_temporal_armcpu_full_pushdown_batch_v1(predicates[i], &rebuild_engine);
        printf("\n");

        exp_native_spatio_temporal_adaptive_pushdown_batch_v1(predicates[i], &rebuild_engine);
        printf("\n");

        //exp_native_spatio_temporal_host_device_parallel_batch_v1(predicates[i], &rebuild_engine);

        //printf("selectivity: %f\n", (1.0 * result_count / 30000000.0));
        printf("\n\n\n");
    }
    end = clock();
    printf("total time: %f\n",(double)(end-start));

    free_spatio_temporal_predicate_mem(predicates, query_num);
    free_query_engine(&rebuild_engine);
}

void exp_id_temporal_query_porto() {
    init_and_mk_fs_for_traj(true);
    print_spdk_static_fs_meta_for_traj();

    struct simple_query_engine rebuild_engine;
    struct my_file data_file_rebuild = {NULL, DATA_FILENAME, "r", SPDK_FS_MODE};
    struct my_file index_file_rebuild = {NULL, INDEX_FILENAME, "r", SPDK_FS_MODE};
    struct my_file meta_file_rebuild = {NULL, SEG_META_FILENAME, "r", SPDK_FS_MODE};
    init_query_engine_with_persistence(&rebuild_engine, &data_file_rebuild, &index_file_rebuild, &meta_file_rebuild);


    rebuild_query_engine_from_file(&rebuild_engine);

    FILE *query_fp = fopen("/home/yangguo/Codes/groundhog/query-workload/porto_id.query", "r");
    // read queries
    int query_num = 2;
    struct id_temporal_predicate **predicates = allocate_id_temporal_predicate_mem(query_num);
    read_id_temporal_queries_from_csv(query_fp, predicates, query_num);

    clock_t start, end;
    start = clock();
    for (int i = 0; i < query_num; i++) {
        printf("oid: %d, time min: %d, time max: %d\n", predicates[i]->oid, predicates[i]->time_min,
               predicates[i]->time_max);
        exp_native_id_temporal_host_io_opt_batch_v1(predicates[i], &rebuild_engine);
        printf("\n");

        exp_native_id_temporal_host_batch_v1(predicates[i], &rebuild_engine);
        printf("\n");

        exp_native_id_temporal_armcpu_full_pushdown_batch_naive_v1(predicates[i], &rebuild_engine);
        printf("\n");

        exp_native_id_temporal_armcpu_full_pushdown_batch_v1(predicates[i], &rebuild_engine);
        printf("\n");

        exp_native_id_temporal_adaptive_pushdown_batch_v1(predicates[i], &rebuild_engine);
        printf("\n");

        //exp_native_spatio_temporal_host_device_parallel_batch_v1(predicates[i], &rebuild_engine);

        //printf("selectivity: %f\n", (1.0 * result_count / 30000000.0));
        printf("\n\n\n");
    }
    end = clock();
    printf("total time: %f\n",(double)(end-start));

    free_id_temporal_predicate_mem(predicates, query_num);
    free_query_engine(&rebuild_engine);
}

void exp_id_temporal_query_nyc() {
    init_and_mk_fs_for_traj(true);
    print_spdk_static_fs_meta_for_traj();

    struct simple_query_engine rebuild_engine;
    struct my_file data_file_rebuild = {NULL, DATA_FILENAME, "r", SPDK_FS_MODE};
    struct my_file index_file_rebuild = {NULL, INDEX_FILENAME, "r", SPDK_FS_MODE};
    struct my_file meta_file_rebuild = {NULL, SEG_META_FILENAME, "r", SPDK_FS_MODE};
    init_query_engine_with_persistence(&rebuild_engine, &data_file_rebuild, &index_file_rebuild, &meta_file_rebuild);


    rebuild_query_engine_from_file(&rebuild_engine);

    FILE *query_fp = fopen("/home/yangguo/Codes/groundhog/query-workload/nyc_id.query", "r");
    // read queries
    int query_num = 2;
    struct id_temporal_predicate **predicates = allocate_id_temporal_predicate_mem(query_num);
    read_id_temporal_queries_from_csv_nyc(query_fp, predicates, query_num);

    clock_t start, end;
    start = clock();
    for (int i = 0; i < query_num; i++) {
        printf("oid: %d, time min: %d, time max: %d\n", predicates[i]->oid, predicates[i]->time_min,
               predicates[i]->time_max);
        exp_native_id_temporal_host_batch_v1(predicates[i], &rebuild_engine);
        printf("\n");

        exp_native_id_temporal_armcpu_full_pushdown_batch_v1(predicates[i], &rebuild_engine);
        printf("\n");

        exp_native_id_temporal_adaptive_pushdown_batch_v1(predicates[i], &rebuild_engine);
        printf("\n");

        //exp_native_spatio_temporal_host_device_parallel_batch_v1(predicates[i], &rebuild_engine);

        //printf("selectivity: %f\n", (1.0 * result_count / 30000000.0));
        printf("\n\n\n");
    }
    end = clock();
    printf("total time: %f\n",(double)(end-start));

    free_id_temporal_predicate_mem(predicates, query_num);
    free_query_engine(&rebuild_engine);
}

void exp_spatio_temporal_count_query_porto() {
    init_and_mk_fs_for_traj(true);
    print_spdk_static_fs_meta_for_traj();

    struct simple_query_engine rebuild_engine;
    struct my_file data_file_rebuild = {NULL, DATA_FILENAME, "r", SPDK_FS_MODE};
    struct my_file index_file_rebuild = {NULL, INDEX_FILENAME, "r", SPDK_FS_MODE};
    struct my_file meta_file_rebuild = {NULL, SEG_META_FILENAME, "r", SPDK_FS_MODE};
    init_query_engine_with_persistence(&rebuild_engine, &data_file_rebuild, &index_file_rebuild, &meta_file_rebuild);


    rebuild_query_engine_from_file(&rebuild_engine);
    struct spatio_temporal_range_predicate predicate1 = {normalize_longitude(-8.8), normalize_longitude(-8.5),
                                                         normalize_latitude(41),
                                                         normalize_latitude(41.3), 466, 932};

    struct spatio_temporal_range_predicate predicate2 = {normalize_longitude(-8.8), normalize_longitude(-8.5),
                                                         normalize_latitude(41),
                                                         normalize_latitude(41.3), 466*5, 932*5};

    clock_t start, end;
    start = clock();

    printf("grid width: 0.01\n");
    exp_native_spatio_temporal_count_query_host_batch_v1(&predicate1, &rebuild_engine);
    printf("\n");

    exp_native_spatio_temporal_count_query_armcpu_full_pushdown_batch_naive_v1(&predicate1, &rebuild_engine);
    printf("\n");

    exp_native_spatio_temporal_count_query_armcpu_full_pushdown_batch_v1(&predicate1, &rebuild_engine);
    printf("\n");

    printf("\n\n\n");

    printf("grid width: 0.05\n");
    exp_native_spatio_temporal_count_query_host_batch_v1(&predicate2, &rebuild_engine);
    printf("\n");

    exp_native_spatio_temporal_count_query_armcpu_full_pushdown_batch_naive_v1(&predicate2, &rebuild_engine);
    printf("\n");

    exp_native_spatio_temporal_count_query_armcpu_full_pushdown_batch_v1(&predicate2, &rebuild_engine);
    printf("\n");

    printf("\n\n\n");

    end = clock();
    printf("total time: %f\n",(double)(end-start));


    free_query_engine(&rebuild_engine);
}

void exp_spatio_temporal_count_query_nyc() {
    init_and_mk_fs_for_traj(true);
    print_spdk_static_fs_meta_for_traj();

    struct simple_query_engine rebuild_engine;
    struct my_file data_file_rebuild = {NULL, DATA_FILENAME, "r", SPDK_FS_MODE};
    struct my_file index_file_rebuild = {NULL, INDEX_FILENAME, "r", SPDK_FS_MODE};
    struct my_file meta_file_rebuild = {NULL, SEG_META_FILENAME, "r", SPDK_FS_MODE};
    init_query_engine_with_persistence(&rebuild_engine, &data_file_rebuild, &index_file_rebuild, &meta_file_rebuild);


    rebuild_query_engine_from_file(&rebuild_engine);
    struct spatio_temporal_range_predicate predicate1 = {normalize_longitude(-74.05), normalize_longitude(-73.75),
                                                         normalize_latitude(40.6),
                                                         normalize_latitude(40.9), 466, 932};

    struct spatio_temporal_range_predicate predicate2 = {normalize_longitude(-74.05), normalize_longitude(-73.75),
                                                         normalize_latitude(40.6),
                                                         normalize_latitude(40.9), 466*5, 932*5};

    clock_t start, end;
    start = clock();

    printf("grid width: 0.01\n");
    exp_native_spatio_temporal_count_query_host_batch_v1(&predicate1, &rebuild_engine);
    printf("\n");

    exp_native_spatio_temporal_count_query_armcpu_full_pushdown_batch_naive_v1(&predicate1, &rebuild_engine);
    printf("\n");

    exp_native_spatio_temporal_count_query_armcpu_full_pushdown_batch_v1(&predicate1, &rebuild_engine);
    printf("\n");

    printf("\n\n\n");

    /*printf("grid width: 0.05\n");
    exp_native_spatio_temporal_count_query_host_batch_v1(&predicate2, &rebuild_engine);
    printf("\n");

    exp_native_spatio_temporal_count_query_armcpu_full_pushdown_batch_naive_v1(&predicate2, &rebuild_engine);
    printf("\n");

    exp_native_spatio_temporal_count_query_armcpu_full_pushdown_batch_v1(&predicate2, &rebuild_engine);
    printf("\n");

    printf("\n\n\n");*/

    end = clock();
    printf("total time: %f\n",(double)(end-start));


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

void exp_spatio_temporal_knn_query_porto_scan() {
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
    /*struct traj_point point = {1, 1, normalize_longitude(-8.610291),
                               normalize_latitude(41.140746)};
    struct spatio_temporal_knn_predicate predicate = {point, 10};*/
    struct spatio_temporal_knn_predicate *predicate_ptr = NULL;


    FILE *query_fp = fopen("/home/yangguo/Codes/groundhog/query-workload/porto_knn_k50.query", "r");
    // read queries
    //int query_num = 100;
    int query_num = 20;
    struct spatio_temporal_knn_predicate **predicates = allocate_spatio_temporal_knn_predicate_mem(query_num);
    read_spatio_temporal_knn_queries_from_csv(query_fp, predicates, query_num);

    long host_time[query_num];
    long host_time_pure[query_num];
    long device_time_naive[query_num];
    long device_time_naive_pure[query_num];
    long device_time_mbr_pruning[query_num];
    long device_time_mbr_pruning_pure[query_num];
    long device_time[query_num];
    long device_time_pure[query_num];

    int running_time;
    for (int i = 0; i < query_num; i++) {
        printf("\n\ni: %d\n", i);
        predicate_ptr = predicates[i];

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
        device_time_naive_pure[i] = running_time;*/

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
    }
    end = clock();
    printf("total time: %f\n",(double)(end-start));

    printf("\n\n[host] average time: %f, average pure time: %f\n", average_values(host_time, query_num), average_values(host_time_pure, query_num));
    printf("[device naive] average time: %f, average pure time: %f\n", average_values(device_time_naive, query_num),
           average_values(device_time_naive_pure, query_num));
    printf("[device add mbr pruning] average time: %f, average pure time: %f\n", average_values(device_time_mbr_pruning, query_num),
           average_values(device_time_mbr_pruning_pure, query_num));
    printf("[device] average time: %f, average pure time: %f\n", average_values(device_time, query_num), average_values(device_time_pure, query_num));

    free_spatio_temporal_knn_predicate_mem(predicates, query_num);
    free_query_engine(&rebuild_engine);
}


void exp_spatio_temporal_knn_query_porto_scan_add_host_io_opt() {
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
    /*struct traj_point point = {1, 1, normalize_longitude(-8.610291),
                               normalize_latitude(41.140746)};
    struct spatio_temporal_knn_predicate predicate = {point, 10};*/
    struct spatio_temporal_knn_predicate *predicate_ptr = NULL;


    FILE *query_fp = fopen("/home/yangguo/Codes/groundhog/query-workload/porto_knn_k90.query", "r");
    // read queries
    int query_num = 100;
    //int query_num = 20;
    struct spatio_temporal_knn_predicate **predicates = allocate_spatio_temporal_knn_predicate_mem(query_num);
    read_spatio_temporal_knn_queries_from_csv(query_fp, predicates, query_num);

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
    for (int i = 0; i < query_num; i++) {
        printf("\n\ni: %d\n", i);
        predicate_ptr = predicates[i];

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
        query_start = clock();
        running_time = exp_native_spatio_temporal_knn_armcpu_pushdown_batch_v1(predicate_ptr, &rebuild_engine, 3);
        query_end = clock();
        device_time[i] = query_end - query_start;
        device_time_pure[i] = running_time;
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

    free_spatio_temporal_knn_predicate_mem(predicates, query_num);
    free_query_engine(&rebuild_engine);
}



static int exp_native_spatio_temporal_knn_join_host_batch_v1(struct spatio_temporal_knn_join_predicate *predicate, struct simple_query_engine *rebuild_engine) {

    clock_t start, end;
    start = clock();
    int engine_result = spatio_temporal_knn_join_query_without_pushdown_batch(rebuild_engine, predicate);
    end = clock();
    printf("[host batch] query time (total time, including all): %f\n", (double )(end - start));
    printf("engine result: %d\n", engine_result);

    return engine_result;
}

static int exp_native_spatio_temporal_knn_join_armcpu_pushdown_batch_v1(struct spatio_temporal_knn_join_predicate *predicate, struct simple_query_engine *rebuild_engine, int option) {

    clock_t start, end;
    start = clock();
    int engine_result = spatio_temporal_knn_join_query_with_pushdown_batch(rebuild_engine, predicate, option);
    end = clock();
    printf("[isp cpu] query time (total time, including all): %f\n", (double )(end - start));
    printf("engine result: %d\n", engine_result);

    return engine_result;
}

void exp_spatio_temporal_knn_join_query_porto_scan() {
    init_and_mk_fs_for_traj(true);
    print_spdk_static_fs_meta_for_traj();

    struct simple_query_engine rebuild_engine;
    struct my_file data_file_rebuild = {NULL, DATA_FILENAME, "r", SPDK_FS_MODE};
    struct my_file index_file_rebuild = {NULL, INDEX_FILENAME, "r", SPDK_FS_MODE};
    struct my_file meta_file_rebuild = {NULL, SEG_META_FILENAME, "r", SPDK_FS_MODE};
    init_query_engine_with_persistence(&rebuild_engine, &data_file_rebuild, &index_file_rebuild, &meta_file_rebuild);


    rebuild_query_engine_from_file(&rebuild_engine);



    clock_t start, end, query_start, query_end;

    long host_time;
    long device_time_naive;
    long device_time_mbr_pruning;
    long device_time;

    start = clock();

    struct spatio_temporal_knn_join_predicate predicate = {0,0, 10};

    query_start = clock();
    exp_native_spatio_temporal_knn_join_host_batch_v1(&predicate, &rebuild_engine);
    query_end = clock();
    host_time = query_end - query_start;

    // naive
    query_start = clock();
    exp_native_spatio_temporal_knn_join_armcpu_pushdown_batch_v1(&predicate, &rebuild_engine, 1);
    query_end = clock();
    device_time_naive = query_end - query_start;

    // add mbr pruning
    query_start = clock();
    exp_native_spatio_temporal_knn_join_armcpu_pushdown_batch_v1(&predicate, &rebuild_engine, 2);
    query_end = clock();
    device_time_mbr_pruning = query_end - query_start;

    query_start = clock();
    exp_native_spatio_temporal_knn_join_armcpu_pushdown_batch_v1(&predicate, &rebuild_engine, 3);
    query_end = clock();
    device_time = query_end - query_start;

    end = clock();

    printf("total time: %f\n",(double)(end-start));
    printf("\n\n[host] time: %ld\n", host_time);
    printf("[device naive] time: %ld\n", device_time_naive);
    printf("[device add mbr pruning] time: %ld\n", device_time_mbr_pruning);
    printf("[device] average time: %ld\n", device_time);


    free_query_engine(&rebuild_engine);
}

int main(void) {
    // block pointer: [32767], time min: 7372575, time max: 7372799
    // block pointer: [65535], time min: 14745375, time max: 14745599
    // block pointer: [131071], time min: 29490975, time max: 29491199
    // block pointer: [262143], time min: 61603605, time max: 61603839
    // block pointer: [1048575], time min: 235929375, time max: 235929599
    // 4G: 1048576  2G: 524288  1G: 262144  512MB: 131072   256MB: 65536    128MB:32768
    //ingest_and_flush_synthetic_data(131072);
    //run_on_synthetic_data_multithread();
    //run_on_synthetic_data();
    //run_on_synthetic_data_batch();
    //exp_spatio_temporal_query_synthetic_scan();


    //ingest_and_flush_porto_data(131072);
    //ingest_and_flush_porto_data_zcurve(1048576);
    //run_on_porto_data_multithread_batch();
    //run_on_porto_data_multithread();

    //ingest_and_flush_synthetic_data_large(524288, 6);
    //print_large_file_info();

    //ingest_and_flush_nyc_data_zcurve(1048576 * 2);



    //ingest_and_flush_porto_data_zcurve_full();
    //ingest_and_flush_porto_data_time_oid_full();
    //exp_spatio_temporal_query_porto_scan();
    //exp_spatio_temporal_query_porto_index_scan();
    //exp_id_temporal_query_porto();
    //exp_spatio_temporal_query_porto_index_scan_query_type();

    //ingest_and_flush_nyc_data_zcurve_full();
    //ingest_and_flush_nyc_data_time_oid_full();
    //exp_spatio_temporal_query_nyc_scan();
    //exp_spatio_temporal_count_query_nyc();


    //ingest_and_flush_porto_data_zcurve_10x_full();
    //ingest_and_flush_porto_data_time_oid_10x_full();
    //exp_spatio_temporal_query_porto_index_scan();
    //exp_id_temporal_query_porto();


    //ingest_and_flush_porto_data_zcurve_full();
    //ingest_and_flush_nyc_data_zcurve_full();
    //exp_spatio_temporal_count_query_porto();
    //exp_spatio_temporal_count_query_nyc();


    //ingest_and_flush_porto_data_zcurve_full();
    //ingest_and_flush_porto_data_time_oid_full();
    //exp_spatio_temporal_knn_query_porto_scan();


    //ingest_and_flush_porto_data_zcurve_full();
    //ingest_and_flush_porto_data_time_oid_full();
    //ingest_and_flush_porto_data(1024);
    //exp_spatio_temporal_knn_join_query_porto_scan();

    //ingest_and_flush_nyc_data_zcurve(197949);
    //ingest_and_flush_nyc_data(197949); // time-oid
    //exp_spatio_temporal_query_nyc_scan();


    // segment num
    //ingest_and_flush_porto_data_zcurve_for_segment_test();
    //exp_spatio_temporal_knn_query_porto_scan();
    //exp_spatio_temporal_query_porto_scan_segment_num();
    //exp_spatio_temporal_query_porto_index_scan_segment_num();


    // index scan: two new queries with small selectivity and add new host io opt
    //ingest_and_flush_porto_data_zcurve_full();
    //ingest_and_flush_porto_data_time_oid_full();
    //exp_spatio_temporal_query_porto_index_scan_low_selectivity_add_host_io_opt();
    //exp_id_temporal_query_porto();
    exp_spatio_temporal_knn_query_porto_scan_add_host_io_opt();


    printf("%d\n", calculate_points_num_via_block_size(TRAJ_BLOCK_SIZE, SPLIT_SEGMENT_NUM));



}