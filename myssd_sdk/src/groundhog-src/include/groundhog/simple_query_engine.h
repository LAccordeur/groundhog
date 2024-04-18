//
// Created by yangguo on 11/16/22.
//

#ifndef TRAJ_BLOCK_FORMAT_SIMPLE_QUERY_ENGINE_H
#define TRAJ_BLOCK_FORMAT_SIMPLE_QUERY_ENGINE_H

#include <stdio.h>

#include "simple_index.h"
#include "simple_storage.h"
#include "seg_meta_store.h"

struct simple_query_engine {
    struct traj_storage data_storage;
    struct index_entry_storage index_storage;
    struct seg_meta_section_entry_storage seg_meta_storage;
};

struct id_temporal_predicate {
    int oid;
    int time_min;
    int time_max;
};

struct spatio_temporal_range_predicate {
    int lon_min;    // normalized value, same below
    int lon_max;
    int lat_min;
    int lat_max;
    int time_min;
    int time_max;
};

void init_query_engine(struct simple_query_engine *engine);

void init_query_engine_with_persistence(struct simple_query_engine *engine, struct my_file *data_file, struct my_file *index_file, struct my_file *meta_file);

void free_query_engine(struct simple_query_engine *engine);

void ingest_data_via_time_partition(struct simple_query_engine *engine, FILE *fp, int block_num);

void ingest_and_flush_data_via_time_partition(struct simple_query_engine *engine, FILE *fp, int block_num);

void rebuild_query_engine_from_file(struct simple_query_engine *engine);

int id_temporal_query(struct simple_query_engine *engine, struct id_temporal_predicate *predicate);

int spatio_temporal_query(struct simple_query_engine *engine, struct spatio_temporal_range_predicate *predicate);

int estimate_id_temporal_result_size(struct seg_meta_section_entry_storage *storage, struct id_temporal_predicate *predicate);

int estimate_spatio_temporal_result_size(struct seg_meta_section_entry_storage *storage, struct spatio_temporal_range_predicate *predicate);


#endif //TRAJ_BLOCK_FORMAT_SIMPLE_QUERY_ENGINE_H
