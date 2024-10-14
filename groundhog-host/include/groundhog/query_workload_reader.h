//
// Created by yangguo on 11/28/22.
//

#ifndef TRAJ_BLOCK_FORMAT_QUERY_WORKLOAD_READER_H
#define TRAJ_BLOCK_FORMAT_QUERY_WORKLOAD_READER_H
#include <stdlib.h>
#include "simple_query_engine.h"

struct id_temporal_predicate** allocate_id_temporal_predicate_mem(int array_size);

void free_id_temporal_predicate_mem(struct id_temporal_predicate **predicate, int array_size);

int read_id_temporal_queries_from_csv(FILE *fp, struct id_temporal_predicate **predicates, int row_count);

int read_id_temporal_queries_from_csv_nyc(FILE *fp, struct id_temporal_predicate **predicate, int row_count);

struct spatio_temporal_range_predicate** allocate_spatio_temporal_predicate_mem(int array_size);

void free_spatio_temporal_predicate_mem(struct spatio_temporal_range_predicate **predicates, int array_size);

int read_spatio_temporal_queries_from_csv(FILE *fp, struct spatio_temporal_range_predicate **predicate, int row_count);

int read_spatio_temporal_queries_from_csv_nyc(FILE *fp, struct spatio_temporal_range_predicate **predicate, int row_count);

int read_spatio_temporal_queries_from_csv_geolife(FILE *fp, struct spatio_temporal_range_predicate **predicate, int row_count);

struct spatio_temporal_knn_predicate** allocate_spatio_temporal_knn_predicate_mem(int array_size);

void free_spatio_temporal_knn_predicate_mem(struct spatio_temporal_knn_predicate **predicates, int array_size);

int read_spatio_temporal_knn_queries_from_csv(FILE *fp, struct spatio_temporal_knn_predicate **predicate, int row_count);

#endif //TRAJ_BLOCK_FORMAT_QUERY_WORKLOAD_READER_H
