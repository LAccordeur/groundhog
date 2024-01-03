//
// Created by yangguo on 11/22/22.
//

#ifndef TRAJ_BLOCK_FORMAT_QUERY_VERIFICATION_UTIL_H
#define TRAJ_BLOCK_FORMAT_QUERY_VERIFICATION_UTIL_H
#include "simple_query_engine.h"

int verify_id_temporal_query(FILE *fp, struct id_temporal_predicate *predicate, int block_num);

int verify_spatio_temporal_query(FILE *fp, struct spatio_temporal_range_predicate *predicate, int block_num);

#endif //TRAJ_BLOCK_FORMAT_QUERY_VERIFICATION_UTIL_H
