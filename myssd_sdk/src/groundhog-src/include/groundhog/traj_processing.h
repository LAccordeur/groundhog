//
// Created by yangguo on 11/11/22.
//

#ifndef TRAJ_BLOCK_FORMAT_TRAJ_PROCESSING_H
#define TRAJ_BLOCK_FORMAT_TRAJ_PROCESSING_H
#include "traj_block_format.h"

void sort_traj_points(struct traj_point **points, int array_size);

/**
 * deprecated
 *
 * @param points
 * @param points_num
 * @param segment_size
 * @param pair_array
 * @param array_size
 * @return
 */
int split_traj_points_via_point_num(struct traj_point **points, int points_num, int segment_size, struct seg_meta_pair_itr *pair_array, int array_size);

int split_traj_points_via_point_num_and_seg_num(struct traj_point **points, int points_num, struct seg_meta_pair_itr *pair_array, int segment_num);

#endif //TRAJ_BLOCK_FORMAT_TRAJ_PROCESSING_H
