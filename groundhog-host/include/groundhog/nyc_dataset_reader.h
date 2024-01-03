//
// Created by yangguo on 23-6-29.
//

#ifndef TRAJ_BLOCK_FORMAT_NYC_DATASET_READER_H
#define TRAJ_BLOCK_FORMAT_NYC_DATASET_READER_H

#include <stdio.h>
#include "traj_block_format.h"

#define NYC_CSV_ROW_SIZE 52

int read_points_from_csv_nyc(FILE *fp, struct traj_point **points, int row_offset, int row_count);

#endif //TRAJ_BLOCK_FORMAT_NYC_DATASET_READER_H
