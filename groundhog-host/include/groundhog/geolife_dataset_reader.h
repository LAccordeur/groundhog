//
// Created by yangguo on 24-6-6.
//

#ifndef TRAJ_BLOCK_FORMAT_GEOLIFE_DATASET_READER_H
#define TRAJ_BLOCK_FORMAT_GEOLIFE_DATASET_READER_H

#include <stdio.h>
#include "traj_block_format.h"

#define GEOLIFE_CSV_ROW_SIZE 32

int read_points_from_csv_geolife(FILE *fp, struct traj_point **points, int row_offset, int row_count);

#endif //TRAJ_BLOCK_FORMAT_GEOLIFE_DATASET_READER_H
