//
// Created by yangguo on 24-4-8.
//

#ifndef TRAJ_BLOCK_FORMAT_OSM_DATASET_READER_H
#define TRAJ_BLOCK_FORMAT_OSM_DATASET_READER_H

#include <stdio.h>
#include "traj_block_format.h"
#define OSM_CSV_ROW_SIZE 32

int read_points_from_csv_osm(FILE *fp, struct traj_point **points, int row_offset, int row_count);

#endif //TRAJ_BLOCK_FORMAT_OSM_DATASET_READER_H
