//
// Created by yangguo on 11/11/22.
//

#ifndef TRAJ_BLOCK_FORMAT_PORTO_DATASET_READER_H
#define TRAJ_BLOCK_FORMAT_PORTO_DATASET_READER_H

#include <stdio.h>
#include "traj_block_format.h"
#define CSV_ROW_SIZE 70  // the csv file is porto_taxi_data_v2.csv


/**
 * read trajectory points and store them with serialized format
 * @param fp
 * @param buffer
 * @param row_offset start which row to read data
 * @param row_count the total count of rows to read
 * @return
 */
int read_one_buffer_block_from_csv(FILE *fp, void *buffer, int row_offset, int row_count);

/**
 * read trajectory points and store them with struct format
 * @param fp
 * @param points
 * @param row_offset
 * @param row_count
 * @return
 */
int read_points_from_csv(FILE *fp, struct traj_point **points, int row_offset, int row_count);

int generate_synthetic_points(struct traj_point **points, int row_offset, int row_count);

int generate_synthetic_random_points(struct traj_point **points, int row_offset, int row_count, int max_value);

#endif //TRAJ_BLOCK_FORMAT_PORTO_DATASET_READER_H
