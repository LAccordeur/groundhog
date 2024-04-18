//
// Created by yangguo on 11/11/22.
//

#include <string.h>
#include <stdlib.h>
#include "groundhog/porto_dataset_reader.h"
#include "groundhog/normalization_util.h"


static void parse_porto_row(char* line, struct traj_point *destination_row);
static void* row_slot_in_buffer_block(void* buffer_block, size_t row_offset);

static void parse_porto_row(char* line, struct traj_point *destination_row) {
    char *token = strtok(line, ",");
    // trip_id
    if (token) {

    }

    // call_type
    token = strtok(NULL, ",");
    if (token) {
        //destination_row->call_type = *token;
    }

    // taxi_id
    token = strtok(NULL, ",");
    if (token) {
        destination_row->oid = atoi(token);
    }

    // timestamp_sec
    token = strtok(NULL, ",");
    if (token) {
        destination_row->timestamp_sec = atoi(token);
    }

    // day_type
    token = strtok(NULL, ",");
    if (token) {

    }

    // missing_data
    token = strtok(NULL, ",");
    if (token) {

    }

    // longitude
    token = strtok(NULL, ",");
    double longitude = atof(token);
    if (token) {
        int normalized_value = normalize_longitude(longitude);
        destination_row->normalized_longitude = normalized_value;
    }

    // latitude
    token = strtok(NULL, "\n");
    double latitude = atof(token);
    if (token) {
        int normalized_value = normalize_latitude(latitude);
        destination_row->normalized_latitude = normalized_value;
    }

}

static void* row_slot_in_buffer_block(void* buffer_block, size_t row_offset) {
    char* b = buffer_block;
    return b + row_offset * get_traj_point_size();
}


int read_one_buffer_block_from_csv(FILE *fp, void *buffer, int row_offset, int row_count) {
    int file_offset = row_offset * CSV_ROW_SIZE;
    fseek(fp, file_offset, SEEK_SET);

    char line[128];
    int line_count = 0;
    while (fgets(line, 128, fp)) {

        struct traj_point row;
        parse_porto_row(line, &row);
        serialize_traj_point(&row, row_slot_in_buffer_block(buffer, line_count));
        line_count++;
        //printf("line count: %d\n", line_count);
        if (line_count >= row_count) {
            break;
        }
    }
    return line_count;
}

int read_points_from_csv(FILE *fp, struct traj_point **points, int row_offset, int row_count) {
    int file_offset = row_offset * CSV_ROW_SIZE;
    fseek(fp, file_offset, SEEK_SET);

    char line[128];
    int line_count = 0;
    while (fgets(line, 128, fp)) {

        struct traj_point *row = points[line_count];
        parse_porto_row(line, row);
        line_count++;
        //printf("line count: %d\n", line_count);
        if (line_count >= row_count) {
            break;
        }
    }
    return line_count;
}

/*
void convert_serialized_to_struct_traj(void *serialized_buffer, struct traj_point **points, int total_size) {

    for (int i = 0; i < total_size; i++) {
        void *buffer_ptr = row_slot_in_buffer_block(serialized_buffer, i);
        deserialize_traj_point(buffer_ptr, points[i]);
    }

}

void convert_struct_to_serialized_traj(struct traj_point **points, void *serialized_buffer, int total_size) {

    for (int i = 0; i < total_size; i++) {
        struct traj_point *point = points[i];
        void *buffer_ptr = row_slot_in_buffer_block(serialized_buffer, i);
        serialize_traj_point(point, buffer_ptr);
    }
}*/
